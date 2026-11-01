#!/usr/bin/env python3
"""SLAM 小车 Web 控制台

单文件、零第三方依赖（只用标准库 + ROS 2 Python），运行在 Jetson 上：
    python3 ~/webui.py            # 监听 0.0.0.0:8080

功能：
  * 实时预览：栅格地图 + 当前激光扫描 + 车位姿（Canvas，可缩放/拖动）
  * 行驶控制：键盘 WASD，松手 400 ms 自动停车（服务端也有一层死手开关）
  * 建图控制：开始建图 / 建图完成（Cartographer finish_trajectory + write_state + 降噪入库）/ 停止节点
 * 地图库：下拉选择、使用（定位模式加载 pbstream）、现场校验、删除
  * IMU 校准：仅软件侧（把当前姿态设为零位，显示层相对化；不动 MCU 零偏）

浏览器打开 http://<jetson-ip>:8080
"""
import json
import math
import os
import re
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState

import map_tools
from scan_geometry import project_scan
from std_msgs.msg import String

from nav_manager import NavManager
from slam_manager import finish_map, slam_start, slam_stop
import localization_manager
from localization_manager import (start_localization, stop_localization,
                                  wait_ready)
import map_localize
from webui_state import S, rle_encode

PORT = 8080
MAP_SAVE_NAME = '/home/nvidia/maps/car_map'
DEADMAN_MS = 400
V_MAX = 0.35
W_MAX = 1.2

NAV = None

# 建图/定位/重定位互斥：多次点击或后台监控同时进来时，只允许一个真正执行
_map_op = threading.Lock()

# 地图列表缓存：SSE 每 100 ms 重建一次状态，不能每次都去读所有 meta.json
_maps_cache = {'t': 0.0, 'list': []}

# 单主机 SSE 上限：浏览器同主机只给 6 条连接，事件流占满后连页面都刷不出来
MAX_STREAMS = 3
_streams = 0
_streams_lock = threading.Lock()


def _maps_cached(ttl=2.0):
    now = time.time()
    if now - _maps_cache['t'] >= ttl:
        _maps_cache['list'] = map_tools.list_maps()[:20]
        _maps_cache['t'] = now
    return _maps_cache['list']


# --------------------------------------------------------------------------
# ROS 节点
# --------------------------------------------------------------------------
class WebNode(Node):
    def __init__(self):
        super().__init__('webui')
        q_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        q_sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(OccupancyGrid, '/map', self.on_map, q_map)
        self.create_subscription(LaserScan, '/scan', self.on_scan, q_sensor)
        self.create_subscription(Odometry, '/odom', self.on_odom, q_sensor)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, q_sensor)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(String, '/safety_guard/state', self.on_safety, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.pub_status = self.create_publisher(String, '/webui/status', 10)
        self.create_timer(0.5, self.tick_status)
        self.cli_finish = self.create_client(FinishTrajectory, '/finish_trajectory')
        self.cli_write_state = self.create_client(WriteState, '/write_state')

        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)

        self._scan_t = []
        self._scan_n = 0
        self._scan_last = 0.0
        self.create_timer(0.05, self.tick_cmd)      # 20 Hz 下发/死手
        self.get_logger().info('webui node up')

    # ---- 订阅回调 ----
    def on_map(self, m):
        info = m.info
        data = list(m.data)
        stamp = (int(m.header.stamp.sec), int(m.header.stamp.nanosec))
        with S.lock:
            previous = S.map_stamp
            stamp_rewound = previous is not None and stamp < previous['stamp']
            if stamp_rewound and not S.localizing:
                S.map_session = uuid.uuid4().hex
            S.map_stamp = {
                'width': int(info.width), 'height': int(info.height),
                'resolution': float(info.resolution),
                'origin_x': float(info.origin.position.x),
                'origin_y': float(info.origin.position.y),
                'stamp': stamp,
            }
            payload = {
                'w': info.width, 'h': info.height, 'res': info.resolution,
                'ox': info.origin.position.x, 'oy': info.origin.position.y,
                'rle': rle_encode(data),
            }
            S.map_meta = payload
            S.map_last_msg = m          # 入库时要拿原始栅格导出 PGM
            S.map_seen_t = time.monotonic()
            S.map_ver += 1

    def last_map_msg(self):
        with S.lock:
            return S.map_last_msg

    def last_scan_msg(self):
        with S.lock:
            return S.last_scan_msg

    def on_scan(self, s):
        with S.lock:
            S.last_scan_msg = s
            S.last_scan_t = time.monotonic()
        stamp = Time.from_msg(s.header.stamp)
        laser_frame = s.header.frame_id or 'laser_frame'
        laser_tf = self._lookup_tf('map', laser_frame, stamp)
        base_tf = self._lookup_tf('map', 'base_link', stamp)
        if laser_tf is None or base_tf is None:
            # AMCL 静止时不一定按扫描时刻重发 map→odom，退回最新 TF，
            # 否则页面上的点云/位姿会在停车时冻住
            now_t = Time()
            if laser_tf is None:
                laser_tf = self._lookup_tf('map', laser_frame, now_t)
            if base_tf is None:
                base_tf = self._lookup_tf('map', 'base_link', now_t)
        if laser_tf is None or base_tf is None:
            return

        pts = project_scan(
            ranges=s.ranges,
            angle_min=s.angle_min,
            angle_increment=s.angle_increment,
            range_min=s.range_min,
            range_max=s.range_max,
            map_laser=laser_tf,
        )
        now = time.time()
        with S.lock:
            S.map_pose = [base_tf[0], base_tf[1], base_tf[2]]
            S.scan_pts = pts
            S.scan_ver += 1
            S.scan_count += 1
            ts = getattr(self, '_ts', [])
            ts.append(now)
            ts = [t for t in ts if now - t < 2.0]
            self._ts = ts
            if len(ts) >= 3:
                S.scan_hz = (len(ts) - 1) / (ts[-1] - ts[0])

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with S.lock:
            S.pose = (p.x, p.y, th, m.twist.twist.linear.x, m.twist.twist.angular.z)

    def on_imu(self, m):
        q = m.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        with S.lock:
            S.imu = (roll, pitch, yaw)

    def on_plan(self, m):
        pts = []
        for i, p in enumerate(m.poses):
            if i % 2:
                continue
            pts.append(round(p.pose.position.x, 3))
            pts.append(round(p.pose.position.y, 3))
            if len(pts) >= 600:
                break
        with S.lock:
            S.path = pts
            S.path_ver += 1

    def on_safety(self, m):
        try:
            data = json.loads(m.data)
        except Exception:
            return
        with S.lock:
            S.safety = data

    # ---- 死手开关 ----
    def _lookup_tf(self, target_frame, source_frame, stamp):
        """查指定时间的 TF，并返回 (x, y, yaw)。"""
        try:
            tr = self.tf_buf.lookup_transform(
                target_frame, source_frame, stamp,
                timeout=Duration(seconds=0.08))
        except Exception:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return float(t.x), float(t.y), float(yaw)

    def tick_status(self):
        """给 data_logger 等外部记录用的紧凑状态（2 Hz）。"""
        with S.lock:
            q = S.loc_quality or {}
            nav = S.nav or {}
            safety = S.safety or {}
            data = {
                'mode': S.mode,
                'localizing': S.localizing,
                'map_static': S.map_static,
                'map_id': S.map_id,
                'loc_inlier': q.get('inlier'),
                'loc_ok': q.get('ok'),
                'nav_status': nav.get('status'),
                'nav_active': nav.get('active'),
                'nav_source': nav.get('source'),
                'nav_distance': nav.get('distance'),
                'home_valid': bool((S.home or {}).get('valid')),
                'home_dist': S.home_dist,
                'safety_blocked': list((safety or {}).get('blocked') or []),
                'safety_stale': (safety or {}).get('stale'),
                'scan_hz': S.scan_hz,
            }
        self.pub_status.publish(
            String(data=json.dumps(data, separators=(',', ':'))))

    def tick_cmd(self):
        now = time.time()
        with S.lock:
            mode = S.mode
            fresh = (now - S.cmd_t) * 1000.0 < DEADMAN_MS
            v, w = S.cmd if (fresh and mode == 'manual') else (0.0, 0.0)
            if mode == 'manual' and not fresh and (S.cmd[0] or S.cmd[1]):
                S.cmd = [0.0, 0.0]
        if mode != 'manual':
            return          # 自动模式由 Nav2 直接发 /cmd_vel
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub_cmd.publish(t)

    # ---- 服务调用（同步等待，最多 5 s）----
    def _call(self, client, req, timeout=5.0):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=2.0):
                return None, '服务不可用（slam_toolbox 没在跑？）'
        fut = client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        if not fut.done():
            return None, '服务超时'
        try:
            return fut.result(), None
        except Exception as e:
            return None, str(e)

    def _finish_map(self):
        with S.lock:
            if S.localizing:
                return '定位模式下不能结束建图；请先「开始建图」'
        if not _map_op.acquire(blocking=False):
            return '地图操作进行中，请稍候再试'
        try:
            _, msg = finish_map(NODE)
        except Exception as e:
            return '入库失败: %s: %s' % (type(e).__name__, e)
        finally:
            _map_op.release()
        return msg

    def _trusted(self, best):
        return (best is not None
                and best['inlier'] >= map_localize.MIN_INLIER
                and best['mean_dist'] <= map_localize.MAX_MEAN)

    def use_map(self, mid):
        """AMCL 真定位：停 Cartographer/Nav2，起 map_server + AMCL，地图静态。

        同一张图已在定位中时不重启 AMCL（重启会丢粒子、打断持续跟踪）；
        多次点击只让第一个请求真正执行。
        """
        if not re.fullmatch(r'\d{8}_\d{6}', mid or ''):
            return '地图 id 非法'
        d = map_tools.map_dir(mid)
        map_yaml = os.path.join(d, 'map.yaml')
        if not os.path.isfile(map_yaml):
            return '地图不完整（缺少 map.yaml）: %s' % mid
        if not _map_op.acquire(blocking=False):
            return '地图操作进行中，请稍候再点'
        try:
            with S.lock:
                same = S.localizing and S.map_id == mid
                have_map = S.map_meta is not None
            if same and localization_manager.is_active(NODE):
                if not have_map and not wait_ready(NODE, 10.0):
                    return 'AMCL/map_server 未就绪，请查看 /tmp/localization_web.log'
                msg = NAV.start_nav()
                loc = self._relocalize_locked()
                S.note('已在定位中，未重启 AMCL/map_server：%s' % mid)
                return '该地图已在定位中（AMCL 持续跟踪）；%s；%s' % (loc, msg)

            NAV.stop_nav()
            stop_localization()
            NAV.switch_map(mid)
            proc, err = start_localization(map_yaml)
            if err:
                NAV.exit_localization()
                return err
            if not wait_ready(NODE, 15.0):
                NAV.stop_nav()
                stop_localization()
                NAV.exit_localization()
                S.note('AMCL/map_server 未就绪，已清理')
                return 'AMCL/map_server 未就绪，已清理；请查看 /tmp/localization_web.log'
            best = map_localize.global_localize(NODE.last_map_msg(),
                                                NODE.last_scan_msg())
            if self._trusted(best):
                pose = [best['x'], best['y'], best['yaw']]
                loc = '全局定位 inlier=%.2f mean=%.2f m' % (
                    best['inlier'], best['mean_dist'])
            else:
                pose = [0.0, 0.0, 0.0]
                loc = '全局定位低置信度，初始为 map 原点；请用「设初始位姿」或「重新全局定位」'
            NAV.set_initial_pose(float(pose[0]), float(pose[1]),
                                 float(pose[2]), trusted=self._trusted(best))
            NAV.start_nav()
            msg = '已切到 AMCL 定位：%s（静态地图）；%s' % (mid, loc)
            S.note(msg)
            return msg
        finally:
            _map_op.release()

    def relocalize(self):
        """用当前扫描对静态地图做全局拟合，并重置 AMCL 位姿。"""
        if not _map_op.acquire(blocking=False):
            return '地图操作进行中，请稍候再试'
        try:
            return self._relocalize_locked()
        finally:
            _map_op.release()

    def _relocalize_locked(self):
        """调用方必须已持有 _map_op。"""
        with S.lock:
            localizing = S.localizing
            map_msg = S.map_last_msg
            v, w = S.pose[3], S.pose[4]
        if not localizing:
            return '当前不是定位模式，无法重定位'
        if abs(v) > 0.03 or abs(w) > 0.08:
            return '车辆行驶中，请先停车再重定位'
        if map_msg is None:
            return '地图尚未加载'
        if not localization_manager.is_active(NODE):
            return 'AMCL/map_server 未在运行，请重新点「使用」'
        scan = NODE.last_scan_msg()
        if scan is None:
            return '还没有激光数据'
        best = map_localize.global_localize(map_msg, scan)
        if best is None:
            return '全局定位失败（有效扫描点太少）'
        if not self._trusted(best):
            return ('重定位失败：置信度不足（inlier %.2f，均距 %.3f m），'
                    '请用「设初始位姿」' % (best['inlier'],
                                           best['mean_dist']))
        NAV.set_initial_pose(float(best['x']), float(best['y']),
                             float(best['yaw']), trusted=True)
        with S.lock:
            S.loc_quality = {'inlier': round(best['inlier'], 3),
                             'mean': round(best['mean_dist'], 3),
                             'ok': True, 't': time.time()}
        msg = ('重定位完成：x=%.2f y=%.2f yaw=%.1f°，inlier %.2f / %.3f m' %
               (best['x'], best['y'], math.degrees(best['yaw']),
                best['inlier'], best['mean_dist']))
        S.note(msg)
        return msg

    def start_mapping(self):
        if not _map_op.acquire(blocking=False):
            return '地图操作进行中，请稍候再试'
        try:
            NAV.stop_nav()
            stop_localization()
            NAV.exit_localization()
            return slam_start()
        finally:
            _map_op.release()

    def stop_mapping(self):
        if not _map_op.acquire(blocking=False):
            return '地图操作进行中，请稍候再试'
        try:
            NAV.stop_nav()
            stop_localization()
            NAV.exit_localization()
            return slam_stop()
        finally:
            _map_op.release()

    def _fit_check(self, map_id, which='map'):
        if not map_id:
            return '缺少 map_id'
        if which not in ('map', 'raw'):
            which = 'map'
        d = map_tools.map_dir(map_id)
        pgm = os.path.join(d, which + '.pgm')
        if not os.path.isfile(pgm):
            return '地图不存在: %s/%s.pgm' % (map_id, which)

        import json
        with open(os.path.join(d, 'meta.json')) as f:
            meta = json.load(f)

        with S.lock:
            pts = list(S.scan_pts)     # 已经是 map 系世界坐标
            if not pts:
                return '还没有扫描数据'
            xy = np.asarray(pts, dtype=np.float64).reshape(-1, 2)

        grid = map_tools.load_pgm(pgm)
        r = map_tools.fit_check(grid, meta['resolution'], meta['origin'][:2], xy)
        if not r.get('ok'):
            return '校验失败: %s' % r.get('reason')
        meta['fit_check_' + which] = r
        if which == 'map':
            meta['fit_check'] = r       # 地图列表仍读这个键
        map_tools.write_meta(map_id, meta)
        S.note('地图 %s 校验: inlier %.2f, 均距 %.3f m → %s' %
               (map_id, r['inlier_ratio'], r['mean_dist_m'],
                '通过' if r['passed'] else '不通过'))
        return '%s 校验%s：inlier %.2f，平均距离 %.3f m（%d 点）' % (
            '原始图' if which == 'raw' else '降噪图',
            '通过' if r['passed'] else '**不通过**',
            r['inlier_ratio'], r['mean_dist_m'], r['points'])





class LocMonitor(threading.Thread):
    """持续体检 AMCL：扫描对地图匹配差且车静止时，自动做一次全局重定位。

    AMCL 的运动模型依赖 odom，车被手动搬动（odom 不变）时它不会自己发现；
    这里用当前 AMCL 位姿把扫描投影到地图上量吻合度，连续多帧失配就重定位。
    """

    BAD_INLIER = 0.55
    BAD_MEAN = 0.20
    BAD_FRAMES = 3
    MIN_GAP = 6.0

    def __init__(self):
        super().__init__(daemon=True)
        self._ver = -1
        self._field = None
        self._bad = 0
        self._fails = 0
        self._last_try = 0.0

    def _field_of(self, msg, ver):
        if ver != self._ver or self._field is None:
            grid, dist, res, ox, oy, h, w = map_localize._grid(msg)
            self._field = (dist, res, ox, oy, h, w)
            self._ver = ver
        return self._field

    def _quality(self, scan, pose, msg, ver):
        base = map_localize._scan_base(scan)
        if base is None:
            return None
        dist, res, ox, oy, h, w = self._field_of(msg, ver)
        pts = map_localize._rot(base, pose[2]) + np.array([pose[0], pose[1]])
        _s, inl, mean = map_localize._scores(pts[None, :, :], dist, res,
                                             ox, oy, h, w)
        return float(inl[0]), float(mean[0])

    def run(self):
        while True:
            time.sleep(1.0)
            now = time.monotonic()
            with S.lock:
                localizing = S.localizing
                busy = S.reloc_busy
                scan = S.last_scan_msg
                scan_t = S.last_scan_t
                pose = S.map_pose
                pose_t = S.map_pose_t
                v, w = S.pose[3], S.pose[4]
                nav_active = bool((S.nav or {}).get('active'))
                msg = S.map_last_msg
                ver = S.map_ver
                have_map = S.map_meta is not None
            if (not localizing or busy or _map_op.locked() or not have_map
                    or msg is None or scan is None or now - scan_t > 1.5
                    or pose is None or now - pose_t > 1.5):
                self._bad = 0
                continue
            try:
                q = self._quality(scan, pose, msg, ver)
            except Exception as e:
                S.note('定位体检异常: %s: %s' % (type(e).__name__, e))
                time.sleep(1.0)
                continue
            if q is None:
                continue
            inl, mean = q
            ok = inl >= self.BAD_INLIER and mean <= self.BAD_MEAN
            with S.lock:
                S.loc_quality = {'inlier': round(inl, 3),
                                 'mean': round(mean, 3),
                                 'ok': ok, 't': time.time()}
            if ok:
                self._bad = 0
                self._fails = 0
                continue
            if nav_active or abs(v) > 0.03 or abs(w) > 0.08:
                self._bad = 0          # 行驶中交给 AMCL，不抢位姿
                continue
            self._bad += 1
            gap = self.MIN_GAP * (2 ** min(self._fails, 4))
            if self._bad < self.BAD_FRAMES or now - self._last_try < gap:
                continue
            self._bad = 0
            self._last_try = now
            with S.lock:
                S.reloc_busy = True
            try:
                res = NODE.relocalize()
                self._fails = 0 if res.startswith('重定位完成') else self._fails + 1
                S.note('定位失配（inlier %.2f / %.3f m），自动重定位：%s'
                       % (inl, mean, res))
            except Exception as e:
                self._fails += 1
                S.note('自动重定位异常: %s: %s' % (type(e).__name__, e))
            finally:
                with S.lock:
                    S.reloc_busy = False


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = open(os.path.join(HERE, 'webui.html'), 'rb').read()
JS = open(os.path.join(HERE, 'webui.js'), 'rb').read()


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
        elif self.path == '/webui.js':
            self.send_response(200)
            self.send_header('Content-Type', 'application/javascript; charset=utf-8')
            self.send_header('Content-Length', str(len(JS)))
            self.end_headers()
            self.wfile.write(JS)
        elif self.path.startswith('/stream'):
            self.stream()
        else:
            self.send_error(404)

    def do_POST(self):
        try:
            n = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(n) or b'{}')
        except Exception:
            body = {}
        out = {'ok': True, 'msg': ''}
        try:
            out['msg'] = self.dispatch(body)
        except Exception as e:
            out = {'ok': False, 'msg': '%s: %s' % (type(e).__name__, e)}
        raw = json.dumps(out).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    # ---- 指令分发 ----
    def dispatch(self, b):
        a = b.get('cmd')
        if a == 'vel':
            with S.lock:
                if S.mode != 'manual':
                    return '自动模式下手动速度已忽略'
            v = max(-V_MAX, min(V_MAX, float(b.get('v', 0))))
            w = max(-W_MAX, min(W_MAX, float(b.get('w', 0))))
            with S.lock:
                S.cmd = [v, w]
                S.cmd_t = time.time()
            return ''
        if a == 'stop':
            with S.lock:
                mode = S.mode
                S.cmd = [0.0, 0.0]
            if mode == 'auto':
                return NAV.estop()
            return '已停车'
        if a == 'estop':
            return NAV.estop()
        if a == 'mode':
            return NAV.set_mode(str(b.get('mode', '')))
        if a == 'nav_goal':
            return NAV.send_goal(float(b.get('x', 0.0)), float(b.get('y', 0.0)),
                                 float(b.get('yaw', 0.0)))
        if a == 'nav_home':
            return NAV.return_home()
        if a == 'set_home':
            return NAV.set_home()
        if a == 'nav_cancel':
            return NAV.cancel()
        if a == 'nav_start':
            return NAV.start_nav()
        if a == 'nav_stop':
            return NAV.stop_nav()
        if a == 'pause':
            return 'Cartographer 没有暂停服务，请用「结束本图」入库'
        if a == 'save':
            return NODE._finish_map()
        if a == 'map_finish':
            return NODE._finish_map()
        if a == 'map_use':
            mid = (b.get('map_id') or '').strip()
            if not mid:
                return '缺少 map_id'
            return NODE.use_map(mid)
        if a == 'map_relocalize':
            return NODE.relocalize()
        if a == 'initial_pose':
            return NAV.set_initial_pose(float(b.get('x', 0.0)),
                                        float(b.get('y', 0.0)),
                                        float(b.get('yaw', 0.0)))
        if a == 'map_delete':
            ok, msg = map_tools.delete_map((b.get('map_id') or '').strip())
            S.note(msg)
            return msg
        if a == 'map_check':
            return NODE._fit_check((b.get('map_id') or '').strip(),
                                   (b.get('file') or 'map').strip())
        if a == 'slam_start':
            return NODE.start_mapping()
        if a == 'slam_stop':
            return NODE.stop_mapping()
        if a == 'imu_zero':
            with S.lock:
                S.imu_zero = list(S.imu)
            return 'IMU 已设为零位（仅显示层，MCU 零偏未动）'
        if a == 'imu_clear':
            with S.lock:
                S.imu_zero = [0.0, 0.0, 0.0]
            return 'IMU 零位已清除'
        return '未知指令'

    # ---- SSE ----
    def stream(self):
        global _streams
        with _streams_lock:
            if _streams >= MAX_STREAMS:
                self.send_error(503, 'too many event streams')
                return
            _streams += 1
        try:
            self._stream_loop()
        finally:
            with _streams_lock:
                _streams -= 1

    def _stream_loop(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        last_map = -1
        last_scan = -1
        last_path = -1
        last_state_t = 0.0
        last_scan_t = 0.0
        try:
            while True:
                now = time.monotonic()
                st = None
                with S.lock:
                    mv, sv, pv = S.map_ver, S.scan_ver, S.path_ver
                    mp = S.map_meta
                    scan = S.scan_pts
                    path = list(S.path)
                    if now - last_state_t >= 0.2:
                        running = ((S.proc is not None and S.proc.poll() is None)
                                   or NODE.cli_finish.service_is_ready())
                        st = {
                            'pose': S.pose, 'imu': S.imu,
                            'imu_zero': S.imu_zero,
                            'map_pose': S.map_pose,
                            'cmd': list(S.cmd), 'scan_hz': S.scan_hz,
                            'paused': S.paused,
                            'slam_running': running,
                            'log': S.log[-6:],
                            'have_map': mp is not None,
                            'mode': S.mode,
                            'localizing': S.localizing,
                            'map_static': S.map_static,
                            'map_id': S.map_id,
                            'loc_quality': S.loc_quality,
                            'reloc_busy': S.reloc_busy,
                            'nav': dict(S.nav),
                            'safety': dict(S.safety),
                            'home': dict(S.home),
                            'home_dist': S.home_dist,
                            'maps': _maps_cached(),
                        }
                if mv != last_map and mp is not None:
                    self._ev('map', mp)
                    last_map = mv
                if sv != last_scan and scan and now - last_scan_t >= 0.2:
                    self._ev('scan', {'pts': scan})
                    last_scan = sv
                    last_scan_t = now
                if pv != last_path:
                    self._ev('path', {'pts': path})
                    last_path = pv
                if st is not None:
                    self._ev('state', st)
                    last_state_t = now
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _ev(self, name, obj):
        data = 'event: %s\ndata: %s\n\n' % (name, json.dumps(obj, separators=(',', ':')))
        self.wfile.write(data.encode())
        self.wfile.flush()


# --------------------------------------------------------------------------
def main():
    global NODE, NAV
    rclpy.init()
    NODE = WebNode()
    NAV = NavManager(NODE, S, S.lock, NODE.pub_cmd)
    ex = MultiThreadedExecutor()
    ex.add_node(NODE)
    threading.Thread(target=ex.spin, daemon=True).start()
    LocMonitor().start()

    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    srv.daemon_threads = True
    print('webui: http://0.0.0.0:%d  (Ctrl+C 退出)' % PORT)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        NODE.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
