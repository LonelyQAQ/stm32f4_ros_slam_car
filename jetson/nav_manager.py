#!/usr/bin/env python3
"""Nav2 目标 / 返航原点管理（由 webui.py 导入，共用 webui 节点）

职责：
  * 发 NavigateToPose 目标、取消目标、跟踪反馈；
  * 手动/自动模式切换，1 Hz 心跳发布 /nav_mode；
  * “设为原点”用 map→base_link 记录当前位姿，持久化到 ~/maps/home_pose.json；
  * 启动/停止 Nav2 launch 进程。
"""
import json
import math
import os
import signal
import statistics
import subprocess
import threading
import time
import uuid

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

STATUS_TEXT = {
    0: 'unknown', 1: 'accepted', 2: 'executing', 3: 'canceling',
    4: 'succeeded', 5: 'canceled', 6: 'aborted',
}

# 原点取多帧去抖；定位质量不足时拒绝设原点（单帧 TF 可能偏 5~10 cm）
HOME_SAMPLES = 10
HOME_SAMPLE_SPAN = 1.0
HOME_MIN_INLIER = 0.80


class NavManager:
    def __init__(self, node, state, lock, cmd_pub):
        self.node = node
        self.state = state
        self.lock = lock
        self.cmd_pub = cmd_pub

        self.home_file = node.declare_parameter(
            'home_file', os.path.expanduser('~/maps/home_pose.json')).value
        self.nav_params = node.declare_parameter(
            'nav_params', '/home/nvidia/nav2_tmini.yaml').value
        # 到点后自动微调阈值（m）；0 = 关闭。
        # 默认关闭：13 cm 级的近距离目标 DWB 只生成旋转轨迹，会原地打转。
        self.home_refine_m = float(node.declare_parameter(
            'home_refine_m', 0.0).value)

        self.mode = 'manual'
        self.nav_active = False
        self.goal_handle = None
        self._goal_source = None
        self._home_refine_used = False
        self.nav_proc = None
        self.home = self._load_home()
        self._slam_present = None
        self._slam_check_t = 0.0

        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.mode_pub = node.create_publisher(String, '/nav_mode', 10)
        self.pub_brake = node.create_publisher(Bool, '/cmd_brake', 10)
        self.pub_initialpose = node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        node.create_timer(1.0, self._publish_mode)
        node.create_timer(0.05, self._tick_tf)
        self._publish_mode()
        self._update_state()
        node.get_logger().info(f'nav_manager: home={self.home} params={self.nav_params}')

    # ---------------- 状态 ----------------
    def _tick_tf(self):
        now = time.monotonic()
        with self.lock:
            localizing = getattr(self.state, 'localizing', False)
        if not localizing:
            if now - self._slam_check_t >= 1.0:
                self._slam_check_t = now
                slam_present = subprocess.run(
                    ['pgrep', '-f', 'cartographer_node'],
                    capture_output=True).returncode == 0
            else:
                slam_present = self._slam_present
            if self._slam_present is None:
                self._slam_present = slam_present
            elif slam_present != self._slam_present:
                with self.lock:
                    self.state.map_session = uuid.uuid4().hex
                    self.state.map_stamp = None
                    self.state.map_meta = None
                    self.state.map_seen_t = 0.0
                    self.state.map_ver += 1
                self._slam_present = slam_present
                if not slam_present:
                    self._invalidate_home('Cartographer 已停止')
                else:
                    self.node.get_logger().warn(
                        'Cartographer 已重启，当前地图会话已切换')

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.state.map_pose = (t.x, t.y, yaw)
            self.state.map_pose_t = time.monotonic()
            if self.home.get('valid'):
                self.state.home_dist = round(math.hypot(
                    t.x - self.home['x'], t.y - self.home['y']), 3)
            else:
                self.state.home_dist = None
            map_session = self.state.map_session
            localizing = getattr(self.state, 'localizing', False)
            if localizing:
                map_fresh = self.state.map_meta is not None
            else:
                map_fresh = (self.state.map_stamp is not None and
                             time.monotonic() - self.state.map_seen_t <= 3.0)
        if self.home.get('valid') and self.home.get('map_session') != map_session:
            self._invalidate_home('地图会话已变化')
        elif not localizing and not map_fresh and self.home.get('valid'):
            self._invalidate_home('实时地图已超时')

    def _publish_mode(self):
        self.mode_pub.publish(String(data=self.mode))

    def _update_state(self):
        with self.lock:
            nav = dict(getattr(self.state, 'nav', {}) or {})
            nav['active'] = self.nav_active
            self.state.nav = nav
            self.state.mode = self.mode
            self.state.home = dict(self.home)

    def _nav_state(self):
        with self.lock:
            nav = dict(getattr(self.state, 'nav', {}) or {})
        nav['active'] = self.nav_active
        return nav

    def _set_nav(self, **kw):
        with self.lock:
            nav = dict(getattr(self.state, 'nav', {}) or {})
            nav.update(kw)
            nav['active'] = self.nav_active
            self.state.nav = nav

    def _zero_cmd(self):
        self.cmd_pub.publish(Twist())

    # ---------------- 模式 ----------------
    def set_mode(self, mode):
        mode = mode.strip().lower()
        if mode not in ('manual', 'auto'):
            return '未知模式（manual/auto）'
        if mode == self.mode:
            return f'当前已是 {mode} 模式'
        if mode == 'manual' and self.nav_active:
            self._cancel_goal()
        self.mode = mode
        self._publish_mode()
        self._update_state()
        if mode == 'manual':
            self._zero_cmd()
        return '已切换到自动模式' if mode == 'auto' else '已切换到手动模式'

    # ---------------- 目标 ----------------
    def send_goal(self, x, y, yaw, source='web'):
        if not self.client.wait_for_server(timeout_sec=2.0):
            return 'Nav2 action server 不可用（导航节点没启动？）'
        self.set_mode('auto')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.nav_active = True
        self._goal_source = source
        extra = {'home_error': None} if source == 'home' else {}
        self._set_nav(status='sending', goal=[float(x), float(y), float(yaw)],
                      distance=None, eta=None, source=source, **extra)
        future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        return f'已发送目标 x={x:.2f} y={y:.2f} yaw={yaw:.2f}'

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.nav_active = False
            self._set_nav(status='error', message=str(exc))
            return
        if not handle.accepted:
            self.nav_active = False
            self._set_nav(status='rejected')
            self.mode = 'manual'
            self._publish_mode()
            self._zero_cmd()
            self._update_state()
            self.node.get_logger().warn('Nav2 拒绝目标')
            return
        self.goal_handle = handle
        self._set_nav(status='executing')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        fb = msg.feedback
        eta = getattr(fb, 'estimated_time_remaining', None)
        if eta is not None:
            eta = float(eta.sec) + float(eta.nanosec) * 1e-9
        self._set_nav(
            status='executing',
            distance=getattr(fb, 'distance_remaining', None),
            eta=eta,
            recoveries=getattr(fb, 'number_of_recoveries', None),
        )

    def _on_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.node.get_logger().error(f'导航结果异常: {exc}')
            status = None
        text = STATUS_TEXT.get(status, 'unknown')
        source = self._goal_source
        self._goal_source = None
        self.nav_active = False
        self.goal_handle = None
        self._set_nav(status=text, distance=None, eta=None)
        self._clear_path()
        if text in ('succeeded', 'canceled', 'aborted'):
            self.mode = 'manual'
            self._publish_mode()
            self._zero_cmd()
        self._update_state()
        self.node.get_logger().info(f'导航结束: {text}')
        if text == 'succeeded' and source in ('home', 'home_refine'):
            # 等 AMCL 再更新一次，量一下真正的到点误差（不自动重发，避免终点蠕动）
            with self.lock:
                pose0 = getattr(self.state, 'map_pose', None)
                t0 = getattr(self.state, 'map_pose_t', 0.0)
            threading.Thread(target=self._measure_home_error,
                             args=(pose0, t0), daemon=True).start()

    def _measure_home_error(self, pose_at_success=None, pose_t=0.0):
        time.sleep(1.0)
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=1.0))
        except Exception as exc:
            self.node.get_logger().warn(f'返航误差测量失败: {exc}')
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dx = float(t.x) - self.home['x']
        dy = float(t.y) - self.home['y']
        dist = math.hypot(dx, dy)
        dyaw = math.atan2(math.sin(yaw - self.home['yaw']),
                          math.cos(yaw - self.home['yaw']))
        # 成功瞬间到 1 s 后的位姿变化：>5 cm 说明 AMCL 停车后又修正/跳变，
        # 不是控制器停偏（用来区分"导航误差"和"定位收敛误差"）
        jump = None
        if pose_at_success is not None and pose_t > 0:
            jump = math.hypot(float(t.x) - pose_at_success[0],
                              float(t.y) - pose_at_success[1])
        err = {'distance': round(dist, 3),
               'dx': round(dx, 3), 'dy': round(dy, 3),
               'yaw': round(math.degrees(dyaw), 1)}
        if jump is not None:
            err['jump'] = round(jump, 3)
        self._set_nav(home_error=err)
        self.node.get_logger().info(
            f'返航终点误差 {dist * 100:.1f} cm（dx={dx * 100:.1f} '
            f'dy={dy * 100:.1f} cm, 航向 {math.degrees(dyaw):.1f}°'
            + (f', 停车后跳变 {jump * 100:.1f} cm' if jump is not None else '')
            + '）')
        # 自动微调默认关闭（home_refine_m=0）：近距离目标 DWB 会原地打转。
        # 需要时可把 home_refine_m 设成 0.08 打开，但只补一次。
        if (self.home_refine_m > 0 and dist > self.home_refine_m
                and not self._home_refine_used and not self.nav_active
                and self.home.get('valid')):
            self._home_refine_used = True
            res = self.send_goal(self.home['x'], self.home['y'],
                                 self.home['yaw'], source='home_refine')
            self.node.get_logger().info(
                f'返航误差 {dist * 100:.1f} cm > {self.home_refine_m * 100:.0f} cm，'
                f'补发一次微调目标：{res}')
            self.state.note('返航误差 %.1f cm，自动微调一次（车会再动一下）'
                            % (dist * 100))

    def _clear_path(self):
        with self.lock:
            self.state.path = []
            self.state.path_ver += 1

    def _cancel_goal(self):
        handle = self.goal_handle
        self.goal_handle = None
        self.nav_active = False
        if handle is not None:
            handle.cancel_goal_async()
        self._set_nav(status='canceled', distance=None, eta=None)
        self._clear_path()

    def cancel(self):
        self._cancel_goal()
        self.mode = 'manual'
        self._publish_mode()
        self._zero_cmd()
        self._update_state()
        return '已取消导航并切回手动'

    def estop(self):
        self._cancel_goal()
        self.mode = 'manual'
        self._publish_mode()
        self._zero_cmd()
        # 制动请求：桥接侧保持 0.5 s 后自动回落滑行，MCU 侧另有同样的兜底
        if getattr(self, 'pub_brake', None) is not None:
            self.pub_brake.publish(Bool(data=True))
        self._update_state()
        return '急停：已取消导航、切手动、制动 0.5 s 后滑行'

    # ---------------- 原点 ----------------
    def _invalidate_home(self, reason):
        if not self.home.get('valid'):
            return
        self.home['valid'] = False
        self.home['invalid_reason'] = reason
        self._save_home()
        self._update_state()
        self.node.get_logger().warn(f'返航原点已失效: {reason}')

    def _pose_samples(self, n=HOME_SAMPLES, span=HOME_SAMPLE_SPAN):
        """连续取 n 帧 map→base_link，用于取中位数去抖。"""
        out = []
        t0 = time.monotonic()
        while len(out) < n and time.monotonic() - t0 < span + 0.5:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_link', Time(),
                    timeout=Duration(seconds=0.2))
            except Exception:
                time.sleep(0.05)
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            out.append((float(t.x), float(t.y), yaw))
            time.sleep(0.08)
        return out

    def set_home(self):
        with self.lock:
            localizing = getattr(self.state, 'localizing', False)
            map_session = self.state.map_session
            map_stamp = self.state.map_stamp
            quality = getattr(self.state, 'loc_quality', None)
            if localizing:
                map_fresh = self.state.map_meta is not None
            else:
                map_fresh = (map_stamp is not None and
                             time.monotonic() - self.state.map_seen_t <= 3.0)
        if not map_fresh or (not localizing and map_stamp is None):
            return '当前实时地图不可用或已超时，不能设置返航原点'
        if localizing:
            if not quality:
                return '还没有定位质量数据，请等状态栏出现「匹配」后再设原点'
            if not quality.get('ok') or quality.get('inlier', 0.0) < HOME_MIN_INLIER:
                return ('定位匹配质量不足（inlier %.2f），请等状态栏匹配正常后再设原点'
                        % quality.get('inlier', 0.0))
        samples = self._pose_samples()
        if len(samples) < HOME_SAMPLES // 2:
            return '取当前位置失败（TF/AMCL 未就绪），请稍后再试'
        x = statistics.median(s[0] for s in samples)
        y = statistics.median(s[1] for s in samples)
        sy = sum(math.sin(s[2]) for s in samples)
        cy = sum(math.cos(s[2]) for s in samples)
        yaw = math.atan2(sy, cy)
        spread = max(math.hypot(s[0] - x, s[1] - y) for s in samples)
        previous_home = self.home
        self.home = {'x': x, 'y': y, 'yaw': yaw,
                     'source': 'web', 'stamp': time.time(),
                     'map_session': map_session, 'valid': True,
                     'invalid_reason': '',
                     'samples': len(samples), 'spread': round(spread, 3)}
        if not self._save_home():
            self.home = previous_home
            return '保存返航原点失败'
        self._update_state()
        msg = f'当前点已设为原点 x={x:.2f} y={y:.2f}（{len(samples)} 帧，抖动 {spread * 100:.1f} cm）'
        if spread > 0.05:
            msg += '；抖动偏大，建议静止后再设一次'
        return msg

    def return_home(self):
        with self.lock:
            current_session = self.state.map_session
            localizing = getattr(self.state, 'localizing', False)
            if localizing:
                map_fresh = self.state.map_meta is not None
            else:
                map_fresh = (self.state.map_stamp is not None and
                             time.monotonic() - self.state.map_seen_t <= 3.0)
            map_pose_t = getattr(self.state, 'map_pose_t', 0.0)
            pose_ok = (time.monotonic() - map_pose_t) <= 1.0
        if not self.home.get('valid'):
            return '返航原点无效，请在当前地图中重新设定'
        if self.home.get('map_session') != current_session:
            self._invalidate_home('地图会话已变化')
            return '返航原点属于旧地图，请重新设定'
        if not map_fresh:
            self._invalidate_home('实时地图不可用')
            return '实时地图不可用，已取消返航'
        if localizing and not pose_ok:
            return 'AMCL 位姿尚未就绪，请稍后再试或先设初始位姿'
        self._home_refine_used = False
        self._set_nav(home_error=None)
        return self.send_goal(self.home['x'], self.home['y'], self.home['yaw'],
                              source='home')

    # ---------------- 地图模式 ----------------
    def switch_map(self, map_id):
        """切到 AMCL 静态地图：新地图会话 + 旧 home 失效。"""
        with self.lock:
            self.state.localizing = True
            self.state.map_static = True
            self.state.map_id = map_id
            self.state.map_session = map_id
            self.state.map_stamp = None
            self.state.map_meta = None
            self.state.map_seen_t = 0.0
            self.state.map_ver += 1
            self.state.home_dist = None
        self.home = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'source': 'unset',
                     'stamp': 0, 'map_session': None, 'valid': False,
                     'invalid_reason': '地图已切换，请重新设原点'}
        self._save_home()
        self._update_state()

    def exit_localization(self):
        """切回 Cartographer 建图。"""
        with self.lock:
            self.state.localizing = False
            self.state.map_static = False
            self.state.map_id = None
            self.state.map_session = uuid.uuid4().hex
            self.state.map_stamp = None
            self.state.map_meta = None
            self.state.map_seen_t = 0.0
            self.state.map_ver += 1
            self.state.home_dist = None
        if self.home.get('valid'):
            self.home['valid'] = False
            self.home['invalid_reason'] = '已切回建图，请重新设原点'
            self._save_home()
        self._update_state()

    def set_initial_pose(self, x, y, yaw, trusted=False):
        with self.lock:
            localizing = getattr(self.state, 'localizing', False)
            have_map = self.state.map_meta is not None
        if not localizing:
            return '当前不是定位模式，不能设初始位姿'
        if not have_map:
            return 'AMCL 地图尚未加载，请稍后再试'
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        # 全局匹配可信时收紧协方差，让粒子快速收敛；手动点图/低置信度保持宽分布
        cov_xy = 0.04 if trusted else 0.25
        cov_yaw = 0.01 if trusted else 0.0685
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = cov_yaw
        self.pub_initialpose.publish(msg)
        return '初始位姿已发送 x=%.2f y=%.2f yaw=%.2f%s' % (
            x, y, yaw, '（高置信）' if trusted else '')

    def _load_home(self):
        try:
            with open(self.home_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            return {'x': float(data['x']), 'y': float(data['y']),
                    'yaw': float(data['yaw']),
                    'source': data.get('source', 'file'),
                    'stamp': data.get('stamp', 0),
                    'map_session': data.get('map_session'),
                    'valid': False,
                    'invalid_reason': '启动后必须在当前地图重新设定'}
        except Exception:
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'source': 'unset',
                    'stamp': 0, 'map_session': None, 'valid': False,
                    'invalid_reason': '尚未设置返航原点'}

    def _save_home(self):
        try:
            parent = os.path.dirname(self.home_file)
            os.makedirs(parent, exist_ok=True)
            with open(self.home_file, 'w', encoding='utf-8') as f:
                json.dump(self.home, f, ensure_ascii=False, indent=2)
            return True
        except Exception as exc:
            self.node.get_logger().error(f'保存原点失败: {exc}')
            return False

    # ---------------- Nav2 进程 ----------------
    def start_nav(self):
        if self.nav_proc is not None and self.nav_proc.poll() is None:
            return 'Nav2 已在运行'
        if subprocess.run(['pgrep', '-f', 'navigation_launch.py'],
                          capture_output=True).returncode == 0:
            return 'Nav2 已在运行（外部启动）'
        logf = open('/tmp/nav2_web.log', 'ab', buffering=0)
        cmd = ('source /opt/ros/humble/setup.bash && '
               'exec ros2 launch nav2_bringup navigation_launch.py '
               f'params_file:={self.nav_params} use_sim_time:=false')
        self.nav_proc = subprocess.Popen(
            ['bash', '-lc', cmd], stdout=logf, stderr=logf,
            start_new_session=True)
        self.node.get_logger().info(f'启动 Nav2 pid={self.nav_proc.pid}')
        return '正在启动 Nav2（约 5~10 s，日志 /tmp/nav2_web.log）'

    def stop_nav(self):
        if self.nav_active:
            self._cancel_goal()
        self.mode = 'manual'
        self._publish_mode()
        self._zero_cmd()
        self._update_state()

        proc = self.nav_proc
        self.nav_proc = None
        found = False
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception:
                proc.terminate()
            found = True
        else:
            r = subprocess.run(['pgrep', '-f', 'navigation_launch.py'],
                               capture_output=True, text=True)
            pids = [int(x) for x in r.stdout.split() if x.strip().isdigit()]
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGINT)
                except Exception:
                    pass
            found = bool(pids)

        # Nav2 生命周期清理要 5~10 s，等它真正退出再返回
        deadline = time.time() + 12.0
        while time.time() < deadline:
            r = subprocess.run(['pgrep', '-f', 'navigation_launch.py'],
                               capture_output=True)
            if r.returncode != 0:
                break
            time.sleep(0.3)
        else:
            subprocess.run(['pkill', '-TERM', '-f', 'navigation_launch.py'],
                           capture_output=True)
            time.sleep(1.0)
            subprocess.run(['pkill', '-KILL', '-f', 'navigation_launch.py'],
                           capture_output=True)

        # 清掉可能残留的 Nav2 子进程
        for pattern in ('nav2_controller', 'nav2_planner', 'nav2_bt_navigator',
                        'nav2_behaviors', 'nav2_smoother', 'nav2_waypoint_follower',
                        'nav2_velocity_smoother', 'nav2_lifecycle_manager'):
            subprocess.run(['pkill', '-INT', '-f', pattern], capture_output=True)
        return '已停止 Nav2' if found else 'Nav2 未在运行'
