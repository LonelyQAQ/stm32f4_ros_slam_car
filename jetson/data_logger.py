#!/usr/bin/env python3
"""30 Hz 本地数据记录（txt），随雷达 launch 自动启动。

内容：IMU、MCU 里程/轮速/编码器/指令回显、EKF、雷达摘要、导航/定位/安全状态、
      Jetson CPU 温度/负载/内存。时间戳为 Jetson 本地时间。

文件：~/logs/car_YYYYmmdd_HHMMSS.txt，每 30 min 自动换一个；
      ~/logs/latest.txt 始终指向当前文件。
用法：source /opt/ros/humble/setup.bash && python3 ~/data_logger.py
      （正常由 ydlidar_tmini.launch.py 自动拉起；同一时间只允许一个实例）
"""
import datetime
import fcntl
import json
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray, String
from geometry_msgs.msg import Twist

RATE = 30.0
DURATION = 1800.0
RETENTION = 1800.0      # 超过 30 min 的旧记录自动删除（当前文件永不删）
LOG_DIR = os.path.expanduser('~/logs')
LOCK = '/tmp/car_data_logger.lock'

FIELDS = [
    't', 'roll', 'pitch', 'yaw', 'gx', 'gy', 'gz', 'ax', 'ay', 'az',
    'imu_temp', 'odom_x', 'odom_y', 'odom_yaw', 'v', 'w',
    'ekf_x', 'ekf_y', 'ekf_yaw', 'vl', 'vr', 'enc_l', 'enc_r',
    'cmd_v', 'cmd_w', 'echo_v', 'echo_w', 'scan_n', 'scan_min', 'scan_front',
    'bias_z', 'rest', 'zupt', 'mode', 'nav_mode', 'nav_status', 'nav_active',
    'localizing', 'map_id', 'loc_inlier', 'loc_ok', 'home_dist',
    'safety_blocked', 'safety_stale', 'cpu_c', 'load1', 'mem_free_mb',
]


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _rpy(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0,
                                    2.0 * (q.w * q.y - q.z * q.x))))
    return math.degrees(roll), math.degrees(pitch), math.degrees(_yaw(q))


class Logger(Node):
    def __init__(self):
        super().__init__('car_data_logger')
        qs = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        qr = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST)
        self.imu = self.temp = self.diag = None
        self.odom = self.ekf = None
        self.wheel = self.enc = self.echo = self.cmd = None
        self.scan = None
        self.nav_mode = ''
        self.status = {}
        self.safety = {}
        self._sys = (float('nan'),) * 3
        self._sys_t = 0.0
        self._file = None
        self._t0 = time.monotonic()
        self._flush_t = 0.0
        self._clean_t = 0.0
        self._path = ''

        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qs)
        self.create_subscription(Float32, '/imu/temp', self.on_temp, qs)
        self.create_subscription(Float32MultiArray, '/imu/diag',
                                 self.on_diag, qs)
        self.create_subscription(Odometry, '/odom', self.on_odom, qs)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self.on_ekf, qs)
        self.create_subscription(Float32MultiArray, '/wheel_speed',
                                 self.on_wheel, qs)
        self.create_subscription(Int32MultiArray, '/enc_ticks',
                                 self.on_enc, qs)
        self.create_subscription(Twist, '/cmd_echo', self.on_echo, qs)
        self.create_subscription(Twist, '/cmd_vel_safe', self.on_cmd, qs)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qs)
        self.create_subscription(String, '/nav_mode', self.on_nav_mode, qr)
        self.create_subscription(String, '/safety_guard/state',
                                 self.on_safety, qr)
        self.create_subscription(String, '/webui/status', self.on_status, qr)
        self.create_timer(1.0 / RATE, self.tick)

    # ---- 订阅 ----
    def on_imu(self, m):
        self.imu = m

    def on_temp(self, m):
        self.temp = m.data

    def on_diag(self, m):
        self.diag = list(m.data)

    def on_odom(self, m):
        self.odom = m

    def on_ekf(self, m):
        self.ekf = m

    def on_wheel(self, m):
        self.wheel = list(m.data)

    def on_enc(self, m):
        self.enc = list(m.data)

    def on_echo(self, m):
        self.echo = m

    def on_cmd(self, m):
        self.cmd = m

    def on_scan(self, m):
        self.scan = m

    def on_nav_mode(self, m):
        self.nav_mode = m.data

    def on_safety(self, m):
        try:
            self.safety = json.loads(m.data)
        except Exception:
            self.safety = {}

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except Exception:
            self.status = {}

    # ---- 系统信息（1 Hz 缓存）----
    def _sysinfo(self):
        now = time.monotonic()
        if now - self._sys_t < 1.0:
            return self._sys
        self._sys_t = now
        cpu = float('nan')
        try:
            vals = []
            for z in os.listdir('/sys/class/thermal'):
                if not z.startswith('thermal_zone'):
                    continue
                p = '/sys/class/thermal/%s/temp' % z
                if os.path.isfile(p):
                    vals.append(int(open(p).read()) / 1000.0)
            if vals:
                cpu = max(vals)
        except Exception:
            pass
        try:
            load1 = os.getloadavg()[0]
        except Exception:
            load1 = float('nan')
        mem = float('nan')
        try:
            for line in open('/proc/meminfo'):
                if line.startswith('MemAvailable:'):
                    mem = int(line.split()[1]) / 1024.0
                    break
        except Exception:
            pass
        self._sys = (cpu, load1, mem)
        return self._sys

    # ---- 文件 ----
    def _open_file(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._path = os.path.join(LOG_DIR, 'car_%s.txt' % stamp)
        self._file = open(self._path, 'a', encoding='utf-8', buffering=1)
        self._file.write('# SLAM car data log\n')
        self._file.write('# started %s (Jetson local, TZ %s)\n' % (
            datetime.datetime.now().astimezone().isoformat(),
            time.strftime('%z')))
        self._file.write('# rate %.0f Hz, rotate %.0f s, unit: SI '
                         '(m, m/s, rad, rad/s, deg for attitude/temp)\n'
                         % (RATE, DURATION))
        self._file.write('# ' + ' '.join(FIELDS) + '\n')
        self._file.write('# mode/nav_mode/nav_status/map_id are text; '
                         'missing values are nan\n')
        self._file.flush()
        link = os.path.join(LOG_DIR, 'latest.txt')
        try:
            if os.path.islink(link) or os.path.exists(link):
                os.remove(link)
            os.symlink(self._path, link)
        except Exception:
            pass
        self._cleanup()
        self.get_logger().info('数据记录: %s' % self._path)

    def _cleanup(self):
        """删除超过 RETENTION 的旧记录（当前文件与 latest 链接不动）。"""
        now = time.time()
        try:
            names = os.listdir(LOG_DIR)
        except OSError:
            return
        for name in names:
            if not (name.startswith('car_') and name.endswith('.txt')):
                continue
            p = os.path.join(LOG_DIR, name)
            if p == self._path:
                continue
            try:
                if now - os.path.getmtime(p) > RETENTION:
                    os.remove(p)
                    self.get_logger().info('清理旧记录: %s' % name)
            except OSError:
                pass

    def _rotate_if_needed(self):
        if self._file is None:
            self._open_file()
        elif time.monotonic() - self._t0 >= DURATION:
            self._file.close()
            self._t0 = time.monotonic()
            self._open_file()

    # ---- 采样 ----
    def tick(self):
        self._rotate_if_needed()
        now = datetime.datetime.now()
        ts = now.strftime('%Y-%m-%dT%H:%M:%S.') + '%03d' % (now.microsecond // 1000)
        vals = [ts]
        imu = self.imu
        if imu is None:
            vals += ['nan'] * 10
        else:
            r, p, y = _rpy(imu.orientation)
            av = imu.angular_velocity
            la = imu.linear_acceleration
            vals += ['%.4f' % v for v in (r, p, y,
                                          av.x, av.y, av.z,
                                          la.x, la.y, la.z)]
            vals.append('nan' if self.temp is None else '%.2f' % self.temp)
        vals += self._pose(self.odom)
        vals += self._twist(self.odom)
        vals += self._pose(self.ekf)
        vals += self._vec(self.wheel, 2)
        vals += self._vec(self.enc, 2)
        vals += self._twist(self.cmd)
        vals += self._twist(self.echo)
        vals += self._scan()
        vals += self._vec(self.diag, 3)
        st = self.status
        vals += [str(st.get('mode', '')), self.nav_mode or '',
                 str(st.get('nav_status', '')),
                 '1' if st.get('nav_active') else '0',
                 '1' if st.get('localizing') else '0',
                 str(st.get('map_id', '')),
                 self._f(st.get('loc_inlier')),
                 '1' if st.get('loc_ok') else '0',
                 self._f(st.get('home_dist'))]
        blocked = self.safety.get('blocked') or []
        vals.append('|'.join(str(b) for b in blocked) if blocked else '-')
        vals.append('1' if self.safety.get('stale') else '0')
        cpu, load1, mem = self._sysinfo()
        vals += ['%.1f' % cpu if cpu == cpu else 'nan',
                 '%.3f' % load1 if load1 == load1 else 'nan',
                 '%.0f' % mem if mem == mem else 'nan']
        self._file.write(' '.join(vals) + '\n')
        if time.monotonic() - self._flush_t >= 1.0:
            self._flush_t = time.monotonic()
            self._file.flush()
        if time.monotonic() - self._clean_t >= 60.0:
            self._clean_t = time.monotonic()
            self._cleanup()

    @staticmethod
    def _f(v):
        try:
            return '%.4f' % float(v)
        except (TypeError, ValueError):
            return 'nan'

    def _pose(self, m):
        if m is None:
            return ['nan'] * 3
        p = m.pose.pose.position
        return ['%.4f' % p.x, '%.4f' % p.y, '%.4f' % _yaw(m.pose.pose.orientation)]

    def _twist(self, m):
        """兼容 Odometry（/odom）和 Twist（/cmd_vel_safe、/cmd_echo）。"""
        if m is None:
            return ['nan'] * 2
        tw = m.twist.twist if hasattr(m, 'twist') else m
        return ['%.4f' % tw.linear.x, '%.4f' % tw.angular.z]

    @staticmethod
    def _vec(v, n):
        if not v or len(v) < n:
            return ['nan'] * n
        return ['%.4f' % x for x in v[:n]]

    def _scan(self):
        s = self.scan
        if s is None:
            return ['nan'] * 3
        rmin, rmax = float(s.range_min), float(s.range_max)
        n = 0
        lo = float('inf')
        front = []
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            n += 1
            lo = min(lo, r)
            a = s.angle_min + i * s.angle_increment
            da = (a - math.pi + math.pi) % (2 * math.pi) - math.pi
            if abs(da) < math.radians(5.0):
                front.append(r)
        f = sorted(front)[len(front) // 2] if front else float('nan')
        return ['%d' % n,
                '%.3f' % lo if n else 'nan',
                '%.3f' % f if front else 'nan']


def main():
    lock = open(LOCK, 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print('已有 data_logger 在运行，退出')
        return
    lock.write(str(os.getpid()))
    lock.flush()
    rclpy.init()
    node = Logger()
    try:
        rclpy.spin(node)
    finally:
        if node._file is not None:
            node._file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
