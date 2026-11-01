#!/usr/bin/env python3
"""用雷达当尺子标定轮径：车前对墙直行，比较墙距变化与轮速里程。

不需要卷尺：
    d0 = 起步时雷达到墙的垂直距离
    d1 = 停车时雷达到墙的垂直距离
    真实位移 ≈ (d0 - d1) / cos(墙偏角)
    k = 真实位移 / 轮速直线位移；建议轮径 = 0.065 × k

用法（先 source ROS）：
    source /opt/ros/humble/setup.bash
    python3 ~/odom_lidar_calib.py

步骤：
  1. 车头尽量垂直对墙（脚本会显示偏角，< 5° 最好）；
     空间小也没关系，距墙 1.2–2 m、走 0.8–1.5 m 就够（雷达 ±1 cm，1 m 约 1%）；
  2. 看到 d0 后，慢速直行（不要撞墙），停稳；
  3. Ctrl+C，读 k；重复 3 次取平均。
"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

FRONT = math.pi                 # 车头在 laser 坐标中的角度（base→laser yaw=π）
WINDOW = math.radians(25)       # 取车头 ±25° 的墙面点拟合直线
RMIN, RMAX = 0.30, 6.0


class Calib(Node):
    def __init__(self):
        super().__init__('odom_lidar_calib')
        q = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.scan = None
        self.odom = None
        self.start = None
        self.path = 0.0
        self.create_subscription(LaserScan, '/scan', self.on_scan, q)
        self.create_subscription(Odometry, '/odom', self.on_odom, q)

    def on_scan(self, s):
        self.scan = s

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        cur = (float(p.x), float(p.y), yaw)
        if self.start is None:
            self.start = cur
        elif self.odom is not None:
            self.path += math.hypot(cur[0] - self.odom[0],
                                    cur[1] - self.odom[1])
        self.odom = cur


def wall_fit(scan):
    """返回 (垂直墙距, 墙相对车头偏角, 拟合残差, 点数)；失败 None。"""
    if scan is None:
        return None
    xs, ys = [], []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < RMIN or r > RMAX:
            continue
        a = scan.angle_min + i * scan.angle_increment
        da = (a - FRONT + math.pi) % (2.0 * math.pi) - math.pi
        if abs(da) > WINDOW:
            continue
        xs.append(r * math.cos(a))
        ys.append(r * math.sin(a))
    if len(xs) < 15:
        return None
    xs = np.asarray(xs)
    ys = np.asarray(ys)
    # 车头在 laser 的 -X 方向，墙面近似 x = a*y + c
    a, c = np.polyfit(ys, xs, 1)
    resid = float(np.std(xs - (a * ys + c)))
    dist = abs(c) / math.sqrt(1.0 + a * a)
    return dist, math.atan(a), resid, len(xs)


def main():
    rclpy.init()
    n = Calib()
    print('车头对墙（尽量垂直），距墙 2.5–4 m；直行 2–3 m 后 Ctrl+C。')
    start_wall = None
    t_print = 0.0
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.1)
            w = wall_fit(n.scan)
            if w is None or n.odom is None:
                continue
            if start_wall is None:
                start_wall = w
                print('\nd0 = %.3f m，墙偏角 %+.1f°，残差 %.3f m'
                      % (w[0], math.degrees(w[1]), w[2]))
                continue
            now = time.time()
            if now - t_print < 0.5:
                continue
            t_print = now
            chord = math.hypot(n.odom[0] - n.start[0],
                               n.odom[1] - n.start[1])
            dyaw = math.degrees(n.odom[2] - n.start[2])
            print('墙距 %5.3f m（偏角 %+5.1f°）  直线 %5.3f m  航向 %+5.1f°'
                  % (w[0], math.degrees(w[1]), chord, dyaw), end='\r')
    except KeyboardInterrupt:
        print()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    end_wall = wall_fit(n.scan)
    if start_wall is None or end_wall is None or n.start is None:
        print('没有拿到有效的墙面数据（车头前方要有 ≥0.3 m 的平墙）')
        return
    delta_wall = start_wall[0] - end_wall[0]
    theta = 0.5 * (abs(start_wall[1]) + abs(end_wall[1]))
    chord = math.hypot(n.odom[0] - n.start[0], n.odom[1] - n.start[1])
    dyaw = math.degrees(n.odom[2] - n.start[2])
    print('---')
    print('墙距变化 %.3f m（d0 %.3f → d1 %.3f，偏角 %.1f°）'
          % (delta_wall, start_wall[0], end_wall[0], math.degrees(theta)))
    print('轮速直线位移 %.3f m，航向变化 %+.1f°' % (chord, dyaw))
    if abs(dyaw) > 3.0 or math.degrees(theta) > 10.0:
        print('注意：航向 %.1f° / 墙偏角 %.1f°，结果仅供参考，建议重走直一些'
              % (dyaw, math.degrees(theta)))
    if delta_wall < 0.5 or chord < 0.4:
        print('位移太短（墙距变化 %.2f m），请至少走 0.8 m 再测' % delta_wall)
        return
    if start_wall[2] > 0.02 or end_wall[2] > 0.02:
        print('注意：墙面拟合残差偏大（%.3f m），前方可能不是平墙' % end_wall[2])
    true_move = delta_wall / max(0.9, math.cos(theta))
    k = true_move / chord
    print('k = %.4f → 建议轮径 %.4f m（当前 0.0650），空载理论 %.3f m/s'
          % (k, 0.065 * k, 320.0 / 60.0 * math.pi * 0.065 * k))
    print('精度估计：雷达 ±1 cm / 位移 %.2f m ≈ ±%.1f%%（4.5%% 的误差可以分辨）'
          % (delta_wall, 100.0 * 0.01 / max(0.3, delta_wall)))
    if delta_wall < 1.0:
        print('提示：位移偏短，建议再走一次 1.2–1.6 m，取多次平均更稳')


if __name__ == '__main__':
    main()
