#!/usr/bin/env python3
"""直行标定助手：实时显示轮速里程与 EKF 里程，退出时按卷尺距离算比例。

用法（车上，先让车静止在起点；必须先 source ROS，否则 import rclpy 会失败）：
    source /opt/ros/humble/setup.bash
    python3 ~/odom_tape.py                 # 直行到终点后 Ctrl+C，再输入卷尺距离
    python3 ~/odom_tape.py --tape 2.0      # 启动时就知道实测距离，退出直接给结果

输出：
    mcu  = /odom（STM32 用编码器积分，轮径/CPR 的直接体现）
    ekf  = /odometry/filtered（Nav2 实际使用的里程）
    k    = 实测距离 / 里程；建议 WHEEL_DIAMETER_M = 0.065 × k
"""
import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)


class Tape(Node):
    def __init__(self):
        super().__init__('odom_tape')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.mcu = None
        self.ekf = None
        self.mcu_start = None
        self.ekf_start = None
        self.mcu_path = 0.0
        self.ekf_path = 0.0
        self.mcu_yaw = 0.0
        self.create_subscription(Odometry, '/odom', self.on_mcu, qos)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self.on_ekf, qos)

    @staticmethod
    def _pose(m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (float(p.x), float(p.y), yaw)

    def on_mcu(self, m):
        p = self._pose(m)
        if self.mcu_start is None:
            self.mcu_start = p
        elif self.mcu is not None:
            self.mcu_path += math.hypot(p[0] - self.mcu[0], p[1] - self.mcu[1])
        self.mcu = p

    def on_ekf(self, m):
        p = self._pose(m)
        if self.ekf_start is None:
            self.ekf_start = p
        elif self.ekf is not None:
            self.ekf_path += math.hypot(p[0] - self.ekf[0], p[1] - self.ekf[1])
        self.ekf = p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tape', type=float, default=None,
                    help='卷尺实测距离（m），不给则退出时输入')
    args = ap.parse_args()

    rclpy.init()
    n = Tape()
    print('起点已记录。直行到终点后 Ctrl+C。')
    t0 = time.time()
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - t0 < 0.5:
                continue
            t0 = time.time()
            if n.mcu is None:
                print('等待 /odom ...', end='\r')
                continue
            yaw = math.degrees(n.mcu[2] - n.mcu_start[2])
            chord = math.hypot(n.mcu[0] - n.mcu_start[0],
                               n.mcu[1] - n.mcu_start[1])
            print('mcu 路径 %5.3f / 直线 %5.3f m   ekf %5.3f m   航向 %+6.1f°   %s'
                  % (n.mcu_path, chord, n.ekf_path, yaw,
                     time.strftime('%H:%M:%S')), end='\r')
    except KeyboardInterrupt:
        print()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if n.mcu_start is None:
        print('没有收到 /odom，无法标定')
        return
    tape = args.tape
    if tape is None:
        try:
            tape = float(input('卷尺实测距离 (m): ').strip())
        except Exception:
            print('未输入实测距离，退出')
            return
    yaw = math.degrees(n.mcu[2] - n.mcu_start[2])
    chord = math.hypot(n.mcu[0] - n.mcu_start[0], n.mcu[1] - n.mcu_start[1])
    e_chord = (math.hypot(n.ekf[0] - n.ekf_start[0], n.ekf[1] - n.ekf_start[1])
               if n.ekf and n.ekf_start else 0.0)
    if abs(yaw) > 5.0:
        print('注意：航向变化 %.1f°，路径不够直，结果仅供参考' % yaw)
    if chord < 0.5:
        print('直线位移太短（%.3f m），请直行 ≥1 m 再测' % chord)
        return
    km = tape / chord                 # 卷尺量的是起点到终点的直线
    ke = tape / e_chord if e_chord > 0.01 else float('nan')
    print('---')
    print('mcu 路径 %.3f m / 直线 %.3f m → k=%.4f，建议轮径 %.4f m（当前 0.0650）'
          % (n.mcu_path, chord, km, 0.065 * km))
    print('ekf 路径 %.3f m / 直线 %.3f m → k=%.4f'
          % (n.ekf_path, e_chord, ke))
    if n.mcu_path / chord > 1.02:
        print('提示：路径/直线 = %.3f，明显走弯了，建议再走一次更直的' %
              (n.mcu_path / chord))
    print('理论：1320 counts/转，0.1547 mm/count；%.3f m ≈ %d counts'
          % (tape, round(tape / (math.pi * 0.065 / 1320))))


if __name__ == '__main__':
    main()
