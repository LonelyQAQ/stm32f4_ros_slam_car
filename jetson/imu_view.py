#!/usr/bin/env python3
"""BMI088 实时观测：/imu/data_raw + /imu/temp 单行刷新显示。

用法：
    source /opt/ros/humble/setup.bash
    python3 ~/imu_view.py            # 默认 5 Hz 刷新
    python3 ~/imu_view.py 0.2        # 自定义刷新周期（秒）
Ctrl+C 退出。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2


class Viewer(Node):
    def __init__(self):
        super().__init__('imu_view')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.imu = None
        self.temp = None
        self.n = 0
        self.win = []
        self.last_rx = 0.0
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos)
        self.create_subscription(Float32, '/imu/temp', self.on_temp, qos)
        self.create_timer(RATE, self.report)
        print('  acc[m/s2]   x       y       z     |g|    gyro[dps]  x      y      z      temp   rate')

    def on_imu(self, m):
        self.imu = m
        self.n += 1
        self.last_rx = time.time()
        self.win.append(self.last_rx)
        self.win = [t for t in self.win if self.last_rx - t < 2.0]

    def on_temp(self, m):
        self.temp = m.data

    def report(self):
        if self.imu is None or time.time() - self.last_rx > 1.0:
            print('  (等待 /imu/data_raw ...)', end='\r', flush=True)
            return
        a = self.imu.linear_acceleration
        g = self.imu.angular_velocity
        mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        hz = len(self.win) / 2.0
        tp = self.temp if self.temp is not None else float('nan')
        print('          %+6.2f %+6.2f %+6.2f %5.2f          %+6.1f %+6.1f %+6.1f %5.1fC %4.1fHz n=%d  '
              % (a.x, a.y, a.z, mag,
                 g.x * 57.2958, g.y * 57.2958, g.z * 57.2958, tp, hz, self.n),
              end='\r', flush=True)


def main():
    rclpy.init()
    v = Viewer()
    try:
        rclpy.spin(v)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print()
        v.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
