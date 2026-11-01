"""把 /imu/data_raw 的姿态按时间打印成表，用来对照"第几秒做了什么动作"。
用法：python3 ~/att_log.py [秒数] [间隔秒]
输出：每行 "t=xx.x  roll=..  pitch=..  yaw=..  |q|=.."
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
STEP = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

rclpy.init()
n = Node('att_log')
qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
last = {'m': None}
n.create_subscription(Imu, '/imu/data_raw', lambda m: last.__setitem__('m', m), qos)

t0 = time.time()
print('t(s)    roll     pitch    yaw     |q|')
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.02)
    el = time.time() - t0
    m = last['m']
    if m is None:
        continue
    q = (m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z)
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 57.2958
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))) * 57.2958
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 57.2958
    nq = math.sqrt(sum(v * v for v in q))
    print('%5.1f  %+7.2f  %+7.2f  %+7.2f  %.5f' % (el, roll, pitch, yaw, nq), flush=True)
    time.sleep(STEP)

rclpy.shutdown()
