"""读 /imu/data_raw 的 VQF 四元数，转欧拉角打印统计。
用法：python3 ~/att_chk.py [秒数]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0

rclpy.init()
n = Node('att_chk')
qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
rec = []
n.create_subscription(Imu, '/imu/data_raw', lambda m: rec.append(m), qos)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)


def euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll * 57.2958, pitch * 57.2958, yaw * 57.2958


def mean(v):
    return sum(v) / len(v)


def std(v):
    m = mean(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


E = [euler((m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z)) for m in rec]
norm = [math.sqrt(m.orientation.w ** 2 + m.orientation.x ** 2 + m.orientation.y ** 2 + m.orientation.z ** 2)
        for m in rec]
print('samples %d  span %.1f s' % (len(rec), time.time() - t0))
if not rec:
    print('no data')
    raise SystemExit
print('quat  [w %.4f  x %.4f  y %.4f  z %.4f]'
      % (rec[-1].orientation.w, rec[-1].orientation.x, rec[-1].orientation.y, rec[-1].orientation.z))
print('|q|   mean %.5f  min %.5f  max %.5f' % (mean(norm), min(norm), max(norm)))
for name, i in (('roll ', 0), ('pitch', 1), ('yaw  ', 2)):
    v = [e[i] for e in E]
    print('%s mean %+7.2f  std %5.2f  min %+7.2f  max %+7.2f  (°)' % (name, mean(v), std(v), min(v), max(v)))
rclpy.shutdown()
