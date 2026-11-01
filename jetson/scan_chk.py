"""一次性检查 /scan：点数、角度范围、距离分布，并画一张 PNG。"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node('scan_chk')
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE)
got = []
n.create_subscription(LaserScan, '/scan', lambda m: got.append(m), qos)
t0 = time.time()
while len(got) < 2 and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)

if not got:
    print('no scan')
    raise SystemExit

s = got[-1]
rng = [r for r in s.ranges if math.isfinite(r) and s.range_min <= r <= s.range_max]
print('frame_id     ', s.header.frame_id)
print('points       ', len(s.ranges))
print('angle        %.1f .. %.1f deg (step %.2f)' % (
    math.degrees(s.angle_min), math.degrees(s.angle_max), math.degrees(s.angle_increment)))
print('range_limit  %.2f .. %.2f m' % (s.range_min, s.range_max))
if rng:
    rng.sort()
    print('valid pts    %d  min %.2f  median %.2f  max %.2f' % (
        len(rng), rng[0], rng[len(rng) // 2], rng[-1]))
    print('first 12     ', ' '.join('%.2f' % r for r in s.ranges[:12]))

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    xs, ys = [], []
    for i, r in enumerate(s.ranges):
        if math.isfinite(r) and s.range_min <= r <= s.range_max:
            a = s.angle_min + i * s.angle_increment
            xs.append(r * math.cos(a))
            ys.append(r * math.sin(a))
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(xs, ys, s=3, c='b')
    ax.plot([0], [0], 'r^', ms=8)
    lim = 6
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_title('T-mini Plus /scan  (%d pts)' % len(xs))
    fig.savefig('/tmp/scan_now.png', dpi=110)
    print('saved /tmp/scan_now.png')
except Exception as e:
    print('plot skipped:', e)

rclpy.shutdown()
