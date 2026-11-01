"""把 /map + 当前 /scan 画成 PNG，便于在没有 RViz 的情况下看建图效果。"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/map_now.png'

rclpy.init()
n = Node('map_png')

q_latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL)
q_scan = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
maps, scans = [], []
n.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m), q_latch)
n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), q_scan)

t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
    if maps and scans:
        break

if not maps:
    print('no /map')
    raise SystemExit

m = maps[-1]
w, h, res = m.info.width, m.info.height, m.info.resolution
ox = m.info.origin.position.x
oy = m.info.origin.position.y
print('map %dx%d  res %.3f m  origin (%.2f, %.2f)  cells=%d' % (w, h, res, ox, oy, len(m.data)))
known = sum(1 for v in m.data if v >= 0)
occ = sum(1 for v in m.data if v > 50)
print('known %d  occupied %d  (%.1f%%)' % (known, occ, 100.0 * known / max(1, w * h)))

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    grid = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            v = m.data[y * w + x]
            grid[y][x] = 2 if v > 50 else (1 if v >= 0 else 0)
    cmap = ListedColormap(['#dddddd', '#ffffff', '#000000'])

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(grid, cmap=cmap, origin='lower',
              extent=(ox, ox + w * res, oy, oy + h * res))
    if scans:
        s = scans[-1]
        xs, ys = [], []
        for i, r in enumerate(s.ranges):
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                a = s.angle_min + i * s.angle_increment
                xs.append(r * math.cos(a))
                ys.append(r * math.sin(a))
        ax.scatter(xs, ys, s=2, c='r', label='current scan')
    ax.plot([0], [0], 'b^', ms=9, label='robot')
    ax.set_title('map (%d occupied cells)' % occ)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(alpha=0.25)
    ax.legend(loc='upper right', fontsize=8)
    fig.savefig(OUT, dpi=110, bbox_inches='tight')
    print('saved', OUT)
except Exception as e:
    print('plot failed:', e)

rclpy.shutdown()
