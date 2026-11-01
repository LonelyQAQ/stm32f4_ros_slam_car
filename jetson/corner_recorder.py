#!/usr/bin/env python3
"""45 秒墙角掉头/返航记录器（只读，不发任何控制）"""
import json
import math
import os
import sys
import threading
import time
import urllib.request
from datetime import datetime

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import Buffer, TransformListener

DUR = 45.0
INF = float('inf')
q_best = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    durability=DurabilityPolicy.VOLATILE)
q_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


rclpy.init()
node = Node('corner_recorder')
data = {'odom': None, 'cmd': None, 'manual': None, 'raw': None,
        'safe': None, 'echo': None, 'enc': None, 'safety': None,
        'scan': None, 'sse': None, 'map_info': None}
tf_buffer = Buffer()
TransformListener(tf_buffer, node)


def on_odom(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    data['odom'] = (round(p.x, 3), round(p.y, 3), round(yaw_of_q(q), 4),
                    round(m.twist.twist.linear.x, 3),
                    round(m.twist.twist.angular.z, 3))


def on_twist(key):
    def cb(m):
        data[key] = (round(m.linear.x, 3), round(m.angular.z, 3))
    return cb


def on_enc(m):
    data['enc'] = (int(m.data[0]), int(m.data[1]))


def on_safety(m):
    try:
        data['safety'] = json.loads(m.data)
    except Exception:
        pass


def on_scan(m):
    data['scan'] = (tuple(m.ranges), float(m.angle_min),
                    float(m.angle_increment))


def on_map(m):
    i = m.info
    data['map_info'] = {
        'w': int(i.width), 'h': int(i.height), 'res': float(i.resolution),
        'ox': round(float(i.origin.position.x), 3),
        'oy': round(float(i.origin.position.y), 3),
        'stamp': (int(m.header.stamp.sec), int(m.header.stamp.nanosec)),
    }


node.create_subscription(Odometry, '/odom', on_odom, q_best)
node.create_subscription(Twist, '/cmd_vel', on_twist('cmd'), 10)
node.create_subscription(Twist, '/cmd_vel_manual', on_twist('manual'), 10)
node.create_subscription(Twist, '/cmd_vel_raw', on_twist('raw'), 10)
node.create_subscription(Twist, '/cmd_vel_safe', on_twist('safe'), 10)
node.create_subscription(Twist, '/cmd_echo', on_twist('echo'), q_best)
node.create_subscription(Int32MultiArray, '/enc_ticks', on_enc, q_best)
node.create_subscription(String, '/safety_guard/state', on_safety, 10)
node.create_subscription(LaserScan, '/scan', on_scan, q_best)
node.create_subscription(OccupancyGrid, '/map', on_map, q_map)


def sse_reader():
    while True:
        try:
            with urllib.request.urlopen(
                    'http://127.0.0.1:8080/stream', timeout=10) as r:
                ev = None
                for raw in r:
                    line = raw.decode('utf-8', 'replace').rstrip('\n')
                    if line.startswith('event: '):
                        ev = line[7:].strip()
                    elif line.startswith('data: ') and ev == 'state':
                        try:
                            data['sse'] = json.loads(line[6:])
                        except Exception:
                            pass
        except Exception:
            time.sleep(0.5)


threading.Thread(target=sse_reader, daemon=True).start()
executor = SingleThreadedExecutor()
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()


def sector_minima(scan):
    if scan is None:
        return {}
    ranges, amin, inc = scan
    mins = {'front': INF, 'rear': INF, 'left': INF, 'right': INF}
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < 0.1 or r > 6.0:
            continue
        a = wrap(amin + i * inc + math.pi)
        if abs(a) <= math.radians(40):
            s = 'front'
        elif abs(a) >= math.pi - math.radians(40):
            s = 'rear'
        else:
            s = 'left' if a > 0 else 'right'
        if r < mins[s]:
            mins[s] = r
    return {k: (None if math.isinf(v) else round(v, 3))
            for k, v in mins.items()}


records = []
t0 = time.monotonic()


def tick():
    rec = {'t': round(time.monotonic() - t0, 3)}
    rec['odom'] = data['odom']
    try:
        tf = tf_buffer.lookup_transform('map', 'base_link', Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        rec['map'] = (round(t.x, 3), round(t.y, 3), round(yaw_of_q(q), 4))
    except Exception:
        rec['map'] = None
    for k in ('cmd', 'manual', 'raw', 'safe', 'echo', 'enc'):
        rec[k] = data[k]
    rec['scan'] = sector_minima(data['scan'])
    rec['map_info'] = data['map_info']
    safety = data['safety'] or {}
    rec['safety'] = {'blocked': safety.get('blocked'),
                     'min': safety.get('min'),
                     'components': safety.get('blocked_components'),
                     'reason': safety.get('reason')}
    sse = data['sse'] or {}
    nav = sse.get('nav') or {}
    rec['mode'] = sse.get('mode')
    rec['paused'] = sse.get('paused')
    rec['nav'] = {'active': nav.get('active'), 'status': nav.get('status'),
                  'goal': nav.get('goal'), 'distance': nav.get('distance'),
                  'recoveries': nav.get('recoveries'),
                  'source': nav.get('source')}
    home = sse.get('home') or {}
    rec['home'] = {'x': home.get('x'), 'y': home.get('y'),
                   'yaw': home.get('yaw'), 'valid': home.get('valid'),
                   'reason': home.get('invalid_reason')}
    records.append(rec)


node.create_timer(0.1, tick)
print('REC START', datetime.now().strftime('%H:%M:%S'), flush=True)
time.sleep(DUR)
ts = datetime.now().strftime('%Y%m%d_%H%M%S')
path = f'/tmp/corner_run_{ts}.jsonl'
with open(path, 'w', encoding='utf-8') as f:
    for r in records:
        f.write(json.dumps(r, ensure_ascii=False) + '\n')
print('RECORD', path, 'samples', len(records),
      'duration', round(time.monotonic() - t0, 1), flush=True)
print('map_info', records[-1]['map_info'] if records else None, flush=True)

last = (None, None, None)
for r in records:
    key = (r['mode'], (r['nav'] or {}).get('status'),
           tuple((r['safety'] or {}).get('blocked') or []))
    if key != last:
        print('%6.2fs mode=%s nav=%s dist=%s home=%s blocked=%s min=%s' % (
            r['t'], r['mode'], (r['nav'] or {}).get('status'),
            (r['nav'] or {}).get('distance'), r['home'],
            (r['safety'] or {}).get('blocked'), (r['safety'] or {}).get('min')),
            flush=True)
        last = key

print('---- 2s samples ----', flush=True)
next_t = 0.0
for r in records:
    if r['t'] >= next_t:
        print('%6.2fs odom=%s map=%s man=%s raw=%s safe=%s echo=%s enc=%s scan=%s' % (
            r['t'], r['odom'], r['map'], r['manual'], r['raw'], r['safe'],
            r['echo'], r['enc'], r['scan']), flush=True)
        next_t = r['t'] + 2.0


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]) if a and b else 0.0


maxdiff = max((dist(r['map'], r['odom']) for r in records), default=0.0)
maxjump = 0.0
jump_t = None
for a, b in zip(records, records[1:]):
    if a['map'] and b['map']:
        d = dist(a['map'], b['map'])
        if d > maxjump:
            maxjump = d
            jump_t = b['t']
print('max |map-odom| = %.3f m' % maxdiff, flush=True)
print('max map jump between 0.1s samples = %.3f m at t=%s' % (maxjump, jump_t),
      flush=True)
if records:
    e0 = records[0]['enc']
    e1 = records[-1]['enc']
    if e0 and e1:
        print('enc delta L=%d R=%d' % (e1[0] - e0[0], e1[1] - e0[1]), flush=True)
    print('final map=%s odom=%s nav=%s home=%s' % (
        records[-1]['map'], records[-1]['odom'], records[-1]['nav'],
        records[-1]['home']), flush=True)
sys.stdout.flush()
# 记录器是一次性调试进程；直接退出，避免 rclpy 后台线程在关闭时刷 terminate 异常
os._exit(0)
