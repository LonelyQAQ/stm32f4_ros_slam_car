import json
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BASE = 'http://127.0.0.1:8080'
TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/cmd_vel_safe'
rclpy.init()
n = Node('cmd_probe')
got = []
n.create_subscription(Twist, TOPIC, lambda m: got.append((m.linear.x, m.angular.z)), 10)

th = threading.Thread(target=lambda: rclpy.spin(n), daemon=True)
th.start()


def post(obj):
    req = urllib.request.Request(BASE + '/api', data=json.dumps(obj).encode(),
                                 headers={'Content-Type': 'application/json'}, method='POST')
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode())


# 等 DDS 发现完成
time.sleep(2.5)

# 按住 1 秒（模拟键盘长按，每 100 ms 发一次）
t0 = time.time()
while time.time() - t0 < 1.0:
    post({'cmd': 'vel', 'v': 0.12, 'w': -0.4})
    time.sleep(0.1)
nz = [g for g in got if g != (0.0, 0.0)]
print('while holding: %d msgs, nonzero %d, sample %s' % (len(got), len(nz), nz[0] if nz else None))

# 松手后等待死手开关
time.sleep(0.7)
tail = got[-6:]
print('after release, last 6:', tail)
ok = all(g == (0.0, 0.0) for g in tail)
print('deadman -> zero:', ok)

rclpy.shutdown()
