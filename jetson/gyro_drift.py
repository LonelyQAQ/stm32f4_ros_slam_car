import statistics as st
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

DUR = 30.0
rclpy.init()
n = Node('gyro_drift')
qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
rec = []
n.create_subscription(Imu, '/imu/data_raw', lambda m: rec.append(m), qos)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)

gz = [m.angular_velocity.z for m in rec]
gx = [m.angular_velocity.x for m in rec]
gy = [m.angular_velocity.y for m in rec]
span = rec[-1].header.stamp.sec - rec[0].header.stamp.sec + \
    (rec[-1].header.stamp.nanosec - rec[0].header.stamp.nanosec) * 1e-9
print('samples %d  span %.1f s  rate %.1f Hz' % (len(rec), span, len(rec) / span))
for name, v in (('gx', gx), ('gy', gy), ('gz', gz)):
    print('%s mean %+7.4f dps  std %6.4f  min %+7.4f  max %+7.4f'
          % (name, st.mean(v) * 57.2958, st.pstdev(v) * 57.2958,
             min(v) * 57.2958, max(v) * 57.2958))
mz = st.mean(gz) * 57.2958
print('---')
print('if bias stayed %.2f dps: yaw drift %.1f deg/min, %.0f deg/hour' % (mz, mz * 60, mz * 3600))
print('this %.0f s segment drifted %.1f deg' % (span, mz * span))
rclpy.shutdown()
