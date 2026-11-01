#!/usr/bin/env python3
"""近距避障安全层

输入：/scan（雷达）、/cmd_vel_raw（仲裁后的速度）
输出：/cmd_vel_safe（最终给 STM32）、/safety_guard/state（JSON 状态）

触发条件（每个方向扇区）：
  1. 0.1–6.0 m 内的有效点；
  2. 相邻点组成点簇，簇内至少 3 个点；
  3. 连续 3 帧同一扇区都有该簇；
  4. 簇的最小距离没有明显变大（稳定或逼近），且簇中心没有跳变。
解除条件：该方向连续 3 帧最近点都大于 0.7 m（回差）。

只拦截“朝障碍物方向”的运动：前进受 front 约束、后退受 rear 约束、
转向受 side_clearance 约束；不替代 Nav2 的代价地图，只做最后一道保险。
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

SECTORS = ('front', 'rear', 'left', 'right')
INF = float('inf')


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def angle_distance(a, b):
    return abs(wrap_angle(a - b))


class SafetyGuard(Node):
    def __init__(self):
        super().__init__('safety_guard')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_in_topic', '/cmd_vel_raw')
        self.declare_parameter('cmd_out_topic', '/cmd_vel_safe')
        self.declare_parameter('state_topic', '/safety_guard/state')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('avoid_distance_m', 0.5)
        self.declare_parameter('release_distance_m', 0.7)
        self.declare_parameter('confirm_scans', 3)
        self.declare_parameter('release_scans', 3)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('neighbor_angle_deg', 8.0)
        self.declare_parameter('front_half_angle_deg', 40.0)
        self.declare_parameter('rear_half_angle_deg', 40.0)
        self.declare_parameter('side_clearance_m', 0.3)
        self.declare_parameter('input_timeout_ms', 300)
        self.declare_parameter('scan_timeout_ms', 800)
        self.declare_parameter('min_range_m', 0.1)
        self.declare_parameter('max_range_m', 6.0)
        self.declare_parameter('scan_yaw_offset_rad', 0.0)
        self.declare_parameter('approach_tolerance_m', 0.05)
        self.declare_parameter('cluster_center_tol_deg', 12.0)

        p = self.get_parameter
        self.rate_hz = float(p('rate_hz').value)
        self.avoid_distance = float(p('avoid_distance_m').value)
        self.release_distance = float(p('release_distance_m').value)
        self.confirm_scans = int(p('confirm_scans').value)
        self.release_scans = int(p('release_scans').value)
        self.min_cluster = int(p('min_cluster_points').value)
        self.neighbor_gap = math.radians(float(p('neighbor_angle_deg').value))
        self.front_half = math.radians(float(p('front_half_angle_deg').value))
        self.rear_half = math.radians(float(p('rear_half_angle_deg').value))
        self.side_clearance = float(p('side_clearance_m').value)
        self.input_timeout = float(p('input_timeout_ms').value) / 1000.0
        self.scan_timeout = float(p('scan_timeout_ms').value) / 1000.0
        self.min_range = float(p('min_range_m').value)
        self.max_range = float(p('max_range_m').value)
        self.scan_yaw_offset = float(p('scan_yaw_offset_rad').value)
        self.approach_tol = float(p('approach_tolerance_m').value)
        self.center_tol = math.radians(float(p('cluster_center_tol_deg').value))

        self.cmd = None
        self.cmd_t = 0.0
        self.scan = None
        self.scan_t = 0.0
        self.scan_seq = 0
        self.last_eval_seq = -1
        self.last_state_t = 0.0
        self.hits = {s: 0 for s in SECTORS}
        self.clears = {s: 0 for s in SECTORS}
        self.blocked = {s: False for s in SECTORS}
        self.last_min = {s: None for s in SECTORS}
        self.last_center = {s: None for s in SECTORS}
        self.sector_min = {s: INF for s in SECTORS}
        self.cluster_min = {s: INF for s in SECTORS}
        self.last_blocked_components = {'linear_x': False, 'angular_z': False}

        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, p('scan_topic').value, self.on_scan, scan_qos)
        self.create_subscription(Twist, p('cmd_in_topic').value, self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, p('cmd_out_topic').value, 10)
        self.pub_state = self.create_publisher(String, p('state_topic').value, 10)
        # 只有"避障真的拦下了一个速度分量"才请求制动；超时/雷达丢帧只停车不刹
        self.pub_brake = self.create_publisher(Bool, '/cmd_brake', 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f'safety_guard: avoid={self.avoid_distance:.2f} m confirm={self.confirm_scans} '
            f'scans / {self.rate_hz:.0f} Hz')

    # ---------------- 输入 ----------------
    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_t = time.monotonic()

    def on_scan(self, msg):
        self.scan = (tuple(msg.ranges), float(msg.angle_min), float(msg.angle_increment),
                     max(self.min_range, float(msg.range_min)),
                     min(self.max_range, float(msg.range_max)))
        self.scan_t = time.monotonic()
        self.scan_seq += 1

    # ---------------- 扇区/点簇 ----------------
    def sector_of(self, angle):
        a = abs(angle)
        if a <= self.front_half:
            return 'front'
        if a >= math.pi - self.rear_half:
            return 'rear'
        return 'left' if angle > 0.0 else 'right'

    def scan_points(self):
        ranges, amin, inc, rmin, rmax = self.scan
        pts = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            a = wrap_angle(amin + i * inc + self.scan_yaw_offset)
            pts.append((a, r))
        pts.sort(key=lambda item: item[0])
        return pts

    def clusters(self, pts):
        if not pts:
            return []
        groups = []
        cur = [pts[0]]
        for point in pts[1:]:
            if point[0] - cur[-1][0] <= self.neighbor_gap:
                cur.append(point)
            else:
                groups.append(cur)
                cur = [point]
        groups.append(cur)
        # 合并 ±π 处被切断的点簇
        if len(groups) > 1:
            gap = groups[0][0][0] + 2.0 * math.pi - groups[-1][-1][0]
            if gap <= self.neighbor_gap:
                groups[0] = groups[-1] + groups[0]
                groups.pop()
        out = []
        for group in groups:
            if len(group) < self.min_cluster:
                continue
            base = group[0][0]
            unwrapped = []
            for a, _ in group:
                aa = a
                while aa - base > math.pi:
                    aa -= 2.0 * math.pi
                while aa - base < -math.pi:
                    aa += 2.0 * math.pi
                unwrapped.append(aa)
            center = wrap_angle(sum(unwrapped) / len(unwrapped))
            min_r = min(r for _, r in group)
            out.append((center, min_r, len(group)))
        return out

    def sector_minima(self, pts):
        mins = {s: INF for s in SECTORS}
        for a, r in pts:
            s = self.sector_of(a)
            if r < mins[s]:
                mins[s] = r
        return mins

    # ---------------- 判定 ----------------
    def evaluate(self):
        pts = self.scan_points()
        close_pts = [(a, r) for a, r in pts if r <= self.avoid_distance]
        found = {}
        for center, min_r, count in self.clusters(close_pts):
            sector = self.sector_of(center)
            if sector not in found or min_r < found[sector][1]:
                found[sector] = (center, min_r, count)
        self.sector_min = self.sector_minima(pts)

        for sector in SECTORS:
            if sector in found:
                center, min_r, _ = found[sector]
                prev_min = self.last_min[sector]
                prev_center = self.last_center[sector]
                approaching = prev_min is None or min_r <= prev_min + self.approach_tol
                same_cluster = (prev_center is None or
                                angle_distance(center, prev_center) <= self.center_tol)
                self.hits[sector] = self.hits[sector] + 1 if (approaching and same_cluster) else 0
                self.last_min[sector] = min_r
                self.last_center[sector] = center
                self.cluster_min[sector] = min_r
            else:
                self.hits[sector] = 0
                self.last_min[sector] = None
                self.last_center[sector] = None
                self.cluster_min[sector] = INF

            if sector not in found and self.sector_min[sector] > self.release_distance:
                self.clears[sector] += 1
            else:
                self.clears[sector] = 0

            was_blocked = self.blocked[sector]
            if self.hits[sector] >= self.confirm_scans:
                self.blocked[sector] = True
            if self.clears[sector] >= self.release_scans:
                self.blocked[sector] = False
            if self.blocked[sector] != was_blocked:
                self.get_logger().warn(
                    f'{sector} {"避障触发" if self.blocked[sector] else "避障解除"} '
                    f'min={self.sector_min[sector]:.2f} m')

    def apply(self, cmd):
        out = Twist()
        out.linear.x = cmd.linear.x
        out.angular.z = cmd.angular.z
        reason = ''
        if self.blocked['front'] and out.linear.x > 0.0:
            out.linear.x = 0.0
            reason = 'front'
        if self.blocked['rear'] and out.linear.x < 0.0:
            out.linear.x = 0.0
            reason = 'rear'
        if self.cluster_min['left'] < self.side_clearance and out.angular.z > 0.0:
            out.angular.z = 0.0
            reason = reason or 'left'
        if self.cluster_min['right'] < self.side_clearance and out.angular.z < 0.0:
            out.angular.z = 0.0
            reason = reason or 'right'
        self.last_blocked_components = {
            'linear_x': cmd.linear.x != out.linear.x,
            'angular_z': cmd.angular.z != out.angular.z,
        }
        return out, reason

    # ---------------- 周期输出 ----------------
    def tick(self):
        now = time.monotonic()
        cmd_fresh = self.cmd is not None and (now - self.cmd_t) <= self.input_timeout
        scan_fresh = self.scan is not None and (now - self.scan_t) <= self.scan_timeout

        if not scan_fresh:
            out, reason = Twist(), 'scan_stale'
            self.last_blocked_components = {'linear_x': True, 'angular_z': True}
        elif not cmd_fresh:
            out, reason = Twist(), 'cmd_timeout'
            self.last_blocked_components = {'linear_x': True, 'angular_z': True}
        else:
            if self.scan_seq != self.last_eval_seq:
                self.evaluate()
                self.last_eval_seq = self.scan_seq
            out, reason = self.apply(self.cmd)

        obstacle_block = reason in ('front', 'rear', 'left', 'right')
        self.pub_brake.publish(Bool(data=obstacle_block))
        self.pub.publish(out)
        if now - self.last_state_t >= 0.2:
            self.last_state_t = now
            state = {
                'blocked': [s for s in SECTORS if self.blocked[s]],
                'min': {s: (None if math.isinf(self.sector_min[s])
                            else round(self.sector_min[s], 3)) for s in SECTORS},
                'hits': dict(self.hits),
                'reason': reason,
                'blocked_components': dict(self.last_blocked_components),
                'stale': not scan_fresh,
            }
            self.pub_state.publish(String(data=json.dumps(state, separators=(',', ':'))))


def main():
    rclpy.init()
    node = SafetyGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
