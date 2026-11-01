#!/usr/bin/env python3
"""手动/自动速度仲裁

订阅：
    /cmd_vel_manual  geometry_msgs/Twist   网页 WASD（手动）
    /cmd_vel         geometry_msgs/Twist   Nav2 最终速度（自动）
    /nav_mode        std_msgs/String       manual | auto
发布：
    /cmd_vel_raw     geometry_msgs/Twist   交给 safety_guard

规则：同一时刻只转发当前模式的速度；来源超时或刚切模式时输出 0。
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')
        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('nav_topic', '/cmd_vel')
        self.declare_parameter('mode_topic', '/nav_mode')
        self.declare_parameter('out_topic', '/cmd_vel_raw')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('input_timeout_ms', 400)
        self.declare_parameter('switch_zero_ms', 200)

        p = self.get_parameter
        manual_topic = p('manual_topic').value
        nav_topic = p('nav_topic').value
        mode_topic = p('mode_topic').value
        out_topic = p('out_topic').value
        self.rate_hz = float(p('rate_hz').value)
        self.timeout_s = float(p('input_timeout_ms').value) / 1000.0
        self.switch_zero_s = float(p('switch_zero_ms').value) / 1000.0

        self.mode = 'manual'
        self.manual_msg = None
        self.manual_t = 0.0
        self.nav_msg = None
        self.nav_t = 0.0
        self.zero_until = 0.0

        self.create_subscription(Twist, manual_topic, self.on_manual, 10)
        self.create_subscription(Twist, nav_topic, self.on_nav, 10)
        self.create_subscription(String, mode_topic, self.on_mode, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f'cmd_mux: manual={manual_topic} nav={nav_topic} out={out_topic} '
            f'{self.rate_hz:.0f} Hz')

    def on_manual(self, msg):
        self.manual_msg = msg
        self.manual_t = time.monotonic()

    def on_nav(self, msg):
        self.nav_msg = msg
        self.nav_t = time.monotonic()

    def on_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode not in ('manual', 'auto'):
            self.get_logger().warn(f'忽略未知模式: {msg.data!r}')
            return
        if mode != self.mode:
            self.mode = mode
            self.zero_until = time.monotonic() + self.switch_zero_s
            self.manual_msg = None
            self.nav_msg = None
            self.get_logger().info(f'模式切换 -> {mode}')

    def publish_zero(self):
        self.pub.publish(Twist())

    def tick(self):
        now = time.monotonic()
        if now < self.zero_until:
            self.publish_zero()
            return
        if self.mode == 'manual':
            msg, stamp = self.manual_msg, self.manual_t
        else:
            msg, stamp = self.nav_msg, self.nav_t
        if msg is None or (now - stamp) > self.timeout_s:
            self.publish_zero()
            return
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
