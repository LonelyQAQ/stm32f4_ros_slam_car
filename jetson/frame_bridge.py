#!/usr/bin/env python3
"""frame_bridge.py —— F407 自定义帧 <-> ROS 2 话题桥接

帧：0xAA 0x55 | type | len | payload | crc16(LE)
  TELEM(0x01) 60 B @50 Hz  F407 -> 本节点
  CMD  (0x02)  8 B @20 Hz  本节点 -> F407

发布：/odom(50Hz) /imu/data_raw(50Hz) /wheel_speed(20Hz)
订阅：/cmd_vel_safe(20Hz 下发，默认；可参数覆盖)

用法：
  python3 frame_bridge.py /dev/ttyTHS1 921600
"""
import math
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import Twist
import serial

H1, H2 = 0xAA, 0x55
T_TELEM, T_CMD = 0x01, 0x02
TELEM_FMT = ('<I' 'fffff' 'ii' 'fffffff' 'ff' 'ffff' 'fBB')   # 90 B：…/四元数/零偏诊断
CMD_FMT = '<ffB'                    # 9 B：v、w、flags(bit0=制动)
CMD_FLAG_BRAKE = 0x01
BRAKE_HOLD_S = 0.5                   # /cmd_brake 置 True 后保持多久（MCU 侧另有 500 ms 兜底）
TRACK_M = 0.180                      # 与 board_cfg.h 的 WHEEL_TRACK_M 保持一致


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_cmd(v: float, w: float, brake: bool = False) -> bytes:
    flags = CMD_FLAG_BRAKE if brake else 0
    payload = struct.pack(CMD_FMT, v, w, flags)
    body = bytes([T_CMD, len(payload)]) + payload
    return bytes([H1, H2]) + body + struct.pack('<H', crc16_ccitt(body))


class FrameBridge(Node):
    def __init__(self, port: str, baud: int):
        super().__init__('frame_bridge')
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.lock = threading.Lock()
        self.telem = None          # 最新一帧遥测
        self.telem_count = 0
        self.crc_err = 0
        self.len_err = 0
        self.target_v = 0.0
        self.brake = False
        self.brake_t = 0.0
        self.target_w = 0.0
        self.last_telem_t = 0.0
        self.running = True

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_odom = self.create_publisher(Odometry, '/odom', qos)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', qos)
        self.pub_wheel = self.create_publisher(Float32MultiArray, '/wheel_speed', qos)
        self.pub_cmd_echo = self.create_publisher(Twist, '/cmd_echo', qos)
        self.pub_enc = self.create_publisher(Int32MultiArray, '/enc_ticks', qos)
        self.pub_temp = self.create_publisher(Float32, '/imu/temp', qos)
        self.pub_diag = self.create_publisher(Float32MultiArray, '/imu/diag', qos)
        cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel_safe').value
        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_subscription(Bool, '/cmd_brake', self.on_brake, 10)
        self.get_logger().info(f'cmd_vel <- {cmd_topic}')

        self.create_timer(0.02, self.tick_pub)     # 50 Hz 发布
        self.create_timer(0.05, self.tick_cmd)     # 20 Hz 下发
        self.create_timer(2.0, self.tick_diag)     # 2 s 诊断

        self.th = threading.Thread(target=self.read_loop, daemon=True)
        self.th.start()
        self.get_logger().info(f'frame_bridge 打开 {port} @ {baud}')

    # ---- 串口读线程 ----
    def read_loop(self):
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(256)
            except Exception as e:      # 串口被拔/出错
                self.get_logger().error(f'serial: {e}')
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            self.parse(buf)

    def parse(self, buf: bytearray):
        while True:
            i = buf.find(bytes([H1, H2]))
            if i < 0:
                if len(buf) > 1:
                    del buf[:-1]
                return
            if i:
                del buf[:i]
            if len(buf) < 4:
                return
            ln = buf[3]
            total = 4 + ln + 2
            if len(buf) < total:
                return
            body = bytes(buf[2:4 + ln])
            crc = struct.unpack('<H', bytes(buf[4 + ln:total]))[0]
            if crc16_ccitt(body) != crc:
                self.crc_err += 1
                del buf[:2]
                continue
            if buf[2] == T_TELEM and ln != struct.calcsize(TELEM_FMT):
                self.len_err += 1
                if self.len_err <= 3:
                    self.get_logger().error(
                        f'TELEM 长度不符: 收到 {ln}, 期望 {struct.calcsize(TELEM_FMT)}')
            if buf[2] == T_TELEM and ln == struct.calcsize(TELEM_FMT):
                vals = struct.unpack(TELEM_FMT, bytes(buf[4:4 + ln]))
                with self.lock:
                    self.telem = vals
                    self.telem_count += 1
                    self.last_telem_t = time.time()
            del buf[:total]

    # ---- 发布 ----
    def tick_pub(self):
        with self.lock:
            t = self.telem
        if t is None:
            return
        (tick_ms, x, y, th, v, w, lt, rt, gx, gy, gz, ax, ay, az, temp,
         cmd_v, cmd_w, qw, qx, qy, qz, bias_z, rest, zupt) = t
        stamp = self.get_clock().now().to_msg()

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = float(x)
        od.pose.pose.position.y = float(y)
        od.pose.pose.orientation.z = math.sin(th * 0.5)
        od.pose.pose.orientation.w = math.cos(th * 0.5)
        od.twist.twist.linear.x = float(v)
        od.twist.twist.angular.z = float(w)
        od.pose.covariance[0] = 1e-3
        od.pose.covariance[7] = 1e-3
        od.pose.covariance[35] = 1e-2
        od.twist.covariance[0] = 1e-2
        od.twist.covariance[35] = 1e-2
        self.pub_odom.publish(od)

        # odom -> base_link 改由 robot_localization 的 ekf_node 发布（eKF 融合轮速 + IMU 偏航角速度）。
        # 这里再发一次会造成同一 TF 两个发布者，tf2 会持续告警并随机跳变。

        imu = Imu()
        imu.header.stamp = stamp
        # 固件已把 BMI088 的 90° 安装偏置补偿到车体系，所以这里直接标 base_link，
        # 不再需要 base_link -> imu_link 的静态变换。
        imu.header.frame_id = 'base_link'
        imu.orientation.w = float(qw)
        imu.orientation.x = float(qx)
        imu.orientation.y = float(qy)
        imu.orientation.z = float(qz)
        # roll/pitch 有重力约束，yaw 无磁力计只能靠陀螺积分 → 协方差给大，EKF 别信绝对航向
        imu.orientation_covariance[0] = 1e-2
        imu.orientation_covariance[4] = 1e-2
        imu.orientation_covariance[8] = 1.0
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        for i in (0, 4, 8):
            imu.angular_velocity_covariance[i] = 2e-4
            imu.linear_acceleration_covariance[i] = 4e-2
        self.pub_imu.publish(imu)

        tp = Float32()
        tp.data = float(temp)
        self.pub_temp.publish(tp)

        dg = Float32MultiArray()
        dg.data = [float(bias_z), float(rest), float(zupt)]
        self.pub_diag.publish(dg)

        # 左右轮线速度由 v/w 反推（差速模型），与 board_cfg.h 的轮距一致
        vl = float(v) - float(w) * TRACK_M * 0.5
        vr = float(v) + float(w) * TRACK_M * 0.5
        ws = Float32MultiArray()
        ws.data = [vl, vr]
        self.pub_wheel.publish(ws)

        # 回显 MCU 实际收到的指令，用于验证下行链路
        er = Int32MultiArray()
        er.data = [int(lt), int(rt)]
        self.pub_enc.publish(er)

        ce = Twist()
        ce.linear.x = float(cmd_v)
        ce.angular.z = float(cmd_w)
        self.pub_cmd_echo.publish(ce)

    def on_brake(self, msg: Bool):
        # 只接受 True：制动是"请求"，由这里保持 BRAKE_HOLD_S 后自动失效
        if msg.data:
            self.brake = True
            self.brake_t = time.time()

    def tick_cmd(self):
        brake_now = self.brake and (time.time() - self.brake_t) <= BRAKE_HOLD_S
        try:
            self.ser.write(build_cmd(self.target_v, self.target_w, brake_now))
        except Exception as e:
            self.get_logger().error(f'write: {e}')

    def tick_diag(self):
        with self.lock:
            n, crc, last = self.telem_count, self.crc_err, self.last_telem_t
        age = (time.time() - last) if last else 999.0
        self.get_logger().info(f'telem={n} crc_err={crc} len_err={self.len_err} age={age:.2f}s')
        if age > 1.0:
            self.get_logger().warn('超过 1 s 没收到遥测帧 —— 检查波特率/接线/固件是否在发')

    def on_cmd(self, msg: Twist):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyTHS1'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
    rclpy.init()
    node = FrameBridge(port, baud)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
