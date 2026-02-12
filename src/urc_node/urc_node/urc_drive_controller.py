#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import serial
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, UInt8


class SwerveDriveController(Node):
    """
    Jetson-side serial bridge
    - Subscribes: /cmd_vel_mux (Twist) by default
    - Subscribes: /urc/safety_state_bits (UInt8) produced by urc_state_manager
    - Sends fixed-rate serial packets to Portenta (12 bytes, compatible with existing PcComm)
      Safety byte layout (total 8 bits):
        b0: estop_any
        b1: alive toggle (this node toggles every packet)
        b2: drive_enable
        b3: cmd_valid
        b4: joy_active
        b5: lora_active
        b6: is_autonomous
        b7: nav_arrived
    """
    def __init__(self):
        super().__init__('urc_drive_controller')

        # ===== Parameters =====
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_joy')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_hz', 50.0)
        self.declare_parameter('cmd_timeout', 0.6)  # if cmd_vel becomes stale -> send zeros

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baudrate').value)

        self.send_hz = float(self.get_parameter('send_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # ===== ROS I/O =====
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 15)
        self.bits_sub = self.create_subscription(UInt8, '/urc/safety_state_bits', self.bits_callback, 15)

        # kept (legacy / optional)
        self.steering_pub = self.create_publisher(Float64MultiArray, '/urc_steering_controller/commands', 15)
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/urc_velocity_controller/commands', 15)

        # ===== Serial =====
        try:
            self.serial = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Serial connected to {port} @ {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            self.serial = None

        # ===== Internal state =====
        self.alive_bit = 0  # b1 toggle
        self.state_bits = 0  # all bits except b1 are overwritten from /urc/safety_state_bits

        self.last_cmd: Twist = Twist()
        self.last_cmd_time: Optional[Time] = None

        # ===== Timer =====
        period = 1.0 / max(1.0, self.send_hz)
        self.create_timer(period, self.on_timer)

        self.get_logger().info(f"Subscribed cmd_vel_topic: {cmd_vel_topic}")
        self.get_logger().info("Waiting for /urc/safety_state_bits from urc_state_manager...")

    def _now(self) -> Time:
        return self.get_clock().now()

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self._now()

    def bits_callback(self, msg: UInt8):
        # ensure b1 is not set by upstream (reserved for alive toggle)
        self.state_bits = int(msg.data) & 0xFD  # 0b11111101

    # float → int16 (스케일링 후 클램프)
    def _to_s16(self, x: float) -> int:
        if math.isnan(x) or math.isinf(x):
            x = 0.0
        i = int(round(x))
        if i > 32767:
            i = 32767
        if i < -32768:
            i = -32768
        return i

    def on_timer(self):
        if self.serial is None:
            return

        # Decide current command (stale -> zeros)
        now = self._now()
        vx = float(self.last_cmd.linear.x)
        vy = float(self.last_cmd.linear.y)
        omega = float(self.last_cmd.angular.z)

        if self.last_cmd_time is None:
            vx = 0.0
            vy = 0.0
            omega = 0.0
        else:
            age = (now - self.last_cmd_time).nanoseconds * 1e-9
            if age >= self.cmd_timeout:
                vx = 0.0
                vy = 0.0
                omega = 0.0

        self.send_serial(vx, vy, omega)

    def send_serial(self, vx: float, vy: float, omega: float):
        # sanity clamp
        vx = 0.0 if abs(vx) >= 100.0 else vx
        vy = 0.0 if abs(vy) >= 100.0 else vy
        omega = max(-1.0, min(1.0, omega))

        # scale: vx, vy ×100 / omega ×1000
        vx_i = self._to_s16(vx * 100.0)
        vy_i = self._to_s16(vy * 100.0)
        w_i  = self._to_s16(-(omega * 1000.0))

        # alive toggle (b1)
        self.alive_bit ^= 1

        # merge safety byte
        safety = (self.state_bits & 0xFD) | ((self.alive_bit & 1) << 1)

        packet = bytearray(12)
        packet[0:3] = b'STX'  # Portenta PcComm expects 0x53,0x54,0x58
        packet[3:5] = vx_i.to_bytes(2, 'big', signed=True)
        packet[5:7] = vy_i.to_bytes(2, 'big', signed=True)
        packet[7:9] = w_i.to_bytes(2, 'big', signed=True)
        packet[9] = safety
        packet[10] = 0x0D
        packet[11] = 0x0A

        try:
            self.serial.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    controller = SwerveDriveController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    if controller.serial:
        controller.serial.close()

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
