#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
urc_state_manager
- Generates URC state topics on Jetson side:
  - /urc/is_autonomous (Bool)
  - /urc/nav_arrived   (Bool) : LED-ready (hold/clear policy applied)
- Also publishes a packed byte for Portenta serial safety/state flags:
  - /urc/safety_state_bits (UInt8): bits excluding ALIVE toggle (b1)
    b0: estop_any
    b2: drive_enable (default True / optional)
    b3: cmd_valid (always 1 while node alive)
    b4: joy_active
    b5: lora_active
    b6: is_autonomous
    b7: nav_arrived
"""

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8


@dataclass
class Timeouts:
    joy: float
    lora: float
    nav: float


class UrcStateManager(Node):
    def __init__(self) -> None:
        super().__init__('urc_state_manager')

        # ---- Parameters (default values aligned with twist_mux.yaml) ----
        self.declare_parameter('joy_topic', '/cmd_vel_joy')
        self.declare_parameter('lora_topic', '/cmd_vel_lora')
        self.declare_parameter('nav_topic', '/cmd_vel')
        self.declare_parameter('nav_arrived_raw_topic', '/urc/nav_arrived_raw')

        self.declare_parameter('estop_topic', '/estop')
        self.declare_parameter('estop_lora_topic', '/estop_lora')

        self.declare_parameter('joy_timeout', 0.5)
        self.declare_parameter('lora_timeout', 1.0)
        self.declare_parameter('nav_timeout', 0.5)

        self.declare_parameter('arrived_hold_sec', 2.5)
        self.declare_parameter('publish_hz', 20.0)

        # optional safety gate (GUI/ARM)
        self.declare_parameter('drive_enable', True)

        joy_topic = self.get_parameter('joy_topic').value
        lora_topic = self.get_parameter('lora_topic').value
        nav_topic = self.get_parameter('nav_topic').value
        nav_arrived_raw_topic = self.get_parameter('nav_arrived_raw_topic').value
        estop_topic = self.get_parameter('estop_topic').value
        estop_lora_topic = self.get_parameter('estop_lora_topic').value

        self.timeouts = Timeouts(
            joy=float(self.get_parameter('joy_timeout').value),
            lora=float(self.get_parameter('lora_timeout').value),
            nav=float(self.get_parameter('nav_timeout').value),
        )
        self.arrived_hold_sec = float(self.get_parameter('arrived_hold_sec').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.drive_enable = bool(self.get_parameter('drive_enable').value)

        # ---- State holders ----
        self.last_joy: Optional[Time] = None
        self.last_lora: Optional[Time] = None
        self.last_nav: Optional[Time] = None

        self.estop_pc: bool = False
        self.estop_lora: bool = False

        self.nav_arrived_raw: bool = False
        self._nav_arrived_raw_prev: bool = False

        self.nav_arrived: bool = False
        self._arrived_on_time: Optional[Time] = None

        # ---- Subs ----
        self.create_subscription(Twist, joy_topic, self._cb_joy, 10)
        self.create_subscription(Twist, lora_topic, self._cb_lora, 10)
        self.create_subscription(Twist, nav_topic, self._cb_nav, 10)
        self.create_subscription(Bool, nav_arrived_raw_topic, self._cb_arrived_raw, 10)
        self.create_subscription(Bool, estop_topic, self._cb_estop_pc, 10)
        self.create_subscription(Bool, estop_lora_topic, self._cb_estop_lora, 10)

        # ---- Pubs ----
        self.pub_is_autonomous = self.create_publisher(Bool, '/urc/is_autonomous', 10)
        self.pub_nav_arrived = self.create_publisher(Bool, '/urc/nav_arrived', 10)

        # debug helpers (optional but useful)
        self.pub_joy_active = self.create_publisher(Bool, '/urc/is_joy_active', 10)
        self.pub_lora_active = self.create_publisher(Bool, '/urc/is_lora_active', 10)
        self.pub_estop_any = self.create_publisher(Bool, '/urc/estop_any', 10)

        self.pub_bits = self.create_publisher(UInt8, '/urc/safety_state_bits', 10)

        # ---- Timer ----
        period = 1.0 / max(1.0, publish_hz)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'URC state manager started. joy={joy_topic}, lora={lora_topic}, nav={nav_topic}, '
            f'arrived_raw={nav_arrived_raw_topic}, estop={estop_topic}, estop_lora={estop_lora_topic}'
        )

    def _now(self) -> Time:
        return self.get_clock().now()

    def _cb_joy(self, _: Twist) -> None:
        self.last_joy = self._now()

    def _cb_lora(self, _: Twist) -> None:
        self.last_lora = self._now()

    def _cb_nav(self, _: Twist) -> None:
        self.last_nav = self._now()

    def _cb_estop_pc(self, msg: Bool) -> None:
        self.estop_pc = bool(msg.data)

    def _cb_estop_lora(self, msg: Bool) -> None:
        self.estop_lora = bool(msg.data)

    def _cb_arrived_raw(self, msg: Bool) -> None:
        self.nav_arrived_raw = bool(msg.data)

    def _is_active(self, last: Optional[Time], timeout_s: float, now: Time) -> bool:
        if last is None:
            return False
        dt = (now - last).nanoseconds * 1e-9
        return dt < timeout_s

    def _on_timer(self) -> None:
        now = self._now()

        joy_active = self._is_active(self.last_joy, self.timeouts.joy, now)
        lora_active = self._is_active(self.last_lora, self.timeouts.lora, now)
        nav_active = self._is_active(self.last_nav, self.timeouts.nav, now)

        estop_any = bool(self.estop_pc or self.estop_lora)

        # ----- nav_arrived state machine -----
        # rising edge of raw -> latch on
        if (not self._nav_arrived_raw_prev) and self.nav_arrived_raw:
            self.nav_arrived = True
            self._arrived_on_time = now

        self._nav_arrived_raw_prev = self.nav_arrived_raw

        # clear conditions
        if estop_any:
            self.nav_arrived = False
        elif self.nav_arrived:
            # (1) operator intervention clears immediately
            if joy_active or lora_active:
                self.nav_arrived = False
            # (2) mission resets raw -> clear
            elif not self.nav_arrived_raw:
                self.nav_arrived = False
            # (3) hold timeout -> clear
            elif self._arrived_on_time is not None:
                held = (now - self._arrived_on_time).nanoseconds * 1e-9
                if held >= self.arrived_hold_sec:
                    self.nav_arrived = False

        # ----- is_autonomous -----
        is_autonomous = bool(nav_active and (not joy_active) and (not lora_active) and (not estop_any))

        # ----- Publish bools -----
        self.pub_is_autonomous.publish(Bool(data=is_autonomous))
        self.pub_nav_arrived.publish(Bool(data=self.nav_arrived))

        self.pub_joy_active.publish(Bool(data=joy_active))
        self.pub_lora_active.publish(Bool(data=lora_active))
        self.pub_estop_any.publish(Bool(data=estop_any))

        # ----- Pack bits (excluding alive toggle b1) -----
        bits = 0
        # b0 estop_any
        if estop_any:
            bits |= (1 << 0)
        # b2 drive_enable
        if self.drive_enable:
            bits |= (1 << 2)
        # b3 cmd_valid (node is alive)
        bits |= (1 << 3)
        # b4 joy_active, b5 lora_active, b6 is_autonomous, b7 nav_arrived
        if joy_active:
            bits |= (1 << 4)
        if lora_active:
            bits |= (1 << 5)
        if is_autonomous:
            bits |= (1 << 6)
        if self.nav_arrived:
            bits |= (1 << 7)

        self.pub_bits.publish(UInt8(data=bits))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UrcStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
