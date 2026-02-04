#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import queue
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from .serial_transport import SerialTransport
from .lora_protocol import CmdVelPacket, packet_type, PKT_CMD_VEL, FLAG_ESTOP


class LoRaRoverRx(Node):
    """Rover <- Base RX (cmd_vel + E-STOP).

    Publishes
      - /cmd_vel_lora (Twist)
      - /estop_lora (Bool)

    Policy
      - On CMD_VEL with FLAG_ESTOP=1: estop_latched=True, publish STOP
      - On CMD_VEL with FLAG_ESTOP=0: estop_latched=False, publish decoded cmd
      - Link timeout (failsafe_stop_after_sec): publish STOP, keep estop_latched as last known
    """

    def __init__(self, transport: Optional[SerialTransport] = None, param_overrides: Optional[dict] = None):
        super().__init__('lora_rover_rx')

        # serial
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        # topics
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel_lora')
        self.declare_parameter('estop_out_topic', '/estop_lora')

        # safety
        self.declare_parameter('failsafe_stop_after_sec', 0.5)
        self.declare_parameter('poll_hz', 200.0)
        self.declare_parameter('max_packets_per_poll', 50)

        # scaling (must match base)
        self.declare_parameter('vx_scale', 1000.0)
        self.declare_parameter('vy_scale', 1000.0)
        self.declare_parameter('wz_scale', 1000.0)

        if param_overrides:
            self._apply_param_overrides(param_overrides)

        self._failsafe_sec = float(self.get_parameter('failsafe_stop_after_sec').value)
        poll_hz = float(self.get_parameter('poll_hz').value)
        self._max_per = int(self.get_parameter('max_packets_per_poll').value)

        self._vx_scale = float(self.get_parameter('vx_scale').value)
        self._vy_scale = float(self.get_parameter('vy_scale').value)
        self._wz_scale = float(self.get_parameter('wz_scale').value)

        # transport
        self._own_transport = False
        if transport is None:
            port = str(self.get_parameter('serial_port').value)
            baud = int(self.get_parameter('baudrate').value)
            transport = SerialTransport(port=port, baudrate=baud)
            self._own_transport = True
            self.get_logger().info(f"[LoRaRoverRx] Serial open {port} @ {baud}")
        self.transport = transport
        self._q: "queue.Queue[bytes]" = self.transport.recv_queue()

        # pubs
        self.pub_cmd = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_out_topic').value), 10)
        self.pub_estop = self.create_publisher(Bool, str(self.get_parameter('estop_out_topic').value), 10)

        # state
        self._estop_latched = False
        self._last_cmd_rx_t = 0.0
        self._last_stop_pub_t = 0.0

        self.timer = self.create_timer(1.0 / max(10.0, poll_hz), self._poll)

    def destroy_node(self):
        try:
            if self._own_transport:
                self.transport.close()
        except Exception:
            pass
        super().destroy_node()

    def _apply_param_overrides(self, param_overrides: dict):
        # duplex runner가 모아준 override dict에는 rx가 선언하지 않은 파라미터가 섞일 수 있다.
        # undeclared 파라미터가 1개라도 섞이면 set_parameters() 전체가 실패할 수 있으므로
        # "개별 적용"으로 안전하게 처리한다.
        for k, v in (param_overrides or {}).items():
            try:
                self.set_parameters([Parameter(name=str(k), value=v)])
            except Exception:
                continue

    def get_last_cmd_rx_time(self) -> float:
        return self._last_cmd_rx_t

    def is_estop_latched(self) -> bool:
        return self._estop_latched

    def _publish_stop(self):
        cmd = Twist()
        self.pub_cmd.publish(cmd)
        self.pub_estop.publish(Bool(data=bool(self._estop_latched)))

    def _poll(self):
        processed = 0
        now = time.time()

        # 1) drain packets
        while processed < self._max_per:
            try:
                pkt = self._q.get_nowait()
            except queue.Empty:
                break

            processed += 1
            try:
                if packet_type(pkt) != PKT_CMD_VEL:
                    continue
                cmdp = CmdVelPacket.from_bytes_with_crc(pkt)
            except Exception:
                continue

            self._last_cmd_rx_t = now

            # update estop latch based on latest packet flag
            self._estop_latched = bool(cmdp.flags & FLAG_ESTOP)

            if self._estop_latched:
                self._publish_stop()
                continue

            cmd = Twist()
            cmd.linear.x = float(cmdp.vx_i16) / self._vx_scale
            cmd.linear.y = float(cmdp.vy_i16) / self._vy_scale
            cmd.angular.z = float(cmdp.wz_i16) / self._wz_scale
            self.pub_cmd.publish(cmd)
            self.pub_estop.publish(Bool(data=False))

        # 2) failsafe stop on link timeout
        if self._failsafe_sec > 0.0:
            if (now - self._last_cmd_rx_t) > self._failsafe_sec:
                # publish STOP at <=10Hz to avoid spamming
                if (now - self._last_stop_pub_t) > 0.1:
                    self._last_stop_pub_t = now
                    self._publish_stop()


def main():
    rclpy.init()
    node = LoRaRoverRx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
