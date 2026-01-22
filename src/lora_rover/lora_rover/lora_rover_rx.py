#!/usr/bin/env python3
import time
import threading
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from .lora_protocol import FrameDecoder, CmdVelPacket, FLAG_ESTOP


class LoRaRoverRx(Node):
    """
    Rover(Jetson) RX:
      - Reads E22(USB-serial) frames
      - Publishes:
          /cmd_vel_lora (Twist)
          /estop_lora   (Bool)

    Updated Policy (실전 안정형):
      - /estop_lora는 "상태"이므로 주기적으로 publish(heartbeat) 한다.
        -> twist_mux lock이 마지막 메시지에 덜 끌려가고, rqt/echo에서도 항상 확인 가능.
      - ESTOP flag가 들어오면 estop_lora=True (+ cmd_vel=0)
      - ESTOP 아닌 패킷이 들어오면 estop_lora=False
      - 링크가 끊기면 stop(0) 1회만 보내고, estop은 False 유지(락 해제 방향)
    """

    def __init__(self):
        super().__init__('lora_rover_rx')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('cmd_vel_out', '/cmd_vel_lora')
        self.declare_parameter('estop_out', '/estop_lora')

        self.declare_parameter('vx_scale', 1000.0)
        self.declare_parameter('vy_scale', 1000.0)
        self.declare_parameter('wz_scale', 1000.0)

        self.declare_parameter('failsafe_stop_after_sec', 0.7)

        # estop 상태 heartbeat 주기 (Hz)
        self.declare_parameter('estop_heartbeat_hz', 5.0)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baudrate').value)

        self.vx_scale = float(self.get_parameter('vx_scale').value)
        self.vy_scale = float(self.get_parameter('vy_scale').value)
        self.wz_scale = float(self.get_parameter('wz_scale').value)

        self.failsafe_stop_after = float(self.get_parameter('failsafe_stop_after_sec').value)

        estop_hz = float(self.get_parameter('estop_heartbeat_hz').value)
        self._estop_period = 1.0 / max(0.1, estop_hz)

        self.pub_cmd = self.create_publisher(Twist, self.get_parameter('cmd_vel_out').value, 10)
        self.pub_estop = self.create_publisher(Bool, self.get_parameter('estop_out').value, 10)

        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.get_logger().info(f"[LoRaRoverRx] Serial open {port} @ {baud}")

        self.decoder = FrameDecoder(max_frame_len=64)

        self._last_rx_t = 0.0
        self._had_rx = False
        self._failsafe_sent = False

        # estop 상태(기본 False)
        self._estop_state = False
        self._last_estop_pub_t = 0.0

        # 시작하자마자 한번은 False 쏴서 lock이 기본 free가 되도록
        self.pub_estop.publish(Bool(data=False))
        self._last_estop_pub_t = time.time()

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        # watchdog: 링크 끊김 stop 1회
        self.timer_watchdog = self.create_timer(0.05, self._watchdog)
        # estop 상태 heartbeat
        self.timer_estop = self.create_timer(self._estop_period, self._publish_estop_heartbeat)

    def destroy_node(self):
        try:
            self._stop_event.set()
            self._thread.join(timeout=0.5)
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def _publish_cmd(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.pub_cmd.publish(msg)

    def _set_estop(self, val: bool):
        # 상태 갱신
        self._estop_state = bool(val)

        # 즉시 1회 publish (상태변화 즉시 반영)
        self.pub_estop.publish(Bool(data=self._estop_state))
        self._last_estop_pub_t = time.time()

    def _publish_estop_heartbeat(self):
        # 주기적으로 현재 상태 publish (echo/rqt/lock 안정화)
        self.pub_estop.publish(Bool(data=self._estop_state))
        self._last_estop_pub_t = time.time()

    def _read_loop(self):
        while (not self._stop_event.is_set()) and rclpy.ok():
            data = self.ser.read(256)
            if not data:
                continue

            for decoded in self.decoder.push(data):
                try:
                    pkt = CmdVelPacket.from_bytes(decoded)
                except Exception:
                    continue

                vx = pkt.vx_i / self.vx_scale
                vy = pkt.vy_i / self.vy_scale
                wz = pkt.wz_i / self.wz_scale

                estop = bool(pkt.flags & FLAG_ESTOP)

                if estop:
                    # 들어오면 True + 즉시 정지
                    self._set_estop(True)
                    self._publish_cmd(0.0, 0.0, 0.0)
                else:
                    # 평소에는 False + 속도 반영
                    self._set_estop(False)
                    self._publish_cmd(vx, vy, wz)

                self._last_rx_t = time.time()
                self._had_rx = True
                self._failsafe_sent = False

    def _watchdog(self):
        if not self._had_rx or self._failsafe_sent:
            return

        if (time.time() - self._last_rx_t) > self.failsafe_stop_after:
            # 링크 끊김: stop 1회만
            self._publish_cmd(0.0, 0.0, 0.0)

            # 링크 끊김 자체를 estop으로 잠그지 않음
            # 필요하면 True로 바꾸는 것도 가능, 요청대이 들어올 시 true만 유지.
            self._set_estop(False)

            self._failsafe_sent = True


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

