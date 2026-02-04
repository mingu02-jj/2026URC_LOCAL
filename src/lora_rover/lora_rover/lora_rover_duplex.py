#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Single-process duplex runner for rover side.

Creates two ROS2 nodes in one process, sharing /dev/ttyUSBx:
  - lora_rover_rx (Base->Rover cmd_vel + estop)
  - lora_rover_tx (Rover->Base telemetry)

All parameters should be set on the process node name `lora_rover_duplex`
(via YAML/launch). They will be forwarded into the two internal nodes.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .serial_transport import SerialTransport
from .lora_rover_rx import LoRaRoverRx
from .lora_rover_tx import LoRaRoverTx


def main(args=None):
    rclpy.init(args=args)

    tmp = rclpy.create_node(
        'lora_rover_duplex',
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    # collect all parameters passed to this process
    # NOTE: 일부 rclpy 배포본에서는 Node.list_parameters()가 없을 수 있음.
    #       런치/YAML override 파라미터를 안전하게 수집하기 위해 fallback을 둔다.
    param_overrides = {}
    try:
        if hasattr(tmp, 'list_parameters'):
            names = tmp.list_parameters([], depth=10).names
            param_overrides = {n: tmp.get_parameter(n).value for n in names}
        else:
            _params = getattr(tmp, '_parameters', {})
            param_overrides = {name: p.value for name, p in _params.items()}
    except Exception:
        param_overrides = {}

    serial_port = str(param_overrides.get('serial_port', '/dev/ttyUSB0'))
    baudrate = int(param_overrides.get('baudrate', 115200))

    transport = SerialTransport(port=serial_port, baudrate=baudrate)

    rx = LoRaRoverRx(transport=transport, param_overrides=param_overrides)
    tx = LoRaRoverTx(transport=transport, param_overrides=param_overrides)

    # coordinator timer (runs on the process node `tmp`)
    tmp.create_timer(0.02, lambda: tx.update_link_state(rx.get_last_cmd_rx_time(), rx.is_estop_latched()))

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(tmp)
    executor.add_node(rx)
    executor.add_node(tx)

    tmp.get_logger().info(f"[lora_rover_duplex] using {serial_port} @ {baudrate}")

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    tx.destroy_node()
    rx.destroy_node()
    tmp.destroy_node()
    transport.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
