# lora_rover (v0.2.0)
Rover(Jetson)에서 E22(USB-serial)로 수신한 cmd를 /cmd_vel_lora로 publish.

- 입력: E22 UART (COBS + CRC 프레임)
- 출력: /cmd_vel_lora (Twist), /estop_lora (Bool)

정책:
- 패킷을 받았을 때만 publish
- 링크 끊김 감지(failsafe_stop_after_sec) 시 STOP 1회 publish 후 침묵 (twist_mux timeout 전환 유도)

## Build
sudo apt-get install -y python3-serial
colcon build --packages-select lora_rover

## Run
ros2 launch lora_rover rover_lora.launch.py
