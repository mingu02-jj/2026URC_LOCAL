# lora_rover (v0.4.1)
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

## FRONT_HAZARD 토픽/해석 기준 (v0.4.1)
Rover가 ZED depth로 전방 위험도를 계산해 Base로 텔레메트리를 보냅니다.

### 입력/전처리
- 입력 토픽: `/zed/zed_node/depth/depth_registered` (encoding: 32FC1 또는 16UC1)
- ROI (세로): `roi_y_min~roi_y_max` (기본 0.55~0.95) — 화면 아래(지면/정면 경로) 위주
- ROI (가로): `roi_x_min~roi_x_max` (기본 0.0~1.0) — 좌/우 가장자리(depth 스파이크/로버 몸체) 제외용 (추천 0.10~0.90)
- 유효 거리: `min_valid_m~max_valid_m` (기본 0.4~8.0m)
- 섹터 분할: `sector_n=12` (가로 12등분)
- 섹터 값: 각 섹터에서 유효 depth들의 `percentile=5` (min 대신 노이즈 완화)
- 시간 필터: `median_frames=3` (최근 3프레임 median)

### 출력 값(LoRa → Base)
- `/lora/hazard/sector_min_mm` : u16[12], 각 섹터 최소거리(mm)
  - 섹터 0=좌측 ... 5~6=정면(중앙) ... 11=우측
- `/lora/hazard/forward_min_mm` : u16, **정면 섹터(기본 [5,6]) 기준 최소거리(mm)**
  - `forward_sector_indices`로 변경 가능
- `/lora/hazard/score` : u8(0~100), **forward_min 기반 위험 점수**
  - `score_stop_m` 이하: 100점 (즉시 위험)
  - `score_warn_m` 이상: 0점 (여유)
  - 그 사이는 `score_gamma`로 비선형 맵핑 (기본 0.5 → 3~4m에서도 점수가 의미있게 상승)
- `/lora/hazard/age_ms` : u16, 사용된 depth 프레임의 staleness(ms)
  - 0~120ms: 정상, 200ms+: 지연/드랍 가능성

### 조종/판단 예시(권장)
- `age_ms > 300ms`면 hazard 신뢰도 낮음(센서 상태 경고)
- `forward_min_mm <= 800` 또는 `score >= 90`이면 정지/회피 권장
- 회피 방향은 `sector_min_mm` 배열에서 좌/우 어느 쪽이 더 큰지(더 뚫렸는지)로 판단

