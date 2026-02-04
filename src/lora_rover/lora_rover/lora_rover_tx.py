#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import NavSatFix, Imu, Image

from .serial_transport import SerialTransport
from .lora_protocol import (
    HbStatusPacket,
    PoseStatusPacket,
    FrontHazardPacket,
    frame_encode,
    RTK_UNKNOWN,
    RTK_NO_FIX,
    RTK_2D,
    RTK_3D,
    RTK_FLOAT,
    RTK_FIXED,
)

# ublox NAV-PVT (optional)
# 프로젝트(ublox_dgnss) 구성에 따라 메시지 이름이 다를 수 있어 2가지를 모두 시도합니다.
try:
    from ublox_ubx_msgs.msg import UBXNavPVT as _NavPVTMsg
    _HAVE_NAVPVT = True
except Exception:
    try:
        from ublox_ubx_msgs.msg import _NavPVTMsg as _NavPVTMsg
        _HAVE_NAVPVT = True
    except Exception:
        _NavPVTMsg = None
        _HAVE_NAVPVT = False


try:
    import numpy as np
    _HAVE_NUMPY = True
except Exception:
    np = None
    _HAVE_NUMPY = False



def _get_any(msg, names, default=0):
    for n in names:
        if hasattr(msg, n):
            try:
                return getattr(msg, n)
            except Exception:
                pass
    return default


def _to_int(x, default=0):
    if x is None:
        return default
    if isinstance(x, (bool, int)):
        return int(x)
    if isinstance(x, float):
        return int(x)
    # std_msgs/XXX often has .data
    for attr in ("data", "value"):
        if hasattr(x, attr):
            try:
                return int(getattr(x, attr))
            except Exception:
                pass
    try:
        return int(x)
    except Exception:
        return default


def _wrap_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z) from quaternion (assuming ENU)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _now_ms() -> int:
    return int(round(time.time() * 1000.0))


class LoRaRoverTx(Node):
    """Rover -> Base telemetry TX.

    Sends (per prompt)
      - 0x10 HB_STATUS     (1 Hz)
      - 0x11 POSE_STATUS   (1~2 Hz, default 1 Hz)
      - 0x12 FRONT_HAZARD  (2 Hz)

    Half-duplex collision avoidance
      - If Base->Rover CMD_VEL packets are being received frequently, we try to transmit
        shortly AFTER those receptions.
      - If Base is silent, we transmit on our own timers.

    NOTE
      - numpy is strongly recommended for hazard percentile/median filtering.
    """

    def __init__(self, transport: Optional[SerialTransport] = None, param_overrides: Optional[dict] = None):
        super().__init__('lora_rover_tx')

        # serial
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        # input topics
        self.declare_parameter('fix_topic', '/mb00b/fix')
        self.declare_parameter('heading_topic', '/imu/gnss_heading')
        self.declare_parameter('navpvt_topic', '/mb00b/ubx_nav_pvt')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')

        # heading
        self.declare_parameter('yaw_offset_rad', 0.0206175)  # must match localization navsat yaw_offset

        # rates
        self.declare_parameter('hb_hz', 1.0)
        self.declare_parameter('pose_hz', 1.0)
        self.declare_parameter('hazard_hz', 2.0)

        # cmd-sync behavior (collision reduction)
        self.declare_parameter('cmd_active_window_sec', 0.25)
        self.declare_parameter('cmd_edge_poll_hz', 50.0)

        # hazard config
        self.declare_parameter('sector_n', 12)
        self.declare_parameter('roi_y_min', 0.55)
        self.declare_parameter('roi_y_max', 0.95)
        # ROI X crop (0.0~1.0). Use to ignore left/right edges (rover body / depth artifacts).
        # Default keeps full width.
        self.declare_parameter('roi_x_min', 0.0)
        self.declare_parameter('roi_x_max', 1.0)
        # Heuristic: if a sector patch has mostly invalid/zero depth (often happens when object is closer than camera min depth),
        # treat it as "too close" instead of "far".
        # Set to 0.0 to disable.
        self.declare_parameter('too_close_zero_frac_threshold', 0.6)
        self.declare_parameter('min_valid_m', 0.4)
        self.declare_parameter('max_valid_m', 8.0)
        self.declare_parameter('percentile', 5.0)
        self.declare_parameter('median_frames', 3)
        # FRONT_HAZARD (forward cone + score mapping)
        # forward_sector_indices: 전방(정면)으로 간주할 섹터 인덱스 목록 (0..sector_n-1)
        # 예) sector_n=12일 때 중앙 2개 = [5, 6]
        self.declare_parameter('forward_sector_indices', [5, 6])

        # score mapping (전방 기준)
        # - score_stop_m 이하: 100점(즉시 위험)
        # - score_warn_m 이상: 0점(여유)
        # - 그 사이는 (r ** score_gamma)로 비선형 맵핑
        self.declare_parameter('score_stop_m', 0.8)
        self.declare_parameter('score_warn_m', 4.0)
        self.declare_parameter('score_gamma', 0.5)


        if param_overrides:
            self._apply_param_overrides(param_overrides)

        self._yaw_offset = float(self.get_parameter('yaw_offset_rad').value)

        self._hb_period = 1.0 / max(0.1, float(self.get_parameter('hb_hz').value))
        self._pose_period = 1.0 / max(0.1, float(self.get_parameter('pose_hz').value))
        self._haz_period = 1.0 / max(0.1, float(self.get_parameter('hazard_hz').value))

        self._cmd_active_window = float(self.get_parameter('cmd_active_window_sec').value)
        self._cmd_edge_poll_hz = float(self.get_parameter('cmd_edge_poll_hz').value)

        # hazard params
        self._sector_n = int(self.get_parameter('sector_n').value)
        self._roi_y_min = float(self.get_parameter('roi_y_min').value)
        self._roi_y_max = float(self.get_parameter('roi_y_max').value)
        self._roi_x_min = float(self.get_parameter('roi_x_min').value)
        self._roi_x_max = float(self.get_parameter('roi_x_max').value)
        self._too_close_zero_frac_th = float(self.get_parameter('too_close_zero_frac_threshold').value)
        self._min_valid_m = float(self.get_parameter('min_valid_m').value)
        self._max_valid_m = float(self.get_parameter('max_valid_m').value)
        self._pct = float(self.get_parameter('percentile').value)
        self._median_n = int(self.get_parameter('median_frames').value)

        # sanitize ROI fractions
        self._roi_y_min = max(0.0, min(1.0, self._roi_y_min))
        self._roi_y_max = max(0.0, min(1.0, self._roi_y_max))
        if self._roi_y_max <= self._roi_y_min:
            self._roi_y_min, self._roi_y_max = 0.55, 0.95

        self._roi_x_min = max(0.0, min(1.0, self._roi_x_min))
        self._roi_x_max = max(0.0, min(1.0, self._roi_x_max))
        if self._roi_x_max <= self._roi_x_min:
            self._roi_x_min, self._roi_x_max = 0.0, 1.0

        self._too_close_zero_frac_th = max(0.0, min(1.0, self._too_close_zero_frac_th))

        # forward cone + score mapping
        try:
            fwd_idxs = list(self.get_parameter('forward_sector_indices').value)
        except Exception:
            fwd_idxs = []
        self._forward_sector_indices = [int(x) for x in fwd_idxs] if fwd_idxs else []

        self._score_stop_m = float(self.get_parameter('score_stop_m').value)
        self._score_warn_m = float(self.get_parameter('score_warn_m').value)
        self._score_gamma = float(self.get_parameter('score_gamma').value)

        # sanitize
        self._forward_sector_indices = [i for i in self._forward_sector_indices if 0 <= i < self._sector_n]
        if not self._forward_sector_indices:
            mid = self._sector_n // 2
            self._forward_sector_indices = sorted(set([max(0, mid - 1), min(self._sector_n - 1, mid)]))

        if self._score_gamma <= 0.0:
            self._score_gamma = 1.0
        if self._score_warn_m <= self._score_stop_m:
            self._score_warn_m = self._score_stop_m + 0.1

        if not _HAVE_NUMPY:
            self.get_logger().warn("numpy is not available; FRONT_HAZARD will be disabled")

        # transport
        self._own_transport = False
        if transport is None:
            port = str(self.get_parameter('serial_port').value)
            baud = int(self.get_parameter('baudrate').value)
            transport = SerialTransport(port=port, baudrate=baud)
            self._own_transport = True
            self.get_logger().info(f"[LoRaRoverTx] Serial open {port} @ {baud}")
        self.transport = transport

        # state buffers
        self._t0 = time.time()

        self._fix: Optional[NavSatFix] = None
        self._fix_t: float = 0.0

        self._yaw_rad: float = 0.0
        self._yaw_t: float = 0.0

        self._rtk_state: int = RTK_UNKNOWN
        self._num_sv: int = 0
        self._h_acc_mm: int = 0
        self._navpvt_t: float = 0.0

        self._last_cmd_rx_t: float = 0.0  # injected/updated by duplex via shared dict, or by standalone param
        self._last_seen_cmd_rx_t: float = 0.0
        self._estop_latched: bool = False

        # hazard buffers
        self._depth_msg: Optional[Image] = None
        self._haz_hist = []  # list of np.ndarray shape (sector_n,), dtype=uint16
        self._haz_last: Optional[FrontHazardPacket] = None
        self._haz_stamp_ms: int = 0

        # subscribers
        self.create_subscription(NavSatFix, str(self.get_parameter('fix_topic').value), self._on_fix, 10)
        self.create_subscription(Imu, str(self.get_parameter('heading_topic').value), self._on_heading, 10)
        if _HAVE_NAVPVT:
            self.create_subscription(_NavPVTMsg, str(self.get_parameter('navpvt_topic').value), self._on_navpvt, 10)
        else:
            self.get_logger().warn("ublox_ubx_msgs/UBXNavPVT not available; RTK 상태는 unknown으로 송신됩니다")

        if _HAVE_NUMPY:
            self.create_subscription(Image, str(self.get_parameter('depth_topic').value), self._on_depth, 10)

        # timers (standalone send)
        self._last_hb_tx = 0.0
        self._last_pose_tx = 0.0
        self._last_haz_tx = 0.0

        self.timer_hazard = self.create_timer(self._haz_period, self._update_hazard)
        self.timer_send = self.create_timer(0.02, self._send_if_base_silent)  # 50 Hz check

        # cmd-edge poll (when base active)
        self.timer_cmdsync = self.create_timer(1.0 / max(5.0, self._cmd_edge_poll_hz), self._on_cmd_edge)

    def destroy_node(self):
        try:
            if self._own_transport:
                self.transport.close()
        except Exception:
            pass
        super().destroy_node()

    def _apply_param_overrides(self, param_overrides: dict):
        # duplex runner가 모아준 override dict에는 tx가 선언하지 않은 파라미터가 섞일 수 있다.
        # undeclared 파라미터가 1개라도 섞이면 set_parameters() 전체가 실패할 수 있으므로
        # "개별 적용"으로 안전하게 처리한다.
        for k, v in (param_overrides or {}).items():
            try:
                self.set_parameters([Parameter(name=str(k), value=v)])
            except Exception:
                continue

    # ----- injected by duplex runner -----
    def update_link_state(self, last_cmd_rx_t: float, estop_latched: bool):
        self._last_cmd_rx_t = float(last_cmd_rx_t)
        self._estop_latched = bool(estop_latched)

    # ----- subscriptions -----
    def _on_fix(self, msg: NavSatFix):
        self._fix = msg
        self._fix_t = time.time()

    def _on_heading(self, msg: Imu):
        q = msg.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self._yaw_rad = yaw
        self._yaw_t = time.time()

    def _on_navpvt(self, msg):
        # UBX NAV-PVT: carrSoln은 구현/메시지 정의에 따라
        # (1) flags2 bits6..7 또는 (2) carr_soln 필드로 들어올 수 있습니다.
        self._num_sv = _to_int(_get_any(msg, ['num_sv', 'numSV', 'numsv'], 0))
        # 일부 구현은 h_acc(mm)로 내보내기도 해서 후보를 넓게 둠
        self._h_acc_mm = _to_int(_get_any(msg, ['h_acc_mm', 'h_acc', 'hAcc', 'hacc'], 0))
        self._navpvt_t = time.time()

        fix_type = _to_int(_get_any(msg, ['fix_type', 'fixType', 'fixtype'], 0))

        carr_soln = None
        if hasattr(msg, 'carr_soln'):
            carr_soln = _to_int(getattr(msg, 'carr_soln'), -1)
        elif hasattr(msg, 'carrSoln'):
            carr_soln = _to_int(getattr(msg, 'carrSoln'), -1)
        else:
            flags2 = _to_int(_get_any(msg, ['flags2', 'flags_2'], 0))
            carr_soln = (flags2 >> 6) & 0x03

        if carr_soln == 2:
            self._rtk_state = RTK_FIXED
        elif carr_soln == 1:
            self._rtk_state = RTK_FLOAT
        else:
            # degrade to GPS fix
            if fix_type == 0:
                self._rtk_state = RTK_NO_FIX
            elif fix_type == 2:
                self._rtk_state = RTK_2D
            elif fix_type >= 3:
                self._rtk_state = RTK_3D
            else:
                self._rtk_state = RTK_UNKNOWN

    def _on_depth(self, msg: Image):
        # buffer only; computation is on timer
        self._depth_msg = msg

    # ----- hazard -----
    def _update_hazard(self):
        if not _HAVE_NUMPY:
            return
        if self._depth_msg is None:
            return

        msg = self._depth_msg
        h = int(msg.height)
        w = int(msg.width)
        if h <= 0 or w <= 0:
            return

        enc = str(msg.encoding).lower()
        if '32fc1' in enc:
            elem = 4
            dtype = np.float32
            to_m = 1.0
        elif '16uc1' in enc:
            elem = 2
            dtype = np.uint16
            to_m = 0.001
        else:
            # unsupported encoding
            return

        row_elems = int(msg.step) // elem
        if row_elems <= 0:
            return

        buf = np.frombuffer(msg.data, dtype=dtype)
        if buf.size < h * row_elems:
            return
        img = buf[: h * row_elems].reshape((h, row_elems))[:, :w]

        y0 = max(0, min(h - 1, int(h * self._roi_y_min)))
        y1 = max(y0 + 1, min(h, int(h * self._roi_y_max)))
        x_roi0 = max(0, min(w - 1, int(w * self._roi_x_min)))
        x_roi1 = max(x_roi0 + 1, min(w, int(w * self._roi_x_max)))
        roi = img[y0:y1, x_roi0:x_roi1]
        rw = int(roi.shape[1])

        sector_mm = np.zeros((self._sector_n,), dtype=np.uint16)
        for i in range(self._sector_n):
            x0 = int(rw * i / self._sector_n)
            x1 = int(rw * (i + 1) / self._sector_n)
            if x1 <= x0:
                x1 = min(rw, x0 + 1)
            patch = roi[:, x0:x1]
            d = patch.astype(np.float32) * to_m

            # valid range / too-close handling
            finite = np.isfinite(d)
            in_range = finite & (d >= self._min_valid_m) & (d <= self._max_valid_m)
            if np.any(in_range):
                val = float(np.percentile(d[in_range], self._pct))
            else:
                # If we see finite positive values below min_valid, treat as "too close" (camera can't measure reliably)
                too_close = finite & (d > 0.0) & (d < self._min_valid_m)
                if np.any(too_close):
                    val = float(self._min_valid_m)
                else:
                    # Heuristic: if the patch is dominated by invalid/zero depth, it is often because the scene is closer
                    # than camera min depth. In that case, be conservative (too close) rather than "far".
                    if getattr(self, '_too_close_zero_frac_th', 0.0) > 0.0:
                        invalid = (~finite) | (d <= 0.0)
                        invalid_frac = float(np.mean(invalid))
                        if invalid_frac >= float(self._too_close_zero_frac_th):
                            val = float(self._min_valid_m)
                        else:
                            val = float(self._max_valid_m)
                    else:
                        val = float(self._max_valid_m)

            mm = int(round(val * 1000.0))
            if mm < 0:
                mm = 0
            if mm > 65535:
                mm = 65535
            sector_mm[i] = mm

        # history median filter
        self._haz_hist.append(sector_mm)
        if len(self._haz_hist) > max(1, self._median_n):
            self._haz_hist.pop(0)

        med = np.median(np.stack(self._haz_hist, axis=0), axis=0).astype(np.uint16)
        
        # forward_min_mm: 전방(정면) 섹터 기준 최소거리
        if med.size:
            idxs = [i for i in self._forward_sector_indices if 0 <= i < med.size]
            if idxs:
                fwd_min_mm = int(np.min(med[idxs]))
            else:
                # fallback: 설정이 비정상일 때는 전체 최소로 대체
                fwd_min_mm = int(med.min())
        else:
            fwd_min_mm = 65535
        
        # hazard_score: forward_min 기준 (warn/stop/gamma)
        d_m = float(fwd_min_mm) / 1000.0
        warn = float(self._score_warn_m)
        stop = float(self._score_stop_m)
        gamma = float(self._score_gamma) if self._score_gamma > 0.0 else 1.0
        if warn <= stop:
            warn = stop + 0.1
        
        if d_m >= warn:
            score = 0
        elif d_m <= stop:
            score = 100
        else:
            r = (warn - d_m) / (warn - stop)  # 0..1
            if r < 0.0:
                r = 0.0
            elif r > 1.0:
                r = 1.0
            score = int(round((r ** gamma) * 100.0))
        score = max(0, min(100, int(score)))
        
        # age_ms based on depth stamp
        stamp_ms = int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1e6)
        now_ms = _now_ms()
        age_ms = now_ms - stamp_ms
        age_ms = max(0, min(65535, int(age_ms)))

        # cache
        self._haz_stamp_ms = stamp_ms
        self._haz_last = FrontHazardPacket(seq=0, forward_min_mm=fwd_min_mm, hazard_score=score,
                                           sector_min_mm=[int(x) for x in med.tolist()], age_ms=age_ms)

    # ----- scheduling / sending -----
    def _base_active(self) -> bool:
        if self._last_cmd_rx_t <= 0.0:
            return False
        return (time.time() - self._last_cmd_rx_t) < self._cmd_active_window

    def _send_packet(self, pkt_obj):
        # pkt_obj must have to_bytes_with_crc() and seq field
        raw = pkt_obj.to_bytes_with_crc()
        frame = frame_encode(raw)
        self.transport.write(frame)

    def _make_hb(self, seq: int) -> HbStatusPacket:
        uptime_ms = int(max(0.0, (time.time() - self._t0) * 1000.0)) & 0xFFFFFFFF
        last_cmd_age_ms = 65535
        if self._last_cmd_rx_t > 0.0:
            last_cmd_age_ms = int(min(65535, max(0.0, (time.time() - self._last_cmd_rx_t) * 1000.0)))
        return HbStatusPacket(seq=seq, uptime_ms=uptime_ms, estop_latched=int(self._estop_latched),
                              rtk_state=int(self._rtk_state), last_cmd_age_ms=last_cmd_age_ms)

    def _make_pose(self, seq: int) -> Optional[PoseStatusPacket]:
        if self._fix is None:
            return None

        lat_e7 = int(round(float(self._fix.latitude) * 1e7))
        lon_e7 = int(round(float(self._fix.longitude) * 1e7))

        yaw_send = _wrap_pi(self._yaw_rad + self._yaw_offset)
        heading_deg = yaw_send * 180.0 / math.pi
        heading_cdeg = int(round(heading_deg * 100.0))
        # clamp int16
        while heading_cdeg > 32767:
            heading_cdeg -= 65536
        while heading_cdeg < -32768:
            heading_cdeg += 65536

        fix_status = int(getattr(self._fix.status, 'status', -1))

        # age_ms from fix stamp
        stamp_ms = int(self._fix.header.stamp.sec * 1000 + self._fix.header.stamp.nanosec / 1e6)
        now_ms = _now_ms()
        age_ms = max(0, min(65535, now_ms - stamp_ms))

        return PoseStatusPacket(
            seq=seq,
            lat_e7=lat_e7,
            lon_e7=lon_e7,
            heading_cdeg=heading_cdeg,
            fix_status=fix_status,
            num_sv=int(self._num_sv),
            h_acc_mm=int(max(0, min(65535, self._h_acc_mm))),
            age_ms=int(age_ms),
            rtk_state=int(self._rtk_state),
        )

    def _make_hazard(self, seq: int) -> Optional[FrontHazardPacket]:
        if self._haz_last is None:
            return None
        hz = self._haz_last
        return FrontHazardPacket(seq=seq, forward_min_mm=hz.forward_min_mm, hazard_score=hz.hazard_score,
                                 sector_min_mm=hz.sector_min_mm, age_ms=hz.age_ms)

    def _send_if_base_silent(self):
        # When base is silent (no recent CMD), just send on timers
        if self._base_active():
            return

        now = time.time()
        seq_base = int(now * 1000.0) & 0xFFFF  # cheap seq seed for standalone

        if (now - self._last_hb_tx) >= self._hb_period:
            self._last_hb_tx = now
            self._send_packet(self._make_hb(seq_base))

        if (now - self._last_pose_tx) >= self._pose_period:
            pose = self._make_pose((seq_base + 1) & 0xFFFF)
            if pose is not None:
                self._last_pose_tx = now
                self._send_packet(pose)

        if (now - self._last_haz_tx) >= self._haz_period:
            hz = self._make_hazard((seq_base + 2) & 0xFFFF)
            if hz is not None:
                self._last_haz_tx = now
                self._send_packet(hz)

    def _on_cmd_edge(self):
        # When base is active (CMD packets), detect cmd edges and send 1 telemetry packet per edge
        if not self._base_active():
            self._last_seen_cmd_rx_t = self._last_cmd_rx_t
            return

        t = self._last_cmd_rx_t
        if t <= 0.0 or t == self._last_seen_cmd_rx_t:
            return

        self._last_seen_cmd_rx_t = t
        now = time.time()

        # choose the most overdue telemetry (priority: hazard > pose > hb)
        overdue = []
        overdue.append(('haz', now - self._last_haz_tx, self._haz_period))
        overdue.append(('pose', now - self._last_pose_tx, self._pose_period))
        overdue.append(('hb', now - self._last_hb_tx, self._hb_period))

        # pick item with (delta/period) largest
        best = None
        best_ratio = -1.0
        for name, delta, period in overdue:
            ratio = delta / max(0.001, period)
            if ratio > best_ratio and delta >= period:
                best_ratio = ratio
                best = name

        seq = int((now * 1000.0)) & 0xFFFF

        if best == 'haz':
            hz = self._make_hazard(seq)
            if hz is not None:
                self._last_haz_tx = now
                self._send_packet(hz)
        elif best == 'pose':
            pose = self._make_pose(seq)
            if pose is not None:
                self._last_pose_tx = now
                self._send_packet(pose)
        elif best == 'hb':
            self._last_hb_tx = now
            self._send_packet(self._make_hb(seq))


def main():
    rclpy.init()
    node = LoRaRoverTx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
