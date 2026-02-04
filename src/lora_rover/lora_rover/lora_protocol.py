#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URC 900MHz LoRa(E22) backup link protocol.

Framing / Integrity
- COBS framing with 0x00 delimiter
- CRC16-CCITT(FALSE)
- Little-endian

On-wire common layout
- BODY = [type:u8][seq:u16][payload...]
- PACKET = BODY + [crc16:u16]
- FRAME = COBS(PACKET) + 0x00

Packet types
- 0x01 CMD_VEL
- 0x10 HB_STATUS
- 0x11 POSE_STATUS
- 0x12 FRONT_HAZARD

Notes
- seq is u16 (rollover at 65535)
- All integer fields are little-endian
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable, Optional

# -----------------------------
# Constants
# -----------------------------
PKT_CMD_VEL = 0x01
PKT_HB_STATUS = 0x10
PKT_POSE_STATUS = 0x11
PKT_FRONT_HAZARD = 0x12

FLAG_ESTOP = 1 << 0

# RTK state (u8)
RTK_UNKNOWN = 0
RTK_NO_FIX = 1
RTK_2D = 2
RTK_3D = 3
RTK_FLOAT = 4
RTK_FIXED = 5

# -----------------------------
# CRC16-CCITT(FALSE)
# -----------------------------

def crc16_ccitt_false(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


# -----------------------------
# COBS
# -----------------------------

def cobs_encode(data: bytes) -> bytes:
    if not data:
        return b"\x01"
    out = bytearray()
    idx = 0
    while idx < len(data):
        code_pos = len(out)
        out.append(0)
        code = 1
        while idx < len(data) and data[idx] != 0 and code < 0xFF:
            out.append(data[idx])
            idx += 1
            code += 1
        out[code_pos] = code
        if idx < len(data) and data[idx] == 0:
            idx += 1
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        if code == 0:
            raise ValueError("COBS: zero code")
        idx += 1
        n = code - 1
        if idx + n > len(data) and code != 1:
            raise ValueError("COBS: overrun")
        out.extend(data[idx:idx + n])
        idx += n
        if code < 0xFF and idx < len(data):
            out.append(0)
    return bytes(out)


def frame_encode(packet: bytes) -> bytes:
    return cobs_encode(packet) + b"\x00"


class FrameDecoder:
    """Push raw UART bytes, yields decoded packets (still include CRC)."""

    def __init__(self, max_frame_len: int = 256):
        self._buf = bytearray()
        self._max = int(max_frame_len)

    def push(self, data: bytes) -> Iterable[bytes]:
        for b in data:
            if b == 0:
                if not self._buf:
                    continue
                raw = bytes(self._buf)
                self._buf.clear()
                try:
                    yield cobs_decode(raw)
                except Exception:
                    continue
            else:
                if len(self._buf) < self._max:
                    self._buf.append(b)
                else:
                    self._buf.clear()


# -----------------------------
# Packet helpers
# -----------------------------

def _append_crc(body: bytes) -> bytes:
    crc = crc16_ccitt_false(body)
    return body + struct.pack('<H', crc)


def _verify_and_strip_crc(packet: bytes) -> bytes:
    if len(packet) < 1 + 2 + 2:
        raise ValueError("too short")
    body = packet[:-2]
    rx_crc = struct.unpack('<H', packet[-2:])[0]
    calc = crc16_ccitt_false(body)
    if rx_crc != calc:
        raise ValueError("crc mismatch")
    return body


def rtk_state_to_str(v: int) -> str:
    return {
        RTK_UNKNOWN: 'unknown',
        RTK_NO_FIX: 'no_fix',
        RTK_2D: '2d',
        RTK_3D: '3d',
        RTK_FLOAT: 'float',
        RTK_FIXED: 'fixed',
    }.get(int(v), 'unknown')


# -----------------------------
# Packets
# -----------------------------

@dataclass
class CmdVelPacket:
    seq: int
    flags: int
    vx_i16: int
    vy_i16: int
    wz_i16: int

    _FMT_BODY = struct.Struct('<BHBhhh')  # type, seq(u16), flags, vx, vy, wz

    def to_bytes_with_crc(self) -> bytes:
        body = self._FMT_BODY.pack(
            PKT_CMD_VEL,
            int(self.seq) & 0xFFFF,
            int(self.flags) & 0xFF,
            int(self.vx_i16),
            int(self.vy_i16),
            int(self.wz_i16),
        )
        return _append_crc(body)

    @staticmethod
    def from_bytes_with_crc(packet: bytes) -> 'CmdVelPacket':
        body = _verify_and_strip_crc(packet)
        if len(body) != CmdVelPacket._FMT_BODY.size:
            raise ValueError('bad length')
        t, seq, flags, vx, vy, wz = CmdVelPacket._FMT_BODY.unpack(body)
        if t != PKT_CMD_VEL:
            raise ValueError('bad type')
        return CmdVelPacket(seq=seq, flags=flags, vx_i16=vx, vy_i16=vy, wz_i16=wz)


@dataclass
class HbStatusPacket:
    seq: int
    uptime_ms: int
    estop_latched: int
    rtk_state: int
    last_cmd_age_ms: int

    _FMT_BODY = struct.Struct('<BHI BBH')  # type, seq, uptime_ms, estop, rtk, last_cmd_age

    def to_bytes_with_crc(self) -> bytes:
        body = self._FMT_BODY.pack(
            PKT_HB_STATUS,
            int(self.seq) & 0xFFFF,
            int(self.uptime_ms) & 0xFFFFFFFF,
            int(self.estop_latched) & 0xFF,
            int(self.rtk_state) & 0xFF,
            int(self.last_cmd_age_ms) & 0xFFFF,
        )
        return _append_crc(body)

    @staticmethod
    def from_bytes_with_crc(packet: bytes) -> 'HbStatusPacket':
        body = _verify_and_strip_crc(packet)
        if len(body) != HbStatusPacket._FMT_BODY.size:
            raise ValueError('bad length')
        t, seq, uptime_ms, estop_latched, rtk_state, last_cmd_age_ms = HbStatusPacket._FMT_BODY.unpack(body)
        if t != PKT_HB_STATUS:
            raise ValueError('bad type')
        return HbStatusPacket(seq=seq, uptime_ms=uptime_ms, estop_latched=estop_latched,
                              rtk_state=rtk_state, last_cmd_age_ms=last_cmd_age_ms)


@dataclass
class PoseStatusPacket:
    seq: int
    lat_e7: int
    lon_e7: int
    heading_cdeg: int
    fix_status: int
    num_sv: int
    h_acc_mm: int
    age_ms: int
    rtk_state: int

    _FMT_BODY = struct.Struct('<BHii h bb H H b x')
    # type, seq, lat, lon, heading_cdeg, fix_status(i8), num_sv(i8), h_acc_mm(u16), age_ms(u16), rtk_state(i8), pad

    def to_bytes_with_crc(self) -> bytes:
        body = self._FMT_BODY.pack(
            PKT_POSE_STATUS,
            int(self.seq) & 0xFFFF,
            int(self.lat_e7),
            int(self.lon_e7),
            int(self.heading_cdeg),
            int(self.fix_status),
            int(self.num_sv),
            int(self.h_acc_mm) & 0xFFFF,
            int(self.age_ms) & 0xFFFF,
            int(self.rtk_state),
        )
        return _append_crc(body)

    @staticmethod
    def from_bytes_with_crc(packet: bytes) -> 'PoseStatusPacket':
        body = _verify_and_strip_crc(packet)
        if len(body) != PoseStatusPacket._FMT_BODY.size:
            raise ValueError('bad length')
        (t, seq, lat_e7, lon_e7, heading_cdeg, fix_status, num_sv, h_acc_mm, age_ms, rtk_state) = PoseStatusPacket._FMT_BODY.unpack(body)
        if t != PKT_POSE_STATUS:
            raise ValueError('bad type')
        return PoseStatusPacket(
            seq=seq,
            lat_e7=lat_e7,
            lon_e7=lon_e7,
            heading_cdeg=heading_cdeg,
            fix_status=fix_status,
            num_sv=num_sv,
            h_acc_mm=h_acc_mm,
            age_ms=age_ms,
            rtk_state=rtk_state,
        )


@dataclass
class FrontHazardPacket:
    seq: int
    forward_min_mm: int
    hazard_score: int
    sector_min_mm: list[int]  # len=12
    age_ms: int

    _FMT_HEAD = struct.Struct('<BH H B')  # type, seq, forward_min_mm, hazard_score
    _FMT_TAIL = struct.Struct('<H')       # age_ms

    def to_bytes_with_crc(self) -> bytes:
        if len(self.sector_min_mm) != 12:
            raise ValueError('sector_min_mm must have len=12')
        body = bytearray()
        body += self._FMT_HEAD.pack(
            PKT_FRONT_HAZARD,
            int(self.seq) & 0xFFFF,
            int(self.forward_min_mm) & 0xFFFF,
            int(self.hazard_score) & 0xFF,
        )
        for d in self.sector_min_mm:
            body += struct.pack('<H', int(d) & 0xFFFF)
        body += self._FMT_TAIL.pack(int(self.age_ms) & 0xFFFF)
        return _append_crc(bytes(body))

    @staticmethod
    def from_bytes_with_crc(packet: bytes) -> 'FrontHazardPacket':
        body = _verify_and_strip_crc(packet)
        # expected: head(1+2+2+1)=6 + 12*2=24 + tail(2)=32
        if len(body) != 32:
            raise ValueError('bad length')
        t, seq, forward_min_mm, hazard_score = FrontHazardPacket._FMT_HEAD.unpack(body[:FrontHazardPacket._FMT_HEAD.size])
        if t != PKT_FRONT_HAZARD:
            raise ValueError('bad type')
        off = FrontHazardPacket._FMT_HEAD.size
        sector = []
        for _ in range(12):
            sector.append(struct.unpack('<H', body[off:off+2])[0])
            off += 2
        age_ms = FrontHazardPacket._FMT_TAIL.unpack(body[off:off+2])[0]
        return FrontHazardPacket(seq=seq, forward_min_mm=forward_min_mm, hazard_score=hazard_score,
                                 sector_min_mm=sector, age_ms=age_ms)


def packet_type(packet: bytes) -> int:
    """Return packet type after CRC check."""
    body = _verify_and_strip_crc(packet)
    return body[0]
