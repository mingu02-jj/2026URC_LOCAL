#!/usr/bin/env python3
"""
COBS-framed, CRC-protected tiny binary protocol for cmd_vel over E22 (transparent UART).
Frame delimiter: 0x00
Payload (11 bytes) = [type(1)][seq(1)][flags(1)][vx_i16][vy_i16][wz_i16][crc16]
- vx_i16 = round(vx * 1000)   where vx in [-1, 1]
- wz_i16 = round(wz * 1000)   where wz in [-0.5, 0.5]
- vy included for future; set 0 if not used.
"""
from __future__ import annotations
from dataclasses import dataclass

CMD_TYPE_CMDVEL = 0x01
FLAG_ESTOP = 1 << 0

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

def cobs_encode(data: bytes) -> bytes:
    if not data:
        return b'\x01'
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
        out.extend(data[idx:idx+n])
        idx += n
        if code < 0xFF and idx < len(data):
            out.append(0)
    return bytes(out)

def frame_encode(payload_with_crc: bytes) -> bytes:
    return cobs_encode(payload_with_crc) + b'\x00'

class FrameDecoder:
    def __init__(self, max_frame_len: int = 64):
        self._buf = bytearray()
        self._max = max_frame_len
    def push(self, data: bytes):
        for b in data:
            if b == 0:
                if self._buf:
                    raw = bytes(self._buf)
                    self._buf.clear()
                    try:
                        yield cobs_decode(raw)
                    except Exception:
                        continue
                else:
                    continue
            else:
                if len(self._buf) < self._max:
                    self._buf.append(b)
                else:
                    self._buf.clear()

@dataclass
class CmdVelPacket:
    seq: int
    flags: int
    vx_i: int
    vy_i: int
    wz_i: int

    def to_bytes(self) -> bytes:
        body = bytearray()
        body.append(CMD_TYPE_CMDVEL)
        body.append(self.seq & 0xFF)
        body.append(self.flags & 0xFF)
        body += int(self.vx_i).to_bytes(2, 'little', signed=True)
        body += int(self.vy_i).to_bytes(2, 'little', signed=True)
        body += int(self.wz_i).to_bytes(2, 'little', signed=True)
        crc = crc16_ccitt_false(bytes(body))
        body += crc.to_bytes(2, 'little', signed=False)
        return bytes(body)

    @staticmethod
    def from_bytes(body_with_crc: bytes) -> "CmdVelPacket":
        if len(body_with_crc) != 11:
            raise ValueError("Bad length")
        body = body_with_crc[:-2]
        rx_crc = int.from_bytes(body_with_crc[-2:], 'little', signed=False)
        calc = crc16_ccitt_false(body)
        if rx_crc != calc:
            raise ValueError("CRC mismatch")
        if body[0] != CMD_TYPE_CMDVEL:
            raise ValueError("Bad type")
        seq = body[1]
        flags = body[2]
        vx_i = int.from_bytes(body[3:5], 'little', signed=True)
        vy_i = int.from_bytes(body[5:7], 'little', signed=True)
        wz_i = int.from_bytes(body[7:9], 'little', signed=True)
        return CmdVelPacket(seq=seq, flags=flags, vx_i=vx_i, vy_i=vy_i, wz_i=wz_i)
