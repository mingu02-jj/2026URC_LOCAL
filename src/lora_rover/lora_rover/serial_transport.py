#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import queue
import time
from typing import Optional

import serial

from .lora_protocol import FrameDecoder


class SerialTransport:
    """Owns the UART serial port and provides:
    - background reader thread that pushes decoded packets(bytes, still include CRC) into a Queue
    - thread-safe write() with an internal lock

    NOTE: For E22 half-duplex, opening the same /dev/ttyUSBx from multiple processes will conflict.
          Use a single process (duplex runner) to share this transport.
    """

    def __init__(
        self,
        port: str,
        baudrate: int,
        read_timeout: float = 0.01,
        write_timeout: float = 0.0,
        max_frame_len: int = 256,
        read_chunk: int = 256,
    ):
        self._port = port
        self._baud = int(baudrate)
        self._read_chunk = int(read_chunk)

        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baud,
            timeout=read_timeout,
            write_timeout=write_timeout,
        )

        self._decoder = FrameDecoder(max_frame_len=max_frame_len)
        self._rx_q: "queue.Queue[bytes]" = queue.Queue(maxsize=200)

        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._read_loop, daemon=True)
        self._thr.start()

    @property
    def port(self) -> str:
        return self._port

    @property
    def baudrate(self) -> int:
        return self._baud

    def close(self):
        self._stop.set()
        try:
            self._thr.join(timeout=0.5)
        except Exception:
            pass
        try:
            self._ser.close()
        except Exception:
            pass

    def write(self, frame: bytes):
        with self._tx_lock:
            self._ser.write(frame)

    def recv_queue(self) -> "queue.Queue[bytes]":
        return self._rx_q

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                data = self._ser.read(self._read_chunk)
            except Exception:
                time.sleep(0.02)
                continue
            if not data:
                continue
            for pkt in self._decoder.push(data):
                try:
                    self._rx_q.put_nowait(pkt)
                except queue.Full:
                    # drop oldest to keep freshest
                    try:
                        _ = self._rx_q.get_nowait()
                        self._rx_q.put_nowait(pkt)
                    except Exception:
                        pass
