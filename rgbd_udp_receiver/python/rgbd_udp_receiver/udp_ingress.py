import socket
import threading
import time
from collections import defaultdict
from typing import Callable, Dict, List, Optional, Tuple

from .config import ReceiverConfig
from .models import CompleteFrame, StreamType
from .protocol import PacketValidationError, ProtocolLimits, parse_datagram
from .reassembly import ReassemblyManager


FrameCallback = Callable[[CompleteFrame], None]
DiscontinuityCallback = Callable[[StreamType, str], None]
WarningCallback = Callable[[str], None]


class UdpIngress:
    def __init__(
        self,
        config: ReceiverConfig,
        frame_callback: FrameCallback,
        discontinuity_callback: DiscontinuityCallback,
        warning_callback: WarningCallback,
    ):
        self._config = config
        self._frame_callback = frame_callback
        self._discontinuity_callback = discontinuity_callback
        self._warning_callback = warning_callback
        self._limits = ProtocolLimits(
            expected_source_id=config.expected_source_id,
            max_datagram_bytes=config.max_datagram_bytes,
            max_frame_bytes=config.max_frame_bytes,
            max_fragments=config.max_fragments,
        )
        self._reassembly = ReassemblyManager(
            timeout_ns=config.reassembly_timeout_ms * 1_000_000,
            max_inflight_frames=config.max_inflight_frames,
            max_inflight_bytes=config.max_inflight_bytes,
            drop_callback=discontinuity_callback,
        )
        self._stop = threading.Event()
        self._sockets: List[socket.socket] = []
        self._threads: List[threading.Thread] = []
        self._stats: Dict[str, int] = defaultdict(int)
        self._stats_lock = threading.Lock()

    def _increment(self, name: str) -> None:
        with self._stats_lock:
            self._stats[name] += 1

    def start(self) -> None:
        if self._threads:
            raise RuntimeError("UDP ingress has already been started")
        endpoints = (
            (StreamType.H265, self._config.h265_port),
            (StreamType.ZDEPTH, self._config.zdepth_port),
            (StreamType.CALIBRATION, self._config.calibration_port),
        )
        try:
            for stream, port in endpoints:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_RCVBUF,
                    self._config.socket_receive_buffer_bytes,
                )
                sock.settimeout(self._config.socket_timeout_ms / 1000.0)
                sock.bind((self._config.bind_address, port))
                actual = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
                if actual < self._config.socket_receive_buffer_bytes:
                    self._warning_callback(
                        f"UDP port {port} SO_RCVBUF={actual}, lower than requested "
                        f"{self._config.socket_receive_buffer_bytes}"
                    )
                self._sockets.append(sock)
                thread = threading.Thread(
                    target=self._receive_loop,
                    args=(sock, stream, port),
                    name=f"rgbd-udp-{port}",
                    daemon=False,
                )
                self._threads.append(thread)
            for thread in self._threads:
                thread.start()
        except Exception:
            self.stop()
            raise

    def _receive_loop(self, sock: socket.socket, stream: StreamType, port: int) -> None:
        label = stream.name.lower()
        while not self._stop.is_set():
            try:
                data, _peer = sock.recvfrom(self._config.max_datagram_bytes + 1)
            except socket.timeout:
                try:
                    self._reassembly.expire(time.monotonic_ns())
                except Exception:
                    self._increment("reassembly_unexpected_errors")
                continue
            except OSError:
                if not self._stop.is_set():
                    self._increment(f"{label}_socket_errors")
                break
            self._increment(f"{label}_datagrams")
            try:
                datagram = parse_datagram(data, stream, self._limits)
            except PacketValidationError as exc:
                self._increment(f"invalid_{exc.code}")
                if stream == StreamType.H265:
                    self._discontinuity_callback(stream, "invalid_packet")
                continue
            try:
                frame = self._reassembly.push(datagram, time.monotonic_ns())
            except Exception:
                self._increment("reassembly_unexpected_errors")
                if stream == StreamType.H265:
                    self._discontinuity_callback(stream, "reassembly_error")
                continue
            if frame is None:
                continue
            self._increment(f"{label}_frames")
            try:
                self._frame_callback(frame)
            except Exception:
                self._increment(f"{label}_dispatch_errors")

    def stop(self) -> List[str]:
        self._stop.set()
        for sock in self._sockets:
            try:
                sock.close()
            except OSError:
                pass
        for thread in self._threads:
            if thread is not threading.current_thread() and thread.is_alive():
                thread.join(timeout=2.0)
        alive = [thread.name for thread in self._threads if thread.is_alive()]
        self._sockets.clear()
        return alive

    def snapshot(self) -> dict:
        with self._stats_lock:
            result = dict(self._stats)
        result.update({f"reassembly_{key}": value for key, value in self._reassembly.snapshot().items()})
        return result
