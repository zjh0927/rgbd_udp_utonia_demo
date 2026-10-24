from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional

import av
import numpy as np
from av.error import FFmpegError

from .models import FLAG_KEYFRAME, CompleteFrame, DecodedColorFrame, StreamType


class H265DecodeError(RuntimeError):
    pass


@dataclass(frozen=True)
class _Metadata:
    source_id: int
    calibration_id: int
    bundle_sequence: int
    source_timestamp_ns: int


class H265Decoder:
    def __init__(self, width: int, height: int, max_pending_metadata: int):
        self._width = width
        self._height = height
        self._max_pending = max_pending_metadata
        self._codec = None
        self._pending: Deque[_Metadata] = deque()
        self._waiting_for_keyframe = True

    @property
    def waiting_for_keyframe(self) -> bool:
        return self._waiting_for_keyframe

    def _new_codec(self):
        self._codec = av.CodecContext.create("hevc", "r")

    def reset_for_discontinuity(self) -> None:
        self._codec = None
        self._pending.clear()
        self._waiting_for_keyframe = True

    def close(self) -> None:
        self.reset_for_discontinuity()

    def decode(self, frame: CompleteFrame) -> List[DecodedColorFrame]:
        if frame.stream_type != StreamType.H265:
            raise H265DecodeError("received a non-H.265 frame")
        keyframe = bool(frame.flags & FLAG_KEYFRAME)
        if self._waiting_for_keyframe:
            if not keyframe:
                return []
            self._new_codec()
            self._waiting_for_keyframe = False

        metadata = _Metadata(
            source_id=frame.source_id,
            calibration_id=frame.calibration_id,
            bundle_sequence=frame.bundle_sequence,
            source_timestamp_ns=frame.source_timestamp_ns,
        )
        self._pending.append(metadata)
        if len(self._pending) > self._max_pending:
            self.reset_for_discontinuity()
            raise H265DecodeError("H.265 decoder metadata queue exceeded its hard limit")

        try:
            decoded = self._codec.decode(av.Packet(frame.payload))
        except FFmpegError as exc:
            self.reset_for_discontinuity()
            raise H265DecodeError(f"FFmpeg HEVC decode failed: {exc}") from exc

        # 发送端明确按 AU 对齐并禁用 B 帧。严格要求一个 AU 对应一张图，
        # 否则无法证明 FIFO 元数据仍与画面对应，宁可等下一个 IDR 恢复。
        if len(decoded) != 1:
            count = len(decoded)
            self.reset_for_discontinuity()
            raise H265DecodeError(
                f"one H.265 access unit produced {count} images; metadata association is unsafe"
            )

        output: List[DecodedColorFrame] = []
        for image in decoded:
            if not self._pending:
                self.reset_for_discontinuity()
                raise H265DecodeError("decoder produced a frame without matching metadata")
            matched = self._pending.popleft()
            if image.width != self._width or image.height != self._height:
                self.reset_for_discontinuity()
                raise H265DecodeError(
                    f"unexpected decoded dimensions {image.width}x{image.height}; "
                    f"expected {self._width}x{self._height}"
                )
            pixels = np.ascontiguousarray(image.to_ndarray(format="rgb24"))
            output.append(
                DecodedColorFrame(
                    source_id=matched.source_id,
                    calibration_id=matched.calibration_id,
                    bundle_sequence=matched.bundle_sequence,
                    source_timestamp_ns=matched.source_timestamp_ns,
                    pixels=pixels,
                )
            )
        return output
