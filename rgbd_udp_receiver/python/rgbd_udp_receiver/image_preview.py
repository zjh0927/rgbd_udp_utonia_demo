import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from .models import DecodedBundle


def depth_preview_lut(max_depth_mm: int) -> np.ndarray:
    """Map valid near depths to light gray and invalid depths to black."""
    lut = np.zeros(65536, dtype=np.uint8)
    if max_depth_mm == 1:
        lut[1] = 255
    else:
        distances = np.arange(1, max_depth_mm + 1, dtype=np.uint32)
        lut[1 : max_depth_mm + 1] = 255 - (
            (distances - 1) * 254 // (max_depth_mm - 1)
        ).astype(np.uint8)
    return lut


def make_preview_images(
    bundle: DecodedBundle, frame_id: str, depth_lut: np.ndarray
) -> tuple[Image, Image, Image]:
    color = bundle.color.pixels
    depth = bundle.depth.millimeters
    height, width = depth.shape
    if color.shape != (height, width, 3) or color.dtype != np.uint8:
        raise ValueError("color image must be uint8 RGB with the depth dimensions")
    if depth.dtype != np.uint16:
        raise ValueError("depth image must be uint16")

    timestamp_ns = bundle.depth.source_timestamp_ns
    header = Header(
        stamp=Time(
            sec=int(timestamp_ns // 1_000_000_000),
            nanosec=int(timestamp_ns % 1_000_000_000),
        ),
        frame_id=frame_id,
    )
    rgb = Image(
        header=header,
        height=height,
        width=width,
        encoding="rgb8",
        is_bigendian=0,
        step=width * 3,
        data=color.tobytes(order="C"),
    )
    raw_depth = Image(
        header=header,
        height=height,
        width=width,
        encoding="16UC1",
        is_bigendian=0,
        step=width * 2,
        data=depth.astype("<u2", copy=False).tobytes(order="C"),
    )
    gray = depth_lut[depth]
    preview = Image(
        header=header,
        height=height,
        width=width,
        encoding="mono8",
        is_bigendian=0,
        step=width,
        data=gray.tobytes(order="C"),
    )
    return rgb, raw_depth, preview
