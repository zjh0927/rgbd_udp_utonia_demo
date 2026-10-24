import copy
from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class CloudFormatError(ValueError):
    """Raised when an input PointCloud2 is incompatible with the wire layout."""


@dataclass(frozen=True)
class CloudFrame:
    xyz: np.ndarray
    rgb: np.ndarray
    header: Header

    @property
    def count(self) -> int:
        return int(self.xyz.shape[0])


_EXPECTED_FIELDS = {
    "x": (0, PointField.FLOAT32, 1),
    "y": (4, PointField.FLOAT32, 1),
    "z": (8, PointField.FLOAT32, 1),
    "rgb": (12, PointField.FLOAT32, 1),
}


def _validate_layout(message: PointCloud2) -> int:
    if message.is_bigendian:
        raise CloudFormatError("big-endian PointCloud2 is not supported")
    if message.point_step != 16:
        raise CloudFormatError(f"point_step must be 16, got {message.point_step}")
    if message.height < 1 or message.width < 0:
        raise CloudFormatError("height must be positive and width must be non-negative")
    expected_row_step = message.width * message.point_step
    if message.row_step != expected_row_step:
        raise CloudFormatError(
            f"row_step must be {expected_row_step}, got {message.row_step}"
        )
    expected_bytes = message.row_step * message.height
    if len(message.data) != expected_bytes:
        raise CloudFormatError(
            f"data length must be {expected_bytes}, got {len(message.data)}"
        )

    fields = {field.name: field for field in message.fields}
    if set(fields) != set(_EXPECTED_FIELDS):
        raise CloudFormatError("fields must be exactly x, y, z and rgb")
    for name, expected in _EXPECTED_FIELDS.items():
        field = fields[name]
        actual = (field.offset, field.datatype, field.count)
        if actual != expected:
            raise CloudFormatError(
                f"field {name} must be offset/datatype/count={expected}, got {actual}"
            )
    return int(message.width * message.height)


def decode_xyzrgb(message: PointCloud2) -> CloudFrame:
    count = _validate_layout(message)
    if count == 0:
        raise CloudFormatError("point cloud is empty")

    packed = np.frombuffer(message.data, dtype="<f4", count=count * 4).reshape(count, 4)
    xyz = np.asarray(packed[:, :3], dtype=np.float32)
    rgb_u32 = packed[:, 3].view("<u4")
    rgb = np.empty((count, 3), dtype=np.uint8)
    rgb[:, 0] = (rgb_u32 >> 16) & 0xFF
    rgb[:, 1] = (rgb_u32 >> 8) & 0xFF
    rgb[:, 2] = rgb_u32 & 0xFF

    finite = np.isfinite(xyz).all(axis=1)
    if not finite.any():
        raise CloudFormatError("point cloud has no finite XYZ samples")
    xyz = np.ascontiguousarray(xyz[finite], dtype=np.float32)
    rgb = np.ascontiguousarray(rgb[finite], dtype=np.uint8)
    return CloudFrame(xyz=xyz, rgb=rgb, header=copy.deepcopy(message.header))


def make_xyzrgb(frame: CloudFrame, colors: np.ndarray) -> PointCloud2:
    xyz = np.asarray(frame.xyz)
    colors = np.asarray(colors)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.dtype != np.float32:
        raise ValueError("frame.xyz must have dtype float32 and shape (N, 3)")
    if colors.shape != xyz.shape:
        raise ValueError("colors must have shape (N, 3)")
    if not np.isfinite(xyz).all():
        raise ValueError("frame.xyz must contain only finite values")
    if colors.dtype != np.uint8:
        raise ValueError("colors must have dtype uint8")

    count = xyz.shape[0]
    packed = np.empty((count, 4), dtype="<f4")
    packed[:, :3] = xyz
    color_u32 = colors.astype(np.uint32, copy=False)
    rgb_u32 = (color_u32[:, 0] << 16) | (color_u32[:, 1] << 8) | color_u32[:, 2]
    packed[:, 3] = rgb_u32.astype("<u4", copy=False).view("<f4")

    message = PointCloud2()
    message.header = copy.deepcopy(frame.header)
    message.height = 1
    message.width = count
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = count * message.point_step
    message.data = packed.tobytes(order="C")
    message.is_dense = True
    return message
