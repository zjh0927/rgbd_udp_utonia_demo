import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from .models import ColoredPoints


def make_pointcloud2(points: ColoredPoints, timestamp_ns: int, frame_id: str) -> PointCloud2:
    if points.xyz.ndim != 2 or points.xyz.shape[1] != 3:
        raise ValueError("xyz must have shape (N, 3)")
    if points.rgb.shape != points.xyz.shape:
        raise ValueError("rgb must have shape (N, 3)")
    count = points.xyz.shape[0]
    packed = np.empty((count, 4), dtype="<f4")
    packed[:, :3] = points.xyz
    colors = points.rgb.astype(np.uint32, copy=False)
    rgb_u32 = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    packed[:, 3] = rgb_u32.astype("<u4", copy=False).view("<f4")

    stamp = Time(sec=int(timestamp_ns // 1_000_000_000), nanosec=int(timestamp_ns % 1_000_000_000))
    message = PointCloud2()
    message.header = Header(stamp=stamp, frame_id=frame_id)
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
    message.row_step = message.point_step * count
    message.data = packed.tobytes(order="C")
    message.is_dense = True
    return message
