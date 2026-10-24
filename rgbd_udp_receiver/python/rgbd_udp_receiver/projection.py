import numpy as np

from .models import Calibration, ColoredPoints, DecodedBundle


class ProjectionError(ValueError):
    pass


def project_color_depth(
    bundle: DecodedBundle,
    calibration: Calibration,
    max_points: int,
    max_depth_mm: int,
) -> ColoredPoints:
    depth = bundle.depth.millimeters
    color = bundle.color.pixels
    expected = (calibration.height, calibration.width)
    if depth.shape != expected or depth.dtype != np.uint16:
        raise ProjectionError("depth image shape or dtype does not match calibration")
    if color.shape != expected + (3,) or color.dtype != np.uint8:
        raise ProjectionError("color image shape or dtype does not match calibration")

    valid = (depth > 0) & (depth <= max_depth_mm)
    rows, columns = np.nonzero(valid)
    count = rows.size
    if count == 0:
        return ColoredPoints(
            xyz=np.empty((0, 3), dtype=np.float32),
            rgb=np.empty((0, 3), dtype=np.uint8),
        )
    if count > max_points:
        selected = (np.arange(max_points, dtype=np.int64) * count) // max_points
        rows = rows[selected]
        columns = columns[selected]

    z = depth[rows, columns].astype(np.float32) * np.float32(calibration.depth_scale_m)
    fx = np.float32(calibration.k[0])
    fy = np.float32(calibration.k[4])
    cx = np.float32(calibration.k[2])
    cy = np.float32(calibration.k[5])
    x = (columns.astype(np.float32) - cx) * z / fx
    y = (rows.astype(np.float32) - cy) * z / fy
    xyz = np.ascontiguousarray(np.column_stack((x, y, z)), dtype=np.float32)
    rgb = np.ascontiguousarray(color[rows, columns], dtype=np.uint8)
    finite = np.isfinite(xyz).all(axis=1)
    if not finite.all():
        xyz = np.ascontiguousarray(xyz[finite])
        rgb = np.ascontiguousarray(rgb[finite])
    return ColoredPoints(xyz=xyz, rgb=rgb)
