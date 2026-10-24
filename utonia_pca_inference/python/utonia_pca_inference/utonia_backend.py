import copy
import hashlib
import os
import sys
from dataclasses import dataclass
from typing import Any

import numpy as np

from .cloud_io import CloudFrame
from .config import InferenceConfig


class BackendError(RuntimeError):
    pass


class CudaOutOfMemory(BackendError):
    pass


@dataclass(frozen=True)
class FeatureFrame:
    xyz: np.ndarray
    features: Any
    header: Any

    @property
    def count(self) -> int:
        return int(self.xyz.shape[0])


def _sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _uniform_indices(count: int, limit: int) -> np.ndarray:
    if count <= limit:
        return np.arange(count, dtype=np.int64)
    return (np.arange(limit, dtype=np.int64) * count) // limit


def _first_source_per_grid(inverse: np.ndarray, grid_count: int) -> np.ndarray:
    if inverse.ndim != 1 or inverse.size == 0:
        raise BackendError("GridSample inverse must be a non-empty vector")
    if inverse.min() < 0 or inverse.max() >= grid_count:
        raise BackendError("GridSample inverse contains an out-of-range index")
    order = np.argsort(inverse, kind="stable")
    grid_ids, first_positions = np.unique(inverse[order], return_index=True)
    if grid_ids.size != grid_count:
        raise BackendError("GridSample inverse does not cover every model point")
    first = np.empty(grid_count, dtype=np.int64)
    first[grid_ids] = order[first_positions]
    return first


class UtoniaBackend:
    def __init__(self, config: InferenceConfig):
        self.config = config
        source = os.path.abspath(os.path.expanduser(config.utonia_source_path))
        checkpoint = os.path.abspath(os.path.expanduser(config.checkpoint_path))
        if not os.path.isdir(os.path.join(source, "utonia")):
            raise BackendError(f"Utonia source tree not found: {source}")
        if not os.path.isfile(checkpoint):
            raise BackendError(f"Utonia checkpoint not found: {checkpoint}")
        if source not in sys.path:
            sys.path.insert(0, source)

        try:
            import torch
            import utonia
        except Exception as exc:
            raise BackendError(f"cannot import Utonia runtime: {exc}") from exc
        if not torch.cuda.is_available():
            raise BackendError("CUDA is unavailable in the active Python environment")

        self.torch = torch
        self.utonia = utonia
        self.device = torch.device("cuda:0")
        self.checkpoint_path = checkpoint
        self.model_sha256 = _sha256_file(checkpoint)
        try:
            self.model = utonia.model.load(
                checkpoint,
                custom_config={"enable_flash": False},
            ).to(self.device).eval()
        except Exception as exc:
            raise BackendError(f"failed to load Utonia Tiny: {exc}") from exc
        self.transform = utonia.transform.default(
            scale=config.utonia_scale,
            apply_z_positive=True,
            normalize_coord=False,
        )

    def warmup(self) -> FeatureFrame:
        from std_msgs.msg import Header

        count = self.config.warmup_points
        side = int(np.ceil(count ** (1.0 / 3.0)))
        grid = np.indices((side, side, side), dtype=np.float32).reshape(3, -1).T[:count]
        grid = (grid - np.float32(side / 2.0)) * np.float32(0.025)
        xyz = np.empty_like(grid, dtype=np.float32)
        xyz[:, 0] = grid[:, 0]
        xyz[:, 1] = grid[:, 1]
        xyz[:, 2] = grid[:, 2] + np.float32(2.0)
        rgb = np.full((count, 3), 127, dtype=np.uint8)
        return self.infer(CloudFrame(xyz=xyz, rgb=rgb, header=Header(frame_id="warmup")))

    def infer(self, frame: CloudFrame, max_points: int = None) -> FeatureFrame:
        if frame.count < self.config.min_model_points:
            raise BackendError(
                f"point cloud has {frame.count} points; need at least "
                f"{self.config.min_model_points}"
            )
        limit = self.config.max_model_points if max_points is None else int(max_points)
        limit = min(max(limit, self.config.min_model_points), self.config.max_model_points)
        selected = _uniform_indices(frame.count, limit)
        optical_xyz = np.ascontiguousarray(frame.xyz[selected], dtype=np.float32)
        rgb = np.ascontiguousarray(frame.rgb[selected], dtype=np.float32)

        model_xyz = np.empty_like(optical_xyz)
        model_xyz[:, 0] = optical_xyz[:, 0]
        model_xyz[:, 1] = optical_xyz[:, 2]
        model_xyz[:, 2] = -optical_xyz[:, 1]
        point = {
            "coord": model_xyz,
            "color": rgb,
            "normal": np.zeros_like(model_xyz, dtype=np.float32),
        }

        np.random.seed(self.config.random_seed)
        self.torch.manual_seed(self.config.random_seed)
        try:
            point = self.transform(point)
            inverse_cpu = point["inverse"].detach().cpu().numpy().astype(np.int64)
            grid_count = int(point["coord"].shape[0])
            first_source = _first_source_per_grid(inverse_cpu, grid_count)
            publish_xyz = np.ascontiguousarray(optical_xyz[first_source], dtype=np.float32)

            for key, value in tuple(point.items()):
                if isinstance(value, self.torch.Tensor):
                    point[key] = value.to(self.device, non_blocking=True)

            with self.torch.inference_mode():
                output = self.model(point)
                for _ in range(2):
                    if not {"pooling_parent", "pooling_inverse"}.issubset(output.keys()):
                        raise BackendError("Utonia hierarchy ended before two feature joins")
                    parent = output.pop("pooling_parent")
                    inverse = output.pop("pooling_inverse")
                    parent.feat = self.torch.cat(
                        [parent.feat, output.feat[inverse]], dim=-1
                    )
                    output = parent
                while "pooling_parent" in output.keys():
                    if "pooling_inverse" not in output.keys():
                        raise BackendError("Utonia hierarchy is missing pooling_inverse")
                    parent = output.pop("pooling_parent")
                    inverse = output.pop("pooling_inverse")
                    parent.feat = output.feat[inverse]
                    output = parent
                features = output.feat.detach().contiguous()
                # PyTorch CUDA kernels are asynchronous.  Synchronize here so
                # inference timing and OOM handling belong to this frame,
                # instead of being deferred into the following PCA stage.
                self.torch.cuda.synchronize(self.device)
        except BackendError:
            raise
        except self.torch.cuda.OutOfMemoryError as exc:
            raise CudaOutOfMemory(str(exc)) from exc
        except RuntimeError as exc:
            if "out of memory" in str(exc).lower():
                raise CudaOutOfMemory(str(exc)) from exc
            raise BackendError(f"Utonia forward failed: {exc}") from exc
        except Exception as exc:
            raise BackendError(f"Utonia input transform failed: {exc}") from exc

        if features.ndim != 2 or features.shape[0] != publish_xyz.shape[0]:
            raise BackendError(
                f"feature/XYZ mismatch: features={tuple(features.shape)}, "
                f"xyz={publish_xyz.shape}"
            )
        if features.shape[1] < 6:
            raise BackendError("Utonia feature dimension is smaller than six")
        return FeatureFrame(
            xyz=publish_xyz,
            features=features,
            header=copy.deepcopy(frame.header),
        )

    def empty_cache(self) -> None:
        self.torch.cuda.empty_cache()
