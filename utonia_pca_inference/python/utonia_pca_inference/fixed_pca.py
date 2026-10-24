import os
import tempfile
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np


PROFILE_SCHEMA_VERSION = 1


class PcaError(RuntimeError):
    pass


@dataclass(frozen=True)
class PcaIdentity:
    model_sha256: str
    feature_dim: int
    utonia_scale: float
    max_model_points: int


class FixedPca:
    def __init__(
        self,
        profile_path: str,
        calibration_frames: int,
        max_samples: int,
        identity: PcaIdentity,
        random_seed: int = 37,
    ):
        if identity.feature_dim < 6:
            raise ValueError("PCA requires a feature dimension of at least six")
        if calibration_frames < 2:
            raise ValueError("calibration_frames must be at least two")
        if max_samples < 6:
            raise ValueError("max_samples must be at least six")
        self.profile_path = os.path.abspath(os.path.expanduser(profile_path))
        self.calibration_frames = int(calibration_frames)
        self.max_samples = int(max_samples)
        self.identity = identity
        self._rng = np.random.default_rng(random_seed)
        self._frames = 0
        self._reservoir_features = np.empty((0, identity.feature_dim), dtype=np.float32)
        self._reservoir_keys = np.empty((0,), dtype=np.float64)
        self._mean: Optional[np.ndarray] = None
        self._components: Optional[np.ndarray] = None
        self._low: Optional[np.ndarray] = None
        self._high: Optional[np.ndarray] = None
        self.load_error = ""
        self.loaded_existing = self._try_load()

    @property
    def ready(self) -> bool:
        return self._mean is not None

    @property
    def observed_frames(self) -> int:
        return self._frames

    @property
    def remaining_frames(self) -> int:
        return max(0, self.calibration_frames - self._frames)

    def observe(self, features: Any) -> Optional[np.ndarray]:
        self._validate_features(features)
        if self.ready:
            return self.colorize(features)

        samples = features.detach().to(device="cpu", dtype=features.new_zeros(1).dtype)
        samples_np = np.ascontiguousarray(samples.float().numpy(), dtype=np.float32)
        if not np.isfinite(samples_np).all():
            raise PcaError("calibration features contain NaN or Inf")
        keys = self._rng.random(samples_np.shape[0])
        merged_features = np.concatenate((self._reservoir_features, samples_np), axis=0)
        merged_keys = np.concatenate((self._reservoir_keys, keys), axis=0)
        if merged_keys.size > self.max_samples:
            keep = np.argpartition(merged_keys, self.max_samples - 1)[: self.max_samples]
            merged_features = merged_features[keep]
            merged_keys = merged_keys[keep]
        self._reservoir_features = np.ascontiguousarray(merged_features, dtype=np.float32)
        self._reservoir_keys = np.ascontiguousarray(merged_keys, dtype=np.float64)
        self._frames += 1

        if self._frames < self.calibration_frames:
            return None
        self._fit(features.device)
        self._save_atomic()
        return self.colorize(features)

    def colorize(self, features: Any) -> np.ndarray:
        self._validate_features(features)
        if not self.ready:
            raise PcaError("PCA profile is not ready")
        torch = __import__("torch")
        mean = torch.as_tensor(self._mean, device=features.device, dtype=features.dtype)
        components = torch.as_tensor(
            self._components, device=features.device, dtype=features.dtype
        )
        low = torch.as_tensor(self._low, device=features.device, dtype=features.dtype)
        high = torch.as_tensor(self._high, device=features.device, dtype=features.dtype)
        with torch.inference_mode():
            projection = (features - mean) @ components
            combined = projection[:, :3] * 0.6 + projection[:, 3:6] * 0.4
            normalized = ((combined - low) / torch.clamp(high - low, min=1e-6)).clamp(
                0.0, 1.0
            )
            colors = torch.round(normalized * 255.0).to(torch.uint8).cpu().numpy()
        return np.ascontiguousarray(colors, dtype=np.uint8)

    def _fit(self, device: Any) -> None:
        if self._reservoir_features.shape[0] < 6:
            raise PcaError("not enough calibration samples to fit six PCA components")
        torch = __import__("torch")
        samples = torch.from_numpy(self._reservoir_features).to(device=device)
        with torch.inference_mode():
            mean = samples.mean(dim=0)
            centered = samples - mean
            torch.manual_seed(37)
            _, _, components = torch.pca_lowrank(
                centered, q=6, center=False, niter=5
            )
            projection = centered @ components
            combined = projection[:, :3] * 0.6 + projection[:, 3:6] * 0.4
            low = torch.quantile(combined, 0.01, dim=0)
            high = torch.quantile(combined, 0.99, dim=0)
            if torch.any(high - low <= 1e-6):
                raise PcaError("PCA calibration produced a degenerate color range")
        self._mean = mean.float().cpu().numpy()
        self._components = components.float().cpu().numpy()
        self._low = low.float().cpu().numpy()
        self._high = high.float().cpu().numpy()
        self._reservoir_features = np.empty(
            (0, self.identity.feature_dim), dtype=np.float32
        )
        self._reservoir_keys = np.empty((0,), dtype=np.float64)

    def _validate_features(self, features: Any) -> None:
        if not hasattr(features, "ndim") or features.ndim != 2:
            raise PcaError("features must be a rank-two tensor")
        if features.shape[0] == 0:
            raise PcaError("features must not be empty")
        if features.shape[1] != self.identity.feature_dim:
            raise PcaError(
                f"feature dimension is {features.shape[1]}, expected "
                f"{self.identity.feature_dim}"
            )

    def _save_atomic(self) -> None:
        parent = os.path.dirname(self.profile_path) or "."
        os.makedirs(parent, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb", dir=parent, prefix=".utonia-pca-", suffix=".npz", delete=False
            ) as stream:
                temporary = stream.name
                np.savez(
                    stream,
                    schema_version=np.int64(PROFILE_SCHEMA_VERSION),
                    model_sha256=np.asarray(self.identity.model_sha256),
                    feature_dim=np.int64(self.identity.feature_dim),
                    utonia_scale=np.float64(self.identity.utonia_scale),
                    max_model_points=np.int64(self.identity.max_model_points),
                    mean=self._mean,
                    components=self._components,
                    low=self._low,
                    high=self._high,
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.profile_path)
        finally:
            if temporary is not None and os.path.exists(temporary):
                os.unlink(temporary)

    def _try_load(self) -> bool:
        if not os.path.exists(self.profile_path):
            return False
        try:
            with np.load(self.profile_path, allow_pickle=False) as profile:
                required = {
                    "schema_version",
                    "model_sha256",
                    "feature_dim",
                    "utonia_scale",
                    "max_model_points",
                    "mean",
                    "components",
                    "low",
                    "high",
                }
                if not required.issubset(profile.files):
                    raise PcaError("profile is missing required arrays")
                if int(profile["schema_version"]) != PROFILE_SCHEMA_VERSION:
                    raise PcaError("profile schema version does not match")
                if str(profile["model_sha256"]) != self.identity.model_sha256:
                    raise PcaError("profile model SHA-256 does not match")
                if int(profile["feature_dim"]) != self.identity.feature_dim:
                    raise PcaError("profile feature dimension does not match")
                if not np.isclose(
                    float(profile["utonia_scale"]), self.identity.utonia_scale
                ):
                    raise PcaError("profile Utonia scale does not match")
                if int(profile["max_model_points"]) != self.identity.max_model_points:
                    raise PcaError("profile max_model_points does not match")
                mean = np.asarray(profile["mean"], dtype=np.float32)
                components = np.asarray(profile["components"], dtype=np.float32)
                low = np.asarray(profile["low"], dtype=np.float32)
                high = np.asarray(profile["high"], dtype=np.float32)
            expected_dim = self.identity.feature_dim
            if mean.shape != (expected_dim,):
                raise PcaError("profile mean has the wrong shape")
            if components.shape != (expected_dim, 6):
                raise PcaError("profile components have the wrong shape")
            if low.shape != (3,) or high.shape != (3,):
                raise PcaError("profile color bounds have the wrong shape")
            if not all(np.isfinite(value).all() for value in (mean, components, low, high)):
                raise PcaError("profile contains NaN or Inf")
            if np.any(high - low <= 1e-6):
                raise PcaError("profile color bounds are degenerate")
            self._mean = mean
            self._components = components
            self._low = low
            self._high = high
            return True
        except Exception as exc:
            self.load_error = str(exc)
            self._mean = None
            self._components = None
            self._low = None
            self._high = None
            return False
