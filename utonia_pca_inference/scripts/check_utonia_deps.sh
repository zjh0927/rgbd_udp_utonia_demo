#!/usr/bin/env bash
set -uo pipefail

EXPECTED_ENV="${EXPECTED_UTONIA_ENV:-${CONDA_PREFIX:-}}"
UTONIA_SOURCE="${UTONIA_SOURCE_PATH:-${HOME}/utonia_ws/Utonia}"
CHECKPOINT="${UTONIA_CHECKPOINT_PATH:-${HOME}/utonia_ws/Utonia/ckpt/utonia_tiny.pth}"
EXPECTED_COMMIT="${EXPECTED_UTONIA_COMMIT:-de1f9deef4ee65e508d74347f8d0eb848d878abb}"

echo "== Utonia PCA inference dependency probe (read-only) =="
echo "ROS_DISTRO=${ROS_DISTRO:-<not sourced>}"

if [[ -z "${EXPECTED_ENV}" ]]; then
  echo "Dependency probe failed: activate the target Conda environment first." >&2
  exit 1
fi

PYTHONPATH="${UTONIA_SOURCE}${PYTHONPATH:+:${PYTHONPATH}}" \
EXPECTED_ENV="${EXPECTED_ENV}" \
UTONIA_SOURCE="${UTONIA_SOURCE}" \
CHECKPOINT="${CHECKPOINT}" \
EXPECTED_COMMIT="${EXPECTED_COMMIT}" \
python3 - <<'PY'
import importlib
import os
import subprocess
import sys

errors = []
expected_env = os.path.realpath(os.environ["EXPECTED_ENV"])
actual_env = os.path.realpath(sys.prefix)
print(f"Python executable: {sys.executable}")
print(f"Python version: {sys.version.split()[0]}")
if actual_env != expected_env:
    errors.append(f"active Python prefix is {actual_env}, expected {expected_env}")

modules = (
    "numpy",
    "torch",
    "torchvision",
    "rclpy",
    "sensor_msgs",
    "spconv.pytorch",
    "torch_scatter",
    "timm",
    "huggingface_hub",
    "scipy",
    "addict",
    "utonia",
)
for name in modules:
    try:
        module = importlib.import_module(name)
        version = getattr(module, "__version__", "available")
        print(f"[ok] {name}: {version}")
    except Exception as exc:
        errors.append(f"cannot import {name}: {type(exc).__name__}: {exc}")

try:
    import torch

    print(f"Torch CUDA build: {torch.version.cuda}")
    print(f"CUDA available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"GPU: {torch.cuda.get_device_name(0)}")
    else:
        errors.append("torch.cuda.is_available() is false")
except Exception as exc:
    errors.append(f"CUDA probe failed: {type(exc).__name__}: {exc}")

source = os.environ["UTONIA_SOURCE"]
try:
    commit = subprocess.check_output(
        ["git", "-C", source, "rev-parse", "HEAD"], text=True
    ).strip()
    print(f"Utonia commit: {commit}")
    if commit != os.environ["EXPECTED_COMMIT"]:
        errors.append(
            f"Utonia commit is {commit}, expected {os.environ['EXPECTED_COMMIT']}"
        )
except Exception as exc:
    errors.append(f"cannot inspect Utonia git commit: {exc}")

checkpoint = os.environ["CHECKPOINT"]
if not os.path.isfile(checkpoint):
    errors.append(f"checkpoint not found: {checkpoint}")
else:
    size = os.path.getsize(checkpoint)
    print(f"Checkpoint: {checkpoint} ({size} bytes)")
    if size < 1024 * 1024:
        errors.append("checkpoint is unexpectedly small")

if errors:
    print("Dependency probe failed:", file=sys.stderr)
    for error in errors:
        print(f"  - {error}", file=sys.stderr)
    raise SystemExit(1)
print("Dependency probe passed.")
PY
