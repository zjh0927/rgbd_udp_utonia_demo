#!/usr/bin/env bash
set -u

failed=0
check_command() {
  if command -v "$1" >/dev/null 2>&1; then
    echo "[ok] command $1: $(command -v "$1")"
  else
    echo "[missing] command $1"
    failed=1
  fi
}

echo "== RGB-D receiver dependency probe (read-only) =="
echo "ROS_DISTRO=${ROS_DISTRO:-<not sourced>}"
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "[error] Conda is active (${CONDA_PREFIX}); run: conda deactivate"
  failed=1
fi
check_command cmake
check_command g++
check_command git
check_command python3
check_command colcon

python3 - <<'PY' || failed=1
import sys
print(f"[ok] Python: {sys.version.split()[0]}")
if sys.version_info[:2] != (3, 10):
    print("[missing] ROS 2 Humble receiver requires Python 3.10")
    raise SystemExit(1)
for name in ("numpy", "av", "rclpy", "sensor_msgs"):
    try:
        module = __import__(name)
        print(f"[ok] Python module {name}: {getattr(module, '__version__', 'available')}")
    except Exception as exc:
        print(f"[missing] Python module {name}: {exc}")
        raise SystemExit(1)
PY

for package in python3-dev pybind11-dev; do
  if dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -q 'install ok installed'; then
    echo "[ok] package ${package}"
  else
    echo "[missing] package ${package}"
    failed=1
  fi
done

exit "${failed}"
