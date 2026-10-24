#!/usr/bin/env bash
set -u

missing=0

say() {
  printf '%s\n' "$*"
}

say "== Jetson RGB-D sender dependency probe =="
say "ROS_DISTRO=${ROS_DISTRO:-<not sourced>}"
say "Architecture=$(uname -m 2>/dev/null || printf '<unknown>')"

if [[ -r /etc/nv_tegra_release ]]; then
  say "L4T=$(head -n 1 /etc/nv_tegra_release)"
else
  say "L4T=<not a detected Jetson Linux rootfs>"
  missing=$((missing + 1))
fi

for command_name in cmake g++ git pkg-config gst-inspect-1.0; do
  if command -v "${command_name}" >/dev/null 2>&1; then
    say "[ok] command ${command_name}: $(command -v "${command_name}")"
  else
    say "[missing] command ${command_name}"
    missing=$((missing + 1))
  fi
done

for module_name in gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0; do
  if pkg-config --exists "${module_name}" 2>/dev/null; then
    say "[ok] pkg-config ${module_name}: $(pkg-config --modversion "${module_name}")"
  else
    say "[missing] pkg-config ${module_name}"
    missing=$((missing + 1))
  fi
done

for element_name in appsrc appsink videoconvert nvvidconv nvv4l2h265enc h265parse; do
  if gst-inspect-1.0 "${element_name}" >/dev/null 2>&1; then
    say "[ok] GStreamer element ${element_name}"
  else
    say "[missing] GStreamer element ${element_name}"
    missing=$((missing + 1))
  fi
done

packages=(
  build-essential
  cmake
  git
  pkg-config
  gstreamer1.0-tools
  libgstreamer1.0-dev
  libgstreamer-plugins-base1.0-dev
  nlohmann-json3-dev
  zlib1g-dev
  ros-humble-rclcpp
  ros-humble-sensor-msgs
  ros-humble-message-filters
)

for package_name in "${packages[@]}"; do
  if dpkg-query -W -f='${Status}' "${package_name}" 2>/dev/null | grep -q '^install ok installed$'; then
    say "[ok] package ${package_name}"
  else
    say "[missing] package ${package_name}"
    missing=$((missing + 1))
  fi
done

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  say "[missing] source ROS 2 Humble before building: source /opt/ros/humble/setup.bash"
  missing=$((missing + 1))
fi

if (( missing > 0 )); then
  say "Probe failed: ${missing} required item(s) missing. This script changed nothing."
  exit 1
fi

say "Probe passed. This script changed nothing."
