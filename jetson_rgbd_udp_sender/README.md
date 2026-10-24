# Jetson RGB-D UDP 发送端

ROS 2 Humble C++ 包，订阅 D435i 的 RGB、对齐深度和 CameraInfo：

```text
RGB -> Jetson nvv4l2h265enc -> UDP 5000
Depth -> Zdepth -> UDP 5001
CameraInfo -> JSON -> UDP 5002
```

默认输入为 `640x480@30`，RGB-D 近似同步窗口为 10 ms。处理跟不上时覆盖旧帧，
避免累积延迟。H.265 使用硬件编码、GOP 10、无 B 帧、插入 VPS/SPS/PPS；Zdepth
使用 `kNotQuantized8191mm` 且每帧独立编码。

Zdepth 固定源码已包含在 `third_party/Zdepth`，不需要联网获取。依赖检查脚本：

```bash
source /opt/ros/humble/setup.bash
bash scripts/check_jetson_deps.sh
```

本包只能在 Jetson 上构建：

```bash
mkdir -p ~/rgbd_ws/src
cp -a jetson_rgbd_udp_sender ~/rgbd_ws/src/
cd ~/rgbd_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select jetson_rgbd_udp_sender \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
ros2 launch jetson_rgbd_udp_sender sender.launch.py receiver_ip:=192.168.1.107
```

完整依赖、RealSense 启动方法和 PC 配套步骤见仓库顶层 `README.md`。

