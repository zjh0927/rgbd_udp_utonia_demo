# Jetson RGB-D UDP 传输与 Utonia PCA 可视化

本项目在 Jetson Orin NX 上采集 Intel RealSense D435i 的 RGB 和对齐深度图，
将 RGB 编码为 H.265、深度编码为 Zdepth，通过自定义 UDP 协议发送到配有
RTX 4060 的电脑。电脑端解码并重建彩色点云，再用 Utonia Tiny 提取逐点特征，
通过固定 PCA 映射为伪语义颜色并发布给 RViz2。

跨机器链路不使用 ROS 2 DDS；ROS 2 只用于两台机器各自的本地进程通信。

```text
D435i ROS 2
  -> RGB/H.265 + Depth/Zdepth + CameraInfo
  -> UDP 5000/5001/5002
  -> PC 解码与 XYZRGB 重建
  -> /rgbd/color_points
  -> Utonia Tiny + 固定 PCA
  -> /utonia/pca_points
  -> RViz2
```

详细协议与数据流程见 [docs/system-design.md](docs/system-design.md)。第三方组件和
模型许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 1. 目录结构

```text
jetson_rgbd_udp_sender/  Jetson C++ 采集、H.265/Zdepth 压缩和 UDP 发送
rgbd_udp_receiver/      PC UDP 重组、解码、点云重建和 ROS 2 发布
utonia_pca_inference/   PC Utonia Tiny 推理、固定 PCA 和 ROS 2 发布
docs/                   系统设计
```

`build/`、`install/` 和 `log/` 是本机构建产物，已由 `.gitignore` 排除，不能上传
Git。模型权重和 PCA profile 也不上传。

## 2. 已验证环境

### Jetson

- Jetson Orin NX 16 GB
- NVIDIA L4T R36.4.7，aarch64
- Ubuntu 22.04 / ROS 2 Humble
- GStreamer 1.20
- `nvv4l2h265enc` 与 `nvvidconv`
- RealSense D435i 与 `realsense2_camera`

### PC

- Ubuntu 22.04 / ROS 2 Humble / 系统 Python 3.10
- NVIDIA RTX 4060 Laptop，驱动 570.133.07
- PyAV 16.0.1、NumPy 1.26.x
- Conda 环境 `utonia`：Python 3.10、PyTorch 2.5.0+cu124、
  torchvision 0.20.0+cu124、torch-scatter 2.1.2+pt25cu124、spconv-cu124
- Utonia Tiny，FlashAttention 关闭

## 3. 依赖检查脚本

三个脚本都只读取环境，不会自动安装或修改系统：

```text
jetson_rgbd_udp_sender/scripts/check_jetson_deps.sh
rgbd_udp_receiver/scripts/check_receiver_deps.sh
utonia_pca_inference/scripts/check_utonia_deps.sh
```

先运行对应脚本，再安装它明确报告缺失的项目。

## 4. Jetson 端准备与编译

本节只在 Jetson 上执行。不要在 x86 PC 上构建发送端。

### 4.1 安装 ROS 2 和编译依赖

系统需要已有 ROS 2 Humble 和 RealSense ROS 2 包。发送端的普通开发依赖为：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  gstreamer1.0-tools \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  nlohmann-json3-dev zlib1g-dev \
  ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-message-filters
```

`nvvidconv` 和 `nvv4l2h265enc` 来自 Jetson BSP 多媒体组件；不要为了本项目盲目
升级 L4T、刷机或安装整个 JetPack 元包。

Zdepth 源码已经包含在包内，构建过程不访问网络。

### 4.2 复制和检查

```bash
mkdir -p ~/rgbd_ws/src
cp -a jetson_rgbd_udp_sender ~/rgbd_ws/src/

source /opt/ros/humble/setup.bash
bash ~/rgbd_ws/src/jetson_rgbd_udp_sender/scripts/check_jetson_deps.sh
```

### 4.3 编译

```bash
cd ~/rgbd_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select jetson_rgbd_udp_sender \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF

source ~/rgbd_ws/install/setup.bash
```

### 4.4 启动 D435i 和发送端

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30 \
  align_depth.enable:=true \
  enable_sync:=true \
  pointcloud.enable:=false
```

另开 Jetson 终端：

```bash
source /opt/ros/humble/setup.bash
source ~/rgbd_ws/install/setup.bash

ros2 launch jetson_rgbd_udp_sender sender.launch.py \
  receiver_ip:=192.168.1.107
```

默认端口是 H.265 `5000`、Zdepth `5001`、标定 `5002`。当前 H.265 目标码率为
3 Mbps，配置见 `jetson_rgbd_udp_sender/config/sender.yaml`。

## 5. PC 接收端依赖与统一编译

PC 上构建 `rgbd_udp_receiver` 和 `utonia_pca_inference` 时必须先退出 Conda，
确保 CMake 使用 ROS 2 Humble 对应的 `/usr/bin/python3`。

### 5.1 系统依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-dev python3-numpy python3-pip pybind11-dev \
  python3-colcon-common-extensions \
  ros-humble-ament-cmake ros-humble-ament-cmake-python \
  ros-humble-rclpy ros-humble-sensor-msgs
```

确认 PyAV；缺失时再安装：

```bash
/usr/bin/python3 -m pip install av==16.0.1
```

检查：

```bash
conda deactivate 2>/dev/null || true
hash -r
source /opt/ros/humble/setup.bash

bash rgbd_udp_receiver/scripts/check_receiver_deps.sh
```

### 5.2 两个 PC 包共用 build/install/log

从本仓库根目录执行：

```bash
conda deactivate 2>/dev/null || true
hash -r
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash

colcon --log-base log build \
  --base-paths rgbd_udp_receiver utonia_pca_inference \
  --build-base build \
  --install-base install \
  --packages-select rgbd_udp_receiver utonia_pca_inference \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

不要移动或拼接其他工作区已有的 build/install 目录；CMake 缓存包含绝对路径，
必须在本目录重新构建。

## 6. Utonia 环境和权重

Utonia 源码与权重不在本仓库中。下面是已验证组合：

```bash
conda create -n utonia python=3.10 -y
conda activate utonia

python -m pip install \
  torch==2.5.0 torchvision==0.20.0 \
  --index-url https://download.pytorch.org/whl/cu124

python -m pip install \
  numpy==1.26.4 spconv-cu124 timm huggingface_hub scipy addict

python -m pip install --no-index torch-scatter \
  -f https://data.pyg.org/whl/torch-2.5.0+cu124.html
```

准备固定版本 Utonia：

```bash
mkdir -p ~/utonia_ws
git clone https://github.com/Pointcept/Utonia.git ~/utonia_ws/Utonia
git -C ~/utonia_ws/Utonia checkout de1f9deef4ee65e508d74347f8d0eb848d878abb

mkdir -p ~/utonia_ws/Utonia/ckpt
wget -c -O ~/utonia_ws/Utonia/ckpt/utonia_tiny.pth \
  https://huggingface.co/Pointcept/Utonia/resolve/main/utonia_tiny.pth
```

若源码或权重放在其他位置，修改
`utonia_pca_inference/config/inference.yaml` 中的路径。

检查 Conda 推理环境：

```bash
conda activate utonia
unset LD_LIBRARY_PATH PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$HOME/utonia_ws/Utonia:${PYTHONPATH:-}"

bash utonia_pca_inference/scripts/check_utonia_deps.sh
```

## 7. PC 端运行

### 终端 A：UDP 接收和点云重建

```bash
conda deactivate 2>/dev/null || true
hash -r
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rgbd_udp_receiver receiver.launch.py \
  bind_address:=0.0.0.0 \
  h265_port:=5000 \
  zdepth_port:=5001 \
  calibration_port:=5002
```

确认普通彩色点云：

```bash
ros2 topic hz /rgbd/color_points
```

### 终端 B：Utonia Tiny 和固定 PCA

```bash
conda activate utonia
unset LD_LIBRARY_PATH PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$HOME/utonia_ws/Utonia:${PYTHONPATH:-}"

ros2 launch utonia_pca_inference inference.launch.py
```

第一次运行会收集 20 个成功推理帧拟合固定 PCA，然后保存 profile。后续启动会
复用该 profile；模型哈希、特征维度或关键参数不匹配时会重新校准。

### 终端 C：RViz2

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2
```

添加 `PointCloud2`：

- Topic：`/utonia/pca_points`
- Reliability：`Best Effort`
- Position Transformer：`XYZ`
- Color Transformer：`RGB8` 或 `RGB`
- Fixed Frame：消息中的 `frame_id`，通常为 `camera_color_optical_frame`
- Decay Time：`0`

PCA 颜色表示预训练特征空间，不是带类别名称的语义分割结果。

## 8. 已完成的实机验证

下面是开发阶段使用 5 Mbps H.265 配置得到的结果；当前 3 Mbps 配置需要重新记录：

| 环节 | 已观察结果 |
|---|---:|
| Jetson RGB-D 同步 | 约 29～30 FPS |
| H.265 | 约 5 Mbps |
| Zdepth | 约 24～28 Mbps，编码约 15～16 ms/帧 |
| `/rgbd/color_points` | 约 7～11 Hz，网络状态相关 |
| Utonia Tiny | 约 36k～38k 点，约 5～6 Hz |
| RViz2 | 已成功显示固定 PCA 伪彩色点云 |

不要把原始编译日志提交到 Git。新的实测结果应写明硬件、参数、场景、采样时间和
统计口径后，以表格形式更新本节。

## 9. 上传 Git 前检查

本项目自有代码采用 MIT License，正文见 [LICENSE](LICENSE)。Zdepth 与 Utonia
仍分别遵循各自的第三方许可证，详见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

```bash
find . -type d \( -name build -o -name install -o -name log -o -name __pycache__ \)
find . -type f \( -name '*.pyc' -o -name '*.log' -o -name '*.pth' -o -name '*.npz' \)

git init
git status --ignored
git add .
git status
```

确认 `build/`、`install/`、`log/`、模型权重、PCA profile 和 Python 缓存均未进入
暂存区后再提交。

