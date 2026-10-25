# Jetson RGB-D UDP 传输与 Utonia PCA 可视化

本项目在 Jetson Orin NX 上采集 Intel RealSense D435i 的 RGB 和对齐深度图，
将 RGB 编码为 H.265、深度编码为 Zdepth，通过自定义 UDP 协议发送到配有
RTX 4060 的电脑。电脑端解码并重建彩色点云，再用 Utonia Tiny 提取逐点特征，
通过固定 PCA 映射为伪语义颜色并发布给 RViz2。

跨机器链路不使用 ROS 2 DDS；PC 端解码、点云重建和推理在同一进程中传递
NumPy 数组，仅将最终 PCA 点云发布为 ROS 2 话题。

```text
D435i ROS 2
  -> RGB/H.265 + Depth/Zdepth + CameraInfo
  -> UDP 5000/5001/5002
  -> PC 解码与 XYZRGB 重建（进程内 NumPy 数组，最多 20000 点）
  -> Utonia Tiny + 固定 PCA
  -> /utonia/pca_points
  -> RViz2
```

详细协议与数据流程见 [docs/system-design.md](docs/system-design.md)。第三方组件和
模型许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 1. 目录结构

```text
jetson_rgbd_udp_sender/  Jetson C++ 采集、H.265/Zdepth 压缩和 UDP 发送
rgbd_udp_receiver/      PC UDP 重组、解码和点云重建
utonia_pca_inference/   PC 单进程入口、Utonia Tiny、固定 PCA 和 ROS 2 发布
docs/                   系统设计
```

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

## 3. 安装 ROS 2 Humble

Jetson 和 PC 都使用 Ubuntu 22.04，可通过小鱼一键安装工具安装 ROS 2：

```bash
sudo apt update
sudo apt install -y wget
source <(wget -qO- https://fishros.com/install)
```

在交互菜单中选择“一键安装 ROS”，安装 ROS 2 Humble。Jetson 可选择基础版，PC
需要运行 RViz2，应选择桌面版。安装完成后，两台设备都安装开发工具：

```bash
sudo apt update
sudo apt install -y ros-dev-tools
```

安装后在每个新终端加载 ROS 2：

```bash
source /opt/ros/humble/setup.bash
```

安装工具主页：<https://fishros.com/page/>；源码：
<https://github.com/fishros/install>。

### 3.1 依赖检查脚本

三个脚本都只读取环境，不会自动安装或修改系统：

```text
jetson_rgbd_udp_sender/scripts/check_jetson_deps.sh
rgbd_udp_receiver/scripts/check_receiver_deps.sh
utonia_pca_inference/scripts/check_utonia_deps.sh
```

先运行对应脚本，再安装它明确报告缺失的项目。

## 4. Jetson 端准备与编译

本节只在 Jetson 上执行。不要在 x86 PC 上构建发送端。

### 4.1 安装 RealSense、GStreamer 和编译依赖

安装 RealSense SDK 的 ROS 二进制包及 ROS 2 相机驱动：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-librealsense2 \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description
```

RealSense 官方要求 SDK 与 ROS Wrapper 只选择一种安装来源，避免同时安装源码版和
二进制版造成冲突。本项目使用上面的 ROS 软件源二进制包。官方说明：
<https://github.com/realsenseai/realsense-ros#installation-on-ubuntu>。

安装 NVIDIA 硬件编码组件、GStreamer 运行库及发送端编译依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  nvidia-l4t-gstreamer nlohmann-json3-dev zlib1g-dev \
  ros-humble-ament-cmake ros-humble-rclcpp \
  ros-humble-sensor-msgs ros-humble-message-filters \
  ros-humble-ros2launch ros-humble-launch-ros \
  ros-humble-ament-index-python
```

`nvvidconv` 和 `nvv4l2h265enc` 由与当前 L4T 版本匹配的
`nvidia-l4t-gstreamer` 提供。不要混装其他 JetPack/L4T 版本的软件源。NVIDIA
安装说明：<https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html>。

Zdepth 及其使用的 Zstd 源码已经包含在包内，不需要另外安装。

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

ros2 launch jetson_rgbd_udp_sender sender.launch.py
```

默认端口是 H.265 `5000`、Zdepth `5001`、标定 `5002`。当前 H.265 目标码率为
3 Mbps，目标 IP 为 `192.168.1.107`，配置见
`jetson_rgbd_udp_sender/config/sender.yaml`。如果 Jetson 上的接收 IP 不同，修改
Jetson 工作区中的 `sender.yaml`，然后启动时用 `config_file` 指向该文件；
`sender.launch.py` 不接受 `receiver_ip` 参数。

## 5. PC 接收端依赖与统一编译

PC 上构建 `rgbd_udp_receiver` 和 `utonia_pca_inference` 时必须先退出 Conda，
确保 CMake 使用 ROS 2 Humble 对应的 `/usr/bin/python3`。

### 5.1 NVIDIA 驱动和系统依赖

PC 需要可正常驱动 RTX 显卡。若 `nvidia-smi` 还不能运行，可使用 Ubuntu 推荐的
专有驱动；安装完成后重启：

```bash
sudo apt update
sudo apt install -y ubuntu-drivers-common
sudo ubuntu-drivers install
sudo reboot
```

重启后先确认 `nvidia-smi` 能识别显卡，再安装编译和 ROS 2 Python 依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git wget \
  python3-dev python3-numpy python3-pip pybind11-dev \
  python3-colcon-common-extensions \
  ros-humble-ament-cmake ros-humble-ament-cmake-python \
  ros-humble-rclpy ros-humble-sensor-msgs \
  ros-humble-std-msgs ros-humble-builtin-interfaces \
  ros-humble-launch-ros
```

旧双进程模式需要接收端在系统 Python 中运行，可单独为 `/usr/bin/python3`
安装 PyAV；融合模式则只需在 `utonia` Conda 环境中安装 PyAV：

```bash
/usr/bin/python3 -m pip install --user av==16.0.1
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

Utonia 源码与权重不在本仓库中。使用已有的 Conda 创建独立推理环境，并安装本项目
实际使用的 Utonia 依赖：

```bash
conda create -n utonia python=3.10 -y
conda activate utonia

python -m pip install \
  torch==2.5.0 torchvision==0.20.0 \
  --index-url https://download.pytorch.org/whl/cu124

python -m pip install \
  numpy==1.26.4 av==16.0.1 spconv-cu124 timm huggingface_hub scipy addict

python -m pip install --no-index torch-scatter \
  -f https://data.pyg.org/whl/torch-2.5.0+cu124.html
```

这里使用 PyTorch 自带的 CUDA 12.4 运行库，不要求另外安装完整 CUDA Toolkit，
但 NVIDIA 驱动必须可用。本项目关闭 FlashAttention，因此无需安装 `flash-attn`；
Open3D、Gradio 等仅用于 Utonia 官方 Demo 的包也不是本项目依赖。
如果需要 PC 本地图像弹窗，可在 `utonia` 环境额外安装带 GUI 的 OpenCV：

```bash
python -m pip install --no-deps opencv-python==4.11.0.86
```

本地弹窗不开启时不需要 OpenCV。不要安装 `opencv-python-headless` 作为窗口依赖。

准备固定版本 Utonia：

```bash
mkdir -p ~/utonia_ws
git clone https://github.com/Pointcept/Utonia.git ~/utonia_ws/Utonia
git -C ~/utonia_ws/Utonia checkout de1f9deef4ee65e508d74347f8d0eb848d878abb

mkdir -p ~/utonia_ws/Utonia/ckpt
wget -c -O ~/utonia_ws/Utonia/ckpt/utonia_tiny.pth \
  https://huggingface.co/Pointcept/Utonia/resolve/main/utonia_tiny.pth
```

Utonia 官方安装说明：<https://github.com/Pointcept/Utonia#installation>。

若源码或权重放在其他位置，修改
`utonia_pca_inference/config/inference.yaml` 中的路径。

在已加载 ROS 2 工作区的 `utonia` 环境中检查推理依赖：

```bash
bash utonia_pca_inference/scripts/check_utonia_deps.sh
```

## 7. PC 端运行

PC 上使用一个融合进程完成 UDP 接收、解码、点云重建、Utonia 推理和 PCA 发布。
该进程使用 `utonia` Conda 环境，且必须能够导入 ROS 2 Humble 的 Python 包、
PyAV 和本项目编译的 `rgbd_udp_receiver._zdepth` 扩展。启动前加载 ROS 2 和
本工作区的 `install/setup.bash`；RViz2 在 ROS 2 Humble 环境中运行。

| 进程 | 使用环境 |
|---|---|
| `rgbd_utonia_node` | 安装了 PyAV、PyTorch 和 Utonia 依赖的 `utonia` Conda 环境，同时可访问 ROS 2 和 Zdepth 扩展 |
| RViz2 | ROS 2 Humble 环境 |

### 终端 A：接收、解码和推理

```bash
ros2 launch utonia_pca_inference fused.launch.py \
  bind_address:=0.0.0.0 \
  h265_port:=5000 \
  zdepth_port:=5001 \
  calibration_port:=5002
```

需要直接在 PC 弹窗查看 RGB 与深度图时，给上述命令加 `gui_fps:=30`。
窗口由主线程处理，ROS 回调和推理在后台运行；按 `q` 或 Esc 关闭窗口并停止程序。
弹窗只读取最新配对帧，不构造中间 ROS 图像消息，日志中的 `gui fps/ms p95`
记录实际显示帧率和画面转换、提交到窗口的耗时。`gui_fps` 是显示频率上限，
实际帧率取决于收到的配对帧和电脑负载；默认 `gui_fps:=0` 不打开窗口。

如需在 RViz2 中查看接收到的图像，也可以给命令加 `preview_fps:=5`。
默认 `preview_fps:=0` 不发布图像话题，以免影响吞吐量测量。
预览使用已经配对的 RGB-D 帧，只保留最新帧；可在 RViz2 添加 `Image` 显示：

| 话题 | 格式 | 用途 |
|---|---|---|
| `/rgbd/color/image_raw` | `rgb8` | 接收到的彩色图 |
| `/rgbd/depth/image_raw` | `16UC1` | 原始毫米深度值 |
| `/rgbd/depth/preview` | `mono8` | 便于查看的灰度深度图，近处亮、远处暗、无效值为黑色 |

RViz2 图像显示使用 `Best Effort` 可靠性。ROS 图像预览会额外构造和发布图像消息，
其耗时单独打印为 `preview ms p95`；评估推理帧率时建议关闭图像预览和本地弹窗。

第一次运行会收集 20 个成功推理帧拟合固定 PCA，然后保存 profile。后续启动会
复用该 profile；模型哈希、特征维度或关键参数不匹配时会重新校准。
融合模式不会发布 `/rgbd/color_points`。旧的 `receiver.launch.py` 与
`inference.launch.py` 仍可分别启动，用于对照原双进程流程；不要同时运行两套
接收端，否则 UDP 端口会冲突。

### 终端 B：RViz2

```bash
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

实机环境为 Jetson Orin NX 16 GB、D435i、无线 Mesh 网络和 RTX 4060 Laptop。
下面的数据来自优化前的双进程实机记录。输入为 `640x480@30`，H.265 目标码率为
3 Mbps；旧接收端最多重建 50000 点，Utonia Tiny 最多输入 20000 点。接收端与
推理端数据来自不同采样窗口，结果用于展示旧版运行情况，不作为严格同步的端到端
基准测试。融合模式默认最多重建 20000 点，实测结果见下文。
表中的 FPS 表示各环节每秒处理或发布的帧数（吞吐量），不表示单帧耗时或端到端延迟。

| 环节 | 实测结果 |
|---|---|
| Jetson RGB-D 同步 | 约 30 FPS |
| H.265 硬件编码 | 目标码率 3 Mbps |
| Zdepth 编码 | 约 24～28 Mbps，约 15～16 ms/帧 |
| H.265 解码及 RGB24 转换 | 约 14.0 FPS |
| Zdepth 解码 | 约 19.4 FPS |
| 50000 点重建与发布 | 约 12.1 FPS |
| Utonia Tiny 推理与 PCA 发布 | 全窗口约 4.1 FPS；输入连续时约 9～10 FPS |
| RViz2 可视化 | 成功显示 `/utonia/pca_points` 伪彩色点云 |

采样期间未出现 H.265/Zdepth 解码错误、CRC 错误、RGB-D 配对错误或 CUDA OOM。
帧率波动主要来自无线 UDP 丢包与抖动、点云重建和 Utonia 推理开销；系统采用
“只保留最新帧”策略丢弃来不及处理的旧帧，以优先保证实时可视化的低延迟。

融合模式在 2026-10-25 的一次实机运行中，关闭图像预览、重建 20000 点。日志
最后 20 秒内，接收端平均输出约 27.7 FPS，PCA 点云平均发布约 9.2 FPS；
H.265 解码、RGB 转换、Zdepth 解码、XYZ 重建的 p95 分别约为
4.2、0.2、2.3、2.6 ms。推理平均约 97.8 ms，最终点云构造和发布平均约
10.5 ms。该窗口没有解码、CRC 或推理错误；当前最终帧率主要受推理与发布耗时
限制。这些是各环节统计，仍不是端到端延迟测量。

### 可改进方向

- 使用有线网络进行对照测试，并进一步优化 UDP 缓冲、超时和丢包恢复策略。
- 测量融合模式的端到端延迟，并继续优化模型推理与最终点云发布。
- 尝试 NVDEC 硬件解码、混合精度推理和 FlashAttention，进一步提高处理帧率。

## 9. 上传 Git 前检查

本项目自有代码采用 MIT License，正文见 [LICENSE](LICENSE)。Zdepth 与 Utonia
仍分别遵循各自的第三方许可证，详见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

确认 `build/`、`install/`、`log/`、模型权重、PCA profile 和 Python 缓存均未进入
暂存区后再提交。

