# RGB-D UDP 与 Utonia PCA 系统设计

## 数据链路

```text
Jetson Orin NX
D435i RGB + aligned depth + CameraInfo
  -> ApproximateTime 同步（10 ms）
  -> bundle_sequence
  -> RGB: NVENC H.265 -> UDP 5000
  -> Depth: Zdepth -> UDP 5001
  -> Calibration JSON -> UDP 5002

PC
UDP 校验与有界重组
  -> H.265/PyAV 解码 + Zdepth/C++ 解码
  -> 按 bundle_sequence 配对
  -> 相机内参反投影为 XYZRGB
  -> /rgbd/color_points
  -> Utonia Tiny（每点 432 维特征）
  -> 固定 PCA 映射为 RGB
  -> /utonia/pca_points
  -> RViz2
```

## Jetson 端

RGB 经以下 GStreamer 管线进入 Jetson 硬件编码器：

```text
appsrc RGB/BGR
 -> videoconvert/I420
 -> nvvidconv/NVMM/NV12
 -> nvv4l2h265enc
 -> h265parse(byte-stream, AU aligned)
 -> appsink
```

当前配置为 `640x480@30`、3 Mbps、GOP/IDR 10、无 B 帧。每个输出 Access Unit
对应一个 RGB 帧；关键帧带参数集，丢包后可在后续 IDR 恢复。

深度是 `16UC1` 毫米图，使用 Zdepth `kNotQuantized8191mm`。`1..8190 mm`
整数值保持不变，0 和超出范围值作为无效深度。每帧独立编码，丢失一帧不会影响
下一帧解码。

所有线程间缓存都是容量 1 的 latest-value 槽，过载时覆盖旧帧而不是积累延迟。

## UDP 协议

每个应用层 UDP 数据报最多 1200 字节，其中固定协议头为 56 字节。协议头包含：

- magic/version/stream type/flags
- source ID 和 calibration ID
- bundle sequence
- 原始 ROS 时间戳和发送时间戳
- 完整帧长度和 CRC32
- 分片序号、总分片数和当前载荷长度

协议不提供 ACK、重传、FEC、加密或拥塞控制。任意分片丢失会导致整帧丢弃；
接收器通过 100 ms 超时、帧数上限、总内存上限和 CRC32 防止无限等待与内存增长。

## PC 解码和点云重建

接收器使用 PyAV/FFmpeg 软件解码 H.265，使用 pybind11 包装的 Zdepth C++
解码深度。两路数据必须具有一致的 source ID、bundle sequence 和 calibration ID，
且原始时间戳差不超过 10 ms。

深度已对齐到彩色相机，按同一像素取 RGB，并用彩色相机内参反投影：

```text
z = depth_mm * 0.001
x = (u - cx) * z / fx
y = (v - cy) * z / fy
```

有效点最多均匀选取 50,000 个，以 `PointCloud2` 的 `x/y/z/rgb` 字段发布。

## Utonia 和固定 PCA

推理进程与接收器使用不同 Python 环境，通过本机 ROS 2 话题连接。D435i 光学
坐标 `(x,y,z)` 转为模型使用的 `(x,z,-y)`，再经过 Utonia 官方变换与 Tiny 模型。
模型约 9.33M 参数，当前产生每点 432 维特征。

第一次运行收集 20 个推理帧，拟合固定 PCA 基底和颜色分位范围。之后每帧使用
同一基底进行 432 维到 RGB 的映射，避免逐帧 PCA 带来的主成分换序和符号翻转。
输出颜色反映特征相似性，不代表带名称的语义类别。

## 实时策略与边界

- 输入、解码、配对、发布和推理都优先处理最新帧。
- H.265 丢失后等待下一 IDR；Zdepth 下一完整帧立即恢复。
- 当前没有网络可靠性、认证、加密和自适应码率。
- PC H.265 解码当前使用 CPU；RTX 4060 主要用于 Utonia 推理。
- 跨机端到端性能必须在 Jetson、Mesh 和目标 PC 上实测。

