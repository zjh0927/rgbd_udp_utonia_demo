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
  -> 进程内 NumPy XYZRGB
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

D435i 的对齐深度图采用 ROS 2 `16UC1` 编码，每个像素以 16 位无符号整数表示毫米距离。发送端使用 Zdepth 的 `kNotQuantized8191mm` 模式：`1～8190 mm` 范围内的深度值无损保留，`0` 或不在该范围内的值均作为无效深度处理。每张深度图独立压缩，因此某一帧丢失不会影响后续完整帧的解码。

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

融合模式最多均匀选取 20,000 个有效点，在进程内把 NumPy 的 XYZ 和 RGB 数组
交给推理线程。旧双进程模式仍可将点云打包成 `/rgbd/color_points`，供对照使用。

## Utonia 和固定 PCA

融合模式在同一个 Python 进程中完成接收和推理，仅最终结果通过 ROS 2 发布。
D435i 光学坐标 `(x,y,z)` 转为模型使用的 `(x,z,-y)`，再经过 Utonia 官方变换与
Tiny 模型。模型约 9.33M 参数，当前产生每点 432 维特征。

第一次运行收集 20 个推理帧，拟合固定 PCA 基底和颜色分位范围。之后每帧使用
同一基底进行 432 维到 RGB 的映射，避免逐帧 PCA 带来的主成分换序和符号翻转。
输出颜色反映特征相似性，不代表带名称的语义类别。

## 实时策略与边界

- 输入、解码、配对、发布和推理都优先处理最新帧。
- H.265 丢失后等待下一 IDR；Zdepth 下一完整帧立即恢复。
- PC H.265 解码当前使用 CPU软件解码；RTX 4060 主要用于 Utonia 模型推理。
- H.265 解码后仍需提取 RGB 颜色作为模型输入；目前保留整帧 RGB24 转换，并
  分别记录视频解码和颜色转换耗时。
- 融合模式的性能需要实机重测；旧双进程数据不代表融合模式的帧率。


