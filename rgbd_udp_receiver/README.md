# PC RGB-D UDP 接收端

ROS 2 Humble 包，完成三路 UDP 重组、CRC32 校验、PyAV/FFmpeg H.265 解码、
Zdepth 原生解码、RGB-D 配对和 XYZRGB 点云重建。融合模式把 NumPy 数组直接交给
Utonia 推理，不发布中间 ROS 话题；旧双进程模式仍可发布：

```text
/rgbd/color_points
```

可选 `preview_fps` 参数默认是 0；设置为 5 时，以最多 5 Hz 在 PC 发布
`/rgbd/color/image_raw`、`/rgbd/depth/image_raw` 和 `/rgbd/depth/preview`，
分别供查看彩色图、毫米深度数据和灰度深度预览。
融合模式也可通过 `gui_fps:=30` 直接在 PC 打开 RGB 与深度图窗口；此模式只读取
最新配对帧，不需要发布 ROS 图像话题，但运行环境需要带 GUI 的 OpenCV。

构建时使用系统 Python 3.10，不要在 Conda 环境中构建。依赖检查：

```bash
conda deactivate 2>/dev/null || true
hash -r
source /opt/ros/humble/setup.bash
bash scripts/check_receiver_deps.sh
```

本包与 `utonia_pca_inference` 共用仓库根目录的 `build/install/log`，统一构建和
运行命令见顶层 `README.md`。Zdepth 固定源码已包含在 `third_party/Zdepth`。

