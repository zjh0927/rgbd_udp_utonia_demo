# PC RGB-D UDP 接收端

ROS 2 Humble 包，完成三路 UDP 重组、CRC32 校验、PyAV/FFmpeg H.265 解码、
Zdepth 原生解码、RGB-D 配对和 XYZRGB 点云重建，发布：

```text
/rgbd/color_points
```

接收端使用系统 Python 3.10，不要在 Conda 环境中构建。依赖检查：

```bash
conda deactivate 2>/dev/null || true
hash -r
source /opt/ros/humble/setup.bash
bash scripts/check_receiver_deps.sh
```

本包与 `utonia_pca_inference` 共用仓库根目录的 `build/install/log`，统一构建和
运行命令见顶层 `README.md`。Zdepth 固定源码已包含在 `third_party/Zdepth`。

