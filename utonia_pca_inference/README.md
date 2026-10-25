# Utonia Tiny 固定 PCA 推理节点

推荐使用 `fused.launch.py`：与 UDP 接收、解码和点云重建运行在同一 Python 进程，
直接接收 NumPy 点云数组，只保留最新帧，在 RTX GPU 上执行 Utonia Tiny
特征提取和固定 PCA 上色，发布：

```text
/utonia/pca_points
```

旧的 `inference.launch.py` 仍可订阅 `/rgbd/color_points`，用于双进程对照。
融合启动可加 `gui_fps:=10`，直接在 PC 打开 RGB 与深度图窗口；这一模式需要在
Conda 环境安装带 GUI 的 `opencv-python`，但不发布 ROS 图像话题。按 `q` 或 Esc
退出窗口及融合进程。
节点运行在 Python 3.10 Conda 环境中。融合模式还需要在该环境安装 PyAV 16.0.1，
并能导入本工作区编译的 Zdepth 扩展。Utonia 环境使用 PyTorch 2.5.0+cu124、
torchvision 0.20.0+cu124、torch-scatter 2.1.2+pt25cu124 和 spconv-cu124。

Utonia 源码、Tiny 权重和 PCA profile 不包含在仓库中。默认路径位于
`config/inference.yaml`，可按本机目录修改。激活 Conda 环境并加载 ROS 2 与
本工作区后，检查依赖：

```bash
bash utonia_pca_inference/scripts/check_utonia_deps.sh
```

完整安装、统一构建、启动和 RViz2 设置见仓库顶层 `README.md`。

