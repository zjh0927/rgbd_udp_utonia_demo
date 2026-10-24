# Utonia Tiny 固定 PCA 推理节点

订阅 `/rgbd/color_points`，只保留最新点云，在 RTX GPU 上执行 Utonia Tiny
特征提取和固定 PCA 上色，发布：

```text
/utonia/pca_points
```

节点运行在 Python 3.10 Conda 环境中。已验证 PyTorch 2.5.0+cu124、
torchvision 0.20.0+cu124、torch-scatter 2.1.2+pt25cu124 和 spconv-cu124。

Utonia 源码、Tiny 权重和 PCA profile 不包含在仓库中。默认路径位于
`config/inference.yaml`，可按本机目录修改。环境检查：

```bash
conda activate utonia
unset LD_LIBRARY_PATH PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$HOME/utonia_ws/Utonia:${PYTHONPATH:-}"

bash utonia_pca_inference/scripts/check_utonia_deps.sh
```

完整安装、统一构建、启动和 RViz2 设置见仓库顶层 `README.md`。

