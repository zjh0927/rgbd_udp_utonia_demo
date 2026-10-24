# 第三方软件与模型说明

本仓库自有代码采用 MIT License，见顶层 `LICENSE`。以下第三方组件不因本项目
采用 MIT 而改变其原有许可条款。

## Zdepth

本项目在 Jetson 发送端和 PC 接收端中随源码包含 `catid/Zdepth`，固定来源提交为：

```text
abc01e77ed5a905fbd87c2f011cfcf50653cfbda
```

上游项目：https://github.com/catid/Zdepth

Zdepth 采用 BSD 3-Clause License。完整许可证分别保留在两个包的
`third_party/Zdepth/LICENSE` 中。发布副本移除了上游 Git 元数据和测试程序，
没有改变压缩、解压源码及其许可证。

## Utonia

Utonia 源码和模型权重不包含在本仓库中。推理节点按配置引用用户单独下载的
Utonia 源码和 `utonia_tiny.pth`：

- 源码提交：`de1f9deef4ee65e508d74347f8d0eb848d878abb`
- Utonia 源码许可：Apache License 2.0
- Tiny 权重来源：Pointcept/Utonia
- Tiny 权重许可：CC-BY-NC 4.0，仅可在许可允许的非商业场景中使用

使用者有责任在下载、发布或演示模型权重前核对上游最新许可条款。

