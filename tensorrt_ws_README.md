# Orin Nano 多模型 TensorRT 部署

`build_engine.sh`/`convert_model.py` 支持 Ultralytics 的检测、分割、分类、姿态和 OBB
`.pt` 模型，以及任意单输入 NCHW `.onnx` 模型。模型任务默认从 checkpoint 自动识别，
TensorRT 会原样保留模型的全部输出，因此 YOLO 分割模型的检测输出和 mask prototypes 都会保留。

对 `.pt` 输入，`convert_model.py` 默认先在 CPU 生成 ONNX，再释放 PyTorch 模型并
由 TensorRT 使用 GPU 构建 engine，同时保留并验证任务、类别元数据和空白帧推理结果；
这一步不能用裸 `trtexec` 替代，因为直接从 ONNX 生成的 plan 不包含 Ultralytics
的类别/任务元数据。alignment 使用的 checkpoint 必须是 `task=detect`，不能把
`yolo11n-seg` 分割 checkpoint 当成检测模型。导出前请停止 `real1.launch.py` 和
不必要的远程 IDE 后台进程；脚本会在可用内存低于 3 GiB 时警告，但不会自动杀进程。
如确实需要旧的 ONNX -> `trtexec` 流程，可显式加 `--trtexec`，但生成的 engine
需要由调用方自行提供元数据和后处理。

> TensorRT engine 与 GPU 架构、TensorRT/CUDA 版本绑定，必须在最终运行 engine 的 Orin 上构建。

## YOLO11 分割模型

如果 Orin 已安装兼容版本的 PyTorch 和 Ultralytics，可直接完成 `.pt -> ONNX -> engine`：

```bash
./build_engine.sh best.pt \
  --task detect \
  --imgsz 640 \
  --batch 1 \
  --precision fp16 \
  --engine duantou_fp16.engine \
  --force
```

通常更稳妥的流程是在训练机只导出 ONNX，然后复制到 Orin 构建 engine：

```bash
# 训练机（需要 PyTorch、Ultralytics 和 ONNX 导出依赖）
./build_engine.sh ../yolo11n-seg.pt --imgsz 640 --batch 1 --onnx-only

# 将 yolo11n-seg.onnx 复制到 Orin 后（只需要 JetPack/TensorRT）
./build_engine.sh yolo11n-seg.onnx --imgsz 640 --batch 1 --precision fp16
```

静态 batch=1 是摄像头实时推理最简单、通常也最快的配置。需要一个 engine 支持多种
batch 时，应在导出 `.pt` 时就启用动态轴，并为 TensorRT 指定同一组 profile：

```bash
./build_engine.sh ../yolo11n-seg.pt --imgsz 640 --dynamic \
  --min-batch 1 --opt-batch 2 --max-batch 4
```

一次转换多个模型时使用统一参数和输出目录：

```bash
./build_engine.sh model_detect.onnx model_segment.onnx model_pose.onnx \
  --imgsz 640 --batch 1 --output-dir engines
```

使用 `--dry-run` 可在没有 TensorRT 的机器上检查将要执行的命令。完整参数见：

```bash
./build_engine.sh --help
```

## 原 ResNet18 三路分类部署

三路分类推理脚本只使用系统自带的 TensorRT、OpenCV 和 NumPy，不需要 PyTorch、
ONNX Runtime 或 PyCUDA。三路图像组成一个动态 TensorRT Batch，默认优化形状为
`3x3x224x224`。

原构建命令保持兼容：

```bash
cd /home/jetson/workspaces/TensorRT_ws
./build_engine.sh resnet18_batch3.onnx resnet18_metadata.json resnet18_batch3_fp16.engine
```

先做不依赖摄像头的 Batch=3 验证与测速：

```bash
python3 trt_three_camera.py --self-test --iterations 100
```

三路 USB/V4L2 摄像头推理：

```bash
python3 trt_three_camera.py --cameras 0 1 2
```

Jetson CSI/Argus 摄像头推理（sensor-id 必须与实际硬件一致）：

```bash
python3 trt_three_camera.py --cameras csi:0 csi:1 csi:2 --width 1280 --height 720 --fps 60
```

也可把自定义管线写成 `gst:<GStreamer pipeline>`。默认只在终端打印结果；本地图形桌面下可加
`--display`。当前引擎最大 Batch 为 3，连接一到三路摄像头都可运行。当前测试设备只枚举到
两颗 IMX219，因此实机验证命令为 `--cameras csi:0 csi:1`。
