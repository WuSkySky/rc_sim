# robot_r2_target_alignment

该包在 ROS 2 Humble 下使用 YOLO11 检测二维目标，并通过底盘横移使目标中心
与画面中心对齐。检测和控制是两个独立节点，默认启用不会发布实际控制消息的
测试模式。

## 模型

默认模型资源为：

```text
package://robot_r2_detect/model/duantou.pt
```

对应源码位置为 `src/robot_r2_detect/model/duantou.pt`。模型文件需要在构建
`robot_r2_detect` 前放入该目录，构建后会安装到包的 share 目录。节点不会在
机器人运行时自动联网下载模型。也可以在启动时通过
`model_path:=/absolute/path/to/model.pt` 指定绝对路径。

模型必须是目标检测模型，不能使用分类模型。官方 COCO `yolo11n.pt` 可用于
COCO 类别；自定义目标需要使用基于 YOLO11n 训练得到的权重。

Python 依赖：

```bash
python3 -m pip install -r src/robot_r2_target_alignment/requirements.txt
```

默认使用 Jetson 的第 0 块 CUDA GPU（`model.device: "0"`）执行推理，FP16
默认关闭。需要临时回退到 CPU 时，将 `model.device` 设置为 `"cpu"`；CPU
模式下不能启用 `model.half`。

## 启动

默认测试模式只打印检测与候选控制状态，不发布 `/r2/cmd_vel`：

```bash
source install/setup.bash
ros2 launch robot_r2_target_alignment target_alignment.launch.py \
  test_mode:=true
```

可视化默认开启，发布包含检测框、目标中心和画面中心线的图像：

```text
/r2/target_alignment/debug_image
```

可使用 `rqt_image_view` 查看：

```bash
ros2 run rqt_image_view rqt_image_view \
  /r2/target_alignment/debug_image
```

测试模式只禁止发布底盘控制指令，不影响可视化话题。若不需要可视化，可在
启动时添加 `visualization_enabled:=false`。

确认目标选择、方向和速度正确后，显式关闭测试模式：

```bash
ros2 launch robot_r2_target_alignment target_alignment.launch.py \
  test_mode:=false
```

仿真时追加 `use_sim_time:=true`。相机和控制话题只能通过 launch remapping
配置，例如：

```bash
ros2 launch robot_r2_target_alignment target_alignment.launch.py \
  camera_topic:=/r2/left_camera/image_raw \
  debug_image_topic:=/r2/target_alignment/debug_image \
  cmd_vel_topic:=/r2/cmd_vel
```

## 目标筛选

在 `config/yolo_target_detector.yaml` 中设置目标类别：

```yaml
target:
  class_names: "person"
  class_ids: ""
```

多个值使用逗号分隔，例如 `"person,sports ball"`。名称和 ID 同时设置时，
检测结果必须同时匹配二者。两个过滤器都为空时允许所有类别。

## 动态参数

所有 YAML 参数都支持运行时更新。例如：

```bash
ros2 param set /r2/target_alignment/yolo_target_detector \
  inference.confidence 0.65
ros2 param set /r2/target_alignment/target_alignment_controller \
  control.output_limit 0.25
ros2 param set /r2/target_alignment/target_alignment_controller \
  test_mode false
```

模型路径、输入尺寸、设备或 FP16 配置变化时，新模型会先完成加载和预热，
成功后才替换旧模型。非法参数会被拒绝，现有配置保持不变。

## 通信接口

- 输入 `camera/image_raw`：`robot_r2_interfaces/CameraFrame`，launch 默认映射到
  `/r2/front_camera/image_raw`；
- 中间结果 `detections`：`robot_r2_interfaces/TargetDetection`；
- 可视化 `debug_image`：`sensor_msgs/Image`，launch 默认映射到
  `/r2/target_alignment/debug_image`；
- 输出 `cmd_vel`：`geometry_msgs/Twist`，launch 默认映射到 `/r2/cmd_vel`。

控制节点只设置 `Twist.linear.y`。默认方向与现有 R2 对齐控制一致：目标位于
画面右侧时输出负 `linear.y`。若实车安装方向相反，动态设置
`control.invert_output=false`。

不要同时运行多个向 `/r2/cmd_vel` 发布非零速度的控制节点。实际模式下，
目标丢失、检测过期、控制停用、进入测试模式和正常退出都会触发零速度；串口桥
的 `cmd_vel_timeout_sec` 负责节点异常退出后的最终超时停车。
