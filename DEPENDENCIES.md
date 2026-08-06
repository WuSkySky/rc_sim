# rc_sim 依赖清单

## 目标环境

- Jetson Orin Nano，Ubuntu 22.04，ARM64
- ROS 2 Humble
- JetPack 提供 CUDA、TensorRT、VPI、Argus 和 Jetson GStreamer 插件
- 当前工作区：`/home/jetson/workspaces/rc_sim`

本仓库同时包含仿真、真实相机、Odin 驱动、串口控制和 CUDA/TensorRT 视觉节点。只运行仿真时不需要安装 Jetson 相机、Odin SDK 或 TensorRT。

## ROS 2 包依赖

| 功能 | ROS 2 依赖 |
| --- | --- |
| bringup | `ament_index_python`、`launch`、`launch_ros`、`gazebo_ros`、各业务包 |
| 场地仿真 | `rclpy`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`gazebo_msgs`、`gazebo_ros`、`launch`、`launch_ros` |
| 控制节点 | `rclpy`、`rcl_interfaces`、`geometry_msgs`、`std_msgs`、`robot_r2_interfaces` |
| Gazebo 模型/插件 | `gazebo_dev`、`gazebo_ros`、`rclcpp`、`rcl_interfaces`、`geometry_msgs`、`sensor_msgs`、`std_msgs` |
| Python 视觉 | `rclpy`、`rcl_interfaces`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`robot_r2_interfaces`、`launch`、`launch_ros` |
| CUDA/TensorRT 视觉 | `rclcpp`、`robot_r2_interfaces`、`sensor_msgs`、`std_msgs`、`ament_index_cpp` |
| MIPI 相机 | `rclcpp`、`rcl_interfaces`、`sensor_msgs`、`robot_r2_interfaces` |
| Odin 后处理 | `rclpy`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`tf2_ros` |
| Odin 驱动 | `rclcpp`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`visualization_msgs`、`cv_bridge`、`image_transport`、`pcl_conversions`、`message_filters`、`tf2*` |
| 串口桥 | `rclpy`、`geometry_msgs`、`std_msgs`、`robot_r2_interfaces` |
| 接口生成 | `rosidl_default_generators`、`rosidl_default_runtime`、`std_msgs` |

已补齐源码中原来未声明的主要 ROS 依赖：launch、launch_ros、gazebo_msgs、visualization_msgs、ament_index_python、python3-yaml 和 OpenCV。

## Ubuntu/ROS 安装依赖

在 Jetson 上可安装以下公开依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config python3-pip python3-dev \
  python3-numpy python3-opencv python3-yaml python3-scipy \
  python3-pynput python3-serial python3-pytest \
  libopencv-dev libeigen3-dev libpcl-dev libyaml-cpp-dev \
  libssl-dev libusb-1.0-0-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base
```

ROS 依赖建议由 rosdep 安装：

```bash
source /opt/ros/humble/setup.bash
rosdep update
cd /home/jetson/workspaces/rc_sim
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
  --skip-keys ament_python
```

当前 Humble rosdep 数据库无法解析 `ament_python` 这个构建类型键，但 ROS 环境本身已经提供 Python 构建支持，因此命令中暂时跳过该键。

### Jetson 真实硬件部署

Jetson Orin Nano 的 ARM64 ROS Humble 源不提供可安装的 Gazebo Classic
运行时，`ros-humble-gazebo-ros` 不存在，`ros-humble-gazebo-dev` 也会因
`gazebo`/`libgazebo-dev` 不可用而失败。Gazebo 只用于仿真，不是
`real1.launch.py`、`real2.launch.py` 或真实视觉节点的运行依赖。

在 Jetson 上安装真实硬件依赖时跳过 Gazebo：

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
  --skip-keys ament_python \
  --skip-keys gazebo_ros \
  --skip-keys gazebo_dev
```

真实硬件工作区建议构建时排除仿真包：

```bash
colcon build --symlink-install --parallel-workers 2 \
  --packages-ignore rc2026_field robot_r2_description
```

Gazebo 仿真应在安装了 Gazebo Classic 和对应 ROS 2 插件的 x86_64 Ubuntu
环境或独立仿真容器中构建，不建议在当前 Orin Nano 上强行安装不匹配的
Gazebo 版本。

## Jetson 专用依赖

### CUDA/TensorRT

`robot_r2_detect_cpp` 需要：

- CUDA Toolkit、CUDA Runtime
- TensorRT C++ 头文件和库：`NvInfer.h`、`libnvinfer.so`
- Python TensorRT 模块：`import tensorrt`
- Jetson Orin 的 CUDA 架构：`sm_87`
- Jetson 本机生成的 `resnet18_batch3_fp16.engine`

TensorRT engine 与 TensorRT 版本、设备架构和系统环境相关，不能从 x86 电脑直接复制生成后长期使用；应在目标 Jetson 上生成或验证。

### MIPI 相机

- JetPack/L4T 的 `nvarguscamerasrc`
- `nvvidconv`
- GStreamer appsink 和 base plugins
- 两个相机设备，例如 `/dev/mipi_left`、`/dev/mipi_right`

### Odin

Odin 驱动还需要仓库内的 ARM 预编译 SDK：

```text
src/sim_to_real/odin/odin_ros_driver/lib/liblydHostApi_arm.*
```

同时需要 Odin USB 设备、对应 udev 权限和运行时配置。该 SDK 不是可通过 rosdep 或 pip 安装的普通依赖。

## Python 可选依赖

以下依赖不是所有启动路径都需要：

- `ttkbootstrap`：`rc2026_field/field_gui.py` 图形界面
- `ultralytics`：旧版 YOLO 路径，当前 TensorRT 融合节点不依赖
- `PyYAML`：部分场地/Odin launch 和配置工具使用
- `pytest`、`ament_flake8`、`ament_pep257`、`ament_copyright`：测试和代码检查

例如图形界面需要时再安装：

```bash
python3 -m pip install --user ttkbootstrap
```

## 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/jetson/workspaces/rc_sim
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

部署真实硬件前应确认 TensorRT engine、Odin ARM SDK、MIPI 设备节点和串口设备均已存在。
