# 仓库提示

## 项目介绍

- 这是 RoboCon 2026 R2 机器人的 ROS 2 Humble `colcon` 工作区，同时支持 Gazebo Classic 仿真和实车部署。
- 仓库包含场地与机器人模型、Gazebo 插件、硬件功能控制、任务编排、视觉检测、实车设备接入和自定义 ROS 2 接口。
- 仿真模型入口为 `src/robot_r2_description/urdf/robot_r2.urdf`。

## 目录约定

- `src/bringup`：仿真、实车及共用控制节点的启动组合。
- `src/rc2026_field`、`src/robot_r2_description`：仿真场地、URDF 和 Gazebo C++ 插件。
- `src/robot_r2_controller`、`src/robot_r2_control`：单一硬件功能控制与高层任务编排。
- `src/robot_r2_detect`、`src/robot_r2_detect_cpp`：图像处理与 KFS/LED 视觉检测。
- `src/robot_r2_interfaces`：共享的 `msg` 和 `srv` 定义。
- `src/sim_to_real`：实车相机、Odin 和串口接入。
- `src/test_pkg`：独立测试与性能测量工具。

## 构建与运行

- 常用构建命令：`colcon build --symlink-install`；运行前执行 `source install/setup.bash`。
- 仿真入口：`ros2 launch bringup sim.launch.py`。
- 实车分两台 ROS 2 主机：`real1.launch.py` 负责控制、串口、Odin 和 LED 检测，`real2.launch.py` 负责 MIPI 相机和 KFS 视觉。
- 具体启动参数和 Service 用法见 `README.md`。

## 架构与设计原则

- `robot_r2_controller` 只封装单一硬件功能，`robot_r2_control` 负责任务编排，底层不依赖上层。
- 节点、Service 和配置按功能拆分；启动文件只做组合、参数传递和 remapping。
- 保持对称逻辑一致，避免重复配置，改名或替换功能时同步清理旧逻辑。

## 要求（仅用户修改）

- 没有提出构建要求就不用构建
- 没有提出运行调试要求就不用运行调试
- 你在终端中使用 python 的时候可能会遇到 conda 的问题，不用担心在用户的终端中是正常的
- 不用管 `__pycache__`
- 构建遇到存在失效配置链接而失败，可以直接清除现有构建结果，重新构建

## 参数规范

- **参数适用范围**：用户可能经常调整、且不改变 ROS 计算图结构的配置必须使用 ROS 2 参数，例如阈值、增益、限位、超时、采样率、模型路径和功能开关；不要将这类配置写成散落在业务逻辑中的常量。
- **通信名称固定**：topic、service、action、TF、frame、node namespace 等通信与坐标系名称不得作为节点参数；节点内使用明确且稳定的默认名称，需要适配不同机器人、传感器或多实例时，统一在 launch 中通过 ROS remapping 配置。
- **YAML 管理参数**：每个节点的用户可配置参数必须集中放在对应包的 `config/*.yaml` 中，参数名与节点职责保持一致。节点只负责声明、校验和使用参数，不得自行查找、打开或解析 YAML；由 launch 通过 `parameters=[...]` 将 YAML 传给节点。
- **支持动态配置**：暴露的参数必须支持运行时通过 `ros2 param set` 修改。节点必须注册参数变更回调，在接受修改前校验类型、范围以及参数间约束，并安全、及时地更新相关派生状态；无效修改必须返回失败且保留原配置。
- **并发与一致性**：动态参数可能与订阅、服务、动作或定时器回调并发发生，更新共享配置和重建依赖对象时必须使用适当的锁或原子替换，避免回调观察到部分更新状态。
