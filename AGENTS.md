# 仓库提示

## 项目介绍

- 这是一个基于 ROS 2 (Humble) 和 Gazebo Classic 的 RoboCon 2026 仿真工作区，使用 `colcon` 构建
- 当前仓库主要包含比赛场地（`rc2026_field`）、`robot_r2` 机器人模型、控制节点、视觉检测节点、Gazebo C++ 插件，以及自定义 ROS 2 接口
- `robot_r2` 的模型入口为 URDF（`robot_r2_description/urdf/robot_r2.urdf`），配合自定义 Gazebo C++ 插件实现底层物理仿真（麦克纳姆轮运动、抬升机构、夹爪机构、定位反馈等）
- 控制架构分为三层：
  - **底层**：Gazebo C++ 插件（`robot_r2_description/src/`），直接与仿真物理引擎交互，发布关节状态、接收关节指令
  - **中层**：底层控制器节点（`robot_r2_controller/`），每个节点封装单一硬件功能（底盘伺服、抬升、夹爪操作、KFS 对位），通过 ROS 2 Service 对外暴露
  - **上层**：高层控制节点（`robot_r2_control/`），负责任务编排、状态机和键盘遥操作，调用中层 Service 完成复杂流程
- 视觉检测基于 YOLO 模型（`robot_r2_detect/`），订阅相机图像话题，发布 KFS 检测结果
- 这个仓库的重点是快速迭代仿真结构与接口，而不是一开始就追求完全真实的物理建模

## 目录约定

- 这是一个 ROS 2 `colcon` 工作区，主要源码都在 `src/` 下
- `src/robot_r2_interfaces`：自定义 ROS 2 消息（msg/）和服务（srv/），CMake 包（`rosidl_interface_packages`），所有包共享的接口定义
- `src/robot_r2_description`：机器人 URDF 模型和 Gazebo C++ 插件（`src/`），CMake 包，插件编译为 `.so` 供 Gazebo 加载
- `src/robot_r2_controller`：底层控制器节点（Python），每个节点封装单一硬件操作，通过 Service 对外暴露，配置由 YAML 参数文件管理（`config/`）
- `src/robot_r2_control`：高层控制节点（Python），包括键盘遥操作（`teleop_control`）、阶段二任务编排（`stage_two_control`）、KFS 装载状态机（`kfs_loader`），配置由 YAML 参数文件管理（`config/`）
- `src/robot_r2_detect`：YOLO 视觉检测节点（Python），订阅相机图像，发布 KFS 原始和处理后的检测结果
- `src/rc2026_field`：比赛场地资源，包括 world 文件（`worlds/`）、Gazebo 模型（`models/`，包含真假 KFS 模型）、场地 GUI、KFS 管理器、随机摆放脚本（`scripts/`）
- `src/bringup`：启动文件（`launch/`），只做节点组合和参数传递，不放复杂业务逻辑
- `src/test_pkg`：独立可执行测试脚本，用于测试控制 Service，不依赖仿真运行

## 构建与运行

- 顶层 `build/` 和 `install/` 是 `colcon build` 的输出目录
- C++ 包（`robot_r2_description`、`robot_r2_interfaces`）需要通过 `colcon build` 编译
- Python 包修改后一般只需 `colcon build --symlink-install` 或直接 source `install/setup.bash` 后运行
- 主入口：`ros2 launch bringup sim.launch.py`，会依次启动场地、spawn 机器人、启动全部控制节点
- `log/` 目录存放运行日志

## 架构与设计原则

- **职责分离**：`robot_r2_controller`（底层硬件抽象）和 `robot_r2_control`（高层任务编排）严格分开，底层不依赖上层
- **静态配置**：静态配置由 ROS 2 参数机制（YAML 文件）管理，不在 topic 或 service 中传入
- **单一职责**：每个 controller 节点封装一个硬件功能，通过一个 Service 对外暴露，节点名、Service 名、配置文件名保持一致
- **对称性**：注意对称和平等性，不能因为需要简单调整而破坏原本对称平等的逻辑
- **文件职责**：注意文件的职责划分，避免重复持有相同的配置参数
- **及时清理**：注意及时清除不再使用的逻辑和需要改名的地方

## 目前进度

- 底盘：麦克纳姆轮底盘模型与键盘遥操作（WASD + QE 旋转）已完成
- 抬升：前后独立抬升機構模型、Gazebo 插件与控制器已完成，支持底盘抬升以攀爬台阶
- 夹爪：KFS 夹爪模型与多自由度控制（升降/旋转/尖端旋转/夹持）已完成
- KFS 装载：KFS 对位（`kfs_alignment`）、定点装载（`kfs_loader`）、上下台阶装载流程已完成
- 视觉检测：YOLO-based KFS 检测节点已完成，支持原始框和处理后检测结果发布
- 阶段二：`stage_two_control` 任务编排节点已完成，协调检测与装载流程
- 场地：比赛场地 world 包含真假 KFS 模型（红蓝双方各 15 个真 KFS + 15 个假 KFS），支持随机摆放

## 要求（仅用户修改）

- 没有提出构建要求就不用构建
- 没有提出运行调试要求就不用运行调试
- 你在终端中使用 python 的时候可能会遇到 conda 的问题，不用担心在用户的终端中是正常的
- 不用管 `__pycache__`
