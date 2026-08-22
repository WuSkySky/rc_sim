# rc_sim

## 编译

普通开发电脑没有 Jetson 的 CUDA/TensorRT 环境时，跳过融合检测包；纯 OpenCV
的 KFS ROI 节点仍会由 `robot_r2_kfs_roi` 正常构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-ignore robot_r2_detect_cpp mipi_camera
colcon build --symlink-install --packages-ignore robot_r2_detect_cpp mipi_camera yahboom_camera
source install/setup.bash
```

Jetson 上具有 CUDA/TensorRT 开发环境，使用正常的全量构建命令，不跳过
`robot_r2_detect_cpp`。

## 启动

仿真：

```bash
source install/setup.bash
ros2 launch bringup sim.launch.py
```

需要使用机构和底盘图形控制时，在已启动仿真或实机控制节点的电脑上另开终端：

```bash
source install/setup.bash
ros2 launch bringup gui_control.launch.py
```

正式比赛 GUI 提供待执行阶段选择、任务配置和手动关节控制，不包含调试、重定位
或参数写入按钮。通过独立可执行文件启动：

```bash
ros2 run robot_r2_control competition_gui
```

正式 GUI 左栏使用 Step1、Step2、Step3 三选一选择待执行阶段，并选择红/蓝方、
配置任务。选择本身不会执行阶段；`real1.launch.py` 常驻的 `all_step_control`
收到 `/r2/serial/button` 的物理按钮按下状态后才启动当前阶段。未运行 GUI 时，
总控默认准备红方 Step1。Step2 固定使用路线模式：两个 4 行×3 列
矩阵分别表示移动路线和需要 Load 的 KFS。矩阵上方明确标注出口、下方明确标注
入口，不显示服务内部的格子坐标或索引；路线格按点击顺序显示“第 N 步”，KFS
格只显示“KFS”。每个矩阵均可独立清空。Step2 成功返回 `loaded_count=1..3` 时
会自动更新 Step3 的 KFS 数量。任一阶段运行期间，其他阶段选择、配置和手动关节
控件均会暂时禁用。手动关节范围集中在
`robot_r2_control/config/competition_gui.yaml`，也支持运行时动态修改。

总控执行 Step1 时先调用 `/r2/set_base_pose`，默认重定位到
`(0,0,0,0,0,0)`；执行 Step2 时先调用 `/r2/set_base_pose_odin`，默认使用蓝方
基准 `(5.568,-2.2,0,0,0,π)`，红方将 Y 镜像为 `+2.2`。重定位成功后才会调用
对应阶段服务。按钮屏蔽时间和两个六维位姿集中在
`robot_r2_control/config/all_step_control.yaml`，支持运行时动态修改；Step3 仍由
阶段节点自身完成重定位。

矩阵坐标始终采用用户面对场地的固定视角：出口在上、入口在下，屏幕左、右
就是实际场地的左、右。切换队伍不会移动或清空已经选择的格子。GUI 对两队都直接
传递 `lateral_index=1/2/3`，不再额外交换索引；服务端根据 `team` 将队内格号
映射到实际 Y 坐标和高度布局。

调试 GUI 窗口获得焦点时，可按住实体键盘的 `W/A/S/D/Q/E` 控制底盘，松开时
停止；窗口失去焦点也会立即停止键盘控制。底盘测试区提供定时速度测试和基于
当前 `/r2/pose_feedback` 的相对位置伺服测试。

### 实车网络连接

两台 Jetson 的 SSH 用户均为 `jetson`。两台机器的主机名目前相同，因此连接和
部署时使用下面的固定 IP，不使用主机名区分设备。

| 角色 | 有线 IP（交换机） | 有线 SSH | 无线热点 | 热点侧 IP | 无线 SSH |
| --- | --- | --- | --- | --- | --- |
| real1 | `10.42.0.2/24` | `ssh jetson@10.42.0.2` | `Jetson_Orin_Hot1` | `192.168.50.1/24` | `ssh jetson@192.168.50.1` |
| real2 | `10.42.0.3/24` | `ssh jetson@10.42.0.3` | `Jetson_Orin_Hot2` | `192.168.50.2/24` | `ssh jetson@192.168.50.2` |

两台实机共同运行 ROS 2 时，使用有线交换机连接。开发电脑配置在
`10.42.0.0/24` 网段（当前使用 `10.42.0.1/24`），两台 Jetson 分别保持上述
`.2` 和 `.3` 地址，即可通过交换机进行 ROS 2 节点发现、Topic 和 Service
通信。Jetson1（real1）固定使用 `10.42.0.2`，Jetson2（real2）固定使用
`10.42.0.3`；两台机器均使用 ROS 2 Humble 和 `ROS_DOMAIN_ID=99`。

网络连接和远程调试必须优先使用上述有线地址。未经操作者明确同意，不得切换
开发电脑的 Wi-Fi，也不得连接任一 Jetson 热点。无线热点只用于经过明确授权的
单机维护：电脑连接目标 Jetson 的热点后，通过对应的 `192.168.50.x` 地址 SSH。
两个热点属于独立接入点，不用于替代两台 Jetson 之间的有线 ROS 2 网络。

实车使用两个 ROS 2 主机，二者需要处于同一网络、ROS domain，并使用仓库中的
Fast DDS 配置。real1 负责控制、串口、KFS/端头对齐控制，以及下位机和
Odin 两套独立里程计。底盘位置伺服可由每个请求选择下位机或 Odin 位姿
作为闭环来源。real1 不再启动 MIPI 相机；real2 发布端头相机话题
`/r2/tip_camera/image_raw`，在本机运行端头 YOLO 检测，并将 `/r2/tip/roi`
发回 real1 供对齐控制使用。Odin 后摄像头发布
`/r2/rear_camera/image_raw`。LED 检测路线仍不由 real1 启动：

```bash
source install/setup.bash
ros2 launch bringup real1.launch.py
```

real2 使用 Yahboom USB 前相机和端头 MIPI 相机，负责 KFS 识别、KFS ROI 和
端头 YOLO 目标检测。原左右 MIPI 相机已移除。前相机发布自定义
`/r2/front_camera/image_raw`，供 KFS 融合节点的 front 输入和 ROI 节点使用；
端头相机发布 `/r2/tip_camera/image_raw`，供端头 YOLO 使用。
KFS 和端头检测的调试图均默认关闭，可通过对应
动态参数开启。融合节点不会等待未接入的相机，因此只有部分相机
在线时也能正常推理：

```bash
source install/setup.bash
ros2 launch bringup real2.launch.py
```

real2 的 ROI 默认使用端头 MIPI 相机，real1 的阶段任务默认使用融合
KFS 识别服务 `/r2/detection/get_type`。

## ROS 2 服务

以下命令需在已经加载工作区环境的终端中执行：

```bash
source install/setup.bash
```

可用 `ros2 service list` 查看当前已启动的服务，使用下面的命令查看某个自定义
服务的完整请求和响应字段：

```bash
ros2 interface show robot_r2_interfaces/srv/MoveToPose
```

### 服务速查


| 功能         | 服务名                          | 服务类型                                          | 可用环境  |
| ---------- | ---------------------------- | --------------------------------------------- | ----- |
| 阶段 1       | `/r2/stage_one`              | `robot_r2_interfaces/srv/StageOne`            | 仅实机   |
| 完整阶段二      | `/r2/stage_two`              | `robot_r2_interfaces/srv/StageTwo`            | 仿真、实机 |
| 阶段 2.1     | `/r2/stage_two_point_one`    | `robot_r2_interfaces/srv/StageTwoPointOne`    | 仿真、实机 |
| 阶段 2.2     | `/r2/stage_two_point_two`    | `robot_r2_interfaces/srv/StageTwoPointTwo`    | 仿真、实机 |
| 阶段 2.2 后续离场 | `/r2/stage_two_point_two_exit` | `robot_r2_interfaces/srv/StageTwoPointTwoExit` | 仿真、实机 |
| 阶段 3       | `/r2/stage_three`            | `robot_r2_interfaces/srv/StageThree`          | 仿真、实机 |
| 配置物理按钮待执行阶段 | `/r2/all_step/configure`      | `robot_r2_interfaces/srv/ConfigureAllStep`    | 仅实机   |
| 底盘绝对位置伺服   | `/r2/move_to_pose`           | `robot_r2_interfaces/srv/MoveToPose`          | 仿真、实机 |
| 底盘相对位置伺服   | `/r2/move_relative`          | `robot_r2_interfaces/srv/MoveRelative`        | 仿真、实机 |
| 重置或设置里程计位姿 | `/r2/set_base_pose`          | `robot_r2_interfaces/srv/SetBasePose`         | 仅实机   |
| 重置或设置 Odin 位姿 | `/r2/set_base_pose_odin`     | `robot_r2_interfaces/srv/SetBasePose`         | 仅实机   |
| 四轮抬升       | `/r2/lift/set`               | `robot_r2_interfaces/srv/SetLift`             | 仿真、实机 |
| 跨越台阶       | `/r2/step_traverse`          | `robot_r2_interfaces/srv/TraverseStep`        | 仿真、实机 |
| KFS 视觉对齐   | `/r2/align_to_kfs`           | `robot_r2_interfaces/srv/Align`               | 仿真、实机 |
| 端头视觉对齐    | `/r2/align_to_tip`           | `robot_r2_interfaces/srv/Align`               | 仅实机   |
| KFS 类型检测   | `/r2/detection/{front,left,right}/get_type` | `robot_r2_interfaces/srv/GetKfsType` | 仅实机 |
| LED 状态检测   | `/r2/led_detection/detect`   | `robot_r2_interfaces/srv/DetectLed`           | 仿真、实机 |
| KFS 装载、释放、弹出 | `/r2/kfs/action`             | `robot_r2_interfaces/srv/KfsAction`           | 仿真、实机 |
| KFS 夹爪升降   | `/r2/kfs_lift`               | `robot_r2_interfaces/srv/SetJointPosition`    | 仿真、实机 |
| KFS 夹爪根部旋转 | `/r2/gripper/set_rotate`     | `robot_r2_interfaces/srv/SetJointPosition`    | 仿真、实机 |
| KFS 夹爪末端旋转 | `/r2/gripper/set_tip_rotate` | `robot_r2_interfaces/srv/SetJointPosition`    | 仿真、实机 |
| KFS 夹爪开合   | `/r2/gripper/set_grip`       | `robot_r2_interfaces/srv/SetJointPosition`    | 仿真、实机 |
| 武器旋转       | `/r2/weapon/set_rotate`      | `robot_r2_interfaces/srv/SetJointPosition`    | 仅实机   |
| 武器夹爪开合    | `/r2/weapon/set_grip`        | `robot_r2_interfaces/srv/SetJointPosition`    | 仅实机   |
| 重新随机摆放 KFS | `/simulation/reset_kfs`      | `std_srvs/srv/Trigger`                        | 仅仿真   |

### 一次性中止正在执行的任务

任务编排、底盘运动、升降、夹爪、视觉对齐和任务依赖的检测服务都订阅
`/r2/system/abort`（`std_msgs/msg/Empty`）。收到一条消息后，当时已经开始的
相关服务会尽快停止并以 `success: false` 返回；之后发起的新请求不受影响。

```bash
ros2 topic pub --once /r2/system/abort std_msgs/msg/Empty "{}"
```

调试 GUI 和正式比赛 GUI 均提供始终可用的“取消当前任务”按钮，用于发布同一
中止消息；调试 GUI 还会同步停止本机键盘控制和定时速度测试。

底盘会立即发布零速度，位置控制执行器会在存在有效反馈时重新下发当前位置以
保持。该机制用于软件任务中止，不替代硬件急停或下位机失联保护。


### 阶段一任务

阶段一仅由 `real1.launch.py` 启动。请求中的 `team` 只接受 `red` 或 `blue`；当前
动作参数按红方定义，蓝方执行时仅反转左右平移方向。服务会依次执行底盘升降、
斜向移动、端头视觉对齐、武器旋转与夹取，以及最终底盘转向。转向完成后调用
`/r2/led_detection/detect`，只有 LED 状态连续稳定匹配请求目标并返回成功，才会
松开武器夹爪；检测失败、服务不可用或等待超时时保持夹持并令 Step1 返回失败。
端头视觉对齐如果返回 `Alignment timeout:` 超时结果，Step1 会记录警告并继续执行
剩余动作；最终 `message` 会包含该超时信息。对齐服务不可用、被中止或其他非超时
错误仍会令 Step1 失败。
初始左右/前后位置伺服与端头旋转、夹爪张开同时执行，三者全部完成后才继续。
夹爪闭合后，底盘会在当前工作高度基础上再向上抬升
`action_8_lift_increment_m`（默认 `0.03 m`）并将武器旋转到最终角度，再使用
下位机相对位置伺服前进
`action_9_pre_lower_forward_m`（默认 `0.05 m`）。随后底盘降回 `0.01 m`，继续
前移 `0.20 m`。

```bash
ros2 service call /r2/stage_one robot_r2_interfaces/srv/StageOne \
  "{team: red}"

ros2 service call /r2/stage_one robot_r2_interfaces/srv/StageOne \
  "{team: blue}"
```

GUI 顶部的比赛队伍选择器由 Step1、Step2、Step3 共用；`Step1` 区域提供一个执行
按钮和“从 YAML 写入 Step1 参数”按钮。执行按钮会先调用
`/r2/set_base_pose`，成功后再使用对应的 `team` 调用 `/r2/stage_one`。默认重定位
位姿为全零，可通过 `gui_control.yaml` 中的 `stage_one_relocalization_pose`
动态调整。参数写入按钮会加载工作区源码中的
`src/robot_r2_control/config/stage_one.yaml` 到 `/stage_one` 节点。

动作目标、容差和超时集中在 `robot_r2_control/config/stage_one.yaml`，均支持运行时
动态修改。参数修改在下一次阶段一服务调用时整体生效，例如：

```bash
ros2 param set /stage_one action_2_left_m 0.790
ros2 param set /stage_one weapon_grip_tolerance_m 0.0015
ros2 param set /stage_one led_target_states "[true]"
ros2 param set /stage_one final_weapon_grip_m 0.028
```


### 阶段二任务

完整执行阶段 2.1 → 2.2。`team` 只接受 `red` 或 `blue`。配置统一以位于
负 Y 半场的蓝方为基准：蓝方 `lane 1/2/3` 分别使用
`Y=-4.2/-3.0/-1.8`；红方同时镜像 lane 编号、Y 坐标和高度布局，因此
红方 `lane 1/2/3` 分别为 `Y=+1.8/+3.0/+4.2`。yaw 不额外镜像，移动
朝向根据实际目标坐标计算。假 KFS 决策：`1=左`，`2=右`。总服务模式为
`0=标准识别`、`1=skip 识别`、`2=路线`。

```bash
ros2 service call /r2/stage_two robot_r2_interfaces/srv/StageTwo \
  "{team: red, fake_kfs_decision: 1, mode: 0, move_cells: [], kfs_cells: []}"
```

标准和 skip 模式会忽略 `move_cells`、`kfs_cells`，并依次以相同模式调用 2.1、
2.2。路线模式使用统一的四行三列坐标：`forward_index=1..4`、
`lateral_index=1..3`。`move_cells` 仅作为 2.2 的移动路线；`kfs_cells` 是无序的
真实 KFS 占用集合。第 4 行入口侧的 KFS 按 `2,1,3` 过滤顺序交给 2.1，其他
三行交给 2.2。入口侧没有 KFS 时跳过 2.1，直接从 `loaded_count=0` 执行 2.2。
红蓝两队交给 2.1 和 2.2 的都是相同的队内格号，镜像只在各阶段服务内完成。
每次 2.2 服务开始都会先位置伺服到 `(5,2)` 格心，再执行第一段上台阶动作。

```bash
ros2 service call /r2/stage_two robot_r2_interfaces/srv/StageTwo \
  "{team: red, fake_kfs_decision: 0, mode: 2, \
    move_cells: [
      {forward_index: 4, lateral_index: 2},
      {forward_index: 3, lateral_index: 2},
      {forward_index: 2, lateral_index: 2},
      {forward_index: 1, lateral_index: 2},
      {forward_index: 1, lateral_index: 1}],
    kfs_cells: [
      {forward_index: 4, lateral_index: 3},
      {forward_index: 3, lateral_index: 1},
      {forward_index: 1, lateral_index: 2}]}"
```

单独执行阶段 2.1：

```bash
ros2 service call /r2/stage_two_point_one \
  robot_r2_interfaces/srv/StageTwoPointOne \
  "{team: red, loaded_count: 0, mode: 0, route_cells: []}"
```

阶段 2.1 的 `mode`：`0=标准识别`、`1=skip 识别`、`2=路线`。标准和 skip
模式固定访问 `[2, 1, 3]`；蓝方从 `Y=-2.2` 入口开始时先到
`lane 2 (Y=-3.0)`，首段移动方向为 `-Y`。路线模式通过 `route_cells` 指定第 5
列中的横向格子编号，数组必须非空、不能重复且只能包含 `1`、`2`、`3`。路线模式
不调用 KFS 类型识别，但到达每个格子并调整底盘高度后仍会先执行视觉对齐再装载。例如依次到达
`(5,1)`、`(5,3)` 并装载：

```bash
ros2 service call /r2/stage_two_point_one \
  robot_r2_interfaces/srv/StageTwoPointOne \
  "{team: red, loaded_count: 0, mode: 2, route_cells: [1, 3]}"
```

skip 模式同样不需要视觉服务，但只移动、不装载：

```bash
ros2 service call /r2/stage_two_point_one \
  robot_r2_interfaces/srv/StageTwoPointOne \
  "{team: red, loaded_count: 0, mode: 1, route_cells: []}"
```

单独执行阶段 2.2：

```bash
ros2 service call /r2/stage_two_point_two \
  robot_r2_interfaces/srv/StageTwoPointTwo \
  "{team: red, fake_kfs_decision: 1, loaded_count: 0, mode: 0, move_cells: [], load_cells: []}"
```

阶段 2.2 的 `mode` 同样为 `0=标准识别`、`1=skip 识别`、`2=路线`。路线模式
使用结构化格子数组：`move_cells` 表示机器人依次到达的格心，`load_cells` 表示
需要装载的 KFS 所在格。移动路线必须从 `(4,2)` 开始，以 `(1,1)` 或 `(1,3)`
结束；服务随后会自动补走到对应的 `(0,1)` 或 `(0,3)`。路线模式不调用 KFS
类型检测，但装载前仍会执行视觉对齐。例如：

```bash
ros2 service call /r2/stage_two_point_two \
  robot_r2_interfaces/srv/StageTwoPointTwo \
  "{team: red, fake_kfs_decision: 0, loaded_count: 0, mode: 2, \
    move_cells: [
      {forward_index: 4, lateral_index: 2},
      {forward_index: 3, lateral_index: 2},
      {forward_index: 2, lateral_index: 2},
      {forward_index: 1, lateral_index: 2},
      {forward_index: 1, lateral_index: 1}],
    load_cells: [
      {forward_index: 4, lateral_index: 1},
      {forward_index: 0, lateral_index: 1}]}"
```

每个 load 格必须能在路线某个格心成为机器人到达朝向的前、左或右邻格；同一
格心有多个目标时，下一移动方向上的 KFS 最后装载。路线和 load 格会在移动前
完成合法性校验。GUI 不提供 2.2 路线编辑入口，正常和 skip 按钮仍可照常使用。
`stage_two_point_two.yaml` 中的超时、网格、格子高度、索引、边缘偏移和采样数均
支持运行时动态修改；为保证任务路线一致，网格相关参数在 2.2 或离场服务执行中
会拒绝修改，任务结束后可正常更新。

单独执行阶段 2.2 后续离场动作：

```bash
ros2 service call /r2/stage_two_point_two_exit \
  robot_r2_interfaces/srv/StageTwoPointTwoExit \
  "{team: red}"
```

服务开始后会先通过 `/r2/lift/set` 将前后底盘绝对抬升到 `0.03 m`，成功后才会
执行两段 Odin 绝对位置伺服。抬升高度和超时可分别通过
`exit_lift_height`、`lift_timeout_sec` 动态修改；抬升或任一移动失败都会立即停止。

该独立服务不会自动追加到阶段 2.2 或完整阶段二流程。它使用 Odin 绝对位置伺服，
先到 `(0,0)` 外推格心，再保持 Y 和 yaw 不变，沿世界负 X 移动
`exit_x_offset`（默认 `2.9 m`）。蓝方默认两段目标为
`(-2.6, -5.4, π)`、`(-5.5, -5.4, π)`，即先向 `-Y` 离开网格，再向 `-X`
移动；红方镜像 Y，目标为 `(-2.6, 5.4, π)`、`(-5.5, 5.4, π)`。格心和离场距离
可通过 `stage_two_point_two.yaml` 中的 `exit_cell_0_0_pose`、`exit_x_offset`
动态修改。

GUI 使用 Step1、Step2、Step3 共用的红/蓝方选择器，默认红方。调试 GUI 提供与
正式 GUI 相同的完整 Step2 双矩阵路线测试：矩阵上方为出口、下方为入口，不显示
内部格子坐标；启动后以 ROUTE 模式调用 `/r2/stage_two`。执行前先重定位到 `(5,2)`
格心，蓝方为 `(3.4,-3.0,π)`、红方为 `(3.4,3.0,π)`，两方均朝世界 `-X` 的
出口方向。该位姿由 `gui_control.yaml` 的 `stage_two_relocalization_pose` 配置。

原有分段测试继续保留：2.1 提供正常识别、skip 识别以及路线直接装载，2.2 提供
正常识别和 skip 识别。2.1 路线输入使用逗号分隔，例如 `2,1,3`。GUI 会先按所选
队伍调用 Odin 重定位，成功后再把相同的 `team` 传给阶段服务。分段重定位位姿配置在
`gui_control.yaml` 的 `stage_two_point_one_relocalization_pose` 和
`stage_two_point_two_relocalization_pose` 中，填写的是蓝方基准，红方自动镜像其中的 Y。
2.1 默认 X 保持 `5.568`；蓝方默认 Y 为 `(4,3)` 格心的 `-1.8` 再减少
`0.4 m`，即 `-2.2`，红方对称为 `+2.2`。Step2.2 区域的
“2.2 后续动作（重定位后执行）”按钮会先按队伍重定位到 `(0,3)` 格心（蓝方默认
`(-2.6, -1.8, π)`，红方镜像 Y），成功后调用独立离场服务；该重定位基准可通过
`stage_two_point_two_exit_relocalization_pose` 动态修改。

### 阶段三任务

阶段三服务请求中的 `team` 只接受 `red` 或 `blue`，`loaded_count` 只接受
`1`、`2` 或 `3`。服务开始后先进行 Odin 重定位：蓝方位姿由阶段 2 后续动作终点
`(-5.5, -5.4, π)` 沿 X 减小 `0.19 m`、沿 Y 增大 `0.13 m` 得到
`(-5.69, -5.27, π/2)`，朝向 `+Y`；红方关于 X 轴对称，重定位为
`(-5.69, +5.27, -π/2)`，朝向 `-Y`。

蓝方三段放置位置依次为 `(-5.29, -0.64, π/2)`、
`(-4.75, -0.64, π/2)`、`(-4.21, -0.64, π/2)`；从重定位到第一个点同时
向 `+Y` 前进、向 `+X` 右移。红方镜像 Y 和 yaw。
每段先进行 Odin 绝对位置伺服，再 POP，随后使用下位机相对位置伺服前进
`0.25 m`，最后使用下位机相对位置伺服执行原有后退：

- 重定位后前往第一个放置点时使用 `1.125 m/s` 线速度上限，即位置伺服默认
  `0.45 m/s` 上限的 `2.5` 倍；后续移动不覆盖默认速度。
- 每种数量都只在第一次 POP 前将前后底盘抬升到 `0.23 m`，后续 POP 不重复设置。
- 已有 3 个 KFS：依次使用 POP 模式 1、模式 2、夹爪抬升后模式 2；前两段各后退
  `0.25 m`，最后后退 `3 m`。
- 已有 2 个 KFS：依次使用 POP 模式 2、夹爪抬升后模式 2；第一段后退 `0.25 m`，
  最后后退 `3 m`。
- 已有 1 个 KFS：第一次 POP 同时也是最后一次，因此先抬升底盘到 `0.23 m`，再将
  夹爪升降到 `0.25 m`，使用 POP 模式 2，最后后退 `2 m`。

```bash
ros2 service call /r2/stage_three robot_r2_interfaces/srv/StageThree \
  "{team: blue, loaded_count: 3}"
```

重定位基准、位置偏移、段间距、放置后前进距离、后退距离、KFS 与底盘抬升目标和
各动作超时集中在 `robot_r2_control/config/stage_three.yaml`，均支持运行时动态修改。
GUI 的 Step3 区域提供“已有 1 个 KFS”“已有 2 个 KFS”“已有 3 个 KFS”三个按钮，
并使用顶部共用的比赛队伍选择。

### 底盘、里程计与抬升

底盘速度控制使用 Topic 而不是 Service。`linear.x` 为前后速度、`linear.y` 为
左右速度，`angular.z` 为旋转角速度：

```bash
ros2 topic pub -r 20 /r2/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

底盘位置伺服的坐标单位为米，偏航角单位为弧度。`pose_source` 必须显式填写
`serial` 或 `odin`；前者使用 `/r2/pose_feedback`，后者使用
`/r2/pose_feedback_odin`。`linear_speed_limit` 可为单次请求覆盖 X/Y 线速度上限；
省略或填写 `0.0` 时继续使用 `chassis_pose_servo.yaml` 中当前的 PID 输出上限。
绝对移动目标位于所选来源的 `map` 坐标下：

```bash
ros2 service call /r2/move_to_pose robot_r2_interfaces/srv/MoveToPose \
  "{pose_source: odin, x: 0.0, y: 0.0, yaw: 1.5708, position_tolerance: 0.0, yaw_tolerance: 0.0, timeout_sec: 20.0}"
```

相对移动的 `forward`、`left` 和 `yaw_delta` 以请求开始时所选来源中的机器人
位姿为基准。以下命令使用串口里程计闭环前进 `0.1 m`：

```bash
ros2 service call /r2/move_relative robot_r2_interfaces/srv/MoveRelative \
  "{pose_source: serial, forward: 0.1, left: 0.0, yaw_delta: 0.0, position_tolerance: 0.0, yaw_tolerance: 0.0, timeout_sec: 20.0}"
```

相邻请求可以不重定位而直接切换来源。例如先执行上述串口相对移动，再执行 Odin
绝对移动。位置伺服不会融合两路位姿、检查连续性或自动回退；每条请求从开始到结束
始终只使用其指定来源。Step1 的底盘动作使用 `serial` 相对移动；台阶跨越使用
`serial` 相对距离；阶段 2.1 与 2.2 的显式移动使用 Odin 绝对移动（2.2 经台阶
跨越服务间接执行的仍是 `serial` 相对移动）。请求开始时默认等待所选来源的新
反馈最多 `2 s`；运动开始后的反馈中断仍由该请求的总超时负责结束。

实机重置主里程计位姿，将调用时刻的 `base_link_serial` 对齐到 `map` 原点：

```bash
ros2 service call /r2/set_base_pose \
  robot_r2_interfaces/srv/SetBasePose "{}"
```

也可以将当前位姿设置为指定值。平移单位为米，RPY 角度单位为弧度：

```bash
ros2 service call /r2/set_base_pose \
  robot_r2_interfaces/srv/SetBasePose \
  "{x: 1.5, y: -0.5, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 1.5708}"
```

该服务由 `real1.launch.py` 启动的 `odometry_tf` 提供。它根据下位机（串口）里程计
更新 `map -> odom_serial -> base_link_serial`，从而校正 `/r2/pose_feedback` 和
底盘位置伺服使用的地图位姿；不会清空下位机发布的原始里程计数据。尚未收到下位机
里程计时，服务会返回失败。

Odin 使用独立分支
`map -> odom_odin -> base_link_odin -> odin_link`，其位姿发布在
`/r2/pose_feedback_odin`。如需单独重定位 Odin 分支：

```bash
ros2 service call /r2/set_base_pose_odin \
  robot_r2_interfaces/srv/SetBasePose \
  "{x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"
```

两套重定位彼此独立；位置伺服可由每个请求独立选择串口或 Odin 分支。

四轮抬升：

```bash
ros2 service call /r2/lift/set robot_r2_interfaces/srv/SetLift \
  "{front_lift: 0.2, rear_lift: 0.2, tolerance: 0.0, timeout_sec: 15.0}"
```

跨越台阶，方向：`0=上`、`1=下`，距离允许为负数。

```bash
ros2 service call /r2/step_traverse robot_r2_interfaces/srv/TraverseStep \
  "{direction: 0, distance_to_step: 0.2}"
```

### KFS 与视觉

KFS 视觉对齐会根据 `/r2/kfs/roi` 中的红蓝区域横向移动底盘。仿真 ROI 使用
前相机；实机的 Yahboom USB 前相机和 ROI 节点都由 real2 启动，ROI 默认使用
`/r2/front_camera/image_raw`。real1 不处理该图像链路，只有对齐节点订阅
`/r2/kfs/roi`。该话题只携带时间戳、源帧序号、有效性、左右边界、左右
边界列的最下方掩膜点和横向中心偏差，不包含图像像素：

使用 `alignment.yaml` 中的默认容差和超时时间执行对齐：

```bash
ros2 service call /r2/align_to_kfs \
  robot_r2_interfaces/srv/Align \
  "{pixel_tolerance: 0.0, timeout_sec: 0.0}"
```

也可以在单次调用中指定横向像素容差和超时时间：

```bash
ros2 service call /r2/align_to_kfs \
  robot_r2_interfaces/srv/Align \
  "{pixel_tolerance: 20.0, timeout_sec: 10.0}"
```

`pixel_tolerance` 是允许的图像横向像素误差，`timeout_sec` 是整个对齐过程的
最大等待时间。两项都填写 `0.0` 时使用 `alignment.yaml` 中的默认值。
返回值中的 `success` 表示是否连续稳定达到容差，`final_offset_x` 是最后一次
有效检测的横向像素误差。调用前应确保目标红色或蓝色区域位于当前 ROI 输入相机
的视野内。

对齐容差、稳定帧数、默认超时、PID、速度限幅和输出方向均支持运行时动态修改。
`reverse_direction` 为 `true` 时只反转最终发布的 `linear.y`，不改变检测偏差、
容差或完成判定。例如：

```bash
ros2 param set /kfs_alignment output_limit 0.05
ros2 param set /kfs_alignment pixel_tolerance 10
ros2 param set /tip_alignment reverse_direction false
```

real1 还会启动使用相同基础参数的 `/tip_alignment` 实例。它订阅
`/r2/tip/roi`，通过 `/r2/align_to_tip` 提供端头对齐服务，并将速度输出映射到
`/r2/cmd_vel`。该实例通过 `tip_alignment.yaml` 设置
`reverse_direction: true`，并将横向最大速度单独限制为 `0.075 m/s`（基础配置
`0.1 m/s` 的 75%）；KFS 和仿真实例仍使用基础速度配置。例如：

```bash
ros2 service call /r2/align_to_tip \
  robot_r2_interfaces/srv/Align \
  "{pixel_tolerance: 0.0, timeout_sec: 0.0}"
```

端头检测上游应在 `/r2/tip/roi` 发布
`robot_r2_interfaces/msg/AlignmentDetection`。KFS ROI 节点在
`/r2/kfs/roi` 发布相同的通用检测类型。

参数修改会整体校验后原子生效；无效值会被拒绝。服务请求中的容差或超时为
`0.0` 时，正在执行的任务会使用对应参数的最新值；请求中显式指定的正数不受
后续参数修改影响。

可视化默认关闭。大多数可能发布调试图像的节点使用动态参数
`visualization_enabled`；kfs_detect_fused 的三路调试图像由
`visualization_enabled_front/left/right` 独立控制，端头 YOLO 检测节点则使用
`visualization.enabled`。这些参数都可以在运行时开启或关闭：

```bash
ros2 param set /kfs_roi visualization_enabled true
# 实机检测节点（三路独立开关）：
ros2 param set /kfs_detect_fused visualization_enabled_front true
ros2 param set /kfs_detect_fused visualization_enabled_left true
ros2 param set /kfs_detect_fused visualization_enabled_right true
ros2 param set /r2/front_camera_controller visualization_enabled true
ros2 param set /tip_mipi_camera visualization_enabled true
ros2 param set /r2/target_alignment/yolo_target_detector \
  visualization.enabled true
```

将最后的 `true` 改为 `false` 即可关闭。`kfs_roi` 的五阶段调试图像发布在
`/r2/kfs/roi/debug`；当前实机 KFS 检测图像为 `/r2/detection/front/debug`；
端头 YOLO 检测调试图为
`/r2/target_alignment/debug_image`。
仿真前相机和实机端头 MIPI 相机也使用同一个动态参数控制各自的
`/debug` 图像。
这些调试话题均使用 `sensor_msgs/msg/Image`。
图像主链路和所有调试图统一使用 Best Effort、Keep Last 1、Volatile QoS。
查看端必须使用兼容的 Best Effort 订阅 QoS。

实机 KFS 类型检测：

```bash
ros2 service call /r2/detection/get_type \
  robot_r2_interfaces/srv/GetKfsType \
  "{sample_count: 10, timeout_sec: 10.0}"
```

当前实机仅端头 MIPI 相机提供 KFS 图像输入，在融合节点中作为 front 路。
仿真启动文件不再启动 KFS 类型检测节点。

LED 状态检测。示例表示等待三个 LED 的状态稳定匹配“亮、灭、亮”：

实机 `real1.launch.py` 使用 Odin 后摄像头
`/r2/rear_camera/image_raw`。开启 `visualization_enabled` 时节点持续订阅图像并
发布 `/r2/led_detection/debug`；关闭可视化后，仅在服务调用期间或持续检测模式
下保留订阅。服务连续 5 帧与请求状态一致时返回成功，30 秒内未满足则返回失败。rear
`CameraFrame` 发布和 LED 处理频率都限制为最高 15 Hz（当前 Odin 设备配置
实际为 10 Hz）。可视化图像使用 Best Effort QoS。

```bash
ros2 service call /r2/led_detection/detect \
  robot_r2_interfaces/srv/DetectLed \
  "{target_states: [true, false, true]}"
```

KFS 动作通过 `action` 区分：`load=装载`、`release=释放`、`pop=弹出`。
`mode` 的含义由动作决定。装载请求通过 `mode=1..5` 选择完整轨迹：

| mode | 装载方向 | 当前装载数量 | 动作结果 |
|---:|---|---:|---|
| 1 | 前方 | 0 | 装载到车上 |
| 2 | 前方 | 2 | 留在夹爪 |
| 3 | 上方 | 0 | 装载到车上 |
| 4 | 上方 | 2 | 留在夹爪 |
| 5 | 上方 | 1 | 使用单件上方装载轨迹 |

```bash
ros2 service call /r2/kfs/action robot_r2_interfaces/srv/KfsAction \
  "{action: load, mode: 1}"
```

装载、释放和弹出动作由 `robot_r2_control/config/kfs_loader.yaml` 中的八条轨迹配置：
`mode_1_sequence` 到 `mode_5_sequence`、`release_sequence`、
`pop_1_sequence` 和 `pop_2_sequence`。
每连续六个数表示一个同步步骤，字段顺序为根部位置、末端位置、夹爪位置以及
三者各自的容差。每一步会同时向三个电机发送目标，全部到达后才执行下一步；
无需运动的电机重复填写上一目标值。

轨迹支持运行时整体替换。以下示例把模式 2 改为一个步骤：

```bash
ros2 param set /kfs_loader_control mode_2_sequence \
  "[0.0, 0.0, 0.145, 0.01, 0.01, 0.005]"
```

轨迹数组必须非空且长度为六的倍数。动态更新在下一次装载、释放或弹出调用时
生效，不会改变正在执行的动作。

释放 KFS：

```bash
ros2 service call /r2/kfs/action robot_r2_interfaces/srv/KfsAction \
  "{action: release}"
```

弹出 KFS 时，`mode=1` 表示从夹爪直接放置，`mode=2` 表示先从车上拿取再放置：

```bash
ros2 service call /r2/kfs/action robot_r2_interfaces/srv/KfsAction \
  "{action: pop, mode: 1}"

ros2 service call /r2/kfs/action robot_r2_interfaces/srv/KfsAction \
  "{action: pop, mode: 2}"
```

以下服务用于直接调试 KFS 夹爪机构。`position` 分别表示升降位置、根部角度、
末端角度和开合位置；升降及开合单位为米，角度单位为弧度。`tolerance` 和
`timeout_sec` 填写 `0.0` 时使用对应控制器配置中的默认值。夹爪开合位置
`0.0` 表示闭合，正值表示打开，`0.209` 表示完全打开。末端旋转以初始姿态
为 `0 rad`，沿工作旋转方向使用负值，范围为 `-π–0 rad`。

```bash
ros2 service call /r2/kfs_lift robot_r2_interfaces/srv/SetJointPosition \
  "{position: 0.0, tolerance: 0.0, timeout_sec: 0.0}"

ros2 service call /r2/gripper/set_rotate \
  robot_r2_interfaces/srv/SetJointPosition \
  "{position: 0.0, tolerance: 0.0, timeout_sec: 0.0}"

ros2 service call /r2/gripper/set_tip_rotate \
  robot_r2_interfaces/srv/SetJointPosition \
  "{position: -1.5708, tolerance: 0.0, timeout_sec: 0.0}"

ros2 service call /r2/gripper/set_grip \
  robot_r2_interfaces/srv/SetJointPosition \
  "{position: 0.209, tolerance: 0.0, timeout_sec: 0.0}"
```

武器机构同样使用 `SetJointPosition`。旋转单位为弧度，允许范围默认为
`0–3.49066 rad`（`0–200°`）；夹爪开合单位为米，`0.0` 表示闭合，默认最大开度
为 `0.03 m`：

```bash
ros2 service call /r2/weapon/set_rotate \
  robot_r2_interfaces/srv/SetJointPosition \
  "{position: 1.5707963268, tolerance: 0.01, timeout_sec: 10.0}"

ros2 service call /r2/weapon/set_grip \
  robot_r2_interfaces/srv/SetJointPosition \
  "{position: 0.028, tolerance: 0.001, timeout_sec: 10.0}"
```

### 仿真场地

重新随机摆放 KFS：

```bash
ros2 service call /simulation/reset_kfs std_srvs/srv/Trigger "{}"
```

## 串口协议

串口默认使用 `115200 8N1`。命令帧和反馈帧长度不同：

```text
TX（46 字节）：0xAA | 11 × IEEE 754 binary32 | 0x55
RX（59 字节）：0xAA | 14 × IEEE 754 binary32 | button | 0x55
```

- 字节 `0`：帧头 `0xAA`
- TX 字节 `1~44`：11 个连续的控制字段，无填充
- TX 字节 `45`：帧尾 `0x55`
- RX 字节 `1~56`：14 个连续的反馈字段，无填充
- RX 字节 `57`：物理按钮，`1` 表示按下，`0` 表示抬起
- RX 字节 `58`：帧尾 `0x55`
- 浮点数字节序由 `float_endianness` 配置，当前为小端
- 线位移、线速度使用米和米每秒，角度、角速度使用弧度和弧度每秒

字段排列如下，下发帧表示目标值，反馈帧表示相同机构的实际值：


| 序号  | 字节    | 字段                | 下发含义         | 反馈含义         | 单位    |
| ---: | -----: | ----------------- | ------------ | ------------ | ----- |
| 0   | 1~4   | `vx`              | 底盘前后目标速度     | 底盘前后实际速度     | m/s   |
| 1   | 5~8   | `vy`              | 底盘左右目标速度     | 底盘左右实际速度     | m/s   |
| 2   | 9~12  | `vw`              | 底盘目标角速度      | 底盘实际角速度      | rad/s |
| 3   | 13~16 | `front_lift`      | 前轮抬升目标位置     | 前轮抬升实际位置     | m     |
| 4   | 17~20 | `rear_lift`       | 后轮抬升目标位置     | 后轮抬升实际位置     | m     |
| 5   | 21~24 | `kfs_lift`        | KFS 升降目标位置   | KFS 升降实际位置   | m     |
| 6   | 25~28 | `kfs_root_rotate` | KFS 根部目标角度   | KFS 根部实际角度   | rad   |
| 7   | 29~32 | `kfs_tip_rotate`  | KFS 末端目标角度（0 初始，工作方向为负） | KFS 末端实际角度（0 初始） | rad   |
| 8   | 33~36 | `kfs_grip`        | KFS 夹爪目标开度（0 闭合） | KFS 夹爪实际开度（0 闭合） | m     |
| 9   | 37~40 | `weapon_rotate`   | 武器目标角度       | 武器实际角度       | rad   |
| 10  | 41~44 | `weapon_grip`     | 武器目标开合位置     | 武器实际开合位置     | m     |
| 11  | 45~48 | `odometry_x`      | TX 中不存在         | 下位机里程计 X       | m     |
| 12  | 49~52 | `odometry_y`      | TX 中不存在         | 下位机里程计 Y       | m     |
| 13  | 53~56 | `odometry_yaw`    | TX 中不存在         | 下位机里程计偏航角    | rad   |


### 1. 上位机到下位机：命令协议

串口节点汇总 ROS 2 控制话题的最新目标值，默认以 50 Hz 发送完整命令帧。实际发送的原始帧同时发布到：

```text
/r2/serial/raw_tx  std_msgs/msg/String
```

### 2. 下位机到上位机：反馈协议

下位机按相同浮点字段顺序返回反馈帧，并在帧尾前追加按钮字节。串口节点校验帧头、
59 字节固定长度、帧尾、按钮值和浮点数有效性后，发布以下反馈话题：


| 反馈内容       | ROS 2 话题                          | 消息类型                                   |
| ---------- | --------------------------------- | -------------------------------------- |
| 底盘实际速度     | `/r2/velocity_feedback`           | `geometry_msgs/msg/Twist`              |
| 前后轮抬升位置    | `/r2/lift/position_feedback`      | `robot_r2_interfaces/msg/LiftFeedback` |
| KFS 升降位置   | `/r2/kfs_lift/feedback`           | `std_msgs/msg/Float64`                 |
| KFS 根部角度   | `/r2/gripper/rotate_feedback`     | `std_msgs/msg/Float64`                 |
| KFS 末端角度   | `/r2/gripper/tip_rotate_feedback` | `std_msgs/msg/Float64`                 |
| KFS 夹爪开合位置 | `/r2/gripper/grip_feedback`       | `std_msgs/msg/Float64`                 |
| 武器角度       | `/r2/weapon/rotate_feedback`      | `std_msgs/msg/Float64`                 |
| 武器开合位置     | `/r2/weapon/grip_feedback`        | `std_msgs/msg/Float64`                 |
| 物理按钮状态     | `/r2/serial/button`               | `std_msgs/msg/Bool`                    |

`all_step_control` 对第一次按下启动 5 秒屏蔽；5 秒后若阶段仍在执行则继续忽略，
阶段空闲后恢复检测。恢复后收到的下一条 `1` 可以再次执行当前所选阶段。总控状态发布
在 `/r2/all_step/status`（`robot_r2_interfaces/msg/AllStepStatus`）。


协议只反馈前、后两组抬升位置，因此发布 `LiftFeedback` 时同一组左右轮使用相同值。原始反馈帧同时发布到：

```text
/r2/serial/raw_rx  std_msgs/msg/String
```

串口无法打开时，节点在本轮连续失败的第一次 WARN 中发布一次
`/r2/system/abort`，后续重连失败不会重复发布。串口成功打开后会重新武装；如果之后
再次断开且无法打开，则会为新一轮故障再发布一次中止消息。

## 调试

查看完整串口发送帧：

```bash
ros2 topic echo /r2/serial/raw_tx --field data --full-length --once
```

查看完整串口反馈帧：

```bash
ros2 topic echo /r2/serial/raw_rx --field data --full-length --once
```
