# rc_sim

## 编译

普通开发电脑没有 Jetson 的 CUDA/TensorRT 环境时，跳过融合检测包；纯 OpenCV
的 KFS ROI 节点仍会由 `robot_r2_kfs_roi` 正常构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-ignore robot_r2_detect_cpp
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

GUI 窗口获得焦点时，可按住实体键盘的 `W/A/S/D/Q/E` 控制底盘，松开时
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
Fast DDS 配置。real1 负责控制、串口、KFS 对齐、Odin 后摄像头及 LED 检测：

```bash
source install/setup.bash
ros2 launch bringup real1.launch.py
```

real2 负责前置海康 USB3 相机、左右 MIPI 相机、前/左/右三路融合 KFS 识别和
KFS ROI。前置海康相机默认发布自定义 `/r2/front_camera/image_raw`，
ROI 节点默认直接使用该话题。两个节点的标准调试图均默认关闭，可通过
`visualization_enabled` 动态开启。融合节点不会等待未接入的相机，因此只有部分相机
在线时也能正常推理：

```bash
source install/setup.bash
ros2 launch bringup real2.launch.py
```

real2 的 ROI 默认使用前置海康相机，real1 的阶段任务默认使用左侧 KFS 识别
服务。需要改用右侧时分别执行：

```bash
ros2 launch bringup real2.launch.py \
  roi_image_topic:=/r2/right_camera/image_raw
ros2 launch bringup real1.launch.py \
  kfs_get_type_service:=/r2/detection/right/get_type
```

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
| 完整阶段二      | `/r2/stage_two`              | `robot_r2_interfaces/srv/StageTwo`            | 仿真、实机 |
| 阶段 2.1     | `/r2/stage_two_point_one`    | `robot_r2_interfaces/srv/StageTwoPointOne`    | 仿真、实机 |
| 阶段 2.2     | `/r2/stage_two_point_two`    | `robot_r2_interfaces/srv/StageTwoPointTwo`    | 仿真、实机 |
| 底盘位置伺服     | `/r2/move_to_pose`           | `robot_r2_interfaces/srv/MoveToPose`          | 仿真、实机 |
| 重置或设置里程计位姿 | `/r2/set_base_pose`          | `robot_r2_interfaces/srv/SetBasePose`         | 仅实机   |
| 四轮抬升       | `/r2/lift/set`               | `robot_r2_interfaces/srv/SetLift`             | 仿真、实机 |
| 跨越台阶       | `/r2/step_traverse`          | `robot_r2_interfaces/srv/TraverseStep`        | 仿真、实机 |
| KFS 视觉对齐   | `/r2/align_to_kfs`           | `robot_r2_interfaces/srv/AlignToKFS`          | 仿真、实机 |
| KFS 类型检测   | `/r2/detection/{front,left,right}/get_type` | `robot_r2_interfaces/srv/GetKfsType` | 仅实机 |
| LED 状态检测   | `/r2/led_detection/detect`   | `robot_r2_interfaces/srv/DetectLed`           | 仿真、实机 |
| KFS 装载     | `/r2/kfs/load`               | `robot_r2_interfaces/srv/LoadKfs`             | 仿真、实机 |
| KFS 释放     | `/r2/kfs/release`            | `robot_r2_interfaces/srv/ReleaseKfs`          | 仿真、实机 |
| KFS 夹爪升降   | `/r2/kfs_lift`               | `robot_r2_interfaces/srv/SetKfsLift`          | 仿真、实机 |
| KFS 夹爪根部旋转 | `/r2/gripper/set_rotate`     | `robot_r2_interfaces/srv/SetGripperRotate`    | 仿真、实机 |
| KFS 夹爪末端旋转 | `/r2/gripper/set_tip_rotate` | `robot_r2_interfaces/srv/SetGripperTipRotate` | 仿真、实机 |
| KFS 夹爪开合   | `/r2/gripper/set_grip`       | `robot_r2_interfaces/srv/SetGripperGrip`      | 仿真、实机 |
| 重新随机摆放 KFS | `/simulation/reset_kfs`      | `std_srvs/srv/Trigger`                        | 仅仿真   |


### 阶段二任务

完整执行阶段 2.1 → 2.2。假 KFS 决策：`1=左`，`2=右`。

```bash
ros2 service call /r2/stage_two robot_r2_interfaces/srv/StageTwo \
  "{fake_kfs_decision: 1}"
```

单独执行阶段 2.1：

```bash
ros2 service call /r2/stage_two_point_one \
  robot_r2_interfaces/srv/StageTwoPointOne "{loaded_count: 0}"
```

单独执行阶段 2.2：

```bash
ros2 service call /r2/stage_two_point_two \
  robot_r2_interfaces/srv/StageTwoPointTwo \
  "{fake_kfs_decision: 1, loaded_count: 0}"
```

### 底盘、里程计与抬升

底盘速度控制使用 Topic 而不是 Service。`linear.x` 为前后速度、`linear.y` 为
左右速度，`angular.z` 为旋转角速度：

```bash
ros2 topic pub -r 20 /r2/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

底盘位置伺服的坐标单位为米，偏航角单位为弧度：

```bash
ros2 service call /r2/move_to_pose robot_r2_interfaces/srv/MoveToPose \
  "{x: 0.0, y: 0.0, yaw: 1.5708, position_tolerance: 0.0, yaw_tolerance: 0.0, timeout_sec: 20.0}"
```

实机重置里程计位姿，将调用时刻的 `base_link` 对齐到 `map` 原点：

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

该服务由 `real1.launch.py` 启动的 `odometry_postprocess` 提供。它根据 Odin
里程计和外参更新 `map -> odom`，从而校正 `/r2/pose_feedback` 和底盘位置伺服
使用的地图位姿；不会清空 Odin 发布的原始里程计数据。尚未收到 Odin 里程计时，
服务会返回失败。

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
前相机；实机的前置海康相机和 ROI 节点都由 real2 启动，ROI 默认使用
`/r2/front_camera/image_raw`。real1 不处理该图像链路，只有对齐节点订阅
`/r2/kfs/roi`。该话题只携带时间戳、源帧序号、有效性、左右边界、左右
边界列的最下方掩膜点和横向中心偏差，不包含图像像素：

使用 `kfs_alignment.yaml` 中的默认容差和超时时间执行对齐：

```bash
ros2 service call /r2/align_to_kfs \
  robot_r2_interfaces/srv/AlignToKFS \
  "{pixel_tolerance: 0.0, timeout_sec: 0.0}"
```

也可以在单次调用中指定横向像素容差和超时时间：

```bash
ros2 service call /r2/align_to_kfs \
  robot_r2_interfaces/srv/AlignToKFS \
  "{pixel_tolerance: 20.0, timeout_sec: 10.0}"
```

`pixel_tolerance` 是允许的图像横向像素误差，`timeout_sec` 是整个对齐过程的
最大等待时间。两项都填写 `0.0` 时使用 `kfs_alignment.yaml` 中的默认值。
返回值中的 `success` 表示是否连续稳定达到容差，`final_offset_x` 是最后一次
有效检测的横向像素误差。调用前应确保目标红色或蓝色区域位于当前 ROI 输入相机
的视野内。

对齐容差、稳定帧数、默认超时、PID 和速度限幅均支持运行时动态修改。例如：

```bash
ros2 param set /kfs_alignment output_limit 0.05
ros2 param set /kfs_alignment pixel_tolerance 10
```

参数修改会整体校验后原子生效；无效值会被拒绝。服务请求中的容差或超时为
`0.0` 时，正在执行的任务会使用对应参数的最新值；请求中显式指定的正数不受
后续参数修改影响。

可视化默认关闭。所有可能发布调试图像的节点统一使用动态参数
`visualization_enabled`，可在运行时开启或关闭：

```bash
ros2 param set /kfs_roi visualization_enabled true
# 实机检测节点：
ros2 param set /kfs_detect_fused visualization_enabled true
ros2 param set /led_detect visualization_enabled true
ros2 param set /r2/front_camera_controller visualization_enabled true
ros2 param set /camera_frame_postprocess visualization_enabled true
ros2 param set /front_hik_camera visualization_enabled true
ros2 param set /left_mipi_camera visualization_enabled true
ros2 param set /right_mipi_camera visualization_enabled true
```

将最后的 `true` 改为 `false` 即可关闭。`kfs_roi` 的五阶段调试图像发布在
`/r2/kfs/roi/debug`；实机三路检测分别为
`/r2/detection/{front,left,right}/debug`；
LED 调试图像为 `/r2/led_detection/debug`。仿真前相机、Odin 后摄像头后处理、前置
海康相机以及左右 MIPI 相机也使用同一个动态参数控制各自的 `/debug` 图像。
这些调试话题均使用 `sensor_msgs/msg/Image`。

实机 KFS 类型检测，以前相机为例：

```bash
ros2 service call /r2/detection/front/get_type \
  robot_r2_interfaces/srv/GetKfsType \
  "{sample_count: 10, timeout_sec: 10.0}"
```

前、左、右相机分别使用 `/r2/detection/{front,left,right}/get_type`，请求格式相同。
仿真启动文件不再启动 KFS 类型检测节点。

LED 状态检测。示例表示等待三个 LED 的状态稳定匹配“亮、灭、亮”：

```bash
ros2 service call /r2/led_detection/detect \
  robot_r2_interfaces/srv/DetectLed \
  "{target_states: [true, false, true]}"
```

KFS 装载，位置：`0=前方`、`1=上方`；方式：`0=标准`、`1=转移`。

```bash
ros2 service call /r2/kfs/load robot_r2_interfaces/srv/LoadKfs \
  "{mode: 0, load_method: 0}"
```

释放 KFS：

```bash
ros2 service call /r2/kfs/release robot_r2_interfaces/srv/ReleaseKfs "{}"
```

以下服务用于直接调试 KFS 夹爪机构。`position` 分别表示升降位置、根部角度、
末端角度和开合位置；升降及开合单位为米，角度单位为弧度。`tolerance` 和
`timeout_sec` 填写 `0.0` 时使用对应控制器配置中的默认值。夹爪开合位置
`0.0` 表示闭合，正值表示打开，`0.209` 表示完全打开。末端旋转以初始姿态
为 `0 rad`，沿工作旋转方向使用负值，范围为 `-π–0 rad`。

```bash
ros2 service call /r2/kfs_lift robot_r2_interfaces/srv/SetKfsLift \
  "{position: 0.0, tolerance: 0.0, timeout_sec: 0.0}"

ros2 service call /r2/gripper/set_rotate \
  robot_r2_interfaces/srv/SetGripperRotate \
  "{position: 0.0, tolerance: 0.0, timeout_sec: 0.0}"

ros2 service call /r2/gripper/set_tip_rotate \
  robot_r2_interfaces/srv/SetGripperTipRotate \
  "{position: -1.5708, tolerance: 0.0, timeout_sec: 0.0}"

ros2 service call /r2/gripper/set_grip \
  robot_r2_interfaces/srv/SetGripperGrip \
  "{position: 0.209, tolerance: 0.0, timeout_sec: 0.0}"
```

### 仿真场地

重新随机摆放 KFS：

```bash
ros2 service call /simulation/reset_kfs std_srvs/srv/Trigger "{}"
```

## 串口协议

串口默认使用 `115200 8N1`。下发和反馈均为固定 46 字节帧：

```text
0xAA | 11 × IEEE 754 binary32 | 0x55
```

- 字节 `0`：帧头 `0xAA`
- 字节 `1~44`：11 个连续的 32 位浮点数，无填充
- 字节 `45`：帧尾 `0x55`
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


### 1. 上位机到下位机：命令协议

串口节点汇总 ROS 2 控制话题的最新目标值，默认以 50 Hz 发送完整命令帧。实际发送的原始帧同时发布到：

```text
/r2/serial/raw_tx  std_msgs/msg/String
```

### 2. 下位机到上位机：反馈协议

下位机按相同字段顺序返回完整反馈帧。串口节点校验帧头、固定长度、帧尾和浮点数有效性后，发布以下反馈话题：


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


协议只反馈前、后两组抬升位置，因此发布 `LiftFeedback` 时同一组左右轮使用相同值。原始反馈帧同时发布到：

```text
/r2/serial/raw_rx  std_msgs/msg/String
```

## 调试

查看完整串口发送帧：

```bash
ros2 topic echo /r2/serial/raw_tx --field data --full-length --once
```

查看完整串口反馈帧：

```bash
ros2 topic echo /r2/serial/raw_rx --field data --full-length --once
```
