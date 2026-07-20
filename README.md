# rc_sim

## 启动

```bash
source install/setup.bash
ros2 launch bringup sim.launch.py
```

实车控制与串口：

```bash
source install/setup.bash
ros2 launch bringup real.launch.py
```

## 阶段二

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

## 常用服务

位置伺服，坐标单位为米，偏航角单位为弧度：

```bash
ros2 service call /r2/move_to_pose robot_r2_interfaces/srv/MoveToPose \
  "{x: 0.0, y: 0.0, yaw: 1.5708, position_tolerance: 0.0, yaw_tolerance: 0.0, timeout_sec: 20.0}"
```

KFS 检测：

```bash
ros2 service call /r2/detection/get_type \
  robot_r2_interfaces/srv/GetKfsType \
  "{sample_count: 10, timeout_sec: 10.0}"
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

跨越台阶，方向：`0=上`、`1=下`，距离允许为负数。

```bash
ros2 service call /r2/step_traverse robot_r2_interfaces/srv/TraverseStep \
  "{direction: 0, distance_to_step: 0.2}"
```

四轮抬升：

```bash
ros2 service call /r2/lift/set robot_r2_interfaces/srv/SetLift \
  "{front_lift: 0.2, rear_lift: 0.2, tolerance: 0.0, timeout_sec: 15.0}"
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

| 序号 | 字节 | 字段 | 下发含义 | 反馈含义 | 单位 |
| ---: | ---: | --- | --- | --- | --- |
| 0 | 1~4 | `vx` | 底盘前后目标速度 | 底盘前后实际速度 | m/s |
| 1 | 5~8 | `vy` | 底盘左右目标速度 | 底盘左右实际速度 | m/s |
| 2 | 9~12 | `vw` | 底盘目标角速度 | 底盘实际角速度 | rad/s |
| 3 | 13~16 | `front_lift` | 前轮抬升目标位置 | 前轮抬升实际位置 | m |
| 4 | 17~20 | `rear_lift` | 后轮抬升目标位置 | 后轮抬升实际位置 | m |
| 5 | 21~24 | `kfs_lift` | KFS 升降目标位置 | KFS 升降实际位置 | m |
| 6 | 25~28 | `kfs_root_rotate` | KFS 根部目标角度 | KFS 根部实际角度 | rad |
| 7 | 29~32 | `kfs_tip_rotate` | KFS 末端目标角度 | KFS 末端实际角度 | rad |
| 8 | 33~36 | `kfs_grip` | KFS 夹爪目标开合位置 | KFS 夹爪实际开合位置 | m |
| 9 | 37~40 | `weapon_rotate` | 武器目标角度 | 武器实际角度 | rad |
| 10 | 41~44 | `weapon_grip` | 武器目标开合位置 | 武器实际开合位置 | m |

### 1. 上位机到下位机：命令协议

串口节点汇总 ROS 2 控制话题的最新目标值，默认以 50 Hz 发送完整命令帧。实际发送的原始帧同时发布到：

```text
/r2/serial/raw_tx  std_msgs/msg/String
```

### 2. 下位机到上位机：反馈协议

下位机按相同字段顺序返回完整反馈帧。串口节点校验帧头、固定长度、帧尾和浮点数有效性后，发布以下反馈话题：

| 反馈内容 | ROS 2 话题 | 消息类型 |
| --- | --- | --- |
| 底盘实际速度 | `/r2/velocity_feedback` | `geometry_msgs/msg/Twist` |
| 前后轮抬升位置 | `/r2/lift/position_feedback` | `robot_r2_interfaces/msg/LiftFeedback` |
| KFS 升降位置 | `/r2/kfs_lift/feedback` | `std_msgs/msg/Float64` |
| KFS 根部角度 | `/r2/gripper/rotate_feedback` | `std_msgs/msg/Float64` |
| KFS 末端角度 | `/r2/gripper/tip_rotate_feedback` | `std_msgs/msg/Float64` |
| KFS 夹爪开合位置 | `/r2/gripper/grip_feedback` | `std_msgs/msg/Float64` |
| 武器角度 | `/r2/weapon/rotate_feedback` | `std_msgs/msg/Float64` |
| 武器开合位置 | `/r2/weapon/grip_feedback` | `std_msgs/msg/Float64` |

协议只反馈前、后两组抬升位置，因此发布 `LiftFeedback` 时同一组左右轮使用相同值。原始反馈帧同时发布到：

```text
/r2/serial/raw_rx  std_msgs/msg/String
```

## 调试

重置随机 KFS：

```bash
ros2 service call /simulation/reset_kfs std_srvs/srv/Trigger "{}"
```

查看完整串口发送帧：

```bash
ros2 topic echo /r2/serial/raw_tx --field data --full-length --once
```

查看完整串口反馈帧：

```bash
ros2 topic echo /r2/serial/raw_rx --field data --full-length --once
```
