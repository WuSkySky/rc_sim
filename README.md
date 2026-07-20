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

## 调试

重置随机 KFS：

```bash
ros2 service call /simulation/reset_kfs std_srvs/srv/Trigger "{}"
```

查看完整串口发送帧：

```bash
ros2 topic echo /r2/serial/raw_tx --field data --full-length --once
```
