# odin_data_postprocess

Odin 里程计数据的 ROS 2 后处理节点，包括：

- 发布校正后的 `odom -> base_link` TF；
- 将 `map -> base_link` 转发到 `/r2/pose_feedback`；
- 发布可通过服务重设的 `map -> odom` TF。

## `map_odom_tf_publisher` 调试

以下命令均在工作区根目录执行。

### 构建并加载环境

新增了自定义服务接口，首次使用前需要同时构建接口包和节点包：

```bash
colcon build --symlink-install --packages-select \
  robot_r2_interfaces odin_data_postprocess bringup
source install/setup.bash
```

### 单独启动节点

运行前需要确保系统中已经存在 `odom -> base_link` TF：

```bash
ros2 run odin_data_postprocess map_odom_tf_publisher \
  --ros-args \
  --params-file src/sim_to_real/odin/odin_data_postprocess/config/map_odom_tf_publisher.yaml
```

也可以启动完整实机系统，新节点已接入 `real.launch.py`：

```bash
ros2 launch bringup real.launch.py
```

实机启动使用 Odin 配置中的 `use_host_ros_time: 1`，使 Odin 里程计和
`map_odom_tf_publisher` 使用相同的主机 ROS 时间。不要改回未经对齐的设备时间，
否则 `map -> odom -> base_link` 可能因时间戳差距过大而无法组合查询。

### 检查节点、服务和 TF

```bash
ros2 node info /map_odom_tf_publisher
ros2 service type /r2/set_base_pose
ros2 topic echo /r2/pose_feedback --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

### 调用位姿重设服务

请求字段为 `x`、`y`、`z`、`roll`、`pitch`、`yaw`。平移单位为米，
RPY 单位为弧度。省略的字段使用 ROS 2 默认值 `0.0`。

将当前 `base_link` 对齐到 `map` 原点：

```bash
ros2 service call /r2/set_base_pose \
  robot_r2_interfaces/srv/SetBasePose '{}'
```

仅设置平面位置和朝向，其余字段保持默认的零：

```bash
ros2 service call /r2/set_base_pose \
  robot_r2_interfaces/srv/SetBasePose \
  '{x: 1.5, y: -0.5, yaw: 1.57079632679}'
```

设置完整六自由度位姿：

```bash
ros2 service call /r2/set_base_pose \
  robot_r2_interfaces/srv/SetBasePose \
  '{x: 1.0, y: 2.0, z: 0.3, roll: 0.0, pitch: 0.1, yaw: -0.5}'
```

服务调用成功后，`map -> odom` 会立即更新，使调用时刻的
`base_link` 与请求位姿重合。若 `odom -> base_link` 尚不可用，服务会返回失败，
已生效的 `map -> odom` 不会改变。

`/r2/pose_feedback` 的 `header.frame_id` 为 `map`，因此服务更新也会反映到
底盘定点控制和订阅该话题的上层流程中。

该节点面向 Odin 的 `custom_map_mode: 0` 里程计模式使用。不要与其他
`map -> odom` 发布者或 Odin 重定位模式同时启用。
