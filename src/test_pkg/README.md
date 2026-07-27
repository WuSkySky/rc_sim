# test_pkg

This is an `ament_cmake` + `ament_cmake_python` mixed package. The existing
Python commands remain available:

```bash
ros2 run test_pkg move_to_center_test
ros2 run test_pkg step_traverse_test
ros2 run test_pkg rotate_90_servo_test
```

## Camera benchmark

`camera_benchmark` compares the old `sensor_msgs/Image` path (`standard`) with
the bounded `robot_r2_interfaces/CameraFrame` path (`bounded`). Both use Best
Effort, Keep Last 1, Volatile QoS.

Processing modes:

- `transport`: validates metadata and constructs a callback-scoped `cv::Mat`
  view without reading the complete frame.
- `opencv_mean`: runs `cv::mean` to force a complete OpenCV read.

The launch file loads the same Fast DDS Data Sharing profile as the camera
driver. One left-camera bounded run:

```bash
ros2 launch test_pkg camera_benchmark.launch.py \
  message_type:=bounded processing_mode:=transport
```

Run both camera subscribers by enabling the right side:

```bash
ros2 launch test_pkg camera_benchmark.launch.py \
  message_type:=bounded processing_mode:=opencv_mean \
  left:=true right:=true
```

For a baseline against an old camera publisher:

```bash
ros2 launch test_pkg camera_benchmark.launch.py \
  message_type:=standard processing_mode:=transport
```

Defaults are 3 seconds of warmup and 20 seconds of measurement. Override them
with `warmup_sec` and `duration_sec`.

Each process prints `RESULT` lines containing received FPS, latency,
callback time, metadata errors, subscriber CPU/RSS, and `/dev/shm` usage.
`CameraFrame` also reports sequence loss. `sensor_msgs/Image` cannot report
true sequence loss because it has no sequence field.

For Jetson comparisons, run the same command for each resolution/camera-count
combination and use `pidstat -p <camera-pid>,<benchmark-pid> 1` for separate
publisher/subscriber CPU and RSS measurements. The benchmark's `/dev/shm`
numbers are system-wide and can be compared before, during, and after a run.
