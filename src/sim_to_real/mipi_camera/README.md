# mipi_camera

ROS 2 driver for the two IMX219 CSI cameras connected to the Jetson.
It uses a C++ GStreamer appsink with NVIDIA Argus rather than OpenCV camera
capture, avoiding Python serialization overhead for raw image frames.

The image topics use `robot_r2_interfaces/CameraFrame`, not
`sensor_msgs/Image`. Its payload is a bounded byte sequence with a maximum of
24,245,760 bytes (3280 x 2464 x 3). The driver reserves that capacity once and
reuses the same message buffer at the configured, fixed resolution. This
version intentionally does not use the ROS loaned-message API.

The stable device aliases describe the physical Jetson MIPI connectors,
independently of Robot R2:

- `/dev/mipi_left`: left MIPI connector (`imx219 9-0010`)
- `/dev/mipi_right`: right MIPI connector (`imx219 10-0010`)

The configured device is resolved from its udev symlink to the current
`/dev/videoN` target, then mapped to Argus `sensor-id=N`.

The launch file starts both physical cameras. The common `mode` parameter is
one `[width, height, framerate]` array shared by both nodes. It must be one of
the IMX219 modes detected on the Jetson:

- `[3280, 2464, 21]`
- `[3280, 1848, 28]`
- `[1920, 1080, 30]`
- `[1640, 1232, 30]`
- `[1280, 720, 60]`

Each MIPI camera instance publishes generic topics internally:

- `/r2/mipi_camera/image_raw` (`robot_r2_interfaces/CameraFrame`)
- `/r2/mipi_camera/image_raw/debug` (`sensor_msgs/Image`, disabled by default)
- `/r2/mipi_camera/camera_info`

The launch file remaps them to:

- `/r2/left_camera/image_raw`
- `/r2/left_camera/image_raw/debug`
- `/r2/left_camera/camera_info`
- `/r2/right_camera/image_raw`
- `/r2/right_camera/image_raw/debug`
- `/r2/right_camera/camera_info`

The `CameraInfo` messages contain the image dimensions but no calibration
matrix until the cameras have been calibrated.

`mipi_camera.launch.py` selects `rmw_fastrtps_cpp` and loads the shared Fast
DDS profile installed by `robot_r2_interfaces`. Image QoS is Best Effort,
Keep Last 1, Volatile; Data Sharing is `AUTOMATIC`.

```bash
ros2 launch mipi_camera mipi_camera.launch.py
```

Debug image publication is controlled by the dynamic
`visualization_enabled` parameter and is disabled by default. It can be
changed without restarting either camera node:

```bash
ros2 param set /left_mipi_camera visualization_enabled true
ros2 param set /right_mipi_camera visualization_enabled true
```

Normal processing nodes subscribe to the bounded topic.
