# mipi_camera

ROS 2 driver for the IMX219 CSI cameras connected to the Jetsons.
It uses a C++ GStreamer appsink with NVIDIA Argus rather than OpenCV camera
capture, avoiding Python serialization overhead for raw image frames.

The image topics use `robot_r2_interfaces/CameraFrame`, not
`sensor_msgs/Image`. Its payload is a bounded byte sequence with a maximum of
24,245,760 bytes (3280 x 2464 x 3). The driver reserves that capacity once and
reuses the same message buffer at the configured, fixed resolution. This
version intentionally does not use the ROS loaned-message API.

The stable device alias describes the physical Jetson MIPI connector:

- `/dev/mipi_tip`: the single weapon-tip camera on real2 CSI-A or CSI-B

The configured device is resolved from its udev symlink to the current
`/dev/videoN` target, then mapped to Argus `sensor-id=N`.

The real2 bringup starts the single physical camera node. The common `mode`
parameter is one `[width, height, framerate]` array. It
must be one of the IMX219 modes detected on the Jetson:

- `[3280, 2464, 21]`
- `[3280, 1848, 28]`
- `[1920, 1080, 30]`
- `[1640, 1232, 30]`
- `[1280, 720, 60]`

Each MIPI camera instance publishes generic topics internally:

- `/r2/mipi_camera/image_raw` (`robot_r2_interfaces/CameraFrame`)
- `/r2/mipi_camera/image_raw/debug` (`sensor_msgs/Image`, disabled by default)
- `/r2/mipi_camera/camera_info`

The real2 bringup remaps them to:

- `/r2/tip_camera/image_raw`
- `/r2/tip_camera/image_raw/debug`
- `/r2/tip_camera/camera_info`

The tip camera is wired to one CSI connector on real2 (Jetson2). It publishes
`CameraFrame` on
`/r2/tip_camera/image_raw` for the tip-detection upstream (`/r2/tip/roi`).
`real1.launch.py` does not start a MIPI camera.

The `CameraInfo` messages contain the image dimensions but no calibration
matrix until the cameras have been calibrated.

`real2.launch.py` selects `rmw_fastrtps_cpp`, loads the shared Fast DDS profile
installed by `robot_r2_interfaces`, and directly starts the camera driver.
The bounded `CameraFrame` stream and optional `sensor_msgs/Image` debug stream
both use Best Effort, Keep Last 1, Volatile QoS. Data Sharing is `AUTOMATIC`.

```bash
ros2 launch bringup real2.launch.py
```

Debug image publication is controlled by the dynamic
`visualization_enabled` parameter and is disabled by default. It can be
changed without restarting the camera node:

```bash
ros2 param set /tip_mipi_camera visualization_enabled true
```

Normal processing nodes subscribe to the bounded topic.

## udev aliases

The `udev/99-mipi-cameras.rules` reference file maps either real2 CSI connector
(`9-0010` or `10-0010`) to `/dev/mipi_tip`. This is unambiguous because real2
has only one MIPI camera. real1 no longer has a MIPI camera and does not need
this rule.

Install the rule on real2 and reload udev:

```bash
sudo cp udev/99-mipi-cameras.rules /etc/udev/rules.d/99-mipi-cameras.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
```
