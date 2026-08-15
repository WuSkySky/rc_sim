# mipi_camera

ROS 2 driver for the IMX219 CSI cameras connected to the Jetsons.
It uses a C++ GStreamer appsink with NVIDIA Argus rather than OpenCV camera
capture, avoiding Python serialization overhead for raw image frames.

The image topics use `robot_r2_interfaces/CameraFrame`, not
`sensor_msgs/Image`. Its payload is a bounded byte sequence with a maximum of
24,245,760 bytes (3280 x 2464 x 3). The driver reserves that capacity once and
reuses the same message buffer at the configured, fixed resolution. This
version intentionally does not use the ROS loaned-message API.

The stable device aliases describe the physical Jetson MIPI connectors,
independently of Robot R2:

- `/dev/mipi_left`: left MIPI connector (`imx219 9-0010`, real2)
- `/dev/mipi_right`: right MIPI connector (`imx219 10-0010`, real2)
- `/dev/mipi_tip`: weapon-tip MIPI connector (`imx219 9-0010`, real1)

The configured device is resolved from its udev symlink to the current
`/dev/videoN` target, then mapped to Argus `sensor-id=N`.

The real2 bringup starts both physical camera nodes. The common `mode`
parameter is one `[width, height, framerate]` array shared by both nodes. It
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

- `/r2/left_camera/image_raw`
- `/r2/left_camera/image_raw/debug`
- `/r2/left_camera/camera_info`
- `/r2/right_camera/image_raw`
- `/r2/right_camera/image_raw/debug`
- `/r2/right_camera/camera_info`

`real1.launch.py` starts the single weapon-tip camera and remaps it to:

- `/r2/tip_camera/image_raw`
- `/r2/tip_camera/image_raw/debug`
- `/r2/tip_camera/camera_info`

The tip camera uses the same IMX219 model as left/right and is wired to the
`9-0010` connector on real1 (Jetson1). It publishes `CameraFrame` on
`/r2/tip_camera/image_raw` for the tip-detection upstream (`/r2/tip/roi`).

The `CameraInfo` messages contain the image dimensions but no calibration
matrix until the cameras have been calibrated.

`real2.launch.py` selects `rmw_fastrtps_cpp`, loads the shared Fast DDS profile
installed by `robot_r2_interfaces`, and directly starts the two driver nodes.
Image QoS is Best Effort, Keep Last 1, Volatile; Data Sharing is `AUTOMATIC`.

```bash
ros2 launch bringup real2.launch.py
```

Debug image publication is controlled by the dynamic
`visualization_enabled` parameter and is disabled by default. It can be
changed without restarting either camera node:

```bash
ros2 param set /left_mipi_camera visualization_enabled true
ros2 param set /right_mipi_camera visualization_enabled true
```

Normal processing nodes subscribe to the bounded topic.

## udev aliases

The `udev/99-mipi-cameras.rules` reference file maps the two connectors to the
left/right aliases used by real2. Because the `9-0010` connector is the
weapon-tip camera on real1 (Jetson1) but the left camera on real2, the aliases
are host-specific:

- real2 (`/etc/udev/rules.d/99-mipi-cameras.rules`): `9-0010` -> `mipi_left`,
  `10-0010` -> `mipi_right`.
- real1 (`/etc/udev/rules.d/99-mipi-cameras.rules`): `9-0010` -> `mipi_tip`.

Install the matching rule on each host and reload udev:

```bash
sudo cp udev/99-mipi-cameras.rules /etc/udev/rules.d/99-mipi-cameras.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
```

