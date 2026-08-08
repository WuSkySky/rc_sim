# hik_camera

ROS 2 Humble driver for HIKROBOT USB3 industrial cameras. The package carries
the x86_64 and aarch64 MVS runtime libraries migrated from the Jetson2 driver.
At build time CMake installs only the libraries for the current architecture.

The node publishes fixed generic topics. Robot-specific camera roles are
configured only with launch remapping:

- `/r2/hik_camera/image_raw` (`robot_r2_interfaces/CameraFrame`, BGR8)
- `/r2/hik_camera/image_raw/debug` (`sensor_msgs/Image`, disabled by default)
- `/r2/hik_camera/camera_info` (`sensor_msgs/CameraInfo`)

Image QoS is Best Effort, Keep Last 1, Volatile. `CameraInfo` contains the
current dimensions but no calibration matrix until this camera and lens have
been calibrated.

## USB permission

Install the supplied udev rule once on every deployment host, reload the
rules, and reconnect the camera:

```bash
sudo install -m 0644 \
  src/sim_to_real/hik_camera/udev/80-drivers-SDK-2bdf.rules \
  /etc/udev/rules.d/80-drivers-SDK-2bdf.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
```

The runtime user must belong to `plugdev`.

## Run with default topics

```bash
ros2 launch hik_camera hik_camera.launch.py
```

The parameters are installed from `config/hik_camera.yaml`. Exposure, gain,
frame rate, acquisition timeout, failure threshold, and debug publication all
support validated runtime updates. For example:

```bash
ros2 param set /hik_camera exposure_time_us 4000.0
ros2 param set /hik_camera visualization_enabled true
```

`bringup real2.launch.py` starts this node as `front_hik_camera` and remaps its
three topics to `/r2/front_camera/*`.
