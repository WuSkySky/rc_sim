# yahboom_camera

ROS 2 driver for the Yahboom USB UVC front camera, captured through
GStreamer `v4l2src`, so the driver works on both the x86_64 development
machine and the Jetson hosts. Two camera generations are supported with one
config file each:

- Old model: Sunplus `SDYH-8P0P` (USB `1bcf:0b09`) — `config/yahboom_camera.yaml`
  (720p@30 default).
- New model: HD USB Camera (USB `05a3:9230`) — `config/yahboom_camera_hd.yaml`,
  selected by the standalone `yahboom_camera.launch.py` (1080p@30 default).

The node publishes fixed generic topics. Robot-specific camera roles are
configured only with launch remapping:

- `/r2/yahboom_camera/image_raw` (`robot_r2_interfaces/CameraFrame`, BGR8)
- `/r2/yahboom_camera/image_raw/debug` (`sensor_msgs/Image`, disabled by default)
- `/r2/yahboom_camera/camera_info` (`sensor_msgs/CameraInfo`)

Image QoS is Best Effort, Keep Last 1, Volatile. `CameraInfo` contains the
current dimensions but no calibration matrix until the camera and lens have
been calibrated.

## Supported modes

Old model (Sunplus `SDYH-8P0P`), MJPG all at 30 fps:

- `[1280, 720, 30]` (default; 720p keeps downstream KFS ROI/detection within 30 Hz)
- `[1920, 1080, 30]`
- `[800, 600, 30]`
- `[720, 1000, 30]`
- `[640, 480, 30]`
- `[640, 360, 30]`
- `[600, 600, 30]`
- `[320, 240, 30]`

New model (HD USB Camera `05a3:9230`), MJPG. GStreamer `v4l2src` negotiates
strictly against the intervals the device advertises, so a framerate that is
not listed for a size fails with `not-negotiated` (720p is 60 fps only):

- `[1920, 1080, 30]` (default in `yahboom_camera_hd.yaml`)
- `[1280, 720, 60]`, `[800, 600, 60]`
- `[1024, 768, 30]`, `[1280, 1024, 30]`
- `[640, 480, 120]`, `[320, 240, 120]`

YUYV (raw) tops out at 5 fps at 1080p and 10 fps at 720p, so the 30 fps front
camera configuration uses MJPG.

## Stable device alias

Install the supplied udev rule once on every deployment host so the camera gets
a stable `/dev/yahboom_front` alias instead of a host-dependent `/dev/videoN`:

```bash
sudo install -m 0644 \
  src/sim_to_real/yahboom_camera/udev/90-yahboom-camera.rules \
  /etc/udev/rules.d/90-yahboom-camera.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
```

The rule matches only nodes whose `ID_V4L_CAPABILITIES` contains `capture`.
This matters for UVC devices that expose both a video node and a metadata-only
node: the metadata node must never replace `/dev/yahboom_front`.

Then set `device: /dev/yahboom_front` in `config/yahboom_camera.yaml` (or pass
`-p device:=/dev/yahboom_front`). Until the rule is installed, use the current
node path, for example `/dev/video2` on the development machine.

## Run

The standalone launch selects `config/yahboom_camera_hd.yaml` (new model,
1920x1080@30); switch the `config` path in the launch to
`yahboom_camera.yaml` to run the old model at 720p@30.

```bash
ros2 launch yahboom_camera yahboom_camera.launch.py
```

Override the device or mode without editing the YAML:

```bash
ros2 run yahboom_camera yahboom_camera \
  --ros-args -p device:=/dev/video2 -p mode:=[1280,720,30]
```

`bringup real2.launch.py` starts the HD USB camera as
`front_yahboom_camera` and remaps its topics to `/r2/front_camera/*`.

## Parameters

All parameters are declared from `config/yahboom_camera.yaml`.

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `device` | string | `/dev/video2` (old) / `/dev/yahboom_front` (HD) | Video device node or stable udev alias. |
| `pixel_format` | string | `MJPG` | `MJPG` or `YUYV`. |
| `mode` | int array | `[1280, 720, 30]` (`yahboom_camera.yaml`) / `[1920, 1080, 30]` (`yahboom_camera_hd.yaml`) | `[width, height, framerate]`. |
| `visualization_enabled` | bool | `false` | Publish the `/debug` image. Dynamic. |

`visualization_enabled` supports validated runtime updates:

```bash
ros2 param set /front_yahboom_camera visualization_enabled true
```
