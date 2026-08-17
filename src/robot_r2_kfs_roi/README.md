# KFS ROI

`robot_r2_kfs_roi` provides the CPU-only OpenCV `kfs_roi` node used by
simulation and real-camera ROI extraction.

The node subscribes to `/r2/front_camera/image_raw`, publishes the qualified
left/right columns, their lowest mask pixels, and the horizontal center offset
on `/r2/kfs/roi`, and optionally publishes a two-stage visualization (stage 1
source image and stage 4 column-selection mask) on `/r2/kfs/roi/debug`. The
output type is
`robot_r2_interfaces/msg/AlignmentDetection`; the message never contains image
pixels. Camera selection remains a launch remapping.

`center_offset_px` (default `0`) shifts the published KFS center by the given
number of pixels: it is added to both `center_u` and `center_offset_x`, so the
alignment consumers aim at the adjusted center. The debug image draws the
image center line (cyan), the offset-adjusted center line (red), the qualified
ROI bounds (green), and the left/right lowest mask pixels (magenta/yellow
crosses). Its text shows the detected offset, configured shift, and final
published offset separately.

All thresholds support validated runtime parameter updates. Their defaults
are installed from `config/kfs_roi.yaml`.

## Build this package

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_r2_kfs_roi --symlink-install
source install/setup.bash
```

The CUDA/TensorRT classifier remains in `robot_r2_detect_cpp` and is not a
dependency of this package.

## Build the workspace on a development PC

On a computer without the Jetson CUDA/TensorRT development environment, ignore
the fused classifier package while building the complete workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-ignore robot_r2_detect_cpp
source install/setup.bash
```
