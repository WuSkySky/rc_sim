# KFS ROI

`robot_r2_kfs_roi` provides the CPU-only OpenCV `kfs_roi` node used by
simulation, YOLO testing, and real-camera ROI extraction.

The node subscribes to `/r2/front_camera/image_raw`, publishes detections on
`/r2/kfs/roi`, and optionally publishes a six-stage visualization on
`/r2/kfs/roi/debug`. Camera selection remains a launch remapping.

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
