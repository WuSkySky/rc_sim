# C++ KFS vision nodes

`kfs_roi` extracts the red or blue KFS panel used by alignment. It applies
strict HSV filtering, a configurable morphological opening, largest-component
selection, and independent row/column projection thresholds. The node
preserves the `/r2/kfs/roi` result interface and publishes only ROI
visualization on `/r2/kfs/roi/debug`.

All ROI thresholds support runtime updates. Their defaults are stored in
`robot_r2_detect/config/kfs_roi.yaml`; camera selection remains a launch
remapping of `/r2/front_camera/image_raw`. `column_threshold_ratio` and
`row_threshold_ratio` independently tune the X and Y bounds.

When visualization is enabled, `/r2/kfs/roi/debug` is a six-stage `3x2`
composite containing the annotated source, raw HSV union, morphological-open
mask, largest component, axis thresholds/bounds, and final cropped ROI. Each
tile is half the source width and height, so the composite remains about 1.5
times the source width and the same height.

## Fused classifier

`kfs_detect_fused` replaces the three Python TensorRT classifier processes in
the real-camera launch. It keeps the existing per-camera image, raw result,
processed result, debug image, and `GetKfsType` service names.

The node caches only the newest frame from each camera and runs one TensorRT
batch on a timer. It does not synchronize or wait for all cameras: one, two,
or three live cameras are all valid. A fixed batch-3 engine is padded by
duplicating the first active tensor; padding results are discarded.

Preprocessing is implemented by a CUDA kernel and matches the previous
Resize(short side), CenterCrop, BGR-to-RGB, ImageNet normalization pipeline.
The source image upload, CUDA preprocessing, TensorRT enqueue, and output copy
share one CUDA stream. Buffers grow only when necessary and are reused.

## Jetson build

Install the normal JetPack TensorRT development packages, then build both the
interface, Python detection, and C++ detection packages:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_r2_detect robot_r2_detect_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The default CUDA architecture is `87` for Jetson Orin. The TensorRT engine must
be rebuilt on the target Jetson when its JetPack/TensorRT version changes.

## Runtime

```bash
ros2 launch robot_r2_detect kfs_detect_multi.launch.py
```

Useful parameters in `robot_r2_detect/config/kfs_detect.yaml`:

- `inference_rate`: maximum fused batch frequency.
- `frame_stale_timeout_sec`: ignore old cached frames.
- `visualization_enabled`: debug images are disabled by default.

If one camera stops, its output simply stops updating; the other cameras keep
running. `GetKfsType` for the missing camera times out normally while the other
two services remain usable.
