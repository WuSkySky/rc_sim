# Fused CUDA/TensorRT KFS classifier

`kfs_detect_fused` replaces the three Python TensorRT classifier processes in
the real-camera launch. It keeps the existing per-camera image, raw result,
processed result, debug image, and `GetKfsType` service names.
`robot_r2_detect` still owns the shared model, YAML, launch file, and legacy
Python classifier sources, while the active real-camera launch executes only
this C++ node. The C++ inference path does not call the Python classifier.

The node caches only the newest frame from each camera and runs one TensorRT
batch on a timer. It does not synchronize or wait for all cameras: one, two,
or three live cameras are all valid. A fixed batch-3 engine is padded by
duplicating the first active tensor; padding results are discarded.

Preprocessing is implemented by a CUDA kernel. It directly resizes the complete
post-crop image to the square model input, then performs BGR-to-RGB conversion
and ImageNet normalization. It does not apply an additional center crop. The
source image upload, CUDA preprocessing, TensorRT enqueue, and output copy share
one CUDA stream. Buffers grow only when necessary and are reused.

Before GPU upload, the node crops the source image horizontally. The default
`horizontal_crop_ratio: 0.2` removes one fifth from both the left and right,
leaving the centered three fifths at the original height. Processed-result
dimensions and debug images describe this cropped input. The ratio can be
changed at runtime and must remain in `[0.0, 0.5)`.

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

- `model_path`, `model_input_size`, `model_class_names`, `model_mean`, and
  `model_std`: TensorRT engine and its external metadata.
- `conf`: minimum confidence for processed detections.
- `inference_rate`: maximum fused batch frequency.
- `frame_stale_timeout_sec`: ignore old cached frames.
- `horizontal_crop_ratio`: fraction removed from each horizontal side.
- `visualization_enabled_front`, `visualization_enabled_left`,
  `visualization_enabled_right`: per-camera annotated debug images, disabled
  by default. Each publishes on `/r2/detection/{front,left,right}/debug` only
  while enabled and subscribed.
- `default_vote_timeout_sec`: fallback timeout used by `GetKfsType` requests.

All parameters declared by `kfs_detect_fused` support atomic runtime updates.
Changing model or preprocessing metadata first builds a replacement TensorRT
classifier; an invalid replacement is rejected while the active classifier and
parameter values remain unchanged. Changing `inference_rate` replaces the timer,
and lightweight settings use one immutable configuration snapshot per batch.

For example, restore the full-width image or return to the default crop at
runtime with:

```bash
ros2 param set /kfs_detect_fused horizontal_crop_ratio 0.0
ros2 param set /kfs_detect_fused horizontal_crop_ratio 0.2
ros2 param set /kfs_detect_fused conf 0.65
ros2 param set /kfs_detect_fused visualization_enabled_front true
ros2 param set /kfs_detect_fused visualization_enabled_left true
ros2 param set /kfs_detect_fused visualization_enabled_right true
ros2 param set /kfs_detect_fused inference_rate 20.0
```

If one camera stops, its output simply stops updating; the other cameras keep
running. `GetKfsType` for the missing camera times out normally while the other
two services remain usable.
