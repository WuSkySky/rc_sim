# 会话交接：HIK、real2 帧率与 alignment CUDA 迁移

更新时间：2026-08-11（Asia/Shanghai）

## 当前目标

`robot_r2_target_alignment` 从 CPU 到 Jetson CUDA 的迁移已经完成。远端
`jetson@10.42.0.3` 的工作区 `/home/jetson/workspaces/rc_sim` 已完成哈希核对、
单包构建以及 `real2 + alignment(test_mode)` 实机验证。

验证期间发现 Humble 的 Python `sensor_msgs/Image.data` setter 会逐字节校验
`bytes`，使每帧调试图构造耗时约 188 ms。现已改用其原生 `array('B')` 快速路径，
降至约 0.216 ms，并增加回归测试。默认可视化开启时，最终实测检测循环约
8.03 Hz、调试图约 7.04 Hz；日志无错误，验证进程均已清理。

随后已在本地修复限频逻辑并通过测试，但按用户要求暂不向 Jetson 同步。远端仍是
上述 8.03/7.04 Hz 验证时使用的旧限频实现，必须等用户明确允许后再同步和实测。

## 主机与路径

- 本地仓库：`/home/artorias/RC/rc_sim`
- Jetson：`jetson@10.42.0.3`
- Jetson 工作区：`/home/jetson/workspaces/rc_sim`
- ROS 2：Humble
- Jetson 型号：Orin；CUDA 12.6 可用
- Jetson 电源模式：`MAXN_SUPER`

## 已完成的 alignment CUDA 修改

本地修改文件：

1. `src/robot_r2_target_alignment/config/yolo_target_detector.yaml`
   - `model.device` 从 `cpu` 改为字符串 `"0"`，即第 0 块 CUDA GPU。
   - 当前部署已改为目标 Jetson 上导出的 FP16 TensorRT engine；精度固化在
     engine 中，不再使用运行时 `model.quantize` 参数。
2. `src/robot_r2_target_alignment/robot_r2_target_alignment/yolo_target_detector.py`
   - ROS 参数 `model.device` 的代码默认值从 `cpu` 改为 `"0"`。
   - 本地将限频基准改为推理开始时间，处理耗时不再与完整限频周期相加。
3. `src/robot_r2_target_alignment/README.md`
   - 增加默认 CUDA 设备和 CPU 回退说明。
4. `src/robot_r2_target_alignment/robot_r2_target_alignment/camera_frame.py`
   - 调试图数据改用 `array('B')` 构造，绕过 Humble 消息 setter 的逐字节校验。
5. `src/robot_r2_target_alignment/test/test_camera_frame.py`
   - 验证原生 uint8 数组类型、图像元数据与像素字节完全一致。
6. `src/robot_r2_target_alignment/robot_r2_target_alignment/detector_core.py`
   - 新增 ROS 无关的 start-to-start 等待时间计算函数。
7. `src/robot_r2_target_alignment/test/test_detector_core.py`
   - 覆盖首次推理、周期剩余等待和处理超时不追加等待三种情况。

本地文件 SHA-256：

```text
69849eed63e18ff85ba429f429d012bfdf9064fa14a4fbda2125fad041e6731d  src/robot_r2_target_alignment/config/yolo_target_detector.yaml
4e742589f6c0267047fa862f9759aa05ede29795c126eee1b3c9ea1fa6ab0a59  src/robot_r2_target_alignment/robot_r2_target_alignment/yolo_target_detector.py
60fc0f49c0f76598c6779942ed856a4bae0acc97f55beb0a116cf88254e3007d  src/robot_r2_target_alignment/robot_r2_target_alignment/detector_core.py
52d1ccb0e2e04a0be1ad258d186021da0a7e32111c496baf15e0358b8f196202  src/robot_r2_target_alignment/robot_r2_target_alignment/camera_frame.py
2ecab856673ef682807d64a067159290e1033795b3867027189ebf737f008d5a  src/robot_r2_target_alignment/test/test_detector_core.py
026a45396cec8157898c498d48601b5e274f73d078794e8729f26dca185d0d8a  src/robot_r2_target_alignment/test/test_camera_frame.py
1cc05e9a6a03b31892d0fa879f2cacb9fbfc133de6e3dae0711d9fca37669d8e  src/robot_r2_target_alignment/README.md
```

本地验证：

- `python3 -m compileall` 通过。
- 包内 12 个单元测试全部通过。
- 本机 pytest 需避免不兼容的全局 `launch_testing` 插件，并加入源码包路径：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=/home/artorias/RC/rc_sim/src/robot_r2_target_alignment:$PYTHONPATH \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q src/robot_r2_target_alignment/test
```

## 远端同步与构建状态

以下三个 `scp` 操作此前均返回成功：

```text
src/robot_r2_target_alignment/config/yolo_target_detector.yaml
src/robot_r2_target_alignment/robot_r2_target_alignment/yolo_target_detector.py
src/robot_r2_target_alignment/README.md
```

目标均为 Jetson 上对应的
`/home/jetson/workspaces/rc_sim/src/robot_r2_target_alignment/...`。

三个 CUDA 配置文件在首次远端验证时 SHA-256 一致。新增的调试图优化和测试也已
同步，远端哈希分别为：

```text
52d1ccb0e2e04a0be1ad258d186021da0a7e32111c496baf15e0358b8f196202  robot_r2_target_alignment/camera_frame.py
026a45396cec8157898c498d48601b5e274f73d078794e8729f26dca185d0d8a  test/test_camera_frame.py
```

远端 `colcon build --symlink-install --packages-select
robot_r2_target_alignment --allow-overriding robot_r2_target_alignment` 成功。远端包在
Git 中仍整体显示为 `?? src/robot_r2_target_alignment/`，不要清理或重置它。

注意：本地随后修改了 `yolo_target_detector.py`、`detector_core.py` 和
`test_detector_core.py` 来修复限频，但尚未同步。远端 `yolo_target_detector.py`
仍为旧哈希 `85912f1d...`，本地新哈希为 `4e742589...`。

## real2 + alignment 低帧率诊断结论

已在 Jetson 上用真实 HIK 相机和 `duantou.pt` 完成三组对照测量：

| 启动组合 | HIK 原始话题接收率 | alignment 标注图 |
|---|---:|---:|
| 仅 HIK | 29.94 Hz | — |
| `real2` | 24.88 Hz | — |
| `real2 + alignment`（CPU） | 21.62 Hz | 1.16 Hz |

关键证据：

- HIK-only 输出为 720×540、2×2 binning、29.94 Hz，说明相机硬件、USB 链路、
  binning 和 HIK 驱动不是低帧率根因。
- CPU 配置下 YOLO 占用约 96% 单核，标注图间隔约 860–906 ms。
- `real2` 同时启动 HIK、两路 MIPI、`kfs_detect_fused` 和 `kfs_roi`。
- `kfs_roi` 实测约 87%–102% 单核；两路 MIPI 各约 28%–35%；融合 KFS 约
  16%–19%。
- `kfs_roi` 配置为 30 Hz，并出现过 48–75 ms 大于 33.33 ms 周期的 overrun。
- 相机与 alignment 订阅 QoS 均为 BEST_EFFORT，没有 QoS 不兼容。
- 可视化框绘制仅约 0.73 ms，不是主要瓶颈。

真实模型离线基准（720×540 图像、`imgsz=640`）：

| 推理设备 | 单帧中位耗时 | 理论最大帧率 |
|---|---:|---:|
| CPU | 740.3 ms | 1.35 FPS |
| CUDA FP32 | 34.8 ms | 28.7 FPS |
| CUDA FP16 | 30.1 ms | 33.3 FPS |

## 本地已修复、尚未远端验证的 alignment 限频问题

远端已验证版本在每次推理完成后更新时间，下一轮又等待完整的
`1 / rate_hz`。因此远端真实周期仍为：

```text
推理耗时 + 1 / inference.rate_hz
```

本地现已记录每轮推理的开始时间，下一轮只等待目标 start-to-start 周期的剩余部分，
新周期为：

```text
max(处理耗时, 1 / inference.rate_hz)
```

首次推理不等待；处理超过周期时直接处理最新帧。相关纯逻辑测试均通过。完整
`real2` 负载下的 8.03 Hz 检测和 7.04 Hz 调试图仍是旧逻辑基线，不是修复后的
结果。按用户要求暂不远端同步、构建或运行；下一步需等待用户明确允许。

## 已完成的远端完整验证

### 1. 同步内容

```bash
ssh -o BatchMode=yes -o ConnectTimeout=10 jetson@10.42.0.3 \
  sha256sum \
  /home/jetson/workspaces/rc_sim/src/robot_r2_target_alignment/config/yolo_target_detector.yaml \
  /home/jetson/workspaces/rc_sim/src/robot_r2_target_alignment/robot_r2_target_alignment/yolo_target_detector.py \
  /home/jetson/workspaces/rc_sim/src/robot_r2_target_alignment/README.md
```

结果与上面的三个本地哈希完全一致。

### 2. 进程安全

```bash
ssh jetson@10.42.0.3 \
  "ps -ef | grep -E 'real2.launch|target_alignment.launch|yolo_target_detector' | grep -v grep"
```

每次启动前均确认没有用户视觉进程。验证使用独立进程组，结束后只清理本次记录的
进程组；最终检查无残留进程。

### 3. 单包构建

```bash
ssh jetson@10.42.0.3 \
  "bash -lc 'cd /home/jetson/workspaces/rc_sim && \
  source /opt/ros/humble/setup.bash && \
  colcon build --symlink-install \
    --packages-select robot_r2_target_alignment \
    --allow-overriding robot_r2_target_alignment'"
```

构建成功，输出为 `1 package finished`。

### 4. 安装空间与 CUDA

```bash
ssh jetson@10.42.0.3 \
  "sed -n '1,18p' \
  /home/jetson/workspaces/rc_sim/install/robot_r2_target_alignment/share/robot_r2_target_alignment/config/yolo_target_detector.yaml"

ssh jetson@10.42.0.3 \
  "python3 -c 'import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))'"
```

安装配置显示 `device: "0"`；PyTorch 2.5.0a0 报告 CUDA 可用，设备名为 `Orin`，
Jetson 电源模式为 `MAXN_SUPER`。离线同参数 FP32 wall time 中位数约 29–34 ms，
模型参数实际位于 `cuda:0`。

### 5. 运行验证

必须使用 `test_mode:=true`，避免发布实际底盘速度。建议使用两个独立进程组启动，
只终止本次记录的进程组，不使用宽泛的 `pkill`：

```bash
cd /home/jetson/workspaces/rc_sim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bringup real2.launch.py
```

另一终端：

```bash
cd /home/jetson/workspaces/rc_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_r2_target_alignment target_alignment.launch.py test_mode:=true
```

alignment 日志必须包含：

```text
Loaded YOLO detector ... on device 0
```

运行时可检查：

```bash
ros2 param get /r2/target_alignment/yolo_target_detector model.device
ros2 topic hz /r2/target_alignment/debug_image
tegrastats
```

最终结果：

- 参数：`model.device=0`、`visualization.enabled=true`、`test_mode=true`。
- 日志包含 `Loaded YOLO detector ... on device 0`，此前验证中实际识别到
  `duantou` 并只记录候选速度 `[TEST]`，未发布底盘速度。
- 检测循环约 8.03 Hz；实际订阅调试图约 7.04 Hz。
- `tegrastats` 采样可见 `GR3D_FREQ` 18%–46%，确认运行时使用 GPU。
- alignment 日志未发现 `Traceback` 或错误，HIK 以 720×540、30 FPS、2×2
  binning 正常打开。
- 验证后无 `real2`、alignment 或 YOLO 残留进程。

## HIK 相关历史上下文

- 远端相机：HIKROBOT `MV-CS016-10UC`，序列号 `DA6511371`。
- 当前远端实际启动参数曾验证为：30 FPS、曝光 3000 us、增益 16 dB、2×2
  binning 开启、输出 720×540。
- HIK 配置已经去掉宽高裁剪参数，保留 `binning_2x2_enabled` 开关；2×2 binning
  提供完整视野低分辨率画面。
- 早期 Jetson MVS SDK 不支持 `MVCC_ENUMVALUE_EX` 和
  `MV_CC_GetEnumValueEx`，源码后来改用旧 SDK 兼容接口。不要恢复 Ex 接口。
- 本地 `src/sim_to_real/hik_camera/config/hik_camera.yaml` 当前增益为 32 dB，远端
  安装配置此前为 16 dB，二者存在差异；这不是本次 alignment CUDA 任务范围。

## Git 工作区注意事项

本地工作树已有用户修改，必须保留，不要 reset 或覆盖：

```text
 M DEPENDENCIES.md
 M rsync.sh
 M src/robot_r2_detect/__pycache__/setup.cpython-310.pyc
 M src/robot_r2_detect/setup.py
 M src/robot_r2_interfaces/CMakeLists.txt
 M src/sim_to_real/hik_camera/config/hik_camera.yaml
 M src/sim_to_real/hik_camera/src/hik_camera.cpp
?? src/robot_r2_detect/model/duantou.pt
?? src/robot_r2_interfaces/msg/TargetDetection.msg
?? src/robot_r2_target_alignment/
```

`robot_r2_target_alignment`、`TargetDetection.msg` 和 `duantou.pt` 都还是未跟踪内容，
但它们互相依赖，不能删除。当前没有创建提交。

## 临时诊断文件与进程

- 本次创建的 BEST_EFFORT 帧率探针与两个微基准脚本已从本机和 Jetson `/tmp`
  清除，不属于仓库。
- 本次启动的 real2/alignment/HIK 进程均已确认清理。
