# ROS 2 同机图像 Topic 通信方案

本文定义一种可跨 ROS 2 工程复用的高吞吐图像 Topic 通信方案。方案基于：

- bounded 自定义图像消息
- Fast DDS Data Sharing
- Best Effort、Keep Last 1、Volatile QoS
- 发布端缓冲区复用
- 订阅端直接建立图像视图

方案面向相机发布端和图像订阅端运行在同一台计算机上的场景，不依赖 Loaned
Message。跨主机通信仍可由 DDS 完成，但无法获得同机 Data Sharing 的效果，
不属于本方案的主要目标。

## 1. 公共接口包

不同工程之间必须依赖同一个独立 ROS 2 接口包，并使用完全相同的消息定义。
接口包应单独维护版本，避免每个工程复制一份相似但不兼容的消息。

推荐消息结构如下：

```text
uint8 ENCODING_BGR8=0
uint8 ENCODING_RGB8=1
uint8 ENCODING_MONO8=2
uint8 LAYOUT_VERSION=1
uint32 FRAME_ID_CAPACITY=128
uint32 DATA_CAPACITY=24245760

uint64 sequence
int32 stamp_sec
uint32 stamp_nanosec

uint32 width
uint32 height
uint32 step
uint32 data_size

uint8 encoding
uint8 is_bigendian
uint8 layout_version
uint8 frame_id_size

uint8[128] frame_id
uint8[<=24245760] data
```

其中：

- `data` 必须是有明确上限的 bounded sequence，不能使用无界 `uint8[]`。
- 示例中的数据上限是 `24,245,760` 字节。实际接口应按系统允许的最大图像
  计算，至少满足 `最大高度 × 最大 step`；ROS 2 消息的数组边界必须写成
  字面量。
- `data_size` 表示当前帧的实际有效字节数，只传输有效部分，而不是每帧发送
  整个最大容量。
- `frame_id` 使用固定数组和实际长度字段，避免无界字符串。
- `encoding` 使用双方约定的数字常量，避免可变长度编码字符串。
- `layout_version` 用于识别消息布局版本。

如果最大分辨率、步长或编码变化导致一帧超过接口容量，发布端必须
拒绝发布。需要扩大容量时，应升级公共接口包并重新构建所有发布端和订阅端。

## 2. Fast DDS Data Sharing

所有参与图像通信的进程统一使用 `rmw_fastrtps_cpp`，并加载相同的 Fast DDS
XML 配置：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <publisher profile_name="camera data sharing publisher"
             is_default_profile="true">
    <qos>
      <publishMode>
        <kind>SYNCHRONOUS</kind>
      </publishMode>
      <data_sharing>
        <kind>AUTOMATIC</kind>
      </data_sharing>
    </qos>
    <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
  </publisher>

  <subscriber profile_name="camera data sharing subscriber"
              is_default_profile="true">
    <qos>
      <data_sharing>
        <kind>AUTOMATIC</kind>
      </data_sharing>
    </qos>
    <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
  </subscriber>
</profiles>
```

进程启动前设置：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTDDS_DEFAULT_PROFILES_FILE=/absolute/path/to/fastdds_camera.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=/absolute/path/to/fastdds_camera.xml
```

这些环境变量应由 launch 文件、容器入口或统一启动脚本设置，不能依赖用户手动
输入。发布端和订阅端还必须使用相同的 `ROS_DOMAIN_ID`。

该 XML 使用默认 Publisher/Subscriber profile，因此会影响同一进程中的其他
DDS 实体。若进程还包含具有不同 DDS 策略的非图像节点，应将图像节点拆为独立
进程，或为该系统设计更细分的 DDS profile。

## 3. 图像 QoS

发布端和所有订阅端统一使用：

```text
Reliability: Best Effort
History:     Keep Last
Depth:       1
Durability:  Volatile
```

实时图像应优先处理最新帧。当订阅端处理速度低于相机帧率时，允许丢弃旧帧，
不能通过增加队列深度积压大图像。

相机标定信息继续使用 `sensor_msgs/msg/CameraInfo`，通过单独 Topic 发布。

## 4. 发布端约束

- 每个相机使用独立节点和独立 Topic。
- 启动时创建并长期复用消息对象，为 `data` 预留消息定义的最大容量。
- 每帧只调整 `data` 的实际长度并写入 `data_size` 字节，不清零整个最大缓冲区。
- 相机或解码器输出完成后，直接复制到复用的消息缓冲区，避免额外的中间图像
  容器和重复拷贝。
- 发布前校验：
  - `width` 和 `height` 非零
  - `data_size == height × step`
  - `step` 满足当前编码所需的最小行字节数
  - `data_size` 不超过消息定义的最大容量
  - 编码和布局版本受支持
  - `frame_id` 未超过固定容量
- 每个相机独立维护连续递增的 `sequence`。
- 在完整帧可用后立即生成时间戳，统一使用 ROS clock。
- 输入为 MJPEG、YUYV 或其他格式时，先完成解码或转换，再按照消息中声明的
  BGR8、RGB8 或 MONO8 布局发布。

Data Sharing 不消除相机采集缓冲区到 DDS 消息缓冲区之间的一次必要复制。
本方案的目标是减少 DDS 进程间传输产生的额外开销。

发布端优先使用 C++ `rclcpp`。Python `rclpy` 发布端也兼容本方案，但必须长期
复用同一个消息对象及其 `data` 缓冲区，不能逐帧重新创建数 MB 的 `bytes`、
`list` 或消息对象。

### 4.1 调试图像发布

所有可能额外发布 `sensor_msgs/msg/Image` 调试图像的节点都必须声明同名
ROS 2 参数 `visualization_enabled`：

- 参数类型为 `bool`，默认值为 `false`。
- 参数必须支持运行时动态修改，不得只在节点启动或插件加载时读取一次。
- 调试图像发布器可以常驻，但参数为 `false` 时不得构造或发布调试图像。
- 参数只控制 `/debug` 图像，不影响主 `CameraFrame`、检测结果或相机信息。
- 一个节点发布多个调试图像 Topic 时，由该节点的同一个
  `visualization_enabled` 参数统一控制。
- 例外：`kfs_detect_fused` 的前/左/右三路调试图像
  （`/r2/detection/{front,left,right}/debug`）由
  `visualization_enabled_front/left/right` 三个参数独立控制，行为相同。

例如：

```bash
ros2 param set /node_name visualization_enabled true
ros2 param set /node_name visualization_enabled false
```

## 5. 订阅端约束

- 使用与发布端完全相同的接口包版本和 QoS。
- 收到消息后先校验布局、尺寸、编码、步长和数据长度。
- 消息缓冲区建立的图像视图只能在该消息的生命周期内使用。
- 如果图像需要进入异步队列或在回调结束后继续使用，必须明确复制并管理新的
  缓冲区所有权。
- 不要让耗时处理阻塞所有相机回调。多相机应用应使用独立 callback group 和
  MultiThreadedExecutor，或使用多个独立进程。
- 每路原始图像尽量只有一个主要处理入口。检测结果、状态和可视化信息应转换
  为较小的消息再分发，避免多个进程重复接收并复制所有原始帧。

### 5.1 C++ 订阅

C++ OpenCV 处理可以直接基于消息缓冲区建立 `cv::Mat` 视图：

```cpp
cv::Mat image(
  static_cast<int>(msg.height),
  static_cast<int>(msg.width),
  CV_8UC3,
  const_cast<uint8_t *>(msg.data.data()),
  static_cast<size_t>(msg.step));
```

该视图不复制图像数据。订阅回调只读取图像时，不应通过 `const_cast` 得到的
指针修改消息内容。

### 5.2 Python 订阅

Python `rclpy` 可以正常发布和订阅 bounded 图像消息，底层仍使用相同的 Fast
DDS Data Sharing 和 QoS。收到消息后，可以通过 NumPy 直接建立视图：

```python
rows = np.frombuffer(
    msg.data,
    dtype=np.uint8,
    count=msg.data_size,
).reshape((msg.height, msg.step))

channels = 3  # 根据 encoding 选择 1 或 3
image = rows[:, :msg.width * channels].reshape(
    (msg.height, msg.width, channels)
)
```

`np.frombuffer` 不会在 Python 消息缓冲区之外再次复制整帧。不能改用
`np.array(msg.data)` 或 `list(msg.data)`，否则会产生额外的全帧复制。

Data Sharing 无法消除 `rclpy` 将 DDS 数据转换成 Python 消息对象时的序列化
和反序列化开销，因此 Python 的 CPU 占用和延迟通常高于等价 C++ 实现。这个
差异不影响协议兼容性：C++ 和 Python 节点可以任意组合发布和订阅同一个图像
Topic。

## 6. 实现语言选择

C++ 是本方案的默认和优先实现语言，特别适合：

- 相机采集和图像发布端
- 多相机、高分辨率或高帧率场景
- 对 CPU、延迟和抖动敏感的处理节点
- 需要持续遍历、转换或复制整帧的节点

Python 也可以直接使用本方案，适合：

- 更重视开发效率和算法迭代速度的节点
- 主要处理 NumPy、OpenCV Python 或 Python 推理框架的节点
- 相机数量和计算负载可控，并且系统仍有足够 CPU 余量的场景

选择原则是“优先 C++，允许 Python”。不能因为订阅端使用 Python 而退回
`sensor_msgs/msg/Image`、更改 Topic 类型或取消 Data Sharing。Python 节点
必须遵守相同的 bounded 消息、QoS、缓冲区生命周期和避免额外复制的约束。

## 7. Topic 与工程集成

建议统一采用下面的命名形式：

```text
/camera/<camera_name>/image_raw
/camera/<camera_name>/camera_info
```

具体前缀可以由系统约定，但同一套系统必须保持一致，不能让消息类型或 QoS
依赖 Topic 名称来隐式区分。

跨工程集成时应保证：

1. 公共接口包来自同一版本。
2. 所有工程使用兼容的 ROS 2、`rmw_fastrtps_cpp` 和 Fast DDS 版本。
3. 发布端和订阅端加载相同的 Data Sharing XML。
4. 发布端和订阅端使用相同的 `ROS_DOMAIN_ID`、消息类型、Topic 名称和 QoS。
5. 接口定义变化后重新构建并重新 source 所有相关工作空间。

## 8. 使用边界

- 当前方案不要求发布端或订阅端支持 Loaned Message。
- 不把 `sensor_msgs/msg/Image` 与 bounded 自定义消息混用在同一个图像 Topic
  上。
- 不使用 Reliable QoS 或深队列传输实时原始图像，除非具体应用明确要求完整
  保存每一帧，并能承担由此产生的阻塞和内存开销。
- 多相机系统中，图像解码、颜色转换、视觉算法以及重复订阅通常比 Topic 通信
  本身更容易成为瓶颈，应避免不必要的全帧复制和多路原始图像分发。
