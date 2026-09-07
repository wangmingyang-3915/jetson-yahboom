# AprilTag ROS 2 节点

这个 ROS 2 节点使用 AprilTag 库检测图像中的 AprilTag，并发布其位姿、ID 和其他元数据。

有关 AprilTag、论文和参考实现的更多信息，请参阅：https://april.eecs.umich.edu/software/apriltag.html

## 话题

### 订阅：

节点通过 `image_transport::CameraSubscriber` 订阅话题 `image_rect` 上的校正图像。具体的话题名称取决于所选择的图像传输类型（参数 `image_transport`，可选 `raw` 或 `compressed`）：

- `image_rect`（`raw`，类型：`sensor_msgs/msg/Image`）
- `image_rect/compressed`（`compressed`，类型：`sensor_msgs/msg/CompressedImage`）
- `camera_info`（类型：`sensor_msgs/msg/CameraInfo`）

### 发布：

- `/tf`（类型：`tf2_msgs/msg/TFMessage`）
- `detections`（类型：`apriltag_msgs/msg/AprilTagDetectionArray`）

节点使用 `CameraInfo` 中的相机内参 `P`，根据单应矩阵 `H` 计算标签位姿 `T`。图像与相机内参必须具有相同的时间戳。

标签位姿通过标准 TF 话题 `/tf` 发布，其消息头设置为图像的消息头，`child_frame_id` 设置为 `tag<family>:<id>`（例如 `tag36h11:0`），或者设置为配置文件中选择的坐标系名称。检测到的标签的其他信息以 `AprilTagDetectionArray` 消息发布，其中包含原始单应矩阵、检测结果的 `hamming` 距离和 `decision_margin`（判决裕量）。

## 配置

节点通过 YAML 配置文件进行配置。完整的 ROS YAML 参数文件语法请参阅：https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser。

配置文件格式如下：

```yaml
apriltag:                 # 节点名称
  ros__parameters:
    # 基本设置（默认值）
    image_transport: raw  # 图像格式："raw" 或 "compressed"
    family: 36h11         # 标签族名称：16h5、25h9、36h11
    size: 1.0             # 默认标签边长，单位为米
    profile: false        # 是否将性能分析信息输出到标准输出

    # 检测参数调优（默认值）
    max_hamming: 0        # 允许的最大汉明距离（纠正的位数）
    detector:
      threads: 1          # 线程数
      decimate: 2.0       # 四边形检测时的分辨率降采样倍数
      blur: 0.0           # 四边形检测所用高斯模糊的 sigma 值
      refine: 1           # 是否将边缘对齐到强梯度位置
      sharpening: 0.25    # 对解码图像进行锐化的强度
      debug: 0            # 是否向当前工作目录写入额外的调试图像

    pose_estimation_method: "pnp" # 标签位姿估计方法

    # （可选）标签列表
    # 如果定义了此列表，'frames' 和 'sizes' 的长度必须与 'ids' 相同。
    tag:
      ids:    [<id1>, <id2>, ...]         # 要发布变换的标签 ID
      frames: [<frame1>, <frame2>, ...]   # 坐标系名称
      sizes:  [<size1>, <size1>, ...]     # 各标签的边长，会覆盖默认的 'size'
```

`family`（字符串）定义检测器使用的标签族，必须是 `16h5`、`25h9`、`36h11`、`Circle21h7`、`Circle49h12`、`Custom48h12`、`Standard41h12` 或 `Standard52h13` 之一。`size`（浮点数）表示标签的边长，单位为米，并假定标签为正方形。

如果不想发布所有标签的位姿，可以使用 `tag.ids` 列表，使节点只发布选定标签 ID 的位姿。每个标签都可以在 `tag.frames` 中关联一个子坐标系名称，并在 `tag.sizes` 中指定该标签的尺寸。这些列表的长度必须与 `tag.ids` 相同，也可以为空。列表为空时，节点将使用 `tag<family>:<id>` 形式的默认坐标系名称和默认标签边长 `size`。

其余参数采用 AprilTag 库中的默认值。有关这些参数功能的详细说明，请参阅 `apriltag.h`。

有关发布 36h11 标签族中特定标签位姿的配置示例，请参阅 [tags_36h11.yaml](cfg/tags_36h11.yaml)。

## 节点

### 独立可执行节点

可以使用话题重映射和配置文件启动 `apriltag_node` 可执行程序：

```sh
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/image \
    -r camera_info:=/camera/camera_info \
    --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
```

### 可组合节点

为了提高进程内通信效率，本软件包提供了可组合节点：

```sh
$ ros2 component types
apriltag_ros
  AprilTagNode
```

可以将这个 `AprilTagNode` 组件与其他节点一起加载到一个“容器节点”进程中。它们使用共享内存通信，从而避免不必要的数据复制。示例启动文件 [camera_36h11.launch.yml](launch/camera_36h11.launch.yml) 将 `AprilTagNode` 组件与 [`camera_ros` 软件包](https://index.ros.org/p/camera_ros/) 中的 `camera::CameraNode` 组件（使用 `sudo apt install ros-$ROS_DISTRO-camera-ros` 安装）加载到同一个容器，并为二者启用 `use_intra_process_comms`：

```sh
ros2 launch apriltag_ros camera_36h11.launch.yml
```
