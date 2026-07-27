# Jetson Yahboom onboard workspace

Yahboom Jetson 机载视觉与 PX4 通信侧的可部署文件归档。

当前配置对应：

- Ubuntu 22.04 / ROS 2 Humble；
- AprilTag `tag36h11`，1280×1024 @ 60 FPS；
- `ROS_DOMAIN_ID=99`；
- Jetson 直连网口 `192.168.1.10/24`；
- PX4 FMU-v6X `192.168.1.3/24`；
- Micro XRCE-DDS Agent UDP4 `0.0.0.0:8888`；
- PX4 命名空间 `/uav1`；
- ROS 2 输入话题 `/uav1/fmu/in/target_relative_pose`。

## 仓库内容

```text
src/px4_msgs/                    与当前 PX4 固件匹配的完整消息包
src/target_relative_pose_bridge/ AprilTag TF 到 PX4 相对位姿的 ROS 2 桥
config/apriltag/                 AprilTag 检测参数
config/camera_info/              1280×1024 相机标定
start_apriltag_stack.sh          相机、TF、AprilTag、位姿桥和 Agent 启动入口
start_xrce_agent_ethernet.sh     Ethernet Micro XRCE-DDS Agent 重连脚本
systemd/                         开机自启与 Jetson 性能模式服务
docs/                            部署、标定和链路验证记录
```

`build/`、`install/`、`log/`、密码、SSH 私钥和 API 密钥均不进入仓库。

## 重要边界

本仓库包含当前 PX4 配套的 ROS 2 包和最终机载配置，但历史上修改过的
`apriltag_ros` 与 `vision_opencv` 源码原仓库当前不可访问，因此没有伪造或替换
这些源码。部署前需要在 `src/` 中另行提供与 ROS 2 Humble、系统
`libapriltag` 和 OpenCV 版本兼容的 `apriltag_ros`；必须应用
`config/apriltag/tags_36h11.yaml` 中的 `sensor_data` QoS 参数。

详细兼容要求和历史修改说明见
[`docs/jetson_onboard_rebuild_archive_zh.md`](docs/jetson_onboard_rebuild_archive_zh.md)。

## 建议部署路径

启动脚本和 systemd 单元按以下路径编写：

```text
/home/jetson/yahboom_ws
```

克隆时可直接指定该目录：

```bash
git clone git@github.com:wangmingyang-3915/jetson-yahboom.git \
  /home/jetson/yahboom_ws
cd /home/jetson/yahboom_ws
```

准备相机参数：

```bash
mkdir -p /home/jetson/.ros/camera_info
install -m 0644 \
  config/camera_info/decxin_camera:_decxin_camera.yaml \
  /home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml
```

将 AprilTag 参数复制到实际包中：

```bash
install -m 0644 \
  config/apriltag/tags_36h11.yaml \
  src/apriltag_ros/cfg/tags_36h11.yaml
```

安装 ROS 依赖并构建：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

安装启动文件：

```bash
chmod 755 start_apriltag_stack.sh start_xrce_agent_ethernet.sh
sudo install -m 0644 systemd/apriltag-stack.service \
  /etc/systemd/system/apriltag-stack.service
sudo install -m 0644 systemd/apriltag-performance.service \
  /etc/systemd/system/apriltag-performance.service
sudo systemctl daemon-reload
sudo systemctl enable --now apriltag-performance.service apriltag-stack.service
```

## 直连以太网

Jetson 网口不设置默认网关：

```bash
sudo nmcli connection add \
  type ethernet \
  ifname enP8p1s0 \
  con-name px4-direct \
  ipv4.method manual \
  ipv4.addresses 192.168.1.10/24 \
  ipv6.method disabled
sudo nmcli connection up px4-direct
ping -I enP8p1s0 -c 2 192.168.1.3
```

若 `px4-direct` 已存在，应修改现有连接，不要重复创建。

## 快速验证

```bash
systemctl --no-pager --full status apriltag-stack.service
ros2 topic hz /uav1/fmu/in/target_relative_pose
ros2 topic hz /uav1/fmu/out/vehicle_attitude
ros2 topic echo /uav1/fmu/out/vehicle_status --once
journalctl -u apriltag-stack.service -n 100 --no-pager
```

重新使用相机内参或外参前，请先阅读
[`docs/onboard_camera_intrinsics_extrinsics_zh.md`](docs/onboard_camera_intrinsics_extrinsics_zh.md)；
相机、镜头、焦距、分辨率或机械安装变化后必须重新标定/测量。
