# Jetson 视觉链路修改记录与旧版本清理

整理日期：2026-10-20（北京时间）。设备：`jetson@10.97.154.145`，主机名 `yahboom`。

本文将视觉链路散落的旧代码副本整理为修改记录。当前使用版本以 `/home/jetson/yahboom_ws` 的源码、启动脚本和实际服务引用为准，不按文件名中是否包含 `new` 判断。

记录依据为清理前实际文件内容、版本间差异、systemd 配置、活动进程及构建目录。带日期的备份名称表示修改前快照，不能单独证明修改完成日期；下表日期用于定位迭代批次。历史修改的目标与现有代码行为可以核实，但未对每个历史版本重新运行性能测试。

## 当前保留版本

| 部分 | 现行入口或文件 | 当前行为 |
|---|---|---|
| 自动启动 | `/etc/systemd/system/apriltag-stack.service` → `start_apriltag_stack.sh` | 相机、识别、TF、位姿桥接和以太网 XRCE Agent 统一启动；节点退出由 systemd 重启 |
| 相机控制 | `camera_controls.conf` | 保存当前软件控制值；启动前通过 `v4l2-ctl` 恢复 |
| 相机采集 | 启动脚本中的 GStreamer 管线 | MJPEG，1280×1024，配置 60 FPS；`nvjpegdec` 硬件解码，输出 `mono8` |
| 相机内参 | `/home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml` | 已于 2026-10-20 调焦后重新标定并部署；详见文末 |
| 检测配置 | `src/apriltag_ros/cfg/tags_36h11.yaml` | 36h11；ID 0/1/2；边长 0.08 m；6 线程；decimate 2.0 |
| 检测及 PnP | `src/apriltag_ros/src/AprilTagNode.cpp`、`pose_estimation.cpp/.hpp` | 缓存内参、仅保留最新图像；原图 K/D 参数；IPPE 候选和姿态连续性选择 |
| 相对位姿桥接 | `src/target_relative_pose_bridge/target_relative_pose_bridge/node.py` | 本机 body FRD 在目标 body FRD 中的位姿；转换成 PX4 wxyz 四元数 |
| 飞控连接 | `start_xrce_agent_ethernet.sh` | UDP4 端口 8888；进程退出后重试 |
| 仪表盘 | `vision_dashboard/server.py`、`launch.sh`、`image_relay/` | C++ 持续订阅生产话题并限频转发，Python 提供监控界面 |
| 仪表盘运行中继 | `vision_dashboard/relay_install/` | 当前运行安装目录；对应保留的构建目录是 `relay_build_new/` |

## 历次修改及作用

### 早期采集、同步与性能优化

1. **相机链路替换为硬件解码。** `start_apriltag_stack.sh.pre_hw_decode` 中使用 `usb_cam_node_exe`；现行版本改为 GStreamer MJPEG → `nvjpegdec` → 灰度图，减少软件解码和颜色转换开销。分辨率和目标帧率保留在启动脚本中。
2. **CameraInfo 同步方式调整。** `AprilTagNode.cpp.pre_sync_optimization` 使用 `image_transport::CameraSubscriber`；后续快照改为分别订阅图像和 CameraInfo，缓存最近内参并加互斥保护。这样不必为匹配相同时间戳而等待或重复处理整幅图像。`.orig` 本身也包含缓存实现，因此它是某个中间快照，不宜称为未修改的上游版本。
3. **积压帧处理优化（2026-07-27 批次）。** 图像 QoS 改为 `keep_last(1)`，检测器 `decimate` 从 1.5 调到 2.0。目的为减少检测耗时和处理过期图像的积压；降采样同时会减少小靶标细节，效果随场景变化。
4. **稳定设备选择及时间跳变处理。** 当前脚本按 USB by-id 定位相机；未发现设备时有限等待。若开机时尚未完成系统校时，后台等待同步后请求整套链路重启，以重新计算 GStreamer 时间偏移。这些行为在多份旧快照中已存在，不能仅凭快照确定首次引入日期。

### 2026-07-27：坐标系、通信与平面位姿求解

| 修改 | 代码差异与作用 | 依据快照 |
|---|---|---|
| UAV 命名空间 | 位姿输出由 `/fmu/in/target_relative_pose` 改为 `/uav1/fmu/in/target_relative_pose`，对齐目标飞控实例 | `start_apriltag_stack.sh.before_uav1_20260727` |
| 以太网 XRCE 通信 | USB 串口方式被生产链路中的 UDP4:8888 替代；保留以太网重试脚本 | `start_apriltag_stack.sh.before_ethernet_20260727` 与 USB/以太网脚本 |
| 目标坐标轴修正 | tag→目标 body FRD 的旋转从早期单轴旋转改为组合旋转，再作安装方向补偿 | `before_target_rotation_fix_20260727`、`before_relative_orientation_calibration_20260727` |
| 原图内参与畸变 | 直接处理 `/image_raw` 时，使用 CameraInfo 的 K 和 D，替代校正图投影矩阵 P 与空畸变参数 | `AprilTagNode.cpp.before_ippe_fix_20260727` 及 `pose_estimation.cpp/.hpp.before_ippe_fix_20260727` |
| 平面双解处理 | `solvePnPGeneric(...SOLVEPNP_IPPE)` 保留候选；排除相机后方解并计算重投影误差 | 同上 |
| 姿态分支连续性 | 在 0.5 s 新鲜历史内，用角距离为主、重投影误差为辅选择候选，并对齐四元数符号，减少相邻帧分支翻转 | 现行 `selectPose()` 与上述快照的差异 |
| PX4 消息包 | 早期仅含 TargetRelativePose 的最小消息包被现行完整 `src/px4_msgs/` 替代，支持心跳和其他 PX4 消息 | `yahboom_ws_backups/px4_msgs.minimal_before_full_20260727` |

### 2026-07-28：多靶标和安装姿态补偿

1. **单靶标扩展为 ID 0/1/2。** 桥接增加 `target_frames`、`target_ids` 数组，校验长度、ID 范围和重复值；保留单目标参数兼容性。各 ID 对应独立 TF，选择可用且较新的目标。依据 `node.py.before_multi_tag_20260728` 与后续版本。
2. **短时丢失保持机制。** 增加 `dropout_grace_s`、上一份有效位姿缓存和恢复日志；保持时保留原始采样时间。早期部署曾配置 1.0 s 宽限；当前部署已恢复为 0.0 s，失联处理交给飞控状态机。依据 `node.py.before_dropout_grace_20260728` 等快照。
3. **安装姿态连续调平。** `before_level_recalibration`、`before_final_level_compensation`、`before_last_level_trim` 三组快照记录 tag→目标机旋转的调整；平移保持 `[0.010, 0.036, -0.080]` m。现行旋转（xyzw）为 `[-0.509742963809257, 0.510702549407070, -0.488625888120183, 0.490499498812014]`。这些是部署补偿参数，本次未重新测量或验证其精度。

### 2026-07-30：心跳、命名空间和丢失宽限调整

| 修改 | 变化及目的 | 依据快照 |
|---|---|---|
| Offboard 心跳 | 在相对位姿之外增加 OffboardControlMode 发布，声明位置控制级别 | `node.py.before_offboard_heartbeat_20260730` |
| 心跳命名空间推导 | 未显式配置时，从位姿输出话题前缀推导心跳话题，避免 `/uav1` 与根命名空间混用 | `node.py.before_namespace_inference_20260730` |
| 宽限时间迭代 | 可见快照中，代码默认值从 0.0 调到 1.0，再缩到 0.5 s；启动参数也有相应调整 | `before_dropout_default_20260730`、`before_dropout_05_20260730` |

### 2026-08-13：与全向控制链路同步

`backups/codex_20260813_fullvector_sync` 保存了本次同步前的脚本和桥接节点。可直接核实的变化包括将旧 0.5 s 丢失保持收紧到 0.0 s，避免把旧姿态继续当作新测量。

同目录下 `start_apriltag_stack.sh.before_fixed_target_id_1` 的文件名包含“固定 ID 1”，但实际内容仍为 `[0, 1, 2]` 多目标列表。不能据此断言某个固定 ID 1 版本曾部署；本记录以实际代码为准。

### 2026-08-17：心跳与测量有效性解耦的迭代

可见快照展示了三种历史实现：

- 心跳位置使能直接等于位姿有效标志：短暂检测丢失也会撤销位置控制声明。
- 一度尝试持续声明位置控制，并采用 0.15 s 测量保持；后续又恢复较严格的测量有效性策略。
- 随后的实现将心跳独立为 20 Hz 定时器，位姿保持 60 Hz；首次有效位姿后才置就绪，QoS durability 从 TRANSIENT_LOCAL 改为 VOLATILE。

依据 `backups/20260817_180734_offboard_heartbeat`、`backups/20260817_190400_control_safety`、`deploy_backup_20260817_offboard_fix` 和 `node.py.before_state_machine_sync_20260818`。上述是可见版本之间的行为变化，不将所有中间策略都称为最终优化。

### 2026-08-18：桥接侧与飞控失联状态机对齐

现行版本相对 `before_state_machine_sync_20260818` 增加：

- 连续 1.5 s 未接收通过校验的位姿时，撤销 Offboard 就绪、清理旧位姿及活动目标。
- 就绪会话期间锁定目标 ID，避免在多个靶标间跳变。
- 位置门限为 `0.2 m + 3.0 m/s × 样本间隔`；姿态角门限为 `0.2 rad + 2.0 rad/s × 样本间隔`，样本间隔截断在 0～1 s。
- 只有通过全部门控的位姿才刷新就绪超时计时。
- 测量最大年龄保持 0.2 s，桥接侧丢失宽限保持 0.0 s。

这些门控可以拒绝异常值，也可能导致“识别正常但输出位姿无效”；是否发生必须读取运行时状态，不能仅看检测 ID。

### 2026-09-21：仪表盘中继扩展

旧 `relay_install_old` 二进制可见输出仅包括图像预览及图像频率。现行 `image_relay.cpp` 将相机内参、检测结果、相对位姿、心跳、飞控状态及 TF 一并纳入监控；生产输入持续订阅，各类仪表盘副本分别限频，频率计数每秒发布。

保留 `relay_install`、源码 `image_relay/` 及其对应的 `relay_build_new`/`relay_log_new`；删除前一代 `relay_install_old` 和对应旧 `relay_build`/`relay_log`。后缀 `_new` 代表当前构建目录，不代表可删除的临时目录。

### 2026-10-20：调焦与参数持久化

- 实际镜头调焦后，原图视觉清晰度改善；此前约 10 s 检测观察收到 346 条检测消息，均包含 ID 0。这仅是当时静态条件下的结果。
- 新增 `camera_controls.conf` 并在启动脚本中恢复控制值：brightness 0、contrast 52、saturation 64、hue 0、自动白平衡 1、gamma 300、gain 64、sharpness 50、背光补偿 0、自动曝光模式 3、动态帧率 0。
- 软件控制值与调焦前相同；清晰度改善不能归因于改写这些控制值。镜头机械焦点没有对应的可保存软件参数。
- Q12-150-10（11×8 内角点，10 mm 方格）重新标定程序曾启动，随后按用户要求关闭。该会话未完成新内参部署，当前仍使用旧标定。

## 清理边界

删除的是已核实脱离生产引用的旧版本副本：旧启动脚本、算法/桥接备份、历史部署备份目录、旧独立 `apriltag_ws`、最小 px4_msgs 备份、旧仪表盘中继，以及 `yahboom_ws.pre_rebuild_20260727` 中的旧 camera 源码、构建和安装包。

保留当前源码、构建及安装产物、当前 Git 元数据、现有标定数据和日志、当前相机内参、最新仪表盘和运行依赖。`/home/jetson/Apriltag/apriltag` 作为底层库源码保留，不把库和 ROS 包误判为同一个程序的重复版本。旧 pre_rebuild 工作空间中的大模型、语音、接口等非视觉内容不在本次范围内。

删除前检查活动进程的程序路径、工作目录、打开文件、内存映射，保留目录的软链接，以及当前启动入口和源码引用。删除后再次检查服务、生产话题及当前文件指纹；本次不重启视觉服务。

## 清理清单与验证

以下清单由实际删除计划生成。目录指纹为其文件路径与内容哈希的聚合 SHA-256；仅用于审计，不提供旧代码恢复能力。当前保留源码仍可按正常 Git 工作流维护。

已清理 **39 项、497 个文件/链接**，内容大小约 **16.64 MiB**（实际释放空间受文件系统影响）。

| 路径 | 文件/链接数 | 内容字节数 | 聚合 SHA-256 |
|---|---:|---:|---|
| `/home/jetson/apriltag_ws` | 252 | 13711249 | `406cb851f50d644233dbd4866ae9812f1352ccff4872775de1dd7e3803530828` |
| `/home/jetson/yahboom_ws.pre_rebuild_20260727/build/camera` | 18 | 29189 | `1f18e0bad94cc5ff98db5d730484d1a50299fad9fe706bda2ce4d16a3cee77aa` |
| `/home/jetson/yahboom_ws.pre_rebuild_20260727/install/camera` | 35 | 56927 | `b3a5c6f94e7baa47e41b8dab68d580a535530dd14914c160415cb542e2f4fbb0` |
| `/home/jetson/yahboom_ws.pre_rebuild_20260727/src/camera` | 15 | 37747 | `b147c2ca0adf22a7f381ba76222ba48ef64aa4e96303fde6af7e129870187c85` |
| `/home/jetson/yahboom_ws/backups` | 7 | 50850 | `9419c2ba8717842b351a279e9799f588274fd8960ddcf354c8c10a8c8ef76479` |
| `/home/jetson/yahboom_ws/deploy_backup_20260817_offboard_fix` | 2 | 15654 | `dda8ec9813874bf5d70ce8e575402163d6c5d589524b274d42f90a82a2dbf25d` |
| `/home/jetson/yahboom_ws/install/apriltag_ros/share/apriltag_ros/cfg/tags_36h11.yaml.before_dropout_fix_20260727` | 1 | 409 | `6c2b632171fbd724ac6c0e080fed9fbed9cb829cc104142fca7b74c2597186b5` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/cfg/tags_36h11.yaml.before_dropout_fix_20260727` | 1 | 409 | `19123a1f614d92d5e2b86cda8bce0227906501392c245dbc646fed62962b6087` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/src/AprilTagNode.cpp.before_dropout_fix_20260727` | 1 | 12195 | `1fa7d06f2ff33f88ec3e1218fdf1609637b1553dec8b05a83e17f4d17543f07e` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/src/AprilTagNode.cpp.before_ippe_fix_20260727` | 1 | 12366 | `261b9f6560bbeefb7a959217817924adcbe52c72e13d290fe0752665a10a1dcc` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/src/AprilTagNode.cpp.orig` | 1 | 12195 | `5c08ddaabb812d2c32c2351d64f15285d4304c1294f1ebd5fa44252a3f9fe785` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/src/AprilTagNode.cpp.pre_sync_optimization` | 1 | 11234 | `94ff511baaf0d73f40c43fdce6e80ab8de9e52d9e7060b10d4c9f5e721e6d018` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/src/pose_estimation.cpp.before_ippe_fix_20260727` | 1 | 1239 | `34be3c9fd843fd033a98b1af74e798613464e1b5bf54ff51d321aaaeba85c406` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/src/pose_estimation.hpp.before_ippe_fix_20260727` | 1 | 381 | `2924f38288bb2de13fc39a35626abd03a82db9909b75e6fc0e2cfa81d20b01d5` |
| `/home/jetson/yahboom_ws/src/target_relative_pose_bridge/target_relative_pose_bridge/node.py.before_dropout_05_20260730` | 1 | 11529 | `97cac3300a2f5da18dc70293ea997e44f6baa24355c0f21ecaf7311174d57736` |
| `/home/jetson/yahboom_ws/src/target_relative_pose_bridge/target_relative_pose_bridge/node.py.before_dropout_default_20260730` | 1 | 11529 | `c1dc45848e6f1de8e7a252437511541b19cd1d4f5391bdf022e2a3a50101bdf6` |
| `/home/jetson/yahboom_ws/src/target_relative_pose_bridge/target_relative_pose_bridge/node.py.before_dropout_grace_20260728` | 1 | 7858 | `272c7274f42e845054732f8685d6836fb648227f6ed37a2e6f6002d6aa55160e` |
| `/home/jetson/yahboom_ws/src/target_relative_pose_bridge/target_relative_pose_bridge/node.py.before_multi_tag_20260728` | 1 | 5257 | `96daf78ed7fb4df5233acf248268cbb07b0e8f3307323a6c4a7808e5dd42d92f` |
| `/home/jetson/yahboom_ws/src/target_relative_pose_bridge/target_relative_pose_bridge/node.py.before_namespace_inference_20260730` | 1 | 10996 | `384e819d5e56782685fd9ae45c73390895692f6534022c7462826cbfcb82fa82` |
| `/home/jetson/yahboom_ws/src/target_relative_pose_bridge/target_relative_pose_bridge/node.py.before_offboard_heartbeat_20260730` | 1 | 9694 | `7614c48a1e191cef69498122832f130cb23f60722591157a247b802cb52790fe` |
| `/home/jetson/yahboom_ws/src/target_relative_pose_bridge/target_relative_pose_bridge/node.py.before_state_machine_sync_20260818` | 1 | 12242 | `7f0e20545db2787c757078a7d907c3fcef119ac258b425add728630be3438ad9` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.backup-20261020-155717` | 1 | 4267 | `908d38e8875f0903a500eee088232ddc20249e3d9fa3a79ecc9c46f8ea146d3a` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_dropout_05_20260730` | 1 | 4040 | `77061af817a63443df9d7979c39a7ef7a8880ac14fb0b5df413e3a533b65a59a` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_dropout_grace_20260728` | 1 | 4010 | `b88dea4909bae7168f0dcf535e04dfe2a5f1ef528b9182da68f67497daddfe30` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_ethernet_20260727` | 1 | 3382 | `1313cb5e95ab0013c6a8cdd7394880c3f08fcd9bcf3cc9ef00428f48e33bb1a9` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_final_level_compensation_20260728` | 1 | 4040 | `264518455ce808d503b73e38f8bf48ade765bc9e3916a78c420c8ae358335601` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_last_level_trim_20260728` | 1 | 4040 | `5da51adf9a71a17cfb80fdb663dcfefe9b32c2331af9eedea1e2fc494493fdb9` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_level_recalibration_20260728` | 1 | 4040 | `d86325c5a72c1da0ee178bb76cabeec3ebe29dad16c7e5747166293b90fa1228` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_multi_tag_20260728` | 1 | 3414 | `3cdead0c7b66bee0adeaa64b6ff685c317e65edf4334fcbd29d4da00f59602a3` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_relative_orientation_calibration_20260727` | 1 | 3358 | `de3a76663386582b4af9ef3797d832dd172aeb456526af77e30f3b88007ccf46` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_state_machine_sync_20260818` | 1 | 4082 | `70f95d5f6647298c211f21991e62ba8e8d7960279650053e658d3532cf3370c0` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_target_rotation_fix_20260727` | 1 | 3387 | `b987008f9b385d51328bfcb42b6a7cf7bf971906c17b41c8bf75bdb3622f1254` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.before_uav1_20260727` | 1 | 3377 | `73b2383118718567398936c5d1e51f21fcc41fffd3159ba070573cfff07529c7` |
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh.pre_hw_decode` | 1 | 1251 | `ea0487bfc12ef8542a367aa6b899d6a2801bf24da37cc75051dd265913f47f12` |
| `/home/jetson/yahboom_ws/start_xrce_agent_usb.sh` | 1 | 742 | `e459df51d75144bcc1e5eddda5762b65ff5c4c3b6ddedc28f5449e3ffef55920` |
| `/home/jetson/yahboom_ws/vision_dashboard/relay_build` | 83 | 2527010 | `b2c626cfa708f5fe23e5d860cf8be2e838d6e18d2971426d4b7122e1929dde28` |
| `/home/jetson/yahboom_ws/vision_dashboard/relay_install_old` | 36 | 755518 | `29ff181a96a8b2c17676d36d00e3e0ffcc0cb206fe7b5cd5d63e7f31bc94fdff` |
| `/home/jetson/yahboom_ws/vision_dashboard/relay_log` | 17 | 90858 | `1432d6342c2eea3d04f7f574026edf3b76b0afeb37f5497045a601d741558126` |
| `/home/jetson/yahboom_ws_backups` | 3 | 1109 | `02534f8f9869a0267f0d541ee22ccbe52d23274b84152f5daa82c583aaccae0c` |

### 保留文件指纹

用于清理后核对当前代码、可执行文件及相机内参未变更。

| 文件 | SHA-256 |
|---|---|
| `/home/jetson/yahboom_ws/start_apriltag_stack.sh` | `1dc3d76acc05f8445cc1bdfbbb8d2698d40255ed2ca4b5f78512ba807fa8c3a3` |
| `/home/jetson/yahboom_ws/camera_controls.conf` | `8cdab8ea04f87f03771a177af22bad479963c45c0c6d8aada33a2b02e98734b4` |
| `/home/jetson/yahboom_ws/start_xrce_agent_ethernet.sh` | `8f74a29b34a9cc18798ad84eb25a9dc19c90c0d8d8099b0d4a444a06631a6aab` |
| `/home/jetson/yahboom_ws/src/target_relative_pose_bridge/target_relative_pose_bridge/node.py` | `ff3504ae2c1c682ba63c3b40fee5d52bdbf60d292487d0fcc466d37bcdaf586c` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/src/AprilTagNode.cpp` | `65a25432cecaf6512f32673b20ce009e55a58e6ab0fbe2609ad8b2e27e92665e` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/src/pose_estimation.cpp` | `9b7f3de5d4bd94cfa4b893b7e08e44f7caa3e43ca97770cb7ac65e3fa75a1d15` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/src/pose_estimation.hpp` | `5104912bbc681a3cbd3791e289a59644df9f9dad0678ddd83aabd4cdbf107876` |
| `/home/jetson/yahboom_ws/src/apriltag_ros/cfg/tags_36h11.yaml` | `a838d493a71bbf1fc9e1df5ef8d96f26594e697f80f35e3951e74bdb1765279d` |
| `/home/jetson/yahboom_ws/vision_dashboard/server.py` | `0b68bf1eacea0b605acbcb348f68442123f04b85cd93f9647ea315e0e6d5fff0` |
| `/home/jetson/yahboom_ws/vision_dashboard/launch.sh` | `0b900de3f7edbda36c4a6b1917badbd7fe3297ea74493af1baa6a2e370fa18ab` |
| `/home/jetson/yahboom_ws/vision_dashboard/image_relay/src/image_relay.cpp` | `7ad2230e831c3b7b36bb48aca8483869cd4b40a430edade496fb71f5c0731d4e` |
| `/home/jetson/yahboom_ws/vision_dashboard/relay_install/lib/vision_dashboard_image_relay/image_relay` | `42a2ee41459d8625550db04063a9b342061683a9dc2a154d3272ef883462eec3` |
| `/home/jetson/yahboom_ws/install/apriltag_ros/lib/apriltag_ros/apriltag_node` | `53175a83658a76e866888d59e488d9c956b6c7ae7d67dabbf0eb54a33c0d84e5` |
| `/home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml` | `0a56b430343ac91964102da8e86fb622e4585dafea2ec344310f377b65928ea4` |


### 执行结果

清理执行时间：2026-10-20T16:26:24（Jetson 本地时间）。上述 39 项均已删除；保留文件指纹全部一致。服务与话题验证结果见后续记录。

### 清理后实机验证

- 39 项旧版本清单均已不存在；保留文件 SHA-256 再次核对一致。
- apriltag-stack.service 为 active/running，MainPID 仍为 3599，与清理前一致；未重启服务。NRestarts=1 是服务本次开机累计值。
- 当前 apriltag_ros 与桥接包均解析到 yahboom_ws/install。启动脚本 bash 语法检查通过。
- 约 6 s 观察窗口内，四个生产话题均收到数据；计数受发现与订阅启动时间影响，不作为精确频率测试。
- 检测结果包含 ID 0；位姿有效标志仍为 false，与本次清理前观察一致，该问题不属于旧代码清理范围。
- 仪表盘 /api/health 返回 vision-dashboard。

```text
MainPID=3599
NRestarts=1
ActiveState=active
SubState=running
/home/jetson/yahboom_ws/install/apriltag_ros
/home/jetson/yahboom_ws/install/target_relative_pose_bridge
TOPIC_COUNTS {"/uav1/fmu/in/target_relative_pose": 312, "/uav1/fmu/in/offboard_control_mode": 106, "/image_raw": 283, "/detections": 141}
LATEST_STATES {"/uav1/fmu/in/target_relative_pose": {"position_valid": false, "orientation_valid": false, "target_id": 0}, "/detections": {"ids": [0]}}
DASHBOARD_HEALTH {"app": "vision-dashboard"}
```


## 2026-10-20 调焦后重新标定完成

Q12-150-10，11×8 内角点，方格 0.010 m，图像 1280×1024。保存 68 张样本；全分辨率重新提取角点成功 66 张，固定新 K/D 求各视图外参的重投影 RMS 为 0.27693 px，最差单张 0.74095 px。其余两张未通过离线角点提取；该指标是采样数据上的复核，不是独立精度验证。

新 K：fx=689.15091、fy=689.19498、cx=637.55340、cy=494.45791。新 D：[0.008525, -0.026792, -0.000677, 0.000218, 0.0]。通过 /set_camera_info 实时部署，并验证 /camera_info 已发布新 K/D；配置同步到 config/camera_info/decxin_camera.yaml。原标定、采样归档、误差报告及部署报告保存于 calibration/2026-10-20_Q12-150-10_163953/。随后按用户要求关闭标定程序。


## 2026-10-27：稳定对接基准与配套仓库同步

采集靶标 0 有效相对位姿约 25 秒，最后稳定 5 秒的 301 个样本作为安装补偿基准。
补偿前 RPY 为 `[-0.1471876°, 0.2464548°, 3.2530089°]`。
`tag0 -> target_body_frd_0` 新四元数 wxyz 为
`[0.5082522408249879, -0.4876832650038729, 0.4878392943378241, -0.5156137271558183]`。
平移保持 `[0.010, 0.036, -0.080] m`，tag1/tag2 和相机外参未改。

机载启动脚本已备份、更新并重启，飞控位置参数同步写入为
`[0.0438, -0.3936, 0.1089] m`，三轴相对姿态目标均为 0 rad，参数回读一致。
飞机已离开对接位置，按用户要求不做现场归零残差验证；无效位姿占位值不算验证结果。
当前机载发布仓库为 `wangmingyang-3915/jetson-yahboom`，工作目录仍为
`/home/jetson/yahboom_ws`。此前章节的文件指纹、验证频率与旋转值均属于各次历史记录。

计算公式、参数语义和原始数据见[本次对接校正记录](docking_calibration_2026-10-27_zh.md)。
