# 机载相机内参与外参

核验日期：2026-07-27
核验对象：Jetson 机载电脑 `yahboom`
相机用途：AprilTag 相对位姿检测

## 1. 核验结论

本文参数已从机载 Jetson 在线读取，不是仅从工程归档摘录。

- `apriltag-stack.service` 状态为 `active/running`。
- 机载标定 YAML 与运行中的 ROS 2 `/camera_info` 完全一致。
- 机载启动脚本中的静态外参与运行中的 TF 完全一致。
- 当前相机工作模式为 `1280 × 1024 @ 60 FPS`，灰度图 `mono8`。
- 当前有效的相机机体外参是 `body_frd -> camera_link`：
  `[-0.005, 0.086, -0.040] m`，不是旧文档中的
  `[0.000, 0.086, -0.068] m`。

## 2. 相机与参数来源

### 2.1 硬件标识

| 项目 | 当前值 |
|---|---|
| V4L2 产品 | `USB Camera: USB Camera` |
| 厂商 | `Generic` |
| 型号 | `USB_Camera` |
| 序列标识 | `Generic_USB_Camera_YHTek` |
| 稳定设备链接 | `/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0` |
| 本次实际设备 | `/dev/video0` |
| USB 路径 | `platform-3610000.usb-usb-0:1.3:1.0` |
| ROS 相机名 | `usb_camera` |
| 图像/光学坐标系 | `camera_link` |

运行配置使用稳定的 `/dev/v4l/by-id/` 链接寻找设备，不依赖可能随启动顺序变化的
`/dev/video0`。

### 2.2 机载文件

内参文件：

```text
/home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml
SHA-256: 0a56b430343ac91964102da8e86fb622e4585dafea2ec344310f377b65928ea4
修改时间: 2026-07-27 09:46:12 +08:00
```

外参与视觉链路启动脚本：

```text
/home/jetson/yahboom_ws/start_apriltag_stack.sh
SHA-256: 123673ba593ca147b782a6871fd43af93bcbdf17a4f857d3e72015112ff651d3
修改时间: 2026-07-27 12:03:47 +08:00
```

## 3. 相机内参

### 3.1 标定条件

| 项目 | 值 |
|---|---:|
| 图像宽度 | 1280 px |
| 图像高度 | 1024 px |
| 标定板 | Q12-150-10 棋盘格 |
| OpenCV 内角点 | 11 × 8 |
| 单格边长 | 0.010 m |
| 保存视角 | 61 |
| 鲁棒保留视角 | 53 |
| 整体重投影 RMS | 0.19948 px |
| 最大保留单视角 RMS | 0.40712 px |
| 标定日期 | 2026-07-24 |

### 3.2 相机矩阵

采用针孔相机模型：

```text
K = [ fx   0  cx ]
    [  0  fy  cy ]
    [  0   0   1 ]
```

当前数值：

```text
K = [701.08848    0.00000  639.75700]
    [  0.00000  701.07489  494.67863]
    [  0.00000    0.00000    1.00000]
```

| 参数 | 值 | 单位 |
|---|---:|---|
| `fx` | 701.08848 | px |
| `fy` | 701.07489 | px |
| `cx` | 639.75700 | px |
| `cy` | 494.67863 | px |

### 3.3 畸变参数

畸变模型为 ROS/OpenCV `plumb_bob`。系数顺序为
`[k1, k2, p1, p2, k3]`：

```text
D = [0.0194891, -0.0412005, 0.0001929, 0.0002377, 0.0]
```

| 参数 | 含义 | 值 |
|---|---|---:|
| `k1` | 一阶径向畸变 | 0.0194891 |
| `k2` | 二阶径向畸变 | -0.0412005 |
| `p1` | 第一切向畸变 | 0.0001929 |
| `p2` | 第二切向畸变 | 0.0002377 |
| `k3` | 三阶径向畸变 | 0.0 |

### 3.4 校正矩阵与投影矩阵

校正矩阵：

```text
R = [1  0  0]
    [0  1  0]
    [0  0  1]
```

投影矩阵：

```text
P = [691.6428754861    0.0000000000  640.2508905641  0]
    [  0.0000000000  700.0901996257  494.4479699466  0]
    [  0.0000000000    0.0000000000    1.0000000000  0]
```

当前 `/camera_info` 同时表明：

```text
binning_x = 0
binning_y = 0
ROI       = 未启用
do_rectify = false
```

内参只适用于当前相机、镜头状态和 `1280 × 1024` 图像模式。更换相机、镜头，
转动对焦环，或改变裁剪、缩放、像素合并和传感器模式后都应重新标定。

## 4. 相机相对机体外参

### 4.1 坐标系约定

机体坐标系 `body_frd` 使用 FRD：

```text
+X：机头向前
+Y：机体向右
+Z：机体向下
```

相机坐标系 `camera_link` 使用 OpenCV/ROS 光学坐标：

```text
+X：图像向右
+Y：图像向下
+Z：镜头朝向
```

### 4.2 `body_frd -> camera_link`

TF 父坐标系为 `body_frd`，子坐标系为 `camera_link`。

平移，即相机光心在 `body_frd` 中的位置：

```text
t_body_camera = [-0.005, 0.086, -0.040] m
```

旋转四元数，顺序为 ROS `xyzw`：

```text
q_body_camera = [0.0, 0.7071067811865476,
                 0.7071067811865476, 0.0]
```

等效欧拉角仅用于阅读，不建议用于程序内部换算：

```text
RPY = [90°, 0°, 180°]
```

齐次变换定义为：

```text
p_body = T_body_camera · p_camera
```

其矩阵为：

```text
T_body_camera =
[-1  0  0  -0.005]
[ 0  0  1   0.086]
[ 0  1  0  -0.040]
[ 0  0  0   1.000]
```

物理含义：

- 相机光心位于机体中心后方 5 mm、右侧 86 mm、上方 40 mm。
- 相机镜头朝机体右侧：相机 `+Z` 对应机体 `+Y`。
- 图像向右方向，即相机 `+X`，对应机体后方 `-X`。
- 图像向下方向，即相机 `+Y`，对应机体下方 `+Z`。

## 5. 配套 AprilTag 靶标安装外参

此项不是相机内参，但它是当前相对位姿 TF 链的一部分，故完整记录其正向安装
外参和运行时发布的逆向外参。

### 5.1 靶标与目标机体坐标系

目标机体坐标系 `target_body_frd` 同样使用 FRD：

```text
+X：目标机机头向前
+Y：目标机机体向右
+Z：目标机机体向下
```

靶标检测坐标系为 `tag1`。根据当前实机安装外参，其坐标轴与目标机体轴的对应
关系为：

```text
tag1 +X -> target_body_frd +X    # 目标机前方
tag1 +Y -> target_body_frd +Z    # 目标机下方
tag1 +Z -> target_body_frd -Y    # 目标机左侧，标签正面向外
```

因此标签平面法向 `tag1 +Z` 水平指向目标机左侧。

### 5.2 直接安装外参：`target_body_frd -> tag1`

这是最直观的机械安装表达：标签中心在目标机机体坐标系中的位置为

```text
t_target_body_tag = [-0.010, -0.080, -0.036] m
```

即：

- 标签中心位于目标机中心后方 10 mm；
- 标签中心位于目标机中心左侧 80 mm；
- 标签中心位于目标机中心上方 36 mm；
- 标签正面水平朝向目标机左侧。

旋转四元数，顺序为 ROS `xyzw`：

```text
q_target_body_tag = [0.7071067811865476, 0.0, 0.0,
                     0.7071067811865476]
```

等效欧拉角仅用于阅读：

```text
RPY = [90°, 0°, 0°]
```

齐次变换定义为：

```text
p_target_body = T_target_body_tag · p_tag
```

其矩阵为：

```text
T_target_body_tag =
[1  0   0  -0.010]
[0  0  -1  -0.080]
[0  1   0  -0.036]
[0  0   0   1.000]
```

### 5.3 运行时发布外参：`tag1 -> target_body_frd`

`apriltag_ros` 已实时发布 `camera_link -> tag1`。为保持 TF 树中每个子坐标系
只有一个父坐标系，启动脚本没有再次把 `tag1` 作为静态子坐标系，而是发布上述
安装外参的逆变换：

```text
T_tag_target_body = inverse(T_target_body_tag)
```

运行中实际发布的数值为：

```text
translation xyz = [0.010, 0.036, -0.080] m
quaternion xyzw = [-0.7071067811865476, 0.0, 0.0,
                    0.7071067811865476]
RPY degree       = [-90°, 0°, 0°]
```

齐次变换定义为：

```text
p_tag = T_tag_target_body · p_target_body
```

其矩阵为：

```text
T_tag_target_body =
[1   0  0   0.010]
[0   0  1   0.036]
[0  -1  0  -0.080]
[0   0  0   1.000]
```

正向和逆向参数描述的是同一套机械安装关系，不能把两组平移直接互换或只改变
符号；求逆时平移还必须经过旋转矩阵变换。

### 5.4 当前 TF 链

```text
body_frd
  -> camera_link
    -> tag1                 # AprilTag 实时检测
      -> target_body_frd    # 靶标安装外参的逆变换
```

机械安装发生改变，或标签被旋转、翻面、重新粘贴后，必须重新测量
`target_body_frd -> tag1`，再计算并更新启动脚本中的逆变换。

## 6. 在线核验结果

2026-07-27 在线读取结果如下：

| 核验项 | 结果 |
|---|---|
| `apriltag-stack.service` | `active/running` |
| 服务本次启动时间 | 2026-07-27 15:10:10 CST |
| YAML 与 `/camera_info` | 数值一致 |
| 启动脚本与 `body_frd -> camera_link` 实时 TF | 数值一致 |
| 启动脚本与 `tag1 -> target_body_frd` 实时 TF | 数值一致 |
| `/camera_info.frame_id` | `camera_link` |
| `/camera_info` 分辨率 | 1280 × 1024 |

可在 Jetson 上使用以下只读命令复核：

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/yahboom_ws/install/setup.bash
export ROS_DOMAIN_ID=99

ros2 topic echo /camera_info --once
ros2 run tf2_ros tf2_echo body_frd camera_link
ros2 run tf2_ros tf2_echo tag1 target_body_frd
```

## 7. 使用注意事项

1. 程序中必须明确四元数顺序。ROS/TF 使用 `xyzw`，PX4 常见数组使用
   `wxyz`，不能直接照搬。
2. 外参平移是“子坐标系原点在父坐标系中的位置”，不是简单的正负方向描述。
3. 机械安装发生位移或旋转后，必须重新测量外参并做三个轴的方向验证。
4. `docs/relative_pose_ros2_px4_integration_zh.md` 中记录的
   `[0.000, 0.086, -0.068] m` 是旧外参，不应继续用于当前实机。
5. 本文以机载 YAML、当前启动脚本和实时 ROS 2 话题/TF 为最终依据。
