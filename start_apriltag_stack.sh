#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/jetson/yahboom_ws/install/setup.bash

set -u

pids=()

camera_link=/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0
camera_device=

for _ in {1..30}; do
    if [[ -e "$camera_link" ]]; then
        camera_device=$(readlink -f "$camera_link")
        break
    fi

    sleep 1
done

if [[ -z "$camera_device" ]]; then
    echo "未找到 AprilTag USB 摄像头：$camera_link" >&2
    exit 1
fi

cleanup() {
    trap - EXIT INT TERM

    if ((${#pids[@]})); then
        kill "${pids[@]}" 2>/dev/null || true
        wait "${pids[@]}" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

# Jetson 在无 RTC 或 RTC 时间不准时会先启动相机，随后由 NTP 跳变系统时间。
# gscam 的 GStreamer 时间偏移只在启动时计算一次；若本次启动尚未完成校时，
# 后台等待同步标记出现后退出，让 systemd 自动重启整套链路并刷新时间偏移。
# 等待过程不阻塞视觉启动，离线运行时也不会受影响。
if [[ ! -e /run/systemd/timesync/synchronized ]]; then
    (
        while [[ ! -e /run/systemd/timesync/synchronized ]]; do
            sleep 1
        done
        echo "系统时间已同步，请求 systemd 重启视觉链路以刷新相机时间戳"
    ) &
    pids+=("$!")
fi

/opt/ros/humble/lib/tf2_ros/static_transform_publisher \
    --x -0.005 --y 0.086 --z -0.040 \
    --qx 0.0 --qy 0.7071067811865476 --qz 0.7071067811865476 --qw 0.0 \
    --frame-id body_frd \
    --child-frame-id camera_link &
pids+=("$!")

/opt/ros/humble/lib/tf2_ros/static_transform_publisher \
    --x 0.010 --y 0.036 --z -0.080 \
    --qx -0.509742963809257 --qy 0.510702549407070 --qz -0.488625888120183 --qw 0.490499498812014 \
    --frame-id tag0 \
    --child-frame-id target_body_frd_0 &
pids+=("$!")

/opt/ros/humble/lib/tf2_ros/static_transform_publisher \
    --x 0.010 --y 0.036 --z -0.080 \
    --qx -0.509742963809257 --qy 0.510702549407070 --qz -0.488625888120183 --qw 0.490499498812014 \
    --frame-id tag1 \
    --child-frame-id target_body_frd &
pids+=("$!")

/opt/ros/humble/lib/tf2_ros/static_transform_publisher \
    --x 0.010 --y 0.036 --z -0.080 \
    --qx -0.509742963809257 --qy 0.510702549407070 --qz -0.488625888120183 --qw 0.490499498812014 \
    --frame-id tag2 \
    --child-frame-id target_body_frd_2 &
pids+=("$!")

# Jetson 硬件 MJPEG 解码，并直接输出 AprilTag 所需的单通道灰度图。
gscam_pipeline="v4l2src device=${camera_device} io-mode=2 do-timestamp=true ! image/jpeg,width=1280,height=1024,framerate=60/1 ! nvjpegdec ! nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! nvvidconv ! video/x-raw,format=GRAY8"

/opt/ros/humble/lib/gscam/gscam_node \
    --ros-args \
    -r camera/image_raw:=/image_raw \
    -r camera/camera_info:=/camera_info \
    -p gscam_config:="$gscam_pipeline" \
    -p camera_name:=usb_camera \
    -p camera_info_url:=file:///home/jetson/.ros/camera_info/decxin_camera:_decxin_camera.yaml \
    -p frame_id:=camera_link \
    -p use_gst_timestamps:=true \
    -p sync_sink:=false \
    -p image_encoding:=mono8 &
pids+=("$!")

sleep 2

/home/jetson/yahboom_ws/install/apriltag_ros/lib/apriltag_ros/apriltag_node \
    --ros-args \
    -r image_rect:=/image_raw \
    -r camera_info:=/camera_info \
    --params-file /home/jetson/yahboom_ws/src/apriltag_ros/cfg/tags_36h11.yaml &
pids+=("$!")

/home/jetson/yahboom_ws/install/target_relative_pose_bridge/lib/target_relative_pose_bridge/target_relative_pose_bridge \
    --ros-args \
    -p parent_frame:=body_frd \
    -p "target_frames:=[target_body_frd_0, target_body_frd, target_body_frd_2]" \
    -p "target_ids:=[0, 1, 2]" \
    -p output_topic:=/uav1/fmu/in/target_relative_pose \
    -p publish_rate_hz:=60.0 \
    -p offboard_heartbeat_rate_hz:=20.0 \
    -p max_pose_age_s:=0.2 \
    -p dropout_grace_s:=0.0 \
    -p offboard_ready_timeout_s:=1.5 \
    -p position_jump_m:=0.2 \
    -p position_rate_limit_m_s:=3.0 \
    -p orientation_jump_rad:=0.2 \
    -p orientation_rate_limit_rad_s:=2.0 &
pids+=("$!")

/home/jetson/yahboom_ws/start_xrce_agent_ethernet.sh &
pids+=("$!")

# 任一节点退出时，让 systemd 重启整套视觉链路。
wait -n "${pids[@]}"
exit 1
