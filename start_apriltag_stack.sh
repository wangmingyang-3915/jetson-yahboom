#!/usr/bin/env bash
# shellcheck disable=SC1091
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

# If NTP changes the clock after gscam starts, exit once synchronization
# completes so systemd restarts the stack with a fresh timestamp offset.
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
    --qx -0.7071067811865476 --qy 0.0 --qz 0.0 --qw 0.7071067811865476 \
    --frame-id tag1 \
    --child-frame-id target_body_frd &
pids+=("$!")

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
    -p target_frame:=target_body_frd \
    -p target_id:=1 \
    -p output_topic:=/uav1/fmu/in/target_relative_pose \
    -p publish_rate_hz:=60.0 \
    -p max_pose_age_s:=0.2 &
pids+=("$!")

/home/jetson/yahboom_ws/start_xrce_agent_ethernet.sh &
pids+=("$!")

wait -n "${pids[@]}"
exit 1
