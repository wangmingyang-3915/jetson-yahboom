// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

// apriltag
#include "tag_functions.hpp"
#include <algorithm>
#include <apriltag.h>
#include <cmath>
#include <limits>


#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

const static std::unordered_map<std::string, rmw_qos_profile_t> qos_profiles{
    {"default", rmw_qos_profile_default},
    {"sensor_data", rmw_qos_profile_sensor_data},
    {"system_default", rmw_qos_profile_system_default},
};

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    struct PoseHistory
    {
        geometry_msgs::msg::Quaternion orientation;
        int64_t stamp_ns;
    };

    std::unordered_map<int, PoseHistory> pose_history;

    std::function<void(apriltag_family_t*)> tf_destructor;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_ci;
    std::mutex camera_info_mutex;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img);
    void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);
    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    bool selectPose(
        int tag_id,
        int64_t stamp_ns,
        const std::vector<PoseEstimationResult>& candidates,
        geometry_msgs::msg::Transform& selected);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)


AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    tf_broadcaster(
#ifdef tf2_ros_NODE_INTERFACE
        tf2_ros::TransformBroadcaster::RequiredInterfaces { *this }
#else
        this
#endif
    )
{
    declare_parameter("image_transport", "raw", descr({}, true));
    const auto qos_profile = qos_profiles.at(
        declare_parameter("qos_profile", "default",
                          descr("qos profile to use. 'default', 'sensor_data' or 'system_default'", true)));
    rclcpp::QoS qos{rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile};
    // Detection is intentionally latest-frame-only: after an expensive frame,
    // processing queued stale images only extends the control-loop outage.
    qos.keep_last(1);

    // 相机内参在运行期间保持不变，因此直接缓存最新 CameraInfo，避免为同步时间戳再次订阅和反序列化整幅图像。
    sub_ci = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", qos,
        std::bind(&AprilTagNode::onCameraInfo, this, std::placeholders::_1));
    sub_img = create_subscription<sensor_msgs::msg::Image>(
        "image_rect", qos,
        std::bind(&AprilTagNode::onImage, this, std::placeholders::_1));

    // read-only parameters
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // get tag names, IDs and sizes
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // get method for estimating tag pose
    const std::string& pose_estimation_method =
        declare_parameter("pose_estimation_method", "pnp",
                          descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster), "
                                "set to \"\" (empty) to disable pose estimation",
                                true));

    if(!pose_estimation_method.empty()) {
        if(pose_estimation_methods.count(pose_estimation_method)) {
            estimate_pose = pose_estimation_methods.at(pose_estimation_method);
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown pose estimation method '" << pose_estimation_method << "'.");
        }
    }

    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }
}

AprilTagNode::~AprilTagNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    std::lock_guard<std::mutex> lock(camera_info_mutex);
    latest_camera_info = msg_ci;
}

void AprilTagNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img)
{
    sensor_msgs::msg::CameraInfo::ConstSharedPtr msg_ci;

    {
        std::lock_guard<std::mutex> lock(camera_info_mutex);
        msg_ci = latest_camera_info;
    }

    if(msg_ci) {
        onCamera(msg_img, msg_ci);
    }
}

bool AprilTagNode::selectPose(
    int tag_id,
    int64_t stamp_ns,
    const std::vector<PoseEstimationResult>& candidates,
    geometry_msgs::msg::Transform& selected)
{
    if(candidates.empty()) {
        return false;
    }

    const auto history = pose_history.find(tag_id);
    const bool history_is_fresh =
        history != pose_history.end() &&
        stamp_ns >= history->second.stamp_ns &&
        stamp_ns - history->second.stamp_ns <= 500000000;
    const PoseEstimationResult* best = nullptr;
    double best_score = std::numeric_limits<double>::infinity();

    for(const PoseEstimationResult& candidate : candidates) {
        const auto& q = candidate.transform.rotation;
        const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

        if(!std::isfinite(norm) || norm < 1e-9 ||
           !std::isfinite(candidate.reprojection_error)) {
            continue;
        }

        double score = candidate.reprojection_error;

        if(history_is_fresh) {
            const auto& previous = history->second.orientation;
            const double previous_norm = std::sqrt(
                previous.x * previous.x + previous.y * previous.y +
                previous.z * previous.z + previous.w * previous.w);
            const double dot = std::abs(
                                   q.x * previous.x + q.y * previous.y +
                                   q.z * previous.z + q.w * previous.w) /
                               (norm * previous_norm);
            const double angular_distance =
                2.0 * std::acos(std::min(1.0, std::max(0.0, dot)));

            // At camera frame rates a physical pose is continuous. Angular
            // continuity selects the correct IPPE branch; reprojection error
            // remains a small tie-breaker between otherwise similar branches.
            score = angular_distance + 0.05 * candidate.reprojection_error;
        }

        if(score < best_score) {
            best = &candidate;
            best_score = score;
        }
    }

    if(best == nullptr) {
        return false;
    }

    selected = best->transform;

    if(history_is_fresh) {
        const auto& previous = history->second.orientation;
        const double dot =
            selected.rotation.x * previous.x +
            selected.rotation.y * previous.y +
            selected.rotation.z * previous.z +
            selected.rotation.w * previous.w;

        if(dot < 0.0) {
            selected.rotation.x = -selected.rotation.x;
            selected.rotation.y = -selected.rotation.y;
            selected.rotation.z = -selected.rotation.z;
            selected.rotation.w = -selected.rotation.w;
        }
    }

    pose_history[tag_id] = PoseHistory{selected.rotation, stamp_ns};
    return true;
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // The detector consumes /image_raw, so solve PnP with the raw camera
    // matrix and distortion coefficients instead of rectified P with no D.
    const CameraParameters camera{msg_ci->k, msg_ci->d};

    // check for valid intrinsics
    const bool calibrated = msg_ci->width && msg_ci->height &&
                            camera.matrix[0] && camera.matrix[4] &&
                            camera.matrix[2] && camera.matrix[5] &&
                            camera.matrix[8];

    if(estimate_pose != nullptr && !calibrated) {
        RCLCPP_WARN_STREAM(get_logger(), "The camera is not calibrated! Set 'pose_estimation_method' to \"\" (empty) to disable pose estimation and this warning.");
    }

    // convert to 8bit monochrome image
    const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

    // detect tags
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        RCLCPP_DEBUG(get_logger(),
                     "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id,
                     det->hamming, det->decision_margin);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming > max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
        msg_detections.detections.push_back(msg_detection);

        // 3D orientation and position
        if(estimate_pose != nullptr && calibrated) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_img->header;
            // set child frame name by generic tag name or configured tag name
            tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            const auto candidates = estimate_pose(det, camera, size);

            if(selectPose(
                   det->id,
                   rclcpp::Time(msg_img->header.stamp).nanoseconds(),
                   candidates,
                   tf.transform)) {
                tfs.push_back(tf);
            }
        }
    }

    pub_detections->publish(msg_detections);

    if(estimate_pose != nullptr)
        tf_broadcaster.sendTransform(tfs);

    apriltag_detections_destroy(detections);
}

rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);

        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }

    mutex.unlock();

    result.successful = true;

    return result;
}
