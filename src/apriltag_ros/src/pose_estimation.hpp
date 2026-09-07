#pragma once

#include <apriltag/apriltag.h>
#include <array>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>
#include <vector>


struct CameraParameters
{
    std::array<double, 9> matrix;
    std::vector<double> distortion;
};

struct PoseEstimationResult
{
    geometry_msgs::msg::Transform transform;
    double reprojection_error;
};

typedef std::function<std::vector<PoseEstimationResult>(
    apriltag_detection_t* const,
    const CameraParameters&,
    const double&)>
    pose_estimation_f;

extern const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods;
