#include "pose_estimation.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <tf2/convert.h>

std::vector<PoseEstimationResult>
pnp(apriltag_detection_t* const detection, const CameraParameters& camera, double tagsize)
{
    // Keep the existing AprilTag frame convention: X points down in the tag
    // image, Y points right and Z points out of the printed front face.
    const std::vector<cv::Point3d> object_points{
        {-tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, +tagsize / 2, 0},
        {-tagsize / 2, +tagsize / 2, 0},
    };

    const std::vector<cv::Point2d> image_points{
        {detection->p[0][0], detection->p[0][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[3][0], detection->p[3][1]},
    };

    const cv::Matx33d camera_matrix{
        camera.matrix[0],
        camera.matrix[1],
        camera.matrix[2],
        camera.matrix[3],
        camera.matrix[4],
        camera.matrix[5],
        camera.matrix[6],
        camera.matrix[7],
        camera.matrix[8],
    };
    const cv::Mat distortion_coefficients{camera.distortion};

    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;
    cv::solvePnPGeneric(
        object_points,
        image_points,
        camera_matrix,
        distortion_coefficients,
        rotation_vectors,
        translation_vectors,
        false,
        cv::SOLVEPNP_IPPE);

    // IPPE returns both valid planar pose branches. Preserve both here so the
    // node can choose the branch that is continuous with the previous frame.
    std::vector<PoseEstimationResult> results;
    results.reserve(rotation_vectors.size());

    for(std::size_t i = 0; i < rotation_vectors.size(); ++i) {
        const cv::Mat_<double> rotation_vector = rotation_vectors[i];
        const cv::Mat_<double> translation_vector = translation_vectors[i];

        if(translation_vector(2) <= 0.0) {
            continue;
        }

        std::vector<cv::Point2d> projected_points;
        cv::projectPoints(
            object_points,
            rotation_vector,
            translation_vector,
            camera_matrix,
            distortion_coefficients,
            projected_points);

        double squared_error = 0.0;

        for(std::size_t point = 0; point < image_points.size(); ++point) {
            const double dx = projected_points[point].x - image_points[point].x;
            const double dy = projected_points[point].y - image_points[point].y;
            squared_error += dx * dx + dy * dy;
        }

        PoseEstimationResult result;
        result.transform = tf2::toMsg<
            std::pair<cv::Mat_<double>, cv::Mat_<double>>,
            geometry_msgs::msg::Transform>(
            std::make_pair(translation_vector, rotation_vector));
        result.reprojection_error = std::sqrt(squared_error / image_points.size());
        results.push_back(result);
    }

    return results;
}

const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods{
    {"pnp", pnp},
};
