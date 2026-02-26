#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <optional>

struct FeatureMatch {
    cv::KeyPoint kp_left;
    cv::KeyPoint kp_right;
    float distance;
    cv::Point2f pt_left;
    cv::Point2f pt_right;
};

struct StereoResult {
    cv::Mat fundamental_matrix;
    cv::Mat essential_matrix;
    cv::Mat rotation;
    cv::Mat translation;
    std::vector<FeatureMatch> inlier_matches;
    std::vector<cv::Point3f> triangulated_points;
    double reprojection_error;
};

class StereoMatcher {
public:
    enum class DetectorType {
        ORB,
        AKAZE,
        BRISK
    };

    StereoMatcher(DetectorType type = DetectorType::AKAZE);

    void set_max_features(int n);
    void set_ratio_threshold(float ratio);
    void set_ransac_threshold(double threshold);
    void set_camera_matrix(const cv::Mat& K);

    std::vector<FeatureMatch> find_matches(const cv::Mat& img1, const cv::Mat& img2) const;
    std::vector<FeatureMatch> find_matches_in_roi(const cv::Mat& img1, const cv::Mat& img2,
                                                   const cv::Rect& roi1, const cv::Rect& roi2) const;

    std::optional<StereoResult> compute_stereo(const cv::Mat& img1, const cv::Mat& img2,
                                                const std::vector<FeatureMatch>& matches) const;

    cv::Mat visualize_matches(const cv::Mat& img1, const cv::Mat& img2,
                              const std::vector<FeatureMatch>& matches) const;

    cv::Mat visualize_epipolar(const cv::Mat& img1, const cv::Mat& img2,
                                const cv::Mat& F,
                                const std::vector<FeatureMatch>& matches) const;

private:
    DetectorType detector_type_;
    int max_features_;
    float ratio_threshold_;
    double ransac_threshold_;
    cv::Mat camera_matrix_;

    cv::Ptr<cv::Feature2D> create_detector() const;
    cv::Ptr<cv::DescriptorMatcher> create_matcher() const;

    std::vector<FeatureMatch> filter_matches_ratio_test(
        const std::vector<std::vector<cv::DMatch>>& knn_matches,
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::KeyPoint>& kp2) const;
};
