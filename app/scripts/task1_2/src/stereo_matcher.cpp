#include "stereo_matcher.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <numeric>
#include <cmath>

StereoMatcher::StereoMatcher(DetectorType type)
    : detector_type_(type)
    , max_features_(5000)
    , ratio_threshold_(0.75f)
    , ransac_threshold_(3.0) {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
}

void StereoMatcher::set_max_features(int n) { max_features_ = n; }
void StereoMatcher::set_ratio_threshold(float ratio) { ratio_threshold_ = ratio; }
void StereoMatcher::set_ransac_threshold(double threshold) { ransac_threshold_ = threshold; }
void StereoMatcher::set_camera_matrix(const cv::Mat& K) { camera_matrix_ = K.clone(); }

cv::Ptr<cv::Feature2D> StereoMatcher::create_detector() const {
    switch (detector_type_) {
        case DetectorType::ORB:
            return cv::ORB::create(max_features_, 1.2f, 8, 31,
                                    0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
        case DetectorType::AKAZE:
            return cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3,
                                      0.001f, 4, 4);
        case DetectorType::BRISK:
            return cv::BRISK::create(30, 3, 1.0f);
        default:
            return cv::AKAZE::create();
    }
}

cv::Ptr<cv::DescriptorMatcher> StereoMatcher::create_matcher() const {
    if (detector_type_ == DetectorType::ORB || detector_type_ == DetectorType::BRISK) {
        return cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    }
    return cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
}

std::vector<FeatureMatch> StereoMatcher::filter_matches_ratio_test(
    const std::vector<std::vector<cv::DMatch>>& knn_matches,
    const std::vector<cv::KeyPoint>& kp1,
    const std::vector<cv::KeyPoint>& kp2) const {

    std::vector<FeatureMatch> good_matches;

    for (const auto& match_pair : knn_matches) {
        if (match_pair.size() < 2) continue;

        if (match_pair[0].distance < ratio_threshold_ * match_pair[1].distance) {
            FeatureMatch fm;
            fm.kp_left = kp1[match_pair[0].queryIdx];
            fm.kp_right = kp2[match_pair[0].trainIdx];
            fm.distance = match_pair[0].distance;
            fm.pt_left = fm.kp_left.pt;
            fm.pt_right = fm.kp_right.pt;
            good_matches.push_back(fm);
        }
    }

    return good_matches;
}

std::vector<FeatureMatch> StereoMatcher::find_matches(
    const cv::Mat& img1, const cv::Mat& img2) const {

    cv::Mat gray1, gray2;
    if (img1.channels() == 3) cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    else gray1 = img1.clone();
    if (img2.channels() == 3) cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);
    else gray2 = img2.clone();

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray1, gray1);
    clahe->apply(gray2, gray2);

    auto detector = create_detector();

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    detector->detectAndCompute(gray1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(gray2, cv::noArray(), keypoints2, descriptors2);

    std::cout << "[StereoMatcher] Found " << keypoints1.size()
              << " / " << keypoints2.size() << " keypoints\n";

    if (descriptors1.empty() || descriptors2.empty()) {
        std::cerr << "[StereoMatcher] Warning: no descriptors found\n";
        return {};
    }

    auto matcher = create_matcher();
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

    auto good_matches = filter_matches_ratio_test(knn_matches, keypoints1, keypoints2);

    std::cout << "[StereoMatcher] " << good_matches.size()
              << " matches after ratio test\n";

    return good_matches;
}

std::vector<FeatureMatch> StereoMatcher::find_matches_in_roi(
    const cv::Mat& img1, const cv::Mat& img2,
    const cv::Rect& roi1, const cv::Rect& roi2) const {

    int pad = 50;
    cv::Rect expanded_roi1(
        std::max(0, roi1.x - pad), std::max(0, roi1.y - pad),
        std::min(img1.cols - std::max(0, roi1.x - pad), roi1.width + 2 * pad),
        std::min(img1.rows - std::max(0, roi1.y - pad), roi1.height + 2 * pad)
    );
    cv::Rect expanded_roi2(
        std::max(0, roi2.x - pad), std::max(0, roi2.y - pad),
        std::min(img2.cols - std::max(0, roi2.x - pad), roi2.width + 2 * pad),
        std::min(img2.rows - std::max(0, roi2.y - pad), roi2.height + 2 * pad)
    );

    cv::Mat crop1 = img1(expanded_roi1);
    cv::Mat crop2 = img2(expanded_roi2);

    auto matches = find_matches(crop1, crop2);

    for (auto& m : matches) {
        m.pt_left.x += expanded_roi1.x;
        m.pt_left.y += expanded_roi1.y;
        m.kp_left.pt = m.pt_left;
        m.pt_right.x += expanded_roi2.x;
        m.pt_right.y += expanded_roi2.y;
        m.kp_right.pt = m.pt_right;
    }

    return matches;
}

std::optional<StereoResult> StereoMatcher::compute_stereo(
    const cv::Mat& img1, const cv::Mat& img2,
    const std::vector<FeatureMatch>& matches) const {

    if (matches.size() < 8) {
        std::cerr << "[StereoMatcher] Not enough matches for stereo computation ("
                  << matches.size() << " < 8)\n";
        return std::nullopt;
    }

    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& m : matches) {
        pts1.push_back(m.pt_left);
        pts2.push_back(m.pt_right);
    }

    std::vector<uchar> inlier_mask;
    cv::Mat F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC,
                                        ransac_threshold_, 0.999, inlier_mask);

    if (F.empty()) {
        std::cerr << "[StereoMatcher] Failed to find fundamental matrix\n";
        return std::nullopt;
    }

    StereoResult result;
    result.fundamental_matrix = F;

    for (size_t i = 0; i < matches.size(); ++i) {
        if (inlier_mask[i]) {
            result.inlier_matches.push_back(matches[i]);
        }
    }

    std::cout << "[StereoMatcher] " << result.inlier_matches.size()
              << " inlier matches after RANSAC\n";

    cv::Mat K = camera_matrix_;
    if (K.at<double>(0, 0) == 1.0 && K.at<double>(1, 1) == 1.0) {
        double focal = std::max(img1.cols, img1.rows) * 1.2;
        K.at<double>(0, 0) = focal;
        K.at<double>(1, 1) = focal;
        K.at<double>(0, 2) = img1.cols / 2.0;
        K.at<double>(1, 2) = img1.rows / 2.0;
        std::cout << "[StereoMatcher] Using estimated camera matrix (f="
                  << focal << "px)\n";
    }

    result.essential_matrix = K.t() * F * K;

    std::vector<cv::Point2f> inlier_pts1, inlier_pts2;
    for (const auto& m : result.inlier_matches) {
        inlier_pts1.push_back(m.pt_left);
        inlier_pts2.push_back(m.pt_right);
    }

    cv::Mat R, t;
    cv::Mat E_recovered;
    std::vector<uchar> pose_mask;

    E_recovered = cv::findEssentialMat(inlier_pts1, inlier_pts2, K,
                                        cv::RANSAC, 0.999, 1.0, pose_mask);

    if (!E_recovered.empty()) {
        result.essential_matrix = E_recovered;
        cv::recoverPose(E_recovered, inlier_pts1, inlier_pts2, K, R, t, pose_mask);
    } else {
        cv::decomposeEssentialMat(result.essential_matrix, R, t, cv::noArray());
    }

    result.rotation = R;
    result.translation = t;

    cv::Mat P1 = cv::Mat::eye(3, 4, CV_64F);
    cv::Mat P2(3, 4, CV_64F);
    R.copyTo(P2(cv::Rect(0, 0, 3, 3)));
    t.copyTo(P2(cv::Rect(3, 0, 1, 3)));

    cv::Mat proj1 = K * P1;
    cv::Mat proj2 = K * P2;

    cv::Mat points4D;
    cv::triangulatePoints(proj1, proj2, inlier_pts1, inlier_pts2, points4D);

    result.triangulated_points.clear();
    double total_reproj_error = 0.0;
    int valid_count = 0;

    for (int i = 0; i < points4D.cols; i++) {
        float w = points4D.at<float>(3, i);
        if (std::abs(w) < 1e-10) continue;

        cv::Point3f pt(
            points4D.at<float>(0, i) / w,
            points4D.at<float>(1, i) / w,
            points4D.at<float>(2, i) / w
        );

        if (pt.z > 0) {
            result.triangulated_points.push_back(pt);

            cv::Mat pt_h = (cv::Mat_<double>(4, 1) << pt.x, pt.y, pt.z, 1.0);
            cv::Mat reproj1 = proj1 * pt_h;
            cv::Mat reproj2 = proj2 * pt_h;

            cv::Point2f rp1(reproj1.at<double>(0) / reproj1.at<double>(2),
                            reproj1.at<double>(1) / reproj1.at<double>(2));
            cv::Point2f rp2(reproj2.at<double>(0) / reproj2.at<double>(2),
                            reproj2.at<double>(1) / reproj2.at<double>(2));

            if (i < static_cast<int>(inlier_pts1.size())) {
                double e1 = cv::norm(rp1 - inlier_pts1[i]);
                double e2 = cv::norm(rp2 - inlier_pts2[i]);
                total_reproj_error += (e1 + e2) / 2.0;
                valid_count++;
            }
        }
    }

    result.reprojection_error = valid_count > 0 ? total_reproj_error / valid_count : -1.0;

    std::cout << "[StereoMatcher] Triangulated " << result.triangulated_points.size()
              << " 3D points, reproj error = " << result.reprojection_error << "px\n";

    return result;
}

cv::Mat StereoMatcher::visualize_matches(const cv::Mat& img1, const cv::Mat& img2,
                                          const std::vector<FeatureMatch>& matches) const {
    int w = std::max(img1.cols, img2.cols);
    int h1 = img1.rows, h2 = img2.rows;
    cv::Mat canvas(h1 + h2, w, CV_8UC3, cv::Scalar(0));

    img1.copyTo(canvas(cv::Rect(0, 0, img1.cols, h1)));
    img2.copyTo(canvas(cv::Rect(0, h1, img2.cols, h2)));

    for (size_t i = 0; i < matches.size(); ++i) {
        cv::Scalar color(
            static_cast<int>(i * 47) % 256,
            static_cast<int>(i * 113) % 256,
            static_cast<int>(i * 191) % 256
        );

        cv::Point2f pt1 = matches[i].pt_left;
        cv::Point2f pt2 = matches[i].pt_right;
        pt2.y += h1;

        cv::circle(canvas, pt1, 4, color, -1);
        cv::circle(canvas, pt2, 4, color, -1);
        cv::line(canvas, pt1, pt2, color, 1);
    }

    return canvas;
}

cv::Mat StereoMatcher::visualize_epipolar(const cv::Mat& img1, const cv::Mat& img2,
                                            const cv::Mat& F,
                                            const std::vector<FeatureMatch>& matches) const {
    cv::Mat vis1 = img1.clone(), vis2 = img2.clone();

    for (size_t i = 0; i < std::min(matches.size(), size_t(20)); ++i) {
        cv::Scalar color(
            static_cast<int>(i * 47) % 256,
            static_cast<int>(i * 113) % 256,
            static_cast<int>(i * 191) % 256
        );

        cv::circle(vis1, matches[i].pt_left, 5, color, -1);
        cv::circle(vis2, matches[i].pt_right, 5, color, -1);

        std::vector<cv::Vec3f> epilines;
        std::vector<cv::Point2f> pts = {matches[i].pt_left};
        cv::computeCorrespondEpilines(pts, 1, F, epilines);

        float a = epilines[0][0], b = epilines[0][1], c = epilines[0][2];
        cv::Point pt1(0, static_cast<int>(-c / b));
        cv::Point pt2(img2.cols, static_cast<int>(-(c + a * img2.cols) / b));
        cv::line(vis2, pt1, pt2, color, 1);
    }

    int max_h = std::max(vis1.rows, vis2.rows);
    cv::Mat padded1, padded2;
    cv::copyMakeBorder(vis1, padded1, 0, max_h - vis1.rows, 0, 0,
                       cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::copyMakeBorder(vis2, padded2, 0, max_h - vis2.rows, 0, 0,
                       cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    cv::Mat canvas;
    cv::hconcat(padded1, padded2, canvas);
    return canvas;
}
