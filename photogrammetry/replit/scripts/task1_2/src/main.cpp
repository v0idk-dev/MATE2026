#include "purple_detector.hpp"
#include "stereo_matcher.hpp"
#include "distance_calculator.hpp"
#include "calibration.hpp"
#include "pipe_detector.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>

namespace fs = std::filesystem;

struct ProgramConfig {
    std::vector<std::string> image_paths;
    std::string calibration_file;
    double focal_length_mm = 35.0;
    double sensor_width_mm = 36.0;
    double baseline_meters = 0.1;
    double plate_width_meters = 0.3;
    double plate_height_meters = 0.2;
    bool save_debug = false;
    std::string output_dir = "output";
    bool verbose = true;
    std::string mode = "plates";
    double ref_square_side_cm = 10.0;
};

void print_banner() {
    std::cout << "\n"
              << "====================================================================\n"
              << "   STEREO VISION DISTANCE CALCULATOR - Purple Plate Detection\n"
              << "====================================================================\n"
              << "   Multi-method distance estimation using stereo correspondence,\n"
              << "   triangulation, disparity analysis, and known-size projection.\n"
              << "====================================================================\n\n";
}

void print_usage(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [OPTIONS] <image1> <image2> [image3 ...]\n\n"
              << "Required:\n"
              << "  image1, image2    At least two images from different viewpoints\n\n"
              << "Options:\n"
              << "  --focal <mm>      Camera focal length in mm (default: 35.0)\n"
              << "  --sensor <mm>     Sensor width in mm (default: 36.0)\n"
              << "  --baseline <m>    Known baseline between cameras in meters (default: 0.1)\n"
              << "  --plate-width <m> Known plate width in meters (default: 0.3)\n"
              << "  --plate-height <m> Known plate height in meters (default: 0.2)\n"
              << "  --calibration <f> Camera calibration file (YAML/XML)\n"
              << "  --save-debug      Save debug/visualization images\n"
              << "  --output <dir>    Output directory for debug images (default: output)\n"
              << "  --help            Show this help message\n\n"
              << "Examples:\n"
              << "  " << prog_name << " left.jpg right.jpg\n"
              << "  " << prog_name << " --focal 50 --baseline 0.15 img1.png img2.png img3.png\n"
              << "  " << prog_name << " --calibration calib.yml --save-debug left.jpg right.jpg\n\n";
}

ProgramConfig parse_args(int argc, char** argv) {
    ProgramConfig config;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else if (arg == "--focal" && i + 1 < argc) {
            config.focal_length_mm = std::stod(argv[++i]);
        } else if (arg == "--sensor" && i + 1 < argc) {
            config.sensor_width_mm = std::stod(argv[++i]);
        } else if (arg == "--baseline" && i + 1 < argc) {
            config.baseline_meters = std::stod(argv[++i]);
        } else if (arg == "--plate-width" && i + 1 < argc) {
            config.plate_width_meters = std::stod(argv[++i]);
        } else if (arg == "--plate-height" && i + 1 < argc) {
            config.plate_height_meters = std::stod(argv[++i]);
        } else if (arg == "--calibration" && i + 1 < argc) {
            config.calibration_file = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            config.mode = argv[++i];
        } else if (arg == "--ref-square-side" && i + 1 < argc) {
            config.ref_square_side_cm = std::stod(argv[++i]);
        } else if (arg == "--save-debug") {
            config.save_debug = true;
        } else if (arg == "--output" && i + 1 < argc) {
            config.output_dir = argv[++i];
        } else if (arg[0] != '-') {
            config.image_paths.push_back(arg);
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            std::exit(1);
        }
    }

    return config;
}

void print_separator(const std::string& title = "") {
    if (title.empty()) {
        std::cout << std::string(68, '-') << "\n";
    } else {
        int pad = (66 - static_cast<int>(title.size())) / 2;
        std::cout << std::string(std::max(1, pad), '-') << " " << title << " "
                  << std::string(std::max(1, 66 - pad - static_cast<int>(title.size())), '-') << "\n";
    }
}

void print_detection_info(const PlateDetection& det, int idx) {
    std::cout << "  Plate #" << idx << ":\n"
              << "    Centroid:      (" << std::fixed << std::setprecision(1)
              << det.centroid.x << ", " << det.centroid.y << ")\n"
              << "    Bounding Box:  [" << det.bounding_box.x << ", " << det.bounding_box.y
              << ", " << det.bounding_box.width << "x" << det.bounding_box.height << "]\n"
              << "    Rotated Rect:  " << std::setprecision(1)
              << det.rotated_rect.size.width << " x " << det.rotated_rect.size.height
              << " @ " << det.rotated_rect.angle << " deg\n"
              << "    Area:          " << std::setprecision(0) << det.area << " px^2\n"
              << "    Confidence:    " << std::setprecision(1) << det.confidence << "%\n";
}

void print_distance_estimate(const DistanceEstimate& est) {
    std::cout << "    Method: " << est.method << "\n";
    if (est.distance > 0) {
        double inches = est.distance * 39.3701;
        std::cout << "      Distance:    " << std::fixed << std::setprecision(4)
                  << est.distance << " m";
        std::cout << " (" << std::setprecision(1) << inches << " in)";
        std::cout << "\n";
        std::cout << "      3D Point:    (" << std::setprecision(4)
                  << est.world_point.x << ", " << est.world_point.y << ", "
                  << est.world_point.z << ")\n";
        std::cout << "      Confidence:  " << std::setprecision(1) << est.confidence << "%\n";
        if (est.disparity > 0) {
            std::cout << "      Disparity:   " << std::setprecision(2) << est.disparity << " px\n";
        }
    } else {
        std::cout << "      Distance:    INVALID (insufficient data)\n";
    }
}

struct PlateDistanceResult {
    int plate_index;
    PlateDetection left_plate;
    PlateDetection right_plate;
    double distance_meters;
    double distance_inches;
    std::vector<DistanceEstimate> estimates;
};

double compute_consensus_distance(const std::vector<DistanceEstimate>& estimates) {
    std::vector<double> valid_distances;
    for (const auto& est : estimates) {
        if (est.distance > 0 && est.confidence > 10.0) {
            valid_distances.push_back(est.distance);
        }
    }

    if (valid_distances.empty()) return -1.0;

    std::sort(valid_distances.begin(), valid_distances.end());
    double median = valid_distances[valid_distances.size() / 2];

    std::vector<double> filtered;
    for (double d : valid_distances) {
        if (std::abs(d - median) < median * 2.0) {
            filtered.push_back(d);
        }
    }

    if (filtered.empty()) return median;

    double robust_mean = 0;
    for (double d : filtered) robust_mean += d;
    robust_mean /= filtered.size();

    return robust_mean;
}

std::vector<std::pair<int, int>> match_plates_between_images(
    const std::vector<PlateDetection>& left_plates,
    const std::vector<PlateDetection>& right_plates) {

    std::vector<std::pair<int, int>> matches;
    std::vector<bool> right_used(right_plates.size(), false);

    for (size_t i = 0; i < left_plates.size(); ++i) {
        double best_score = 1e9;
        int best_j = -1;

        for (size_t j = 0; j < right_plates.size(); ++j) {
            if (right_used[j]) continue;

            const auto& lp = left_plates[i];
            const auto& rp = right_plates[j];

            double y_diff = std::abs(lp.centroid.y - rp.centroid.y);
            double y_norm = y_diff / std::max(1.0, (double)lp.bounding_box.height);

            double lw = std::max(lp.rotated_rect.size.width, lp.rotated_rect.size.height);
            double lh = std::min(lp.rotated_rect.size.width, lp.rotated_rect.size.height);
            double rw = std::max(rp.rotated_rect.size.width, rp.rotated_rect.size.height);
            double rh = std::min(rp.rotated_rect.size.width, rp.rotated_rect.size.height);

            double aspect_l = (lh > 0) ? lw / lh : 999;
            double aspect_r = (rh > 0) ? rw / rh : 999;
            double aspect_diff = std::abs(aspect_l - aspect_r);

            double area_ratio = std::min(lp.area, rp.area) / std::max(lp.area, rp.area);

            double score = y_norm * 10.0 + aspect_diff * 5.0 + (1.0 - area_ratio) * 5.0;

            if (score < best_score) {
                best_score = score;
                best_j = static_cast<int>(j);
            }
        }

        if (best_j >= 0 && best_score < 20.0) {
            matches.push_back({static_cast<int>(i), best_j});
            right_used[best_j] = true;
        }
    }

    return matches;
}

cv::Mat create_annotated_image(
    const cv::Mat& image,
    const std::vector<PlateDetection>& detections,
    const std::vector<PlateDistanceResult>& results) {

    cv::Mat vis = image.clone();

    std::map<int, const PlateDistanceResult*> plate_results;
    for (const auto& r : results) {
        plate_results[r.plate_index] = &r;
    }

    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];
        bool has_distance = plate_results.count(static_cast<int>(i)) > 0;

        cv::Scalar box_color = has_distance ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 128, 255);

        cv::Point2f vertices[4];
        det.rotated_rect.points(vertices);
        for (int j = 0; j < 4; j++) {
            cv::line(vis, vertices[j], vertices[(j + 1) % 4], box_color, 2);
        }

        cv::circle(vis, det.centroid, 5, cv::Scalar(0, 0, 255), -1);

        if (has_distance) {
            const auto* res = plate_results[static_cast<int>(i)];
            std::ostringstream dist_ss;
            dist_ss << std::fixed << std::setprecision(1) << res->distance_inches << " in";
            std::string dist_label = dist_ss.str();

            std::ostringstream conf_ss;
            conf_ss << "Plate #" << (i + 1) << " conf=" << std::fixed
                    << std::setprecision(0) << det.confidence << "%";
            std::string conf_label = conf_ss.str();

            int font = cv::FONT_HERSHEY_SIMPLEX;
            double dist_scale = 0.7;
            double conf_scale = 0.45;
            int dist_thickness = 2;
            int conf_thickness = 1;

            int dist_baseline = 0, conf_baseline = 0;
            cv::Size dist_size = cv::getTextSize(dist_label, font, dist_scale, dist_thickness, &dist_baseline);
            cv::Size conf_size = cv::getTextSize(conf_label, font, conf_scale, conf_thickness, &conf_baseline);

            int label_w = std::max(dist_size.width, conf_size.width) + 10;
            int label_h = dist_size.height + conf_size.height + 15;

            int label_x = det.bounding_box.x;
            int label_y = det.bounding_box.y - label_h - 5;
            if (label_y < 0) label_y = det.bounding_box.y + det.bounding_box.height + 5;

            label_x = std::max(0, std::min(label_x, vis.cols - label_w));
            label_y = std::max(0, std::min(label_y, vis.rows - label_h));

            cv::rectangle(vis,
                cv::Point(label_x, label_y),
                cv::Point(label_x + label_w, label_y + label_h),
                cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(vis,
                cv::Point(label_x, label_y),
                cv::Point(label_x + label_w, label_y + label_h),
                box_color, 1);

            cv::putText(vis, dist_label,
                cv::Point(label_x + 5, label_y + dist_size.height + 5),
                font, dist_scale, cv::Scalar(0, 255, 255), dist_thickness);

            cv::putText(vis, conf_label,
                cv::Point(label_x + 5, label_y + dist_size.height + conf_size.height + 12),
                font, conf_scale, cv::Scalar(180, 180, 180), conf_thickness);
        } else {
            std::ostringstream ss;
            ss << "Plate #" << (i + 1) << " conf=" << std::fixed
               << std::setprecision(0) << det.confidence << "%";
            std::string label = ss.str();
            cv::putText(vis, label,
                cv::Point(det.bounding_box.x, det.bounding_box.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 128, 255), 1);
        }
    }

    return vis;
}

struct ImageData {
    cv::Mat image;
    std::string path;
    std::vector<PlateDetection> all_detections;
    std::vector<PlateDetection> high_conf_plates;
};

int run_pipe_mode(const ProgramConfig& config) {
    print_separator("Mode: PVC Pipe Length Measurement");
    std::cout << "  Reference square side: " << config.ref_square_side_cm << " cm\n\n";

    std::string img_path = config.image_paths[0];

    if (!fs::exists(img_path)) {
        std::cerr << "  ERROR: File not found: " << img_path << "\n";
        return 1;
    }

    cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "  ERROR: Failed to load image: " << img_path << "\n";
        return 1;
    }

    std::cout << "  Loaded: " << img_path << " (" << image.cols << "x"
              << image.rows << ")\n\n";

    print_separator("Reference Square Detection");

    ReferenceSquareDetector ref_detector;
    ref_detector.set_known_side_cm(config.ref_square_side_cm);
    ref_detector.set_min_area(100);
    ref_detector.set_max_area(image.cols * image.rows * 0.1);

    auto ref_squares = ref_detector.detect(image);
    std::cout << "  Found " << ref_squares.size() << " candidate reference square(s)\n";

    for (size_t i = 0; i < ref_squares.size(); ++i) {
        const auto& sq = ref_squares[i];
        std::cout << "    Square #" << (i + 1) << ": " << sq.color_name
                  << " | side=" << std::fixed << std::setprecision(0)
                  << sq.side_length_pixels << " px | conf="
                  << std::setprecision(0) << sq.confidence << "%\n";
    }

    ScaleInfo scale = ref_detector.compute_scale(ref_squares);

    if (scale.pixels_per_cm > 0) {
        std::cout << "\n  Scale calibration:\n";
        std::cout << "    Pixels per cm:    " << std::setprecision(2) << scale.pixels_per_cm << "\n";
        std::cout << "    Cm per pixel:     " << std::setprecision(4) << scale.cm_per_pixel << "\n";
        std::cout << "    References used:  " << scale.num_references << "\n";
        std::cout << "    Scale confidence: " << std::setprecision(0) << scale.scale_confidence << "%\n";
    } else {
        std::cout << "\n  WARNING: No reference squares detected for scale calibration.\n";
        std::cout << "  Pipe lengths will be reported in pixels only.\n";
    }

    print_separator("PVC Pipe Detection");

    PipeDetector pipe_detector;
    double min_dim = std::min(image.cols, image.rows);
    pipe_detector.set_min_pipe_length_pixels(min_dim * 0.08);
    pipe_detector.set_hough_params(1.0, 1.0, 60, min_dim * 0.06, 10.0);
    pipe_detector.set_pipe_merge_angle_threshold(8.0);
    pipe_detector.set_pipe_merge_distance_threshold(15.0);

    auto pipes = pipe_detector.detect(image);
    std::cout << "  Found " << pipes.size() << " pipe segment(s)\n\n";

    for (size_t i = 0; i < pipes.size(); ++i) {
        auto& pipe = pipes[i];
        if (scale.pixels_per_cm > 0) {
            pipe.length_real = pipe.length_pixels * scale.cm_per_pixel;
        }

        std::cout << "  Pipe #" << (i + 1) << ":\n";
        std::cout << "    Start:    (" << std::setprecision(0) << pipe.start.x
                  << ", " << pipe.start.y << ")\n";
        std::cout << "    End:      (" << pipe.end.x << ", " << pipe.end.y << ")\n";
        std::cout << "    Length:   " << std::setprecision(0) << pipe.length_pixels << " px";
        if (pipe.length_real > 0) {
            std::cout << " | " << std::setprecision(1) << pipe.length_real << " cm";
            std::cout << " | " << std::setprecision(1) << pipe.length_real / 2.54 << " in";
        }
        std::cout << "\n";
        std::cout << "    Angle:    " << std::setprecision(1) << pipe.angle << " deg\n";
    }

    print_separator("Results Summary");
    std::cout << "\n  PIPE LENGTHS:\n";
    for (size_t i = 0; i < pipes.size(); ++i) {
        const auto& pipe = pipes[i];
        std::cout << "    Pipe #" << (i + 1) << ": ";
        if (pipe.length_real > 0) {
            std::cout << std::fixed << std::setprecision(1) << pipe.length_real << " cm ("
                      << std::setprecision(1) << pipe.length_real / 2.54 << " in)\n";
        } else {
            std::cout << std::fixed << std::setprecision(0) << pipe.length_pixels << " px\n";
        }
    }
    std::cout << "\n  SCALE INFO:\n";
    std::cout << "    Ref squares used: " << scale.num_references << "\n";
    if (scale.pixels_per_cm > 0) {
        std::cout << "    Scale: " << std::setprecision(2) << scale.pixels_per_cm << " px/cm\n";
    }
    std::cout << "\n";

    if (config.save_debug) {
        fs::create_directories(config.output_dir);
        std::string prefix = config.output_dir + "/pipe_";

        cv::Mat ref_vis = ref_detector.visualize(image, ref_squares);
        cv::imwrite(prefix + "references.png", ref_vis);

        cv::Mat pipe_vis = pipe_detector.visualize(image, pipes, scale);
        cv::imwrite(prefix + "annotated.png", pipe_vis);

        cv::Mat edges_vis;
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::Canny(gray, edges_vis, 50, 150);
        cv::cvtColor(edges_vis, edges_vis, cv::COLOR_GRAY2BGR);
        cv::imwrite(prefix + "edges.png", edges_vis);

        std::cout << "  Debug images saved to: " << config.output_dir << "/\n";
    }

    return 0;
}

int main(int argc, char** argv) {
    print_banner();

    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    ProgramConfig config = parse_args(argc, argv);

    if (config.image_paths.empty()) {
        std::cerr << "Error: At least 1 image is required.\n";
        print_usage(argv[0]);
        return 1;
    }

    if (config.mode == "pipes") {
        return run_pipe_mode(config);
    }

    if (config.image_paths.size() < 2) {
        std::cerr << "Error: At least 2 images are required for stereo vision.\n";
        print_usage(argv[0]);
        return 1;
    }

    print_separator("Configuration");
    std::cout << "  Mode:           " << config.mode << "\n";
    std::cout << "  Images:         " << config.image_paths.size() << "\n";
    std::cout << "  Focal length:   " << config.focal_length_mm << " mm\n";
    std::cout << "  Sensor width:   " << config.sensor_width_mm << " mm\n";
    std::cout << "  Baseline:       " << config.baseline_meters << " m\n";
    std::cout << "  Plate width:    " << config.plate_width_meters << " m\n";
    std::cout << "  Plate height:   " << config.plate_height_meters << " m\n";
    if (!config.calibration_file.empty()) {
        std::cout << "  Calibration:    " << config.calibration_file << "\n";
    }
    std::cout << "  Save debug:     " << (config.save_debug ? "yes" : "no") << "\n\n";

    print_separator("Loading Images");

    std::vector<ImageData> images;
    for (const auto& path : config.image_paths) {
        ImageData data;
        data.path = path;

        if (!fs::exists(path)) {
            std::cerr << "  ERROR: File not found: " << path << "\n";
            return 1;
        }

        data.image = cv::imread(path, cv::IMREAD_COLOR);
        if (data.image.empty()) {
            std::cerr << "  ERROR: Failed to load image: " << path << "\n";
            return 1;
        }

        std::cout << "  Loaded: " << path << " (" << data.image.cols << "x"
                  << data.image.rows << ", " << data.image.channels() << " channels)\n";

        images.push_back(std::move(data));
    }

    CameraParameters cam_params;
    Calibration calibrator;

    if (!config.calibration_file.empty()) {
        auto calib_result = calibrator.load_calibration(config.calibration_file);
        if (calib_result) {
            cam_params.intrinsic_matrix = calib_result->camera_matrix;
            cam_params.distortion_coeffs = calib_result->dist_coeffs;
            cam_params.image_width = calib_result->image_width;
            cam_params.image_height = calib_result->image_height;
            cam_params.focal_length_mm = config.focal_length_mm;
            cam_params.sensor_width_mm = config.sensor_width_mm;

            std::cout << "\n  Loaded calibration from: " << config.calibration_file << "\n";

            for (auto& img : images) {
                CalibrationResult cr;
                cr.camera_matrix = cam_params.intrinsic_matrix;
                cr.dist_coeffs = cam_params.distortion_coeffs;
                img.image = calibrator.undistort(img.image, cr);
            }
            std::cout << "  Applied lens distortion correction to all images.\n";
        } else {
            std::cerr << "  Warning: Failed to load calibration file, using estimates\n";
            cam_params = CameraParameters::estimate_from_image(
                images[0].image.cols, images[0].image.rows,
                config.focal_length_mm, config.sensor_width_mm);
        }
    } else {
        cam_params = CameraParameters::estimate_from_image(
            images[0].image.cols, images[0].image.rows,
            config.focal_length_mm, config.sensor_width_mm);
        std::cout << "\n  Using estimated camera parameters:\n"
                  << "    Focal length: " << cam_params.focal_length_pixels() << " px\n"
                  << "    Principal point: ("
                  << cam_params.intrinsic_matrix.at<double>(0, 2) << ", "
                  << cam_params.intrinsic_matrix.at<double>(1, 2) << ")\n";
    }

    print_separator("Purple Plate Detection");

    PurplePlateDetector detector;
    detector.set_min_area(200);
    detector.set_max_area(images[0].image.cols * images[0].image.rows * 0.5);

    for (auto& img : images) {
        img.all_detections = detector.detect(img.image);
        std::cout << "  " << img.path << ": found " << img.all_detections.size() << " candidate(s)\n";

        for (size_t k = 0; k < img.all_detections.size(); ++k) {
            print_detection_info(img.all_detections[k], static_cast<int>(k + 1));
        }

        for (const auto& d : img.all_detections) {
            if (d.confidence >= 70.0) {
                img.high_conf_plates.push_back(d);
            }
        }

        std::cout << "    High-confidence plates (>=70%): " << img.high_conf_plates.size() << "\n";
    }

    auto& left = images[0];
    auto& right = images[1];

    if (left.high_conf_plates.empty() || right.high_conf_plates.empty()) {
        std::cerr << "\n  ERROR: Need at least one high-confidence plate in each image\n";
        if (left.high_conf_plates.empty()) std::cerr << "    -> No plates >=70% in LEFT image\n";
        if (right.high_conf_plates.empty()) std::cerr << "    -> No plates >=70% in RIGHT image\n";
        return 1;
    }

    print_separator("Plate Matching Between Images");

    auto plate_matches = match_plates_between_images(left.high_conf_plates, right.high_conf_plates);
    std::cout << "  Matched " << plate_matches.size() << " plate pair(s) between left and right images\n\n";

    if (plate_matches.empty()) {
        std::cerr << "  ERROR: Could not match any plates between images\n";
        return 1;
    }

    for (const auto& [li, ri] : plate_matches) {
        std::cout << "  Left plate #" << (li + 1) << " (conf="
                  << std::fixed << std::setprecision(0) << left.high_conf_plates[li].confidence
                  << "%) <-> Right plate #" << (ri + 1) << " (conf="
                  << right.high_conf_plates[ri].confidence << "%)\n";
    }

    print_separator("Feature Matching & Stereo Geometry");

    StereoMatcher matcher(StereoMatcher::DetectorType::AKAZE);
    matcher.set_camera_matrix(cam_params.intrinsic_matrix);
    matcher.set_max_features(8000);
    matcher.set_ratio_threshold(0.75f);

    auto global_matches = matcher.find_matches(left.image, right.image);
    std::cout << "  Global matches: " << global_matches.size() << "\n";

    std::optional<StereoResult> stereo_result;
    if (global_matches.size() >= 8) {
        stereo_result = matcher.compute_stereo(left.image, right.image, global_matches);
    }

    if (!stereo_result) {
        std::cerr << "  ERROR: Failed to recover stereo geometry from global matches\n";
        return 1;
    }

    cv::Mat rvec;
    cv::Rodrigues(stereo_result->rotation, rvec);
    double angle = cv::norm(rvec) * 180.0 / CV_PI;
    std::cout << "  Rotation angle: " << std::setprecision(2) << angle << " degrees\n";
    std::cout << "  Reprojection error: " << std::setprecision(4)
              << stereo_result->reprojection_error << " px\n";

    print_separator("Distance Estimation Per Plate");

    DistanceCalculator calc;
    calc.set_camera_params(cam_params);
    calc.set_known_plate_width(config.plate_width_meters);
    calc.set_known_plate_height(config.plate_height_meters);

    std::vector<PlateDistanceResult> results;

    std::map<int, int> left_hc_to_all;
    for (size_t hi = 0; hi < left.high_conf_plates.size(); ++hi) {
        for (size_t ai = 0; ai < left.all_detections.size(); ++ai) {
            if (std::abs(left.high_conf_plates[hi].centroid.x - left.all_detections[ai].centroid.x) < 1.0 &&
                std::abs(left.high_conf_plates[hi].centroid.y - left.all_detections[ai].centroid.y) < 1.0) {
                left_hc_to_all[static_cast<int>(hi)] = static_cast<int>(ai);
                break;
            }
        }
    }

    for (const auto& [li, ri] : plate_matches) {
        const auto& lp = left.high_conf_plates[li];
        const auto& rp = right.high_conf_plates[ri];

        int all_idx = left_hc_to_all.count(li) ? left_hc_to_all[li] : li;

        std::cout << "\n  --- Plate #" << (all_idx + 1) << " (left conf="
                  << std::setprecision(0) << lp.confidence << "%) ---\n";

        auto estimates = calc.estimate_all_methods(
            stereo_result->rotation, stereo_result->translation,
            lp.centroid, rp.centroid,
            lp.rotated_rect,
            config.baseline_meters);

        for (const auto& est : estimates) {
            print_distance_estimate(est);
        }

        double consensus = compute_consensus_distance(estimates);

        PlateDistanceResult pdr;
        pdr.plate_index = all_idx;
        pdr.left_plate = lp;
        pdr.right_plate = rp;
        pdr.distance_meters = consensus;
        pdr.distance_inches = consensus > 0 ? consensus * 39.3701 : -1.0;
        pdr.estimates = estimates;

        if (consensus > 0) {
            std::cout << "\n    >>> PLATE #" << (all_idx + 1) << " DISTANCE: "
                      << std::setprecision(4) << consensus << " m ("
                      << std::setprecision(1) << pdr.distance_inches << " in) <<<\n";
        } else {
            std::cout << "\n    >>> PLATE #" << (all_idx + 1) << " DISTANCE: COULD NOT DETERMINE <<<\n";
        }

        results.push_back(pdr);
    }

    print_separator("Results Summary");
    std::cout << "\n  PLATE DISTANCES:\n";
    for (const auto& r : results) {
        if (r.distance_inches > 0) {
            std::cout << "    Plate #" << (r.plate_index + 1) << ": "
                      << std::fixed << std::setprecision(1) << r.distance_inches << " in ("
                      << std::setprecision(4) << r.distance_meters << " m)\n";
        } else {
            std::cout << "    Plate #" << (r.plate_index + 1) << ": UNABLE TO DETERMINE\n";
        }
    }
    std::cout << "\n";

    if (config.save_debug) {
        fs::create_directories(config.output_dir);
        std::string prefix = config.output_dir + "/pair_1_";

        cv::Mat left_vis = detector.visualize(left.image, left.all_detections);
        cv::Mat right_vis = detector.visualize(right.image, right.all_detections);
        cv::imwrite(prefix + "left_detection.png", left_vis);
        cv::imwrite(prefix + "right_detection.png", right_vis);

        auto match_vis = matcher.visualize_matches(left.image, right.image,
                                                    stereo_result->inlier_matches);
        cv::imwrite(prefix + "matches.png", match_vis);

        if (!stereo_result->fundamental_matrix.empty()) {
            auto epipolar_vis = matcher.visualize_epipolar(
                left.image, right.image,
                stereo_result->fundamental_matrix,
                stereo_result->inlier_matches);
            cv::imwrite(prefix + "epipolar.png", epipolar_vis);
        }

        cv::Mat annotated = create_annotated_image(left.image, left.all_detections, results);
        cv::imwrite(prefix + "annotated.png", annotated);

        std::cout << "  Debug images saved to: " << config.output_dir << "/\n";
        std::cout << "  Annotated image: " << prefix << "annotated.png\n";
    }

    return 0;
}
