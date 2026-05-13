#pragma once
// ─────────────────────────────────────────────────────────────────────────
// scene_io.hpp — strict-JSON output schema for the photogrammetry pipeline.
//
// The C++ binary writes ONE JSON document to stdout when --output=json. The
// Flask layer parses it and forwards it to the frontend. Schema fields are
// stable; new optional fields go at the end with sensible defaults so the
// frontend can be older than the binary without breaking.
//
// Every measurement that has a unit is reported as a number paired with
// `unit` ("cm" | "m"); the frontend converts as needed for display. We do
// NOT silently rescale inside C++ — measurements come out in whatever unit
// the calibration was done in, and the JSON declares it.
// ─────────────────────────────────────────────────────────────────────────

#include "calibration_io.hpp"
#include "plate_detector.hpp"
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace mate {

struct SceneRunInfo {
    std::string mode;             // "stereo", "video", "hybrid_ai", "ai_only"
    std::string version = "0.3";  // bump on schema changes
    long long  timing_ms = 0;     // total wall-clock for this run
    std::string warning;          // human-readable, surfaced in the UI banner
    std::string error;            // empty on success
};

struct SceneCalibrationInfo {
    bool   present = false;
    double rms_px = -1.0;
    double avg_epipolar_err_px = -1.0;
    double baseline = 0.0;
    std::string baseline_unit;    // "cm" | "m" | "mm" | etc.
    int    image_width = 0;
    int    image_height = 0;
    int    pairs_used = 0;
};

struct ScenePlate {
    int    id = -1;               // 0..N-1, stable across left/right
    cv::Point2f center_left;
    cv::Point2f center_right;
    std::array<cv::Point2f, 4> corners_left{};
    std::array<cv::Point2f, 4> corners_right{};
    double confidence = 0.0;
    bool   has_3d = false;
    cv::Point3f position_3d{0, 0, 0};   // in `unit` (see SceneOutput)
    // Triangulated plate corners. Populated when has_3d is true.
    std::array<cv::Point3f, 4> corners_3d{};
};

struct ScenePipe {
    int id = -1;
    int junction_a = -1;          // index into SceneOutput::junctions
    int junction_b = -1;
    cv::Point3f a{0, 0, 0};       // 3D endpoint, in `unit`
    cv::Point3f b{0, 0, 0};
    double length = 0.0;
};

struct SceneJunction {
    int id = -1;
    cv::Point3f position{0, 0, 0};
    int degree = 0;               // number of incident pipes
};

struct SceneScale {
    double k = 1.0;               // multiplicative scale applied to coords
    double confidence = 0.0;      // 0..1
    int observations_used = 0;
    std::string source;           // "calibration" | "plate-prior" | "manual"
    std::string reason;           // human-readable diagnosis
};

struct SceneOutput {
    SceneRunInfo run;
    SceneCalibrationInfo calibration;
    std::string unit;             // unit of all 3D coordinates and dimensions
    std::vector<ScenePlate>    plates;
    std::vector<ScenePipe>     pipes;
    std::vector<SceneJunction> junctions;
    SceneScale scale;
    // Scene-level dimensions, in `unit`. -1 = not computed.
    double length = -1.0;
    double height = -1.0;
    // Underwater correction state (informational).
    bool   underwater = false;
    double water_n     = 0.0;     // 0 if disabled
    // Dense MVS output (optional; populated when --dense was passed).
    std::vector<cv::Point3f> dense_cloud_xyz;
    std::vector<cv::Vec3b>   dense_cloud_rgb;
};

// Serialize to JSON string. `pretty=true` returns indented JSON for human
// inspection; the frontend always parses either form.
std::string sceneToJson(const SceneOutput& s, bool pretty = false);

}  // namespace mate
