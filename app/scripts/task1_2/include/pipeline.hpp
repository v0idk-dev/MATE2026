#pragma once
// ─────────────────────────────────────────────────────────────────────────
// pipeline.hpp — orchestrator for the full 9-step photogrammetry pipeline.
//
// User's spec, executed in order:
//   1. Undistort each image of every pair using camera intrinsics.
//   2. Detect PVC pipe segments in each undistorted image.
//   3. Detect colored plates in each undistorted image.
//   4. Build a per-image rough 3-section model from #2 + #3.
//   5. Stereo-align each pair (rectify) using stereo extrinsics.
//   6. Stereo geometry: compute width/height per section via disparity.
//   7. Fuse N per-pair Model3D's into a single fused Model3D.
//   8. (Optional) AI-enhance the fused model via the :5002 proxy.
//   9. (Optional) User-supplied total-width override → uniform rescale.
//
// Everything runs through one entry point: runPipeline(). The caller
// supplies N pair specs; outputs are a fused Model3D plus per-pair debug
// artifacts.
// ─────────────────────────────────────────────────────────────────────────

#include "calibration_io.hpp"
#include "model3d.hpp"
#include <opencv2/core.hpp>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace mate {

struct PairInput {
    std::string left_path;
    std::string right_path;
    // Optional per-pair calibration override; if empty, the global
    // PipelineInput::left_intrinsics / right_intrinsics / stereo_extrinsics
    // is used for every pair.
    std::string left_calib_yaml_override;
    std::string right_calib_yaml_override;
    std::string stereo_extrinsics_yaml_override;
};

struct PipelineInput {
    std::vector<PairInput> pairs;

    // Calibration paths used when the per-pair override is empty.
    std::string left_calib_yaml;
    std::string right_calib_yaml;
    std::string stereo_extrinsics_yaml;

    // Plate detection
    int    plate_target_h = 135;
    int    plate_hue_tolerance = 25;
    int    expected_plates = 8;
    double plate_side_m = 0.10;

    // Underwater
    bool   underwater = false;
    double water_n = 1.333;

    // Optional AI enhancement (step 8). When `ai_enhance=true` the
    // pipeline writes a temp JSON of the fused Model3D and invokes
    // python/ai_caller.py with ai_provider/ai_model. The model returns
    // refinement deltas which are merged in.
    bool        ai_enhance = false;
    std::string ai_provider;          // "openai" | "anthropic" | "google" | "apple"
    std::string ai_model;             // provider-specific model id
    std::string ai_caller_executable; // absolute path to python binary
    std::string ai_caller_script;     // absolute path to ai_caller.py
    // Use Apple's on-device FoundationModels (Apple Intelligence) instead of
    // calling out to a remote provider. Implies ai_enhance=true.
    bool use_apple_intelligence = false;

    // Optional manual scale override (step 9). `manual_total_width_m`
    // rescales the entire model so its width matches the typed value.
    // 0 = disabled.
    double manual_total_width_m = 0.0;

    // Manhattan-world auto-calibration fallback. Triggered automatically
    // when no left/right calibration YAMLs are supplied. Recovers focal
    // length from vanishing points in the first pair's left image and
    // assumes a parallel-axis stereo rig with the user-supplied
    // baseline magnitude (default 0.10 m). Set to 0 to disable the
    // fallback entirely (pipeline then runs in pixel-units mode as
    // before). See manhattan_calib.hpp for details.
    double rig_baseline_m = 0.10;

    // Acceleration toggles (Apple Silicon). When false, falls back to CPU
    // OpenCV. Default ON because the binary is Apple-Silicon-only per spec.
    bool use_metal      = true;   // Metal compute kernels (image ops)
    bool use_accelerate = true;   // Accelerate framework (BLAS, LAPACK, vDSP)
    bool use_vision     = true;   // Apple Vision framework (rectangle/contour)

    // Where to write per-pair debug images. Empty = no debug output.
    std::string debug_dir;

    // When true, each rectified pair is routed through runPipePipeline()
    // (the pipe-first 14-stage stack) instead of the plate-first path.
    // The resulting PipeGraphResult is converted into a minimal Model3D
    // so the rest of the output machinery (JSON / GLB / OBJ) is unchanged.
    bool engine_pipe = false;
};

struct PerPairDebug {
    int pair_index = -1;
    cv::Mat undistorted_left, undistorted_right;
    cv::Mat rectified_left,   rectified_right;
    cv::Mat plates_overlay_left, plates_overlay_right;
    cv::Mat pipes_overlay_left,  pipes_overlay_right;
    int n_plates_left  = 0, n_plates_right = 0;
    int n_pipes_left   = 0, n_pipes_right  = 0;
    Model3D rough_model;          // step 4
    Model3D stereo_model;         // step 6 (sized via stereo geometry)
    std::string error;
};

struct PipelineOutput {
    Model3D model;                // final fused (and possibly AI-enhanced) model
    std::vector<PerPairDebug> per_pair;
    std::string warning;
    std::string error;
    long long timing_ms_total = 0;
    long long timing_ms_per_step[10] = {};   // ms spent in each step (1..9)
};

// One-shot.  Thread-safe (every pair processes in isolation; the fusion at
// step 7 serializes).
PipelineOutput runPipeline(const PipelineInput& in);

// Optional progress callback: called as steps complete with (step_num,
// step_label, fraction_complete). step_num ∈ [1..9].
using ProgressFn = std::function<void(int, const std::string&, double)>;
PipelineOutput runPipeline(const PipelineInput& in, ProgressFn progress);

}  // namespace mate
