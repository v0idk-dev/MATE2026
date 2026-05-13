#pragma once
// ─────────────────────────────────────────────────────────────────────────
// legacy_shims.hpp — free-function shims so the OLD orchestrator
// (`src/pipeline.cpp`) can be compiled and used alongside the new
// `runPipePipeline()` path without us having to delete or rewrite it.
//
// The new path (pipe_pipeline.cpp) is the production code; this shim
// layer just exposes the simpler free-function spellings that the
// pre-overhaul `pipeline.cpp` was written against. None of these are
// performance-critical — they all delegate to the existing class APIs.
// ─────────────────────────────────────────────────────────────────────────

#include "pipe_detector.hpp"
#include "plate_detector.hpp"
#include "lab_segment.hpp"
#include "underwater_restore.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace mate {

// One-call wrapper around `PipeDetector::detect()`. Uses the default
// detector configuration; legacy `pipeline.cpp` never tuned the params.
inline std::vector<PipeSegment2D> detectPipes(const cv::Mat& bgr) {
    return PipeDetector{}.detect(bgr);
}

// One-call wrapper around `PlateDetector::detect()` that takes a config
// rather than constructing a detector at the call site.
inline std::vector<PlateDetection>
detectPlates(const cv::Mat& bgr, const PlateDetectorConfig& cfg = {}) {
    PlateDetector d(cfg);
    return d.detect(bgr, cfg.expected_plate_count);
}

// In-place underwater colour/contrast restoration on a single image.
// The geometry-side refraction correction (n_water on metric depths)
// happens later in the pipeline, on 3D points, via
// `applyRefractionCorrection()` from underwater.hpp. This shim covers
// only the per-image colour restore that legacy `pipeline.cpp` calls
// after undistort.
inline void applyUnderwaterCorrection(cv::Mat& bgr, double n_water) {
    if (bgr.empty()) return;
    underwaterRestore(bgr, n_water);   // in-place
}

}  // namespace mate
