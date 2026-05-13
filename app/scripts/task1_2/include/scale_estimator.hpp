#pragma once
// ─────────────────────────────────────────────────────────────────────────
// scale_estimator.hpp — confidence-weighted scale-factor recovery.
//
// After triangulation we have a 3D scene in the *unit of the calibration*
// (typically cm because stereo_calibrate.py uses cm). That unit is already
// metric, so for the rigid-stereo path we don't need an additional scale.
// HOWEVER:
//
//   • If the user passes manual measurements ("the visible part of this
//     pipe is 1.20 m"), we treat that as ground truth and rescale the
//     entire wireframe so the named segment matches.
//
//   • If multiple plates are detected, each gives an independent estimate
//     of cm-per-pixel-equivalent: we know the plates are 10×10 cm and we
//     measured their corners in 3D. Disagreement among per-plate estimates
//     is a confidence indicator (and surfaces in the UI banner).
//
//   • Underwater shifts effective focal length by ~1.333×; the underwater
//     module reports the multiplier and this fuser combines it.
//
// The fusion rule (priority order from highest to lowest weight):
//   1. Manual measurement      (weight 1.0 — overrides everything)
//   2. In-scene ruler          (weight 0.8 — future, step 7+)
//   3. Plate-size prior        (weight 0.5)
//   4. Calibration baseline    (weight 0.3 — already implicit; we just
//                                check that plate-prior agrees with it)
//
// Output is a single multiplicative scale `k` plus a confidence in [0,1]
// describing how strongly the inputs agreed.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>

namespace mate {

struct PlateScaleObservation {
    // The four 3D corners of one plate, in calibration units (typically cm).
    std::array<cv::Point3f, 4> corners3d;
};

struct ManualMeasurement {
    cv::Point3f a;
    cv::Point3f b;
    double real_world_value;     // user-typed real measurement
    std::string unit;            // "m" | "cm" | "mm" | "in"
};

struct ScaleResult {
    double k = 1.0;              // multiplicative scale factor
    double confidence = 0.0;     // 0..1
    std::string reason;          // human-readable diagnosis
    int observations_used = 0;
};

// Compute the scale that aligns the median plate edge length with the known
// real-world plate edge (e.g. 0.10 m). known_plate_side is in METERS; the
// observations are in `unit_in`. Returns k such that
//   3D_in_meters = 3D_in_unit_in * unitToMeters(unit_in) * k
// In other words k=1.0 is "calibration is already correct"; k≠1 indicates
// systematic mis-scale.
ScaleResult
estimateScaleFromPlates(const std::vector<PlateScaleObservation>& obs,
                        double known_plate_side_m,
                        const std::string& unit_in);

// Fold a manual measurement into a (possibly already-fused) scale.
// `current.k` is the current best estimate; this returns a new ScaleResult
// whose k is the manual measurement's k (manual wins). For diagnostic
// output we also report how far the manual disagrees with `current`.
ScaleResult
applyManualMeasurement(const ScaleResult& current,
                       const ManualMeasurement& m,
                       const std::string& unit_in);

}  // namespace mate
