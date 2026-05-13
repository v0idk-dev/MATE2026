#pragma once
// ─────────────────────────────────────────────────────────────────────────
// underwater.hpp — refraction correction for in-water imaging.
//
// Refraction at the air–water interface (camera housing's flat port → water)
// makes objects appear closer than they really are by approximately the
// refractive index of water (n ≈ 1.333). The standard first-order correction
// scales metric depth by 1/n. This is the simplest and most common model;
// rigorous treatments add a second-order correction for off-axis rays, but
// for our subject distances and FOVs the first-order term dominates by an
// order of magnitude.
//
// Apply this AFTER triangulation, BEFORE wireframe extraction.
//
// Note: if the calibration was done IN water (rare — usually done in air
// because chessboard handling is easier dry), the underwater toggle should
// be off because the calibration's intrinsics already absorb the refraction.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <vector>

namespace mate {

// Refractive index of fresh water at room temperature, sea water in
// MATE's flume tanks is ~1.339; we use the common 1.333 unless overridden.
constexpr double kWaterRefractiveIndex = 1.333;

struct UnderwaterConfig {
    bool enabled = false;
    double n_water = kWaterRefractiveIndex;
};

// Apply 1/n_water scale to every coordinate of every 3D point in-place.
void applyRefractionCorrection(std::vector<cv::Point3f>& pts,
                               const UnderwaterConfig& cfg);

}  // namespace mate
