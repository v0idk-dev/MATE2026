#pragma once
// ─────────────────────────────────────────────────────────────────────────
// plate_fusion.hpp — merge LAB-segment + Vision-rectangle detections.
//
// Strategy:
//   • Compute pairwise IoU between every (lab, vision) detection.
//   • If IoU > 0.40 → it's the same plate; keep the one with the
//     SMALLER corner-localisation uncertainty (estimated from the
//     spread of cornerSubPix iterates; we approximate with confidence).
//   • Otherwise both survive.
//   • Re-id contiguously [0..N).
// ─────────────────────────────────────────────────────────────────────────

#include "plate_detector.hpp"
#include <vector>

namespace mate {

std::vector<PlateDetection>
fusePlates(const std::vector<PlateDetection>& lab,
           const std::vector<PlateDetection>& vision,
           double iou_threshold = 0.40);

}  // namespace mate
