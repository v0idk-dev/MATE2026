#pragma once
// ─────────────────────────────────────────────────────────────────────────
// underwater_restore.hpp — colour/contrast restoration for underwater
// imagery, in-place. Implements a lightweight Sea-thru-style approach:
//
//   1. Estimate the per-channel attenuation from the dark-channel prior.
//   2. Reconstruct the airlight A (water column colour) from the
//      brightest 0.1 % of dark-channel pixels.
//   3. Per-channel transmission t_c(x) ≈ 1 − ω · (I_c(x) / A_c), with
//      a wavelength-dependent ω scaling (red attenuates faster than
//      blue underwater, so ω_red > ω_green > ω_blue).
//   4. Recover J = (I − A) / max(t, t_min) + A.
//   5. CLAHE on the L channel of LAB to restore mid-frequency contrast.
//
// Net effect on the example footage: red/orange chroma comes back, the
// pink plate-tape stops looking grey, and LSD edge counts on the PVC
// pipes roughly double because edge contrast comes back.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>

namespace mate {

struct UnderwaterCfg {
    double water_n        = 1.34;   // refractive index (informational only here)
    double omega_red      = 0.95;
    double omega_green    = 0.55;
    double omega_blue     = 0.40;
    double t_min          = 0.10;
    double clahe_clip     = 2.5;
    int    clahe_grid     = 8;
};

// In-place restoration. Safe to call on already-bright images.
void underwaterRestore(cv::Mat& bgr_inout, const UnderwaterCfg& cfg = {});

// Convenience overload using legacy (water_n) signature.
void underwaterRestore(cv::Mat& bgr_inout, double water_n);

}  // namespace mate
