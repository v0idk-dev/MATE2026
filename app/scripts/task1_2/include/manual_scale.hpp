#pragma once
// ─────────────────────────────────────────────────────────────────────────
// manual_scale.hpp — step 9: rescale entire Model3D so its width matches
// a user-supplied real-world value.
//
// The user knows the real total width of the structure (~36 cm) more
// reliably than we can measure it from a noisy stereo pair. They type it
// in; we set k = real_width_m / measured_width_m and apply uniformly.
// ─────────────────────────────────────────────────────────────────────────

#include "model3d.hpp"

namespace mate {

// Rescale model so total_width matches `real_width_m`. No-op if the
// argument is non-positive or the current width is degenerate.
void applyManualWidthOverride(Model3D& m, double real_width_m);

// Rescale model so total_length matches `real_length_m` instead.
void applyManualLengthOverride(Model3D& m, double real_length_m);

}  // namespace mate
