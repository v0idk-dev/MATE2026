// manual_scale.cpp — step 9.
#include "manual_scale.hpp"
#include <cmath>

namespace mate {

void applyManualWidthOverride(Model3D& m, double real_width_m) {
    if (real_width_m <= 0 || !std::isfinite(real_width_m)) return;
    recomputeBounds(m);
    if (m.total_width <= 1e-6) return;
    const double k = real_width_m / m.total_width;
    applyScale(m, k, "manual",
               "user-supplied total width override (m)", 1.0);
    recomputeBounds(m);
}

void applyManualLengthOverride(Model3D& m, double real_length_m) {
    if (real_length_m <= 0 || !std::isfinite(real_length_m)) return;
    recomputeBounds(m);
    if (m.total_length <= 1e-6) return;
    const double k = real_length_m / m.total_length;
    applyScale(m, k, "manual",
               "user-supplied total length override (m)", 1.0);
    recomputeBounds(m);
}

}  // namespace mate
