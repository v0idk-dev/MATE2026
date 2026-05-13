#include "underwater.hpp"
#include <cmath>

namespace mate {

void applyRefractionCorrection(std::vector<cv::Point3f>& pts,
                               const UnderwaterConfig& cfg) {
    if (!cfg.enabled) return;
    if (cfg.n_water <= 1.0) return;  // n must be >1 for water
    const float k = (float)(1.0 / cfg.n_water);
    for (auto& p : pts) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        p.x *= k; p.y *= k; p.z *= k;
    }
}

}  // namespace mate
