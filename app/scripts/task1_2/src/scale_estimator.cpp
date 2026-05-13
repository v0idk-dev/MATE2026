#include "scale_estimator.hpp"
#include "calibration_io.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace mate {

namespace {

double dist3(const cv::Point3f& a, const cv::Point3f& b) {
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double median(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace

ScaleResult
estimateScaleFromPlates(const std::vector<PlateScaleObservation>& obs,
                        double known_plate_side_m,
                        const std::string& unit_in) {
    ScaleResult r;
    if (obs.empty() || known_plate_side_m <= 0) {
        r.reason = "no plates available for scale estimation";
        return r;
    }
    const double u = unitToMeters(unit_in);  // unit_in → meters
    if (u <= 0) {
        r.reason = "unrecognized unit '" + unit_in + "'";
        return r;
    }

    // For each plate: 4 edge lengths. We use the mean of the four edges
    // (since plates are square in reality) per plate, then take the
    // median across plates as the robust estimate.
    std::vector<double> per_plate_edges_m;
    per_plate_edges_m.reserve(obs.size());
    for (const auto& o : obs) {
        // Skip plates whose any corner is non-finite (failed triangulation).
        bool ok = true;
        for (const auto& c : o.corners3d) {
            if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z)) {
                ok = false; break;
            }
        }
        if (!ok) continue;
        // Edges are corners[0..1], [1..2], [2..3], [3..0] in canonical order.
        double e0 = dist3(o.corners3d[0], o.corners3d[1]);
        double e1 = dist3(o.corners3d[1], o.corners3d[2]);
        double e2 = dist3(o.corners3d[2], o.corners3d[3]);
        double e3 = dist3(o.corners3d[3], o.corners3d[0]);
        double mean_unit = 0.25 * (e0 + e1 + e2 + e3);
        if (mean_unit > 1e-6) per_plate_edges_m.push_back(mean_unit * u);
    }
    if (per_plate_edges_m.empty()) {
        r.reason = "no plates yielded usable edge lengths";
        return r;
    }

    std::vector<double> sorted = per_plate_edges_m;
    double measured_m = median(sorted);
    if (measured_m <= 1e-6) {
        r.reason = "median plate edge is zero — bad triangulation?";
        return r;
    }
    r.k = known_plate_side_m / measured_m;
    r.observations_used = (int)per_plate_edges_m.size();

    // Confidence: how tightly do per-plate estimates agree? CV<5% is good.
    double mean = std::accumulate(per_plate_edges_m.begin(),
                                  per_plate_edges_m.end(), 0.0)
                  / per_plate_edges_m.size();
    double var = 0.0;
    for (double v : per_plate_edges_m) {
        double d = v - mean; var += d * d;
    }
    double sd = std::sqrt(var / std::max<size_t>(1, per_plate_edges_m.size()));
    double cv = (mean > 1e-9) ? (sd / mean) : 1.0;
    r.confidence = std::max(0.0, 1.0 - cv * 10.0);  // 0% CV → 1.0; 10% CV → 0
    r.confidence = std::min(0.5, r.confidence);     // cap at 0.5 weight

    std::ostringstream os;
    os << "plate-prior: " << per_plate_edges_m.size() << " plates, "
       << "median edge = " << measured_m << " m (expected "
       << known_plate_side_m << " m); CV=" << (cv * 100.0) << "%";
    r.reason = os.str();
    return r;
}

ScaleResult
applyManualMeasurement(const ScaleResult& current,
                       const ManualMeasurement& m,
                       const std::string& unit_in) {
    ScaleResult r = current;
    if (!std::isfinite(m.a.x) || !std::isfinite(m.b.x)) {
        r.reason += " | manual: invalid endpoints";
        return r;
    }
    double u_in   = unitToMeters(unit_in);
    double u_real = unitToMeters(m.unit.empty() ? "m" : m.unit);
    if (u_in <= 0 || u_real <= 0) {
        r.reason += " | manual: unrecognized unit";
        return r;
    }
    double measured_m = dist3(m.a, m.b) * u_in * current.k;
    double real_m     = m.real_world_value * u_real;
    if (measured_m <= 1e-6 || real_m <= 1e-6) {
        r.reason += " | manual: degenerate measurement";
        return r;
    }
    double k_manual = real_m / (dist3(m.a, m.b) * u_in);
    double drift = std::abs(real_m - measured_m) / real_m;

    r.k = k_manual;
    r.confidence = 1.0;
    r.observations_used += 1;
    std::ostringstream os;
    os << " | manual override applied; drift from prior estimate = "
       << (drift * 100.0) << "%";
    r.reason += os.str();
    return r;
}

}  // namespace mate
