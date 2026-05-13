#include "wireframe_builder.hpp"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace mate {

namespace {

double dist3(const cv::Point3f& a, const cv::Point3f& b) {
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Add an endpoint to the cluster list, fusing into the nearest existing
// cluster if within merge_dist, otherwise creating a new one. Returns the
// cluster index assigned. The cluster's centroid is updated as a running
// mean.
int assignToCluster(std::vector<cv::Point3f>& cluster_centers,
                    std::vector<int>& cluster_counts,
                    const cv::Point3f& p,
                    double merge_dist) {
    int best = -1;
    double best_d = 1e9;
    for (size_t i = 0; i < cluster_centers.size(); ++i) {
        double d = dist3(cluster_centers[i], p);
        if (d < best_d) { best_d = d; best = (int)i; }
    }
    if (best >= 0 && best_d <= merge_dist) {
        // Update running mean.
        cluster_counts[best] += 1;
        double w = 1.0 / cluster_counts[best];
        cv::Point3f& c = cluster_centers[best];
        c.x = (float)((1.0 - w) * c.x + w * p.x);
        c.y = (float)((1.0 - w) * c.y + w * p.y);
        c.z = (float)((1.0 - w) * c.z + w * p.z);
        return best;
    }
    cluster_centers.push_back(p);
    cluster_counts.push_back(1);
    return (int)cluster_centers.size() - 1;
}

// PCA on a set of 3D points → 3 axes sorted by descending eigenvalue.
// We use cv::PCA which expects rows = samples, cols = dims.
void pcaAxes(const std::vector<cv::Point3f>& pts,
             cv::Point3f& a0, cv::Point3f& a1, cv::Point3f& a2) {
    if (pts.size() < 3) {
        a0 = cv::Point3f(1, 0, 0);
        a1 = cv::Point3f(0, 1, 0);
        a2 = cv::Point3f(0, 0, 1);
        return;
    }
    cv::Mat data((int)pts.size(), 3, CV_64F);
    for (size_t i = 0; i < pts.size(); ++i) {
        data.at<double>((int)i, 0) = pts[i].x;
        data.at<double>((int)i, 1) = pts[i].y;
        data.at<double>((int)i, 2) = pts[i].z;
    }
    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
    auto eigToPt = [&](int row) {
        return cv::Point3f(
            (float)pca.eigenvectors.at<double>(row, 0),
            (float)pca.eigenvectors.at<double>(row, 1),
            (float)pca.eigenvectors.at<double>(row, 2));
    };
    a0 = eigToPt(0); a1 = eigToPt(1); a2 = eigToPt(2);
}

double dot3(const cv::Point3f& a, const cv::Point3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Normalize in-place; returns false if the vector is degenerate.
bool normalize3(cv::Point3f& v) {
    double L = std::sqrt(dot3(v, v));
    if (L < 1e-9) return false;
    v.x = (float)(v.x / L); v.y = (float)(v.y / L); v.z = (float)(v.z / L);
    return true;
}

// Snap a unit-length pipe direction `d` to the closest of the three PCA
// axes (or its negative). Returns the snapped unit vector if the angular
// deviation is below max_dev_deg, else returns d unchanged.
cv::Point3f snapToAxis(const cv::Point3f& d_in,
                       const cv::Point3f& a0,
                       const cv::Point3f& a1,
                       const cv::Point3f& a2,
                       double max_dev_deg) {
    cv::Point3f d = d_in;
    if (!normalize3(d)) return d_in;
    cv::Point3f axes[3] = { a0, a1, a2 };
    double best_cos = -2.0;
    int best_axis = -1;
    int best_sign = 1;
    for (int i = 0; i < 3; ++i) {
        cv::Point3f ax = axes[i];
        if (!normalize3(ax)) continue;
        double c = dot3(d, ax);
        // Lines are undirected — consider ±axis.
        if (std::abs(c) > best_cos) {
            best_cos = std::abs(c);
            best_axis = i;
            best_sign = (c >= 0) ? 1 : -1;
        }
    }
    if (best_axis < 0) return d_in;
    double dev_deg = std::acos(std::min(1.0, std::max(-1.0, best_cos)))
                     * 180.0 / CV_PI;
    if (dev_deg > max_dev_deg) return d_in;  // truly diagonal — leave alone
    cv::Point3f snapped = axes[best_axis];
    if (best_sign < 0) { snapped.x = -snapped.x; snapped.y = -snapped.y; snapped.z = -snapped.z; }
    normalize3(snapped);
    return snapped;
}

}  // namespace

Wireframe
buildWireframe(std::vector<PipeSegment3D>& pipes,
               const WireframeBuilderConfig& cfg) {
    Wireframe wf;
    if (pipes.empty()) return wf;

    // 1. Cluster endpoints into junctions.
    std::vector<cv::Point3f> centers;
    std::vector<int> counts;
    for (auto& p : pipes) {
        p.junction_a = assignToCluster(centers, counts, p.a,
                                       cfg.junction_merge_distance);
        p.junction_b = assignToCluster(centers, counts, p.b,
                                       cfg.junction_merge_distance);
    }
    wf.junctions.reserve(centers.size());
    for (size_t i = 0; i < centers.size(); ++i) {
        WireframeJunction j;
        j.position = centers[i];
        j.degree = 0;
        wf.junctions.push_back(j);
    }
    for (auto& p : pipes) {
        if (p.junction_a >= 0) wf.junctions[p.junction_a].degree += 1;
        if (p.junction_b >= 0) wf.junctions[p.junction_b].degree += 1;
    }

    // 2. (Optional) snap pipe directions to PCA axes. We don't move the
    // junctions themselves — only re-derive the line through (junction_a,
    // junction_b) so the visualization is clean. The junctions stay where
    // the data placed them.
    if (cfg.snap_to_axes && wf.junctions.size() >= 3) {
        std::vector<cv::Point3f> jpts;
        for (const auto& j : wf.junctions) jpts.push_back(j.position);
        cv::Point3f a0, a1, a2;
        pcaAxes(jpts, a0, a1, a2);
        wf.principal_axis    = a0;
        wf.up_axis           = a2;  // the smallest PCA axis is the "up"
                                    // direction for a flat-on-the-floor
                                    // structure
        wf.base_plane_normal = a2;

        for (auto& p : pipes) {
            if (p.junction_a < 0 || p.junction_b < 0) continue;
            cv::Point3f a = wf.junctions[p.junction_a].position;
            cv::Point3f b = wf.junctions[p.junction_b].position;
            cv::Point3f d = cv::Point3f(b.x - a.x, b.y - a.y, b.z - a.z);
            cv::Point3f d_snapped = snapToAxis(d, a0, a1, a2,
                                               cfg.snap_max_deviation_deg);
            // Project the segment length onto the snapped direction; keep
            // the original midpoint. (Rotates the segment about its center
            // to align with the snapped axis without moving its midpoint.)
            cv::Point3f mid((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f, (a.z + b.z) * 0.5f);
            double L = std::sqrt(dot3(d, d));
            cv::Point3f new_a(mid.x - (float)(0.5 * L * d_snapped.x),
                              mid.y - (float)(0.5 * L * d_snapped.y),
                              mid.z - (float)(0.5 * L * d_snapped.z));
            cv::Point3f new_b(mid.x + (float)(0.5 * L * d_snapped.x),
                              mid.y + (float)(0.5 * L * d_snapped.y),
                              mid.z + (float)(0.5 * L * d_snapped.z));
            p.a = new_a; p.b = new_b;
        }
    }

    // 3. Per-pipe length.
    for (auto& p : pipes) p.length = dist3(p.a, p.b);
    wf.pipes = pipes;

    // 4. Length = max projection of any junction-pair onto principal axis.
    if (wf.junctions.size() >= 2) {
        cv::Point3f axis = wf.principal_axis;
        normalize3(axis);
        double pmin = 1e9, pmax = -1e9;
        for (const auto& j : wf.junctions) {
            double t = dot3(j.position, axis);
            pmin = std::min(pmin, t);
            pmax = std::max(pmax, t);
        }
        wf.length = pmax - pmin;
    }

    // 5. Base plane = best-fit plane through the bottom 30% of junctions
    // (sorted by projection onto up_axis). Height = max perpendicular
    // distance above this plane.
    if (wf.junctions.size() >= 3) {
        cv::Point3f up = wf.up_axis;
        normalize3(up);
        std::vector<double> ups;
        ups.reserve(wf.junctions.size());
        for (const auto& j : wf.junctions) ups.push_back(dot3(j.position, up));
        std::vector<double> sorted = ups;
        std::sort(sorted.begin(), sorted.end());
        size_t k = std::max<size_t>(2, sorted.size() * 30 / 100);
        double up_floor = sorted[k - 1];
        double up_ceil  = sorted.back();
        wf.height = std::max(0.0, up_ceil - up_floor);
        wf.base_plane_normal = up;
        // Offset of the plane: average up-projection of bottom-k junctions.
        double sum = 0; size_t cnt = 0;
        for (double v : ups) {
            if (v <= up_floor + 1e-6) { sum += v; ++cnt; }
        }
        wf.base_plane_offset = (cnt > 0) ? sum / cnt : 0.0;
    }

    return wf;
}

}  // namespace mate
