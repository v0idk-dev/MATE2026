// multi_pair_fuse.cpp — step 7.
#include "multi_pair_fuse.hpp"
#include "accelerate_utils.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace mate {

namespace {

// Build a simple "fingerprint" from sorted pairwise plate-center distances.
// Used as a coarse rejection check before running the alignment.
std::vector<double> pairwiseDistances(const std::vector<Vec3>& pts) {
    std::vector<double> out;
    out.reserve(pts.size() * (pts.size()-1) / 2);
    for (size_t i = 0; i < pts.size(); ++i)
        for (size_t j = i+1; j < pts.size(); ++j) {
            const double dx = pts[i].x - pts[j].x;
            const double dy = pts[i].y - pts[j].y;
            const double dz = pts[i].z - pts[j].z;
            out.push_back(std::sqrt(dx*dx + dy*dy + dz*dz));
        }
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace

Model3D fuseModels(const std::vector<Model3D>& per_pair, const FuseConfig& cfg) {
    Model3D out;
    if (per_pair.empty()) return out;
    if (per_pair.size() == 1) { out = per_pair[0]; out.n_pairs_used = 1; return out; }

    // Reference: pair with the most plates (most informative).
    size_t ref = 0;
    for (size_t i = 1; i < per_pair.size(); ++i)
        if (per_pair[i].plates.size() > per_pair[ref].plates.size()) ref = i;

    const auto& R = per_pair[ref];
    const auto refPts = R.raw_plate_centers;

    // For each non-reference pair, register against ref using the first
    // min(N) plates (assumed in matching order — best effort without
    // explicit IDs).
    std::vector<accel::YawXY> transforms(per_pair.size());
    std::vector<bool> keep(per_pair.size(), true);
    for (size_t i = 0; i < per_pair.size(); ++i) {
        if (i == ref) continue;
        const auto& src = per_pair[i].raw_plate_centers;
        const size_t N = std::min(src.size(), refPts.size());
        if (N < 2) { keep[i] = !cfg.drop_outlier_pairs; continue; }
        std::vector<Vec3> S(src.begin(), src.begin()+N);
        std::vector<Vec3> D(refPts.begin(), refPts.begin()+N);
        transforms[i] = accel::rigidAlignXY(S, D);
        if (transforms[i].rms > cfg.max_residual_m && cfg.drop_outlier_pairs)
            keep[i] = false;
    }
    keep[ref] = true;

    // Collect aligned section sizes per "section index" (assumes same
    // order across pairs; per-pair PCA produces sections sorted by their
    // x-coordinate so this is reasonable).
    const size_t S = R.sections.size();
    std::vector<std::vector<double>> Lk(S), Wk(S), Hk(S);
    for (size_t k = 0; k < S; ++k) {
        Lk[k].push_back(R.sections[k].size.x);
        Wk[k].push_back(R.sections[k].size.y);
        Hk[k].push_back(R.sections[k].size.z);
    }
    for (size_t i = 0; i < per_pair.size(); ++i) {
        if (!keep[i] || i == ref) continue;
        for (size_t k = 0; k < std::min(S, per_pair[i].sections.size()); ++k) {
            Lk[k].push_back(per_pair[i].sections[k].size.x);
            Wk[k].push_back(per_pair[i].sections[k].size.y);
            Hk[k].push_back(per_pair[i].sections[k].size.z);
        }
    }

    out = R;
    int used = 1;
    for (auto k : keep) if (k) ++used;
    out.n_pairs_used = used - (ref >= per_pair.size() ? 0 : 0);  // count of `keep` true

    // Take median of each dimension across pairs.
    for (size_t k = 0; k < S; ++k) {
        out.sections[k].size.x = accel::median(Lk[k]);
        out.sections[k].size.y = accel::median(Wk[k]);
        out.sections[k].size.z = accel::median(Hk[k]);
        out.sections[k].confidence = std::min(1.0, (double)Lk[k].size() /
                                              (double)per_pair.size());
    }

    // Average plate (u, v) per (section_id, plate_index_within_section).
    // We bucket by the plate's index in each per-pair model (best effort).
    std::map<int, std::vector<std::pair<double,double>>> plate_uv;
    std::map<int, int> plate_section;
    std::map<int, std::string> plate_face;
    for (size_t i = 0; i < per_pair.size(); ++i) {
        if (!keep[i]) continue;
        for (size_t pi = 0; pi < per_pair[i].plates.size(); ++pi) {
            const int key = (int)pi;
            const auto& p = per_pair[i].plates[pi];
            plate_uv[key].emplace_back(p.u, p.v);
            plate_section[key] = p.section_id;
            plate_face[key] = p.face;
        }
    }
    out.plates.clear();
    for (auto& [k, uvs] : plate_uv) {
        double su = 0, sv = 0;
        for (auto [u, v] : uvs) { su += u; sv += v; }
        Plate p;
        p.id = k;
        p.section_id = plate_section[k];
        p.face = plate_face[k];
        p.u = su / uvs.size();
        p.v = sv / uvs.size();
        p.side_m = 0.10;
        p.confidence = std::min(1.0, uvs.size() / (double)per_pair.size());
        out.plates.push_back(p);
    }

    resolvePlateCorners(out);
    recomputeBounds(out);
    return out;
}

}  // namespace mate
