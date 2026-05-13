#include "match_ransac.hpp"
#include <random>
#include <algorithm>
#include <cmath>

namespace mate {

static double plateSize(const PlateDetection& p) {
    double s = 0;
    for (int i = 0; i < 4; ++i) {
        const auto& a = p.corners[i];
        const auto& b = p.corners[(i + 1) % 4];
        s += std::hypot(a.x - b.x, a.y - b.y);
    }
    return s * 0.25;
}

std::vector<std::pair<int,int>>
ransacMatchPlates(const std::vector<PlateDetection>& L,
                  const std::vector<PlateDetection>& R,
                  const MatchRansacConfig& cfg) {
    // 1) build candidate match set
    struct M { int li, ri; double dy, dx; };
    std::vector<M> cands;
    for (int i = 0; i < (int)L.size(); ++i) {
        double sl = plateSize(L[i]);
        for (int j = 0; j < (int)R.size(); ++j) {
            double dy = std::fabs(L[i].center.y - R[j].center.y);
            if (dy > cfg.epi_tol_px) continue;
            double sr = plateSize(R[j]);
            if (sr <= 0 || sl <= 0) continue;
            double ratio = std::max(sl, sr) / std::min(sl, sr);
            if (ratio > cfg.sz_tol) continue;
            double dx = L[i].center.x - R[j].center.x;          // disparity
            if (dx <= 0) continue;                              // L should be right of R after rect
            cands.push_back({i, j, dy, dx});
        }
    }
    if (cands.size() < 3) {
        // not enough to RANSAC; fall back to greedy nearest-y assignment
        std::vector<std::pair<int,int>> out;
        std::vector<bool> usedL(L.size(), false), usedR(R.size(), false);
        std::sort(cands.begin(), cands.end(), [](const M& a, const M& b){ return a.dy < b.dy; });
        for (auto& m : cands) {
            if (usedL[m.li] || usedR[m.ri]) continue;
            usedL[m.li] = usedR[m.ri] = true;
            out.emplace_back(m.li, m.ri);
        }
        return out;
    }

    // 2) RANSAC: model = piecewise-constant disparity per section (K means)
    std::mt19937 rng(0xC0FFEE);
    std::uniform_int_distribution<int> pick(0, (int)cands.size() - 1);

    std::vector<int> best_inliers;
    std::vector<double> best_centers;

    for (int it = 0; it < cfg.iters; ++it) {
        // Sample K=cfg.n_sections distinct candidates → their disparities
        std::vector<double> centers;
        std::vector<int> seen;
        for (int k = 0; k < cfg.n_sections; ++k) {
            int idx = pick(rng);
            if (std::find(seen.begin(), seen.end(), idx) != seen.end()) { --k; continue; }
            seen.push_back(idx);
            centers.push_back(cands[idx].dx);
        }
        std::sort(centers.begin(), centers.end());

        // Count inliers: assign each candidate to nearest centre, check tol.
        std::vector<int> inliers;
        for (int c = 0; c < (int)cands.size(); ++c) {
            double best = 1e9;
            for (double cc : centers) best = std::min(best, std::fabs(cands[c].dx - cc));
            if (best < cfg.inlier_tol_px) inliers.push_back(c);
        }
        if (inliers.size() > best_inliers.size()) {
            best_inliers = std::move(inliers);
            best_centers = centers;
        }
    }

    // 3) Convert inliers → (li, ri) pairs, enforcing 1-to-1 by sorting
    //    by absolute disparity-residual and picking each Li / Ri only once.
    struct Pick { int li, ri; double res; };
    std::vector<Pick> picks;
    for (int c : best_inliers) {
        double best = 1e9;
        for (double cc : best_centers) best = std::min(best, std::fabs(cands[c].dx - cc));
        picks.push_back({cands[c].li, cands[c].ri, best});
    }
    std::sort(picks.begin(), picks.end(),
              [](const Pick& a, const Pick& b){ return a.res < b.res; });
    std::vector<bool> usedL(L.size(), false), usedR(R.size(), false);
    std::vector<std::pair<int,int>> out;
    for (auto& p : picks) {
        if (usedL[p.li] || usedR[p.ri]) continue;
        usedL[p.li] = usedR[p.ri] = true;
        out.emplace_back(p.li, p.ri);
    }
    return out;
}

}  // namespace mate
