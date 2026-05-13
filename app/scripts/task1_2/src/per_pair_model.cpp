// per_pair_model.cpp — step 4: template-based 3-section model from one stereo pair.
//
// Strategy: start with the canonical coral-garden template, then REFINE it using
// detected plate 2-D positions. Always produce a valid model — never return empty.
//
// Coordinate system (model frame):
//   +X = length (left → right)
//   +Y = width  (front → back, always ~0.36 m)
//   +Z = height (floor → top)
//   Section.origin = center of the BOTTOM face  → origin.y = -width/2 = -0.18
//   Section.size   = {length_x, width_y, height_z}

#include "per_pair_model.hpp"
#include "accelerate_utils.hpp"
#include "triangulator.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>

namespace mate {

namespace {

// ---------------------------------------------------------------------------
// Canonical template
// ---------------------------------------------------------------------------

// Plate slot description: which section (0=left,1=mid,2=right), face, u, v.
struct PlateSlot {
    int section;   // 0=left, 1=middle, 2=right
    std::string face;
    double u, v;
};

// The 8 canonical plate positions per the spec.
static const PlateSlot kCanonicalPlates[8] = {
    // Front face (+y): camera-facing side
    {0, "+y", 0.0, 1.0},   // Plate 0: left section,   top-left
    {1, "+y", 0.0, 1.0},   // Plate 1: middle section, top-left
    {1, "+y", 0.5, 0.0},   // Plate 2: middle section, bottom-mid
    {2, "+y", 1.0, 1.0},   // Plate 3: right section,  top-right
    // Back face (-y)
    {2, "-y", 1.0, 1.0},   // Plate 4: right section,  top-right
    {1, "-y", 1.0, 1.0},   // Plate 5: middle section, top-right
    {1, "-y", 0.5, 0.0},   // Plate 6: middle section, bottom-mid
    {0, "-y", 0.0, 1.0},   // Plate 7: left section,   top-left
};

static const double kWidth = 0.36;
static const double kCanonical[3][3] = {
    // { length, height, x_start }
    { 0.30, 0.25, 0.00 },  // left
    { 0.40, 0.55, 0.30 },  // middle
    { 0.30, 0.40, 0.70 },  // right
};
static const double kTotalLength = 1.00;

// Build Section objects from length/height/x_start triplets.
Section makeSection(int id, double length, double height, double x_start) {
    Section s;
    s.id     = id;
    s.size   = { length, kWidth, height };
    s.origin = { x_start + length * 0.5, -kWidth * 0.5, 0.0 };
    s.confidence = 0.5;
    return s;
}

// ---------------------------------------------------------------------------
// Stereo helpers (kept from original)
// ---------------------------------------------------------------------------

// Progressive-fallback plate matching across L/R images on a rectified pair.
std::vector<std::pair<int,int>>
matchPlatesRectified(const std::vector<PlateDetection>& L,
                     const std::vector<PlateDetection>& R,
                     int row_tol_px = 30) {
    auto greedyByDx = [&](int row_tol, bool require_positive_dx, double area_min) {
        std::vector<std::pair<int,int>> out;
        std::vector<bool> usedR(R.size(), false);
        for (size_t i = 0; i < L.size(); ++i) {
            int best = -1; double bestd = 1e30;
            for (size_t j = 0; j < R.size(); ++j) {
                if (usedR[j]) continue;
                if (std::fabs(L[i].center.y - R[j].center.y) > row_tol) continue;
                const double dx = L[i].center.x - R[j].center.x;
                const double adx = std::fabs(dx);
                if (require_positive_dx ? (dx <= 0) : (adx <= 1e-3)) continue;
                if (area_min > 0) {
                    const double ar = std::min(L[i].area_px, R[j].area_px) /
                                      std::max(L[i].area_px, R[j].area_px);
                    if (ar < area_min) continue;
                }
                if (adx < bestd) { bestd = adx; best = (int)j; }
            }
            if (best >= 0) { out.emplace_back((int)i, best); usedR[best] = true; }
        }
        return out;
    };

    // T1: strict
    auto out = greedyByDx(row_tol_px, true, 0.4);
    if (!out.empty()) return out;

    // T2: flipped baseline sign
    out = greedyByDx(row_tol_px, false, 0.4);
    if (!out.empty()) {
        std::cerr << "matchPlatesRectified: T2 fallback (disparity-agnostic) → "
                  << out.size() << " matches\n";
        return out;
    }

    // T3: relaxed gates
    out = greedyByDx(row_tol_px * 3, false, 0.0);
    if (!out.empty()) {
        std::cerr << "matchPlatesRectified: T3 fallback (relaxed) → "
                  << out.size() << " matches; epipolar alignment is poor\n";
        return out;
    }

    // T4: sort-by-x order pairing (geometry meaningless, but pipeline renders)
    if (!L.empty() && !R.empty()) {
        std::vector<int> lidx(L.size()), ridx(R.size());
        for (size_t i = 0; i < L.size(); ++i) lidx[i] = (int)i;
        for (size_t j = 0; j < R.size(); ++j) ridx[j] = (int)j;
        std::sort(lidx.begin(), lidx.end(),
                  [&](int a, int b){ return L[a].center.x < L[b].center.x; });
        std::sort(ridx.begin(), ridx.end(),
                  [&](int a, int b){ return R[a].center.x < R[b].center.x; });
        const size_t n = std::min(lidx.size(), ridx.size());
        for (size_t k = 0; k < n; ++k) out.emplace_back(lidx[k], ridx[k]);
        std::cerr << "matchPlatesRectified: T4 fallback (order-pairing) → "
                  << out.size() << " matches; **3D positions will be NONSENSE**\n";
    }
    return out;
}

// Triangulate (x_l, y_l, x_r) → 3D using the rectified Q matrix.
Vec3 triangulateRect(const cv::Mat& Q, double xl, double yl, double xr) {
    double d = xl - xr;
    if (std::fabs(d) <= 1e-6) return {0,0,0};
    if (d < 0) d = -d;
    cv::Mat v = (cv::Mat_<double>(4,1) << xl, yl, d, 1.0);
    cv::Mat w = Q * v;
    const double W = w.at<double>(3,0);
    if (std::fabs(W) < 1e-9) return {0,0,0};
    Vec3 out{ w.at<double>(0,0)/W, w.at<double>(1,0)/W, w.at<double>(2,0)/W };
    if (out.z < 0) { out.x = -out.x; out.y = -out.y; out.z = -out.z; }
    return out;
}

// 1-D k-means with k initialised at evenly-spaced quantiles.
std::vector<int> kmeans1d(const std::vector<double>& x, int k, int iters = 20) {
    std::vector<int> lab(x.size(), 0);
    if (x.empty() || k <= 0) return lab;
    std::vector<double> sorted = x; std::sort(sorted.begin(), sorted.end());
    std::vector<double> mu(k);
    for (int j = 0; j < k; ++j)
        mu[j] = sorted[std::min((int)sorted.size()-1,
                                (int)((j+0.5)*sorted.size()/k))];
    for (int it = 0; it < iters; ++it) {
        for (size_t i = 0; i < x.size(); ++i) {
            int best = 0; double bd = std::fabs(x[i] - mu[0]);
            for (int j = 1; j < k; ++j) {
                double d = std::fabs(x[i]-mu[j]);
                if (d < bd) { bd=d; best=j; }
            }
            lab[i] = best;
        }
        std::vector<double> sum(k, 0); std::vector<int> cnt(k, 0);
        for (size_t i = 0; i < x.size(); ++i) { sum[lab[i]] += x[i]; cnt[lab[i]]++; }
        for (int j = 0; j < k; ++j) if (cnt[j]) mu[j] = sum[j] / cnt[j];
    }
    // Re-label so cluster 0 has smallest mean.
    std::vector<std::pair<double,int>> order(k);
    for (int j = 0; j < k; ++j) {
        double s = 0; int c = 0;
        for (size_t i = 0; i < x.size(); ++i) if (lab[i]==j) { s += x[i]; ++c; }
        order[j] = { c ? s/c : 0.0, j };
    }
    std::sort(order.begin(), order.end());
    std::vector<int> remap(k);
    for (int j = 0; j < k; ++j) remap[order[j].second] = j;
    for (auto& l : lab) l = remap[l];
    return lab;
}

// ---------------------------------------------------------------------------
// 2-D plate assignment helpers
// ---------------------------------------------------------------------------

// Given a sorted-by-x list of plate center X pixels in [0, imgW], assign
// each detection to one of 3 sections based on its X position.
// Returns section index 0/1/2.
int assignSection(float cx, float imgW) {
    const float frac = (imgW > 0) ? (cx / imgW) : 0.5f;
    // Canonical: left=30%, middle=40%, right=30% → boundaries at 0.30 and 0.70
    if (frac < 0.30f) return 0;
    if (frac < 0.70f) return 1;
    return 2;
}

// Estimate section length ratios from plate X positions in image space.
// We look at the X-center of each section's detected plates and use the
// spread between section centers to infer relative lengths.
// Returns {left_ratio, mid_ratio, right_ratio} normalized to sum = 1.
std::array<double,3> estimateLengthRatios(
    const std::vector<PlateDetection>& dets,
    float imgW)
{
    // Collect X pixel positions per section
    std::vector<double> secX[3];
    for (const auto& d : dets) {
        int sec = assignSection(d.center.x, imgW);
        secX[sec].push_back(d.center.x);
    }

    // Center of each section in pixel space
    double cx[3] = { -1, -1, -1 };
    for (int s = 0; s < 3; ++s) {
        if (secX[s].empty()) continue;
        double sum = 0;
        for (double v : secX[s]) sum += v;
        cx[s] = sum / secX[s].size();
    }

    // Count how many section centers we have
    int nValid = 0;
    for (int s = 0; s < 3; ++s) if (cx[s] >= 0) ++nValid;

    if (nValid < 2) {
        // Not enough info: use canonical ratios
        return { kCanonical[0][0] / kTotalLength,
                 kCanonical[1][0] / kTotalLength,
                 kCanonical[2][0] / kTotalLength };
    }

    // Fill in missing centers with canonical estimates
    if (cx[0] < 0) cx[0] = cx[1] - (cx[2] - cx[1]) * (kCanonical[0][0] / kCanonical[1][0]);
    if (cx[2] < 0) cx[2] = cx[1] + (cx[1] - cx[0]) * (kCanonical[2][0] / kCanonical[0][0]);
    if (cx[1] < 0) cx[1] = (cx[0] + cx[2]) * 0.5;

    // Boundaries are the midpoints between adjacent section centers
    double b01 = (cx[0] + cx[1]) * 0.5;
    double b12 = (cx[1] + cx[2]) * 0.5;
    double total = imgW;

    // Section lengths proportional to the pixel span each section occupies
    double l0 = b01;                   // left edge of model → midpoint
    double l1 = b12 - b01;            // between the two midpoints
    double l2 = total - b12;          // midpoint → right edge

    // Guard against negative or near-zero values
    l0 = std::max(l0, 0.01 * total);
    l1 = std::max(l1, 0.01 * total);
    l2 = std::max(l2, 0.01 * total);

    double sum = l0 + l1 + l2;
    return { l0/sum, l1/sum, l2/sum };
}

// Estimate section height ratios from plate Y positions.
//
// Key insight: we know each section has a TOP plate (v=1.0, canonical plates
// 0/1/3 on the front, or 4/5/7 on the back) and the middle section has a
// BOTTOM-MID plate (v=0.0, canonical plates 2/6). The pixel distance between
// a bottom plate and a top plate in the SAME section is directly proportional
// to that section's height — it's a true within-frame measurement immune to
// camera distance.
//
// For sections where we only see top plates (left, right), we use the top
// plate Y relative to the bottom-mid plate of the middle section as a
// cross-section height signal (they're both at known fractional heights).
//
// Returns {left_h, mid_h, right_h} ratios normalized so max = 1.0.
std::array<double,3> estimateHeightRatios(
    const std::vector<PlateDetection>& dets,
    float imgW, float imgH)
{
    // Per section: collect topmost Y (top plates, v≈1) and bottommost Y
    // (bottom plates, v≈0). Lower pixel Y = higher in image.
    double topY[3]    = { imgH, imgH, imgH };   // minimum Y pixel seen (topmost)
    double bottomY[3] = { 0,    0,    0    };   // maximum Y pixel seen (bottommost)
    bool   hasTop[3]  = { false, false, false };
    bool   hasBot[3]  = { false, false, false };

    for (const auto& d : dets) {
        int sec = assignSection(d.center.x, imgW);
        if (d.center.y < topY[sec]) {
            topY[sec] = d.center.y;
            hasTop[sec] = true;
        }
        if (d.center.y > bottomY[sec]) {
            bottomY[sec] = d.center.y;
            hasBot[sec] = true;
        }
    }

    double h[3];
    for (int s = 0; s < 3; ++s) {
        if (hasTop[s] && hasBot[s]) {
            // Direct span measurement: pixel distance top→bottom plate.
            // We scale by the known canonical v-fractions:
            //   middle section top plate v=1.0, bottom plate v=0.0 → span = full height
            //   left/right only have top plates (v=1.0); no bottom plate visible
            // For the middle section this gives us the full span directly.
            // For left/right we'll handle below.
            h[s] = bottomY[s] - topY[s];
        } else if (hasTop[s]) {
            // Only top plate seen. Use distance from top plate to the estimated
            // floor line. The floor corresponds to the bottommost plate seen
            // anywhere (most likely the bottom-mid plate of the middle section).
            double floorY = imgH * 0.85f;  // conservative fallback
            for (int j = 0; j < 3; ++j)
                if (hasBot[j]) floorY = std::max(floorY, bottomY[j]);
            h[s] = floorY - topY[s];
        } else {
            // No detections for this section → use canonical ratio as fallback.
            h[s] = -1.0;  // mark as missing
        }
    }

    // Fill missing sections using canonical ratios scaled to the measured sections.
    // Find the best reference: a section with a direct top→bottom measurement.
    double refMeasured = -1, refCanonical = -1;
    for (int s = 0; s < 3; ++s) {
        if (hasTop[s] && hasBot[s]) {
            refMeasured  = h[s];
            refCanonical = kCanonical[s][1];
            break;
        }
    }
    for (int s = 0; s < 3; ++s) {
        if (h[s] < 0) {
            if (refMeasured > 0) {
                // Scale canonical ratio by the measured reference
                h[s] = (kCanonical[s][1] / refCanonical) * refMeasured;
            } else {
                h[s] = kCanonical[s][1];
            }
        }
        h[s] = std::max(h[s], 1.0);  // guard zero/negative
    }

    double maxH = *std::max_element(h, h+3);
    if (maxH < 1e-6) maxH = 1.0;
    return { h[0]/maxH, h[1]/maxH, h[2]/maxH };
}

}  // namespace

// ---------------------------------------------------------------------------
// buildPerPairModel — public entry point
// ---------------------------------------------------------------------------

Model3D buildPerPairModel(const RectifiedPair& rect,
                          const std::vector<PlateDetection>& platesL,
                          const std::vector<PlateDetection>& platesR,
                          const std::vector<PipeSegment2D>& /*pipesL*/,
                          const std::vector<PipeSegment2D>& /*pipesR*/,
                          const PerPairConfig& cfg,
                          const std::string& unit_in)
{
    Model3D m;
    m.unit = unit_in.empty() ? "m" : unit_in;

    // -----------------------------------------------------------------------
    // A. Build canonical template sections (will be refined below)
    // -----------------------------------------------------------------------
    double lengths[3] = { kCanonical[0][0], kCanonical[1][0], kCanonical[2][0] };
    double heights[3] = { kCanonical[0][1], kCanonical[1][1], kCanonical[2][1] };

    const int nPlatesDetected = (int)platesL.size();

    // -----------------------------------------------------------------------
    // B. Estimate geometry from detected 2-D plate positions (LEFT image)
    // -----------------------------------------------------------------------
    float imgW = (float)rect.size.width;
    float imgH = (float)rect.size.height;
    if (imgW < 1) imgW = 1280;
    if (imgH < 1) imgH = 720;

    if (nPlatesDetected >= 2) {
        // Estimate length ratios
        auto lr = estimateLengthRatios(platesL, imgW);
        for (int s = 0; s < 3; ++s)
            lengths[s] = lr[s] * kTotalLength;

        // Estimate height ratios (normalized to canonical tallest section)
        auto hr = estimateHeightRatios(platesL, imgW, imgH);
        // Use the canonical max height as the absolute reference
        const double refMaxH = kCanonical[1][1];  // middle is tallest in canonical
        for (int s = 0; s < 3; ++s)
            heights[s] = hr[s] * refMaxH;

        // Guard minimum section dimensions
        for (int s = 0; s < 3; ++s) {
            lengths[s] = std::max(lengths[s], 0.05);
            heights[s] = std::max(heights[s], 0.05);
        }
    }

    // -----------------------------------------------------------------------
    // C. Attempt stereo triangulation to refine absolute scale
    //    (only if Q matrix is non-trivial — |Q[2][3]| > 0)
    // -----------------------------------------------------------------------
    bool hasStereo = false;
    if (!rect.Q.empty() && rect.Q.rows == 4 && rect.Q.cols == 4) {
        const double q23 = rect.Q.at<double>(2, 3);
        hasStereo = (std::fabs(q23) > 1e-6);
    }

    std::vector<Vec3> plate3d;
    if (hasStereo) {
        auto matches = matchPlatesRectified(platesL, platesR);
        plate3d.reserve(matches.size());
        for (auto [li, ri] : matches) {
            Vec3 p = triangulateRect(rect.Q,
                                     platesL[li].center.x, platesL[li].center.y,
                                     platesR[ri].center.x);
            if (p.z > 0 && std::isfinite(p.z)) plate3d.push_back(p);
        }

        // If we got at least 2 3D plate positions, use their horizontal spread
        // to refine section length estimates.
        if (plate3d.size() >= 2) {
            std::vector<double> pxs;
            pxs.reserve(plate3d.size());
            for (auto& p : plate3d) pxs.push_back(p.x);
            const double xmin = *std::min_element(pxs.begin(), pxs.end());
            const double xmax = *std::max_element(pxs.begin(), pxs.end());
            const double spread3d = xmax - xmin;

            // The spread of plate centers should approximately equal
            // ~0.8 * total_length (the outermost plates sit inside the frame).
            // Use this to set an absolute scale.
            if (spread3d > 0.05 && spread3d < 5.0) {
                const double implied_total = spread3d / 0.80;
                const double scaleFactor   = implied_total / kTotalLength;
                for (int s = 0; s < 3; ++s) {
                    lengths[s] *= scaleFactor;
                    heights[s] *= scaleFactor;
                }
                std::cerr << "per_pair_model: stereo scale refinement → "
                          << "spread3d=" << spread3d
                          << " impliedTotal=" << implied_total << "\n";
            }
        }
    }

    // -----------------------------------------------------------------------
    // D. Build the 3 sections from refined estimates
    // -----------------------------------------------------------------------
    // Ensure contiguous: each section's x_start = previous x_start + length
    double xStart = 0.0;
    for (int s = 0; s < cfg.n_sections && s < 3; ++s) {
        Section sec = makeSection(s, lengths[s], heights[s], xStart);
        // Confidence: higher if we had plates to estimate from
        sec.confidence = (nPlatesDetected >= 2) ? 0.6 : 0.3;
        m.sections.push_back(sec);
        xStart += lengths[s];
    }

    // -----------------------------------------------------------------------
    // E. Attach all 8 canonical plates at their FIXED known positions.
    //    The layout is spec-defined — do NOT attempt dynamic assignment.
    // -----------------------------------------------------------------------
    for (int i = 0; i < 8; ++i) {
        const auto& slot = kCanonicalPlates[i];
        Plate p;
        p.id         = i;
        p.section_id = std::min(slot.section, (int)m.sections.size()-1);
        p.face       = slot.face;
        p.u          = slot.u;
        p.v          = slot.v;
        p.side_m     = 0.10;
        p.confidence = (nPlatesDetected >= 2) ? 0.5 : 0.2;
        m.plates.push_back(p);
    }

    // Store raw 3D plate centers if we triangulated any.
    m.raw_plate_centers = plate3d;

    // -----------------------------------------------------------------------
    // F. Compute confidence and finalize
    // -----------------------------------------------------------------------
    // confidence = fraction of detected plates (0→0.2, 4+→0.8, 8→1.0)
    // (stored in warning/scale/calibration fields — no top-level confidence)
    if (nPlatesDetected == 0) {
        m.warning = "no plates detected — using canonical template";
    } else if (nPlatesDetected < 4) {
        m.warning = "few plates detected — template refined with limited data";
    }

    resolvePlateCorners(m);
    recomputeBounds(m);
    m.n_pairs_used = 1;

    // Summarize
    std::cerr << "per_pair_model: built model with "
              << m.sections.size() << " sections, "
              << m.plates.size() << " plates, "
              << nPlatesDetected << " plates detected, "
              << plate3d.size() << " triangulated\n";

    return m;
}

}  // namespace mate
