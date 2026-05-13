// pipe_pipeline.cpp — orchestrator for the 10-step pipe stack.
#include "pipe_pipeline.hpp"
#include "stereo_math.hpp"
#include "image_quality.hpp"
#include "pipe_graph_validate.hpp"
#include "pipe_sampson.hpp"
#include <opencv2/calib3d.hpp>
#include <chrono>

namespace mate {

namespace {
auto tic() { return std::chrono::steady_clock::now(); }
long long toc_ms(decltype(tic()) t0) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(tic() - t0).count();
}
}  // namespace

PipePipelineOutput runPipePipeline(const cv::Mat& Lr, const cv::Mat& Rr,
                                    const cv::Mat& P1, const cv::Mat& P2,
                                    const cv::Mat& Q,
                                    const PipePipelineConfig& cfg_in) {
    PipePipelineOutput out;
    if (Lr.empty() || Rr.empty() || Lr.size() != Rr.size()) {
        out.error = "rectified pair empty or size mismatch"; return out;
    }
    PipePipelineConfig cfg = cfg_in;

    // Auto-estimate focal if the caller didn't supply one.
    double f_px = cfg.focal_px;
    if (f_px <= 0) {
        cv::Mat K = estimateIntrinsicsFromImage(Lr.cols, Lr.rows);
        f_px = focalFromK(K);
    }

    // ── 1. segmentPvc(L) and segmentPvc(R) ────────────────────────────
    auto t = tic();
    PvcSegmentResult sL = segmentPvc(Lr, cfg.pvc);
    PvcSegmentResult sR = segmentPvc(Rr, cfg.pvc);
    out.diag.ms_per_step[1] = toc_ms(t);
    out.diag.pvc_pixels_left = sL.pixels_kept;
    out.diag.pvc_pixels_right= sR.pixels_kept;
    out.diag.pvc_radius_px_left  = sL.median_radius_px;
    out.diag.pvc_radius_px_right = sR.median_radius_px;
    out.pvc_mask_left  = sL.mask;
    out.pvc_mask_right = sR.mask;

    // ── 0. Image-pair quality gate (blur, exposure, clipping, size) ──
    // Catches the common GIGO pairs (one eye out of focus, one over-
    // exposed, sub-resolution thumbnails). Runs before any heavy work
    // so we fail loudly before burning a second of CPU.
    if (cfg.run_image_quality_check) {
        t = tic();
        ImageQualityReport iqc = checkImagePairQuality(Lr, Rr);
        // Reuse ms_per_step[0] (previously unused per the comment).
        out.diag.ms_per_step[0] = toc_ms(t);
        out.diag.iqc_pass            = iqc.pass;
        out.diag.iqc_blur_var_left   = iqc.blur_var_left;
        out.diag.iqc_blur_var_right  = iqc.blur_var_right;
        out.diag.iqc_mean_lum_left   = iqc.mean_lum_left;
        out.diag.iqc_mean_lum_right  = iqc.mean_lum_right;
        out.diag.iqc_clip_frac_left  = iqc.clip_frac_left;
        out.diag.iqc_clip_frac_right = iqc.clip_frac_right;
        out.diag.iqc_warnings        = iqc.reasons;
        if (!iqc.pass && cfg.fail_on_image_quality) {
            out.error = "image quality gate failed: ";
            for (size_t i = 0; i < iqc.reasons.size(); ++i)
                out.error += (i ? "; " : "") + iqc.reasons[i];
            return out;
        }
    }

    // ── 5. SGBM (run early so we can use mean disparity in matching) ─
    cv::Mat disp, conf;
    if (cfg.run_sgbm) {
        t = tic();
        auto sg = computeSgbmDisparity(Lr, Rr, f_px, cfg.baseline_m, cfg.sgbm);
        out.diag.ms_per_step[5] = toc_ms(t);
        disp = sg.disparity; conf = sg.confidence;
        out.diag.sgbm_disparities_used = sg.num_disparities_used;
        out.diag.sgbm_pct_valid        = sg.pct_valid;
        out.sgbm_disparity = disp;
    }

    // Auto-estimate Z from SGBM if caller didn't.
    // Critical: restrict the depth median to PVC-masked pixels. Sampling
    // the entire frame leaks chalkboards/walls/floor depths into the
    // estimate, which then mis-sets w_min/w_max projected-diameter bands
    // and the σ_Z prior in the bundle adjuster. With the mask in place
    // the median is the actual subject distance regardless of background.
    double Z = cfg.subject_distance_Z;
    if (Z <= 0 && !disp.empty()) {
        std::vector<float> zs;
        cv::Mat xyz; cv::reprojectImageTo3D(disp, xyz, Q, true);
        const bool have_mask = !sL.mask.empty()
                            && sL.mask.rows == xyz.rows
                            && sL.mask.cols == xyz.cols;
        for (int y = 0; y < xyz.rows; y += 4) {
            const uchar* mrow = have_mask ? sL.mask.ptr<uchar>(y) : nullptr;
            for (int x = 0; x < xyz.cols; x += 4) {
                if (have_mask && !mrow[x]) continue;       // skip non-PVC
                float zv = xyz.at<cv::Vec3f>(y,x)[2];
                if (std::isfinite(zv) && zv > 0.05f && zv < 10.f) zs.push_back(zv);
            }
        }
        // Fallback: if mask was empty (no PVC detected) fall through to a
        // whole-frame median rather than producing Z = 1.0.
        if (zs.empty() && have_mask) {
            for (int y = 0; y < xyz.rows; y += 4)
                for (int x = 0; x < xyz.cols; x += 4) {
                    float zv = xyz.at<cv::Vec3f>(y,x)[2];
                    if (std::isfinite(zv) && zv > 0.05f && zv < 10.f) zs.push_back(zv);
                }
        }
        if (!zs.empty()) {
            std::nth_element(zs.begin(), zs.begin()+zs.size()/2, zs.end());
            Z = zs[zs.size()/2];
        }
    }
    if (Z <= 0) Z = 1.0;     // fall-back
    double sigZ = cfg.subject_distance_Z_sigma;
    if (sigZ <= 0) sigZ = depthUncertainty(Z, f_px, cfg.baseline_m, 1.0);

    // ── 2. detectLinesMulti(L) and (R) ────────────────────────────────
    t = tic();
    auto LM_L = detectLinesMulti(Lr, sL.mask, sL.dist_px, sL.skeleton, {}, cfg.lines);
    auto LM_R = detectLinesMulti(Rr, sR.mask, sR.dist_px, sR.skeleton, {}, cfg.lines);
    out.diag.ms_per_step[2] = toc_ms(t);
    out.diag.raw_lines_left  = (int)LM_L.size();
    out.diag.raw_lines_right = (int)LM_R.size();

    // ── 2b. Color-independent parallel-edge-pair detector ────────────
    // Adds pipe candidates that are missed by the LAB+chroma path
    // (heavily-occluded scenes, weak lighting, dirty PVC).
    if (cfg.run_parallel_pair) {
        t = tic();
        out.parallel_pairs_left = detectPipeParallelPairs(
            Lr, LM_L, f_px, std::max(0.05, Z),
            cfg.diameter.r_min_m, cfg.diameter.r_max_m,
            sL.mask, cfg.parallel_pair);
        out.parallel_pairs_right = detectPipeParallelPairs(
            Rr, LM_R, f_px, std::max(0.05, Z),
            cfg.diameter.r_min_m, cfg.diameter.r_max_m,
            sR.mask, cfg.parallel_pair);
        out.diag.ms_per_step[11] = toc_ms(t);
        out.diag.parallel_pairs_left  = (int)out.parallel_pairs_left.size();
        out.diag.parallel_pairs_right = (int)out.parallel_pairs_right.size();

        // Augment LM with the medial lines of confident pairs (votes=2 so
        // they pass the multi-detector vote gate downstream). Pre-fill
        // radius_px from the pair's measured spacing/2.
        auto inject = [&](std::vector<LinesMultiSegment>& dst,
                           const std::vector<PipePair>& pairs) {
            for (const auto& p : pairs) {
                LinesMultiSegment lms;
                lms.seg    = p.seg;
                lms.votes  = 2;        // counts as two-detector confirmation
                lms.radius_px = 0.5 * p.width_px;
                dst.push_back(lms);
            }
        };
        inject(LM_L, out.parallel_pairs_left);
        inject(LM_R, out.parallel_pairs_right);
    }

    // ── 2c. Pink-tape landmark detection ─────────────────────────────
    if (cfg.run_pink_tape) {
        t = tic();
        out.pink_blobs_left  = detectPinkTape(Lr, nullptr, cfg.pink_tape);
        out.pink_blobs_right = detectPinkTape(Rr, nullptr, cfg.pink_tape);
        out.diag.ms_per_step[12] = toc_ms(t);
        out.diag.pink_blobs_left  = (int)out.pink_blobs_left.size();
        out.diag.pink_blobs_right = (int)out.pink_blobs_right.size();
    }

    // ── 3. ransacRefitLines ──────────────────────────────────────────
    t = tic();
    auto RR_L = ransacRefitLines(Lr, sL.mask, LM_L, cfg.ransac);
    auto RR_R = ransacRefitLines(Rr, sR.mask, LM_R, cfg.ransac);
    out.diag.ms_per_step[3] = toc_ms(t);
    out.diag.ransac_kept_left  = (int)RR_L.size();
    out.diag.ransac_kept_right = (int)RR_R.size();

    // ── 4. gateByDiameter ────────────────────────────────────────────
    t = tic();
    auto DG_L = gateByDiameter(RR_L, sL.dist_px, f_px, Z, sigZ, cfg.diameter);
    auto DG_R = gateByDiameter(RR_R, sR.dist_px, f_px, Z, sigZ, cfg.diameter);
    out.diag.ms_per_step[4] = toc_ms(t);
    out.diag.diam_kept_left  = (int)DG_L.size();
    out.diag.diam_kept_right = (int)DG_R.size();

    // ── 6. matchPipesStereo ──────────────────────────────────────────
    t = tic();
    out.matches = matchPipesStereo(DG_L, DG_R, disp, conf, cfg.match);
    out.diag.ms_per_step[6] = toc_ms(t);
    out.diag.matches = (int)out.matches.size();

    // ── 7. + 8. Sampson endpoints + cylinder fit per match ───────────
    if (cfg.run_cylinder && !disp.empty()) {
        t = tic();
        auto cloud = disparityToCloud(disp, Q, Lr, 200000);
        out.diag.ms_per_step[7] = toc_ms(t);

        t = tic();
        for (auto& m : out.matches) {
            // 7. Sampson refine endpoints (purely 2D→3D, also useful when
            //    cloud has missing points along the pipe).
            SampsonRefineReport rep;
            cv::Point3d Xa = sampsonTriangulate(
                cv::Point2d(m.l0.x, m.l0.y), cv::Point2d(m.r0.x, m.r0.y), P1, P2, &rep);
            cv::Point3d Xb = sampsonTriangulate(
                cv::Point2d(m.l1.x, m.l1.y), cv::Point2d(m.r1.x, m.r1.y), P1, P2, &rep);
            (void)Xa; (void)Xb;        // currently unused; cylinder fit is more robust.

            // 8. Cylinder fit on dense cloud restricted to line mask.
            Cylinder3D cyl = fitCylinderFromCloud(m, cloud, sL.mask, Q, Lr.cols, Lr.rows, cfg.cylinder);
            if (cyl.ok) out.cylinders.push_back(cyl);
        }
        out.diag.ms_per_step[8] = toc_ms(t);
        out.diag.cylinders_ok = (int)out.cylinders.size();
    }

    // ── 9. junction graph ────────────────────────────────────────────
    if (cfg.run_graph && !out.cylinders.empty()) {
        t = tic();
        out.graph = buildPipeGraph(out.cylinders, P1, cfg.graph);
        out.diag.ms_per_step[9] = toc_ms(t);
        out.diag.graph_pipes     = (int)out.graph.pipes.size();
        out.diag.graph_junctions = (int)out.graph.junctions.size();
        out.diag.graph_rejected_isolated = out.graph.rejected_isolated;
    }

    // ── 10. bundle adjustment ────────────────────────────────────────
    if (cfg.run_bundle && !out.graph.pipes.empty()) {
        t = tic();
        out.diag.bundle_report = bundleAdjustPipes(out.graph, P1, P2, cfg.bundle);
        out.diag.ms_per_step[10] = toc_ms(t);
    }

    // Snapshot pre-template-injection graph for debug/visualisation.
    out.graph_pre_tmpl = out.graph;

    // ── 11. Template Procrustes-RANSAC fit ───────────────────────────
    if (cfg.run_template_fit && (int)out.graph.junctions.size() >= 3) {
        t = tic();
        out.template_fit = fitTemplate(out.graph, cfg.tmpl, cfg.tmpl_fit);
        out.diag.ms_per_step[13] = toc_ms(t);
        out.diag.template_fit_ok  = out.template_fit.ok;
        out.diag.template_inliers = out.template_fit.inliers;
        out.diag.template_rms_m   = out.template_fit.rms_m;
        out.diag.template_scale   = out.template_fit.s;
        out.diag.template_scale_axis = out.template_fit.s_axis;
    }

    // ── 12. Inject predicted pipes for occluded template edges ───────
    if (cfg.run_inject_predicted && out.template_fit.ok) {
        out.graph = injectPredictedPipes(out.graph, cfg.tmpl, out.template_fit,
                                          cfg.tmpl_fit, &out.diag.injection);
        // Refresh public counts.
        out.diag.graph_pipes     = (int)out.graph.pipes.size();
        out.diag.graph_junctions = (int)out.graph.junctions.size();
    }

    // ── 13. Post-fit graph sanity check ──────────────────────────────
    if (cfg.run_graph_validate && !out.graph.pipes.empty()) {
        auto v = validatePipeGraph(out.graph);
        out.diag.graph_long_pipes_flagged = v.num_long_pipes_flagged;
        out.diag.graph_radius_outliers    = v.num_radius_outliers;
        out.diag.graph_components         = v.num_connected_components;
        out.diag.graph_max_degree         = v.max_observed_degree;
        out.diag.graph_warnings           = v.warnings;
    }

    if (out.cylinders.empty() && out.diag.injection.n_predicted_pipes == 0)
        out.warning = "no validated cylinders — check PVC mask, baseline, lighting";
    return out;
}

}  // namespace mate
