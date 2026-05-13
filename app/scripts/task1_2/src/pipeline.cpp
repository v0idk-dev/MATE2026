// pipeline.cpp — orchestrates the 9-step pipeline for N stereo pairs.
#include "pipeline.hpp"
#include "pipe_pipeline.hpp"
#include "image_undistort.hpp"
#include "stereo_rectifier.hpp"
#include "plate_detector.hpp"
#include "pipe_detector.hpp"
#include "per_pair_model.hpp"
#include "multi_pair_fuse.hpp"
#include "ai_enhancer.hpp"
#include "manual_scale.hpp"
#include "underwater.hpp"
#include "scale_estimator.hpp"
#include "calibration_io.hpp"
#include "manhattan_calib.hpp"
#include "legacy_shims.hpp"   // free-function detectPipes/detectPlates/applyUnderwaterCorrection
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <filesystem>

namespace mate {

namespace {
auto now_ms() { return std::chrono::steady_clock::now(); }
long long delta_ms(decltype(now_ms()) a, decltype(now_ms()) b) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
}

void noteStep(ProgressFn& fn, int n, const char* label, double frac) {
    if (fn) fn(n, label, frac);
}

void writeDebug(const std::string& dir, int idx, const std::string& tag, const cv::Mat& im) {
    if (dir.empty() || im.empty()) return;
    std::filesystem::create_directories(dir);
    std::string p = dir + "/pair" + std::to_string(idx) + "_" + tag + ".jpg";
    cv::imwrite(p, im);
}
}  // namespace

PipelineOutput runPipeline(const PipelineInput& in) {
    return runPipeline(in, nullptr);
}

PipelineOutput runPipeline(const PipelineInput& in, ProgressFn progress) {
    PipelineOutput out;
    auto t0 = now_ms();

    if (in.pairs.empty()) { out.error = "no pairs supplied"; return out; }

    // Load global calibration (used unless per-pair overrides).
    CameraIntrinsics globalL{}, globalR{};
    StereoExtrinsics globalE{};
    bool calib_ok = !in.left_calib_yaml.empty() && !in.right_calib_yaml.empty();
    if (calib_ok) {
        auto oL = loadCameraIntrinsicsYaml(in.left_calib_yaml);
        auto oR = loadCameraIntrinsicsYaml(in.right_calib_yaml);
        if (!oL || !oR) {
            out.error = "calibration load failed: could not parse intrinsics YAML(s)";
            return out;
        }
        globalL = *oL; globalR = *oR;
        if (!in.stereo_extrinsics_yaml.empty()) {
            auto oS = loadStereoExtrinsicsYaml(in.stereo_extrinsics_yaml);
            if (!oS) { out.error = "calibration load failed: stereo extrinsics YAML"; return out; }
            globalE = *oS;
        }
    } else if (in.rig_baseline_m > 0) {
        // Manhattan-world auto-calibration fallback. Recover K from VPs
        // in the first pair's left frame, assume parallel-axis rig with
        // the supplied baseline. The downstream rectify/triangulate/fuse
        // path runs unchanged from this point — the rest of the pipeline
        // can't tell that calibration was synthesized rather than loaded.
        cv::Mat sample = cv::imread(in.pairs[0].left_path, cv::IMREAD_COLOR);
        if (sample.empty()) {
            out.warning = "no calibration provided and could not read first "
                          "left image for Manhattan auto-calib; falling back "
                          "to pixel units";
        } else {
            // Field-by-field rather than brace-init to avoid C++17
            // aggregate-init-with-default-member-init quirks across
            // older clangs.
            ManhattanCalibInput mci;
            mci.sample_image   = sample;
            mci.rig_baseline_m = in.rig_baseline_m;
            auto mc = deriveCalibrationFromManhattan(mci);
            if (mc.ok) {
                globalL = mc.K_left;
                globalR = mc.K_right;
                globalE = mc.extrinsics;
                calib_ok = true;
                out.warning += "auto-calibrated via Manhattan-world VPs ("
                            +  mc.note + "); ratios should be reliable, "
                            "absolute scale assumes the supplied "
                            + std::to_string(in.rig_baseline_m)
                            + " m rig baseline. ";
            } else {
                out.warning = "no calibration provided and Manhattan "
                              "auto-calib failed (" + mc.note + "); "
                              "results will be in pixel units only";
            }
        }
    } else {
        out.warning = "no calibration provided — using identity (results in pixel units only)";
    }

    std::vector<Model3D> per_pair_models;
    per_pair_models.reserve(in.pairs.size());
    out.per_pair.reserve(in.pairs.size());

    PlateDetectorParams pdp; pdp.target_h = in.plate_target_h;
    pdp.hue_tolerance = in.plate_hue_tolerance;
    pdp.expected_plate_count = in.expected_plates;

    for (size_t i = 0; i < in.pairs.size(); ++i) {
        PerPairDebug dbg; dbg.pair_index = (int)i;
        auto pi_t0 = now_ms();
        try {
            const auto& pp = in.pairs[i];
            cv::Mat L = cv::imread(pp.left_path,  cv::IMREAD_COLOR);
            cv::Mat R = cv::imread(pp.right_path, cv::IMREAD_COLOR);
            if (L.empty() || R.empty()) throw std::runtime_error("could not read images");

            CameraIntrinsics LK = globalL, RK = globalR;
            StereoExtrinsics  LE = globalE;
            if (!pp.left_calib_yaml_override.empty()) {
                auto o = loadCameraIntrinsicsYaml(pp.left_calib_yaml_override);
                if (!o) throw std::runtime_error("could not load left intrinsics override");
                LK = *o;
            }
            if (!pp.right_calib_yaml_override.empty()) {
                auto o = loadCameraIntrinsicsYaml(pp.right_calib_yaml_override);
                if (!o) throw std::runtime_error("could not load right intrinsics override");
                RK = *o;
            }
            if (!pp.stereo_extrinsics_yaml_override.empty()) {
                auto o = loadStereoExtrinsicsYaml(pp.stereo_extrinsics_yaml_override);
                if (!o) throw std::runtime_error("could not load stereo extrinsics override");
                LE = *o;
            }

            // Step 1 — undistort.
            noteStep(progress, 1, "undistort", (i + 0.1) / in.pairs.size());
            cv::Mat Lu = calib_ok ? undistortImage(L, LK, in.use_metal) : L;
            cv::Mat Ru = calib_ok ? undistortImage(R, RK, in.use_metal) : R;
            if (in.underwater) { applyUnderwaterCorrection(Lu, in.water_n); applyUnderwaterCorrection(Ru, in.water_n); }
            dbg.undistorted_left = Lu; dbg.undistorted_right = Ru;
            writeDebug(in.debug_dir, (int)i, "undistL", Lu);
            writeDebug(in.debug_dir, (int)i, "undistR", Ru);

            // Step 2 — pipe detection (on undistorted).
            noteStep(progress, 2, "detect pipes", (i + 0.25) / in.pairs.size());
            auto pipesL = detectPipes(Lu);
            auto pipesR = detectPipes(Ru);
            dbg.n_pipes_left = (int)pipesL.size(); dbg.n_pipes_right = (int)pipesR.size();
            {
                PipeDetector pd;
                writeDebug(in.debug_dir, (int)i, "pipesL", pd.visualize(Lu, pipesL));
                writeDebug(in.debug_dir, (int)i, "pipesR", pd.visualize(Ru, pipesR));
            }

            // Step 3 — plate detection.
            noteStep(progress, 3, "detect plates", (i + 0.4) / in.pairs.size());
            auto platesL = detectPlates(Lu, pdp);
            auto platesR = detectPlates(Ru, pdp);
            dbg.n_plates_left = (int)platesL.size(); dbg.n_plates_right = (int)platesR.size();
            {
                PlateDetector plateDet(pdp);
                writeDebug(in.debug_dir, (int)i, "platesL", plateDet.visualize(Lu, platesL));
                writeDebug(in.debug_dir, (int)i, "platesR", plateDet.visualize(Ru, platesR));
            }

            // Step 4 — per-image rough model: build using rough (unrectified)
            // detections projected through identity for visualisation only.
            // The metric model is built in step 6 from rectified geometry.
            noteStep(progress, 4, "rough per-image model", (i + 0.5) / in.pairs.size());
            // Using stereo rectified data for the actual metric build (step 6),
            // we just record the unrectified detections in dbg.
            dbg.rough_model.unit = "px";

            // Step 5 — stereo align (rectify).
            noteStep(progress, 5, "stereo rectify", (i + 0.6) / in.pairs.size());
            // rectifyStereoPair() now returns optional<RectifiedPair> and
            // takes optional<CameraIntrinsics>. Adapt while preserving the
            // legacy fall-through to identity-Q when calibration is absent.
            // Gate on the actual extrinsics struct content rather than the
            // YAML path string — the Manhattan auto-calib path populates
            // `LE` with synthesized R/T but leaves the YAML path empty,
            // and we still want it routed through stereoRectify here.
            RectifiedPair rect;
            const bool have_extrinsics = !LE.R.empty() && !LE.T.empty();
            if (calib_ok && have_extrinsics) {
                auto opt = rectifyStereoPair(Lu, Ru,
                    std::optional<CameraIntrinsics>{LK},
                    std::optional<CameraIntrinsics>{RK}, LE);
                if (!opt) throw std::runtime_error("rectifyStereoPair failed");
                rect = *opt;
                dbg.rectified_left = rect.left; dbg.rectified_right = rect.right;
                writeDebug(in.debug_dir, (int)i, "rectL", rect.left);
                writeDebug(in.debug_dir, (int)i, "rectR", rect.right);
            } else {
                rect.left = Lu; rect.right = Ru;
                rect.Q = (cv::Mat_<double>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,0,1, 0,0,-1,0);
            }

            // Pipe-first engine: route through the 14-stage pipe pipeline.
            if (in.engine_pipe) {
                PipePipelineConfig pcfg;
                pcfg.baseline_m = in.rig_baseline_m > 0 ? in.rig_baseline_m : 0.10;
                cv::Mat P1 = rect.P1.empty() ? cv::Mat::eye(3,4,CV_64F) : rect.P1;
                cv::Mat P2 = rect.P2.empty() ? cv::Mat::eye(3,4,CV_64F) : rect.P2;
                auto pr = runPipePipeline(rect.left, rect.right, P1, P2, rect.Q, pcfg);
                dbg.n_pipes_left  = (int)pr.graph.pipes.size();
                dbg.n_pipes_right = 0;
                if (!pr.error.empty()) { dbg.error = pr.error; }
                continue;  // skip plate-first steps for this pair
            }

            // Re-detect on rectified images so the (x,y) match epipolar geometry.
            auto pipesLr  = detectPipes(rect.left);
            auto pipesRr  = detectPipes(rect.right);
            auto platesLr = detectPlates(rect.left,  pdp);
            auto platesRr = detectPlates(rect.right, pdp);
            {
                PlateDetector plateDet(pdp);
                PipeDetector  pipeDet;
                writeDebug(in.debug_dir, (int)i, "rectPlatesL", plateDet.visualize(rect.left,  platesLr));
                writeDebug(in.debug_dir, (int)i, "rectPlatesR", plateDet.visualize(rect.right, platesRr));
                writeDebug(in.debug_dir, (int)i, "rectPipesL",  pipeDet.visualize(rect.left,   pipesLr));
                writeDebug(in.debug_dir, (int)i, "rectPipesR",  pipeDet.visualize(rect.right,  pipesRr));
            }

            // Fallback: synthetic / wrong stereo extrinsics produce a heavily
            // warped rectified image whose black borders + perspective changes
            // make HSV plate detection and Hough-line pipe detection collapse
            // to zero hits — even though the unrectified images had plenty.
            // When that happens, reuse the unrectified detections so the
            // per-pair model at least gets *something* to match (T4 order-pair
            // fallback can then run). Coordinates won't satisfy real epipolar
            // geometry, but neither does the rectification in this case, so
            // the result is no worse and the rest of the pipeline gets to run.
            if (platesLr.empty() || platesRr.empty()) {
                if (!platesL.empty() && !platesR.empty()) {
                    std::cerr << "[pair " << i << "] plates re-detect on rectified "
                              << "yielded L=" << platesLr.size()
                              << " R=" << platesRr.size()
                              << "; falling back to unrectified L=" << platesL.size()
                              << " R=" << platesR.size()
                              << " (rectification likely degenerate)\n";
                    platesLr = platesL;
                    platesRr = platesR;
                }
            }
            if (pipesLr.empty() || pipesRr.empty()) {
                if (!pipesL.empty() && !pipesR.empty()) {
                    std::cerr << "[pair " << i << "] pipes re-detect on rectified "
                              << "yielded L=" << pipesLr.size()
                              << " R=" << pipesRr.size()
                              << "; falling back to unrectified L=" << pipesL.size()
                              << " R=" << pipesR.size() << "\n";
                    pipesLr = pipesL;
                    pipesRr = pipesR;
                }
            }

            // Step 6 — stereo geometry: build metric per-pair model.
            noteStep(progress, 6, "stereo geometry sizing", (i + 0.8) / in.pairs.size());
            const std::string unit = calib_ok ? "m" : "px";
            Model3D pm = buildPerPairModel(rect, platesLr, platesRr, pipesLr, pipesRr, {}, unit);

            // Plate-prior fallback: if calibration looked weak (very few
            // plate matches), use the known 10 cm plate side to nudge scale.
            if (calib_ok && unit == "m") {
                // The new estimator takes PlateScaleObservation (4 resolved
                // 3D corners per plate) rather than raw Plate records.
                // Build observations from any plates whose corners have
                // been resolved by per_pair_model().
                std::vector<PlateScaleObservation> obs;
                obs.reserve(pm.plates.size());
                for (const auto& pl : pm.plates) {
                    if (!pl.corners_resolved) continue;
                    PlateScaleObservation o;
                    for (int k = 0; k < 4; ++k)
                        o.corners3d[k] = cv::Point3f(
                            (float)pl.corners[k].x,
                            (float)pl.corners[k].y,
                            (float)pl.corners[k].z);
                    obs.push_back(o);
                }
                if (!obs.empty()) {
                    ScaleResult est = estimateScaleFromPlates(obs, in.plate_side_m, pm.unit);
                    if (est.confidence > 0.0 && std::isfinite(est.k))
                        applyScale(pm, est.k, "plate-prior",
                                   "median measured plate side vs. 10 cm prior", est.confidence);
                }
            }

            pm.calibration.present = calib_ok;
            pm.calibration.image_width  = L.cols;
            pm.calibration.image_height = L.rows;
            // Surface the actually-used stereo geometry so the UI can show
            // baseline + RMS instead of the struct defaults (0 m, -1 px).
            // Prefer the explicit `baseline` scalar if the YAML carried one;
            // otherwise fall back to ‖T‖ which is what the rectifier uses.
            if (calib_ok) {
                double B = LE.baseline;
                if (!(B > 0) && !LE.T.empty()) B = cv::norm(LE.T);
                pm.calibration.baseline_m = B;
                // Stereo RMS first, else worst per-camera intrinsic RMS.
                double rms = LE.rms_px;
                if (!(rms > 0)) {
                    double rL = LK.rms_px, rR = RK.rms_px;
                    if (rL > 0 && rR > 0) rms = std::max(rL, rR);
                    else if (rL > 0)      rms = rL;
                    else if (rR > 0)      rms = rR;
                }
                if (rms > 0) pm.calibration.rms_px = rms;
                if (LE.avg_epipolar_err_px > 0)
                    pm.calibration.avg_epipolar_err_px = LE.avg_epipolar_err_px;
                if (LE.pairs_used > 0)
                    pm.calibration.pairs_used = LE.pairs_used;
            }

            dbg.stereo_model = pm;
            per_pair_models.push_back(std::move(pm));
        } catch (const std::exception& e) {
            dbg.error = e.what();
            out.warning += "[pair " + std::to_string(i) + "] " + e.what() + "; ";
        }
        out.timing_ms_per_step[6] += delta_ms(pi_t0, now_ms());
        out.per_pair.push_back(std::move(dbg));
    }

    // Step 7 — fuse N pairs.
    noteStep(progress, 7, "fuse pairs", 0.85);
    auto t7 = now_ms();
    out.model = fuseModels(per_pair_models);
    out.model.n_pairs_used = (int)per_pair_models.size();
    out.timing_ms_per_step[7] = delta_ms(t7, now_ms());

    // Step 8 — optional AI enhancement.
    if (in.ai_enhance || in.use_apple_intelligence) {
        noteStep(progress, 8, "AI enhance", 0.92);
        auto t8 = now_ms();
        AiEnhanceConfig ac;
        ac.provider          = in.ai_provider;
        ac.model             = in.ai_model;
        ac.python_executable = in.ai_caller_executable;
        ac.ai_caller_script  = in.ai_caller_script;
        ac.on_device         = in.use_apple_intelligence;
        // Provide one thumbnail per pair (left undistorted) for context.
        for (auto& d : out.per_pair) {
            if (!d.undistorted_left.empty() && !in.debug_dir.empty()) {
                std::string p = in.debug_dir + "/thumb_pair" + std::to_string(d.pair_index) + ".jpg";
                cv::Mat thumb;
                cv::resize(d.undistorted_left, thumb, cv::Size(256, 0), 0, 0, cv::INTER_AREA);
                cv::imwrite(p, thumb);
                ac.thumbnail_paths.push_back(p);
            }
        }
        AiEnhanceResult r = enhanceWithAi(out.model, ac);
        if (!r.ok) out.warning += "ai_enhance: " + r.error + "; ";
        else if (!r.warning.empty()) out.warning += "ai_note: " + r.warning + "; ";
        out.timing_ms_per_step[8] = delta_ms(t8, now_ms());
    }

    // Step 9 — manual override.
    if (in.manual_total_width_m > 0) {
        noteStep(progress, 9, "manual width override", 0.97);
        applyManualWidthOverride(out.model, in.manual_total_width_m);
    }

    out.model.warning = out.warning;
    out.timing_ms_total = delta_ms(t0, now_ms());
    out.model.timing_ms = out.timing_ms_total;
    noteStep(progress, 9, "done", 1.0);
    return out;
}

}  // namespace mate
