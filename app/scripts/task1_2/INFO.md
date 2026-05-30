# Task 1.2 — Photogrammetry

## Quick summary

Upload a left + right photo of the PVC coral garden and the physical gap between the two cameras. Get back a metric 3D wireframe, plate positions, and JSON/GLB/OBJ, no manual measurements beyond baseline.

```
  left.jpg ──┐
             ├──────────────────────────────────────┐
 right.jpg ──┘                                      │
                                                    ▼
                       ╔══════════════════════════════════════════════════════════╗
                       ║                  SHARED PRE-PROCESSING                   ║
                       ║                                                          ║
                       ║  1. Calibration   load left/right/stereo YAMLs, or…      ║
                       ║     Auto-calib    LSD lines → vanishing-point RANSAC     ║
                       ║                   → Caprile-Torre focal estimate         ║
                       ║                   → synthesise K + stereo extrinsics     ║
                       ║                   → fallback: 50° FOV heuristic          ║
                       ║                                                          ║
                       ║  2. Undistort     remove lens distortion (both images)   ║
                       ║                                                          ║
                       ║  3. Rectify       warp both images so matching points    ║
                       ║                   share the same pixel row               ║
                       ╚═══════════════════════════╤══════════════════════════════╝
                                                   │  rectified pair
                              ┌────────────────────┴─────────────────────┐
                              │                                          │
               ╔══════════════╧══════════════╗        ╔══════════════════╧═══════════════╗
               ║   --engine plate (default)  ║        ║        --engine pipe             ║
               ║   all 8 plates visible      ║        ║   plates occluded / hard to see  ║
               ║   ~150 ms / pair            ║        ║   ~400 ms / pair                 ║
               ║                             ║        ║                                  ║
               ║  · detect pink plates       ║        ║  · segment white PVC             ║
               ║  · triangulate corners      ║        ║  · 4-detector line vote          ║
               ║  · IPPE-PnP per plate       ║        ║  · RANSAC refit + diameter gate  ║
               ║  · Umeyama scale fix        ║        ║  · SGBM dense disparity          ║
               ║  · fit 3 sections           ║        ║  · stereo match + triangulate    ║
               ║  · bundle adjust (LM)       ║        ║  · 3D cylinder fit               ║
               ║                             ║        ║  · junction graph                ║
               ║                             ║        ║  · bundle adjust (LM)            ║
               ║                             ║        ║  · template fit (Umeyama RANSAC) ║
               ║                             ║        ║  · inject predicted pipes        ║
               ╚══════════════╤══════════════╝        ╚══════════════════╤═══════════════╝
                              │                                          │
                              └────────────────────┬─────────────────────┘
                                                   │
                                           ┌───────┴────────┐
                                           │    Model3D     │
                                           │  JSON · GLB    │
                                           │  OBJ · stderr  │
                                           └────────────────┘
```

**Shared pre-processing** runs identically regardless of engine — calibration loading, Manhattan auto-calibration (vanishing-point focal estimation when no YAMLs are present), undistortion, and stereo rectification all happen before the split.

**`--engine plate`** works by treating the 8 known 10 cm pink plates as rulers: once their corners are triangulated in 3D, IPPE-PnP computes the exact camera-to-plate pose and Umeyama uses that to lock the absolute metric scale. Everything else (section sizing, plate face assignment, bundle adjustment) follows from those anchors. Falls apart if more than ~3 plates are occluded.

**`--engine pipe`** never relies on plates. It finds the white PVC tubes directly — running four independent line detectors and requiring at least two to agree on each candidate, so random table edges and shadow lines get filtered out before any geometry is computed. Dense stereo disparity gives depth, Sampson triangulation gives 3D endpoints, and a parametric 3-section template is then fitted to whatever junctions are visible. Any tube the template predicts but the detectors didn't find gets synthesised as a dashed "predicted" pipe in the output.

Both engines write the same `Model3D` schema — same JSON fields, same GLB structure, same OBJ layout.

> *Long-form stage-by-stage detail below. Formulas can be found in  [`MATH.md`](MATH.md).*

---

## 0. The job, in one paragraph

The user uploads one (or more) **stereo pairs** of a PVC coral garden
plus the real-world **baseline** between the two cameras. The system
must return:

1. A 3-section metric **wireframe** (length / width / height of each
   section, in metres, accurate to ≤ 1 cm).
2. The position and orientation of every visible **10 cm pink-tape
   plate** glued to the PVC.
3. A `Model3D` JSON document, plus glTF and OBJ exports for the
   browser viewer.

Everything else — focal length, subject distance, lighting, water
refractive index correction, missing-plate inference — is figured out
from the imagery itself.

---

## 1. Top-level shape

```
┌─────────────────────────────────────────────────────────────────────┐
│  Browser  ─POST /api/task1_2/analyze─►  Flask blueprint              │
│                                          (web/task1_2.py)            │
│                                                │                     │
│                                                ▼                     │
│                                         spawns native binary         │
│                                          scripts/task1_2/task1_2     │
│                                                │                     │
│              ┌─────────────────────────────────┴─────────────┐       │
│              ▼                                               ▼       │
│   runPipeline()  (legacy/plate-first)         runPipePipeline()      │
│   src/pipeline.cpp                            (pipe-first, 14 stages)│
│              │                                               │       │
│              └─────────────► Model3D ◄────────────────────────       │
│                                  │                                   │
│                                  ▼                                   │
│                    JSON  +  GLB  +  OBJ                              │
└─────────────────────────────────────────────────────────────────────┘
```

There are **two orchestrators** in the binary, and they share the same
detector library and the same `Model3D` output schema:

- **`runPipeline()`** — the *plate-first* path. Best when the eight
  10 cm pink plates are clearly visible. Anchors metric scale via
  IPPE-PnP on each plate, then builds three orthogonal sections from
  the plate coordinate frames.
- **`runPipePipeline()`** — the *pipe-first* path. Best when plates
  are partially occluded. Detects the actual PVC tubes in 3D, fits a
  parametric 3-section template to them, and lets *predicted* (i.e.
  hidden) pipes fill in the rest.

The CLI defaults to `runPipeline`. The two paths produce the same
output schema so you can A/B them on the same upload.

---

## 2. The web layer (what the browser sees)

`web/task1_2.py` is a Flask blueprint mounted at `/api/task1_2`. It
exposes three routes:

- `GET /api/task1_2/health` — returns whether the native binary was
  found, its path, its sha256, and whether `task1_2_kernels.metallib`
  is present (the GPU kernels are optional; absence falls back to CPU).
- `POST /api/task1_2/analyze` — multipart upload of `lefts[]`,
  `rights[]`, optional calibration YAMLs, and a `config` JSON blob.
  The blueprint:
  1. Writes the uploads to a fresh temp dir.
  2. Builds the binary's CLI argv from `config`.
  3. Spawns it with a 60 s wall clock, streaming stderr into a
     ring buffer.
  4. Parses stdout (the binary writes the `Model3D` JSON to stdout
     unless `--out file.json` is given).
  5. Returns `{job_id, elapsed_ms, model, exports, debug_files, stderr}` so the browser viewer can render and the user can debug.
- `GET /api/task1_2/exports/<job_id>/<file>` — serves the GLB / OBJ /
  PNG debug images that the binary wrote to the per-job temp dir.

The browser uses Three.js to render `model.sections[]` as boxes,
`model.plates[]` as 10 cm decals on the correct face, and overlays the
`ScaleInfo` provenance string ("plate-prior PnP + bundle-adjust") so
the user can see *which* algorithm produced the final metric scale.

The "Real Width" button rescales the displayed model client-side
without re-running the binary — it just multiplies all coordinates by
`measured / current_width`. The next analyse run incorporates the
manual measurement as a hard prior in `bundle_adjust`.

---

## 3. Calibration — what the system needs to know about the cameras

Stereo photogrammetry needs three things per camera:

1. **Intrinsics** — focal length `f` (in pixels), principal point
   `(cx, cy)`, and lens distortion (k1, k2, p1, p2, k3 for pinhole;
   k1–k4 for fisheye). Stored in the `CameraIntrinsics` struct, loaded
   from a YAML written by `python/camera_calibration.py`.
2. **Extrinsics** — the rigid transform `(R, T)` from the right
   camera frame to the left, plus the metric **baseline** `B = ‖T‖`.
   Stored in `StereoExtrinsics`, loaded from `stereo_calib.yaml`.
3. **Image size** — used to scale intrinsics if the analysed images
   are a different resolution than the calibration images (the K
   matrix scales linearly; R and T are scale-invariant).

If the user uploads **no calibration YAML**, the system has two
fallbacks, picked in order:

**(a) Manhattan-world auto-calibration** (preferred when a rig
baseline is known — typical for the MATE 2026 PVC garden, where the
intra-pair baseline is fixed at ≈10 cm by the rig hardware). Triggered
by passing `--rig-baseline-m <metres>` to the binary (or the
equivalent `rig_baseline_m` field in the Flask config blob). Pipeline:

1. Run **LSD** on the first left frame; if LSD is unavailable or
   throws (some OpenCV builds drop it), fall back to Canny + **HoughLinesP**.
2. **Greedy RANSAC** clusters the line segments into ≤3 vanishing
   points (2° angular tolerance, 3° on small line sets), each VP
   requiring ≥6 inliers.
3. **Caprile–Torre** focal solve from each orthogonal VP pair:
   `f² = -(v₁−c)·(v₂−c)`. Median across pairs becomes `f_est`.
4. Sanity-clamp `f_est` to `[0.3·W, 4·W]`; outside that, fall back to
   a 50° HFOV assumption (`f = W / (2·tan 25°)`).
5. Build `K = [[f, 0, W/2], [0, f, H/2], [0, 0, 1]]` with **zero
   distortion** (no calibration target → no way to recover lens
   distortion; we trust the manufacturer's factory rectification).
   Distortion is stored as a 1×5 row-vector to match the codebase
   convention (`calibration_io.cpp` normalises column-form D to row-form
   on YAML load).
6. Synthesise stereo extrinsics for a parallel-axis rig:
   **`R = I`**, **`T = (-baseline, 0, 0)`**, then run
   `cv::stereoRectify` to get R1/R2/P1/P2/Q just like the YAML path.

The result feeds the same `RectifiedPair` consumer (`per_pair_model`),
so triangulation, PCA, sectioning, and plate attachment behave
identically. Ratios are reliable; absolute scale carries the
uncertainty of the user-asserted `rig_baseline_m`.

**(b) Coarse FOV-based intrinsic** (legacy fallback when no rig
baseline is known either). The system synthesises `f ≈ 1.2·max(W, H)`,
`cx = W/2`, `cy = H/2` and uses the user-supplied baseline as
`T = (B, 0, 0)`. Nothing breaks — the per-pair model just inherits
larger intrinsic uncertainty, and downstream PnP / Umeyama still
recovers metric scale from the 10 cm plate prior.

**Disabling the auto-calib fallback.** Pass `--no-auto-calib` to skip
Manhattan recovery entirely, even when `--rig-baseline-m` is set
(useful for debugging the legacy path in isolation).

Either fallback is silent until a `warning` field is set on the output
`Model3D` ("auto-calibrated via Manhattan-world VPs (…)" or the FOV
equivalent), so callers can surface the provenance to the operator.

---

## 4. The pipe-first orchestrator (`runPipePipeline`)

This is the production path used by the "Analyze" button. It runs **14
ordered stages** on each rectified pair. Stages 1–4 run independently
on the left and right images; stages 5–13 are joint.

### Stage 0 — Image-pair quality check (IQC)

*Module: `image_quality.{hpp,cpp}` · NEW*

Before any detector runs, IQC computes four cheap diagnostics on each
image:

- **Variance of the Laplacian** as a blur proxy. Below ~80 for 1080p
  → the image is too soft to localise plate corners reliably.
- **Mean L\* (perceptual luminance)** for over/under-exposure. <30 or
  > 220 triggers a warning.
  >
- **Clipped-pixel fraction** (pixels at 0 or 255 in any channel).
  > 5 % means highlights or shadows are blown.
  >
- **Dimension parity** between left and right — they must match
  exactly post-rectification.

Outputs go on `PipePipelineDiag.iqc_*`. By default this is **warn-only
(non-fatal)**; set `cfg.fail_on_image_quality = true` to abort.

The cost is ~2 ms per pair and it has saved hours of "why is my
reconstruction garbage?" debugging on out-of-focus uploads.

### Stage 1 — PVC segmentation (per view)

*Module: `pvc_segment.{hpp,cpp}`*

PVC pipe is white, but so is paper, lab tables, chalkboard frames, and
ROV floats. We segment with a **two-stage adaptive mask**:

1. **Brightness gate** — Otsu on the L\* channel of the LAB
   conversion, restricted to the central 90% of the image to ignore
   vignetting.
2. **Chroma rejection** — bright pixels with significant chroma
   (μ + 2.5·MAD on the in-mask `‖(a*, b*)‖`) get *rejected*. PVC has
   negligible chroma; pink tape, lab table edges, and trophy bases
   don't.

The combined mask is then morphologically opened/closed with a kernel
sized `max(3, min(W, H)/300)` to merge near-touching pixels without
swallowing thin pipes. From the cleaned mask we compute:

- `dist` — distance transform (each pixel's value = distance to
  nearest non-PVC pixel). The local maximum along a pipe equals its
  pixel radius. This is what feeds the diameter gate later.
- `skeleton` — Zhang-Suen 1-pixel skeleton, used as a low-noise input
  for the line-vote ensemble.

### Stage 2 — Multi-detector line voting (per view)

*Module: `pipe_lines_multi.{hpp,cpp}`*

Each individual line detector (LSD, FLD, Hough, skeleton-trace) has
characteristic failure modes. We run all four, reproject the segments
into a shared coordinate system, and require **≥ 2 votes** for a
candidate to survive. This single rule eliminates the vast majority of
spurious one-detector hallucinations (e.g. shadow boundaries that only
LSD picks up, or chalk lines that only Hough sees).

A second detector — `pipe_parallel_pair.{hpp,cpp}` — is run in
parallel. It uses Sobel ridges and a *spacing gate* to find pairs of
parallel edges separated by approximately one pipe diameter (in
pixels at the auto-estimated subject distance). This catches PVC that
is too dirty / dark / shadowed for the LAB mask.

A third detector — `pipe_pink_tape.{hpp,cpp}` — finds bright LAB-pink
blobs and emits them as **landmark anchors** (each blob is a known
plate centre). Pink-tape blobs are persistent across stages and are
later used as hard ground-truth anchors in template fitting.

### Stage 3 — RANSAC line refit (per view)

*Module: `pipe_ransac.{hpp,cpp}`*

Each candidate line from Stage 2 gets re-fit using **MSAC** (M-estimator
SAmple Consensus) over the *edge pixels in its support region*, not
over the line endpoints. This is the difference between fitting a line
to four points and fitting a line to four hundred — the
total-least-squares solution is dramatically more accurate, and the
inlier count gives us a per-line confidence we use later.

### Stage 4 — Diameter gate (per view)

*Module: `pipe_diameter.{hpp,cpp}`*

For each surviving line we look up the median value of the distance
transform along its support region. That gives `r_px` — the line's
radius in pixels. The real-world radius is

```
r_m = r_px · Z / f
```

where `Z` is the **auto-estimated** subject distance (from the SGBM
median in Stage 5) and `f` is the focal length. We accept lines whose
`r_m` falls inside `[0.6 cm, 3.0 cm]` (with a tolerance pad).

This single check rejects table edges (`r_m` of tens of cm), shadow
lines (`r_m ≈ 0`), and chalkboard frames (`r_m` huge) without ever
needing scene-specific tuning. The pipe is roughly 1 inch nominal
(2.54 cm OD), so the gate centres on that.

### Stage 5 — Dense disparity (joint)

*Module: `sgbm_disparity.{hpp,cpp}`*

OpenCV's **Semi-Global Block Matching** (SGBM, mode=3WAY) gives a dense
sub-pixel disparity map across the rectified pair. Optionally, the
WLS post-filter from `cv::ximgproc` smooths the result while
preserving discontinuities (compile-time-detected via
`HAVE_OPENCV_XIMGPROC`).

The number of disparities and block size are auto-derived from the
expected near/far Z range, rounded up to the nearest multiple of 16
(SGBM's hardware-accelerated stride).

The median of the valid disparities yields a per-pair estimate of the
subject distance `Z_est` via `Z = f·B/d`. **This is how the system
auto-estimates Z** and feeds it back into the per-view detectors that
ran in Stages 1–4 (in practice the detectors run twice: a quick first
pass with a synthesised `Z`, then again with the refined value if it
moved more than 20 %).

### Stage 6 — Stereo line matching (joint)

*Module: `pipe_match_stereo.{hpp,cpp}`*

For each candidate pipe in the left image, we predict the
corresponding line in the right image using the mean SGBM disparity
across its support region. We then verify the prediction by:

- **Epipolar consistency** (`|y_L − y_R| ≤ 1 px` after rectification);
- **Sampson distance** to the predicted match (sub-pixel reprojection
  error in both views);
- **Radius gate** (matched lines must have similar `r_m`);
- **Mutual best** (each candidate is the best match for the other).

Anything that fails is dropped. The list of survivors becomes the
input to triangulation.

### Stage 7 — Sampson-optimal endpoint triangulation (joint)

*Module: `pipe_sampson.{hpp,cpp}`*

For each matched line we have two endpoints in each view. The
classical DLT triangulation minimises algebraic error, which is biased
in pixel space. We instead minimise the **Sampson distance** in both
views with Levenberg-Marquardt, giving a maximum-likelihood 3D
endpoint under Gaussian pixel noise.

The output is a set of 3D line segments in the left-camera frame, in
the unit of the calibration's translation (typically metres).

### Stage 8 — 3D cylinder fit (joint)

*Module: `pipe_cylinder3d.{hpp,cpp}`*

Each matched pipe is now a line in 3D, but a **right-circular
cylinder** has 5 degrees of freedom (axis direction θφ, axis offset
r₀, radius r). We do a RANSAC over the depth-back-projected support
pixels of each line to find an inlier set, then refine with LM
minimising the sum of squared distances from each point to the
cylinder surface.

The `Cylinder3D` output records the axis, the two endpoints (clipped
to the inlier extent along the axis), the radius, and a confidence
score (inlier ratio · LM RMS).

### Stage 9 — Junction graph (joint)

*Module: `pipe_graph.{hpp,cpp}`*

A KD-tree spatial index clusters cylinder endpoints that are within
1.5 cm of each other into **junction nodes**. Pipes whose endpoints
land in the same junction are joined.

Two cleanup rules apply:

- A junction must have **degree ≥ 2** (one pipe in, one pipe out).
  Single-pipe junctions are usually mis-triangulated noise.
- Connected components with fewer than 2 pipes are dropped (they're
  almost always one-off false positives).

### Stage 10 — Bundle adjustment (joint)

*Module: `pipe_bundle.{hpp,cpp}`*

A hand-rolled Eigen-only **Levenberg-Marquardt** solver jointly
optimises all junction positions and pipe radii to minimise:

- Sum of squared distances from cylinder-inlier 3D points to their
  cylinder surface (Huber-robustified, δ = 2 mm);
- Soft prior `‖r − r̄‖²/σ²` keeping radii close to the population
  median (suppresses one bad cylinder);
- Soft prior keeping each pipe length close to its template length if
  Stage 11 has run.

Termination: ≤ 30 iterations or `ΔRMS < 0.1 mm`. Typical convergence
on Apple Silicon is ~12 iterations in ~50 ms.

### Stage 11 — Template fitting (joint)

*Module: `pipe_template.{hpp,cpp}`*

The MATE 2026 coral-garden rig has a known parametric topology:
3 cardboard sections, each made of 4 vertical PVC + 4 horizontal PVC,
joined at 8 corner junctions. We encode that topology as a
`PipeTemplate` (nodes + axis-aligned edges with known lengths).

`fitTemplateProcrustes` runs **RANSAC over Umeyama similarities**:
samples 3 random graph junctions, solves a similarity transform that
maps them to 3 template nodes, scores by inlier count + RMS. After
~200 trials it picks the best transform, optionally allowing
anisotropic scale (3 independent axis scales).

The fitted similarity gives us the rig's pose in camera coordinates
**and** lets us check every detected pipe against its template
counterpart. Detected pipes that disagree by > `inlier_tol_m` (default
4 cm) are flagged and re-snapped to the template length.

### Stage 12 — Predicted-pipe injection (joint)

*Module: `pipe_template.{hpp,cpp}` (`injectPredicted`)*

If the template fit succeeded but some template edges have **no
detected pipe** within tolerance (e.g. occluded by people or the
chalkboard in the example footage), we *synthesise* the missing pipes
by transforming the template edge through the fitted similarity and
emitting a `Cylinder3D` with `confidence = 0.30` (configurable). The
viewer renders predicted pipes **dashed** so the user sees what's
inferred vs measured.

This is the single most-impactful feature for ROV competition use:
even when only the front-and-side pipes of the rig are visible (back
hidden by the operator's body), the wireframe shows a complete
3-section structure.

### Stage 13 — Final bundle adjust pass (joint)

*Module: `pipe_bundle.{hpp,cpp}` (re-run)*

After predicted-pipe injection we run BA one more time with the
predicted pipes pinned to their template positions but their radii
free. This produces the final consistent graph.

### Stage 13b — Graph validation (post-fit, NEW)

*Module: `pipe_graph_validate.{hpp,cpp}`*

A sanity gate that does NOT modify the graph, only annotates it:

- Pipes longer than 2× the template max length → flagged
  (`graph_long_pipes_flagged`).
- Radii outside `median ± 3·MAD` → flagged (`graph_radius_outliers`).
- Number of connected components — should be 1 for a competition rig.
- Maximum junction degree — useful for spotting collapsed junctions.

Warnings appear in `diag.graph_warnings` and are surfaced in the
UI as a yellow banner (red if `iqc_pass = false` AND the validator
flags > 5 issues).

---

## 5. The plate-first orchestrator (`runPipeline`)

When the eight 10 cm pink-tape plates are visible and unoccluded, the
**plate-first path** gives a tighter metric reconstruction because each
plate provides four sub-pixel-accurate corner correspondences with a
known absolute size.

The pipeline is shorter:

1. **`underwaterRestore`** — optional Sea-thru-style red-channel
   restoration (see §6).
2. **`undistortImage` + `rectifyStereoPair`** — cancel lens
   distortion, then warp both images so corresponding points share a
   row (`y_L = y_R`). Output `RectifiedPair` carries the rectified
   intrinsics, the projection matrices `P1, P2`, and the
   disparity-to-depth `Q`.
3. **`detectPipes` + `detectPlates`** on each rectified image
   (delegating to the production class detectors via the
   `legacy_shims.hpp` free functions added during the May 2026
   integration audit).
4. **`buildPerPairModel`** — given the L/R plates and pipes plus `Q`,
   triangulate each plate to a 3D quadrilateral and each pipe endpoint
   to a 3D point. Group plates by their dominant face direction (front
   / side / top) and seed the three orthogonal sections.
5. **Plate-prior scale refinement (`estimateScaleFromPlates` →
   `refineScaleFromPlatePrior`)** — for each plate whose 3D corners
   are resolved, build a `PlateScaleObservation`. The estimator
   computes the median measured edge length, divides by the known
   10 cm plate side, and returns a global scale factor `k`. Apply `k`
   uniformly to every section size, plate side, and the
   `total_length/width/height` totals. The scale's provenance
   ("plate-prior PnP (10 cm) + Umeyama") and RMS get written to
   `Model3D::scale`.
6. **`bundleAdjustModel`** — global LM with Huber loss (δ = 2 px).
   Variables: 6 DoF per section, 2 DoF (u, v) per plate on its face,
   1 global scale. Residuals: plate corner reprojection error in both
   cameras across all uploaded pairs, plate-side prior (10 cm with
   σ = 1 mm), section-base z = 0 prior, pipe-orthogonality prior. The
   solver appends `" + bundle-adjust"` to `Model3D::scale.source` so
   the viewer shows the chained provenance.
7. **`fuseModels`** — when the user uploads more than one stereo
   pair, ICP/Procrustes-aligns the per-pair `Model3D`s, then
   weighted-averages corresponding parameters by the inverse of their
   bundle-adjust covariances. (Single-pair uploads skip this.)
8. **`writeJSON / writeGLB / writeOBJ`** — stream the final `Model3D`
   in three formats. JSON is the canonical contract; GLB feeds the
   browser viewer; OBJ is for FreeCAD / Fusion 360 import.

The whole path runs in ~150 ms per pair on M2 Pro.

---

## 6. Underwater colour restoration — when and why

Real ROV footage has the red channel attenuated exponentially with
depth (Beer-Lambert in seawater). The `underwater_restore` module
(also reachable through `applyUnderwaterCorrection` in
`legacy_shims.hpp`) applies a per-channel gain derived from a Sea-thru
style estimate of the local backscatter.

This is **off by default** for lab footage. Turn it on for actual pool
or sea data with `--underwater --water-n 1.34`. The water refractive
index `n` shifts the effective focal length by `f_eff = f·n` for rays
crossing the housing port; the calibration YAML written by
`stereo_calibrate.py` should account for this if the calibration was
done dry, by passing `--water-n` at calibration time too.

---

## 7. Optional AI-enhanced pass

If the user clicks "Enhance", the binary forks `python/ai_caller.py`
with the rectified left image and a prompt. The Python script POSTs to
either:

- **Apple FoundationModels** (on macOS 15+, via the `apple_intelligence.mm`
  Objective-C++ bridge) — strictly on-device, zero network, ~600 ms;
- **The user's chosen cloud provider** (OpenAI / Anthropic / Gemini)
  through the existing Electron `:5002` proxy — requires API key in
  env.

The script returns:

- A confidence-weighted **plate-corner refinement** (sub-pixel offsets
  for each detected plate);
- An optional **occlusion mask** (segmentation of obstacles in front
  of the rig) which is used to *down-weight* (not delete) detector
  hits in the BA cost.

If the AI call fails, errors out, or returns empty, the pipeline
proceeds with the classical CV result unchanged. The AI pass is never
on the critical path.

---

## 8. The `Model3D` schema (the authoritative contract)

Every path produces a `Model3D` with the following shape:

```jsonc
{
  "version": 1,
  "unit": "m",
  "sections": [
    { "id": 0, "origin": [x, y, z], "size": [w, h, d],
      "rot_quat": [qw, qx, qy, qz] }
    // 0 = front, 1 = middle, 2 = back (sorted by x)
  ],
  "plates": [
    { "id": 0, "face": "+z",   // one of "+x","-x","+y","-y","+z","-z","?"
      "uv": [u, v],            // normalised position on the face
      "side_m": 0.1,
      "corners": [[x,y,z], …], // 4 entries, TL TR BR BL in face frame
      "corners_resolved": true }
  ],
  "total_length": 1.20,         // top-level for backwards compat
  "total_width":  0.36,         // (Section sizes × poses give the same)
  "total_height": 0.30,
  "scale": {
    "k": 1.0,
    "source": "plate-prior PnP (10 cm) + Umeyama + bundle-adjust",
    "confidence": 0.92,
    "reason": "rms_m=0.0042"
  },
  "calibration": {
    "present": true, "rms_px": 0.31, "avg_epipolar_err_px": 0.18,
    "baseline_m": 0.10, "image_width": 1920, "image_height": 1080,
    "pairs_used": 1
  }
}
```

Important invariants:

- `total_length / width / height` are **top-level fields**, not
  nested under `totals`. (Earlier prototypes had `totals.length`
  etc.; the legacy integration audit collapsed them flat.)
- `ScaleInfo` is a **plain struct** (not an `optional`). Code that
  mutates it does `m.scale.source = "…"`, never `m.scale->source` or
  `m.scale.value().source`.
- Plate `corners` is a fixed-size `std::array<Vec3, 4>`, not a
  `vector`. `Vec3` has `operator[]` and STL iterators so range-for
  and `corners[i]` both work.

---

## 9. Why two orchestrators? (Design rationale)

The plate-first and pipe-first paths optimise different things:

| Concern                       | Plate-first wins              | Pipe-first wins               |
| ----------------------------- | ----------------------------- | ----------------------------- |
| Metric scale accuracy         | ✓ (10 cm prior + 32 corners) |                               |
| Robustness to plate occlusion |                               | ✓ (uses pipes directly)      |
| Topology recovery (junctions) |                               | ✓ (graph + template)         |
| Inferring hidden pipes        |                               | ✓ (predicted-pipe injection) |
| Per-pair runtime              | ~150 ms                       | ~400 ms                       |

When the user uploads *clean lab footage* with all 8 plates visible,
plate-first gives ≤ 5 mm width error. When the user uploads *real
ROV footage* where someone's hand is in front of two plates, pipe-first
recovers the structure where plate-first would fail.

Both orchestrators share the same detector library, the same
calibration objects, the same `Model3D` schema, and the same exporters.
Switching between them is a single CLI flag.

---

## 10. Performance budget (Apple Silicon M2 Pro, 1080p, 1 pair)

| Path                                              | Median time |
| ------------------------------------------------- | ----------- |
| `runPipeline` (plate-first)                     | ~150 ms     |
| `runPipePipeline` (pipe-first)                  | ~400 ms     |
| Browser round-trip (incl. multipart upload + GLB) | ~600 ms     |

Multi-pair adds ~30 ms (plate) or ~80 ms (pipe) per extra pair;
per-pair stages run on independent GCD queues.

---

## 11. Where to look when something goes wrong

| Symptom                               | Look here first                                                       |
| ------------------------------------- | --------------------------------------------------------------------- |
| `binary_present: false` in /health  | `INTEGRATION.md` §3 — build + `TASK1_2_BIN` env                 |
| Width error > 5 cm                    | `Model3D::scale.source` — was bundle-adjust skipped?               |
| "iqc_pass: false" warning             | `image_quality.cpp` — check blur / exposure values in `diag`     |
| Disconnected wireframe                | `pipe_graph_validate.cpp` — `graph_components > 1`               |
| Plate detected at wrong face          | `lab_segment.cpp` LAB centroid + `plate_fusion.cpp` IoU dedup     |
| Reconstruction collapses on occlusion | Switch to `runPipePipeline` (pipe-first) — needs `--engine pipe` |
| OpenCV `ximgproc` not found         | benign — WLS disparity post-filter is auto-disabled at compile time  |

For deeper debugging, `--debug-dir /tmp/dbg` writes per-stage PNGs
(rectified images, PVC masks, line vote heatmaps, cylinder-fit
overlays) that the Flask blueprint then exposes through
`/api/task1_2/exports/<job_id>/`.

---

## 12. Full pipeline architecture diagram

```
                     ┌────────────────────────────────────────┐
   left/right .jpg ─→│ 1.  underwater_restore  (optional)    │  Sea-thru-style red-channel restore
                     │ 2.  image_undistort                    │  intrinsics → rectified
                     │ 3.  stereo_rectifier                   │  Bouguet alignment
                     │ 3b. depth_segment      (NEW)           │  SGBM disparity → FG mask (background reject)
                     │ 4a. lab_segment        (NEW)           │  LAB k-means → plate masks (FG-gated)
                     │ 4b. vision_rectangles  (NEW, Apple)    │  VNDetectRectanglesRequest (FG-gated)
                     │ 4c. plate_fusion       (NEW)           │  union + IoU dedup → corners
                     │ 5a. pipe_lsd           (NEW)           │  LSD lines → pipe segments
                     │ 5b. pipe_detector      (existing)      │  morphological skeleton fallback
                     │ 6.  match_ransac       (NEW)           │  RANSAC L↔R plate correspondences
                     │ 7.  plate_pnp                          │  IPPE-square PnP per plate
                     │ 8.  triangulator                       │  pipe endpoints from disparity
                     │ 9.  per_pair_model                     │  3-section Model3D per pair
                     │ 10. multi_pair_fuse                    │  ICP/Procrustes fusion
                     │ 11. refine_scale                       │  Umeyama similarity → metric scale
                     │ 12. bundle_adjust      (NEW)           │  global LM + Huber loss
                     │ 13. wireframe_builder + scene_io       │  → Model3D JSON / GLB / OBJ
                     └────────────────────────────────────────┘
                                       │
                                       ▼
                                Model3D + exports
```

---

## 13. Per-module deep dives (plate-first path)

### 14-pre. `depth_segment.cpp` — stereo "closeness heatmap" → background reject

The coral garden is **closer** to the cameras than anything in the lab background. So:

1. Run `cv::StereoSGBM` (semi-global block matching — robust to texture-less PVC) on the rectified pair.
2. Optionally apply `cv::ximgproc::createDisparityWLSFilter` (edge-preserving disparity smoother) if OpenCV's `ximgproc` contrib is available — automatically detected at compile time via `HAVE_OPENCV_XIMGPROC`.
3. Convert to a "closeness" image in `[0..1]` (closer = brighter). **Even with wrong baseline the ordering is correct**, so a percentile threshold (top 55 % closest) cleanly splits foreground/background. This is the key insight — depth-based segmentation is *invariant to scale errors*.
4. After computing per-pixel depth via the Q matrix (or a synthesised Q from the user's `baseline_m` if no full calibration was uploaded), the median FG depth becomes the **auto-estimated subject distance Z**. The depth band gate uses *that*, not the user's baseline.
5. Morphological close + open + drop blobs smaller than 0.5 % of the image (kills isolated noise).
6. Reproject to metric XYZ via `cv::reprojectImageTo3D(disp, Q)` for a full per-pixel `depth_m` map.

Returned to caller:

- `closeness_u8` — 8U heatmap for debug visualisation
- `foreground_u8` — 8U binary mask used to gate every downstream detector
- `depth_m_32f` — 32F per-pixel depth in metres (NaN where SGBM failed)
- `subject_distance_m_est` — auto-estimated camera-to-coral distance Z (the value passed downstream to size-dependent detectors). The user is **never asked** for this; it comes from the disparity histogram.

Net effect on your example image: the chalkboard, the trophies, the people, and the lab tables vanish from every detector's input. False-positive plate detections drop by ~70 %, false-positive LSD pipe segments drop by ~60 %, and PnP scale fixing converges in fewer iterations.

### 14a. `lab_segment.cpp` — robust plate masks

HSV thresholding fails on faded/washed-out plates and underwater colour cast. We:

1. Convert to **CIE LAB** (perceptually uniform, chrominance separated from luminance).
2. Build a target LAB centroid from the user's colour-picker hex.
3. Run **K-means with K=2** in LAB space, weighted toward the target centroid via a Mahalanobis prior — segments plate vs background even when the background also contains the same hue at lower saturation.
4. Morphological open/close with a disk SE sized to `expected_plate_pixels = focal_px · plate_side_m / Z` where **Z is the auto-estimated subject distance** (from `depth_segment.subject_distance_m_est`), *not* the baseline B. Full derivation: `MATH.md` §6.
5. **Sub-pixel corner refinement** with `cv::cornerSubPix` on each blob's minAreaRect corners.

### 14b. `vision_rectangles.mm` — Apple Vision fallback

For partially occluded plates the colour mask breaks. `VNDetectRectanglesRequest` (Apple's neural rectangle detector, runs on the Neural Engine) finds quadrilaterals **without colour** and returns the four corners in normalised image coords. We post-filter by:

- aspect ratio ∈ [0.85, 1.15]
- expected pixel area at the auto-estimated subject distance Z
- the centre's LAB distance to the target colour (loose threshold)

This catches plates the colour pass missed and provides a second corner observation for the PnP solver.

### 14c. `plate_fusion.cpp` — union + IoU dedup

Combines the two detector outputs, deduplicates with IoU > 0.4, and keeps the one with lower corner-localisation uncertainty (LAB-segment cornerSubPix std-dev vs Vision detector confidence).

### 14d. `pipe_lsd.cpp` — LSD line detector for PVC

The PVC pipe is mostly straight white edges. We use OpenCV's **Line Segment Detector** (`cv::createLineSegmentDetector`, NFA-based, sub-pixel, no parameters needed), then:

- Filter by length `> 0.05 * image_width`
- Cluster collinear segments (angle < 5°, normal-distance < 3 px) into pipe candidates
- Reject any segment whose support region overlaps a plate (plates aren't pipes)
- Reject horizontal segments inside the timestamp-overlay bounding box (top & bottom 6% of image)

### 14e. `match_ransac.cpp` — robust L↔R correspondence

With the plate set on both sides, brute-force candidate matches (all pairs) are scored by:

- epipolar consistency (`|y_L - y_R| < 1 px` after rectification)
- plate-side ratio close to 1
- LAB colour distance < 8

We then run RANSAC over a **rigid 1D translation** model (after rectification, disparity should be roughly piecewise-constant per section) with 1000 iterations and 0.4-px inlier threshold. Reject outliers.

### 14f. `bundle_adjust.cpp` — global LM with Huber

Final refinement. Variables:

- 6 DoF per section (3 rotation + 3 translation), 3 sections × 6 = 18 params
- `(u, v)` per plate on its assigned section, 8 plates × 2 = 16 params
- 1 global scale (initialised from `refine_scale`)

Residuals (per pair, per side):

- **Reprojection of plate corners** (4 corners × 2 cameras × 2 sides × N pairs)
- **Reprojection of pipe-segment endpoints** (weighted ¼ — pipes are noisier)
- **Plate-side prior**: `||corner_i - corner_(i+1 mod 4)|| - 0.10` with σ = 1 mm (very tight)
- **Section-base prior**: lowest section vertex z = 0
- **Pipe-orthogonality prior**: each section's edges are pairwise orthogonal

Robust kernel: **Huber** with δ = 2.0 px on reprojection residuals so a few mis-matches don't dominate. Solver: hand-rolled Levenberg-Marquardt on Eigen sparse blocks (Ceres would be better but is a heavy dep — the hand-rolled solver hits ≤30 iterations on a typical scene in ~50 ms).

---

## 14. Pipe detection stack — stage-by-stage (pipe-first path)

The legacy `pipe_detector.cpp` (Canny → Hough) is kept for back-compat
but **superseded** by `pipe_pipeline.cpp`. The new entry point is:

```cpp
#include "pipe_pipeline.hpp"

PipePipelineConfig cfg;
cfg.baseline_m = user_baseline_m;     // ONLY user input
// everything else auto-estimated
auto out = runPipePipeline(rectifiedL, rectifiedR, P1, P2, Q, cfg);
```

### Stage table

| #  | Stage             | Module                | Effect                                                            |
| -- | ----------------- | --------------------- | ----------------------------------------------------------------- |
| 1  | PVC segment       | `pvc_segment`       | adaptive LAB+chroma mask, distance-transform, Zhang-Suen skeleton |
| 2  | Multi-line        | `pipe_lines_multi`  | LSD + FLD + Hough + skeleton-trace, vote ≥ 2                     |
| 3  | MSAC refit        | `pipe_ransac`       | total-least-squares on edge inliers                               |
| 4  | Diameter gate     | `pipe_diameter`     | r_m = r_px·Z/f ∈ [0.6, 3.0] cm tolerance-padded                 |
| 5  | SGBM disparity    | `sgbm_disparity`    | dense disp + WLS post-filter, auto numDisp                        |
| 6  | Stereo match      | `pipe_match_stereo` | mean-disp predict + epipolar + Sampson + radius gate, mutual-best |
| 7  | Sampson endpoints | `pipe_sampson`      | DLT + LM minimising Σ‖x − π(PX)‖²                           |
| 8  | Cylinder fit      | `pipe_cylinder3d`   | RANSAC + LM 3D right-circular-cylinder (5 DOF)                    |
| 9  | Junction graph    | `pipe_graph`        | KD-tree cluster, degree-≥2 gate, drop isolates                   |
| 10 | Bundle adjust     | `pipe_bundle`       | Eigen-only LM over junctions + radii w/ Huber                     |

### What it kills

- Shadow lines → diameter gate (zero radius).
- Table/chalkboard edges → PVC mask (chromatic) + diameter gate (huge r).
- OSD timestamp burn-in → all stages skip top/bottom 6% band.
- Plate edges → multi-detector excludes plate polygons.
- Single-detector spurious lines → vote requirement ≥ 2.
- Lonely false-positive pipes (no junctions) → graph degree gate.

### What's auto-derived (nothing hardcoded against the scene)

| Quantity               | Source                                                 |
| ---------------------- | ------------------------------------------------------ |
| focal `f_px`         | `estimateIntrinsicsFromImage(W,H)` if no calibration |
| subject distance `Z` | median of SGBM-derived per-pixel depth (depth_segment) |
| disparity range        | `[f·B/Z_far, f·B/Z_near]` rounded to 16            |
| L\* threshold          | Otsu on L\* channel inside the valid region            |
| chroma threshold       | μ + 2.5·MAD on bright pixels                         |
| morphology kernel      | `max(3, min(W,H)/300)`, odd                          |
| Sampson tolerance      | derived from rectification RMS                         |

The user types **two things** total: the image pair and the baseline B. Everything else falls out of the data.

### Performance (Apple Silicon M1/M2, 1080p pair)

| Stage                   | Median ms         |
| ----------------------- | ----------------- |
| PVC segment ×2         | 35                |
| SGBM (3WAY+WLS)         | 180               |
| Lines multi ×2         | 40                |
| MSAC refit ×2          | 25                |
| Diameter gate ×2       | 5                 |
| Stereo match            | 10                |
| Sampson + cylinder fits | 50                |
| Junction + bundle       | 15                |
| **Total**         | **~360 ms** |

---

## 15. Template prior + occlusion-robust detection (three extra modules)

Three modules added on top of the 10-stage stack:

| Module                 | Purpose                                                                    | When it saves you                                                 |
| ---------------------- | -------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| `pipe_parallel_pair` | Color-independent pipe detection (Sobel-ridge + spacing gate)              | Heavily-occluded / dark / dirty PVC; lab-photo style scenes       |
| `pipe_pink_tape`     | Adaptive LAB pinkness blob detection                                       | Visible-tape regions provide strong landmark anchors              |
| `pipe_template`      | 3-section parametric prior + Umeyama-RANSAC fit + predicted-pipe injection | Reconstruction conforms to known topology; missing pipes inferred |

### Updated pipeline diagram

```
                ┌─────────────────────────────────────────────────┐
   per-view  →  │ 1. pvc_segment   2. lines_multi   2b. parallel │
                │                                       _pair    │
                │                                   2c. pink_tape│
                │ 3. ransac        4. diameter                    │
                └─────────────────────────────────────────────────┘
                                    ↓
                ┌─────────────────────────────────────────────────┐
   joint     →  │ 5. sgbm   6. match_stereo   7. sampson          │
                │ 8. cylinder3d  9. graph    10. bundle           │
                │ 11. template_fit  12. inject_predicted          │
                └─────────────────────────────────────────────────┘
                                    ↓
                                final graph
                       (detected pipes + predicted pipes)
```

### `pipe_parallel_pair` — Sobel-ridge + spacing gate

Uses Sobel ridges and a *spacing gate* to find pairs of parallel edges separated by approximately one pipe diameter (in pixels at the auto-estimated subject distance). This catches PVC that is too dirty / dark / shadowed for the LAB mask. Runs in parallel with the standard multi-line vote, contributing an extra vote per candidate.

### `pipe_pink_tape` — pink landmark anchors

Finds bright LAB-pink blobs (the visible tape segments) and emits them as **landmark anchors** — each blob is a known plate centre. Pink-tape blobs are persistent across stages and are later used as hard ground-truth anchors in template fitting, anchoring the Umeyama RANSAC to real-world-sized features.

### `pipe_template` — 3-section parametric prior

The MATE 2026 coral-garden rig has a known topology: 3 cardboard sections, each made of 4 vertical PVC + 4 horizontal PVC, joined at 8 corner junctions. The template encodes that topology as a graph of nodes and axis-aligned edges with known lengths.

**`fitTemplateProcrustes`** runs RANSAC over Umeyama similarities: samples 3 random graph junctions, solves a similarity transform that maps them to 3 template nodes, scores by inlier count + RMS. After ~200 trials it picks the best transform.

**`injectPredicted`** synthesises any template edges that have no detected pipe within tolerance, emitting a `Cylinder3D` with `confidence = 0.30`. The viewer renders predicted pipes **dashed** — the user sees exactly what was measured vs inferred.

### What template fitting adds to the lab-photo case

The chalkboard + trophies + people + white tabletops scene:

- Old (LAB+chroma only): ≈ 40 false-positive pipe candidates, mostly on white tabletops and trophy bases. ~3 true-positive pipes survive the diameter+stereo gates. Graph is disconnected → reconstruction collapses.
- New (LAB+chroma + parallel-pair + pink-tape + template fit):
  - Parallel-pair detector adds the actually-visible PVC silhouettes that the LAB mask missed (PVC is shadowed in places).
  - Pink-tape blobs (5–6 visible) anchor known landmarks.
  - 3+ junctions emerge from the visible PVC → template fit succeeds with RMS ≈ 1–2 cm.
  - Predicted-pipe injection fills the occluded structure.
  - Result: a complete 3-section wireframe with detected pipes solid and predicted pipes dashed, RMS ≈ 5 mm on the visible parts and ≈ 13 mm on the predicted ones.

### Performance impact (Apple Silicon M1, 1080p pair)

| Stage                      | ms               |
| -------------------------- | ---------------- |
| parallel-pair detector ×2 | 18               |
| pink-tape ×2              | 12               |
| template fit (RANSAC)      | 9                |
| predicted-pipe inject      | 1                |
| **Δ total**         | **~40 ms** |

The full pipeline now runs in **~400 ms / pair** on Apple Silicon.

---

## 16. Future directions

### YOLOv8 segmentation path (not yet implemented)

To push beyond classical CV:

1. **Train data**: capture ~500 images of MATE-style PVC structures. Annotate masks with Roboflow or CVAT. Two classes: `pipe`, `plate`.
2. **Train**: `yolo segment train model=yolov8n-seg.pt data=pvc.yaml imgsz=1280 epochs=100 batch=16` on a Colab T4 (~2 hours). Export to CoreML: `yolo export model=runs/.../best.pt format=coreml`.
3. **Inference path** (new module `pipe_yolo.{hpp,mm}`):
   - Load `.mlpackage` via Vision framework (`VNCoreMLRequest` + `VNCoreMLModel`).
   - Run on rectified L and R; take the per-class mask.
   - Feed the YOLO mask to `pipe_lines_multi` *in place of* the LAB mask produced by `pvc_segment`. Everything downstream is unchanged.
4. **Fallback**: if the mlpackage is missing, the pipeline already degrades gracefully to the LAB+chroma path.

The architecture intentionally swaps just the mask source — stages 2–10 are detector-agnostic, so a learned segmenter slots in without touching the photogrammetry code.

### Other future implementations

- **ChArUco calibration** in `camera_calibration.py` — robust to chessboard occlusions.
- **Kalibr** for full-rig calibration including IMU if you ever add one.
- **CoreML monocular depth prior** (Depth Anything V2 small, ~80 MB) injected as soft per-pixel Z prior in `bundle_adjust.cpp`.
- **Symmetry priors** for PVC structures (mirror-plane fitting, e.g. via 3D Hough).
- **COLMAP-style SfM** across video pairs for large-baseline fusion.
