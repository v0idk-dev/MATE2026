# Stereo Calibration

Add-on tooling for the **MATE 2026 Task 1.2 (Coral Garden Modelling)** photogrammetry pipeline. Builds on the existing single-camera tools in this directory; does **not** replace them.

These scripts produce the `stereo_extrinsics.yaml` file consumed by the **MATE 2026 Robot Controller** app's Photogrammetry pane. They live here, outside the app bundle, by design — calibration is a one-time-per-rig operation and shouldn't ship in the dmg.

The whole rig uses a **single combined CCTV feed** with both cameras side-by-side in one frame; these scripts splice that feed down the middle. Left half = left camera, right half = right camera.

## Workflow

The whole flow is: **calibrate each camera alone (using one half of the feed at a time) → capture paired images (using the full feed and splitting it) → run stereo calibration → import the three result files into the app.**

Before running anything, please download chessboard.pdf and print it on a standard letter sheet at 66.66666% size to acheive 2cm box size.

### Step 1 — Calibrate each camera individually

Run the following scripts **once per camera** (switch the camera in beween) and make sure to use the printed piece of paper from the step before **without bending the paper**.

**Left camera:**

```bash
python3 left_capture.py
python3 left_calibrate.py
mv left_output/calibration_data.pkl left_calibration_data.pkl
```

**Right camera:**

```bash
python3 right_capture.py
python3 rigbt_calibrate.py
mv right_output/calibration_data.pkl right_calibration_data.pkl
```

You should now have `left_calibration_data.pkl` and `right_calibration_data.pkl` next to these scripts.

**Quality gate:** the per-camera RMS reprojection error printed at the end of each `camera_calibration.py` run should be **below 1.0 px**, ideally below 0.5 px. If it's above 1.5 px, recalibrate that side with more images, more angle variety, and the chessboard filling more of the frame. Stereo accuracy cannot be better than the per-camera intrinsics that feed it.

### Step 2 — Capture paired chessboard images (full feed, spliced)

```bash
# Before running, make sure the camera feed has all four cameras
python3 stereo_capture.py
```

A side-by-side preview opens. Hold the chessboard so it's visible in **both** halves of the preview, then press `c` to capture a pair. The window flashes green to confirm. `q` or Escape to quit.

**Capture guidance for accurate stereo:**

- 15–20 pairs minimum.
- Move the chessboard around the full frame area in both halves (corners, edges, center).
- Vary distance: some pairs close, some far.
- Tilt the chessboard at multiple angles (yaw, pitch, roll). A flat, perpendicular chessboard in every pair makes for a degenerate solve.
- Keep the rig **completely rigid** during the entire capture session. Any flex between captures becomes irrecoverable error.

Pairs save to `stereo_calibration_images/left/pair_NN.jpg` and `.../right/pair_NN.jpg` with matching indices.

### Step 3 — Run stereo calibration

```bash
python3 stereo_calibrate.py
```

**Outputs land in `stereo_output/`:**

| Output File in stereo_output         | What it is                                                                                                                        |
| ------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------- |
| **`stereo_extrinsics.yaml`** | The file you import into the app.                                                                                                 |
| `stereo_extrinsics.pkl`            | Same data, pickle form, for any Python tooling.                                                                                   |
| `rectified_pair_*.jpg`             | Side-by-side rectified samples with horizontal lines drawn across — corresponding points should sit on the **same** line. |
| `rectification_check.jpg`          | Highlighted version of the first sample for quick visual inspection.                                                              |
| `calibration_report.txt`           | Human-readable summary with quality gate pass/fail.                                                                               |

**Quality gates (printed and written to the report):**

| Metric                        | Gate      | Why it matters                                                |
| ----------------------------- | --------- | ------------------------------------------------------------- |
| Stereo RMS reprojection error | ≤ 1.5 px | Direct contributor to triangulation noise.                    |
| Avg epipolar error            | ≤ 1.0 px | If high, rectification is wrong and dense matching will fail. |

If either gate fails, the calibration is still written (so you can inspect it), but the script warns you. **Do not import a failing calibration into the app for competition use** — it will not hit the ±5 cm spec.

### Step 4 — Import into the app

Three files import into the app:

1. **Cameras pane** → Import `left_calibration_data.pkl` and `right_calibration_data.pkl` separately. Rename them clearly (e.g. "Rig — Left Cam" / "Rig — Right Cam").
2. **Photogrammetry pane** → Set the "Left Camera Calibration" and "Right Camera Calibration" dropdowns to those two pkls. Import `stereo_extrinsics.yaml` to provide the rig geometry.

The app surfaces the calibration's RMS in the Task 1.2 page as a colored confidence indicator. Green (<1.0 px) is competition-ready; yellow (1.0–2.0 px) will work but with degraded accuracy; red (>2.0 px) blocks analysis.

## Coordinate convention

`SQUARE_SIZE` is the centimeter side length of one chessboard square. Whatever unit you set propagates: `T` (translation between cameras) and every downstream measurement come out in that unit. The defaults assume centimeters; the YAML records this as `unit: cm`.

## Troubleshooting

**"pair count mismatch"** — `stereo_calibration_images/left/` and `.../right/` must have the same number of files with matching names. `capture_stereo_images.py` enforces this; if you copied files in by hand, make sure indices line up.

**"only N pairs had a board detected in BOTH cameras"** — chessboard was only visible in one half in the others. Re-capture with the board fully inside both halves.

**Big baseline reported but rig is small** — `SQUARE_SIZE` is wrong. Re-check the actual side length of one chessboard square in the printed pattern you're using.

**RMS keeps coming out high (>2 px)** — most common causes, in order: not enough angle variety in pairs, board too small in the frame, rig flexing during capture, per-camera intrinsics already bad (re-do step 1 first).

**Splice is off-center** — the script splits at `width // 2`. If your CCTV feed has unequal halves (some do, e.g. with a status bar between them), tell me the exact pixel of the divider and I'll add an offset setting.
