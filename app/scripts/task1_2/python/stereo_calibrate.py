import cv2
import glob
import numpy as np
import os
import pickle
import sys

# ============================================================================
# Stereo calibration tool for the rigid two-camera rig used in MATE Task 1.2.
#
# Inputs:
#   - LEFT_PKL_PATH:  per-camera intrinsics for the LEFT camera (produced by
#                     the existing camera_calibration.py).
#   - RIGHT_PKL_PATH: per-camera intrinsics for the RIGHT camera.
#   - LEFT_IMAGES_GLOB / RIGHT_IMAGES_GLOB: paired chessboard images (the i-th
#                     left image must show the same chessboard pose as the
#                     i-th right image). Use capture_stereo_images.py to make
#                     these — its output naming (pair_NN.jpg) sorts correctly.
#
# Outputs (in OUTPUT_DIRECTORY):
#   - stereo_extrinsics.yaml  ← THIS IS WHAT THE APP CONSUMES
#   - stereo_extrinsics.pkl   ← duplicate of the same data in pickle form,
#                               for any Python tooling that prefers pkl
#   - rectified_pair_*.jpg    ← side-by-side rectified samples for visual QC
#   - rectification_check.jpg ← composite with horizontal lines drawn across
#                               so you can verify epipolar alignment by eye
#   - calibration_report.txt  ← human-readable summary
#
# Run:   python3 stereo_calibrate.py
# ============================================================================

# ─── Settings ────────────────────────────────────────────────────────────────
LEFT_PKL_PATH  = 'left_calibration_data.pkl'
RIGHT_PKL_PATH = 'right_calibration_data.pkl'

LEFT_IMAGES_GLOB  = 'stereo_calibration_images/left/*.jpg'
RIGHT_IMAGES_GLOB = 'stereo_calibration_images/right/*.jpg'

CHESSBOARD_SIZE = (8, 5)   # Inner corners per row × column. MUST match the
                           # board used to generate the per-camera pkl files.
SQUARE_SIZE     = 2   # Chessboard square side in centimeters. MUST match
                           # the value used in your camera_calibration.py run.
                           # The unit propagates: T (translation) and all
                           # downstream measurements come out in centimeters.

OUTPUT_DIRECTORY = 'stereo_output'

# Quality gates — calibration is rejected if these are exceeded. Adjust only
# if you know what you're doing.
MAX_RMS_PIXELS         = 1.5   # Stereo RMS reprojection error (pixels)
MAX_AVG_EPIPOLAR_ERR   = 1.0   # Avg distance from points to epipolar lines (px)
MIN_VALID_PAIRS        = 8     # Refuse to calibrate with fewer pairs

# How many pairs to render as visual rectification samples.
RECTIFY_SAMPLE_COUNT = 5

# Output coordinate convention written into the YAML. The app's C++ code reads
# this; do not change without updating the C++ side too.
UNIT_LABEL = 'cm'
USE_FISHEYE = True   # Must match the per-camera calibration mode.


# ─── Helpers ─────────────────────────────────────────────────────────────────
def _load_intrinsics(pkl_path: str, label: str):
    """Load camera matrix + distortion from a per-camera calibration pkl.

    Tolerates both the dict format produced by camera_calibration.py
    (keys: 'camera_matrix', 'distortion_coefficients') and a tuple/list format
    (mtx, dist, ...). Returns (K, D, rms_or_None).
    """
    if not os.path.exists(pkl_path):
        sys.exit(f"Error: {label} calibration not found at {pkl_path}\n"
                 f"  Run camera_calibration.py for that camera first.")
    with open(pkl_path, 'rb') as f:
        data = pickle.load(f)

    if isinstance(data, dict):
        K = np.asarray(data['camera_matrix'], dtype=np.float64)
        D = np.asarray(data['distortion_coefficients'], dtype=np.float64)
        rms = data.get('reprojection_error', None)
    elif isinstance(data, (list, tuple)) and len(data) >= 2:
        K = np.asarray(data[0], dtype=np.float64)
        D = np.asarray(data[1], dtype=np.float64)
        rms = None
    else:
        sys.exit(f"Error: unrecognized calibration format in {pkl_path}")

    if K.shape != (3, 3):
        sys.exit(f"Error: {label} camera matrix has shape {K.shape}, expected (3,3)")
    # Normalize distortion to a flat (1, N) row vector for OpenCV.
    D = D.reshape(1, -1)
    if D.shape[1] not in (4, 5, 8, 12, 14):
        sys.exit(f"Error: {label} distortion has {D.shape[1]} coeffs; "
                 f"expected 4, 5, 8, 12, or 14.")

    return K, D, rms


def _detect_corners(images, label: str):
    """Detect chessboard corners in every image, refine sub-pixel.

    Returns parallel lists (corners, image_shape, indices_kept). Images that
    fail detection are dropped from the returned lists.
    """
    objpoints_template = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3),
                                  np.float32)
    objpoints_template[:, :2] = np.mgrid[
        0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]
    ].T.reshape(-1, 2)
    objpoints_template *= SQUARE_SIZE

    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                       30, 0.001)

    corners_list = []
    kept_indices = []
    img_size = None

    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            print(f"  [{label}] {os.path.basename(fname)}: unreadable, skipped")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])  # (w, h)
        elif (gray.shape[1], gray.shape[0]) != img_size:
            print(f"  [{label}] {os.path.basename(fname)}: size "
                  f"{gray.shape[1]}x{gray.shape[0]} differs from {img_size}, "
                  f"skipped")
            continue

        # Use adaptive threshold + normalized image for robustness against
        # uneven lighting (common with cheap CCTV cams).
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, flags)
        if not found:
            print(f"  [{label}] {os.path.basename(fname)}: NOT FOUND")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   refine_criteria)
        corners_list.append(corners)
        kept_indices.append(idx)
        print(f"  [{label}] {os.path.basename(fname)}: ok")

    return corners_list, img_size, kept_indices, objpoints_template


def _avg_epipolar_error(pts1, pts2, F):
    """Average distance from points to corresponding epipolar lines (pixels).

    Sanity-check metric: F * x should sit on the line that passes through x',
    so the perpendicular distance from x' to that line should be ~0.
    """
    pts1 = np.asarray(pts1).reshape(-1, 1, 2)
    pts2 = np.asarray(pts2).reshape(-1, 1, 2)
    lines2 = cv2.computeCorrespondEpilines(pts1, 1, F).reshape(-1, 3)
    lines1 = cv2.computeCorrespondEpilines(pts2, 2, F).reshape(-1, 3)
    p1 = pts1.reshape(-1, 2)
    p2 = pts2.reshape(-1, 2)
    # Distance from a point (x,y) to line ax+by+c=0 is |ax+by+c| / sqrt(a²+b²).
    d1 = np.abs(lines1[:, 0] * p1[:, 0] + lines1[:, 1] * p1[:, 1]
                + lines1[:, 2]) / np.sqrt(lines1[:, 0] ** 2 + lines1[:, 1] ** 2)
    d2 = np.abs(lines2[:, 0] * p2[:, 0] + lines2[:, 1] * p2[:, 1]
                + lines2[:, 2]) / np.sqrt(lines2[:, 0] ** 2 + lines2[:, 1] ** 2)
    return float(0.5 * (d1.mean() + d2.mean()))


def _write_yaml(path: str, data: dict):
    """Write OpenCV-compatible YAML.

    OpenCV's FileStorage format for matrices is:
        K_left: !!opencv-matrix
          rows: 3
          cols: 3
          dt: d
          data: [ ... ]
    The C++ reader uses cv::FileStorage which understands this.
    """
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    for k, v in data.items():
        if isinstance(v, np.ndarray):
            fs.write(k, v)
        elif isinstance(v, (int, float)):
            fs.write(k, float(v))
        elif isinstance(v, str):
            fs.write(k, v)
        elif isinstance(v, dict):
            # cv2.FileStorage cannot write nested mappings directly. Flatten:
            for kk, vv in v.items():
                _write_one(fs, f"{k}_{kk}", vv)
        else:
            fs.write(k, str(v))
    fs.release()


def _write_one(fs, key, val):
    if isinstance(val, np.ndarray):
        fs.write(key, val)
    elif isinstance(val, (int, float)):
        fs.write(key, float(val))
    else:
        fs.write(key, str(val))


# ─── Main ────────────────────────────────────────────────────────────────────
def stereo_calibrate():
    print("=" * 70)
    print("Stereo calibration — MATE 2026 Task 1.2")
    print("=" * 70)

    # 1. Load per-camera intrinsics.
    print("\n[1/6] Loading per-camera intrinsics...")
    K1, D1, rms1 = _load_intrinsics(LEFT_PKL_PATH, 'LEFT')
    K2, D2, rms2 = _load_intrinsics(RIGHT_PKL_PATH, 'RIGHT')
    print(f"  LEFT  K = fx={K1[0,0]:.1f}, fy={K1[1,1]:.1f}, "
          f"cx={K1[0,2]:.1f}, cy={K1[1,2]:.1f}, "
          f"RMS={rms1 if rms1 is None else f'{rms1:.3f} px'}")
    print(f"  RIGHT K = fx={K2[0,0]:.1f}, fy={K2[1,1]:.1f}, "
          f"cx={K2[0,2]:.1f}, cy={K2[1,2]:.1f}, "
          f"RMS={rms2 if rms2 is None else f'{rms2:.3f} px'}")
    if rms1 is not None and rms1 > 1.5:
        print(f"  ⚠  LEFT per-cam RMS {rms1:.2f} px is high — accuracy will suffer.")
    if rms2 is not None and rms2 > 1.5:
        print(f"  ⚠  RIGHT per-cam RMS {rms2:.2f} px is high — accuracy will suffer.")

    # 2. Enumerate paired images.
    print("\n[2/6] Enumerating paired images...")
    left_files  = sorted(glob.glob(LEFT_IMAGES_GLOB))
    right_files = sorted(glob.glob(RIGHT_IMAGES_GLOB))
    if len(left_files) != len(right_files):
        sys.exit(f"Error: pair count mismatch — {len(left_files)} left vs "
                 f"{len(right_files)} right. Pairs must be 1:1.")
    if len(left_files) < MIN_VALID_PAIRS:
        sys.exit(f"Error: only {len(left_files)} pairs found, need at least "
                 f"{MIN_VALID_PAIRS}. Capture more with capture_stereo_images.py.")
    print(f"  Found {len(left_files)} candidate pairs")

    # 3. Detect chessboards in both, intersect the index sets.
    print("\n[3/6] Detecting chessboards...")
    left_corners, left_size, left_idx, objp = _detect_corners(left_files, 'L')
    right_corners, right_size, right_idx, _ = _detect_corners(right_files, 'R')

    if left_size is None or right_size is None:
        sys.exit("Error: no usable images.")
    if left_size != right_size:
        # Acceptable if the two cameras run different resolutions, but
        # stereoCalibrate won't tolerate it. Require matching here.
        sys.exit(f"Error: image sizes differ — left {left_size} vs "
                 f"right {right_size}. Re-capture with matching resolution.")

    common = sorted(set(left_idx) & set(right_idx))
    if len(common) < MIN_VALID_PAIRS:
        sys.exit(f"Error: only {len(common)} pairs had a board detected in "
                 f"BOTH cameras, need at least {MIN_VALID_PAIRS}.")
    print(f"  {len(common)} pairs usable (board found in BOTH frames)")

    # Realign corner lists to the common index set.
    left_map  = {idx: pts for idx, pts in zip(left_idx, left_corners)}
    right_map = {idx: pts for idx, pts in zip(right_idx, right_corners)}
    pts_left  = [left_map[i] for i in common]
    pts_right = [right_map[i] for i in common]
    objpoints = [objp.copy() for _ in common]

    # 4. Stereo calibrate.
    print("\n[4/6] Running cv2.stereoCalibrate...")
    # Fixing the per-camera intrinsics is the standard practice when each
    # camera was calibrated alone first. It makes the stereo solve much more
    # stable and isolates extrinsic error from intrinsic error.
    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    if USE_FISHEYE:
        # Fisheye D must be (4,1). Convert if loaded as (1,4).
        D1f = D1.reshape(4, 1) if D1.size == 4 else D1
        D2f = D2.reshape(4, 1) if D2.size == 4 else D2
        # Fisheye expects (N,1,3) objpoints and (N,1,2) imgpoints.
        objp_fe   = [o.reshape(-1, 1, 3) for o in objpoints]
        ptsL_fe   = [p.reshape(-1, 1, 2) for p in pts_left]
        ptsR_fe   = [p.reshape(-1, 1, 2) for p in pts_right]
        fe_flags = (cv2.fisheye.CALIB_FIX_INTRINSIC)
        rms, K1_o, D1_o, K2_o, D2_o, R, T = cv2.fisheye.stereoCalibrate(
            objp_fe, ptsL_fe, ptsR_fe,
            K1, D1f, K2, D2f, left_size,
            flags=fe_flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        )
        # Fisheye doesn't return E and F directly — compute approximations.
        # E = [t]_x R is exact; F is derivable from K1/K2/E (informational only).
        tx = np.array([[0, -T[2,0], T[1,0]],
                       [T[2,0], 0, -T[0,0]],
                       [-T[1,0], T[0,0], 0]])
        E = tx @ R
        F = np.linalg.inv(K2_o).T @ E @ np.linalg.inv(K1_o)
    else:
        rms, K1_o, D1_o, K2_o, D2_o, R, T, E, F = cv2.stereoCalibrate(
            objpoints, pts_left, pts_right,
            K1, D1, K2, D2, left_size,
            criteria=criteria, flags=flags
        )
    baseline = float(np.linalg.norm(T))
    print(f"  Stereo RMS reprojection error: {rms:.3f} px")
    print(f"  Baseline (||T||): {baseline:.3f} {UNIT_LABEL}")
    print(f"  Translation (T):  [{T[0,0]:+.3f}, {T[1,0]:+.3f}, "
          f"{T[2,0]:+.3f}] {UNIT_LABEL}")
    # Convert R to a rotation vector for human-readable angle.
    rvec, _ = cv2.Rodrigues(R)
    angle_deg = float(np.degrees(np.linalg.norm(rvec)))
    print(f"  Rotation magnitude: {angle_deg:.2f}°")

    if rms > MAX_RMS_PIXELS:
        print(f"  ⚠  RMS {rms:.2f} px exceeds gate of {MAX_RMS_PIXELS} px.")
        print(f"     Recommendations: use more pairs (15+), vary chessboard")
        print(f"     pose more, ensure rig is rigid during capture, recheck")
        print(f"     per-camera calibrations.")

    # 5. Stereo rectify, sanity-check epipolar alignment.
    print("\n[5/6] Computing rectification + sanity checks...")
    if USE_FISHEYE:
        R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
            K1_o, D1_o.reshape(4,1), K2_o, D2_o.reshape(4,1),
            left_size, R, T,
            cv2.CALIB_ZERO_DISPARITY,
            newImageSize=left_size, balance=0.0, fov_scale=1.0
        )
    else:
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            K1_o, D1_o, K2_o, D2_o, left_size, R, T,
            alpha=0
        )
    # Average epipolar error using the corner points themselves as a proxy.
    all_left = np.vstack([p.reshape(-1, 2) for p in pts_left])
    all_right = np.vstack([p.reshape(-1, 2) for p in pts_right])
    epi_err = _avg_epipolar_error(all_left, all_right, F)
    print(f"  Avg epipolar error: {epi_err:.3f} px")
    if epi_err > MAX_AVG_EPIPOLAR_ERR:
        print(f"  ⚠  Epipolar error {epi_err:.2f} px exceeds gate of "
              f"{MAX_AVG_EPIPOLAR_ERR} px.")

    # Build undistort+rectify maps so the app can either reuse them or
    # recompute them from the saved K/D/R/P matrices.
    if USE_FISHEYE:
        map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(
            K1_o, D1_o.reshape(4,1), R1, P1, left_size, cv2.CV_16SC2)
        map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(
            K2_o, D2_o.reshape(4,1), R2, P2, left_size, cv2.CV_16SC2)
    else:
        map1_l, map2_l = cv2.initUndistortRectifyMap(
            K1_o, D1_o, R1, P1, left_size, cv2.CV_16SC2)
        map1_r, map2_r = cv2.initUndistortRectifyMap(
            K2_o, D2_o, R2, P2, left_size, cv2.CV_16SC2)

    # Render rectification samples.
    os.makedirs(OUTPUT_DIRECTORY, exist_ok=True)
    sample_indices = common[: min(RECTIFY_SAMPLE_COUNT, len(common))]
    for n, idx in enumerate(sample_indices):
        l_img = cv2.imread(left_files[idx])
        r_img = cv2.imread(right_files[idx])
        if l_img is None or r_img is None:
            continue
        l_rect = cv2.remap(l_img, map1_l, map2_l, cv2.INTER_LINEAR)
        r_rect = cv2.remap(r_img, map1_r, map2_r, cv2.INTER_LINEAR)
        combined = cv2.hconcat([l_rect, r_rect])
        # Draw horizontal lines so epipolar alignment is visible by eye.
        h = combined.shape[0]
        for y in range(0, h, max(20, h // 30)):
            cv2.line(combined, (0, y), (combined.shape[1], y),
                     (0, 255, 0), 1)
        cv2.imwrite(os.path.join(OUTPUT_DIRECTORY,
                                 f"rectified_pair_{n:02d}.jpg"), combined)
    if sample_indices:
        # Save a single labeled "rectification check" image as the showcase.
        showcase = cv2.imread(os.path.join(
            OUTPUT_DIRECTORY, f"rectified_pair_00.jpg"))
        if showcase is not None:
            cv2.putText(showcase, "Corresponding points should sit on the "
                        "SAME horizontal line.", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imwrite(os.path.join(OUTPUT_DIRECTORY,
                                     "rectification_check.jpg"), showcase)

    # 6. Persist results.
    print("\n[6/6] Writing outputs...")
    yaml_path = os.path.join(OUTPUT_DIRECTORY, 'stereo_extrinsics.yaml')
    out = {
        'image_width':  int(left_size[0]),
        'image_height': int(left_size[1]),
        'unit': UNIT_LABEL,
        'square_size': float(SQUARE_SIZE),
        'pairs_used': int(len(common)),
        'stereo_rms_px': float(rms),
        'avg_epipolar_err_px': float(epi_err),
        'baseline': float(baseline),
        'rotation_angle_deg': float(angle_deg),
        'K_left':  K1_o,
        'D_left':  D1_o,
        'K_right': K2_o,
        'D_right': D2_o,
        'R':  R,           # rotation: right cam frame relative to left
        'T':  T,           # translation: right cam origin in left cam frame
        'E':  E,           # essential matrix
        'F':  F,           # fundamental matrix
        'R1': R1,          # left rectification rotation
        'R2': R2,          # right rectification rotation
        'P1': P1,          # left rectified projection (3×4)
        'P2': P2,          # right rectified projection (3×4)
        'Q':  Q,           # disparity-to-depth reprojection 4×4
    }
    _write_yaml(yaml_path, out)
    print(f"  Wrote {yaml_path}")

    pkl_path = os.path.join(OUTPUT_DIRECTORY, 'stereo_extrinsics.pkl')
    with open(pkl_path, 'wb') as f:
        pickle.dump(out, f)
    print(f"  Wrote {pkl_path}")

    report_path = os.path.join(OUTPUT_DIRECTORY, 'calibration_report.txt')
    with open(report_path, 'w') as f:
        f.write("STEREO CALIBRATION REPORT\n")
        f.write("=" * 60 + "\n\n")
        f.write(f"Image size           : {left_size[0]} x {left_size[1]}\n")
        f.write(f"Pairs used           : {len(common)}\n")
        f.write(f"Square size          : {SQUARE_SIZE} {UNIT_LABEL}\n\n")
        f.write(f"Stereo RMS error     : {rms:.4f} px ")
        f.write(f"(gate ≤ {MAX_RMS_PIXELS} px) "
                f"{'OK' if rms <= MAX_RMS_PIXELS else 'FAIL'}\n")
        f.write(f"Avg epipolar error   : {epi_err:.4f} px ")
        f.write(f"(gate ≤ {MAX_AVG_EPIPOLAR_ERR} px) "
                f"{'OK' if epi_err <= MAX_AVG_EPIPOLAR_ERR else 'FAIL'}\n\n")
        f.write(f"Baseline (||T||)     : {baseline:.4f} {UNIT_LABEL}\n")
        f.write(f"Rotation magnitude   : {angle_deg:.4f} deg\n")
        f.write(f"  (≈ 0° = cameras parallel; >5° usually means the rig is "
                f"twisted)\n\n")
        f.write("Translation T (right cam origin in left cam frame):\n")
        f.write(f"  [{T[0,0]:+.4f}, {T[1,0]:+.4f}, {T[2,0]:+.4f}] "
                f"{UNIT_LABEL}\n\n")
        f.write("Rotation R (3x3):\n")
        for row in R:
            f.write("  [" + ", ".join(f"{v:+.6f}" for v in row) + "]\n")
        f.write("\nNext steps:\n")
        f.write("  1. Inspect rectified_pair_*.jpg — corresponding points\n")
        f.write("     in the left half should sit on the SAME horizontal\n")
        f.write("     line as their match in the right half.\n")
        f.write("  2. Import stereo_extrinsics.yaml into the MATE app via\n")
        f.write("     Settings → Photogrammetry.\n")
    print(f"  Wrote {report_path}")
    print(f"  Wrote {len(sample_indices)} rectification sample(s)")

    print("\n" + "=" * 70)
    if rms <= MAX_RMS_PIXELS and epi_err <= MAX_AVG_EPIPOLAR_ERR:
        print("✓ Stereo calibration PASSED quality gates.")
    else:
        print("⚠ Stereo calibration produced results but did NOT pass all")
        print("  gates. Review calibration_report.txt and consider")
        print("  re-capturing with more / better-distributed pairs.")
    print("=" * 70)


if __name__ == "__main__":
    stereo_calibrate()
