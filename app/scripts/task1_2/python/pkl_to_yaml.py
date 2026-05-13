#!/usr/bin/env python3
# =============================================================================
# pkl_to_yaml.py — convert a single-camera calibration .pkl produced by the
# standalone tool into a YAML file the C++ side reads via cv::FileStorage.
#
# The pkl format produced by camera_calibration.py is a dict with keys:
#   camera_matrix, distortion_coefficients, [reprojection_error,
#   rotation_vectors, translation_vectors]
#
# The C++ side expects a YAML with keys:
#   K, D, model ("pinhole"|"fisheye"), image_width, image_height, rms_px
#
# We auto-detect the distortion model from D's shape:
#   • shape (4, 1) → fisheye (4-coef equidistant)
#   • otherwise   → pinhole (Brown-Conrady, any of 4/5/8/12/14 coefs)
#
# Image width/height aren't stored in the pkl, so we either accept them via
# CLI (preferred — Flask passes them based on the rectified frame) or fall
# back to optical-center heuristics. If neither yields a sane answer, we
# leave them as 0; the C++ side then assumes "calibration was done at the
# runtime resolution" which works as long as Flask only ever feeds frames
# from the same source the .pkl was calibrated against.
#
# Usage:
#   python3 pkl_to_yaml.py <input.pkl> <output.yaml>
#                         [--width W] [--height H]
# =============================================================================
import argparse
import os
import pickle
import sys

import numpy as np
import cv2  # only used for FileStorage YAML write


def load_pkl(path):
    with open(path, 'rb') as f:
        d = pickle.load(f)
    if isinstance(d, dict):
        K = np.asarray(d['camera_matrix'], dtype=np.float64)
        D = np.asarray(d['distortion_coefficients'], dtype=np.float64)
        rms = float(d.get('reprojection_error', -1.0))
    elif isinstance(d, (list, tuple)) and len(d) >= 2:
        K = np.asarray(d[0], dtype=np.float64)
        D = np.asarray(d[1], dtype=np.float64)
        rms = -1.0
    else:
        raise ValueError(f"Unrecognized calibration format in {path}")
    if K.shape != (3, 3):
        raise ValueError(f"Camera matrix shape is {K.shape}, expected (3,3)")
    return K, D, rms


def detect_model(D):
    # Fisheye stores D as a (4, 1) column vector; pinhole as (1, N) row.
    if D.shape == (4, 1):
        return 'fisheye', D
    # Normalize anything else to a (1, N) row.
    flat = D.flatten()
    if flat.size not in (4, 5, 8, 12, 14):
        raise ValueError(f"Distortion has {flat.size} coefs; expected one of "
                         "4, 5, 8, 12, 14.")
    return 'pinhole', flat.reshape(1, -1)


def write_yaml(path, K, D, model, w, h, rms):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    try:
        fs.write('model', model)
        fs.write('K', K)
        fs.write('D', D)
        fs.write('image_width',  int(w) if w else 0)
        fs.write('image_height', int(h) if h else 0)
        fs.write('rms_px', float(rms) if rms is not None else -1.0)
    finally:
        fs.release()


def main(argv):
    ap = argparse.ArgumentParser(
        description="Convert per-camera calibration .pkl into the YAML format "
                    "consumed by the photogrammetry C++ binary.")
    ap.add_argument('input_pkl')
    ap.add_argument('output_yaml')
    ap.add_argument('--width',  type=int, default=0,
                    help="Image width  the calibration was done at (optional).")
    ap.add_argument('--height', type=int, default=0,
                    help="Image height the calibration was done at (optional).")
    args = ap.parse_args(argv)

    if not os.path.isfile(args.input_pkl):
        print(f"Error: input pkl not found: {args.input_pkl}", file=sys.stderr)
        return 2

    K, D, rms = load_pkl(args.input_pkl)
    model, D_norm = detect_model(D)

    # Heuristic for image size if not provided: cy*2 ≈ height, cx*2 ≈ width
    # (only reasonable when the principal point is near the center).
    w, h = args.width, args.height
    if not w:
        cx = K[0, 2]
        if cx > 1.0: w = int(round(cx * 2.0))
    if not h:
        cy = K[1, 2]
        if cy > 1.0: h = int(round(cy * 2.0))

    os.makedirs(os.path.dirname(os.path.abspath(args.output_yaml)) or '.',
                exist_ok=True)
    write_yaml(args.output_yaml, K, D_norm, model, w, h, rms)
    print(f"Wrote {args.output_yaml} (model={model}, image={w}x{h}, "
          f"rms={rms:.3f} px)" if rms >= 0 else
          f"Wrote {args.output_yaml} (model={model}, image={w}x{h})")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
