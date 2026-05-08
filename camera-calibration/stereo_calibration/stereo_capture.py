import cv2
import os
import time

# ============================================================================
# Stereo image capture from a SPLICED CCTV feed for the rigid two-camera rig
# used in MATE Task 1.2.
#
# Captures synchronized image pairs from a single combined feed laid out as a
# 2x2 quadrant grid. Bottom-left quadrant → left camera; bottom-right quadrant
# → right camera. Because both quadrants come from the same frame,
# synchronization is perfect — no thread coordination needed.
#
# The pairs feed `stereo_calibrate.py`, which produces `stereo_extrinsics.yaml`
# for the app's Photogrammetry pane.
#
# Press 'c' to capture a synchronized pair (only enabled when the chessboard
# is visible in BOTH quadrants). Press 'q' or Escape to quit.
# ============================================================================

# ─── Settings ────────────────────────────────────────────────────────────────
# Source for the combined feed. Either:
#   - an integer camera index, e.g. 0      (USB capture card showing the CCTV)
#   - a URL string,            e.g. 'rtsp://user:pass@192.168.1.50:554/stream'
FEED_SOURCE = 0

CHESSBOARD_SIZE = (8, 5)   # Inner corners per chessboard row × column.
                           # MUST match what was used for per-camera calibration.

OUTPUT_DIRECTORY = 'stereo_calibration_images'
IMAGE_RES = (1280, 720)    # Combined frame requested resolution. The two
                           # camera views are the bottom-left and bottom-right
                           # quadrants of this 2x2 grid.

# A captured pair is only saved if the chessboard is detected in BOTH quadrants.
# Set False to allow capturing pairs even when one quadrant fails detection
# (useful for debugging camera placement; never use for real calibration).
REQUIRE_BOTH_DETECTED = True


def capture_stereo_images():
    """Capture synchronized stereo pairs by splicing a combined feed."""
    left_dir  = os.path.join(OUTPUT_DIRECTORY, 'left')
    right_dir = os.path.join(OUTPUT_DIRECTORY, 'right')
    os.makedirs(left_dir,  exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    print(f"Opening combined feed (source: {FEED_SOURCE})...")
    cap = cv2.VideoCapture(FEED_SOURCE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  IMAGE_RES[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_RES[1])
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    if not cap.isOpened():
        print(f"Error: Could not open feed {FEED_SOURCE}")
        return

    # Probe to learn actual combined frame size.
    ok, probe = cap.read()
    if not ok:
        print("Error: Failed to read probe frame from feed")
        cap.release()
        return
    full_h, full_w = probe.shape[:2]
    half_w = full_w // 2
    half_h = full_h // 2
    print(f"Feed combined resolution: {full_w}x{full_h}")
    print(f"Each quadrant (per camera): {half_w}x{half_h}")
    print("Splice: bottom-left quadrant = LEFT camera, "
          "bottom-right quadrant = RIGHT camera")
    print("Press 'c' to capture a pair (chessboard must be visible in BOTH quadrants).")
    print("Press 'q' or Escape to quit.")
    print(f"Pairs will be saved to {left_dir} / {right_dir}")

    counter = 0
    flash_until = 0.0  # timestamp until which a "saved!" outline is shown

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.005)
            continue

        # Splice. .copy() because findChessboardCorners may otherwise hold a
        # reference into a buffer that gets overwritten on the next .read().
        l_frame = frame[half_h:, :half_w].copy()
        r_frame = frame[half_h:, half_w:].copy()

        l_gray = cv2.cvtColor(l_frame, cv2.COLOR_BGR2GRAY)
        r_gray = cv2.cvtColor(r_frame, cv2.COLOR_BGR2GRAY)

        l_found, _ = cv2.findChessboardCorners(l_gray, CHESSBOARD_SIZE, None)
        r_found, _ = cv2.findChessboardCorners(r_gray, CHESSBOARD_SIZE, None)

        # Display panes (annotated copies).
        l_disp = l_frame.copy()
        r_disp = r_frame.copy()

        # Detection state overlay per side.
        l_color = (0, 255, 0) if l_found else (0, 0, 255)
        r_color = (0, 255, 0) if r_found else (0, 0, 255)
        cv2.putText(l_disp, "L: " + ("DETECTED" if l_found else "no board"),
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, l_color, 2)
        cv2.putText(r_disp, "R: " + ("DETECTED" if r_found else "no board"),
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, r_color, 2)

        # Pair counter on both panes.
        for d in (l_disp, r_disp):
            cv2.putText(d, f"Pairs: {counter}",
                        (20, d.shape[0] - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # "Saved!" green border flash for 0.5 s after a successful capture.
        if time.time() < flash_until:
            for d in (l_disp, r_disp):
                h, w = d.shape[:2]
                cv2.rectangle(d, (0, 0), (w - 1, h - 1), (0, 255, 0), 8)

        combined = cv2.hconcat([l_disp, r_disp])
        if combined.shape[1] > 1920:
            scale = 1920.0 / combined.shape[1]
            combined = cv2.resize(combined, None, fx=scale, fy=scale)
        cv2.imshow('Stereo Capture (left | right) - spliced from CCTV feed',
                   combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            print("Exiting...")
            break
        elif key == ord('c'):
            if REQUIRE_BOTH_DETECTED and not (l_found and r_found):
                print(f"  Skipped: chessboard not visible in "
                      f"{'bottom-left' if not l_found else 'bottom-right'} quadrant.")
                continue
            l_name = os.path.join(left_dir,  f"pair_{counter:02d}.jpg")
            r_name = os.path.join(right_dir, f"pair_{counter:02d}.jpg")
            cv2.imwrite(l_name, l_frame)
            cv2.imwrite(r_name, r_frame)
            print(f"Captured pair {counter:02d} -> {l_name}, {r_name}")
            counter += 1
            flash_until = time.time() + 0.5

    cap.release()
    cv2.destroyAllWindows()
    print(f"Captured {counter} stereo pair(s).")
    if counter < 10:
        print("Recommend 15+ pairs for a stable stereo calibration. "
              "Vary chessboard angle, distance, and position across the frame.")


if __name__ == "__main__":
    capture_stereo_images()
