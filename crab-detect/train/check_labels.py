import os
import math
from PIL import Image

label_dir = "dataset/labels"
image_dir = "dataset/images"

issues = []

def rotated_box_min_size(w, h, angle_deg):
    """Estimate minimum dimension of box after rotation."""
    angle = math.radians(angle_deg)
    rotated_w = abs(w * math.cos(angle)) + abs(h * math.sin(angle))
    rotated_h = abs(w * math.sin(angle)) + abs(h * math.cos(angle))
    return rotated_w, rotated_h

for split in ["train", "val"]:
    ldir = os.path.join(label_dir, split)
    idir = os.path.join(image_dir, split)

    if not os.path.exists(ldir):
        continue

    for file in sorted(os.listdir(ldir)):
        if not file.endswith(".txt"):
            continue

        label_path = os.path.join(ldir, file)
        base = os.path.splitext(file)[0]

        img_path = None
        for ext in [".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"]:
            candidate = os.path.join(idir, base + ext)
            if os.path.exists(candidate):
                img_path = candidate
                break

        if img_path is None:
            issues.append(f"NO IMAGE: {label_path}")
            continue

        try:
            img = Image.open(img_path)
            img.verify()
            img = Image.open(img_path)
            w_img, h_img = img.size
        except Exception as e:
            issues.append(f"CORRUPT IMAGE: {img_path} — {e}")
            continue

        with open(label_path) as f:
            lines = [l.strip() for l in f.readlines() if l.strip()]

        if not lines:
            continue

        for line_num, line in enumerate(lines, 1):
            parts = line.split()
            if len(parts) != 5:
                issues.append(f"WRONG COLUMNS: {label_path} line {line_num}")
                continue

            try:
                cls, x, y, w, h = map(float, parts)
            except ValueError:
                issues.append(f"NON-NUMERIC: {label_path} line {line_num}")
                continue

            # Basic checks
            if cls not in [0.0, 1.0, 2.0]:
                issues.append(f"BAD CLASS {cls}: {label_path} line {line_num}")
            if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
                issues.append(f"CENTER OUT OF RANGE: {label_path} line {line_num}")
            if not (0.0 < w <= 1.0 and 0.0 < h <= 1.0):
                issues.append(f"BAD SIZE: {label_path} line {line_num}")

            # Stricter small box check (your old script used 0.001)
            if w < 0.01 or h < 0.01:
                issues.append(f"TINY BOX (w={w:.4f}, h={h:.4f}) — will go degenerate after augmentation: {label_path} line {line_num}")

            # Check if box nearly touches image edge (scale=0.5 can push it out) (disabled due to false positives)
#           margin = 0.02
#           if (x - w/2) < margin or (x + w/2) > (1 - margin):
#               issues.append(f"BOX TOO CLOSE TO LEFT/RIGHT EDGE (x={x:.3f}, w={w:.3f}): {label_path} line {line_num}")
#           if (y - h/2) < margin or (y + h/2) > (1 - margin):
#               issues.append(f"BOX TOO CLOSE TO TOP/BOTTOM EDGE (y={y:.3f}, h={h:.3f}): {label_path} line {line_num}")

            # Check if 180deg rotation would make box degenerate
            rw, rh = rotated_box_min_size(w, h, 180)
            if rw < 0.005 or rh < 0.005:
                issues.append(f"BOX GOES DEGENERATE AFTER ROTATION (rw={rw:.4f}, rh={rh:.4f}): {label_path} line {line_num}")

            # Check aspect ratio extremes
            ratio = max(w, h) / min(w, h)
            if ratio > 20:
                issues.append(f"EXTREME ASPECT RATIO {ratio:.1f}:1: {label_path} line {line_num}")

            # Pixel size check — box too small in absolute pixels
            px_w = w * w_img
            px_h = h * h_img
            if px_w < 5 or px_h < 5:
                issues.append(f"BOX TINY IN PIXELS ({px_w:.1f}x{px_h:.1f}px): {label_path} line {line_num}")

if issues:
    print(f"\n{len(issues)} issues found:\n")
    for i in issues:
        print(" ", i)
else:
    print("No issues found.")