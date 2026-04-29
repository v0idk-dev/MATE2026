#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# dmg-bg.sh  –  MATE 2026 Robot Controller DMG background generator
# Usage:  ./dmg-bg.sh <version>  [output_path]
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

VERSION="${1:-}"
OUTPUT="${2:-build/background.png}"

if [[ -z "$VERSION" ]]; then
  echo "Usage: $0 <version> [output_path]"
  echo "  e.g. $0 1.0.0"
  echo "  e.g. $0 1.0.0 build/background.png"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$SCRIPT_DIR/app/Resources/python-runtime/bin/python3"
FONT_REGULAR="$SCRIPT_DIR/build/SpaceMono-Regular.ttf"
FONT_BOLD="$SCRIPT_DIR/build/SpaceMono-Bold.ttf"

if [[ ! -x "$PYTHON" ]];                          then echo "✗ Python runtime not found at: $PYTHON"; exit 1; fi
if [[ ! -f "$FONT_REGULAR" || ! -f "$FONT_BOLD" ]]; then echo "✗ Space Mono fonts not found in build/"; exit 1; fi

"$PYTHON" - "$VERSION" "$OUTPUT" "$FONT_REGULAR" "$FONT_BOLD" <<'PYEOF'
import sys
from PIL import Image, ImageDraw, ImageFont, ImageCms

VERSION  = sys.argv[1]
OUTPUT   = sys.argv[2]
FONT_REG = sys.argv[3]
FONT_BLD = sys.argv[4]

# ── LABEL PILL TUNING ────────────────────────────────────────────────────────
# All values are in pixels at 1x (doubled internally for 2x retina output)

LABEL_OFFSET  = 34    # How far below icon CENTER the pill top starts (px at 1x)
                      # Negative = above center, positive = below center

LABEL_TEXT_PX = 11    # Controls pill height via line height (text size approximation)

APP_PILL_W    = 160   # Width of app icon pill (px at 1x)
                      # "MATE 2026 Robot Controller" ≈ 160px at 12pt system font
APPS_PILL_W   = 100    # Width of Applications pill (px at 1x)
                      # "Applications" ≈ 75px at 12pt system font

APP_LINES     = 1     # Lines of text in app pill (affects pill height)
APPS_LINES    = 1     # Lines of text in Applications pill
# ─────────────────────────────────────────────────────────────────────────────

S = 2
W = 540 * S
H = 380 * S

IX, IY = 130*S, 130*S
AX, AY = 410*S, 130*S

LINE_H = int(LABEL_TEXT_PX * 1.4 * S)

font_bold  = ImageFont.truetype(FONT_BLD, 13*S)
font_small = ImageFont.truetype(FONT_REG, 10*S)

img  = Image.new("RGB", (W, H), (6, 12, 24))
draw = ImageDraw.Draw(img)

# Gradient
for y in range(H):
    t = y / H
    draw.line([(0, y), (W, y)], fill=(int(6+4*t), int(12+8*t), int(24+20*t)))

# Grid
for x in range(0, W, 30*S):
    draw.line([(x, 0), (x, H)], fill=(18, 36, 65), width=1)
for y in range(0, H, 30*S):
    draw.line([(0, y), (W, y)], fill=(18, 36, 65), width=1)

# Glow rings – app icon
for r in range(80*S, 0, -2):
    v = int((1 - r/(80*S)) * 35)
    draw.ellipse([IX-r, IY-r, IX+r, IY+r], outline=(0, 80+v*2, 150+v))
draw.ellipse([IX-48*S, IY-48*S, IX+48*S, IY+48*S], outline=(0, 160, 220), width=S)
draw.ellipse([IX-52*S, IY-52*S, IX+52*S, IY+52*S], outline=(0,  80, 130), width=S)

# Glow rings – applications folder
for r in range(60*S, 0, -2):
    v = int((1 - r/(60*S)) * 30)
    draw.ellipse([AX-r, AY-r, AX+r, AY+r], outline=(0, 60+v, 120+v))

# Arrow — tip stops just outside applications glow
ARROW_START = 185*S
ARROW_END   = 338*S
for x in range(ARROW_START, ARROW_END, 10*S):
    draw.line([(x, IY), (x+6*S, IY)], fill=(0, 180, 220), width=S)
draw.polygon([(ARROW_END, IY-7*S), (ARROW_END+16*S, IY), (ARROW_END, IY+7*S)], fill=(0, 200, 240))

# Pill label holders
def draw_pill(cx, icon_cy, offset_px, width_px, n_lines):
    w       = width_px * S
    pill_h  = LINE_H * n_lines + 10*S
    top     = icon_cy + offset_px*S
    x0, y0  = cx - w//2, top
    x1, y1  = cx + w//2, top + pill_h
    radius  = min(pill_h//2, 12*S)
    draw.rounded_rectangle([x0, y0, x1, y1], radius=radius,
                            fill=(0, 80, 140), outline=(0, 20, 48), width=S+2)

draw_pill(IX, IY, LABEL_OFFSET, APP_PILL_W,  APP_LINES)
draw_pill(AX, AY, LABEL_OFFSET, APPS_PILL_W, APPS_LINES)

# Drag hint
draw.text((W//2, 295*S), "drag to Applications to install",
          font=font_small, fill=(35, 75, 125), anchor="mm")

# Bottom bar
draw.rectangle([0, H-44*S, W, H],         fill=(4, 10, 22))
draw.line(    [(0, H-44*S), (W, H-44*S)], fill=(0, 130, 190), width=S)
draw.text((20*S,   H-22*S), "SFS ROBOTICS",               font=font_small, fill=(0, 130, 180),   anchor="lm")
draw.text((W//2,   H-22*S), "MATE 2026 Robot Controller",  font=font_bold,  fill=(160, 210, 255), anchor="mm")
draw.text((W-20*S, H-22*S), f"v{VERSION} · 2026",          font=font_small, fill=(60, 100, 150),  anchor="rm")

srgb = ImageCms.ImageCmsProfile(ImageCms.createProfile('sRGB')).tobytes()
img.save(OUTPUT, dpi=(144, 144), icc_profile=srgb)
print(f"✓ Saved {OUTPUT}  ({W}×{H}px @ 2x)  version=v{VERSION}")
PYEOF