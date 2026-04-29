#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$SCRIPT_DIR/app/Resources/python-runtime/bin/python3"
VERSION=$(node -p "require('$SCRIPT_DIR/package.json').version")
DMG="$SCRIPT_DIR/dist/MATE 2026 Robot Controller-${VERSION}-arm64.dmg"
BACKGROUND="$SCRIPT_DIR/build/background.png"
VOLUME="MATE 2026 Robot Controller v${VERSION}"

# Read window/icon config from package.json
WIN_W=$(node -p "require('$SCRIPT_DIR/package.json').build.dmg.window.width")
WIN_H=$(node -p "require('$SCRIPT_DIR/package.json').build.dmg.window.height")
APP_X=$(node -p "require('$SCRIPT_DIR/package.json').build.dmg.contents[0].x")
APP_Y=$(node -p "require('$SCRIPT_DIR/package.json').build.dmg.contents[0].y")
LNK_X=$(node -p "require('$SCRIPT_DIR/package.json').build.dmg.contents[1].x")
LNK_Y=$(node -p "require('$SCRIPT_DIR/package.json').build.dmg.contents[1].y")

echo "→ Rebuilding DMG with background: $DMG"

SETTINGS=$(mktemp /tmp/dmgbuild_settings_XXXXXX.py)
cat > "$SETTINGS" << EOF
import os

application = '$SCRIPT_DIR/dist/mac-arm64/MATE 2026 Robot Controller.app'
appname = os.path.basename(application)

files = [application]
symlinks = {'Applications': '/Applications'}

icon_locations = {
    appname: ($APP_X, $APP_Y),
    'Applications': ($LNK_X, $LNK_Y),
}

background = '$BACKGROUND'

show_status_bar = False
show_tab_view = False
show_toolbar = False
show_pathbar = False
show_sidebar = False

window_rect = ((100, 100), ($WIN_W, $WIN_H - 36))

default_view = 'icon-view'
show_icon_preview = False
include_icon_view_settings = 'auto'

icon_size = 65
text_size = 10
arrange_by = None
grid_offset = (0, 0)
grid_spacing = 100
scroll_position = (0, 0)
label_pos = 'bottom'
EOF

rm -f "$DMG"
"$PYTHON" -m dmgbuild -s "$SETTINGS" "$VOLUME" "$DMG"
rm -f "$SETTINGS"

echo "✓ DMG rebuilt with background"