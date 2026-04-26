#!/bin/bash
# Builds libSettingsUI.dylib + settings.node
# Run from anywhere: bash app/native/settings/build.sh
set -euo pipefail

ADDON_SRC="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="$(cd "$ADDON_SRC/../.." && pwd)"

# ── 1. Swift dylib ────────────────────────────────────────────────────────────
mkdir -p "$ADDON_SRC/build/Release"
echo "==> Swift..."
swiftc \
  -target arm64-apple-macos26.0 \
  -emit-library \
  -o "$ADDON_SRC/build/Release/libSettingsUI.dylib" \
  -module-name SettingsUI \
  -Xlinker -install_name \
  -Xlinker @rpath/libSettingsUI.dylib \
  "$ADDON_SRC/SettingsUI.swift"

# ── 2. Node addon (build in /tmp to avoid space-in-path bug) ─────────────────
# Install node-addon-api if node_modules missing (fresh clone)
if [ ! -d "$ADDON_SRC/node_modules" ]; then
  echo "==> npm install (node-addon-api)..."
  (cd "$ADDON_SRC" && npm install --ignore-scripts)
fi

TMP="$(mktemp -d)"
trap "rm -rf '$TMP'" EXIT

cp "$ADDON_SRC/settings.mm"  "$TMP/"
cp "$ADDON_SRC/package.json" "$TMP/"
cp -r "$ADDON_SRC/node_modules" "$TMP/"
# Copy dylib into tmp so the path passed to node-gyp is space-free
cp "$ADDON_SRC/build/Release/libSettingsUI.dylib" "$TMP/libSettingsUI.dylib"

# Write a binding.gyp with a space-free dylib path
cat > "$TMP/binding.gyp" <<GYPEOF
{
  "targets": [{
    "target_name": "settings",
    "sources": ["settings.mm"],
    "include_dirs": ["<!@(node -p \\"require('node-addon-api').include\\")"],
    "defines": ["NAPI_DISABLE_CPP_EXCEPTIONS"],
    "libraries": ["$TMP/libSettingsUI.dylib"],
    "xcode_settings": {
      "CLANG_CXX_LANGUAGE_STANDARD": "c++17",
      "OTHER_LDFLAGS": ["-rpath", "@loader_path"]
    }
  }]
}
GYPEOF

ELECTRON_VER=$(node -p "require('$APP_DIR/node_modules/electron/package.json').version")
echo "==> Node addon (Electron $ELECTRON_VER)..."
"$APP_DIR/node_modules/.bin/electron-rebuild" \
  --module-dir "$TMP" \
  --version "$ELECTRON_VER" \
  --which-module settings \
  --build-from-source

# ── 3. Copy + fix rpaths ──────────────────────────────────────────────────────
echo "==> Copying artifacts..."
cp "$TMP/build/Release/settings.node" "$ADDON_SRC/build/Release/settings.node"

install_name_tool \
  -change "$ADDON_SRC/build/Release/libSettingsUI.dylib" \
  "@loader_path/libSettingsUI.dylib" \
  "$ADDON_SRC/build/Release/settings.node"

echo "==> Done: $ADDON_SRC/build/Release/"
ls -lh "$ADDON_SRC/build/Release/"*.{dylib,node}
