#!/bin/bash
# Make the stereo_distance binary fully portable:
#  1. Rewrite its OpenCV load commands to @rpath/<name>.
#  2. Add an LC_RPATH so @rpath resolves to app/Resources/opencv-libs.
#  3. Recursively bundle every /opt/homebrew/... transitive dep of the
#     bundled OpenCV libs into the same opencv-libs dir.
#  4. Rewrite every lib's LC_ID_DYLIB and load commands to @rpath/<name>.
#  5. Re-apply ad-hoc signatures (install_name_tool invalidates them, and
#     the kernel SIGKILLs binaries with stale hashes at launch).
set -e

printf "\n\e[97;4;1mFixing OpenCV libraries\e[0m\n\n"

# Resolve paths relative to this script so it works from any cwd.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BINARY="${1:-$SCRIPT_DIR/scripts/task1_2/task1_2}"
LIB_DIR="$SCRIPT_DIR/app/Resources/opencv-libs"

if [ ! -f "$BINARY" ]; then
    echo "Error: binary not found: $BINARY" >&2
    exit 1
fi
if [ ! -d "$LIB_DIR" ]; then
    echo "Error: lib dir not found: $LIB_DIR" >&2
    exit 1
fi

# ── 1. Bundle transitive Homebrew deps ───────────────────────────────────────
# Walk every dylib in LIB_DIR; for any /opt/homebrew/... reference, copy that
# lib into LIB_DIR (resolving symlinks) and queue it for the next pass. Repeat
# until no new libs appear.
echo "Recursively bundling transitive Homebrew deps into $LIB_DIR..."

# Look for a missing dep by basename in the typical Homebrew lib roots.
# Prints the resolved full path on stdout, or nothing if not found.
locate_homebrew_lib() {
    local name="$1"
    local p
    # Quick checks for the common cases first
    for p in "/opt/homebrew/lib/$name" \
             "/opt/homebrew/opt/"*"/lib/$name" \
             "/opt/homebrew/Cellar/"*"/"*"/lib/$name"; do
        if [ -f "$p" ]; then
            echo "$p"
            return 0
        fi
    done
    # Fallback: deep search Cellar (handles nested layouts like gcc's
    # Cellar/gcc/<ver>/lib/gcc/current/libgcc_s.1.1.dylib).
    p=$(find /opt/homebrew/Cellar -maxdepth 7 -type f -name "$name" 2>/dev/null | head -1)
    if [ -n "$p" ] && [ -f "$p" ]; then
        echo "$p"
        return 0
    fi
    return 1
}

copied_any=1
pass=0
while [ "$copied_any" = "1" ]; do
    copied_any=0
    pass=$((pass + 1))
    rm -f /tmp/.fixopencv_copied_any
    # Snapshot file list so we don't keep re-scanning libs we add this pass
    mapfile -t libs < <(find "$LIB_DIR" -maxdepth 1 -type f -name "*.dylib")
    for lib in "${libs[@]}"; do
        # Collect every dep that needs bundling. Two cases:
        #   - "/opt/homebrew/..."  → use that path directly
        #   - "@rpath/<name>"      → resolve via locate_homebrew_lib
        # NOTE: skip the lib's own LC_ID_DYLIB (first line) — otool prints it
        # as the first dependency line on macOS but it's the lib itself.
        otool -L "$lib" 2>/dev/null \
            | tail -n +3 \
            | awk '/\/opt\/homebrew\// || /@rpath\// { print $1 }' \
            | while read -r dep; do
                depname=$(basename "$dep")
                # Already bundled?
                if [ -f "$LIB_DIR/$depname" ]; then
                    continue
                fi
                # Resolve actual path on this machine
                src=""
                case "$dep" in
                    /opt/homebrew/*)
                        if [ -e "$dep" ]; then src="$dep"; fi
                        ;;
                    @rpath/*)
                        src=$(locate_homebrew_lib "$depname" || true)
                        ;;
                esac
                if [ -z "$src" ]; then
                    echo "  ⚠ skip (not findable): $dep"
                    continue
                fi
                real=$(readlink -f "$src" 2>/dev/null || python3 -c "import os,sys; print(os.path.realpath(sys.argv[1]))" "$src")
                cp -f "$real" "$LIB_DIR/$depname"
                chmod u+w "$LIB_DIR/$depname"
                echo "  + $depname"
                echo "1" > /tmp/.fixopencv_copied_any
            done
    done
    if [ -f /tmp/.fixopencv_copied_any ]; then
        copied_any=1
        rm -f /tmp/.fixopencv_copied_any
    fi
    if [ "$pass" -gt 15 ]; then
        echo "  ⚠ bailing after 15 passes (cycle?)"
        break
    fi
done
echo "✓ Bundling complete (passes: $pass)"

# ── 2. Rewrite every bundled lib: LC_ID_DYLIB and all /opt/homebrew/ deps ────
echo
echo "Rewriting load commands on all bundled libs..."
for lib in "$LIB_DIR"/*.dylib; do
    [ -f "$lib" ] || continue
    [ -L "$lib" ] && continue
    libname=$(basename "$lib")
    install_name_tool -id "@rpath/$libname" "$lib" 2>/dev/null || true
    # Each /opt/homebrew/... dep -> @rpath/<basename>
    otool -L "$lib" 2>/dev/null \
        | awk '/\/opt\/homebrew\// { print $1 }' \
        | while read -r dep; do
            inner=$(basename "$dep")
            install_name_tool -change "$dep" "@rpath/$inner" "$lib" 2>/dev/null || true
        done
done
echo "✓ Bundled libs rewritten"

# ── 3. Rewrite the main binary ───────────────────────────────────────────────
echo
echo "Rewriting load commands on $BINARY..."
otool -L "$BINARY" \
    | awk '/\/opt\/homebrew\// { print $1 }' \
    | while read -r oldpath; do
        libname=$(basename "$oldpath")
        install_name_tool -change "$oldpath" "@rpath/$libname" "$BINARY"
    done

# Add rpaths if not already present.
add_rpath_if_missing() {
    local rp="$1"
    if otool -l "$BINARY" | awk '/LC_RPATH/{found=1} found && /path /{print; found=0}' \
        | grep -qF " $rp "; then
        return
    fi
    install_name_tool -add_rpath "$rp" "$BINARY" 2>/dev/null || true
}

# Inside the packaged .app: binary at .app/Contents/Resources/scripts/task1_2/
# and libs at .app/Contents/Resources/app/Resources/opencv-libs/
add_rpath_if_missing "@executable_path/../../app/Resources/opencv-libs"
add_rpath_if_missing "@loader_path/../../app/Resources/opencv-libs"
echo "✓ Binary rewritten"

# ── 4. Re-apply ad-hoc signatures ────────────────────────────────────────────
# install_name_tool invalidates the linker-signed signature. The kernel will
# SIGKILL the process at launch on machines where the binary's hashes don't
# match its embedded signature.
echo
echo "Re-signing modified binaries (ad-hoc)..."
codesign --force --sign - "$BINARY" 2>/dev/null || true
for lib in "$LIB_DIR"/*.dylib; do
    [ -f "$lib" ] || continue
    [ -L "$lib" ] && continue
    codesign --force --sign - "$lib" 2>/dev/null || true
done
echo "✓ Re-signed"

# ── 5. Verification ─────────────────────────────────────────────────────────
echo
remaining=$(otool -L "$BINARY" | grep -c "/opt/homebrew" || true)
echo "Binary references to /opt/homebrew: $remaining (should be 0)"
remaining_libs=0
for lib in "$LIB_DIR"/*.dylib; do
    [ -f "$lib" ] || continue
    [ -L "$lib" ] && continue
    n=$(otool -L "$lib" 2>/dev/null | grep -c "/opt/homebrew" || true)
    remaining_libs=$((remaining_libs + n))
done
echo "Bundled-lib references to /opt/homebrew: $remaining_libs (should be 0)"
echo
echo "✓ Done"
