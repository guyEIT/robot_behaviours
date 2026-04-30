#!/usr/bin/env bash
# One-shot NDI 6 SDK install. Extracts vendor/ndi/Install_NDI_SDK_v6_Linux.tar.gz,
# runs the Vizrt installer non-interactively, copies headers + libs into a
# host-stable per-user prefix at $HOME/.ndi-sdk/{include,lib}/.
#
# Why host-stable (not the pixi env): pixi-build-ros builds the bridge package
# inside an isolated build sandbox that can't see the lite-native env's
# include/lib dirs. The build sandbox CAN see $HOME/.ndi-sdk/, so installing
# there means the same SDK serves both the build (CMake find_path) and the
# runtime (bridge dlopens libndi.so.6).
#
# Usage:
#   bash scripts/setup-ndi-host.sh        # standalone, no pixi env required
#   pixi run setup-ndi                    # same script via pixi task
#
# Prerequisite:
#   Drop Install_NDI_SDK_v6_Linux.tar.gz into vendor/ndi/.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
NDI_SDK_TAR="$PROJECT_ROOT/vendor/ndi/Install_NDI_SDK_v6_Linux.tar.gz"
SDK_PREFIX="$HOME/.ndi-sdk"
LIB_DIR="$SDK_PREFIX/lib"
INCLUDE_DIR="$SDK_PREFIX/include"

echo "=== NDI SDK setup ==="
echo "Target prefix: $SDK_PREFIX"
echo

if [ ! -f "$NDI_SDK_TAR" ]; then
    echo "ERROR: NDI SDK not found at $NDI_SDK_TAR"
    echo "Download from https://ndi.video/for-developers/ndi-sdk/download/"
    echo "and place Install_NDI_SDK_v6_Linux.tar.gz in vendor/ndi/."
    exit 1
fi

WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

echo "Extracting tarball..."
tar -xzf "$NDI_SDK_TAR" -C "$WORK_DIR"

INSTALLER=$(find "$WORK_DIR" -name 'Install_NDI_SDK_v6_Linux.sh' -type f | head -1)
if [ -z "$INSTALLER" ]; then
    echo "ERROR: Install_NDI_SDK_v6_Linux.sh not found inside the archive"
    exit 1
fi
chmod +x "$INSTALLER"

echo "Running Vizrt installer non-interactively..."
( cd "$WORK_DIR" && echo "y" | "$INSTALLER" ) || true

NDI_DIR=$(find "$WORK_DIR" -maxdepth 3 -type d -name 'NDI SDK*' | head -1)
if [ -z "$NDI_DIR" ]; then
    NDI_LIB_HIT=$(find "$WORK_DIR" -maxdepth 4 -type f -name 'libndi.so*' -printf '%h\n' | head -1)
    if [ -n "$NDI_LIB_HIT" ]; then
        NDI_DIR=$(dirname "$NDI_LIB_HIT")
    fi
fi
if [ -z "$NDI_DIR" ] || [ ! -d "$NDI_DIR" ]; then
    echo "ERROR: NDI SDK directory not found after extraction. Tree:"
    find "$WORK_DIR" -maxdepth 3 -type f | head -30
    exit 1
fi
echo "Vendor SDK root: $NDI_DIR"

mkdir -p "$LIB_DIR" "$INCLUDE_DIR"

# ── Headers ──────────────────────────────────────────────────────────────────
NDI_HEADERS=$(find "$NDI_DIR" -name 'Processing.NDI.Lib.h' -type f | head -1)
if [ -z "$NDI_HEADERS" ]; then
    echo "ERROR: Processing.NDI.Lib.h not found inside the SDK"
    exit 1
fi
NDI_INC_PATH=$(dirname "$NDI_HEADERS")
echo "Copying headers from $NDI_INC_PATH → $INCLUDE_DIR"
cp -af "$NDI_INC_PATH"/. "$INCLUDE_DIR/"

# ── Libraries ────────────────────────────────────────────────────────────────
# Prefer the x86_64 build of libndi if multiple arches ship in the SDK.
NDI_LIB_HIT=$(find "$NDI_DIR" -path '*x86_64*' -name 'libndi.so*' \( -type f -o -type l \) | head -1)
if [ -z "$NDI_LIB_HIT" ]; then
    NDI_LIB_HIT=$(find "$NDI_DIR" -name 'libndi.so*' -type f | head -1)
fi
if [ -z "$NDI_LIB_HIT" ]; then
    echo "ERROR: libndi.so* not found inside the SDK"
    exit 1
fi
NDI_LIB_DIR=$(dirname "$NDI_LIB_HIT")
echo "Copying libs from $NDI_LIB_DIR → $LIB_DIR"
# -L dereferences symlinks to real files; matches experiment_collector behaviour.
cp -L "$NDI_LIB_DIR"/libndi.so* "$LIB_DIR/" 2>/dev/null || cp -a "$NDI_LIB_DIR"/libndi.so* "$LIB_DIR/"
if [ ! -e "$LIB_DIR/libndi.so" ]; then
    VERSIONED=$(ls "$LIB_DIR"/libndi.so.* 2>/dev/null | head -1)
    if [ -n "$VERSIONED" ]; then
        ln -sf "$(basename "$VERSIONED")" "$LIB_DIR/libndi.so"
    fi
fi

# ── Invalidate pixi-build-ros cache for the bridge package ──────────────────
# pixi caches builds based on input file mtimes. Touch the CMakeLists so the
# next `pixi install` actually re-runs CMake (which now finds NDISdk and
# builds ndi_av_bridge).
BRIDGE_CMAKE="$PROJECT_ROOT/providers/imaging_station/src/imaging_station_ndi_bridge/CMakeLists.txt"
if [ -f "$BRIDGE_CMAKE" ]; then
    touch "$BRIDGE_CMAKE"
    echo "Touched $BRIDGE_CMAKE to force a bridge rebuild."
fi

echo
echo "✔ NDI SDK installed:"
echo "    headers → $INCLUDE_DIR"
echo "    libs    → $LIB_DIR"

# ── Assemble project-local runtime bundle ───────────────────────────────────
# libndi 6.3.x dlopens libavcodec.so.61 / libavutil.so.59 / dav1d / openjpeg /
# speex / lcms2 / libva. Those are provided by the lite-native pixi env via
# gst-libav + sibling deps in [feature.lite-native.dependencies]. The bundle
# script copies them out of $CONDA_PREFIX/lib alongside libndi and creates
# the SONAME aliases (libavcodec-ndi.so.61 etc.) that NDI looks for.
if [ -n "${CONDA_PREFIX:-}" ]; then
    echo
    bash "$PROJECT_ROOT/scripts/setup-ndi-runtime-bundle.sh"
else
    echo
    echo "(skipping runtime bundle — no pixi env active.)"
    echo "Run 'pixi install -e lite-native' then 'pixi run setup-ndi-bundle' to assemble it."
fi

echo
echo "Next:"
echo "  pixi run lite-native-install      # builds ndi_av_bridge against the SDK"
echo "  pixi run imaging-ndi-rviz         # NDI receiver + RViz preview"
