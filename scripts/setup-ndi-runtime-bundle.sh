#!/usr/bin/env bash
# Assemble $PROJECT_ROOT/.ndi-runtime-bundle/lib/ from:
#   - $HOME/.ndi-sdk/lib/libndi.so.6 (vendor SDK; written by setup-ndi-host.sh)
#   - $CONDA_PREFIX/lib/libavcodec.so.* + libavutil.so.* + libdav1d.so.* …
#     (provided by gst-libav / lcms2 / dav1d / openjpeg / speex / libva
#     conda-forge deps in [feature.lite-native.dependencies])
#
# The trick is that NDI 6.3.x dlopens libavcodec.so.61 and the alias
# libavcodec-ndi.so.61 (and similarly libavutil.so.59 / libavutil-ndi.so.59).
# conda-forge ships the actual file as libavcodec.so.61.X.Y.Z; this script
# creates the SONAME + -ndi alias symlinks pointing at it, so libndi sees
# what it expects.
#
# Adapted from experiment_collector/scripts/setup-ndi-runtime-bundle.sh,
# kept tight to one source of truth (their script's tested lib list).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CONDA_LIB_DIR="${CONDA_PREFIX:-$PROJECT_ROOT/.pixi/envs/lite-native}/lib"
NDI_LIB_DIR="$HOME/.ndi-sdk/lib"
OUT_DIR="$PROJECT_ROOT/.ndi-runtime-bundle/lib"

if [[ ! -d "$CONDA_LIB_DIR" ]]; then
    echo "ERROR: conda lib dir $CONDA_LIB_DIR does not exist." >&2
    echo "       Run 'pixi install -e lite-native' first." >&2
    exit 1
fi

if [[ ! -f "$NDI_LIB_DIR/libndi.so.6" ]]; then
    echo "ERROR: $NDI_LIB_DIR/libndi.so.6 missing." >&2
    echo "       Run 'bash scripts/setup-ndi-host.sh' first." >&2
    exit 1
fi

find_one() {
    local dir="$1" pattern="$2"
    find "$dir" -maxdepth 1 -type f -name "$pattern" 2>/dev/null | sort | tail -n 1
}

require_one() {
    local label="$1" dir="$2" pattern="$3" result
    result="$(find_one "$dir" "$pattern")"
    if [[ -z "$result" ]]; then
        echo "ERROR: could not find $label ($pattern) in $dir" >&2
        echo "       Likely missing a conda-forge dep in [feature.lite-native.dependencies]." >&2
        exit 1
    fi
    printf '%s\n' "$result"
}

copy_with_links() {
    local src="$1"; shift
    local resolved base
    resolved="$(readlink -f "$src")"
    base="$(basename "$resolved")"
    cp -a "$resolved" "$OUT_DIR/$base"
    while (($#)); do
        if [[ "$1" != "$base" ]]; then
            ln -sf "$base" "$OUT_DIR/$1"
        fi
        shift
    done
    echo "  $base  ($*)"
}

copy_optional_with_links() {
    local label="$1" dir="$2" pattern="$3"; shift 3
    local src
    src="$(find_one "$dir" "$pattern")"
    if [[ -z "$src" ]]; then
        echo "  (skipping optional $label — $pattern not in $dir)"
        return 0
    fi
    copy_with_links "$src" "$@"
}

mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR"/*

echo "Assembling NDI runtime bundle at $OUT_DIR"
echo "  libndi from $NDI_LIB_DIR"
echo "  FFmpeg + codec helpers from $CONDA_LIB_DIR"
echo

copy_with_links "$(require_one "libndi" "$NDI_LIB_DIR" 'libndi.so.6*')" libndi.so libndi.so.6

# FFmpeg 7.x — gst-libav drags this in. The -ndi.so.X aliases are NDI's
# convention; libndi looks for both bare SONAME and -ndi-suffixed variants.
copy_with_links "$(require_one "libavcodec" "$CONDA_LIB_DIR" 'libavcodec.so.*')" \
    libavcodec.so libavcodec.so.61 libavcodec-ndi.so.61
copy_with_links "$(require_one "libavformat" "$CONDA_LIB_DIR" 'libavformat.so.*')" libavformat.so
copy_with_links "$(require_one "libavutil" "$CONDA_LIB_DIR" 'libavutil.so.*')" \
    libavutil.so libavutil.so.59 libavutil-ndi.so.59
copy_with_links "$(require_one "libswresample" "$CONDA_LIB_DIR" 'libswresample.so.*')" libswresample.so
copy_with_links "$(require_one "libswscale" "$CONDA_LIB_DIR" 'libswscale.so.*')" libswscale.so

# Codec helpers libavcodec dlopens.
copy_with_links "$(require_one "libdav1d"   "$CONDA_LIB_DIR" 'libdav1d.so.*')"   libdav1d.so
copy_with_links "$(require_one "libopenjp2" "$CONDA_LIB_DIR" 'libopenjp2.so.*')" libopenjp2.so
copy_with_links "$(require_one "libspeex"   "$CONDA_LIB_DIR" 'libspeex.so.*')"   libspeex.so
copy_with_links "$(require_one "liblcms2"   "$CONDA_LIB_DIR" 'liblcms2.so.*')"   liblcms2.so

# VAAPI for hardware-accelerated decode (optional but usually present).
copy_optional_with_links "libva"         "$CONDA_LIB_DIR" 'libva.so.*'         libva.so libva.so.2
copy_optional_with_links "libva-drm"     "$CONDA_LIB_DIR" 'libva-drm.so.*'     libva-drm.so
copy_optional_with_links "libva-x11"     "$CONDA_LIB_DIR" 'libva-x11.so.*'     libva-x11.so
copy_optional_with_links "libva-glx"     "$CONDA_LIB_DIR" 'libva-glx.so.*'     libva-glx.so
copy_optional_with_links "libva-wayland" "$CONDA_LIB_DIR" 'libva-wayland.so.*' libva-wayland.so
copy_optional_with_links "libvdpau"      "$CONDA_LIB_DIR" 'libvdpau.so.*'      libvdpau.so libvdpau.so.1

echo
echo "✔ Runtime bundle ready at $OUT_DIR"
