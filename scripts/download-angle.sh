#!/bin/bash
# Downloads pre-built ANGLE shared libraries (EGL + GLESv2) for all 6 desktop platforms.
# Source: kubuszok/sge-angle-natives GitHub releases.
# Matches the logic in SGE's sge-dev native angle cross-collect.

set -euo pipefail

ANGLE_VERSION="${1:-chromium-7151}"
ANGLE_REPO="kubuszok/sge-angle-natives"
BASE_URL="https://github.com/$ANGLE_REPO/releases/download/$ANGLE_VERSION"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CROSS_DIR="$SCRIPT_DIR/../native-components/target/cross"
CACHE_DIR="$SCRIPT_DIR/../native-components/target/angle-cache"

mkdir -p "$CACHE_DIR"

PLATFORMS="macos-aarch64 macos-x86_64 linux-x86_64 linux-aarch64 windows-x86_64 windows-aarch64"

lib_ext() {
  case "$1" in
    macos-*)   echo "dylib" ;;
    windows-*) echo "dll" ;;
    *)         echo "so" ;;
  esac
}

echo "=== Downloading ANGLE $ANGLE_VERSION ==="

for platform in $PLATFORMS; do
  archive="angle-${platform}.tar.gz"
  archive_path="$CACHE_DIR/$archive"
  dest_dir="$CROSS_DIR/$platform"
  mkdir -p "$dest_dir"

  echo "--- $platform ---"

  # Download if not cached
  if [ ! -f "$archive_path" ]; then
    url="$BASE_URL/$archive"
    echo "  Downloading $url..."
    if ! curl -fSL -o "$archive_path" "$url" 2>/dev/null; then
      echo "  WARNING: Download failed, skipping"
      continue
    fi
  fi

  # Extract and copy ANGLE libs
  ext=$(lib_ext "$platform")
  tmp_dir="$CACHE_DIR/extract-$platform"
  rm -rf "$tmp_dir" && mkdir -p "$tmp_dir"
  tar xzf "$archive_path" -C "$tmp_dir"

  # Find and copy libEGL and libGLESv2 (recursively, archives may have subdirs)
  find "$tmp_dir" -name "libEGL.$ext" -exec cp {} "$dest_dir/" \; 2>/dev/null
  find "$tmp_dir" -name "libGLESv2.$ext" -exec cp {} "$dest_dir/" \; 2>/dev/null
  # Windows: also copy import libraries (.dll.lib) and rename to match sn-provider.json
  if [ "$ext" = "dll" ]; then
    find "$tmp_dir" -name "libEGL.dll" -exec cp {} "$dest_dir/" \; 2>/dev/null
    find "$tmp_dir" -name "libGLESv2.dll" -exec cp {} "$dest_dir/" \; 2>/dev/null
    find "$tmp_dir" -name "libEGL.dll.lib" -exec cp {} "$dest_dir/EGL.lib" \; 2>/dev/null
    find "$tmp_dir" -name "libGLESv2.dll.lib" -exec cp {} "$dest_dir/GLESv2.lib" \; 2>/dev/null
  fi

  echo "  Installed: $(ls "$dest_dir"/*EGL* "$dest_dir"/*GLESv2* "$dest_dir"/*GLES* 2>/dev/null | wc -l | tr -d ' ') ANGLE libs"
done

echo ""
echo "=== ANGLE download complete ==="
