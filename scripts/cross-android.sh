#!/bin/bash
# Cross-compile native-components for 3 Android ABIs.
# Requires ANDROID_NDK_HOME to be set.
#
# Android only builds sge_native_ops + sge_audio (no GLFW).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NATIVE_DIR="$SCRIPT_DIR/../native-components"

if [ -z "${ANDROID_NDK_HOME:-}" ]; then
  echo "ERROR: ANDROID_NDK_HOME not set"
  exit 1
fi

# NDK prebuilt toolchain path — use darwin-x86_64 (Rosetta, works on arm64 macs too)
NDK_PREBUILT="$ANDROID_NDK_HOME/toolchains/llvm/prebuilt"
if [ -d "$NDK_PREBUILT/darwin-x86_64" ]; then
  NDK_BIN="$NDK_PREBUILT/darwin-x86_64/bin"
elif [ -d "$NDK_PREBUILT/darwin-arm64" ]; then
  NDK_BIN="$NDK_PREBUILT/darwin-arm64/bin"
elif [ -d "$NDK_PREBUILT/linux-x86_64" ]; then
  NDK_BIN="$NDK_PREBUILT/linux-x86_64/bin"
else
  echo "ERROR: Could not find NDK prebuilt toolchain in $NDK_PREBUILT"
  exit 1
fi
export PATH="$NDK_BIN:$PATH"
echo "NDK toolchain: $NDK_BIN"

# API level 26 (matches SGE's build)
API=26

# Set linkers, CC/CXX, and AR for each Android target.
# NDK uses versioned tool names (e.g. armv7a-linux-androideabi26-clang).
export CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER="$NDK_BIN/aarch64-linux-android${API}-clang"
export CC_aarch64_linux_android="$NDK_BIN/aarch64-linux-android${API}-clang"
export CXX_aarch64_linux_android="$NDK_BIN/aarch64-linux-android${API}-clang++"
export AR_aarch64_linux_android="$NDK_BIN/llvm-ar"

export CARGO_TARGET_ARMV7_LINUX_ANDROIDEABI_LINKER="$NDK_BIN/armv7a-linux-androideabi${API}-clang"
export CC_armv7_linux_androideabi="$NDK_BIN/armv7a-linux-androideabi${API}-clang"
export CXX_armv7_linux_androideabi="$NDK_BIN/armv7a-linux-androideabi${API}-clang++"
export AR_armv7_linux_androideabi="$NDK_BIN/llvm-ar"

export CARGO_TARGET_X86_64_LINUX_ANDROID_LINKER="$NDK_BIN/x86_64-linux-android${API}-clang"
export CC_x86_64_linux_android="$NDK_BIN/x86_64-linux-android${API}-clang"
export CXX_x86_64_linux_android="$NDK_BIN/x86_64-linux-android${API}-clang++"
export AR_x86_64_linux_android="$NDK_BIN/llvm-ar"

ANDROID_TARGETS=(
  "aarch64-linux-android:android-aarch64"
  "armv7-linux-androideabi:android-armv7"
  "x86_64-linux-android:android-x86_64"
)

# Profile selection (same as cross-all.sh)
if [ "${SGE_RELEASE:-}" = "true" ]; then
  CARGO_PROFILE="--release"
  PROFILE_DIR="release"
  echo "=== Release build (optimized) ==="
else
  CARGO_PROFILE="--profile ci"
  PROFILE_DIR="ci"
  echo "=== CI build (fast, unoptimized) ==="
fi

echo "=== Building 3 Android targets ==="

for entry in "${ANDROID_TARGETS[@]}"; do
  IFS=: read -r rust_target classifier <<< "$entry"
  echo ""
  echo "--- $classifier ($rust_target) ---"

  # Android: build all workspace members (no GLFW — handled by build.rs)
  cargo build $CARGO_PROFILE --target "$rust_target" \
    --manifest-path "$NATIVE_DIR/Cargo.toml"

  # Collect
  src_dir="$NATIVE_DIR/target/$rust_target/$PROFILE_DIR"
  dest_dir="$NATIVE_DIR/target/cross/$classifier"
  mkdir -p "$dest_dir"

  for f in libsge_native_ops.so libsge_audio.so libsge_freetype.so libsge_physics.so libsge_physics3d.so; do
    [ -f "$src_dir/$f" ] && cp "$src_dir/$f" "$dest_dir/"
  done

  echo "  $classifier: $(ls "$dest_dir" | wc -l | tr -d ' ') files"
done

echo ""
echo "=== Done ==="
