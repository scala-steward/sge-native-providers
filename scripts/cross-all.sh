#!/bin/bash
# Cross-compile native-components for all 6 desktop targets + collect artifacts.
# Must be run on macOS (native Apple targets, zig for Linux/Windows cross).
#
# Prerequisites: cargo, zig, cargo-zigbuild, cargo-xwin
#   cargo install cargo-zigbuild cargo-xwin

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NATIVE_DIR="$SCRIPT_DIR/../native-components"
CROSS_DIR="$NATIVE_DIR/target/cross"

# Profile selection:
#   SGE_RELEASE=true  → --release (opt-level=3, lto=thin) for tagged releases
#   default           → --profile ci (opt-level=1, lto=false) for fast snapshot builds
if [ "${SGE_RELEASE:-}" = "true" ]; then
  CARGO_PROFILE="--release"
  PROFILE_DIR="release"
  echo "=== Release build (optimized) ==="
else
  CARGO_PROFILE="--profile ci"
  PROFILE_DIR="ci"
  echo "=== CI build (fast, unoptimized) ==="
fi

DESKTOP_TARGETS=(
  "x86_64-apple-darwin:macos-x86_64"
  "aarch64-apple-darwin:macos-aarch64"
  "x86_64-unknown-linux-gnu:linux-x86_64"
  "aarch64-unknown-linux-gnu:linux-aarch64"
  "x86_64-pc-windows-msvc:windows-x86_64"
  "aarch64-pc-windows-msvc:windows-aarch64"
)

echo "=== Building all 6 desktop targets ==="

for entry in "${DESKTOP_TARGETS[@]}"; do
  IFS=: read -r rust_target classifier <<< "$entry"
  echo ""
  echo "--- $classifier ($rust_target) ---"

  case "$rust_target" in
    *-apple-darwin)
      cargo build $CARGO_PROFILE --target "$rust_target" --manifest-path "$NATIVE_DIR/Cargo.toml"
      ;;
    *-linux-gnu)
      cargo zigbuild $CARGO_PROFILE --target "$rust_target" --manifest-path "$NATIVE_DIR/Cargo.toml"
      ;;
    *-windows-msvc)
      # cargo-xwin provides Windows SDK headers (zig doesn't have setjmp.h etc.)
      cargo xwin build $CARGO_PROFILE --target "$rust_target" --manifest-path "$NATIVE_DIR/Cargo.toml"
      ;;
  esac
done

echo ""
echo "=== Collecting artifacts ==="

# Clean and recreate cross dir
rm -rf "$CROSS_DIR"

for entry in "${DESKTOP_TARGETS[@]}"; do
  IFS=: read -r rust_target classifier <<< "$entry"
  src_dir="$NATIVE_DIR/target/$rust_target/$PROFILE_DIR"
  dest_dir="$CROSS_DIR/$classifier"
  mkdir -p "$dest_dir"

  # Static archives (.a)
  for f in libsge_native_ops.a sge_native_ops.lib libsge_audio.a libglfw3.a; do
    [ -f "$src_dir/$f" ] && cp "$src_dir/$f" "$dest_dir/"
  done

  # Shared libraries
  for f in libsge_native_ops.dylib libsge_native_ops.so sge_native_ops.dll sge_native_ops.dll.lib \
           libsge_audio.dylib libsge_audio.so sge_audio.dll \
           libglfw.dylib libglfw.so glfw3.dll; do
    [ -f "$src_dir/$f" ] && cp "$src_dir/$f" "$dest_dir/"
  done

  # FreeType, physics, and physics3d libraries (from workspace member crates)
  for f in libsge_freetype.a libsge_freetype.dylib libsge_freetype.so sge_freetype.dll sge_freetype.dll.lib \
           libsge_physics.a libsge_physics.dylib libsge_physics.so sge_physics.dll sge_physics.dll.lib \
           libsge_physics3d.a libsge_physics3d.dylib libsge_physics3d.so sge_physics3d.dll sge_physics3d.dll.lib; do
    [ -f "$src_dir/$f" ] && cp "$src_dir/$f" "$dest_dir/"
  done

  # FreeType static archive — built by freetype-sys crate (dependency of sge-native-freetype).
  # On native macOS: uses system Homebrew freetype (copy from /opt/homebrew/opt/freetype/lib/).
  # On cross-compiled targets: freetype-sys builds from vendored source, producing libfreetype2.a
  # in target/<triple>/release/build/freetype-sys-<hash>/out/.
  if [[ "$rust_target" == *"-apple-darwin" ]] && [ -f "/opt/homebrew/opt/freetype/lib/libfreetype.a" ]; then
    cp "/opt/homebrew/opt/freetype/lib/libfreetype.a" "$dest_dir/libfreetype.a"
  else
    # Find libfreetype2.a in Cargo's build output (hash in path varies)
    ft_archive=$(find "$NATIVE_DIR/target/$rust_target/$PROFILE_DIR/build" -path "*/freetype-sys-*/out/libfreetype2.a" 2>/dev/null | head -1)
    if [ -n "$ft_archive" ]; then
      cp "$ft_archive" "$dest_dir/libfreetype.a"
    fi
  fi

  echo "  $classifier: $(ls "$dest_dir" | wc -l | tr -d ' ') files"
done

# Generate Windows companion .lib stubs
for classifier in windows-x86_64 windows-aarch64; do
  dest_dir="$CROSS_DIR/$classifier"
  SRC="$dest_dir/sge_native_ops.lib"
  if [ -f "$SRC" ]; then
    for lib in sge_audio glfw3 glfw EGL GLESv2; do
      [ ! -f "$dest_dir/${lib}.lib" ] && cp "$SRC" "$dest_dir/${lib}.lib"
    done
  fi
done

# Generate libobjc stub for Linux and Windows (needed for @link("objc") in Scala Native)
# On macOS the system libobjc is used; on Linux/Windows these stubs satisfy the linker.
for classifier in linux-x86_64 linux-aarch64; do
  dest_dir="$CROSS_DIR/$classifier"
  if [ ! -f "$dest_dir/libobjc.a" ]; then
    STUB_C=$(mktemp /tmp/objc_stub.XXXXXX.c)
    printf 'void *sel_registerName(const char *s) { return (void*)0; }\nvoid *objc_msgSend(void *self, void *sel, ...) { return (void*)0; }\nvoid *objc_getClass(const char *name) { return (void*)0; }\n' > "$STUB_C"
    # Determine cross-compiler for this target
    case "$classifier" in
      linux-x86_64)  ZIG_TARGET="x86_64-linux-gnu" ;;
      linux-aarch64) ZIG_TARGET="aarch64-linux-gnu" ;;
    esac
    STUB_O="${STUB_C%.c}.o"
    zig cc -target "$ZIG_TARGET" -c "$STUB_C" -o "$STUB_O"
    zig ar rcs "$dest_dir/libobjc.a" "$STUB_O"
    rm -f "$STUB_C" "$STUB_O"
    echo "  Generated libobjc.a stub for $classifier"
  fi
done
for classifier in windows-x86_64 windows-aarch64; do
  dest_dir="$CROSS_DIR/$classifier"
  if [ ! -f "$dest_dir/objc.lib" ]; then
    STUB_C=$(mktemp /tmp/objc_stub.XXXXXX.c)
    printf 'void *sel_registerName(const char *s) { return (void*)0; }\nvoid *objc_msgSend(void *self, void *sel, ...) { return (void*)0; }\nvoid *objc_getClass(const char *name) { return (void*)0; }\n' > "$STUB_C"
    case "$classifier" in
      windows-x86_64)  ZIG_TARGET="x86_64-windows-msvc" ;;
      windows-aarch64) ZIG_TARGET="aarch64-windows-msvc" ;;
    esac
    STUB_O="${STUB_C%.c}.obj"
    zig cc -target "$ZIG_TARGET" -c "$STUB_C" -o "$STUB_O"
    zig ar rcs "$dest_dir/objc.lib" "$STUB_O"
    rm -f "$STUB_C" "$STUB_O"
    echo "  Generated objc.lib stub for $classifier"
  fi
done

echo ""
echo "=== Done: artifacts in $CROSS_DIR ==="
