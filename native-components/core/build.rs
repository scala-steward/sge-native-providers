// Build script for sge-native-ops: compiles vendored C libraries and bridges.
//
// Vendored libraries:
//   - miniaudio (public domain / MIT-0) — single-file audio library
//   - GLFW 3.4 (zlib/libpng license) — windowing, input, context creation
//
// Strategy: Rust's cdylib target only exports #[no_mangle] Rust symbols, so C
// libraries that need their own exported symbols are built as SEPARATE shared
// libraries placed alongside libsge_native_ops in the output directory.
//
// Output (all in target/release/):
//   - libsge_native_ops.{dylib,so,dll}  — Rust code (buffer ops, ETC1, transforms)
//   - libsge_audio.{dylib,so,dll}       — miniaudio + audio bridge (37 sge_audio_* functions)
//   - libglfw.{dylib,so,dll}            — GLFW windowing library (built from source)
//
// This means downstream has zero external native dependencies — everything is
// compiled from source and placed in a single directory.

fn main() {
    let out_dir = std::env::var("OUT_DIR").unwrap();
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let target_env = std::env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default();

    // Vendor directory is at the workspace root level (sibling to this crate's dir)
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let vendor_dir = std::path::Path::new(&manifest_dir)
        .parent()
        .unwrap()
        .join("vendor");
    let vendor = vendor_dir.to_str().unwrap().to_string();

    // Determine the final output directory (where libsge_native_ops will be placed).
    // Derive from OUT_DIR to handle both `cargo build` and `cargo build --target <triple>`
    // correctly. When --target is explicit (even if it matches host), cargo uses
    // target/<triple>/<profile>/, but TARGET == HOST so we can't distinguish by comparing them.
    // OUT_DIR is always correct: .../target[/<triple>]/<profile>/build/<pkg>/out
    // (TARGET/HOST are re-read locally inside the build_*_shared helpers that need them
    // to detect Windows/Linux cross-compilation.)
    let release_dir = {
        let out = std::path::Path::new(&out_dir);
        // OUT_DIR = .../target[/<triple>]/<profile>/build/<pkg>/out
        // Go up 3 levels: out -> <pkg> -> build -> <profile>
        out.parent()
            .unwrap()
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .to_str()
            .unwrap()
            .to_string()
    };

    let skip_c_libs = std::env::var("SGE_SKIP_C_LIBS").unwrap_or_default() == "1";
    let is_android = target_os == "android";

    // Build SEPARATE sge_audio.dll / glfw3.dll on EVERY platform — including
    // Windows-cross — exactly like macOS/Linux/native-Windows. The separate-DLL path
    // (build_audio_bridge_shared / build_glfw_shared) already supports Windows-cross via
    // lld-link, and both the audio bridge (SGE_AUDIO_API) and GLFW (_GLFW_BUILD_DLL) now
    // export their public symbols, so the standalone DLLs are actually usable.
    //
    // The previous Windows-cross path merged audio+GLFW *objects* into
    // sge_native_ops.dll, but a Rust cdylib only re-exports #[no_mangle] Rust symbols:
    // the C audio/glfw symbols were swallowed and exported nowhere, AND the extra C code
    // destabilised the sge_native_ops load (UnsatisfiedLinkError on Windows JVM). Keeping
    // sge_native_ops a clean Rust-only cdylib fixes both. The merged path
    // (build_audio_bridge_merged / build_glfw_merged) is retained but no longer used.
    if !skip_c_libs && !is_android {
        // Create separate shared libraries (macOS, Linux, Windows — native and cross).
        build_audio_bridge_shared(&vendor, &out_dir, &release_dir, &target_os);
        build_glfw_shared(&vendor, &out_dir, &release_dir, &target_os, &target_env);
    } else if !skip_c_libs && is_android {
        // Android only needs audio bridge (no GLFW)
        build_audio_bridge_shared(&vendor, &out_dir, &release_dir, &target_os);
    }
    link_system_libs(&target_os);

    // Copy static archives to release dir for Scala Native static linking.
    if !skip_c_libs {
        copy_static_archive(&out_dir, &release_dir, "sge_audio_bridge", "sge_audio");
        if !is_android {
            copy_static_archive(&out_dir, &release_dir, "glfw3", "glfw3");
        }
    }

    // Audio bridge is loaded separately now — no need to link into libsge_native_ops
    println!("cargo:rerun-if-changed={}/sge_audio_bridge.c", vendor);
    println!("cargo:rerun-if-changed={}/miniaudio/miniaudio.c", vendor);
    println!("cargo:rerun-if-changed={}/miniaudio/miniaudio.h", vendor);
    println!("cargo:rerun-if-changed={}/glfw/src", vendor);
    println!("cargo:rerun-if-changed={}/glfw/include", vendor);
    println!("cargo:rerun-if-changed={}/glfw_platform_stubs.c", vendor);
}

/// Compile audio bridge and link it into the main sge_native_ops cdylib (Windows cross-compilation).
/// Uses cargo_metadata(true) so cargo links the C archive into the Rust cdylib.
///
/// Retained for reference: no longer called now that Windows-cross builds a separate
/// sge_audio.dll (a Rust cdylib does not re-export the C symbols, so merging hid them).
#[allow(dead_code)]
fn build_audio_bridge_merged(vendor: &str, target_os: &str) {
    let mut build = cc::Build::new();
    build
        .file(format!("{}/sge_audio_bridge.c", vendor))
        .include(format!("{}/miniaudio", vendor))
        .include(vendor)
        .define("MA_NO_GENERATION", None)
        .warnings(false)
        .pic(true);
    build.compile("sge_audio_bridge");
    // cargo_metadata is true by default — cargo will link the archive into sge_native_ops
    // Link Windows audio system libraries
    if target_os == "windows" {
        println!("cargo:rustc-link-lib=ole32");
        println!("cargo:rustc-link-lib=user32");
        println!("cargo:rustc-link-lib=advapi32");
    }
}

/// Compile GLFW and link it into the main sge_native_ops cdylib (Windows cross-compilation).
///
/// Retained for reference: no longer called now that Windows-cross builds a separate
/// glfw3.dll (a Rust cdylib does not re-export the C symbols, so merging hid them).
#[allow(dead_code)]
fn build_glfw_merged(vendor: &str, target_os: &str, target_env: &str) {
    let glfw_src = format!("{}/glfw/src", vendor);
    let glfw_include = format!("{}/glfw/include", vendor);
    let mut build = cc::Build::new();
    build
        .include(&glfw_include)
        .include(&glfw_src)
        .warnings(false)
        .pic(true);

    // Common sources
    for f in &[
        "context.c",
        "init.c",
        "input.c",
        "monitor.c",
        "platform.c",
        "vulkan.c",
        "window.c",
        "egl_context.c",
        "osmesa_context.c",
        "null_init.c",
        "null_monitor.c",
        "null_window.c",
        "null_joystick.c",
    ] {
        build.file(format!("{}/{}", glfw_src, f));
    }

    if target_os == "windows" {
        build.define("_GLFW_WIN32", None);
        build.define("UNICODE", None);
        build.define("_UNICODE", None);
        if target_env == "gnu" {
            build.define("WINVER", Some("0x0501"));
        }
        for f in &[
            "win32_init.c",
            "win32_joystick.c",
            "win32_monitor.c",
            "win32_window.c",
            "wgl_context.c",
            "win32_module.c",
            "win32_time.c",
            "win32_thread.c",
        ] {
            build.file(format!("{}/{}", glfw_src, f));
        }
    }

    build.file(format!("{}/glfw_platform_stubs.c", vendor));
    build.compile("glfw3");

    // Link Windows system libraries
    if target_os == "windows" {
        println!("cargo:rustc-link-lib=gdi32");
        println!("cargo:rustc-link-lib=user32");
        println!("cargo:rustc-link-lib=shell32");
    }
}

/// Compile miniaudio + audio bridge as a static archive, then link it into a
/// separate shared library (libsge_audio) placed in the release directory.
fn build_audio_bridge_shared(vendor: &str, out_dir: &str, release_dir: &str, target_os: &str) {
    cc::Build::new()
        .file(format!("{}/sge_audio_bridge.c", vendor))
        .include(format!("{}/miniaudio", vendor))
        .include(vendor)
        .define("MA_NO_GENERATION", None)
        .warnings(false)
        .cargo_metadata(false)
        .pic(true) // position-independent code for shared library
        .compile("sge_audio_bridge");

    // Create shared library from the static archive
    let archive = format!("{}/libsge_audio_bridge.a", out_dir);
    let (dylib_name, link_args) = match target_os {
        "macos" | "ios" => (
            "libsge_audio.dylib",
            vec![
                "-dynamiclib".into(),
                "-Wl,-all_load".into(),
                archive.clone(),
                "-framework".into(),
                "AudioToolbox".into(),
                "-framework".into(),
                "CoreAudio".into(),
                "-framework".into(),
                "CoreFoundation".into(),
                "-install_name".into(),
                "@rpath/libsge_audio.dylib".into(),
            ],
        ),
        "windows" => {
            // When cross-compiling with cargo-xwin, drive the DLL link through
            // clang-cl (the xwin-wrapped compiler) rather than hand-rolling lld-link.
            // clang-cl picks the correct target machine and — crucially — links the
            // DLL CRT startup (_initterm, _onexit, ...) for the target, including the
            // ARM64EC CRT that bare lld-link left undefined.
            let target = std::env::var("TARGET").unwrap_or_default();
            let host = std::env::var("HOST").unwrap_or_default();
            let is_cross = target != host && !target.is_empty();
            if is_cross {
                let dll_output = format!("{}/sge_audio.dll", release_dir);
                link_windows_dll_via_clang_cl(
                    &dll_output,
                    &archive,
                    &["ole32.lib", "user32.lib", "advapi32.lib"],
                );
                eprintln!("cargo:warning=Built {}", dll_output);
                return;
            }
            (
                "sge_audio.dll",
                vec![
                    "-shared".into(),
                    "-Wl,--whole-archive".into(),
                    archive.clone(),
                    "-Wl,--no-whole-archive".into(),
                ],
            )
        }
        "android" => (
            "libsge_audio.so",
            vec![
                "-shared".into(),
                "-Wl,--whole-archive".into(),
                archive.clone(),
                "-Wl,--no-whole-archive".into(),
                "-lm".into(),
                "-llog".into(),
                "-lOpenSLES".into(),
            ],
        ),
        _ => (
            "libsge_audio.so",
            vec![
                "-shared".into(),
                "-Wl,--whole-archive".into(),
                archive.clone(),
                "-Wl,--no-whole-archive".into(),
                "-lpthread".into(),
                "-lm".into(),
            ],
        ),
    };

    let output = format!("{}/{}", release_dir, dylib_name);
    let cc_tool = cc::Build::new().get_compiler();
    // Carry the compiler's own args (notably `-arch <arch>` / `--target` for
    // cross builds). Without them the final dylib link defaults to the HOST
    // architecture: cross-compiling macos-x86_64 from an arm64 host then links
    // x86_64 objects into a non-functional ~16 KB stub dylib instead of the
    // real shared library. `.compile()` above already produced a correct
    // target-arch archive; this link step must target the same arch.
    let status = std::process::Command::new(cc_tool.path())
        .args(cc_tool.args())
        .args(&link_args)
        .arg("-o")
        .arg(&output)
        .status()
        .unwrap_or_else(|e| panic!("Failed to link {}: {}", dylib_name, e));
    assert!(status.success(), "Failed to link {}", dylib_name);
    eprintln!("cargo:warning=Built {}", output);
}

/// Compile GLFW from vendored source as a static archive, then link it into a
/// separate shared library (libglfw) placed in the release directory.
fn build_glfw_shared(
    vendor: &str,
    out_dir: &str,
    release_dir: &str,
    target_os: &str,
    target_env: &str,
) {
    let glfw_src = format!("{}/glfw/src", vendor);
    let glfw_include = format!("{}/glfw/include", vendor);

    let mut build = cc::Build::new();
    build
        .include(&glfw_include)
        .include(&glfw_src)
        .warnings(false)
        .cargo_metadata(false)
        .pic(true);

    // Common sources (all platforms)
    for f in &[
        "context.c",
        "init.c",
        "input.c",
        "monitor.c",
        "platform.c",
        "vulkan.c",
        "window.c",
        "egl_context.c",
        "osmesa_context.c",
        "null_init.c",
        "null_monitor.c",
        "null_window.c",
        "null_joystick.c",
    ] {
        build.file(format!("{}/{}", glfw_src, f));
    }

    let framework_args: Vec<String> = match target_os {
        "macos" | "ios" => {
            build.define("_GLFW_COCOA", None);
            for f in &[
                "cocoa_init.m",
                "cocoa_joystick.m",
                "cocoa_monitor.m",
                "cocoa_window.m",
                "nsgl_context.m",
            ] {
                build.file(format!("{}/{}", glfw_src, f));
            }
            for f in &["cocoa_time.c", "posix_module.c", "posix_thread.c"] {
                build.file(format!("{}/{}", glfw_src, f));
            }
            vec![
                "-framework".into(),
                "Cocoa".into(),
                "-framework".into(),
                "IOKit".into(),
                "-framework".into(),
                "CoreFoundation".into(),
                "-framework".into(),
                "CoreVideo".into(),
                "-install_name".into(),
                "@rpath/libglfw.dylib".into(),
            ]
        }
        "windows" => {
            build.define("_GLFW_WIN32", None);
            // Mark GLFW's public API as __declspec(dllexport) so the symbols are
            // actually exported from the resulting glfw3.dll. MSVC/lld-link export
            // nothing by default (unlike GNU ld), so without this the DLL links but
            // exposes no glfw* symbols and the JVM's SymbolLookup.find fails.
            build.define("_GLFW_BUILD_DLL", None);
            build.define("UNICODE", None);
            build.define("_UNICODE", None);
            if target_env == "gnu" {
                build.define("WINVER", Some("0x0501"));
            }
            for f in &[
                "win32_init.c",
                "win32_joystick.c",
                "win32_monitor.c",
                "win32_window.c",
                "wgl_context.c",
                "win32_module.c",
                "win32_time.c",
                "win32_thread.c",
            ] {
                build.file(format!("{}/{}", glfw_src, f));
            }
            vec!["-lgdi32".into(), "-luser32".into(), "-lshell32".into()]
        }
        "linux" | "freebsd" | "dragonfly" | "netbsd" | "openbsd" => {
            build.define("_GLFW_X11", None);
            build.define("_DEFAULT_SOURCE", None);
            // When cross-compiling for Linux (e.g. from macOS via zigbuild), X11
            // headers aren't in the default sysroot. Use vendored headers if available.
            let x11_include = format!("{}/x11-include", vendor);
            if std::path::Path::new(&x11_include)
                .join("X11/Xlib.h")
                .exists()
            {
                build.include(&x11_include);
            }
            for f in &[
                "x11_init.c",
                "x11_monitor.c",
                "x11_window.c",
                "xkb_unicode.c",
                "glx_context.c",
                "posix_module.c",
                "posix_time.c",
                "posix_thread.c",
                "posix_poll.c",
                "linux_joystick.c",
            ] {
                build.file(format!("{}/{}", glfw_src, f));
            }
            // When cross-compiling, system libs (-lX11 etc.) aren't available.
            // GLFW dlopen's X11 at runtime, so we can skip them and allow
            // undefined symbols in the shared library.
            let target = std::env::var("TARGET").unwrap_or_default();
            let host = std::env::var("HOST").unwrap_or_default();
            let is_cross = target != host && !target.is_empty();
            if is_cross {
                vec![
                    "-Wl,--allow-shlib-undefined".into(),
                    "-lpthread".into(),
                    "-lm".into(),
                    "-ldl".into(),
                ]
            } else {
                vec![
                    "-lX11".into(),
                    "-lpthread".into(),
                    "-lm".into(),
                    "-ldl".into(),
                ]
            }
        }
        _ => {
            eprintln!(
                "cargo:warning=GLFW: unsupported target OS '{}', using null backend only",
                target_os
            );
            vec![]
        }
    };

    // Platform stubs for native window handle functions (glfwGetCocoaWindow,
    // glfwGetX11Window, glfwGetWin32Window). Scala Native requires ALL @extern
    // symbols to resolve, even if guarded by runtime checks. The stubs use the
    // same _GLFW_COCOA/_GLFW_X11/_GLFW_WIN32 defines to only provide stubs for
    // functions that don't exist on the current platform.
    build.file(format!("{}/glfw_platform_stubs.c", vendor));

    build.compile("glfw3");

    // Create shared library from the static archive
    let archive = format!("{}/libglfw3.a", out_dir);
    let dylib_name = match target_os {
        "macos" | "ios" => "libglfw.dylib",
        // Must be glfw3.dll (not glfw.dll): the artifact-collection step
        // (scripts/cross-all.sh) and the JVM loader both expect glfw3.dll on
        // Windows. Naming it glfw.dll here would build the DLL but silently drop
        // it during collection. (Latent until Windows-cross started using this
        // separate-DLL path instead of merging into sge_native_ops.dll.)
        "windows" => "glfw3.dll",
        _ => "libglfw.so",
    };

    let shared_flag = match target_os {
        "macos" | "ios" => "-dynamiclib",
        _ => "-shared",
    };

    let whole_archive_flag = match target_os {
        "macos" | "ios" => "-Wl,-all_load",
        _ => "-Wl,--whole-archive",
    };

    let output = format!("{}/{}", release_dir, dylib_name);

    // When cross-compiling for Windows, drive the DLL link through clang-cl (the
    // xwin-wrapped compiler) so the correct target machine + DLL CRT startup are
    // linked — see link_windows_dll_via_clang_cl / the audio-bridge note.
    let target = std::env::var("TARGET").unwrap_or_default();
    let host = std::env::var("HOST").unwrap_or_default();
    let is_cross = target != host && !target.is_empty();
    if target_os == "windows" && is_cross {
        // Convert -lfoo flags to foo.lib
        let link_libs: Vec<String> = framework_args
            .iter()
            .filter(|a| a.starts_with("-l"))
            .map(|a| format!("{}.lib", &a[2..]))
            .collect();
        let link_lib_refs: Vec<&str> = link_libs.iter().map(String::as_str).collect();
        link_windows_dll_via_clang_cl(&output, &archive, &link_lib_refs);
        eprintln!("cargo:warning=Built {}", output);
        return;
    }

    let cc_tool = cc::Build::new().get_compiler();
    let mut cmd = std::process::Command::new(cc_tool.path());
    // Carry the compiler's own args (notably `-arch <arch>` for apple cross
    // builds) so the final dylib link targets the cross architecture rather
    // than the host — otherwise macos-x86_64 GLFW links into a ~16 KB stub.
    cmd.args(cc_tool.args());
    cmd.arg(shared_flag).arg(whole_archive_flag).arg(&archive);

    // On Linux, close the whole-archive group
    if target_os != "macos" && target_os != "ios" {
        cmd.arg("-Wl,--no-whole-archive");
    }

    cmd.args(&framework_args).arg("-o").arg(&output);

    let status = cmd
        .status()
        .unwrap_or_else(|e| panic!("Failed to link {}: {}", dylib_name, e));
    assert!(status.success(), "Failed to link {}", dylib_name);
    eprintln!("cargo:warning=Built {}", output);
}

/// Link system libraries needed by libsge_native_ops itself (Rust code only).
fn link_system_libs(target_os: &str) {
    // libsge_native_ops itself only needs libc (provided by the Rust toolchain).
    // The C libraries (audio, GLFW) are separate shared libraries and handle
    // their own system library dependencies.
    let _ = target_os;
}

/// Link a Windows DLL from a static archive when cross-compiling with cargo-xwin.
///
/// Drives clang-cl (the xwin-wrapped compiler `cc` returns) with `/LD`, rather than
/// invoking lld-link by hand. clang-cl selects the correct target machine from its
/// baked-in `--target=<triple>` and links the DLL CRT startup (`_initterm`,
/// `_onexit`, ...). Hand-rolled lld-link left those "EC symbol" CRT objects undefined
/// for ARM64 (whose xwin SDK import libs are ARM64EC-flavoured), so the windows-aarch64
/// DLL could not be linked.
///
/// `extra_libs` are additional system import libs (e.g. ole32.lib, gdi32.lib) passed
/// to the linker; the core CRT/kernel32/ucrt/vcruntime libs come from clang-cl + xwin.
fn link_windows_dll_via_clang_cl(output: &str, archive: &str, extra_libs: &[&str]) {
    let cc_tool = cc::Build::new().get_compiler();
    let mut cmd = std::process::Command::new(cc_tool.path());
    // Carry the compiler's own args — under cargo-xwin these include the
    // `--target=<triple>` and the `/winsysroot` (or libpaths) that make clang-cl
    // emit code for and link against the correct Windows target. Without them the
    // link would default to the host.
    cmd.args(cc_tool.args());
    // /LD = build a DLL (links the DLL CRT entry point + startup). /Fe<out> names it.
    cmd.arg("/LD").arg(format!("/Fe:{}", output));
    // Force the whole static archive in so all sge_audio_*/glfw* exports survive,
    // then add any extra system import libs. `-link` switches clang-cl to passing the
    // rest straight to lld-link.
    cmd.arg("-link")
        .arg("/force:multiple")
        .arg(format!("/wholearchive:{}", archive));
    for lib in extra_libs {
        cmd.arg(lib);
    }
    let status = cmd
        .status()
        .unwrap_or_else(|e| panic!("Failed to link {}: {}", output, e));
    assert!(status.success(), "Failed to link {}", output);
}

/// Create an lld-link Command for Windows DLL cross-linking.
///
/// MUST use a recent LLVM lld: rustup's bundled rust-lld crashes in
/// `lld::coff::ImportFile::parse()` on the ARM64 Windows SDK import libs
/// (libpath .../arm64/{kernel32,ucrt,vcruntime}.lib), so the windows-aarch64
/// DLL cannot be linked with it. Homebrew's `llvm` / `lld` formulae ship a newer
/// lld-link that parses them fine. Probe those explicit locations first, and only
/// fall back to a `lld-link` on PATH — never silently to rust-lld.
///
/// Retained for reference: the Windows-cross DLL link now goes through clang-cl
/// (link_windows_dll_via_clang_cl), which handles the CRT/machine details.
#[allow(dead_code)]
fn lld_link_command() -> std::process::Command {
    // Explicit override (CI sets SGE_LLD_LINK to an absolute lld-link path found via
    // `brew --prefix llvm`/`which`, so we never depend on guessing the prefix).
    if let Ok(p) = std::env::var("SGE_LLD_LINK") {
        if !p.is_empty() && std::path::Path::new(&p).exists() {
            return std::process::Command::new(p);
        }
    }
    // Direct `lld-link` binaries from Homebrew LLVM (preferred — newest, handles
    // arm64 import libs). Covers both the `llvm` and `lld` formulae on Apple-silicon
    // (/opt/homebrew) and Intel (/usr/local) Homebrew prefixes, plus versioned kegs.
    let mut candidates: Vec<String> = vec![
        "/opt/homebrew/opt/llvm/bin/lld-link".to_string(),
        "/opt/homebrew/opt/lld/bin/lld-link".to_string(),
        "/usr/local/opt/llvm/bin/lld-link".to_string(),
        "/usr/local/opt/lld/bin/lld-link".to_string(),
    ];
    // Versioned LLVM kegs (e.g. llvm@18) under both Homebrew Cellars.
    for prefix in ["/opt/homebrew/Cellar/llvm", "/usr/local/Cellar/llvm"] {
        if let Ok(entries) = std::fs::read_dir(prefix) {
            for e in entries.flatten() {
                let p = e.path().join("bin/lld-link");
                if p.exists() {
                    candidates.push(p.to_string_lossy().into_owned());
                }
            }
        }
    }
    for lld_link in &candidates {
        if std::path::Path::new(lld_link).exists() {
            return std::process::Command::new(lld_link);
        }
    }
    // `lld` driver invoked in COFF (`link`) flavor.
    for lld in [
        "/opt/homebrew/opt/llvm/bin/lld",
        "/opt/homebrew/opt/lld/bin/lld",
        "/usr/local/opt/llvm/bin/lld",
        "/usr/local/opt/lld/bin/lld",
    ] {
        if std::path::Path::new(lld).exists() {
            let mut cmd = std::process::Command::new(lld);
            cmd.arg("-flavor").arg("link");
            return cmd;
        }
    }
    // Last resort: lld-link on PATH (CI adds /opt/homebrew/opt/llvm/bin to PATH).
    std::process::Command::new("lld-link")
}

/// Copy a static archive from the cc::Build output directory to the Cargo
/// release directory, renaming it so that `-l<to_name>` resolves correctly
/// for Scala Native static linking.
fn copy_static_archive(out_dir: &str, release_dir: &str, from_name: &str, to_name: &str) {
    let src = format!("{}/lib{}.a", out_dir, from_name);
    let dst = format!("{}/lib{}.a", release_dir, to_name);
    match std::fs::copy(&src, &dst) {
        Ok(_) => eprintln!("cargo:warning=Copied static archive: {}", dst),
        Err(e) => eprintln!("cargo:warning=Failed to copy {} -> {}: {}", src, dst, e),
    }
}
