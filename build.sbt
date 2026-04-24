import multiarch.core.Platform

lazy val isCI = sys.env.get("CI").contains("true")
ThisBuild / packageDoc / publishArtifact := false

// Version from git tags
ThisBuild / git.useGitDescribe       := true
ThisBuild / git.uncommittedSignifier := Some("SNAPSHOT")
ThisBuild / git.gitUncommittedChanges := git.gitCurrentTags.value.isEmpty

// Used to publish snapshots to Maven Central.
val mavenCentralSnapshots = "Maven Central Snapshots" at "https://central.sonatype.com/repository/maven-snapshots"

val publishSettings = Seq(
  organization := "com.kubuszok",
  homepage := Some(url("https://github.com/kubuszok/sge-native-providers")),
  organizationHomepage := Some(url("https://kubuszok.com")),
  licenses := Seq("Apache-2.0" -> url("https://www.apache.org/licenses/LICENSE-2.0")),
  scmInfo := Some(
    ScmInfo(
      url("https://github.com/kubuszok/sge-native-providers/"),
      "scm:git:git@github.com:kubuszok/sge-native-providers.git"
    )
  ),
  startYear := Some(2026),
  developers := List(
    Developer("MateuszKubuszok", "Mateusz Kubuszok", "", url("https://github.com/MateuszKubuszok"))
  ),
  pomExtra := (
    <issueManagement>
      <system>GitHub issues</system>
      <url>https://github.com/kubuszok/sge-native-providers/issues</url>
    </issueManagement>
  ),
  publishTo := {
    if (isSnapshot.value) Some(mavenCentralSnapshots)
    else localStaging.value
  },
  publishMavenStyle := true,
  Test / publishArtifact := false,
  pomIncludeRepository := { _ =>
    false
  },
  versionScheme := Some("early-semver")
)

val noPublishSettings =
  Seq(publish / skip := true, publishArtifact := false)

// ── Shared helpers ────────────────────────────────────────────────────

// Rust cross-compilation output root
val crossDir = settingKey[File]("Root directory containing cross-compiled native artifacts")
ThisBuild / crossDir := (ThisBuild / baseDirectory).value / "native-components" / "target" / "cross"

/** Create fat JAR mappings: native/<platform-classifier>/<file> for matching files. */
def fatJarMappings(crossRoot: File, platforms: Seq[Platform], fileFilter: String => Boolean): Seq[(File, String)] =
  platforms.flatMap { p =>
    val dir = crossRoot / p.classifier
    if (dir.exists())
      sbt.IO.listFiles(dir).filter(f => f.isFile && fileFilter(f.getName)).map(f => f -> s"native/${p.classifier}/${f.getName}").toSeq
    else Seq.empty
  }

/** Create fat JAR mappings for android: reads from Cargo target/<rustTarget>/release/ */
def androidJarMappings(base: File, platforms: Seq[Platform], fileFilter: String => Boolean): Seq[(File, String)] =
  platforms.flatMap { p =>
    val dir = base / p.rustTarget / "release"
    if (dir.exists())
      sbt.IO.listFiles(dir).filter(f => f.isFile && fileFilter(f.getName)).map(f => f -> s"native/${p.classifier}/${f.getName}").toSeq
    else Seq.empty
  }

// Common provider settings
val providerSettings = Seq(
  autoScalaLibrary := false,
  crossPaths       := false,
  Compile / packageDoc / publishArtifact := false,
  Compile / packageSrc / publishArtifact := false
)

// ── Root project ──────────────────────────────────────────────────────

lazy val root = project
  .in(file("."))
  .enablePlugins(GitVersioning, GitBranchPrompt)
  .settings(publishSettings *)
  .settings(noPublishSettings *)
  .aggregate(
    `sn-provider-sge`,
    `pnm-provider-sge-desktop`,
    `pnm-provider-sge-android`,
    `sn-provider-sge-freetype`,
    `pnm-provider-sge-freetype-desktop`,
    `pnm-provider-sge-freetype-android`,
    `sn-provider-sge-physics`,
    `pnm-provider-sge-physics-desktop`,
    `pnm-provider-sge-physics-android`,
    `sn-provider-sge-physics3d`,
    `pnm-provider-sge-physics3d-desktop`,
    `pnm-provider-sge-physics3d-android`,
    `sn-provider-sge-angle`,
    `pnm-provider-sge-angle`
  )
  .settings(
    name := "sge-native-providers-root",
    commands += Command.command("ci-release") { state =>
      val extracted = Project.extract(state)
      val tags      = extracted.get(git.gitCurrentTags)
      if (tags.nonEmpty) "publishSigned" :: "sonaRelease" :: state
      else "publishSigned" :: state
    }
  )

// ── SGE core native ops (sge_native_ops + sge_audio + glfw3) ─────────
// Depends on angle transitively so pulling sn-provider-sge pulls everything.

lazy val `sn-provider-sge` = project
  .in(file("providers/sn-provider-sge"))
  .dependsOn(`sn-provider-sge-angle`)
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "sn-provider-sge",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set(
        "libsge_native_ops.a", "sge_native_ops.lib",
        "libsge_audio.a",
        "libglfw3.a",
        // Windows companion .lib stubs (merged into sge_native_ops.dll)
        "sge_audio.lib", "glfw3.lib", "glfw.lib", "EGL.lib", "GLESv2.lib",
        // libobjc stubs for Linux/Windows (for @link("objc") in Scala Native)
        "libobjc.a", "objc.lib"
      )
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )

lazy val `pnm-provider-sge-desktop` = project
  .in(file("providers/pnm-provider-sge-desktop"))
  .dependsOn(`pnm-provider-sge-angle`)
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "pnm-provider-sge-desktop",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set(
        "libsge_native_ops.dylib", "libsge_native_ops.so", "sge_native_ops.dll",
        "sge_native_ops.dll.lib",
        "libsge_audio.dylib", "libsge_audio.so", "sge_audio.dll",
        "libglfw.dylib", "libglfw.so", "glfw3.dll"
      )
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )

lazy val `pnm-provider-sge-android` = project
  .in(file("providers/pnm-provider-sge-android"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "pnm-provider-sge-android",
    Compile / packageBin / mappings ++= {
      val base = (ThisBuild / baseDirectory).value / "native-components" / "target"
      val libs = Set("libsge_native_ops.so", "libsge_audio.so")
      androidJarMappings(base, Platform.android, libs.contains)
    }
  )

// ── FreeType (sge_freetype + libfreetype) ─────────────────────────────

lazy val `sn-provider-sge-freetype` = project
  .in(file("providers/sn-provider-sge-freetype"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "sn-provider-sge-freetype",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set("libsge_freetype.a", "sge_freetype.lib", "libfreetype.a")
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )

lazy val `pnm-provider-sge-freetype-desktop` = project
  .in(file("providers/pnm-provider-sge-freetype-desktop"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "pnm-provider-sge-freetype-desktop",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set("libsge_freetype.dylib", "libsge_freetype.so", "sge_freetype.dll")
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )

lazy val `pnm-provider-sge-freetype-android` = project
  .in(file("providers/pnm-provider-sge-freetype-android"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "pnm-provider-sge-freetype-android",
    Compile / packageBin / mappings ++= {
      val base = (ThisBuild / baseDirectory).value / "native-components" / "target"
      val libs = Set("libsge_freetype.so")
      androidJarMappings(base, Platform.android, libs.contains)
    }
  )

// ── Physics (sge_physics via Rapier2D) ────────────────────────────────

lazy val `sn-provider-sge-physics` = project
  .in(file("providers/sn-provider-sge-physics"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "sn-provider-sge-physics",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set("libsge_physics.a", "sge_physics.lib")
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )

lazy val `pnm-provider-sge-physics-desktop` = project
  .in(file("providers/pnm-provider-sge-physics-desktop"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "pnm-provider-sge-physics-desktop",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set("libsge_physics.dylib", "libsge_physics.so", "sge_physics.dll")
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )

lazy val `pnm-provider-sge-physics-android` = project
  .in(file("providers/pnm-provider-sge-physics-android"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "pnm-provider-sge-physics-android",
    Compile / packageBin / mappings ++= {
      val base = (ThisBuild / baseDirectory).value / "native-components" / "target"
      val libs = Set("libsge_physics.so")
      androidJarMappings(base, Platform.android, libs.contains)
    }
  )

// ── Physics3D (sge_physics3d via Rapier3D) ───────────────────────────

lazy val `sn-provider-sge-physics3d` = project
  .in(file("providers/sn-provider-sge-physics3d"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "sn-provider-sge-physics3d",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set("libsge_physics3d.a", "sge_physics3d.lib")
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )

lazy val `pnm-provider-sge-physics3d-desktop` = project
  .in(file("providers/pnm-provider-sge-physics3d-desktop"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "pnm-provider-sge-physics3d-desktop",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set("libsge_physics3d.dylib", "libsge_physics3d.so", "sge_physics3d.dll")
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )

lazy val `pnm-provider-sge-physics3d-android` = project
  .in(file("providers/pnm-provider-sge-physics3d-android"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "pnm-provider-sge-physics3d-android",
    Compile / packageBin / mappings ++= {
      val base = (ThisBuild / baseDirectory).value / "native-components" / "target"
      val libs = Set("libsge_physics3d.so")
      androidJarMappings(base, Platform.android, libs.contains)
    }
  )

// ── ANGLE (EGL + GLESv2) ──────────────────────────────────────────────

lazy val `sn-provider-sge-angle` = project
  .in(file("providers/sn-provider-sge-angle"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "sn-provider-sge-angle",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set(
        "libEGL.dylib", "libEGL.so", "libEGL.dll",
        "libGLESv2.dylib", "libGLESv2.so", "GLESv2.dll"
      )
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )

lazy val `pnm-provider-sge-angle` = project
  .in(file("providers/pnm-provider-sge-angle"))
  .settings(publishSettings *)
  .settings(providerSettings *)
  .settings(
    name := "pnm-provider-sge-angle",
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val libs = Set(
        "libEGL.dylib", "libEGL.so", "libEGL.dll",
        "libGLESv2.dylib", "libGLESv2.so", "GLESv2.dll"
      )
      fatJarMappings(cross, Platform.desktop, libs.contains)
    }
  )
