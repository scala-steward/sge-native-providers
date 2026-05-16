// kubuszok plugin (bundles: sbt-git, sbt-pgp, and more)
addSbtPlugin("com.kubuszok" % "sbt-kubuszok" % "0.1.0")

ThisBuild / libraryDependencySchemes += "org.scala-lang.modules" %% "scala-xml" % VersionScheme.Always

// multiarch-core Platform model (used in build.sbt for platform definitions)
resolvers += "Maven Central Snapshots" at "https://central.sonatype.com/repository/maven-snapshots/"
libraryDependencies += "com.kubuszok" %% "multiarch-core" % "0.1.2"
