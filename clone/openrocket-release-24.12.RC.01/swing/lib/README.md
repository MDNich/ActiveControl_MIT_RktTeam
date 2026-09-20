# GlueGen 2.5.0 macOS native runtime

`gluegen-rt-2.5.0-natives-macosx-universal.jar` is the unmodified JogAmp binary,
with its original manifest. The upstream `GLUEGEN-LICENSE.txt` is included alongside it and in the shadow JAR. It matches the existing JOGL /
GlueGen 2.5.0 dependencies and contains x86_64 and arm64 macOS code.

The JogAmp download server was unreachable from the build host on 2026-09-20.
This copy came from the official ImageJ update repository:
https://sites.imagej.net/Java-8/jars/macosx/gluegen-rt-2.5.0-natives-macosx-universal.jar-20250122172943

SHA-1: `7bd8081e4654e0ae653468bb729c35034887cd82`.

The upstream artifact is:
https://jogamp.org/deployment/maven/org/jogamp/gluegen/gluegen-rt/2.5.0/gluegen-rt-2.5.0-natives-macosx-universal.jar

Other desktop native classifiers are declared normally in `swing/build.gradle`.
The runtime JARs must reach the root shadow JAR; otherwise 3D initialization fails
with `Couldn't load library gluegen_rt`.
