# Windows installer for OpenRocket MIT 6.1

This recipe builds the current OpenRocket Java source and packages a Windows x64 installer with Java 17 included. The application identifies itself as MIT edition **6.1**, based on OpenRocket **24.12.RC.01**; Windows uses the package version **6.1.0**.

The source build runs on macOS or Linux. Installer creation runs on Windows, using portable Temurin and WiX tools. It uses OpenJDK `jpackage`; an install4j license is not required. The resulting installer is unsigned.

## Build the source and prepare the inputs

Requirements: Python 3.11 or newer, Java 17, and `curl`. Gradle uses the wrapper in the OpenRocket checkout. Run from the parent ActiveControl_MIT_RktTeam repository:

```sh
python3 packaging/windows/prepare-windows.py
```

Use `--source /path/to/openrocket` for another checkout. Add `--offline` only when Gradle dependencies and the three packaging downloads are already cached.

The script runs `shadowJar` with `--rerun-tasks`, so Java compilation is performed even if an older JAR exists. Generated inputs, downloads, the compilation log, and source provenance go into `clone/openrocket/build/windows-installer/`. Download versions and SHA-256 checksums are pinned in the script:

| Component | Version |
| --- | --- |
| Windows x64 JDK and bundled runtime | Eclipse Temurin 17.0.20.1+1 |
| Windows installer tools | WiX 3.14.1 |
| Windows GlueGen native library | 2.5.0, matching the fork's Java dependency |

## Create the installer on Windows

Open Windows PowerShell and run the packaging script with the Windows-visible OpenRocket source path, a **new, local** temporary work directory, and an output directory. For the Parallels VM on this machine:

```powershell
$repo = '\\Mac\Home\Developer\ActiveControl_MIT_RktTeam'
$source = "$repo\clone\openrocket-release-24.12.RC.01"
$work = Join-Path $env:TEMP ('OpenRocket-MIT-build-' + [guid]::NewGuid().ToString('N'))
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$repo\packaging\windows\package-windows.ps1" `
    -SourceRoot $source `
    -WorkDirectory $work `
    -OutputDirectory "$source\build\windows-installer\output"
```

`ExecutionPolicy Bypass` applies only to that PowerShell process. On another Windows machine, copy the prepared inputs, downloads, source icon, and packaging script to a Windows-accessible location and adjust these paths. WiX must be able to run there; the tested Windows 11 VM already provided its required .NET Framework support.

The output is `OpenRocket-MIT-6.1-Windows-x64-Setup.exe`, its `.sha256` checksum file, and `windows-build.log`. Copy the installer somewhere outside the Gradle `build` directory before cleaning the project. The temporary work directory retains the unpacked tools and application image for inspection; it can be removed after verification.

## Packaging choices

- Java 17 is bundled, so the recipient does not need to install Java separately. All runtime modules are included, and the `java` launcher is retained for compatibility with the existing MIT updater.
- The current shadow JAR contains JOGL Windows libraries but lacks the Windows GlueGen DLL. The recipe adds the matching `gluegen_rt.dll` beside the JAR and configures `java.library.path` accordingly. It does not rewrite the compiled JAR.
- The installed application is named **OpenRocket MIT**, with a separate installation directory and upgrade identity. The package requests a per-user installation and supplies Start menu and optional desktop shortcuts. It does not register `.ork` file associations.
- An `OpenRocket.exe` launcher alias and matching configuration support the existing updater's executable-name lookup. The updater itself has not been exercised by these packaging tests.
- No application Java or Arduino C++ code is changed by this recipe. The planned FC C++ integration is not included in this build.

## Verified build, 20 September 2026

The source was freshly compiled at repository revision `fa410c71e7c65bb8fedc9850a5166ca3baf37bca`; all 14 Gradle tasks executed successfully. Windows packaging succeeded, and the resulting installer passed installation, installed-JAR hash comparison, GUI launch with the bundled example rocket, GlueGen/OpenGL initialization, and uninstall checks.

Testing used Windows 11 ARM64 in Parallels, running the x64 application and runtime under emulation. Native Intel/AMD Windows testing, interactive installer page review, the full application test suite, and flight simulation validation were not performed. The automated runtime/graphics check used the packaged application image; the GUI launch used the installed application. The test installation was removed afterward, and the preexisting stock OpenRocket installation remained present.

The PowerShell packaging workflow was executed successfully. The Python preparation script records the source-build and staging steps performed for this export; it was syntax checked but has not itself been run end to end. The final installer rename/checksum addition was syntax checked and applied to this artifact after packaging.

Detailed logs, a launch screenshot, and the machine-readable manifest are in `clone/openrocket/build/windows-installer/`.
