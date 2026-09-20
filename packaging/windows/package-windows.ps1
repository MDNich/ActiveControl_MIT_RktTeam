param(
    [Parameter(Mandatory=$true)][string]$SourceRoot,
    [Parameter(Mandatory=$true)][string]$WorkDirectory,
    [Parameter(Mandatory=$true)][string]$OutputDirectory
)
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$buildRoot = Join-Path $SourceRoot 'build\windows-installer'
$downloads = Join-Path $buildRoot 'downloads'
$inputDirectory = Join-Path $buildRoot 'input'
$jdkZip = Join-Path $downloads 'OpenJDK17U-jdk_x64_windows_hotspot_17.0.20.1_1.zip'
$wixZip = Join-Path $downloads 'wix314-binaries.zip'
$jdkExpected = 'E53A79C3C3D86865BD7E787903884331068E71321714FFD44F145785AFFC7CB0'
$wixExpected = '6AC824E1642D6F7277D0ED7EA09411A508F6116BA6FAE0AA5F2C7DAA2FF43D31'
if ((Get-FileHash $jdkZip -Algorithm SHA256).Hash -ne $jdkExpected) { throw 'JDK checksum mismatch' }
if ((Get-FileHash $wixZip -Algorithm SHA256).Hash -ne $wixExpected) { throw 'WiX checksum mismatch' }
if (Test-Path $WorkDirectory) { throw "Use a new work directory: $WorkDirectory" }
New-Item -ItemType Directory -Path $WorkDirectory,$OutputDirectory -Force | Out-Null
Start-Transcript -Path (Join-Path $OutputDirectory 'windows-build.log') -Force
try {
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    Write-Output 'Extracting the Windows x64 JDK and WiX build tools.'
    [System.IO.Compression.ZipFile]::ExtractToDirectory($jdkZip, (Join-Path $WorkDirectory 'jdk'))
    [System.IO.Compression.ZipFile]::ExtractToDirectory($wixZip, (Join-Path $WorkDirectory 'wix'))
    $jdk = (Get-ChildItem (Join-Path $WorkDirectory 'jdk') -Directory | Select-Object -First 1).FullName
    $wix = Join-Path $WorkDirectory 'wix'
    $env:PATH = "$wix;$jdk\bin;$env:PATH"
    $localInput = Join-Path $WorkDirectory 'input'
    Copy-Item -LiteralPath $inputDirectory -Destination $localInput -Recurse
    $icon = Join-Path $WorkDirectory 'openrocket.ico'
    Copy-Item (Join-Path $SourceRoot 'swing\src\main\resources\pix\icon\icon-windows.ico') $icon
    & "$jdk\bin\java.exe" -version
    if ($LASTEXITCODE -ne 0) { throw 'The Windows x64 Java runtime did not start' }
    & "$wix\candle.exe" '-?'
    if ($LASTEXITCODE -ne 0) { throw 'WiX compiler did not start' }

    $runtime = Join-Path $WorkDirectory 'runtime'
    Write-Output 'Building the bundled Java 17 runtime.'
    & "$jdk\bin\jlink.exe" --module-path "$jdk\jmods" --add-modules ALL-MODULE-PATH --strip-debug --no-header-files --no-man-pages --compress=2 --output $runtime
    if ($LASTEXITCODE -ne 0) { throw 'jlink failed' }

    $imageDirectory = Join-Path $WorkDirectory 'image'
    $appName = 'OpenRocket MIT'
    $javaOptions = @(
        '-Dsun.java2d.noddraw=true',
        '-Dsun.java2d.d3d=false',
        '-Dsun.java2d.ddforcevram=true',
        '-Dsun.java2d.ddblit=false',
        '-Dswing.useflipBufferStrategy=True',
        '--add-exports=java.base/java.lang=ALL-UNNAMED',
        '--add-exports=java.desktop/sun.awt=ALL-UNNAMED',
        '--add-exports=java.desktop/sun.java2d=ALL-UNNAMED',
        '-Djava.library.path=$APPDIR\native'
    )
    $imageArgs = @('--type','app-image','--name',$appName,'--app-version','6.1.0',
        '--input',$localInput,'--main-jar','OpenRocket.jar',
        '--main-class','info.openrocket.swing.startup.OpenRocket',
        '--runtime-image',$runtime,'--icon',$icon,'--dest',$imageDirectory)
    foreach ($option in $javaOptions) { $imageArgs += @('--java-options',$option) }
    Write-Output 'Building the Windows application launcher.'
    & "$jdk\bin\jpackage.exe" @imageArgs
    if ($LASTEXITCODE -ne 0) { throw 'Application image creation failed' }

    $appImage = Join-Path $imageDirectory $appName
    # The existing MIT updater searches for this exact executable basename.
    Copy-Item (Join-Path $appImage "$appName.exe") (Join-Path $appImage 'OpenRocket.exe')
    Copy-Item (Join-Path $appImage "app\$appName.cfg") (Join-Path $appImage 'app\OpenRocket.cfg')

    $packageArgs = @('--name',$appName,'--app-version','6.1.0','--app-image',$appImage,
        '--vendor','MIT Rocket Team contributors','--description','OpenRocket MIT Edition 6.1 (OpenRocket 24.12.RC.01)',
        '--copyright','OpenRocket contributors and MIT Rocket Team contributors',
        '--license-file',(Join-Path $localInput 'LICENSE.TXT'),'--icon',$icon,
        '--win-per-user-install','--win-menu','--win-menu-group','OpenRocket MIT',
        '--win-shortcut','--win-shortcut-prompt','--win-dir-chooser',
        '--install-dir','OpenRocket MIT','--win-upgrade-uuid','a3c8caf9-967c-41d6-8efd-542171dc9e8a',
        '--dest',$OutputDirectory)
    Write-Output 'Creating the Windows installer.'
    & "$jdk\bin\jpackage.exe" --type exe @packageArgs
    if ($LASTEXITCODE -ne 0) { throw 'EXE installer creation failed' }
    $installer = Join-Path $OutputDirectory 'OpenRocket-MIT-6.1-Windows-x64-Setup.exe'
    Move-Item (Join-Path $OutputDirectory "$appName-6.1.0.exe") $installer -Force
    $digest = (Get-FileHash $installer -Algorithm SHA256).Hash.ToLowerInvariant()
    "$digest  OpenRocket-MIT-6.1-Windows-x64-Setup.exe" | Set-Content "$installer.sha256" -Encoding ASCII
    Write-Output "Application image: $appImage"
    Get-ChildItem $OutputDirectory -Filter '*.exe' | ForEach-Object { Get-FileHash $_.FullName -Algorithm SHA256 }
} finally {
    Stop-Transcript
}
