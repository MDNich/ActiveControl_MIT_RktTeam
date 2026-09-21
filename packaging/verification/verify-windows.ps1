param([Parameter(Mandatory=$true)][string]$SourceRoot, [switch]$ResumeOwnedTest)
$ErrorActionPreference='Stop'
$ProgressPreference='SilentlyContinue'
$base=Join-Path $SourceRoot 'build\windows-installer\6.2'
$evidence=Join-Path $base 'verification'
New-Item -ItemType Directory -Force $evidence | Out-Null
$manifest=Get-Content (Join-Path $base 'output\build-manifest.json') -Raw | ConvertFrom-Json
$install=Join-Path $env:LOCALAPPDATA 'OpenRocket MIT'
$registry=@('HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\*','HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\*')
$existing=@(Get-ItemProperty $registry -ErrorAction SilentlyContinue | Where-Object {$_.DisplayName -eq 'OpenRocket MIT'})
if ($ResumeOwnedTest) {
    $receipt=Get-Content (Join-Path $evidence 'test-installation.json') -Raw | ConvertFrom-Json
    if ($receipt.install -ne $install -or $receipt.jar_sha256 -ne $manifest.jar_sha256) { throw 'Disposable test ownership receipt does not match' }
} elseif ((Test-Path $install) -or $existing.Count -ne 0) { throw 'Refusing to alter an existing MIT installation during this disposable test' }
Start-Transcript -Path (Join-Path $evidence 'verification.log') -Force
try {
    if (-not $ResumeOwnedTest) {
    $installerLog=Join-Path $evidence 'install.log'
    $process=Start-Process -FilePath $manifest.installer -ArgumentList @('/quiet','/norestart','/log',('"'+$installerLog+'"')) -Wait -PassThru
    if ($process.ExitCode -ne 0) { throw "Installer exit code $($process.ExitCode)" }
    @{install=$install;jar_sha256=$manifest.jar_sha256} | ConvertTo-Json | Set-Content (Join-Path $evidence 'test-installation.json') -Encoding UTF8
    }
    $jar=Join-Path $install 'app\OpenRocket.jar'
    if ((Get-FileHash $jar -Algorithm SHA256).Hash.ToLowerInvariant() -ne $manifest.jar_sha256) { throw 'Installed JAR differs from source build' }
    $java=Join-Path $install 'runtime\bin\java.exe'
    $classes=Join-Path $SourceRoot 'build\package-verification'
    $ErrorActionPreference='Continue'
    & $java "-Djava.library.path=$install\app\native" -cp "$classes;$jar" PackagedSmokeCheck 6.2 2>&1 | Tee-Object -FilePath (Join-Path $evidence 'runtime-smoke.log')
    if ($LASTEXITCODE -ne 0) { throw 'Installed runtime smoke test failed' }
    & $java '-Xmx2g' '--add-exports=java.desktop/sun.awt=ALL-UNNAMED' '--add-exports=java.desktop/sun.java2d=ALL-UNNAMED' "-Djava.library.path=$install\app\native" -cp "$classes;$jar" TrajectoryRenderProbe $evidence 2>&1 | Tee-Object -FilePath (Join-Path $evidence 'renderer.log')
    $ErrorActionPreference='Stop'
    if ($LASTEXITCODE -ne 0) { throw 'Installed 3D renderer failed' }
    $example=Join-Path $SourceRoot 'core\src\main\resources\datafiles\examples\A simple model rocket.ork'
    $agent=Join-Path $env:TEMP 'OpenRocket-MIT-launch-smoke-agent.jar'
    Copy-Item (Join-Path $classes 'launcher-smoke-agent.jar') $agent -Force
    $oldOptions=$env:JAVA_TOOL_OPTIONS
    $env:JAVA_TOOL_OPTIONS="-javaagent:$agent=amd64"
    $stdout=Join-Path $evidence 'launcher-out.log'
    $stderr=Join-Path $evidence 'launcher-err.log'
    $app=$null
    try {
        $app=Start-Process -FilePath (Join-Path $install 'OpenRocket MIT.exe') -ArgumentList ('"'+$example+'"') -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
        # Retain the process handle so Windows PowerShell exposes ExitCode after exit.
        $null=$app.Handle
        if(-not $app.WaitForExit(90000)) { throw 'Installed launcher timed out' }
        $app.Refresh()
        $result=(Get-Content $stdout -Raw)+(Get-Content $stderr -Raw)
        if($app.ExitCode -ne 0 -or $result -notmatch 'LAUNCHER_SMOKE_CHECK=PASS') { throw "Installed launch verification failed: $result" }
        @{pid=$app.Id;main_window_observed=$true;installed_jar_matches=$true} | ConvertTo-Json | Set-Content (Join-Path $evidence 'gui-launch.json') -Encoding UTF8
    } finally {
        $env:JAVA_TOOL_OPTIONS=$oldOptions
        if($app -and -not $app.HasExited) { Stop-Process -Id $app.Id }
    }
    $product=@(Get-ItemProperty $registry -ErrorAction SilentlyContinue | Where-Object {$_.DisplayName -eq 'OpenRocket MIT' -and $_.DisplayVersion -eq '6.2.0'})
    if($product.Count -ne 1) { throw 'Expected exactly one disposable MIT test installation' }
    $uninstallLog=Join-Path $evidence 'uninstall.log'
    $uninstall=Start-Process msiexec.exe -ArgumentList @('/x',$product[0].PSChildName,'/qn','/norestart','/l*v',('"'+$uninstallLog+'"')) -Wait -PassThru
    if($uninstall.ExitCode -ne 0 -or (Test-Path $install)) { throw 'Disposable test uninstall failed' }
    @{installer_exit=0;installed_jar_matches=$true;runtime_smoke='PASS';renderer='PASS';gui_launch='PASS';uninstall_exit=$uninstall.ExitCode;test_installation_removed=$true;stock_installation_present=(Test-Path 'C:\Program Files\OpenRocket')} | ConvertTo-Json | Set-Content (Join-Path $evidence 'verification.json') -Encoding UTF8
} finally { Stop-Transcript }
