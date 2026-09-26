param([Parameter(Mandatory=$true)][string]$SourceRoot, [string[]]$Modes = @('reject','launcher','standalone'), [string]$AppImage = '')
$ErrorActionPreference = 'Stop'
$evidence = Join-Path $SourceRoot 'build\windows-update-verification'
if (-not $AppImage) {
    $properties = ConvertFrom-StringData (Get-Content (Join-Path $SourceRoot 'core\src\main\resources\build.properties') -Raw)
    $build = Join-Path $SourceRoot ('build\windows-installer\' + $properties['build.mit.version'] + '\output\build-manifest.json')
    $AppImage = (Get-Content $build -Raw | ConvertFrom-Json).app_image
}
$root = Join-Path $env:TEMP ('or-update-test-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory $root | Out-Null
$agent = Join-Path $root 'agent.jar'
Copy-Item (Join-Path $evidence 'update-restart-agent.jar') $agent
$classes = Join-Path $root 'classes'
Copy-Item (Join-Path $evidence 'classes') $classes -Recurse
$candidate = Join-Path $root 'candidate.jar'
Copy-Item (Join-Path $evidence 'candidate.jar') $candidate
$original = Join-Path $SourceRoot 'build\libs\OpenRocket-MIT-v6.3.jar'
$originalHash = (Get-FileHash $original).Hash
$candidateHash = (Get-FileHash $candidate).Hash
$oldOptions = $env:JAVA_TOOL_OPTIONS
$oldMarker = $env:OR_UPDATE_TEST_MARKER
$results = @()
Start-Transcript -Path (Join-Path $evidence 'windows-verification.log') -Force
try {
    foreach ($mode in $Modes) {
        $case = Join-Path $root $mode
        New-Item -ItemType Directory $case | Out-Null
        $image = Join-Path $case ("D'Angelo & MIT " + [char]233)
        Copy-Item -LiteralPath $AppImage -Destination $image -Recurse
        $target = Join-Path $image 'app\OpenRocket.jar'
        Copy-Item -LiteralPath $original -Destination $target -Force
        if ($mode -eq 'standalone') {
            Remove-Item -LiteralPath (Join-Path $image 'OpenRocket MIT.exe'),(Join-Path $image 'OpenRocket.exe')
        }
        $java = Join-Path $image 'runtime\bin\java.exe'
        $env:OR_UPDATE_TEST_MARKER = Join-Path $case 'restart.txt'
        $env:JAVA_TOOL_OPTIONS = "-javaagent:$agent"
        $operation = if ($mode -eq 'reject') { 'reject' } else { 'install' }
        Write-Output "Testing $mode in $image"
        $arguments = '-cp "' + $classes + ';' + $target + '" WindowsUpdateProbe "' + $candidate + '" "' + $case + '" ' + $operation
        $process = Start-Process -FilePath $java -ArgumentList $arguments -PassThru -RedirectStandardOutput (Join-Path $case 'probe.out') -RedirectStandardError (Join-Path $case 'probe.err')
        $null = $process.Handle
        if (-not $process.WaitForExit(120000)) { throw "Update harness timed out: $mode" }
        if ($process.ExitCode -ne 0) { throw "Update harness failed: $(Get-Content (Join-Path $case 'probe.err') -Raw)" }
        if ($mode -eq 'reject') {
            if (-not (Test-Path (Join-Path $case 'rejected.txt'))) { throw 'Missing checksum rejection evidence' }
            if ((Get-FileHash $target).Hash -ne $originalHash) { throw 'Rejected update changed original JAR' }
            if (Test-Path "$target.bak") { throw 'Rejected update started replacement' }
        } else {
            $deadline = (Get-Date).AddSeconds(90)
            while (-not (Test-Path $env:OR_UPDATE_TEST_MARKER) -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 500 }
            if (-not (Test-Path $env:OR_UPDATE_TEST_MARKER)) { throw "Restart did not complete: $mode" }
            $marker = Get-Content $env:OR_UPDATE_TEST_MARKER -Raw
            if ($marker -notlike 'PASS:*') { throw $marker }
            if ((Get-FileHash $target).Hash -ne $candidateHash) { throw 'Installed JAR hash differs from candidate' }
            if ((Get-FileHash "$target.bak").Hash -ne $originalHash) { throw 'Original JAR backup is incorrect' }
            $log = Get-Content (Join-Path $case 'Downloads\openrocket-mit-update.log') -Raw
            if ($log -notmatch 'Administrator access required=False') { throw 'Writable installation requested elevation' }
            Write-Output $marker
        }
        $results += @{case=$mode;result='PASS';path=$case}
        $caseEvidence = Join-Path $evidence $mode
        New-Item -ItemType Directory $caseEvidence -Force | Out-Null
        Copy-Item (Join-Path $case '*.txt'),(Join-Path $case 'probe.*'),(Join-Path $case 'restart*.log') $caseEvidence -ErrorAction SilentlyContinue
        if (Test-Path (Join-Path $case 'Downloads')) { Copy-Item (Join-Path $case 'Downloads\*.log') $caseEvidence }
    }
    $results | ConvertTo-Json | Set-Content (Join-Path $evidence 'windows-results.json') -Encoding UTF8
    Write-Output 'WINDOWS_UPDATE_VERIFICATION=PASS'
} finally {
    $env:JAVA_TOOL_OPTIONS = $oldOptions
    $env:OR_UPDATE_TEST_MARKER = $oldMarker
    Stop-Transcript
    # Keep the owned test copies and logs for inspection. Installed apps are never changed.
}
