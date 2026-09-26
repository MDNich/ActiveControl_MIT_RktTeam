param([Parameter(Mandatory=$true)][string]$SourceRoot)
$ErrorActionPreference = 'Stop'
$root = Join-Path $env:TEMP ('or-update-worker-test-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory $root | Out-Null
$script = Join-Path $SourceRoot 'swing\src\main\resources\updates\windows-update.ps1'
function Quote-Literal([string]$value) { return "'" + $value.Replace("'", "''") + "'" }
function Start-Encoded([string]$command) {
    $encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($command))
    return Start-Process powershell.exe -ArgumentList "-NoProfile -NonInteractive -ExecutionPolicy Bypass -EncodedCommand $encoded" -PassThru -WindowStyle Hidden
}
function Await-State([string]$file, [string]$expected) {
    $deadline = (Get-Date).AddSeconds(30)
    while ((Get-Date) -lt $deadline) {
        if ((Test-Path $file) -and ([IO.File]::ReadAllText($file)).StartsWith($expected)) { return }
        Start-Sleep -Milliseconds 100
    }
    throw "Expected $expected in $file"
}
$results = @()
foreach ($mode in @('locked','cancelled','corrupt')) {
    $case = Join-Path $root $mode
    New-Item -ItemType Directory $case | Out-Null
    $source = Join-Path $case 'source.jar'
    $target = Join-Path $case 'target.jar'
    $state = Join-Path $case 'state.txt'
    [IO.File]::WriteAllText($source, 'new version')
    [IO.File]::WriteAllText($target, 'old version')
    $digest = (Get-FileHash $source).Hash
    if ($mode -eq 'corrupt') { $digest = '0' * 64 }
    $parent = Start-Encoded 'Start-Sleep -Seconds 60'
    $held = $null
    try {
        if ($mode -eq 'locked') { $held = [IO.File]::Open($target, 'Open', 'Read', 'Read') }
        $command = '& ' + (Quote-Literal $script) + ' -Worker -ProcessIdToWait ' + $parent.Id
        foreach ($entry in @{Source=$source;Target=$target;StateFile=$state;LogFile=(Join-Path $case 'update.log');JavaPath='unused.exe';ExpectedSha256=$digest}.GetEnumerator()) {
            $command += ' -' + $entry.Key + ' ' + (Quote-Literal $entry.Value)
        }
        $worker = Start-Encoded $command
        if ($mode -eq 'corrupt') {
            Await-State $state 'FAILED:'
        } else {
            Await-State $state 'READY'
            if ($mode -eq 'cancelled') {
                [IO.File]::WriteAllText("$state.cancel", 'cancel')
                Await-State $state 'FAILED:'
            } else {
                Stop-Process -Id $parent.Id
                Start-Sleep -Seconds 2
                if ([IO.File]::ReadAllText($target) -ne 'old version') { throw 'Locked target was altered' }
                $held.Dispose(); $held = $null
                Await-State $state 'INSTALLED'
                if ([IO.File]::ReadAllText($target) -ne 'new version' -or [IO.File]::ReadAllText("$target.bak") -ne 'old version') { throw 'Replacement or backup incorrect' }
            }
        }
        if ($mode -ne 'locked' -and [IO.File]::ReadAllText($target) -ne 'old version') { throw 'Failed/cancelled update changed target' }
        if (-not $worker.WaitForExit(10000)) { throw 'Worker did not exit' }
        $results += @{case=$mode; result='PASS';path=$case}
    } finally {
        if ($held) { $held.Dispose() }
        if (-not $parent.HasExited) { Stop-Process -Id $parent.Id }
    }
}
$results | ConvertTo-Json | Set-Content (Join-Path $SourceRoot 'build\windows-update-verification\worker-results.json') -Encoding UTF8
Write-Output 'WINDOWS_UPDATE_WORKER_VERIFICATION=PASS'
