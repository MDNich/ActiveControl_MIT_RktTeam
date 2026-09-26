param(
    [Parameter(Mandatory=$true)][int]$ProcessIdToWait,
    [Parameter(Mandatory=$true)][string]$Source,
    [Parameter(Mandatory=$true)][string]$Target,
    [string]$Launcher = '',
    [Parameter(Mandatory=$true)][string]$JavaPath,
    [Parameter(Mandatory=$true)][string]$LogFile,
    [Parameter(Mandatory=$true)][string]$StateFile,
    [Parameter(Mandatory=$true)][string]$ExpectedSha256,
    [switch]$Elevate,
    [switch]$Worker
)
$ErrorActionPreference = 'Stop'
$cancelFile = "$StateFile.cancel"
$utf8 = New-Object System.Text.UTF8Encoding($false)

function Write-UpdateLog([string]$Message) {
    try {
        Add-Content -LiteralPath $LogFile -Value "[$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff')] [windows] $Message" -Encoding UTF8
    } catch { }
}
function Write-State([string]$Value) {
    # Readers never see a partially written state.
    $temporary = "$StateFile.tmp"
    [IO.File]::WriteAllText($temporary, $Value, $utf8)
    Move-Item -LiteralPath $temporary -Destination $StateFile -Force
}
function Assert-NotCancelled {
    if (Test-Path -LiteralPath $cancelFile) { throw 'Update cancelled before application shutdown.' }
}
function Assert-Checksum([string]$File) {
    if ((Get-FileHash -LiteralPath $File -Algorithm SHA256).Hash -ne $ExpectedSha256) {
        throw "Update checksum mismatch: $File"
    }
}
function Quote-PowerShell([string]$Value) { return "'" + $Value.Replace("'", "''") + "'" }

$staged = $null
try {
    Assert-NotCancelled
    if ($Worker) {
        Write-UpdateLog "Replacement worker started. Target=$Target"
        if (-not (Test-Path -LiteralPath $Target -PathType Leaf)) { throw "Installed JAR is missing: $Target" }
        Assert-Checksum $Source
        $staged = "$Target.$([guid]::NewGuid().ToString('N')).new"
        Copy-Item -LiteralPath $Source -Destination $staged
        Assert-Checksum $staged
        Assert-NotCancelled
        Write-State 'READY'
        Write-UpdateLog 'Verified update staged; waiting for OpenRocket to exit.'
        $parent = Get-Process -Id $ProcessIdToWait -ErrorAction SilentlyContinue
        while ($parent -and -not $parent.HasExited) {
            Assert-NotCancelled
            Start-Sleep -Milliseconds 250
        }
        Assert-NotCancelled
        # Antivirus or a second app instance can briefly keep the JAR locked.
        $deadline = (Get-Date).AddSeconds(60)
        while ($true) {
            try {
                # Same-volume replacement keeps a recoverable previous version.
                [IO.File]::Replace($staged, $Target, "$Target.bak")
                $staged = $null
                break
            } catch [IO.IOException] {
                if ((Get-Date) -ge $deadline) { throw }
                Start-Sleep -Milliseconds 500
            }
        }
        Write-UpdateLog "Replacement complete. Previous version: $Target.bak"
        Write-State 'INSTALLED'
        exit 0
    }

    # This broker keeps the original user's privileges. Only replacement needs UAC
    # for protected installations; the restarted app never inherits that elevation.
    Write-UpdateLog "Starting update helper. Administrator access required=$Elevate"
    $command = '& ' + (Quote-PowerShell $PSCommandPath) + ' -Worker -ProcessIdToWait ' + $ProcessIdToWait
    foreach ($entry in @{
        Source=$Source; Target=$Target; JavaPath=$JavaPath; LogFile=$LogFile;
        StateFile=$StateFile; ExpectedSha256=$ExpectedSha256
    }.GetEnumerator()) {
        $command += ' -' + $entry.Key + ' ' + (Quote-PowerShell $entry.Value)
    }
    $encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($command))
    $start = @{
        FilePath=(Join-Path $PSHOME 'powershell.exe');
        ArgumentList="-NoProfile -NonInteractive -ExecutionPolicy Bypass -EncodedCommand $encoded";
        WindowStyle='Hidden'; PassThru=$true
    }
    if ($Elevate) { $start.Verb = 'RunAs' }
    $child = Start-Process @start
    $child.WaitForExit()
    $status = if (Test-Path -LiteralPath $StateFile) { [IO.File]::ReadAllText($StateFile) } else { '' }
    if ($status -ne 'INSTALLED') { throw "Replacement did not complete. $status" }

    Write-UpdateLog 'Restarting OpenRocket as the original user.'
    if ($Launcher -and (Test-Path -LiteralPath $Launcher -PathType Leaf)) {
        Start-Process -FilePath $Launcher -WorkingDirectory (Split-Path -Parent $Launcher)
    } else {
        # Windows Start-Process joins ArgumentList values: keep explicit filename quotes.
        Start-Process -FilePath $JavaPath -ArgumentList ('-jar "' + $Target + '"') -WorkingDirectory (Split-Path -Parent $Target)
    }
    Write-UpdateLog 'Update installed and application restarted.'
    Write-State 'COMPLETE'
    Remove-Item -LiteralPath $Source -Force -ErrorAction SilentlyContinue
} catch {
    $message = $_.Exception.Message
    Write-UpdateLog "FAILED: $message"
    try { Write-State "FAILED: $message" } catch { }
    if (-not $Worker -and -not (Test-Path -LiteralPath $cancelFile) -and
        -not (Get-Process -Id $ProcessIdToWait -ErrorAction SilentlyContinue)) {
        # The app can show preparation errors itself; post-shutdown failures need a dialog.
        Add-Type -AssemblyName System.Windows.Forms
        [Windows.Forms.MessageBox]::Show("The OpenRocket update could not finish. You can reopen the app manually.`n`n$message`n`nLog: $LogFile", 'OpenRocket MIT update') | Out-Null
    }
    exit 1
} finally {
    if ($staged -and (Test-Path -LiteralPath $staged)) {
        Remove-Item -LiteralPath $staged -Force -ErrorAction SilentlyContinue
    }
}
