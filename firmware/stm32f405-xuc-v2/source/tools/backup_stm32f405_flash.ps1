param(
    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,

    [switch]$ConfirmRobotIsSafe
)

$ErrorActionPreference = 'Stop'

if (-not $ConfirmRobotIsSafe) {
    throw @'
Safety confirmation missing. Before retrying:
1. Remove all projectiles.
2. Disable/unplug friction-wheel and feeder power.
3. Lift or restrain chassis and gimbal so motion cannot injure anyone.
4. Keep an operator next to the emergency power switch.

Then pass -ConfirmRobotIsSafe. The target will be reset and halted briefly.
'@
}

$openOcdCandidates = Get-ChildItem -Path (
    Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Packages'
) -Filter 'openocd.exe' -Recurse -File -ErrorAction SilentlyContinue |
    Where-Object FullName -Match 'openocd-xpack' |
    Sort-Object FullName

if (-not $openOcdCandidates) {
    throw 'xPack OpenOCD was not found under the WinGet package directory.'
}

$openOcd = $openOcdCandidates[-1].FullName
$timestamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$resolvedOutput = [System.IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Path $resolvedOutput -Force | Out-Null

$backupPath = Join-Path $resolvedOutput "stm32f405_flash_1MiB_$timestamp.bin"
$logPath = Join-Path $resolvedOutput "stm32f405_backup_$timestamp.log"
$openOcdBackupPath = $backupPath.Replace('\', '/')

$commands = @(
    'transport select swd',
    'adapter speed 1000',
    'init',
    'reset halt',
    'flash probe 0',
    'flash info 0',
    'stm32f4x options_read 0',
    "flash read_bank 0 {$openOcdBackupPath} 0 0x100000",
    "verify_image {$openOcdBackupPath} 0x08000000 bin",
    'reset run',
    'shutdown'
)

$arguments = @(
    '-f', 'interface/cmsis-dap.cfg',
    '-f', 'target/stm32f4x.cfg'
)
foreach ($command in $commands) {
    $arguments += @('-c', $command)
}

Write-Host "OPENOCD=$openOcd"
Write-Host "BACKUP_PATH=$backupPath"
Write-Host 'The target will now reset and halt briefly for a read-only backup.'

# OpenOCD writes normal status messages to stderr. Windows PowerShell 5.1 wraps
# native stderr lines as non-terminating NativeCommandError records; with the
# script-wide Stop preference that would abort before OpenOCD reaches the target.
$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
try {
    & $openOcd @arguments 2>&1 | Tee-Object -FilePath $logPath
    $openOcdExitCode = $LASTEXITCODE
}
finally {
    $ErrorActionPreference = $previousErrorActionPreference
}
if ($openOcdExitCode -ne 0) {
    throw "OpenOCD backup failed with exit code $openOcdExitCode. See $logPath"
}

$backup = Get-Item -LiteralPath $backupPath
if ($backup.Length -ne 1048576) {
    throw "Unexpected backup length $($backup.Length); expected 1048576 bytes."
}

$digest = Get-FileHash -Algorithm SHA256 -LiteralPath $backupPath
$checksumPath = "$backupPath.sha256.txt"
"$($digest.Hash.ToLowerInvariant())  $($backup.Name)" |
    Set-Content -LiteralPath $checksumPath -Encoding ascii

Write-Host 'STM32_FLASH_BACKUP=PASS'
Write-Host "SIZE=$($backup.Length)"
Write-Host "SHA256=$($digest.Hash)"
Write-Host "LOG_PATH=$logPath"
Write-Host "CHECKSUM_PATH=$checksumPath"
