param([string]$Keil = 'D:\keil5\core\UV4\UV4.exe')
$ErrorActionPreference = 'Stop'
$target = 'F103_K230_Center_Balance'
$project = Join-Path $PSScriptRoot "MDK-ARM\$target.uvprojx"
$objects = Join-Path $PSScriptRoot 'MDK-ARM\Objects'
New-Item -ItemType Directory -Force -Path $objects | Out-Null
$log = Join-Path $objects 'build.log'
$process = Start-Process -FilePath $Keil -ArgumentList @(
    '-r', ('"' + $project + '"'), '-t', $target,
    '-j0', '-o', ('"' + $log + '"')
) -WindowStyle Hidden -PassThru -Wait
Get-Content -LiteralPath $log
if ($process.ExitCode -ne 0) { throw "Keil build failed: $($process.ExitCode)" }
$firmware = Join-Path $PSScriptRoot "Firmware\$target.hex"
Copy-Item -LiteralPath (Join-Path $objects "$target.hex") -Destination $firmware -Force
$hash = (Get-FileHash -LiteralPath $firmware -Algorithm SHA256).Hash.ToLowerInvariant()
Set-Content -LiteralPath (Join-Path $PSScriptRoot 'Firmware\SHA256SUMS') -Value "$hash  $target.hex" -Encoding ascii
Write-Host "SHA256 $hash"
