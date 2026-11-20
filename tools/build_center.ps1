param([string]$Keil = 'D:\keil5\core\UV4\UV4.exe')
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$project = Join-Path $repo 'stm32f407_uart_test\MDK-ARM\F407_K230_UART.uvprojx'
$log = Join-Path $repo 'stm32f407_uart_test\MDK-ARM\Objects\CenterBalance\center-build.log'
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $log) | Out-Null
$process = Start-Process -FilePath $Keil -ArgumentList @(
    '-r', ('"' + $project + '"'), '-t', 'F407_K230_Center_Balance',
    '-j0', '-o', ('"' + $log + '"')
) -WindowStyle Hidden -PassThru -Wait
Get-Content -LiteralPath $log
if ($process.ExitCode -ne 0) { throw "Keil build failed: $($process.ExitCode)" }
$built = Join-Path $repo 'stm32f407_uart_test\MDK-ARM\Objects\CenterBalance\F407_K230_Center_Balance.hex'
$firmware = Join-Path $repo 'stm32f407_uart_test\Firmware\F407_K230_Center_Balance.hex'
Copy-Item -LiteralPath $built -Destination $firmware -Force
$hash = (Get-FileHash -LiteralPath $firmware -Algorithm SHA256).Hash.ToLowerInvariant()
$sums = Join-Path $repo 'stm32f407_uart_test\Firmware\SHA256SUMS'
$lines = @(Get-Content -LiteralPath $sums | Where-Object { $_ -notmatch '  F407_K230_Center_Balance.hex$' })
$lines += "$hash  F407_K230_Center_Balance.hex"
Set-Content -LiteralPath $sums -Value $lines -Encoding ascii
Write-Host "SHA256 $hash"
