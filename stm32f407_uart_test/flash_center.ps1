param([string]$Programmer = 'D:\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe')
$ErrorActionPreference = 'Stop'
$firmware = Join-Path $PSScriptRoot 'Firmware\F407_K230_Center_Balance.hex'
if (!(Test-Path -LiteralPath $firmware)) { throw "Firmware missing: $firmware" }
Write-Host 'Flashing center firmware. After reset the original startup travel runs automatically.'
& $Programmer -c port=SWD freq=100 -w $firmware -v -rst
if ($LASTEXITCODE -ne 0) { throw "Flash failed: $LASTEXITCODE" }
