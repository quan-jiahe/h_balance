param([string]$Keil = 'D:\keil5\core\UV4\UV4.exe', [switch]$Check)
$ErrorActionPreference = 'Stop'
$project = Join-Path $PSScriptRoot 'MDK-ARM\F407_K230_UART.uvprojx'
if (!(Test-Path -LiteralPath $Keil)) { throw "Keil missing: $Keil" }
$probes = @(Get-PnpDevice -PresentOnly | Where-Object {
    $_.InstanceId -like 'USB\VID_1A86&PID_8012&MI_00\*'
})
if ($probes.Count -ne 1) { throw "Expected one WCH CMSIS-DAP in ARM mode; found $($probes.Count)." }
if ($probes[0].Status -ne 'OK') {
    $problem = (Get-PnpDeviceProperty -InstanceId $probes[0].InstanceId -KeyName 'DEVPKEY_Device_ProblemCode').Data
    throw "WCH CMSIS-DAP driver not ready (Windows code $problem). Install WCHLinkDrv_WHQL_S.exe from the official WCH-LinkUtility package as administrator, then reconnect USB. COM17 is only the separate UART interface."
}
Write-Host 'WCH CMSIS-DAP ready. Project: F407_K230_Center_Balance; SWD 100 kHz, 512 KB Flash.'
if ($Check) { return }

# Keil downloads the AXF, so rebuild it together with the published HEX first.
& (Join-Path (Split-Path -Parent $PSScriptRoot) 'tools\build_center.ps1') -Keil $Keil
$log = Join-Path $PSScriptRoot 'MDK-ARM\Objects\CenterBalance\dap-flash.log'
Write-Host 'Flashing via CMSIS-DAP. Reset starts the original startup travel automatically.'
$download = Start-Process -FilePath $Keil -ArgumentList @(
    '-f', ('"' + $project + '"'), '-t', 'F407_K230_Center_Balance',
    '-j0', '-o', ('"' + $log + '"')
) -WindowStyle Hidden -PassThru -Wait
if (!(Test-Path -LiteralPath $log)) { throw "Keil produced no download log (exit $($download.ExitCode))." }
$messages = Get-Content -Raw -LiteralPath $log
Write-Host $messages
if ($download.ExitCode -ne 0 -or $messages -match '(?im)Flash Download failed|\bError:' -or
        $messages -notmatch '(?im)\bVerify OK\b') {
    throw "CMSIS-DAP flash failed (Keil exit $($download.ExitCode)); see $log"
}
