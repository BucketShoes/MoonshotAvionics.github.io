# flash_and_monitor_base_station.ps1
# Compress web assets, upload firmware + LittleFS to base station (COM9), then monitor.
# To change the COM port: edit $PORT here and upload_port/monitor_port in platformio.ini.

$PIO  = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
$PORT = "COM9"

# Kill any existing monitor holding this port
Get-CimInstance Win32_Process -Filter "Name='python.exe' OR Name='python3.exe'" |
    Where-Object { $_.CommandLine -match [regex]::Escape($PORT) } |
    ForEach-Object {
        Write-Host "Killing existing monitor PID $($_.ProcessId)"
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }
Start-Sleep -Milliseconds 500

Write-Host ""
Write-Host "=== BASE STATION: compressing web assets ==="
& powershell -ExecutionPolicy Bypass -File "$PSScriptRoot\base_station\compress_web.ps1"
if ($LASTEXITCODE -ne 0) {
    Write-Host "Compress FAILED (exit $LASTEXITCODE)"
    Write-Host "Press Enter to close..."
    Read-Host
    exit $LASTEXITCODE
}

Write-Host ""
Write-Host "=== BASE STATION: uploading firmware ==="
& $PIO run -e base_station -t upload
if ($LASTEXITCODE -ne 0) {
    Write-Host "Firmware upload FAILED (exit $LASTEXITCODE)"
    Write-Host "Press Enter to close..."
    Read-Host
    exit $LASTEXITCODE
}

Write-Host ""
Write-Host "=== Waiting for $PORT after firmware reset ==="
$deadline = (Get-Date).AddSeconds(15)
$found = $false
while ((Get-Date) -lt $deadline) {
    if ([System.IO.Ports.SerialPort]::GetPortNames() -contains $PORT) {
        Write-Host "$PORT is up."
        $found = $true
        break
    }
    Start-Sleep -Milliseconds 300
}
if (-not $found) { Write-Host "WARNING: $PORT did not appear, trying LittleFS anyway." }

if (true){
Write-Host ""
Write-Host "=== BASE STATION: uploading LittleFS ==="
& $PIO run -e base_station -t uploadfs
if ($LASTEXITCODE -ne 0) {
    Write-Host "LittleFS upload FAILED (exit $LASTEXITCODE), starting monitor anyway."
}
}

Write-Host ""
Write-Host "=== Waiting for $PORT after LittleFS reset ==="
$deadline = (Get-Date).AddSeconds(15)
$found = $false
while ((Get-Date) -lt $deadline) {
    if ([System.IO.Ports.SerialPort]::GetPortNames() -contains $PORT) {
        Write-Host "$PORT is up."
        $found = $true
        break
    }
    Start-Sleep -Milliseconds 300
}
if (-not $found) { Write-Host "WARNING: $PORT did not appear, trying monitor anyway." }

Write-Host ""
Clear-Host
Write-Host "=== BASE STATION: monitor on $PORT (Ctrl+C to stop) ==="
& $PIO device monitor -e base_station
