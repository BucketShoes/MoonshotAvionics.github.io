# flash_and_monitor_rocket.ps1
# Upload firmware to rocket (COM13) then monitor. Handles port re-enumeration.
# To change the COM port: edit $PORT here and upload_port/monitor_port in platformio.ini.

$PIO  = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
$PORT = "COM13"

# Kill any existing monitor holding this port (pio device monitor runs as python)
Get-CimInstance Win32_Process -Filter "Name='python.exe' OR Name='python3.exe'" |
    Where-Object { $_.CommandLine -match [regex]::Escape($PORT) } |
    ForEach-Object {
        Write-Host "Killing existing monitor PID $($_.ProcessId)"
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }
Start-Sleep -Milliseconds 500

Write-Host "=== ROCKET: uploading firmware ==="
& $PIO run -e rocket -t upload
if ($LASTEXITCODE -ne 0) {
    Write-Host "UPLOAD FAILED (exit $LASTEXITCODE)"
    Read-Host "Press Enter to close"
    exit $LASTEXITCODE
}

Write-Host "=== Waiting for $PORT ==="
$deadline = (Get-Date).AddSeconds(15)
while ((Get-Date) -lt $deadline) {
    if ([System.IO.Ports.SerialPort]::GetPortNames() -contains $PORT) { break }
    Start-Sleep -Milliseconds 200
}

[console]::Clear()
Clear-Host
Write-Host "=== ROCKET: monitoring $PORT (Ctrl+C to stop) ==="
& $PIO device monitor -e rocket
