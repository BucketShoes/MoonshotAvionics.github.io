# flash_and_monitor_rocket.ps1
# Kill any pio monitor holding COM13, upload new firmware, wait for port, monitor.
# Run from repo root. Ctrl+C to stop monitoring.

$PIO  = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
$PORT = "COM13"

Write-Host "=== ROCKET: killing any existing monitor on $PORT ==="
Get-Process | Where-Object {
    try { $_.MainWindowTitle -match $PORT -or ($_.CommandLine -match $PORT -and $_.Name -match 'pio') } catch { $false }
} | Stop-Process -Force -ErrorAction SilentlyContinue

# Kill any pio.exe or python process with COM13 in its command line
Get-CimInstance Win32_Process -Filter "Name='pio.exe' OR Name='python.exe' OR Name='python3.exe'" |
    Where-Object { $_.CommandLine -match [regex]::Escape($PORT) } |
    ForEach-Object {
        Write-Host "  Killing PID $($_.ProcessId)"
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }
Start-Sleep -Milliseconds 800

Write-Host ""
Write-Host "=== ROCKET: uploading firmware ==="
& $PIO run -e rocket -t upload
if ($LASTEXITCODE -ne 0) {
    Write-Host "UPLOAD FAILED (exit $LASTEXITCODE)"
    Write-Host "Press Enter to close..."
    Read-Host
    exit $LASTEXITCODE
}

Write-Host ""
Write-Host "=== Waiting for $PORT to re-enumerate ==="
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
if (-not $found) { Write-Host "WARNING: $PORT did not appear within 15s, trying anyway." }

Write-Host ""
Write-Host "=== ROCKET: monitor on $PORT (Ctrl+C to stop) ==="
& $PIO device monitor -e rocket
