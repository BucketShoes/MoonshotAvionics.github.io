# Master build & flash script for both boards + dual monitors
# Runs sequentially in one terminal: build → flash → monitor

Clear-Host
$pio = "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe"
$ErrorActionPreference = "Stop"

Write-Host "╔════════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║         Building & Flashing Both Boards                        ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# === BUILD ===
Write-Host "Building rocket..." -ForegroundColor Yellow
& $pio run -e rocket
if ($LASTEXITCODE -ne 0) { Write-Error "Rocket build failed"; exit 1 }

Write-Host ""
Write-Host "Building base_station..." -ForegroundColor Yellow
. base_station/compress_web.ps1
& $pio run -e base_station
if ($LASTEXITCODE -ne 0) { Write-Error "Base station build failed"; exit 1 }

Write-Host ""
Write-Host "Build complete." -ForegroundColor Green

# === FLASH ===
Write-Host ""
Write-Host "Uploading rocket to COM13..." -ForegroundColor Yellow
& $pio run -e rocket -t upload
if ($LASTEXITCODE -ne 0) { Write-Error "Rocket upload failed"; exit 1 }

Write-Host ""
Write-Host "Uploading base_station firmware to COM9..." -ForegroundColor Yellow
& $pio run -e base_station -t upload
if ($LASTEXITCODE -ne 0) { Write-Error "Base station upload failed"; exit 1 }

Write-Host ""
Write-Host "Waiting for COM9 reboot..." -ForegroundColor Yellow
$maxWait = 10
$start = Get-Date
while ((Get-Date) - $start -lt [timespan]::FromSeconds($maxWait)) {
    if (Get-CimInstance -ClassName Win32_SerialPort -Filter ("Name='COM9'") -ErrorAction SilentlyContinue) {
        break
    }
    Start-Sleep -Milliseconds 500
}

Write-Host ""
Write-Host "Uploading base_station LittleFS..." -ForegroundColor Yellow
& $pio run -e base_station -t uploadfs
if ($LASTEXITCODE -ne 0) { Write-Error "LittleFS upload failed"; exit 1 }

Write-Host ""
Write-Host "Flash complete." -ForegroundColor Green

# === MONITOR (run in background, parallel) ===
Write-Host ""
Write-Host "╔════════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║              Opening Serial Monitors (Parallel)                ║" -ForegroundColor Cyan
Write-Host "║      Close terminals to end monitors. Close VSCode to exit.    ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# Start both monitors in parallel (in the background via Start-Process)
Start-Process powershell -ArgumentList "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "$PWD/monitor_rocket.ps1" -NoNewWindow
Start-Process powershell -ArgumentList "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "$PWD/monitor_base_station.ps1" -NoNewWindow

Write-Host "Monitors started. Press Ctrl+C in their windows to close them." -ForegroundColor Green
