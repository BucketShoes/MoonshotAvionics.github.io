# Build, upload, LittleFS, and monitor base_station (COM9)

Clear-Host
$pio = "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe"

Write-Host "╔════════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║                   BASE STATION (COM9)                          ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

Write-Host "Compressing web assets..." -ForegroundColor Yellow
. base_station/compress_web.ps1

Write-Host ""
Write-Host "Building base_station..." -ForegroundColor Yellow
& $pio run -e base_station
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed"; Read-Host "Press Enter to close"; exit 1 }

Write-Host ""
Write-Host "Uploading firmware to COM9..." -ForegroundColor Yellow
& $pio run -e base_station -t upload
if ($LASTEXITCODE -ne 0) { Write-Error "Upload failed"; Read-Host "Press Enter to close"; exit 1 }

Write-Host ""
Write-Host "Waiting for reboot..." -ForegroundColor Yellow
$maxWait = 10
$start = Get-Date
while ((Get-Date) - $start -lt [timespan]::FromSeconds($maxWait)) {
    if (Get-CimInstance -ClassName Win32_SerialPort -Filter ("Name='COM9'") -ErrorAction SilentlyContinue) {
        break
    }
    Start-Sleep -Milliseconds 500
}

Write-Host "Uploading LittleFS..." -ForegroundColor Yellow
& $pio run -e base_station -t uploadfs
if ($LASTEXITCODE -ne 0) { Write-Error "LittleFS upload failed"; Read-Host "Press Enter to close"; exit 1 }

Write-Host ""
Write-Host "Opening serial monitor..." -ForegroundColor Green
& $pio device monitor -e base_station
