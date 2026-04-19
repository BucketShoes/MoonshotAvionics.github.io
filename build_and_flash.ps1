# Master build & flash script for both boards + dual monitors
# Build and upload both in parallel, then start monitors in parallel

Clear-Host
$pio = "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe"

Write-Host "╔════════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║      Building Both Boards (Parallel)                          ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# === BUILD BOTH IN PARALLEL ===
$buildRocket = Start-Process powershell -PassThru -NoNewWindow -ArgumentList "-NoProfile", "-Command", "& '$pio' run -e rocket"
Write-Host "Building rocket..." -ForegroundColor Yellow

Write-Host "Building base_station (compress + compile)..." -ForegroundColor Yellow
$buildBase = Start-Process powershell -PassThru -NoNewWindow -ArgumentList "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ". base_station/compress_web.ps1; & '$pio' run -e base_station"

# Wait for both builds to complete
$buildRocket.WaitForExit()
if ($buildRocket.ExitCode -ne 0) { Write-Error "Rocket build failed"; exit 1 }
Write-Host "Rocket build complete." -ForegroundColor Green

$buildBase.WaitForExit()
if ($buildBase.ExitCode -ne 0) { Write-Error "Base station build failed"; exit 1 }
Write-Host "Base station build complete." -ForegroundColor Green

Write-Host ""
Write-Host "╔════════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║      Uploading Both Boards (Parallel)                         ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# === UPLOAD BOTH IN PARALLEL ===
Write-Host "Uploading rocket to COM13..." -ForegroundColor Yellow
$flashRocket = Start-Process powershell -PassThru -NoNewWindow -ArgumentList "-NoProfile", "-Command", "& '$pio' run -e rocket -t upload"

Write-Host "Uploading base_station firmware to COM9..." -ForegroundColor Yellow
$flashBase = Start-Process powershell -PassThru -NoNewWindow -ArgumentList "-NoProfile", "-Command", "& '$pio' run -e base_station -t upload"

# Wait for uploads
$flashRocket.WaitForExit()
if ($flashRocket.ExitCode -ne 0) { Write-Error "Rocket upload failed"; exit 1 }
Write-Host "Rocket upload complete." -ForegroundColor Green

$flashBase.WaitForExit()
if ($flashBase.ExitCode -ne 0) { Write-Error "Base station upload failed"; exit 1 }
Write-Host "Base station upload complete." -ForegroundColor Green

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

Write-Host "Uploading base_station LittleFS..." -ForegroundColor Yellow
& $pio run -e base_station -t uploadfs
if ($LASTEXITCODE -ne 0) { Write-Error "LittleFS upload failed"; exit 1 }
Write-Host "LittleFS upload complete." -ForegroundColor Green

Write-Host ""
Write-Host "╔════════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║           Opening Serial Monitors (Parallel)                  ║" -ForegroundColor Cyan
Write-Host "║     Close monitor windows to stop. Close VSCode to exit.      ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# === START MONITORS IN PARALLEL ===
Start-Process powershell -ArgumentList "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "$PWD/monitor_rocket.ps1" -NoNewWindow
Start-Process powershell -ArgumentList "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "$PWD/monitor_base_station.ps1" -NoNewWindow

Write-Host "Both monitors started in separate windows." -ForegroundColor Green
