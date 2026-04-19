# Build, upload, and monitor rocket (COM13)

Clear-Host
$pio = "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe"

Write-Host "╔════════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║                    ROCKET (COM13)                              ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

Write-Host "Building rocket..." -ForegroundColor Yellow
& $pio run -e rocket
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed"; Read-Host "Press Enter to close"; exit 1 }

Write-Host ""
Write-Host "Uploading to COM13..." -ForegroundColor Yellow
& $pio run -e rocket -t upload
if ($LASTEXITCODE -ne 0) { Write-Error "Upload failed"; Read-Host "Press Enter to close"; exit 1 }

Write-Host ""
Write-Host "Opening serial monitor..." -ForegroundColor Green
& $pio device monitor -e rocket
