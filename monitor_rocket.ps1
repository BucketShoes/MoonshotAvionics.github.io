# Auto-retry serial monitor for rocket (COM13)
# Waits up to 10 seconds for port to appear, then launches monitor.
# Exits cleanly when user closes it (no orphaned processes).

param([int]$MaxRetries = 20, [int]$RetryDelayMs = 500)

$port = "COM13"
$maxWaitMs = $MaxRetries * $RetryDelayMs
$elapsed = 0

Write-Host "Waiting for $port to become available (max ${maxWaitMs}ms)..."
while ($elapsed -lt $maxWaitMs) {
    if (Get-CimInstance -ClassName Win32_SerialPort -Filter "Name = '$port'" -ErrorAction SilentlyContinue) {
        Write-Host "$port found. Starting monitor..."
        & "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe" device monitor -e rocket
        exit $LASTEXITCODE
    }
    Start-Sleep -Milliseconds $RetryDelayMs
    $elapsed += $RetryDelayMs
}

Write-Warning "$port not found after ${maxWaitMs}ms. Continuing anyway..."
& "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe" device monitor -e rocket
exit $LASTEXITCODE
