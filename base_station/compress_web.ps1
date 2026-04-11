# Compresses web assets from docs/ into base_station/data/ for LittleFS upload.
# Edit docs/index.html, docs/style.css, docs/app.js — those are the source of truth.
#
# After running:
#   1. Tools > ESP32 LittleFS Data Upload  (uploads data/ to the filesystem partition)
#   2. Sketch > Upload                     (only needed when .ino code changes)

$src = Join-Path (Split-Path $PSScriptRoot) "docs"
$dst = Join-Path $PSScriptRoot "data"
New-Item -ItemType Directory -Force -Path $dst | Out-Null

foreach ($f in @("index.html", "style.css", "app.js")) {
    $in  = Join-Path $src $f
    $out = Join-Path $dst "$f.gz"
    if (-not (Test-Path $in)) {
        Write-Host "SKIP $f (not found)"
        continue
    }
    $fs_in  = [System.IO.File]::OpenRead($in)
    $fs_out = [System.IO.File]::Create($out)
    $gz     = New-Object System.IO.Compression.GZipStream($fs_out, [System.IO.Compression.CompressionLevel]::Optimal)
    $fs_in.CopyTo($gz)
    $gz.Close(); $fs_out.Close(); $fs_in.Close()
    $sz_in  = (Get-Item $in).Length
    $sz_out = (Get-Item $out).Length
    $pct    = [math]::Round(100 - 100 * $sz_out / $sz_in, 1)
    Write-Host ("{0,-12} {1,7} -> {2,6} bytes  ({3}% reduction)" -f $f, $sz_in, $sz_out, $pct)
}

Write-Host ""
Write-Host "Done. Run 'ESP32 LittleFS Data Upload' from Arduino IDE Tools menu."
