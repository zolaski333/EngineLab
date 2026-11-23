# Runs one `EngineLab --ui-bench` scene and keeps what it measured and showed.
#
#   scripts/ui-bench.ps1 -Exe <EngineLab.exe> -Name <scene> -OutDir <folder> [-BenchArgs '--engine 2JZ --throttle 1']
#
# Writes <OutDir>/<Name>.csv (one row a second, then the medians) and, once the
# scene is ready, <Name>.png (the main window's content) and <Name>-screen.png
# (the whole primary screen, for the tool windows). Windows only: the capture
# copies the screen, so the 3-D view's OpenGL surface is in the picture.
# Compare two builds as CLAUDE.md (Instruments) says: interleaved, the first
# run discarded, the minimum of N.
param(
    [Parameter(Mandatory = $true)][string]$Exe,
    [Parameter(Mandatory = $true)][string]$Name,
    [Parameter(Mandatory = $true)][string]$OutDir,
    [string]$BenchArgs = '',
    [int]$TimeoutSeconds = 180
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms
New-Item -ItemType Directory -Force $OutDir | Out-Null
$csv = Join-Path $OutDir "$Name.csv"
$marker = Join-Path $OutDir "$Name.marker"
Remove-Item -ErrorAction SilentlyContinue $csv, $marker
$arguments = "--ui-bench $BenchArgs --out `"$csv`" --capture-marker `"$marker`" --capture-hold 3"
$process = Start-Process -FilePath $Exe -ArgumentList $arguments -PassThru
$deadline = (Get-Date).AddSeconds($TimeoutSeconds)
while (-not (Test-Path $marker)) {
    if ($process.HasExited) { throw "EngineLab exited before the scene was ready ($Name)" }
    if ((Get-Date) -gt $deadline) { $process.Kill(); throw "timed out waiting for the scene ($Name)" }
    Start-Sleep -Milliseconds 200
}
Start-Sleep -Milliseconds 700
$parts = (Get-Content $marker -Raw).Trim().Split(' ') | ForEach-Object { [int]$_ }
$screen = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
$full = New-Object System.Drawing.Bitmap $screen.Width, $screen.Height
$graphics = [System.Drawing.Graphics]::FromImage($full)
$graphics.CopyFromScreen($screen.X, $screen.Y, 0, 0, $full.Size)
$graphics.Dispose()
$full.Save((Join-Path $OutDir "$Name-screen.png"), [System.Drawing.Imaging.ImageFormat]::Png)
$x = [Math]::Max(0, $parts[0]); $y = [Math]::Max(0, $parts[1])
$w = [Math]::Min($parts[2], $screen.Width - $x); $h = [Math]::Min($parts[3], $screen.Height - $y)
$crop = $full.Clone((New-Object System.Drawing.Rectangle $x, $y, $w, $h), $full.PixelFormat)
$crop.Save((Join-Path $OutDir "$Name.png"), [System.Drawing.Imaging.ImageFormat]::Png)
$crop.Dispose(); $full.Dispose()
if (-not $process.WaitForExit(($TimeoutSeconds) * 1000)) { $process.Kill(); throw "EngineLab did not quit ($Name)" }
Remove-Item -ErrorAction SilentlyContinue $marker
if (-not (Test-Path $csv)) { throw "no CSV written ($Name)" }
Get-Content $csv | Select-Object -Last 1
