# Imports the Visual Studio 2022 x64 build environment into the CURRENT
# PowerShell session, pinned to the toolset the windows-vs2022 tree already
# uses (14.34.31933), and puts CMake, Ninja and sccache on PATH.
#
# DOT-SOURCE it. Shell state does not survive between agent tool calls, so
# every call that runs cmake/ninja/ctest needs its own line:
#
#     . scripts/vsenv.ps1
#
# Launch-VsDevShell.ps1 is deliberately not used: it resolves the install
# through vswhere.exe, whose instance registry is damaged on this machine and
# returns nothing (see CLAUDE.md). vcvars64.bat is a fixed path and works.
#
# The toolset pin matters. A Visual Studio 18 Community install sits beside
# 2022, and letting vcvars pick the newest toolset would compile the Ninja tree
# with a different compiler than the Visual Studio tree -- so a warning that is
# an error in one would not be in the other, which defeats the point of using
# the fast tree to find compile errors.

$ErrorActionPreference = 'Stop'

$vsRoot = 'C:\Program Files\Microsoft Visual Studio\2022\Community'
$vcvars = Join-Path $vsRoot 'VC\Auxiliary\Build\vcvars64.bat'
$toolsetVersion = '14.34'

if (-not (Test-Path $vcvars)) {
    throw "vcvars64.bat not found at $vcvars"
}

# `cmd /c "<batch> && set"` is the only reliable way to capture what a batch
# file exported: the variables die with the cmd process otherwise.
$captured = cmd /c "`"$vcvars`" -vcvars_ver=$toolsetVersion >NUL 2>&1 && set"
if ($LASTEXITCODE -ne 0) {
    throw "vcvars64.bat failed (exit $LASTEXITCODE) for toolset $toolsetVersion"
}
foreach ($line in $captured) {
    if ($line -match '^([^=]+)=(.*)$') {
        Set-Item -Path "env:$($matches[1])" -Value $matches[2]
    }
}

$extraPaths = @(
    (Join-Path $vsRoot 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin'),
    (Join-Path $vsRoot 'Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja'))

# winget installs sccache into a version-stamped directory, so resolve it by
# search rather than pinning a path that the next update would invalidate.
$sccachePackages = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Packages'
if (Test-Path $sccachePackages) {
    $sccache = Get-ChildItem $sccachePackages -Directory -Filter 'Mozilla.sccache*' -ErrorAction SilentlyContinue |
        ForEach-Object { Get-ChildItem $_.FullName -Recurse -Filter 'sccache.exe' -ErrorAction SilentlyContinue } |
        Select-Object -First 1
    if ($null -ne $sccache) { $extraPaths += $sccache.Directory.FullName }
}

foreach ($path in $extraPaths) {
    if ((Test-Path $path) -and ($env:PATH -notlike "*$path*")) {
        $env:PATH = "$path;$env:PATH"
    }
}

# 10 GB is sccache's default and is not enough to hold both build trees plus the
# fetched dependencies across a bisect, which is exactly when the cache pays.
if (-not $env:SCCACHE_CACHE_SIZE) { $env:SCCACHE_CACHE_SIZE = '30G' }

$cmakeVersion = (cmake --version | Select-Object -First 1)
$ninjaVersion = (ninja --version)
$sccacheVersion = if (Get-Command sccache -ErrorAction SilentlyContinue) { (sccache --version) } else { 'absent' }
Write-Host "vsenv: MSVC $toolsetVersion x64 | $cmakeVersion | ninja $ninjaVersion | $sccacheVersion"
