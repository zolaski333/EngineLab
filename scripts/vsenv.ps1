# Imports a Visual Studio 2022 x64 build environment into the CURRENT
# PowerShell session, pinned to one toolset, and puts CMake and Ninja on PATH.
#
# DOT-SOURCE it. Shell state does not survive between agent tool calls, so
# every call that runs cmake/ninja/ctest needs its own line:
#
#     . scripts/vsenv.ps1
#
# 2026-10-20: the authoritative tree builds with Visual Studio 2022
# **BuildTools** (a complete C++ toolchain and a valid "Visual Studio 17 2022"
# generator instance) and its toolset, 14.44.35207 -- the one the windows-2022
# CI runner uses. 2026-10-26: the reference machine (i5-10600) carries it too,
# beside a VS 2022 Community with 14.34 that this script must NOT pick: a tree
# configured on Community needs `-DCMAKE_GENERATOR_INSTANCE` at the BuildTools
# root and a clean rebuild.
#
# Consequence to keep in mind, not a detail: every performance figure and every
# instrument baseline taken on 14.34 is NOT comparable with a number taken on
# 14.44. Re-baseline on 14.44 before trusting any before/after.
#
# The toolset is still pinned rather than left to vcvars' default. A Visual
# Studio 18 Community install (MSVC 14.50) sits beside this one, and letting
# vcvars pick the newest toolset would compile the Ninja tree with a different
# compiler than the Visual Studio tree -- so a warning that is an error in one
# would not be in the other, which defeats the point of using the fast tree to
# find compile errors.
#
# Instances are resolved through vswhere, with the known paths as a fallback.
# CLAUDE.md used to say vswhere was damaged on this machine and returned
# nothing; that is no longer true, it lists both instances correctly.

$ErrorActionPreference = 'Stop'

$toolsetVersion = '14.44'

$candidateRoots = @()
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (Test-Path $vswhere) {
    # -products * is required: BuildTools is not in the default product set.
    $candidateRoots += @(& $vswhere -all -products * -format value -property installationPath |
        Where-Object { $_ -match '2022' })
}
$candidateRoots += @(
    'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools',
    'C:\Program Files\Microsoft Visual Studio\2022\Community',
    'C:\Program Files\Microsoft Visual Studio\2022\Professional',
    'C:\Program Files\Microsoft Visual Studio\2022\Enterprise')

$vsRoot = $null
foreach ($root in ($candidateRoots | Select-Object -Unique)) {
    if (Test-Path (Join-Path $root "VC\Tools\MSVC\$toolsetVersion*")) { $vsRoot = $root; break }
}
if ($null -eq $vsRoot) {
    throw ("No Visual Studio 2022 instance carries MSVC $toolsetVersion. Looked in:`n  " +
        (($candidateRoots | Select-Object -Unique) -join "`n  "))
}

$vcvars = Join-Path $vsRoot 'VC\Auxiliary\Build\vcvars64.bat'
if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found at $vcvars" }

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

# Ninja learns which header a compile depends on by parsing cl.exe's
# /showIncludes output. A compiler cache placed in front of cl swallows it, and
# Ninja then records `#deps 0` and serves stale objects after a header edit --
# see CLAUDE.md. sccache is therefore on PATH but NOT enabled in the presets.
# Verify after any toolchain change, on an object whose source includes a header:
#   ninja -C out/build/ninja-release -t deps <some>.obj    # must NOT say #deps 0
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
Write-Host "vsenv: MSVC $toolsetVersion x64 ($vsRoot) | $cmakeVersion | ninja $ninjaVersion | $sccacheVersion"
