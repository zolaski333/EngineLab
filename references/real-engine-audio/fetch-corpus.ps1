[CmdletBinding()]
param(
    [string] $ManifestPath,
    [switch] $Refresh
)

$ErrorActionPreference = "Stop"
if ([string]::IsNullOrWhiteSpace($ManifestPath)) {
    $ManifestPath = Join-Path $PSScriptRoot "manifest.json"
}
$manifestFile = Get-Item -LiteralPath $ManifestPath
$corpusRoot = $manifestFile.Directory.FullName
$corpusRootPrefix = $corpusRoot.TrimEnd(
    [System.IO.Path]::DirectorySeparatorChar,
    [System.IO.Path]::AltDirectorySeparatorChar) + [System.IO.Path]::DirectorySeparatorChar
$manifest = Get-Content -LiteralPath $manifestFile.FullName -Raw -Encoding UTF8 | ConvertFrom-Json

if ($manifest.schema_version -ne 2) {
    throw "Unsupported corpus manifest schema: $($manifest.schema_version)"
}

foreach ($reference in $manifest.references) {
    $key = [string] $reference.key
    if ([string]::IsNullOrWhiteSpace($key)) {
        throw "Corpus reference has an empty key"
    }

    $distribution = [string] $reference.distribution
    if ($distribution -notin @("redistributable", "evaluation-only")) {
        throw "Unsupported distribution for ${key}: $distribution"
    }
    if ($distribution -eq "evaluation-only") {
        Write-Host "skip evaluation-only ${key}: acquire it manually under its source terms"
        continue
    }

    $relativeFile = [string] $reference.file
    $previewUrl = [string] $reference.preview_url
    $expected = ([string] $reference.sha256).ToLowerInvariant()
    if ([string]::IsNullOrWhiteSpace($relativeFile) -or
        [System.IO.Path]::IsPathRooted($relativeFile)) {
        throw "Invalid corpus file path for ${key}: $relativeFile"
    }
    if (-not $previewUrl.StartsWith("https://", [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Redistributable reference ${key} needs an HTTPS preview_url"
    }
    if ($expected -notmatch "^[0-9a-f]{64}$") {
        throw "Redistributable reference ${key} needs a lowercase SHA-256"
    }

    $target = [System.IO.Path]::GetFullPath((Join-Path $corpusRoot $relativeFile))
    if (-not $target.StartsWith($corpusRootPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing corpus path outside $corpusRoot : $target"
    }

    if (Test-Path -LiteralPath $target) {
        $actual = (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($actual -eq $expected) {
            Write-Host "verified ${key}: $target"
            continue
        }
        if (-not $Refresh) {
            throw "Hash mismatch for $target. Re-run with -Refresh to replace this exact corpus file."
        }
    }

    $parent = Split-Path -Parent $target
    New-Item -ItemType Directory -Force -Path $parent | Out-Null
    $temporary = $target + ".part"
    if (Test-Path -LiteralPath $temporary) {
        Remove-Item -LiteralPath $temporary -Force
    }

    Write-Host "download ${key}: $previewUrl"
    & curl.exe -L -sS --fail --retry 3 --connect-timeout 20 --max-time 300 $previewUrl -o $temporary
    if ($LASTEXITCODE -ne 0) {
        if (Test-Path -LiteralPath $temporary) {
            Remove-Item -LiteralPath $temporary -Force
        }
        throw "Download failed for ${key}"
    }

    $actual = (Get-FileHash -LiteralPath $temporary -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $expected) {
        Remove-Item -LiteralPath $temporary -Force
        throw "Downloaded hash mismatch for ${key}: expected $expected, got $actual"
    }
    Move-Item -LiteralPath $temporary -Destination $target -Force
    Write-Host "verified ${key}: $target"
}
