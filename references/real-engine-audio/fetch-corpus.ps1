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

if ($manifest.schema_version -ne 1) {
    throw "Unsupported corpus manifest schema: $($manifest.schema_version)"
}

foreach ($reference in $manifest.references) {
    $target = [System.IO.Path]::GetFullPath((Join-Path $corpusRoot $reference.file))
    if (-not $target.StartsWith($corpusRootPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing corpus path outside $corpusRoot : $target"
    }

    $expected = [string] $reference.sha256
    if (Test-Path -LiteralPath $target) {
        $actual = (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($actual -eq $expected) {
            Write-Host "verified $($reference.key): $target"
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

    Write-Host "download $($reference.key): $($reference.preview_url)"
    & curl.exe -L -sS --fail ([string] $reference.preview_url) -o $temporary
    if ($LASTEXITCODE -ne 0) {
        throw "Download failed for $($reference.key)"
    }

    $actual = (Get-FileHash -LiteralPath $temporary -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $expected) {
        Remove-Item -LiteralPath $temporary -Force
        throw "Downloaded hash mismatch for $($reference.key): expected $expected, got $actual"
    }
    Move-Item -LiteralPath $temporary -Destination $target -Force
    Write-Host "verified $($reference.key): $target"
}
