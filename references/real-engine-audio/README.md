# Real engine reference corpus

This folder describes the corpus used by `EngineLabAbClipRenderer`. The five
sources are genuine field recordings published under **CC0 1.0**. The audio
files are not versioned: the manifest pins their HQ preview URL, SHA-256,
author, license and how closely each one actually matches the EngineLab engine.

The `exact-platform` level means the vehicle is explicitly identified.
`platform-proxy`, `family-proxy` and `architecture-proxy` are deliberately
weaker: they are for judging a sound character, not for claiming an exact
correlation.

## Verified download

From the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File references/real-engine-audio/fetch-corpus.ps1
```

The script keeps a file only if its SHA-256 matches the manifest. Files are
written under `files/`, which Git ignores.

## Full A/B render

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAbClipRenderer.exe `
  --output out/validation/listening-current `
  --engines "2JZ,LS3,Hayabusa,Big Twin,EJ25" `
  --reference-manifest references/real-engine-audio/manifest.json `
  --require-references
```

The tool decodes WAV/AIFF/FLAC/OGG, resamples to 48 kHz automatically, trims
both sides to the same duration, applies the same fades and lowers both sides
together if either exceeds the peak ceiling. The JSON key keeps the provenance,
license, original sample rate and match level of every reference.
