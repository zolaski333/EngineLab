# High-quality offline audio rendering

## What it does

EngineLab has a single offline render path shared by the command-line tool and
the application:

```text
EngineSimulator -> publishAudioFrame -> RealtimeEngineAudio -> WAV
```

It is not a parallel synthesiser. The HQ render consumes the same firing
events, cylinder pressure samples, acoustic graphs, forced sources and mix
controls as the real-time application. Only the output rate and the disk
writing change.

Available features:

- 48, 96 or 192 kHz;
- stereo WAV, 24-bit PCM (RIFF format code 1) or 32-bit IEEE float (code 3);
- master plus six optional stems: combustion, dry exhaust, IR return, intake,
  forced induction and mechanical;
- reproducible JSON scenario, versioned by `schema_version: 1`;
- 20 ms fade at the start and the end;
- loading of the impulse responses actually written in the engine
  configuration, with no hidden substitute IR;
- JSON manifest with format, exact frame count, mix, files, warnings and
  physical-path counters;
- writing to `.partial` files first, promoted only once the render and the
  metadata are complete;
- cancellation through a callback, and refusal to overwrite an existing result
  by default.

The shared API is declared in
`src/audio/include/enginelab/audio/OfflineAudioExporter.hpp`. The tool is
`EngineLabOfflineAudioExporter`.

## Usage

Recommended example for sound work:

```powershell
.\EngineLabOfflineAudioExporter.exe `
  --engine K20 `
  --output .\exports\k20-96k `
  --sample-rate 96000 `
  --format pcm24 `
  --stems
```

The default profile is:

```text
start -> idle -> full-load rev-up -> rev limiter -> deceleration
```

Every export writes its canonical copy to `scenario.json`. It can be edited and
replayed:

```powershell
.\EngineLabOfflineAudioExporter.exe `
  --engine K20 `
  --output .\exports\k20-custom `
  --scenario .\my-scenario.json `
  --sample-rate 192000 `
  --format float32 `
  --no-stems
```

The schema of a step contains:

```json
{
  "name": "rev_up",
  "duration_seconds": 3.2,
  "ignition": true,
  "starter": false,
  "governed": true,
  "throttle_start": 0.98,
  "throttle_end": 0.98,
  "load_start": 0.0,
  "load_end": 0.0,
  "target_rpm_start": 1600.0,
  "target_rpm_end": 7400.0
}
```

With `governed: true`, the offline dyno follows the RPM target and ignores
`load_start/load_end`. Without the governor, the load is interpolated directly.
Scenarios are limited to 120 seconds to stay within a 32-bit RIFF WAV and to
avoid accidentally creating several gigabytes of stems.

## Automated checks

Command:

```powershell
cmake --build out\build\windows-vs2022 --config Release `
  --target EngineLabOfflineAudioExportTests EngineLabOfflineAudioExporter `
  --parallel 1 -- /nr:false /m:1 /v:minimal

.\out\build\windows-vs2022\tests\Release\EngineLabOfflineAudioExportTests.exe
```

Result:

```text
Offline audio export: PCM24 stems, float32 192 kHz, JSON scenario,
manifest and cancellation PASS
```

The test opens the generated files and checks, byte by byte:

- the RIFF/WAVE signature;
- format code 1 for PCM and 3 for IEEE float;
- two channels;
- the requested sample rate and bit depth;
- the data chunk size and the physical file size;
- master + six stems + metadata;
- the JSON scenario round-trip;
- a manifest naming the real render path;
- rejection of 44.1 kHz;
- cancellation without a final WAV;
- zero delay truncations and zero lost telemetry.

## Two complete control renders

These figures prove the run of 29 July 2026 on the machine of the time. They
are neither a new absolute performance reference nor a perceptual quality
comparison.

### K20, 96 kHz, 24-bit PCM, master + six stems

```text
988800 frames / 10.300000 s
peak 0.425443 / RMS 0.044991
physical exhaust active / compiled exhaust graph yes
delay truncations 0 / invalid boundaries 0
dropped firing events 0 / dropped pressure samples 0
```

### K20, 192 kHz, 32-bit float, master

```text
1977600 frames / 10.300000 s
peak 0.419566 / RMS 0.045786
physical exhaust active / compiled exhaust graph yes
delay truncations 0 / invalid boundaries 0
dropped firing events 0 / dropped pressure samples 0
```

## What this does not claim

- Going to 192 kHz does not automatically make an engine more realistic. It
  gives headroom for spectral work and keeps the real-time audio throughput
  from blocking lab listening.
- A WAV structure test does not judge timbre. The stems exist precisely to
  diagnose and listen to the sources separately.
- Offline success does not replace the catalogue's real-time guard. That is
  still measured separately with `EngineLabRealtimeBudgetHarness --free-run`.
