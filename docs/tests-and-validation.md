# Tests and validation

This page describes the current reliability groundwork: what the test suite
checks, how to run it, and what it does not prove.

## Structural fixes in place

Configuration normalisation treats explicit topologies as authoritative. The
old global properties must not silently overwrite crankshafts, intake paths or
exhaust paths that are already described. The V8 and flat-six presets were
made consistent again with their journals, offsets and path volumes.

The kinematics share one computed reference and cover conventional, master and
articulated connecting rods. TDC positions and chamber volumes account for the
geometry actually configured. Friction at very low speed keeps a resisting
sign.

The gas network uses:

- a signed directional dynamic pressure;
- the real characteristic areas of volumes and restrictions;
- a transfer bounded by pressure equilibrium;
- a simultaneous solution of the flows during valve overlap;
- checked conservation of species, energy and both momentum components.

Events and audio frames are produced at the real sub-steps, with their
simulation time. A reset clears old samples. Per-cycle telemetry comes from the
observed cycle crossings, not from an estimate frozen on the duration of a UI
call.

The runtime and audio were also hardened around bounded queues, the
producer/consumer clock, publishing the engaged gear, ending a dyno run that
does not converge and swapping impulse responses across threads. The continuous
signal stays tied to its exhaust path and the DSP constants follow the real
sample rate.

Finally, the comparison harness explicitly enables pressure-frame production.
Its WAVs can no longer pass while being entirely silent.

## CTest suite

With the tests and harnesses enabled (the default):

```powershell
cmake --preset windows-vs2022
cmake --build --preset windows-release
ctest --preset windows-release
```

The main registered tests are:

| Test | Main subject |
|---|---|
| `EngineLab.Core` | simulation integration, gas, combustion, transmission, serialisation, catalogue and basic audio |
| `EngineLab.PhysicsRegression` | authoritative topology, presets, signed dynamic pressure, flows and kinematics |
| `EngineLab.Calibration` | types, interpolation, validation, transactions, concurrency, JSON and file watcher |
| `EngineLab.EcuCalibration` | immediate application of the tables and the rev limiter by the ECU |
| `EngineLab.Exhaust` | DAG validation, series/parallel losses, aggregated flow properties, routes, migration and JSON/YAML round-trips |
| `EngineLab.ExhaustSimulation` | effect of the authored K on pressure/flow/torque, independence from the legacy geometry |
| `EngineLab.Scripting` | units, diagnostics, base/include, cycles and keeping the last valid script |
| `EngineLab.RenderSnapshot` | bounded scene, 3D layout and angle interpolation |
| `EngineLab.EngineModel3D` | 3-D meshes and poses of every catalogue engine (rod length, wrist pin on axis, crown below deck) and the display crank clock (steady, accelerating, slow motion, freeze, pause) |
| `EngineLab.RealtimeRegression` | timing, audio rate, buffers, path isolation and DAG transmission |
| `EngineLab.CatalogPhysics` | deterministic scenarios and loaded holds governed by a PI brake across the whole catalogue |
| `EngineLab.AudioRender` | multi-engine renders at normalised engine speed, finiteness, dynamics, limiter ceiling and long-run stability |

`ctest --preset windows-release -N` lists the complete set.

`EngineLab.AudioRender` only emits a warning when spectral signatures are too
close; that threshold is not yet a failure cause. This nuance is documented on
purpose: the test prevents invalid or crushed outputs, but does not prove good
perceptual differentiation.

## Comparison harness

The deterministic harness produces, for I4, V8, V-twin and radial:

- a CSV trace per scenario;
- a mono WAV derived from the intra-cycle pressures;
- `summary.csv` with engine speed, torque, AFR, fuel, pressure, IMEP, runner
  resonance, audio RMS and peak;
- an operating gate on every engine of the catalogue.

```powershell
cmake --build out/build/windows-vs2022 --config Release --target EngineLabComparisonHarness
out\build\windows-vs2022\tools\Release\EngineLabComparisonHarness.exe `
  --output comparison-output `
  --catalog-root .
```

The WAV must have non-zero energy and peak. The catalogue thresholds check
start/engine speed, AFR tracking and FMEP on a loaded hold at 65 % of the rev
limiter, as well as the spool of a configured turbo.

To compare with a reference EngineLab result:

```powershell
out\build\windows-vs2022\tools\Release\EngineLabComparisonHarness.exe `
  --output comparison-candidate `
  --catalog-root . `
  --reference comparison-baseline\summary.csv
```

Tolerances are relative: 5 % for mean engine speed and AFR, 2 % for resonance
frequency, 10 % for torque, fuel, pressure and IMEP. The WAV metrics are
recorded and checked non-zero, but do not take part in this numerical reference
comparison yet.

The complete JUCE renderer can be run separately:

```powershell
cmake --build out/build/windows-vs2022 --config Release --target EngineLabAudioRenderHarness
out\build\windows-vs2022\tools\Release\EngineLabAudioRenderHarness.exe --output audio-render-output
```

## Sanitizers and static analysis

In a dedicated build tree:

```powershell
cmake -S . -B build-asan -G "Visual Studio 17 2022" -A x64 `
  -DENGINELAB_ENABLE_SANITIZERS=ON
cmake --build build-asan --config Debug
ctest --test-dir build-asan -C Debug --output-on-failure
```

MSVC enables AddressSanitizer; GCC/Clang toolchains enable AddressSanitizer and
UndefinedBehaviorSanitizer. Another tree can enable
`-DENGINELAB_ENABLE_CLANG_TIDY=ON` if `clang-tidy` is available.

Sanitizer builds are slower, especially `EngineLab.Core` and the harnesses. An
error-free run does not replace dedicated race analysis or a long campaign on
several audio devices.

## What these checks prove — and do not prove

They prove numerical invariants, round-trips, atomic transactions, bounded
outputs and expected trends on the shipped scenarios. They make regressions
visible.

They do not prove the accuracy of a real torque curve, the fidelity of a
specific exhaust, stability on every possible configuration or perceptual
parity with ES2D. Those claims need external data, A/B protocols and physical
references that the repository does not hold today.
