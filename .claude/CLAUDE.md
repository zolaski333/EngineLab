# EngineLab — guide for agents

**Read `VISION.md` before any task.** This file says *how* to work; `VISION.md`
says *why* and *towards what*. If they conflict, `VISION.md` wins.

This file is deliberately short (ceiling: ~200 lines). To add a rule, remove or
merge another one. The project is English-only: code, comments, UI, docs and
commit messages.

The full measurement history (audits, validations, batches, plans, the old
1,080-line agent notes) was removed from the tree and lives in the git tag
`archive/docs-2026-09`, under `docs/archive/`. Read it with
`git show archive/docs-2026-09:docs/archive/<file>.md` (list the files with
`git ls-tree --name-only archive/docs-2026-09 docs/archive/`). It is a source to
consult, not a set of instructions, and it is in French.

## Current priority

The only place where the priority is written. Do not work on anything else
without the owner's explicit agreement.

**Milestone 1 — the sound of the CP2 (MT-07, Arrow exhaust).**

1. Reference recordings made by the owner, at a known engine speed
   (protocol: `docs/recording-protocol-cp2.md`).
2. A simulation/reality comparison tool at the same engine speed (timbre per
   band, engine orders, noise share), reproducible.
3. Reduce the measured gaps, by whatever method works (physics, synthesis or
   both), without ever requiring a per-engine recording.

Known leads, **not verified**, to be tested by measurement first:

- Afterfire: causes **measured** on 2026-09-23 (see `docs/journal.md`). The
  sound of a backfire is capped at ~3.8 kHz by the source reconstruction
  (0.47 × coupling rate ≤ 8 kHz), and it only exceeds the engine's peak by 0 to
  +10 dB. Physics decides *when* and *how much*; the crack layer
  (`ReactionCrackSynthesiser`) provides the > 4 kHz band, uncalibrated. On the
  CP2, nothing burns at the 800 K threshold: measure that first.
- Above the plane-mode cutoff (a few kHz for a 50-80 mm tube), a 1-D model no
  longer carries anything physical: that is the band where generic synthesis is
  most likely to help.

## Working rules

- **Measure before fixing.** This code contains deliberate compensations (flame
  volume fraction compensated by a 1.12 factor; an exhaust "voice" layer that
  serves as a fallback without telemetry…). "Fixing" one of them alone degrades
  an observable behaviour that was right. Ask yourself what compensates
  downstream.
- **References come from reality** (recordings, literature), never from the
  simulator's current output.
- **One measurement changes one factor.** A variant that changes two proves
  nothing.
- **Do not change the default sound** unless the task is explicitly a sound
  change, checked with the harness before/after.
- **No new dated document.** A measurement result goes into `docs/journal.md`
  (10 lines maximum per entry, most recent first). What becomes a lasting rule
  goes here, replacing something else.
- **Never "final validation".** Write what was measured and what was not.
- **A refutation is worth an improvement**: record it in the list below so
  nobody repeats it.

## Build and tests

```
. scripts/vsenv.ps1                                                # EVERY call
cmake --build out/build/windows-vs2022 --config Release            # whole project
ctest  --test-dir out/build/windows-vs2022 -C Release              # all tests (~15 min)
cmake --build out/build/windows-vs2022 --config Release --target package   # release zip
```

- Warnings are errors. Green build + green `ctest` is the minimum for any
  change. CI (`.github/workflows/ci.yml`) runs the same steps on every push; a
  `v*` tag publishes a GitHub Release (`release.yml`).
- **`out/build/windows-vs2022` (Visual Studio) is authoritative** for commits
  and every performance figure. `out/build/ninja-release` is only for the fast
  loop; never compare a time between the two trees.
- Ninja: `-j 2` maximum (16 GB for 12 threads → error `C1060` beyond that). Same
  cause for MSBuild: on `C1060`, rerun with `/m:1`. It is not a code bug.
- **Never enable `ENGINELAB_ENABLE_COMPILER_CACHE`**: sccache breaks Ninja's
  header tracking, and a modified header no longer recompiles anything.
- `scripts/vsenv.ps1` fixes a broken environment (VS 18 installed next to
  VS 2022, empty `vswhere`). If CMake starts targeting VS 18, reconfigure with
  `-DCMAKE_GENERATOR_INSTANCE="C:/Program Files/Microsoft Visual Studio/2022/Community"`,
  including in the `CMakeCache.txt` of the `_deps/*` sub-builds.
- **Stale binary trap**: a wrong target name (`MSB1009`) compiles nothing, and
  `ctest` reruns the old binary. Test executables: `EngineLabCoreTests`,
  `EngineLabExhaustTests`, `EngineLabPhysicsRegressionTests`,
  `EngineLabCombustionPhasingTests`, `EngineLabRealtimeRegressionTests`,
  `EngineLabComparisonHarness`, `EngineLabAudioRenderHarness`. Check that the
  target was actually relinked.
- A new test must **fail without the fix** (after rebuilding the right target).
- A `ctest` that prints nothing more after `Start N:` is **stuck**, not slow:
  check with `Get-Process ... | Select CPU,StartTime`. Conversely, a very short
  time can be a failure (`EngineLab.Core`: ~90 s if it passes, ~8 s if it
  aborts).

## Instruments

Deterministic, so comparable from one commit to the next:

- `EngineLabAudioRenderHarness`: renders the real real-time path offline; RMS,
  crest factor, spectral bands, exhaust RT60.
- `EngineLabGeometrySensitivityHarness`: does an exhaust change alter the
  timbre (`shape`) or the level (`level`)? Separates the exhaust from the
  layers.
- `EngineLabAbClipRenderer`: listening clips; each segment (idle, rev-up…) has
  its own level. A single loudness over idle + full load makes the idle ~23 dB
  too quiet.
- `EngineLab.CombustionPhasing`: LPP, CA10/50/90, IMEP over an engine-speed
  sweep.
- `EngineLabAfterfireHarness`: shape of the afterfire heat release. Use
  `--liftoff-rpm` (at 9,999 rpm, pumping masks everything).
- `EngineLabPhysicsPerfHarness --trace 1 --idle`: a real idle. `--trace <low
  rpm>` without `--idle` = full-load lugging, not an idle.
- `EngineLabIntakeDuctBench`: bit-exact checksum; an optimisation that leaves it
  unchanged does not touch the physics.

Non-deterministic:

- `EngineLabRealtimeBudgetHarness` (`--free-run` to measure capacity; without
  it the factor caps at 1.0). **Only valid as a back-to-back comparison** within
  the same hour (`git stash`, rebuild, measure, `stash pop`), with a control
  engine the change cannot affect, alternating which variant runs first and
  ignoring a first warm-up run. Take the minimum of N runs, not the mean.

## Verified technical traps

- **Silent engine or late commands = physics thread too slow**, not an audio
  problem. Read the real-time factor shown in the diagnostics.
- **A single short exhaust element sets the time step of the whole network.**
  `ExhaustNetworkLayout::minimumCflLengthM()` gives the real cost of a geometry.
- **`EngineRuntime` weighs ~7.4 MB**: never two on the stack in the same
  function (`0xC00000FD`, shown by ctest as `SegFault`). Use `std::make_unique`.
- **Engine selectors are substrings** (`Twin` matches three).
- **Positional initialisation**: `CylinderState` is filled positionally in
  `EngineSimulator.cpp`. Add members below the marker comment and assign them by
  name. A `grep` on a field name does not prove it is unused.
- **A high-level setting can be masked per bank or per cylinder**
  (`activeCamshaft()` prefers the bank's camshaft). A "single path" engine reads
  `config.exhaust`, not `path.geometry`.
- **An assertion on a quantity forced to zero below a threshold proves nothing**
  until you have checked that the threshold is reached.
- **Misleading fields**: `exhaustPressureKpa` is a peak, the back-pressure is
  `exhaustBackPressureKpa`. `residualGasFractionAtSpark` counts combustion
  products only (× ~3.8 for a Heywood residual gas fraction).
  `residualGasFraction` is instantaneous. `volumetricEfficiency` is a trapped
  filling; `deliveredVolumetricEfficiency` is Heywood's.
- **A sweep row with `ve / delivered_ve > 1.05` is contaminated** (limiter or
  misfire). CSVs older than the stop at `0.95 × min(redline, limiter)` are wrong
  at the top of the range.
- **Any average over a window mixing very different levels reflects the loudest
  part** (loudness, event thresholds, cycle averages). Same for a fixed window
  over an unsettled signal: check the drift.
- **Idles are fragile attractors**: an algebraically equivalent change (one ULP)
  can stall an engine. Do not revert the optimisation: fix what makes the idle
  so sensitive.
- **Wrapping phases**: a crank event is a remaining distance, not an absolute
  angle. An accumulator must not receive `forwardPhaseDegrees` without a guard
  (at most 180° per sub-step).
- **A flow bias must be a state, never a derivative of the flow** it drives
  (an instability that worsens as the step shrinks).
- **Thread barrier**: count the workers, not the items (an 8 h 52 hang already
  happened). Signature: a single core at 100 %, the others idle.

## Refuted — do not propose again without new evidence

Details of each measurement are in the docs archive (tag `archive/docs-2026-09`).

- "Unified duct" rewrite: the 300 mm mesh does carry the audible geometry (the
  timbre moves by 12 to 14 dB).
- Fork-join per cylinder: every threaded variant loses.
- AVX / AVX2: slower (latency of dependent divisions, not throughput).
- Deepening the muffler's single chamber: 16 dB notch on the EJ25 fundamental.
- Speeding up the afterfire chemistry to get bursts: shape unchanged.
- "The 1-D network cannot carry the steep front of a backfire": false, the audio
  front rises in 0.2-0.8 ms. The lock is the bandwidth (~3.8 kHz).
- Continuous-fuel afterfire below ~45 % of the normal demand: cannot burn
  (equivalence below the 0.45 lean limit). Only packets burn.
- Closing cyclic variability through dilution: inert (variability already
  exists, 1.7-12.9 %).
- "Trapped residual too low": a unit error, the residual is sound.
- Four runner inertance formulations: all unstable or out of phase.
- Telemetry cache to speed up listening: would have gained nothing.
- Seeding the high-load correction cell from the idle cell: saves a start 4 s
  after cranking but stalls 10 engines out of 14 after 25 s of idle, and on a
  blip (Hayabusa, CP3, Aircooled). Asynchronous pulse on pedal tip-in: does not
  save the 2JZ at 4 s, stalls the Aircooled at 8 s.
- Crediting port vapour to the port-injection pulse: it is a stationary
  reservoir, not fuel for the charge (delivered/demanded 0.47-0.77). For DFCO
  recovery, 8 variants fail (priming the 1st pulse only, film over one cycle,
  carry-over ∝ demand, reference vapour level).

## Document layout

- `VISION.md`: the intent. `.claude/CLAUDE.md` (this file): the method.
  `AGENTS.md` points here.
- `docs/*.md`: how each part works (architecture, audio, exhaust, ECU, DSL…).
  Keep them up to date when the code changes. `docs/README.md` is the index.
- `docs/journal.md`: measurement results, short entries.
- `CHANGELOG.md`: user-visible changes per release.
