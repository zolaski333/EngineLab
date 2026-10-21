# EngineLab — guide for agents

**Read `VISION.md` before any task.** This file says *how* to work; `VISION.md`
says *why* and *towards what*. If they conflict, `VISION.md` wins.

This file is deliberately short (ceiling: ~200 lines). To add a rule, remove or
merge another one. The project is English-only: code, comments, UI, docs and
commit messages.

The measurement history to September 2026 (audits, plans, the old agent notes;
in French) lives in the git tag `archive/docs-2026-09`, under `docs/archive/`:
`git ls-tree --name-only archive/docs-2026-09 docs/archive/` lists it,
`git show archive/docs-2026-09:docs/archive/<file>.md` reads it. A source to
consult, not a set of instructions.

## Current priority

The only place where the priority is written. Do not work on anything else
without the owner's explicit agreement.

**Owner's mandate of 2026-10-20**, in order, each finished before the next:

1. **Linux**: build, tests and CI green there, Windows staying the reference
   platform. What differs between the two: `docs/platforms.md`.
2. **The two-stroke model.** Whatever 2T engine is designed, the curves come
   out chaotic and neither the behaviour nor the sound is a real two-stroke's.
   Free rein to add what a 2T needs (carburettor, reed and power valves, a
   scavenging model) or to rework the ports deeply. The complaint is a
   measurement before it is a fix: `EngineLabDynoSweepHarness --filter RD350`.
3. **Performance**: test duration, build time, real-time factor.
4. **Knock.** Most engines, even the biggest, survive a boost plus an advance
   that would destroy a real one. Fix the knock PHYSICS, not the damage model.
5. **Editing on the engine**: compression ratio by clicking the engine, every
   turbo visible when two or more are fitted, carburettor parameters by
   clicking (if one is added), and two exhaust paths joinable into one (a
   boxer six cannot be made 6-into-1 today).

**Paused: milestone 1, the sound of the CP2** (MT-07, Arrow exhaust): owner's
recordings (`docs/recording-protocol-cp2.md`), a reproducible
simulation/reality comparison at one engine speed (timbre per band, engine
orders, noise share), then close the measured gaps without ever requiring a
per-engine recording. Its two unverified leads live in
`docs/physical-afterfire.md` (the ~3.8 kHz ceiling) and
`docs/thermoacoustic-architecture.md` 25.2 (the plane-mode cutoff).

**Done 2026-10-19: the engine builder from zero** (journal 2026-10-18/19).
Open: the designer window was never checked on screen; the ignition is a
template (no MBT calibration: no knock limit to stop it).

## Working rules

- **Measure before fixing.** This code holds deliberate compensations (flame
  volume fraction × 1.12; an exhaust "voice" layer as a fallback without
  telemetry…). "Fixing" one alone degrades a behaviour that was right: ask what
  compensates downstream.
- **References come from reality** (recordings, literature), never from the
  simulator's current output.
- **One measurement changes one factor.** Changing two proves nothing.
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

On Linux (never the reference; `docs/platforms.md` says what differs):

```
cmake --preset linux-gcc-release && cmake --build --preset linux-gcc-release
ctest --preset linux-gcc-release        # xvfb-run for EngineLab.AudioWorkshop
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
- `scripts/vsenv.ps1` pins the toolset. Since 2026-10-20 the 2022 line here is
  **BuildTools**, MSVC **14.44** (Community and the 14.34 pin are gone; VS 18 /
  MSVC 14.50 sits beside it), so no figure from before that date is comparable.
  If CMake targets VS 18, reconfigure (and the `_deps/*` caches) with
  `-DCMAKE_GENERATOR_INSTANCE` set to the BuildTools root under
  `C:/Program Files (x86)/Microsoft Visual Studio/2022`.
- **Stale binary trap**: a wrong target name (`MSB1009`) compiles nothing, and
  `ctest` reruns the old binary (`EngineLabCoreTests`, `EngineLabExhaustTests`,
  `EngineLabPhysicsRegressionTests`, `EngineLabCombustionPhasingTests`...).
  Check that the target was actually relinked.
- A new test must **fail without the fix** (after rebuilding the right target).
- A `ctest` that prints nothing more after `Start N:` is **stuck**, not slow
  (`Get-Process ... | Select CPU,StartTime`). A very short time can be a
  failure (`EngineLab.Core`: ~90 s if it passes, ~8 s if it aborts). **Never
  run it beside a harness or sweep** (a WSL suite included: shared CPUs):
  `CoreTests` waits on wall-clock deadlines nothing scales by load, so a loaded
  host invents failures. Alone vs in-suite is the cheap discriminator.
- **Release**: bump `project(EngineLab VERSION ...)`, date the `[Unreleased]`
  section of `CHANGELOG.md`, then push the tag `v<version>`: `release.yml`
  refuses a tag that does not match the CMake version.

## Instruments

Every tool has a line in `docs/tests-and-validation.md` (Instruments). The
deterministic ones, comparable from one commit to the next:

- `EngineLabAudioRenderHarness`: renders the real real-time path offline; RMS,
  crest factor, spectral bands, exhaust RT60.
- `EngineLabGeometrySensitivityHarness`: does an exhaust change alter the
  timbre (`shape`) or the level (`level`)? Separates exhaust from layers.
- `EngineLabAbClipRenderer`: listening clips; each segment (idle, rev-up…) has
  its own level. A single loudness over idle + full load makes the idle ~23 dB
  too quiet.
- `EngineLab.CombustionPhasing`: LPP, CA10/50/90, IMEP over an engine-speed sweep.
- `EngineLabDriveScenarioHarness`: a driving complaint as a rerunnable timeline
  (pedals, gears) through the product's coupling; per-gear ratio, 0-100, limits.
- `EngineLabAfterfireHarness`: shape of the afterfire heat release. Use
  `--liftoff-rpm` (at 9,999 rpm, pumping masks everything).
- `EngineLabPhysicsPerfHarness --trace 1 --idle`: a real idle. `--trace <low
  rpm>` without `--idle` = full-load lugging, not an idle.
- `EngineLabIntakeDuctBench`: bit-exact checksum; unchanged = physics untouched.

Non-deterministic:

- `EngineLabRealtimeBudgetHarness` (`--free-run` to measure capacity; without
  it the factor caps at 1.0). **Only valid as a back-to-back comparison** within
  the same hour (`git stash`, rebuild, measure, `stash pop`), with a control
  engine the change cannot affect, alternating which variant runs first and
  ignoring a first warm-up run. Take the minimum of N runs, not the mean.
  `--gas-view` adds the 3-D view's gas-field and probe requests at 60 Hz.

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
- **Screen a sweep row with `ve / delivered_ve > 1.05` separately.** Check
  actual limiter/misfire/fault counters; the ratio alone does not prove their
  cause, especially with lean residual oxygen. Preserve the `0.95 ×
  min(redline, limiter)` ceiling and inspect older top-range CSVs.
- **Any average over a window mixing very different levels reflects the loudest
  part** (loudness, event thresholds, cycle averages). Same for a fixed window
  over an unsettled signal: check the drift.
- **Idles are fragile attractors**: a one-ULP equivalent change can stall an
  engine. Do not revert the optimisation: fix what makes the idle so sensitive.
- **Wrapping phases**: a crank event is a remaining distance, not an absolute
  angle. An accumulator must not receive `forwardPhaseDegrees` without a guard
  (at most 180° per sub-step).
- **Conservation is not acoustic passivity.** A flow bias must be a state,
  never a derivative of the flow it drives. Check a closed perturbation and a
  finite plenum; refine mesh, FV time step and reservoir refresh independently.
- **Thread barrier**: count the workers, not the items (an 8 h 52 hang already
  happened). Signature: a single core at 100 %, the others idle.

## Refuted — do not propose again without new evidence

Details of each measurement are in the docs archive (tag `archive/docs-2026-09`).

- "Unified duct" rewrite: the 300 mm mesh carries the audible geometry (12-14 dB).
- Fork-join per cylinder with woken threads (the resident pool runs the pass
  since 2026-10-20, journal); AVX/AVX2 (dependent-division latency); exhaust CFL
  0.9; MUSCL-Hancock. Two-stroke: 50 mm, or exhaust coupled every substep
  beyond BDC-to-transfer-close (not real time); third-order faces, one-sided
  open ends, characteristic valve, finer transfer coupling (journal 10-16).
- Deepening the muffler's single chamber: 16 dB notch on the EJ25 fundamental.
- Afterfire: faster chemistry (shape unchanged); "1-D cannot carry a backfire
  front" (it rises in 0.2-0.8 ms, the lock is the ~3.8 kHz bandwidth);
  continuous fuel below ~45 % of demand (below the lean limit; packets burn).
- Knock, one input at a time (journal 2026-10-13): the end-gas integral reads
  the MEAN chamber temperature with a delay 2-5 x slow. Isentropic end gas never
  knocks; plus Douaud-Eyzat it over-knocks (1,000-1,200 K); integral control
  pulls 12 deg for nothing.
- Closing cyclic variability through dilution: inert (it exists, 1.7-12.9 %).
- Two-stroke, un-gating exhaust oxidation before fixing scavenging (journal
  2026-10-21): torque 41.2@9751 -> 36.5@7248, egt *down*, four-stroke torque
  and sound moved. Burning a 25-55 % short-circuit at the port mouth returns
  oxygen-free gas: trapping rises while torque falls.
- Two-stroke exhaust geometry (journal 2026-10-21): Blair's 500 m/s and the
  designer's tuned length are right (egt 345-365 C in the tuned band gives
  c 486-493; 940 mm tunes to 8.5k as intended), and lengthening is no fix
  (x1.300 is +33 % overpowered). The curve's TWO humps are both the pipe's --
  they move as 1/L, x1.088 taking 7,245/9,751 to 6,750/9,001 -- so the 8,252
  trough between them, where the real engine peaks, is the chamber's wave
  structure, not scavenging. Do not read a delivery hump off max(delivered_ve):
  it selects the lugged through-flow. A transit-vs-window scavenging race is
  monotonic in rpm by construction: +26 % at 5,752, nothing at the top.
- "Trapped residual too low": a unit error, the residual is sound.
- Intake runners: four inertance formulations are unstable or out of phase.
  The intake charge bias was not the MUSCL limiter, the finite plenum, wall
  heat or valve linearisation (it was the valve nozzle boundary, Courant-
  dependent). Front-loaded or trajectory-resolved forward delivery: inert.
  Individual throttles as a small 0-D cell per cylinder: -30 % WOT torque at
  6,500 rpm (orifice dump loss); the plate belongs inside the 1-D runner.
- Telemetry cache for listening: nothing gained. State-variable valve-flow
  reconstruction against clicks: inert (journal 2026-10-11).
- The other simulator's ingredients, one at a time (journal 2026-10-13/14):
  output stage, pressure-release outlet, valve-area noise, mild room IR: none
  kept (blind listening or measurement). Its fine mesh is not shown to matter.
- Seeding the high-load correction cell from the idle cell: saves a start 4 s
  after cranking but stalls 10 engines out of 14 after 25 s of idle, and on a
  blip (Hayabusa, CP3, Aircooled). Asynchronous pulse on pedal tip-in: does not
  save the 2JZ at 4 s, stalls the Aircooled at 8 s.
- Crediting port vapour to the port-injection pulse: it is a stationary
  reservoir, not fuel for the charge (delivered/demanded 0.47-0.77). For DFCO
  recovery, 8 variants fail (priming the 1st pulse only, film over one cycle,
  carry-over ∝ demand, reference vapour level); 5 more trade the lean first
  cycle against rich or misfires (resume trim, footprint; journal 2026-10-06).

## Document layout

`VISION.md`: the intent; this file: the method (`AGENTS.md` points here).
`docs/*.md`: how each part works, kept up to date with the code (index:
`docs/README.md`). `docs/journal.md`: measurement results, short entries.
`CHANGELOG.md`: user-visible changes per release.
