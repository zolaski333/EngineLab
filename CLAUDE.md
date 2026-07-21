# Working on EngineLab

Guidance for AI agents. It is not a codebase tour — it is the set of things that
were learned the hard way here and are not visible from reading the code.

## Build and test

```
cmake --build out/build/windows-vs2022 --config Release            # whole project
ctest  --test-dir out/build/windows-vs2022 -C Release              # all tests
```

Warnings are errors. A green build and `ctest` run are the bar for any change.

## The one rule that matters most: measure before you "fix"

This codebase reads as if it is full of bugs. Several of them are not — they are
deliberate compensations that deliver correct *observable* behaviour, and
"fixing" them in isolation regresses a working calibration. Reading the code will
mislead you. Measuring will not.

Worked examples from this project, where a confident code-reading hypothesis was
overturned by measurement:

- **Flame model uses a volume burn fraction where a mass fraction is arguably
  correct.** Looks like it must mis-time combustion and dump heat too early. It
  does not: `EngineLab.CombustionPhasing` measures peak-pressure location at
  18-22 deg ATDC and CA50 at 9-11 deg ATDC, stable across 2300-4200 rpm — in the
  MBT window. The internal representation is absorbed by the spark map and the
  turbulent-flame calibration. Correcting it would shift phasing later and force
  a re-tune, with no observable gain.
- **The exhaust "voice" layer re-synthesises a blowdown body that the
  pressure-driven collector/FDN path already produces.** Looks like a redundant
  double model to delete. Measured, its contribution is ~1-2 spectral points
  (already tamed by a 0.20 layer gain), and it is the *only* thing that
  differentiates presets when there is no pressure telemetry — a real fallback.
  Removing it broke `EngineLab.Core` for a negligible gain.

The pattern: this codebase tends to fix a symptom downstream of its cause
(muffler presets tuned around a mis-normalised FDN matrix; a 1.12 turbulent-flame
factor compensating the volume fraction; a safety AGC masking a level offset).
When you find a "bug", first ask what downstream already compensates for it, and
whether fixing it alone makes the delivered behaviour worse. When two such
corrections are coupled, change them together, guarded by a measurement.

Preserve the default audio voicing exactly unless a change is explicitly a
voicing change: the audio-path corrections here were verified bit-identical at
the default `convolution` setting via the harness before and after.

## Use the instruments; do not judge audio or combustion by eye

- **`tools/AudioRenderHarness.cpp`** (`EngineLab.AudioRender`) renders the real
  realtime path offline and reports RMS/crest/DC/spectral-band balance per
  channel over the whole signal, plus a Schroeder RT60 of the exhaust chain. It
  is deterministic. It is how you prove an audio change did what you intended.
  Note: the exhaust chain is only excited by the cylinder-pressure telemetry
  stream, never by a lone firing event — an event-only impulse measures the voice
  envelope, not the muffler.
- **`tests/CombustionPhasingTests.cpp`** (`EngineLab.CombustionPhasing`) drives a
  known engine across an rpm sweep and reports LPP / CA10-50-90 / IMEP. Peak
  pressure location is the bug-independent physical truth; a single operating
  point cannot tell a correct model from a compensated one, so it sweeps.

Reference numbers for these gates come from engine/DSP literature, never from the
simulator's current output, so tightening a gate later cannot re-calibrate the
test onto the behaviour it is meant to catch. Keep it that way.

- **Separate the audio complaints; they have unrelated causes.** "Muffled" is a
  bandwidth problem (the exhaust boundary coupling caps the physical band at low
  speed — see `docs/thermoacoustic-architecture.md` §14). "All engines sound the
  same" is *not*: measured per-third-octave, the catalogue engines differ by
  15-30 dB in every band (`overlay.py`), and the crossplane V8 carries its
  uneven-bank burble at 10x an even I4 (`burble.py`). The engines are
  differentiated; muffling was masking it. Beware: making every engine brighter
  raises the coarse spectral-shape *correlation* even though it improves the
  sound — that metric penalises "everyone gained treble", so do not chase it
  down. Character lives in the firing-pattern envelope, not the steady spectrum.
- **First-order models get first-order tests.** The finite-amplitude duct
  steepening (`NonlinearDuctAcoustics.hpp`) reproduces the Fubini second-harmonic
  law `B2/B1 -> sigma/2` but deliberately under-generates the third harmonic
  (single-probe scheme, ~44% of exact Fubini). Its regression asserts the
  leading-order law, monotone cascade, growth, and passivity — never an exact
  higher-harmonic match, which would have to be calibrated onto the simulator's
  own output.
- **The silencer is `muffler_chamber_*`, not `muffler_restriction`.** The
  restriction is still only a pressure-loss term: the physical exhaust branch
  never reads it, never reads `openness`, never runs the FDN. What silences is
  the expansion chamber (`ExpansionChamberMuffler.hpp`), driven by the chamber
  diameter and length. Zero on either means "no chamber" and the element is an
  exact through-connection — verified in the delivered render, not just in the
  test: the Merlin measures +0.00 dB in every band. **Do not put broadband loss
  back inside that element.** It sits in the collector-outlet feedback loop, so
  a couple of dB per traversal compounds and collapses the low-frequency
  resonance; an earlier absorption term cost the EJ25 11 dB at its rev-range
  fundamental, and loudness normalisation then exposed the renderer's own
  high-frequency floor, which looked exactly like the element generating hiss.
  See `docs/thermoacoustic-architecture.md` §17 and §19.
- **A cycle average cannot tell a healthy engine from one filling with hot gas.**
  `EngineLabPhysicsPerfHarness --filter X --trace <rpm>` prints one cylinder's
  gas exchange every ~2 deg of crank, with the charge state on *both* sides of
  the intake valve. Filling failures are density failures, and they are invisible
  in pressure: the LS3 read a healthy 96 kPa in the intake runner while the
  charge sat at 333 degC, which is 2.6x too little mass. Read `irt_c` against
  `cyl_mass_mg` before blaming a valve or a duct area. The fine step changes the
  substep structure, so take trends from it and absolutes from the swept CSV.
- **A 0-D cell in through-flow must not exceed its own continuity velocity.**
  `injectJetMomentum` used to *add* `movedMass * v_jet` on top of the momentum
  advection `transfer()` already does; a runner passes several of its own masses
  per valve event, so the increments accumulated to ~3x the physical velocity.
  That inflated `dynamicPressureKpa()`, which then opposed the plenum-to-runner
  refill, pulled the runner below ambient, and made every IVO revert 1000 degC
  cylinder gas into the intake. Naturally aspirated engines lost two thirds of
  their air; the supercharged V12 barely noticed, which is exactly why it was
  the one engine that already sounded right. The jet term now *relaxes* each
  cell toward the continuity velocity. `EngineLab.PhysicsRegression` gates it on
  continuity, never on a simulator output. See `docs/physics-audit.md`.
- **The realtime bottleneck is the physics thread, not the audio callback.**
  Measured on a recent 8-core laptop: the callback uses 15-33% of a 256-sample
  budget, while the 240 Hz `EngineRuntime` loop misses 35% of its deadlines on a
  V8 at 6500 rpm (17.5 ms worst lateness). Nothing is dropped, but the telemetry
  that is the exhaust chain's only excitation arrives late and jittery — which is
  why a heavy engine's render is not reproducible run to run. Optimising the
  audio path is aiming at the wrong thread. Profiled, the binding case (V8 at
  high rpm, ~99% of budget) is 41% per-cylinder physics, 29% exhaust FV network,
  30% unparallelised "rest"; the V12 is the opposite (52% network). And **never
  compare a perf CSV across an exhaust-geometry change** — doing so once put the
  V12 at 158% of budget in these docs when it is at 77%. §18 has the numbers.

`docs/realtime-audio.md` and `docs/custom-exhaust.md` document the audio and
exhaust models and the corrections already made — read them before touching those
areas.

## Two concrete traps that cost time here

- **CMake target names, and the stale-binary trap.** `cmake --build --target X`
  with a wrong `X` fails with `MSB1009: project file does not exist`, builds
  nothing, and then `ctest` happily reruns the *previous* binary — so a test can
  appear to pass (or fail) against code you did not compile. The test executables
  are `EngineLabCoreTests`, `EngineLabExhaustTests`, `EngineLabPhysicsRegressionTests`,
  `EngineLabCombustionPhasingTests`, `EngineLabRealtimeRegressionTests`,
  `EngineLabComparisonHarness`, `EngineLabAudioRenderHarness` (note: not
  `EngineLabTests`). Do not filter build output so narrowly (`error C...`) that
  you hide an `MSB` error; confirm the target actually relinked.
- **Non-vacuous tests.** After adding a regression test, verify it fails without
  the fix — but rebuild the *correct* target first, or you will be testing a
  stale binary and conclude wrongly.

## Physics vs audio priority

The exhaust/audio path is the maintained priority. Physics-loop changes
(combustion phasing, legacy back-pressure) affect all catalogue engines and
cannot be validated to a shippable standard without dyno-curve references, so
they are deferred unless explicitly requested. `EngineLab.CatalogPhysics` checks
that engines run, stay finite and stable, and produce non-silent audio; it does
*not* assert absolute torque/power, so passing it is necessary but not sufficient
for a physics change.
