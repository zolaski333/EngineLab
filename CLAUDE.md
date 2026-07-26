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
- **A WOT "hold point" was not held, and the top of every sweep was the rev
  limiter.** Two independent defects in the absorber that all four WOT
  instruments copied. First, integral gain 1.20/s at 1/240 s steps cannot wind up
  inside a settle window: a steady 7% error reaches only ~0.25 of full load, less
  brake torque than a 2 L engine makes at 7000 rpm, so above ~5000 rpm the engine
  drifted upward instead of being held — a 6500 rpm target was measured at 6958,
  and the 15% tolerance passed it. Gain is now 12.0 with a 1.0 ceiling and the
  gate asserts 2%. Second, `maxRpm = min(redline, revLimit)` put the last target
  *on* the latched-with-hysteresis rev limiter, and `flameEvents_[i] = {}` on a
  missing spark zeroes the published combustion efficiency: the same 7000 rpm
  point read combEff 0.167 / IMEP 4.8 bar on the limiter and 0.911 / 13.0 bar
  with the limiter moved to 9000. Both instruments now stop at
  `0.95 * min(redline, revLimit)`. **Never trust a high-rpm number from a CSV
  produced before this**, and recognise the signature: the last row of an engine
  shows absurd torque (the 2JZ read 2.15 Nm). I reported a "combustion cliff"
  twice from these artefacts; both are retracted in `docs/physics-audit.md`.
- **VE here is a trapping figure; `deliveredVolumetricEfficiency` is Heywood's.**
  `volumetricEfficiency` is oxygen-equivalent air present at IVC, which is the
  right thing to meter fuel against but is not what a dyno's air meter reads. The
  two agree to under 1% on a healthy engine, and their ratio going *above* 1 is a
  reliable detector of incomplete combustion (unburned oxygen stays in the
  chamber and inflates the trapped figure while delivery tells the truth) — that
  is how the limiter contamination above was found. Also note
  `CylinderState::residualGasFraction` is the *instantaneous* burned fraction
  despite its name (≈1.0 just after combustion); the residual is
  `residualGasFractionAtSpark`.
- **A grep by field name proves nothing about an aggregate initialised by
  position.** `FlameConditions::burnedGasFraction` appears nowhere outside a unit
  test, so I concluded the flame model's residual-dilution term was dead. It is
  not: it is filled positionally in `EngineSimulator.cpp`. The same trap bites in
  the other direction — `state_.cylinderStates[index] = { ... }` reaches as far
  as `misfiring`, so inserting a `CylinderState` member above that point silently
  shifts every later field. Add members below the marker comment there and assign
  them by name.
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
- **An open pipe end is a reservoir, not a neighbouring cell.** The exhaust
  network used to take a Riemann flux between the last duct state and a cell of
  ambient air, so the exhaust had to shove a semi-infinite column of cold dense
  gas aside and the flux was capped by the *ambient* impedance. The tailpipe sat
  79 kPa over ambient while discharging at 68 m/s where free expansion gives
  699. Widening primaries, collector, outlet and exhaust valve each moved VE by
  0.02-0.04; the boundary moved it by 0.08 and dropped EGT 143 degC into the
  literature window. It was also reflecting with the sign of a *closed* end, so
  fixing it is deliberately a voicing change. Terminal openings now impose the
  reservoir pressure at the exit plane via the outgoing invariant, used as a
  Riemann ghost cell. Do **not** take the boundary state's physical flux
  directly, and do **not** continue the invariant across the contact on
  backflow: both stalled every catalogue engine but one. See
  `docs/physics-audit.md`.
- **A flow bias must be a state, never a derivative of the flow it drives.**
  `ConservativeGasSystem::flow` already sets mass flow from ΔP through an orifice,
  so a `biasKpa*` computed from that flow's own `du/dt` over-determines it, with a
  feedback gain ~`rho*L/dt` that **grows as the substep shrinks** — refining the
  timestep or shrinking a coefficient makes it worse, not better. The intake
  runner's textbook inertial reaction `-rho*L*du/dt` was measured alternating
  between the ±clamp rails on consecutive substeps and stalled eleven catalogue
  engines outright. `rho*u^2/2` from the same velocity is stable (0% saturated) —
  but was *still* wrong, on phase. Four ram formulations have now been measured and
  refuted; read `docs/physics-audit.md` "L'inertance de runner" before proposing a
  fifth. What remains is a relaxation inside the flow law, not an added pressure.
- **`--trace <low rpm>` is WOT lugging, not idle.** The perf harness's dyno
  controller commands whatever load holds the target rpm, the ECU answers that load
  by reopening the plate, and the manifold lands within 1 kPa of the WOT value
  (98.3 vs 99.1 kPa). `--throttle` does not fix it. Use `--trace 1 --idle`, which
  mirrors `EngineLab.IdleStabilityRegression`'s phase 2 (shut throttle, no load) and
  produces real vacuums — CP4 33 kPa, LS3 17 kPa. Any idle claim measured otherwise
  is void; one such claim in `docs/physics-audit.md` had to be retracted. Note the
  Radial R5 idles near ambient (95.6 kPa) and is the one engine sensitive to intake
  terms, but its baseline is healthy, so a failure there is the change's fault.
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

- **An idle failure is usually not caused by the commit that exposed it.** The
  catalogue's idles are marginal attractors and the simulator is deterministic,
  so a change that is only *algebraically* equivalent still moves them. Measured:
  after the solver-cache commit the Radial R5 stalled outright and the Big Twin
  rang at sigma 57.7. Tracing both engines before and after and diffing line by
  line, the runs first differ in the **last printed digit** of `air_mg` (636.4 vs
  636.5 at t=0.40 s; 3782.8 vs 3782.9 at t=0.50 s) — a ULP, amplified over
  seconds into opposite outcomes. **Do not hunt the ULP and do not revert the
  optimisation**: bisect to confirm the boundary, then fix whatever makes the
  idle that sensitive. Here that was deceleration fuel cut firing 0.3 s after
  catch, during the after-start flare, with an empty port film. Corollary: a
  green idle run proves less than it looks, and "engine X now fails" after an
  unrelated change is the expected symptom, not a mystery.
- **Bisecting is cheap here; guessing is not.** `cmake --build ... --target
  EngineLabIdleStabilityRegressionTests` relinks in ~36 s even across a
  `EngineSimulator.cpp` change, and the tree is normally clean, so
  `git checkout <sha>` + build + run is a few minutes per point. Two bisect
  points replaced an afternoon of reading diffs and killed three plausible
  hypotheses (flame ceiling, ECU load axis, injection model) that a code-read had
  ranked highly and that measurement showed to be diesel-guarded or inactive at
  idle.
- **`idleAirOpening` equal to `postStartAirOpening` in an idle trace is not a
  duplicated column.** `idleAirOpening = max(postStartAir, governor, dashpot)`,
  so while the after-start floor owns the actuator the two are equal *by
  construction* — and that is the interesting reading: the governor has no
  authority, and the anti-windup will not let it integrate down while the floor
  wins. Whole seconds of a start transient can pass with the PI loop a spectator.
- **`EngineState::exhaustPressureKpa` is not back pressure.** It is the `max`
  over cylinders of the *instantaneous exhaust runner* pressure — a blowdown peak
  envelope, not a collector mean. The `exh_kpa` column of every swept CSV reads
  like a mean and is not one. Any back-pressure or pumping argument built on it
  is void; one was, and was withdrawn.
- **Cell-centre primitives are not a profile at the shipped mesh.**
  `targetCellLengthM = 0.300` with `minimumCellsPerDuct = 1` gives a 760 mm
  primary three cells, and with the high-order reconstruction `rho*u` varies 2.4x
  along a *constant-area* duct in a state that is provably settled (bit-identical
  at two settle times) and globally conservative. Trust fluxes and boundaries,
  never cell centres. Reading that variation as "not converged" cost a retracted
  claim.
- **The exhaust coupling averages the cylinder state, then takes one flux from
  the average.** The interval is `min(250 us, 1/(16*firingFrequencyHz))`. Because
  the flux is concave in the pressure difference, averaging the state first
  under-predicts transfer wherever the state moves inside an interval — worst
  during blowdown. Measured with `EngineLabGasExchangeTests --oracle-coupling`
  (which advances the network every substep): worth 23-29% of the exhaust-stroke
  pumping loss above 2500 rpm. Generic lesson for this codebase: any state
  averaged over an interval before entering a non-linear law biases the result in
  the direction of the curvature.
- **An unrestricted parallel Release build can exhaust the MSVC compiler heap**
  (`C1060`) on the three large LTO translation units (`EngineLabCoreTests`,
  `EngineLabAbClipRenderer`, `EngineLabAudioAbHarness`). They build fine with
  `--parallel 1`. It is a build-memory limit, not a source error — do not go
  looking for a code cause.

`docs/realtime-audio.md` and `docs/custom-exhaust.md` document the audio and
exhaust models and the corrections already made — read them before touching those
areas. `docs/rework-validation-log.md` is the running log of the 2026-07-26
rework: measurements, and the hypotheses that were tried and refuted.

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
  stale binary and conclude wrongly. Existing green tests deserve the same
  suspicion: `EngineLab.Core`'s "zero valve lift must result in zero volumetric
  efficiency" was green for two unrelated wrong reasons at once — the config
  zeroed only `config.camshafts` while `activeCamshaft()` prefers a **bank**
  camshaft (so the engine still had lift), and the assertion was then satisfied
  by the `rpm > 20` guard because the starter could not turn that engine at all
  (measured rpm 0.00). It only surfaced when an unrelated starter change made
  the crank rotate. **A config knob set at the top level may be shadowed
  per-bank or per-cylinder; and an assertion on a quantity that is force-zeroed
  below a threshold proves nothing until you assert the engine reached it.**
- **A gate that averages a fixed window cannot see an unsettled signal.** The
  idle gate measured t=10-14 s and eleven engines "passed" while all of them
  were ringing 130-180 rpm peak-to-peak; the verdict depended on the phase the
  window caught. It now also asserts drift (second-half mean minus first-half).
  Symptom to recognise: disabling either of two unrelated changes reproduces the
  same failure with near-identical numbers, while disabling both passes — that
  is one fragile attractor, not two causes. The simulator is deterministic
  (sequential runs are bit-identical), so such a pattern is never noise.

## Physics vs audio priority

The exhaust/audio path is the maintained priority. Physics-loop changes
(combustion phasing, legacy back-pressure) affect all catalogue engines and
cannot be validated to a shippable standard without dyno-curve references, so
they are deferred unless explicitly requested. `EngineLab.CatalogPhysics` checks
that engines run, stay finite and stable, and produce non-silent audio; it does
*not* assert absolute torque/power, so passing it is necessary but not sufficient
for a physics change.
