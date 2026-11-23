# EngineLab — guide for agents

**Read `VISION.md` before any task.** This file says *how* to work; `VISION.md`
says *why* and *towards what*. If they conflict, `VISION.md` wins.

This file is deliberately short (ceiling: ~200 lines). To add a rule, remove or
merge another one. The project is English-only: code, comments, UI, docs and
commit messages.

The measurement history to September 2026 (audits, plans, old agent notes; in
French) is in the git tag `archive/docs-2026-09` under `docs/archive/` -- `git
show archive/docs-2026-09:docs/archive/<file>.md` reads one. A source to
consult, not a set of instructions.

## Current priority

The only place where the priority is written. Do not work on anything else
without the owner's explicit agreement.

**The "Test cell" interface, owner go 2026-11-23 for the whole plan**
(`feat/test-cell-ui`, off `main`; the web mockup is `ui-mockup/`, untracked):
step 0 a before-reference (key map and command inventory as tests, window
captures, in-app real-time factor / 3-D fps / message-thread CPU through
`EngineLab --ui-bench`, harness checksums); then Theme and fonts, widgets, top
bar (lamps; menus Design / Tune / Test / Listen: propose the split to the
owner), control desk, cluster, side panel, 3-D overlays, x-ray render, every
secondary window, docs. The 2-D cutaway takes the new colours and stays light
for small PCs. Strobe (picture only) stays apart from real slow-down. No
regression: physics and sound bit-identical, real-time factor and fps within
the interleaved bar, every command still reachable, 1280 x 720 to 2560 x 1440
and 125-150 % scale readable. One release at the end, no intermediate one.
**Done 2026-11-23** (journal 2026-11-23): menus split as proposed and
accepted; unpushed, unreleased, unmerged; waits on the owner.

Done before it: **the audit register of 2026-11-16** (`docs/audit.md`, 90 findings; owner
go 2026-11-16 for lots 0-4 only, autonomous), in this order: (0) a clean
base on `fix/audit-2026-11-16`; (1) crashes and the build: PLAT-01, DR-03,
PG-07, DR-04, A2-07, APP-12, PH-02, RT-05, RT-08; (2) lost user data:
DR-01, APP-01, DR-09, UA-05, UA-06, A2-01, DR-02, DR-06, DR-10, RT2-01;
(3) the instruments: UA-08, V01-V04, TL2-01..04, T2-01, TEST-01/02, RT-01,
RT-02, RT-06, RT-07, A2-06; (4) the app against the running engine: UA-01,
APP-02..06, EXHAUST-01, PH-03. Physics and default sound untouched (the
deterministic harnesses prove it). Lots 0-4 done (2026-11-16). Owner go
2026-11-16 to continue: (5) physics, each change measured before/after:
PH-01, PH-04, PH-06, PG-02, PG-04, PG-05, PG-06, PG-08, DR-07, DR-08,
EXHAUST-02 (done 2026-11-16). Owner go 2026-11-16 for (6) sound, in this
order: A2-02, A2-04 (default sound unchanged, proven bit-exact), then A2-08 +
PG-01, A2-05, A2-03, each measured on the catalogue; a negligible spectral
change passes, a clear one waits on an owner A/B pair (on `exp/audit-sound`,
off the fix branch). Lot 6 done 2026-11-16; A2-08, the one clear change, kept by
the owner after its A/B (2026-11-17). Lots 7-8 done, released 0.10.0
(2026-11-17). Owner 2026-11-17: PH-05, V07/RT-04 and V05/V06 stay as they are
(not to be touched); K01-K03 analysed (journal 2026-11-17). Owner go
2026-11-17: fix K01-K03 and the probe's missing table, on `exp/k-gaps` (K01
kept after an A/B; K02/K03 wait on reference data). Owner go 2026-11-18:
the 3-D layout's envelope and clean tubing (no pipe, collector or turbo
through an engine or its gearbox, the X12-2's 12-into-1 kept, bent first
guesses, a collector placed by its primaries' length; drawing only; journal
2026-11-18). **Closed by the owner 2026-11-18**, merged into `main` with
K01 (not pushed). Owner go 2026-11-18 (exhaust placement "catastrophic"):
the 7-step plan on `feat/exhaust-corridor` -- a plausibility gauge, the
vehicle's corridor, collectors down then back, mirrored banks, broken
tails with collars (drawing only; journal 2026-11-18). **Validated by the
owner 2026-11-19**, merged into `main`. Owner 2026-11-19 (H-pipe setup
"horrible"): + H-PIPE / + X-PIPE in one click, one-bank H/X in New engine;
released 0.11.1 with both (the 0.11.0 tag failed GCC). **Owner go 2026-11-22**
(`feat/odd-v-throttles`, off `exp/two-stroke-parallel-pass`): a radial
two-stroke's carburettors drawn one a cylinder, and refused in New engine
(a radial's one crankcase cannot pump); V engines with an odd count (the
lone cylinder centred, Dual = 3-1 one side, 2-1 the other on a V5); one
throttle or one a cylinder (individual throttles), measured before exposed.
Few builds and tests: only those that prove a step. **Done 2026-11-22**
(journal 2026-11-22), unpushed, unmerged; waits on the owner.

Done before it: **the owner's report of 2026-11-14** (real-time regression,
gearboxes, 3-D mechanism and exhaust, the active valve hidden on a
two-stroke), **the owner's plans of 2026-11-13** (an Equipment window,
tuner group pages, anti-lag by feedback, a pass-by run) **and 2026-11-11**:
(1) the turbo runaway on a lowered stoichiometric AFR; (2) the exhaust
designer's edits; (3) VTEC; (4) one engine file with its ECU maps; (5) menus
inside their window; (6) a loop pack for BeamNG; (7) an active exhaust valve;
(8) a trackside microphone; (9) a nitrous kit; (10) physical anti-lag. Every
new feature is off by default. Refused: macOS, automatic spark calibration,
performance timers. Open: a lean dyno (crank/wheel, maker points), gamepad
and wheel input. Owner: run a full build or `ctest` only when it is really
needed.

Released before it, up to 0.9.1: the history is in `CHANGELOG.md`. 13B power
is a named exception; so is the flat-6's early-blip dip (0.70 x idle, chaotic
in the start, journal 2026-11-05). A two-stroke diesel was never asked for
(it needs a VISION.md amendment).
Standing rule: no limit on what a user
may build; an engine that needs a faster PC than the reference one is allowed.

The plan of 2026-10-26 closed on 2026-10-28, released 0.5.0; the owner
accepted every outcome and every open point below.

What the plan settled (journal 2026-10-28):
- **Base**: the reference PC (i5-10600) builds on BuildTools 14.44; VISION.md
  has no pilot engine.
- **Two-stroke model**: road designs within 2 % of their makers at the 25 mm
  mesh, 11-15 % below at production (accepted); sport designs keep their gap;
  Blair's friction; the reed stays a perfect check valve until a measured gap
  asks for its petals. The RD350's slow coast to idle is a named exception in
  `IdleStabilityRegression` (accepted; its 0.030 kg m2 inertia is unsourced).
- **Optimisation** (quality bar for any later one: bit-exact first; otherwise
  every dyno row within 0.1 % torque and power, `held_rpm_stddev` and every
  catalogue idle unchanged, audio octave bands within 0.1 dB, Geometry
  `shape` at noise level): the pool counts the processors the affinity
  allows; the half-reserve holds at 4 processors; no lead left inside the bar.
- **Knock**: the physics stays; `ignition.knock_limit_curve` holds the map
  under the measured limit (the designer measures it at build).
- Known and accepted: H1 low-speed hunting. The `UserDynoRampCP2Complete`
  start loop had the absorber-inertia signature fixed in 0.7.3; a dyno start
  that cancelled itself was fixed in 0.7.4 and 0.7.5 (journal 2026-10-30).

**Closed**: Linux (2026-10-20, `docs/platforms.md`); editing on the engine
(2026-10-23); backfires (0.4.2, Audio workshop only, off on every load; known
limits in `docs/physical-afterfire.md`, reopened only on a measured lead).

**Sound leads, unverified**: the ~3.8 kHz ceiling
(`docs/physical-afterfire.md`) and the plane-mode cutoff
(`docs/thermoacoustic-architecture.md` 25.2), judged against any real
recording at a known speed (`docs/recording-protocol.md`). No fix may require
a per-engine recording. The ACME single's 6-9 dB radiated excess was the
audio network's cone sections gaining power, not missing damping (journal
2026-11-15): a delay line's two ends must share one admittance.

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
ctest  --test-dir out/build/windows-vs2022 -C Release -j 4         # all tests (~18 min)
cmake --build out/build/windows-vs2022 --config Release --target package   # release zip
```

Linux is never the reference: the `linux-gcc-release` presets build and test it,
and `ctest` wraps the GUI test in Xvfb itself (no `xvfb-run` prefix needed).

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
- `scripts/vsenv.ps1` pins the toolset: since 2026-10-20 **BuildTools**, MSVC
  **14.44**, so no figure from before that date is comparable. If CMake picks up
  VS 18 instead, reconfigure with `-DCMAKE_GENERATOR_INSTANCE` at the
  BuildTools root (and clear `_deps/*`).
- **Stale binary trap**: a wrong target name (`MSB1009`) compiles nothing, and
  `ctest` reruns the old binary (`EngineLabCoreTests`, `EngineLabExhaustTests`,
  `EngineLabPhysicsRegressionTests`, `EngineLabCombustionPhasingTests`...).
  Check that the target was actually relinked.
- A new test must **fail without the fix** (after rebuilding the right target).
- A `ctest` that prints nothing after `Start N:` is **stuck**, not slow: judge a
  running binary by its CPU delta, never by a file's mtime (stdio to a file
  flushes in 4 KB blocks, so a CSV lags the work by minutes). A very short time
  can be a failure (`EngineLab.Core` passes in ~43 s on 14.44, aborts in ~8).
  The whole suite is 95 tests in ~53 min serial, ~18 min at `-j 4` (the
  wall-clock dyno tests are `RUN_SERIAL`; `IdleStabilityRegression`, 10-14
  min, is the floor). **Never run it beside a harness or sweep** (a WSL suite
  included: shared CPUs): `CoreTests` waits on wall-clock deadlines nothing
  scales by load, so a loaded host invents failures. A failure seen only at
  `-j 4` is rerun alone before it is believed.
- **Release**: bump `project(EngineLab VERSION ...)`, date the `[Unreleased]`
  section of `CHANGELOG.md`, then push the tag `v<version>`: `release.yml`
  refuses a tag that does not match the CMake version.

## Instruments

`docs/tests-and-validation.md` (Instruments) describes every tool and its flags.
Three things about them are not in a tool's name.

**Deterministic, so comparable between commits**: `EngineLabAudioRenderHarness`,
`EngineLabGeometrySensitivityHarness`, `EngineLab.CombustionPhasing`,
`EngineLabDriveScenarioHarness`, `EngineLabDynoSweepHarness`,
`EngineLabIntakeDuctBench` (a bit-exact checksum: unchanged = physics
untouched). `EngineLabRealtimeBudgetHarness` is not, and is **only valid
interleaved within the hour**: a control engine the change cannot affect,
alternating which variant runs first, the first run discarded, the minimum of N
taken, and `--free-run` or the factor caps at 1.0.

**Flags whose absence invalidates a run**: `EngineLabAbClipRenderer` needs a
level per segment (one loudness over idle + full load leaves the idle ~23 dB too
quiet); `EngineLabAfterfireHarness` needs `--liftoff-rpm` (at 9,999 rpm pumping
masks everything); `EngineLabPhysicsPerfHarness` needs `--idle` beside
`--trace 1` (`--trace <low rpm>` alone is full-load lugging, not an idle).

**Flags that separate a real effect from an artefact**:
`EngineLabDynoSweepHarness` `--crank-degrees`, `--coupling-us` and
`--frame-hz` resize the mechanical substep, the exhaust flush clock and the
240 Hz outer frame, and a gas-dynamic feature stays where it is when any one
changes (sweep mode only: `--reference-file` ignores all but `--frame-hz`); `EngineLabGeometrySensitivityHarness` says
whether an exhaust change moved the timbre (`shape`) or only the level
(`level`).

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
- **A test's absorber acts once a frame on the frame before's speed**: stiffer
  than half the shaft's inertia a frame (N.m per rad/s) it brakes all or
  nothing, and the mean torque is the swing's (a 25 cc single read 16 bar).
  `state.torqueNm` read once a frame aliases a single's firing (80 Hz at
  9,600 rpm): average the absorber's torque.
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
- **The mean hides a hunting engine.** `torque_nm` is a sample-window average
  and stays smooth while the engine swings 25 % cycle to cycle.
  `held_rpm_stddev` is the column that says the settled engine held, not
  brake-torque spread, which a twin's within-cycle crank ripple inflates (a
  healthy four-stroke control reads 26 % spread at 4.1 rpm of scatter).
- **Idles are fragile attractors**: a one-ULP equivalent change can stall an
  engine. Do not revert the optimisation: fix what makes the idle so sensitive.
- **Wrapping phases**: a crank event is a remaining distance, not an absolute
  angle. An accumulator must not receive `forwardPhaseDegrees` without a guard
  (at most 180° per sub-step). A bias added before `fmod` must be whole
  cycles: 1,440 is 360 modulo a rotary's 1,080 (its ports ran 360° off).
- **Conservation is not acoustic passivity.** A flow bias must be a state,
  never a derivative of the flow it drives. Check a closed perturbation and a
  finite plenum; refine mesh, FV time step and reservoir refresh independently.
- **Thread barrier**: count the workers, not the items (an 8 h 52 hang already
  happened). Signature: a single core at 100 %, the others idle.
- **`--exhaust-cell-mm` saturates at 25 mm** (`buildPhysicalExhaustNetwork`
  clamps to [25, 600] mm; the per-duct cap then bites at 64 cells), so a finer
  request is bit-identical. The two-stroke mesh converges near 12 mm and
  production (100 mm) is 18 % below converged -- a real-time budget limit, not
  a physics one. Do not read a delivery hump off `max(delivered_ve)`: it picks
  the lugged through-flow. A four-stroke's idle rides on the 360 mm cells within
  0.3 m of the outlet: ambient oxygen diffuses to the valve (sport 125 2,074 rpm
  for 1,300; refined there, 1,325, but the sound moves: `exp/outlet-mesh`).
  An open-loop carburettor cannot trim that oxygen out: a designed carb bike
  I4 hunts and stalls, and idles at 1,280 rpm on a 25 mm mesh.

## Refuted — do not propose again without new evidence

Each measurement is in `docs/journal.md` or the docs archive (tag
`archive/docs-2026-09`).

- **Exhaust**: the "unified duct" rewrite (the 300 mm mesh carries the audible
  geometry, 12-14 dB); CFL 0.9; MUSCL-Hancock; third-order faces; one-sided open
  ends; deepening the muffler's single chamber (16 dB notch on the EJ25
  fundamental); the exhaust valve's flow as a four-valve superbike's missing
  power (Cd 0.90 and plateau 1.5 give the designed CBR +3 %), or as the
  turbos' high-speed pumping (plateau 0.65: at most +0.8 %; the fitted
  turbine area is what sets it, journal 2026-11-15); the outlet's edge
  loss, decay along the pipe or the coupling cadence as the BSA A50's
  radiated deficit; an outlet scaling its reflected pressure by
  (1-M)/(1+M), the no-Kutta limit (measured pipes follow Munt with a Kutta
  condition, |R| -> 1; journal 2026-11-16).
- **Performance**: fork-join per cylinder with woken threads (the resident pool
  runs the pass since 2026-10-20); AVX/AVX2 (dependent-division latency); a
  telemetry cache for listening; state-variable valve-flow reconstruction
  against clicks; whole-project LTO (bit-identical, within noise); `/MP2` on
  the JUCE project in CI (C1060 beside the app's LTCG link); using the
  half-reserve at 4 processors (3 workers drop audio, 2 cost the 2JZ);
  coarse closed-valve steps (x1.05, a sound
  source); a 150 mm four-stroke exhaust mesh against the idle's outlet air
  (Hayabusa, CP4, 13B, 2JZ fall under 1.0); a rotary's crank step at 3 deg, its face's 2 deg of 720 (13B
  torque +1.45 % mean, +4.6 % at 4,000 rpm against 1.5 deg; 2 deg +0.34 %).
- **Two-stroke**: a 50 mm mesh; coupling the exhaust every substep beyond
  BDC-to-transfer-close, a characteristic valve, finer transfer coupling (none
  of them real time); un-gating exhaust oxidation before scavenging is right
  (torque 41.2@9751 -> 36.5@7248 and egt *down*: a 25-55 % short-circuit burnt
  at the port mouth returns oxygen-free gas, so trapping rises while torque
  falls); lengthening the pipe (Blair's 500 m/s and the designer's tuned length
  are right; x1.300 is +33 % overpowered, and the curve's TWO humps are both
  the pipe's -- they move as 1/L, so the trough between them is the chamber's
  wave structure, not scavenging); the carburettor's extra roughness as its
  pilot, its richness or mixture noise (at equal lambda it is fuelling the
  crankcase); a transit-vs-window scavenging race (monotonic in rpm by
  construction); a scavenging bias from ONE pressure sample at port opening (a
  hunting limit cycle over 4,500-6,250 rpm: integrate the front); "the trapped
  residual is too low" (a unit error); "the pipe gas is too hot" (418-553 C,
  under a real pipe); the designer's redline from real piston speeds (road
  14.8, sport 17.8 m/s: torque falls as the pipe follows the rpm). A holding band locked to crank speed is
  first a frame-rate alias: `DynoSweepHarness --frame-hz` moves it.
- **Intake**: four inertance formulations (unstable or out of phase); the charge
  bias as the MUSCL limiter, the finite plenum, wall heat or valve linearisation
  (it was the valve nozzle boundary, Courant-dependent); front-loaded or
  trajectory-resolved forward delivery; individual throttles as a small 0-D cell
  per cylinder (-30 % WOT torque at 6,500 rpm, orifice dump loss: the plate
  belongs inside the 1-D runner); the plenum's inlet area as the in-runner
  plates' WOT loss (it was the orifice damping the ram wave, journal
  2026-11-22); Mazda's published 13B port areas as its
  high-rpm gap (+0.4 % at 8,500), or its VDI (no fixed runner passes +1.6 %).
- **Backfire** (journal 2026-10-24/25): "1-D cannot carry a front" (it rises in
  0.2-0.8 ms); "the heat is too slow to become a wave"; the EVO timing as
  what silences a pop (the 100 kPa source bound did clip it: now 1 MPa); slugs collecting in the muffler (phi
  0.03, <= 50 J). With a late spark every cycle burns its own charge, at
  100 % duty, whatever the variant: pockets at the unburnt zone's temperature
  or born lit, a 950-1,100 K ignition, slower autoignition, a carried flame, a
  0.10 or temperature-dependent lean limit, a 50 mm mesh, burn COV, a hot wall,
  flame contact between cells or pockets. Misfires give no pop with 10 ms
  pocket mixing. "The voices add nothing" and "a pop needs ~220 J in 1 ms"
  were the lines' amplitude warp deleting 4-9 dB of each pop (fixed).
- **Combustion and fuel**: closing cyclic variability through dilution (it
  exists, 1.7-12.9 %); continuous fuel below ~45 % of demand (below the lean
  limit, packets burn); crediting port vapour to the port-injection pulse (a
  stationary reservoir, delivered/demanded 0.47-0.77: 8 DFCO-recovery variants
  fail, 5 more trade the lean first cycle for rich or misfires); seeding the
  high-load correction cell from the idle cell (saves a start at 4 s but stalls
  10 engines of 14 after 25 s of idle) and an asynchronous pulse on tip-in;
  Woschni's combustion term on spark engines against the over-efficiency
  (BSFC +2-7 %, EGT down, 13B -15 %: the cause is the gas's gamma); a 3 s
  post-resume lambda-learning hold against the flat-6's early-blip dip (LS3,
  Big Twin, 13B fall under their gates); Poulos-Heywood's S_L dilution as a
  multiplier on the whole turbulent flame speed (2JZ power -18.7 % against
  its maker; journal 2026-11-14). As S_L^(2/3) on the turbulent part (Hires
  et al., SAE 780232) it holds full load but loses the catalogue's idles at
  their 30 % residual (journal 2026-11-15, `exp/hires-dilution`): it needs
  an idle recalibration the owner decides on.
- **Knock** (journal 2026-10-13, -22): end gas at the MEAN temperature, or at
  constant cv (1,000-1,200 K); proportional or integral control; Kalghatgi's
  octane index (boost gains 1-3 octane); an ECU boost cut on knock (I5 363
  N.m); the I5 at 2.1 bar; Douaud-Eyzat's prefactor at x1.5 under the
  thermally perfect gas (NA maps reach their limit, 2JZ/EJ25 makers' points
  -4 %; journal 2026-11-03). Douaud-Eyzat over-predicting knock under high
  boost: as used (x2, from the spark) its delay at 118-250 bar is 1.4-2.7x
  Hoepke's and 2.5-2.8x McKenzie-Cheng's (journal 2026-11-14); a weaker
  pressure exponent has no consensus (1.34, 1.73, 2.37 in three sources).
- **The other simulator's ingredients, one at a time** (journal 2026-10-13/14):
  output stage, pressure-release outlet, valve-area noise, mild room IR -- none
  kept (blind listening or measurement). Its fine mesh is not shown to matter.

## Document layout

`VISION.md`: the intent; this file: the method (`AGENTS.md` points here).
`docs/*.md`: how each part works, kept up to date with the code (index:
`docs/README.md`). `docs/journal.md`: measurement results, short entries.
`CHANGELOG.md`: user-visible changes per release.
