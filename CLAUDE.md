# Working on EngineLab

Guidance for AI agents. It is not a codebase tour — it is the set of things that
were learned the hard way here and are not visible from reading the code.

## Build and test

```
. scripts/vsenv.ps1                                                # EVERY call
cmake --build out/build/windows-vs2022 --config Release            # whole project
ctest  --test-dir out/build/windows-vs2022 -C Release              # all tests
```

Warnings are errors. A green build and `ctest` run are the bar for any change.

### Two build trees, and which one is authoritative

- **`out/build/windows-vs2022`** (Visual Studio generator) is **authoritative**.
  Every gate, every commit and **every performance number** belongs to it. All
  the reference figures in this document were taken there.
- **`out/build/ninja-release`** (Ninja + sccache, `cmake --preset ninja-release`)
  is the **inner loop only**: compile errors and targeted test runs. Both trees
  are pinned to the same toolset (14.34.31933) so a warning-as-error in one is a
  warning-as-error in the other.

**Never compare a timing between the two trees.** Different generator, different
link, different object layout. The realtime-factor drift warned about further
down is already enough to invent a regression that does not exist; a second
toolchain configuration is a new way to do it, and a much more convincing one.

### `. scripts/vsenv.ps1`

Dot-source it in every call that runs cmake/ninja/ctest — shell state does not
persist between agent tool calls. It imports `vcvars64.bat` pinned to toolset
14.34 and puts CMake, Ninja and sccache on `PATH`, replacing the manual prefix
that used to be required here. It deliberately does not use
`Launch-VsDevShell.ps1`, which goes through the broken `vswhere.exe` below.

Three things had to be repaired before Ninja would configure at all, and each is
invisible from the Visual Studio tree:

- **`CMakePresets.json` declared `"version": 6`, which needs CMake 3.25.** VS
  2022 ships **3.24** (VS 18 ships 4.2.3). So the presets file had never been
  readable by the CMake this project actually builds with, which is why every
  build order here is a raw path and never `--preset`. Lowered to 5; nothing in
  the file used a v6 feature.
- **`project()` had to list `C`.** JUCE declares `project(JUCE LANGUAGES C CXX)`,
  but that runs inside the FetchContent subdirectory scope, so the C rule
  variables never reach the top level. MSBuild picks the language from the file
  extension and does not care — the `windows-vs2022` cache has `CMAKE_C_FLAGS`
  but **no `CMAKE_C_COMPILER` at all**. Ninja resolves the compile rule at
  generate time and fails with `CMAKE_C_COMPILE_OBJECT` missing.
- **A compiler cache breaks Ninja's header tracking. Do not enable
  `ENGINELAB_ENABLE_COMPILER_CACHE`.** This one nearly shipped as an
  improvement, and it is the most dangerous kind of defect this repo can have:
  it makes the build *silently* wrong.

  Ninja learns which headers an object depends on by parsing `cl.exe`'s
  `/showIncludes` output. sccache does not pass that through. Measured, on
  `EngineTypes.cpp.obj`: with the launcher, `ninja -t deps` reports **`#deps
  0`**; without it, **`#deps 1`** naming `EngineTypes.hpp`. So with the cache on,
  editing a header rebuilds **nothing** — a full `ninja` after changing
  `EngineTypes.hpp` executed exactly one edge, the CMake regen. Neither
  `SCCACHE_DIRECT=false` nor a forced `SCCACHE_RECACHE=1` recovers it, so it is
  not a cache-hit artefact.

  That is fatal precisely where the cache was supposed to pay: a `git checkout`
  during a bisect mostly moves headers. The prize was real — 94.5 % hits and a
  **56.5 s** full rebuild against ~**474 s** cold — and it is still not worth a
  tree that can compile against a header it no longer matches. sccache stays on
  `PATH` and the option stays in `CMakeLists.txt` so the finding is
  reproducible; the presets ship it OFF.

  Two lessons generalise. **`CMAKE_CXX_COMPILER_LAUNCHER` is silently ignored by
  the Visual Studio generator**, so nothing wrapped that way can ever affect the
  authoritative tree. And **verify a new build tree tracks headers before
  trusting it**: build an object, edit a header it includes, and check that the
  rebuild is not a no-op — `ninja -t deps <obj>` must not say `#deps 0`. The
  symptom otherwise is the stale-binary trap below, but arriving without a wrong
  target name to explain it.

- **Build the Ninja tree at `-j 2`.** The `C1060` heap exhaustion documented
  further down is not specific to MSBuild: this machine has **15.9 GB for 12
  logical cores**, and concurrent `cl.exe` on the large translation units
  exhausts it. Measured, all failing on the compiler-generated constructor of
  `SpscQueue<CylinderPressureSample, 8192>`: `-j 6` fails, `-j 3` fails, `-j 2`
  completes with zero failures, and that same single object alone at `-j 1`
  compiles fine in **75.4 s**. So the limit is *cumulative* memory, not that one
  instantiation — a hypothesis worth refuting before touching `SpscQueue.hpp`,
  which is what the measurement did. Note the outlier for what it is: the mean
  compile here is **5.5 s** over 127 objects and that one is 75.

- The old manual prefix, if you ever need it without the script:
  `$env:PATH = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;$env:PATH"`.
- A **Visual Studio 18** Community install sits beside 2022, `vswhere.exe`
  returns **nothing** (the Installer's instance registry is damaged), and CMake
  therefore resolved the "Visual Studio 17 2022" generator onto
  `.../Microsoft Visual Studio/18/Community`. The build then fails with *"the
  instance is not known to the Visual Studio Installer, and no 'version=' field
  was given"*. The build tree was self-contradictory: `CMAKE_GENERATOR_INSTANCE`
  said VS 18 while `CMAKE_LINKER` still pointed into 2022's `14.34.31933`.
- Fix: reconfigure with
  `-DCMAKE_GENERATOR_INSTANCE="C:/Program Files/Microsoft Visual Studio/2022/Community"`.
  That is **not** enough on its own — each FetchContent sub-build
  (`_deps/juce-subbuild`, `_deps/juce-build/tools`, `_deps/nlohmann_json-subbuild`,
  `_deps/yaml_cpp-subbuild`) caches its own instance and must be retargeted too.
  Rewrite that one line in each `CMakeCache.txt` rather than deleting the caches,
  which would re-trigger the downloads. This keeps the original 14.34.31933
  toolset, so it is an environment repair and not a toolchain change.

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
- **`tools/RealtimeBudgetHarness.cpp`** (`EngineLabRealtimeBudgetHarness`) runs the
  real `EngineRuntime` thread, at real thread priority, holding each catalogue
  engine at a commanded speed, and reports the **realtime factor**: simulated
  seconds produced per wall second. 1.0 is healthy; below 1.0 the whole simulation
  is in slow motion. This is the only instrument that can see that failure —
  `PhysicsPerfHarness` measures CPU per step and cannot know whether the step fit
  in its slot, and the overrun counter says a deadline was missed but not by how
  much work. `--rpm N --seconds S --filter NAME`, and `--enforce F` makes it a
  gate. It deliberately does not report dropped telemetry: no audio thread drains
  the queue here, so that count overflows on every engine and would mean nothing.

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
- **One loudness gain over a clip that holds both an idle and full load presents
  the idle ~23 dB too quiet, and a listener will report that as an engine
  fault.** BS.1770 integrated loudness is set by the loud part, so a single gain
  landing a 15.3 s idle-to-limiter trajectory at −20 LUFS put the CP2's idle
  window at about −43. Measured: whole trajectory −21.55 LUFS, its `idle` segment
  −44.89, its `rev` segment −20.30. The first human pass reported "le ralenti est
  trop faible" for every engine and a large part of it was this. `AbClipRenderer`
  now renders ONE trajectory and cuts it into segments levelled on their own
  content (`listeningSegments`), publishing `level_error_removed_db` per pair.
  **Do not "fix" this by raising the idle in the voicing** — that fabricates the
  thing the correction exists to reveal, and the render must stay untouched (the
  trajectory still measures −21.55, unchanged). Same shape as the cycle-average
  and fixed-window-gate traps below: any integrated measure over a window that
  spans widely different levels reports the loudest part.
- **A reference recording must be windowed per CONDITION, not per file.** Real
  takes hold their idle and their rev at different offsets, so one
  `clip_start_seconds` cannot match both; pairing a simulated idle against a
  recording under load yields a confident verdict about nothing. Manifest schema 3
  adds `segment_windows`; a segment with no window gets NO reference and says so,
  and `--allow-unmatched-reference-window` is an explicit opt-in that marks the
  pair unpublishable. Corollary found the same day: a pair with an empty control
  side must be excluded from scoring, because a listener who graded that side
  graded silence — `scripts/listening.py` used to crash there, which was at least
  visible; averaging it in would not have been.
- **The listening iteration loop is bounded by header fan-out, not by physics.**
  Measured on the 12-thread desktop: one engine's 15.3 s trajectory renders in
  5.9 s (CP2) to 15.6 s (Merlin), and relinking `EngineLabAbClipRenderer` after
  touching an audio **`.cpp`** costs 11.0 s — so that loop is already 17-27 s.
  Touching `RealtimeEngineAudio.hpp` instead costs **115.1 s**. A telemetry cache
  that replays the physics into the audio path was scoped to fix this and would
  have addressed **none** of it; it was dropped before being written. What removes
  the build from the loop is making voicing *data* rather than code.
- **Never label structural modes `measured` because they sound plausible.**
  Schema 5 accepts sourced modal frequency, damping, mass, radiating area,
  radiation efficiency, force drive and signed participation per cylinder.
  Validation deliberately rejects a measured claim without modes/source or an
  incomplete shape. The shipping catalogue has no such dataset and therefore
  remains visibly `estimatedFamily`; use `EngineLab.StructuralNvh` and
  `docs/structural-nvh-configuration.md` before adding real survey data.
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
- **`cycle_variation_cov = 0` does NOT mean the cycles repeat.** Fifteen of the
  sixteen catalogue engines author zero, and reading that as "combustion is
  perfectly periodic, hence the idle sounds synthetic" is wrong — I proposed a
  dilution-driven variability closure on that basis and measurement killed it.
  Measured with `EngineLabCyclicVariabilityHarness` on the shipped catalogue,
  COV(IMEP) per cylinder about its OWN mean is already **1.7–7.0 %** at light
  load and **0.9–12.9 %** at WOT with no authored dispersion at all: the
  gas/film/wave/ECU coupling makes consecutive cycles genuinely different. The
  simulator is deterministic (two runs are identical) but it is not *periodic*.
  Three things came out of that measurement and all three are live: the ordering
  is **inverted** (seven engines are rougher at WOT than at light load, and it is
  not the absorber — held speed varies 0.10–0.48 % while the work varies 5–13 %);
  a **free idle is not a valid instrument for this** — its COV(IMEP) is the
  governor hunting, which read 90 % on the radial, so hold the speed; and a third
  finding, "the trapped residual is an order of magnitude too low", was **WRONG
  and is retracted** — see the next entry. See `docs/physics-audit.md`.
- **RETRACTED: the trapped residual is not too low. It is healthy, and the claim
  was a unit error on top of a wrong operating point.** I reported that
  `residualGasFractionAtSpark` reads 0.004–0.026 against the 15–25 % a
  production SI engine traps, and the claim reached three documents. Both halves
  were wrong.
  - **The quantity is not a residual gas fraction.** The mixture model carries
    four species — oxygen, inert, fuel, burned — and the nitrogen that arrives
    with the air stays `inert` forever, so `burned / total` counts the combustion
    PRODUCTS alone while Heywood's figure counts the whole trapped exhaust.
    With the default gasoline (`productMolesPerFuelMole` 17.0,
    `oxygenMolesPerFuelMole` 12.5) stoichiometric exhaust is 17 product moles out
    of 17 + 12.5 × 3.7619 = 64.0, so the field is **0.266 × RGF** — a factor of
    3.8.
  - **The low numbers came from the wrong point.** They were measured at a
    speed-held 10 % throttle condition, not a free idle.
  Measured properly with `--trace 1 --idle` on eight engines: the field reads
  **0.063–0.142**, i.e. **24–53 % residual**, at or above Heywood's ~20 % idle
  figure. At a WOT hold it reads 0.014–0.016 on the naturally aspirated engines,
  i.e. **5.3–6.0 %**, inside Heywood's 3–7 % band; the boosted 2JZ sits at 9.0 %
  at 196 kPa manifold, which is the right direction. The generic lesson: **a
  literature threshold in a doc comment is only usable if the comment states the
  units of the field it sits on.** That comment cited Heywood's percentages
  beside a mole fraction of products and cost a whole investigation; it now
  carries the conversion and the corrected thresholds
  (0.008–0.019 at WOT, ~0.053 at idle, failure above ~0.04 at WOT).
- **A crank EVENT is a distance, not an angle — an angle cannot cross the cycle
  boundary.** The spark schedule was latched at phase 540 as an absolute
  `sparkPhase` and disarmed at the 0 crossing, so its reachable set was the arc
  (540, 720]. But `minimumIgnitionAdvanceDegrees` is **−10**, reached by knock
  retard (`knockLevel * 12`) or the over-temperature pull, and that event lands
  at phase [0, 10) — *after* the boundary. The boundary reset consumed it first
  and the cylinder went dark: measured 0 sparks and 0 ignitions where a retard
  was commanded, not the low-torque hot-exhaust behaviour a retard should give.
  The schedule is now a remaining-travel countdown, which has no arc, and the
  leak the reset existed to stop is excluded by the validated ranges (advance
  [−10, 55], per-cylinder offset [−30, 30] ⇒ distance ∈ [95, 220]). Two things
  generalise. **Any "wait until phase X" written against a wrapping phase silently
  assumes X is inside the current wrap** — check that assumption whenever the
  target is computed rather than constant. And **an accumulator cannot be fed
  `forwardPhaseDegrees` unguarded**: it wraps, so one backward sub-step (start
  kickback is real here) reports ~720° of forward travel. A crossing test shrugs
  that off; an accumulator fires immediately. Credit at most 180° per sub-step.
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
- **The realtime bottleneck is the physics thread, not the audio callback**, and
  the failure mode is not a dropped frame — it is *slow motion*. `EngineRuntime::run`
  advances a fixed `1/240 s` of simulated time per iteration and sleeps to a wall
  deadline; there is no accumulator, and past four steps of lateness it sets
  `deadline = now` and discards the debt. So when a step costs more wall time than
  it advances, simulated time falls behind real time permanently. Measure it with
  **`EngineLabRealtimeBudgetHarness`**, which reports the *realtime factor*
  (simulated seconds produced per wall second) from the real runtime thread —
  neither CPU-time-per-step nor the overrun count can see this. On a 16-thread
  laptop at a 5000 rpm dyno hold, after the fork-join removal below: CP2 twin
  0.999, CP3 0.993, Hayabusa 0.803, K20A I4 **0.433**, Merlin V12 0.388, LS3 V8
  **0.357**. The V8 therefore runs at about a third of real time. Two user-visible
  consequences follow directly, and both were reported from the app before being
  measured: controls respond late in proportion to cylinder count, and the
  cylinder-pressure telemetry — the exhaust chain's *only* excitation — is
  produced far slower than the audio thread consumes it, so the biggest engines
  render nearly silent. **A silent V8 or V12 is a physics-thread symptom; do not
  go looking for it in the audio path.**
- **The sub-step cost is the 1-D intake network, by an order of magnitude.**
  **STALE — this split predates the multirate intake coupling and must be
  re-measured before it is used to justify anything.** The runner advance is now
  gated behind `flushIntakeNetworks` at a 400 us default
  (`intakeCouplingIntervalSeconds`, `EngineSimulator.cpp` ~1749-1790, called at
  ~2105 and ~2242), plus a forced flush at intake-valve closing — so the runners
  are no longer advanced twice per mechanical sub-step as described below. The
  percentages and the "~26 us / 30 mm cells" arithmetic belong to the earlier
  every-sub-step, 30 mm scheme. Do not quote them as the current profile.
  Measured per-block inside the sub-step loop (K20A / LS3 / Merlin / CP4):
  intake 1-D **83.5 / 81.9 / 83.6 / 75.3%**, per-cylinder physics 7.3 / 7.3 / 4.9 /
  10.3%, exhaust 1-D 4.2 / 7.1 / 8.9 / 6.4%, ECU under 0.6%. This **supersedes the
  41% cylinder / 29% exhaust / 30% rest profile** previously recorded here, which
  predates the 1-D intake runners. It is not CFL thrashing — `accepted/call` is
  1.09 and `rejected/call` is 0.000 — it is ~8.1 us of largely fixed per-call cost
  to advance one 6-12 cell runner, paid twice per mechanical sub-step per cylinder.
  The runner mesh is ~30 mm cells whose own CFL limit allows ~66 us while the
  mechanical sub-step calls it every ~26 us, so it is driven ~2.5x finer than its
  own stability needs. Cutting that cost is the single highest-value performance
  work left; note that multirating it the way the exhaust is multirated would
  import the same averaging bias documented above, on the side that sets VE.
- **A per-cylinder fork-join cannot pay for itself here, and was removed.** One
  barrier per gas sub-step is ~19,000 dispatches/s on a V8 at 5,940 rpm. Four
  variants were measured (realtime factor, LS3 / Merlin): broadcast wake plus a
  2048-yield spin 0.266 / 0.286; per-worker targeted wake plus spin 0.249 / 0.256;
  per-worker wake, no spin 0.282 / 0.336; **inline 0.343 / 0.396**. Every threaded
  variant lost, and the spin actively stole cycles from the thread the barrier was
  waiting on. Removal was verified bit-identical (LS3 torque, IMEP, VE, air_mg to
  every printed digit). A real parallelisation would have to keep workers resident
  *across* sub-steps — a different architecture, not a tuning of that one. Note
  `decoupledSharedVolumeCylinderThreshold` (8) survives and is now purely the
  Jacobi/Gauss-Seidel shared-volume choice: changing it moves large-engine
  calibration for real. **That verdict does NOT extend to the 1-D intake runner
  advance**, which `CylinderWorkerPool` now dispatches and which pays — see
  below.
- **A work-stealing barrier must count workers, not items — and a hung `ctest`
  looks like nothing at all.** `CylinderWorkerPool` first counted outstanding
  *items* and released the master when the count hit zero. That deadlocked
  `EngineLab.CombustionPhasing` for **8 h 52 min** on a test that takes 7.81 s.
  The hole: a worker that has *observed* a generation but has not yet entered
  the queue is invisible to the master, so the master can finish the job,
  publish the next one, and only then have that worker enter — claiming an item
  of the **new** job and decrementing a counter the master is about to
  overwrite. The count is then permanently one too high. No guard on "a worker
  is about to enter" can close it; the window lies between two of that worker's
  own instructions. A generation-acknowledgement barrier (every worker must ack
  g before g+1 exists) deletes the notion of a stale worker, and costs nothing —
  dropping the per-item atomic decrement pays for the extra edge. **Recognise
  the signature: exactly one core at 100% with every other worker parked, i.e.
  process CPU time ≈ wall time regardless of thread count.** Two reading
  lessons came with it: a background `ctest` that has printed `Start 10:` and
  nothing since is *hung*, not slow — check it with `Get-Process ... | Select
  CPU,StartTime`, not by waiting; and conversely a **short** time in a ctest
  line can be the bad news, since `EngineLab.Core` takes ~90 s when it passes
  and ~8 s when it aborts on a failure.
- **`EngineLabRealtimeBudgetHarness` numbers are only comparable BACK TO BACK.**
  The documented run-to-run spread of ~10% is the *within-minute* figure. Across
  a working session the machine drifts far more: the same commit `c03b6d3`
  measured LS3 0.762 / K20A 0.754 early on and **0.618 / 0.629 two hours later**,
  after a run of builds and test suites — 20%, one way, on every engine. That is
  enough to invent a regression that does not exist, and it did: a change
  measured against the morning's table looked like a 13% loss and was, measured
  against a baseline rebuilt in the same hour, an 11% gain. **Never compare a
  factor to one recorded in a document, a commit message, or an earlier message
  in your own session.** `git stash` + rebuild + measure + `git stash pop` costs
  two builds and is the only valid A/B. The deterministic instruments
  (`EngineLabIntakeDuctBench`, checksums, `EngineLab.CombustionPhasing`) do not
  have this problem — prefer them, and use the realtime harness only for the
  final ratio. Two corollaries, both learned by getting them wrong:
  **always run a NULL CONTROL** — an engine the change cannot affect by
  construction — beside any A/B on this harness. A thread-count experiment
  produced "+23%, three rounds out of three" whose control, whose true value was
  0%, read +8%; a later protocol put the same control at −19%. And
  **`A, B, B, A` is not counterbalanced**: position 1 pays cold start (page
  faults, catalogue file cache, frequency ramp) and always falls to A, while B
  inherits the two warm middle slots — a measured +8% bias. Discard a warm-up
  run and alternate which variant leads.
- **The current six-core worker policy is measured, not the old formula.** On
  the 12-thread desktop, a counterbalanced 2/3/4-worker sweep (six runs per
  variant, Big Twin as zero-worker control) gives LS3 means
  1.1160/1.1045/1.0805 and Merlin 1.1095/1.1222/1.1032. Production therefore
  uses **2 workers for 3–9 cylinders and 3 from 10 cylinders**, still bounded by
  hardware and `cylinderCount - 1`; explicit benchmark overrides bypass the
  policy. Do not restore `hardware_concurrency/2 - 1` as an automatic target:
  `EngineLabRealtimeBudgetHarness` runs neither the audio callback nor the UI,
  and more pool threads measurably oversubscribe the real application.
- **The realtime factor saturates at 1.0; use `--free-run` for capacity.**
  `EngineRuntime::run` sleeps to a wall deadline, so an engine with 3x of margin
  and one exactly breaking even both report `1.000` — six catalogue engines were
  sitting at that ceiling indistinguishably.
  `EngineLabRealtimeBudgetHarness --free-run` removes the sleep
  (`setRealtimeThrottleEnabled`, instrumentation only — the audio thread, the
  telemetry queues and the dyno controller are all paced by it) and the same
  ratio reads as capacity. Do not target 1.0 either: at exactly break-even,
  scheduler jitter alternates the cylinder-pressure telemetry between early and
  late, and that telemetry is the exhaust acoustic chain's only excitation.
- **A shared 0-D plenum cannot be frozen, lagged, or midpointed.** Running the
  runner advances concurrently forces every cylinder in a group to read ONE
  plenum state, and every cheap answer is wrong. Measured against
  `EngineLab.Core`'s "a stationary engine must settle at ambient", where the
  serial reference is flat to 0.002 kPa: frozen (plain Jacobi) **−0.62 kPa, and
  it is a wrong fixed point, not slow relaxation** — twenty times the settling
  time recovers 0.06 of it; predicted from the previous pass's draws, **a
  sustained 0.17 kPa limit cycle**; every cylinder at the midpoint of the total
  draw, −0.095 kPa. What works is reconstructing the Gauss-Seidel staircase from
  a SAME-INSTANT prediction (`ExhaustGasNetwork::predictOutletTransfer`),
  iterated twice because the staircase is a fixed point, with the prediction
  itself second order (a Heun step on the terminal cell — `dt*F1` alone leaves
  −0.011 kPa): **+0.001 kPa**. The lag result is the one to carry forward: the
  correction is only ~0.04% of plenum mass, but the terminal cell's acoustic
  response time is volume/(mouth area × c) ≈ 89 µs against a 26 µs lag.
  **Nothing that pass reads may be stale.** And the prediction phase must itself
  be concurrent — serial, it hands Amdahl a term of the same order as the
  advance it exists to order, and cost the V8 a third of its parallelism
  (0.726 → 0.586).
- **`EngineLabIntakeDuctBench` prints a bit-exact checksum.** It advances a
  runner-shaped duct at the simulator's real cadence (a half mechanical
  sub-step, not a 240 Hz frame) with the wall model on, and fingerprints every
  conservative variable and wall temperature afterwards. An optimisation that
  leaves the checksums unchanged is provably not a physics change, in thirty
  seconds instead of a ten-minute catalogue re-run. Measured attribution at
  6 cells, ~533 ns per cell per sub-step and essentially all of it per-cell (no
  fixed per-call overhead left to remove): **walls 28%, MUSCL reconstruction
  25%, wall friction 6%**, the rest flux and RK2.
- **The duct wall model is not 28% of wall arithmetic; most of it is a second
  primitive recovery.** Measured with the bench, per cell per sub-step: the
  coefficient chain (Sutherland viscosity, Reynolds, Gnielinski Nusselt) is
  **6.2%**, the exchange itself (`expm1`/`exp` and ~8 divisions) **11.0%**, and
  the remaining **~10.8%** is the extra full `recoverPrimitiveStates` pass that
  exists only because the wall step mutates cell energy between the recovery and
  `prepareStateCache`. Sub-rating the exchange therefore buys more than the wall
  arithmetic it skips, because it skips that pass too. Also: `std::pow(x, 2.0)`
  in the friction factor **is** a real libm call here, worth 5% of the whole duct
  solver, and rewriting it as `x*x` is **bit-identical** on all six bench
  configurations — the source comment claiming otherwise was wrong and is now
  corrected.
- **A self-timed sub-rate desynchronises across a barrier.** Each runner network
  timing its own wall cadence from its own accepted sub-steps drifts out of phase
  with its siblings within a few mechanical sub-steps, and a barrier costs the
  *maximum* over participants, not the mean — so the burst lands on a different
  dispatch for every cylinder and is paid on nearly all of them. The cadence is
  therefore driven by `EngineSimulator` (`requestWallHeatUpdate`), fired on the
  **second** half-step, which is the only pass where every cylinder is dispatched
  — the first advances only the runners whose intake valve is open. Worth ~3% of
  the V8 over the self-timed version. Generic: any per-cylinder work made bursty
  must be made bursty *in phase*, or the barrier eats the saving.
- **The divisions are not the lever either — that hypothesis was measured and
  refuted.** `recoverPrimitive` runs 7 times per cell per sub-step and its four
  `massFractions` divisions are ~31% of every division the duct solver performs.
  Removing them buys **1.4%**, with all six bench checksums **bit-identical**
  (which also proves those fields are unread on this path). The reconciliation:
  those four divisions are *independent of one another*, so they pipeline at
  division throughput (~4 cycles), not latency (~14). What is latency-bound is
  the *dependent* chain `density → velocity → internal energy → temperature →
  pressure → gamma → sound speed → sqrt`, and that cannot be shortened by
  deleting independent work — the same reason AVX2 measures slower. The largest
  item left is the MUSCL block (limiter + two reconstructions + the two
  primitive recoveries they need) at **16-29%**, and it is not available:
  zeroing the slopes *is* dropping to first order.
- **The previous 2026-07-27 machine was a Ryzen 7 8840U — a 15-28 W mobile
  part — and it throttled hard.** The same bench on the same binary measured
  **467 ns/cell** early in a
  session and **1404** after hours of builds and test suites. That single fact
  explains the realtime harness's 20% session drift and the ±20% noise that made
  a thread-count experiment unmeasurable. Measure in short batches, compare only
  within a batch, and take the **minimum of N runs** rather than the mean —
  interference can only add time. Minimum-of-six takes the bench from ~16%
  repeatability to 2-5%.
- **Widening SIMD is not the lever, and it is slower.** `/arch:AVX` measures
  616 ns/cell/sub-step and `/arch:AVX2` 578, against **533** for the shipped
  baseline, checksums identical in both. The duct solver is bound by the
  *latency* of dependent division/sqrt chains — about 90 divisions per cell per
  sub-step, `recoverPrimitive` alone runs 7 times — not by vector throughput.
  Dropping the four unread `PrimitiveState::massFractions` divisions from
  `recoverPrimitive` is likewise 1-3%, inside the noise.
- **The accepted runner mesh is 95 mm, guarded against a 30 mm/RK2 oracle.**
  The old six-cell-everywhere ablation did buy LS3 ×1.26 / Merlin ×2.0, but cost
  the Big Twin +16.7% VE and the Merlin −22.3% torque at 3,000 rpm. The later
  75/95 mm A/B found +5.9% LS3 and +9.1% Merlin against +0.4% on the CP2 null
  control; 95 mm stays within **4.460%** of the oracle (4.916% with protected
  wall heat). **120 mm is rejected at 16.580%** on the doubled runner. Do not
  weaken the oracle or infer convergence from cell count alone.
- **Never compare a perf CSV across an exhaust-geometry change** — doing so once
  put the V12 at 158% of budget in these docs when it is at 77%. §18 has the
  numbers. The audio callback itself remains comfortable at 15-33% of a
  256-sample budget; that thread is not the problem.
- **A swept CSV row whose `ve / delivered_ve` exceeds ~1.05 is contaminated.**
  Unburned oxygen inflates the trapped figure while delivery tells the truth, so
  that ratio is the documented detector of incomplete combustion — which on a
  sweep means the point sat on the rev limiter or misfired. The Merlin's last
  row reads 351 Nm against 2521 Nm one step earlier. Comparing two such rows
  compares two artefacts; filter on the ratio before believing any before/after
  percentage.

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
- **A requested trapped-charge fuel mass is not an injector liquid mass.**
  For port injection, a fresh pulse makes only
  `(1-X) + X * filmAvailableBeforeSpark` of its metered mass available to the
  next charge. The simulator used to subtract the available inventory from the
  target, then command that raw deficit as liquid; after DFCO emptied the wall
  film this guaranteed a lean first cycle. Divide the deficit by that available
  fraction. Do not raise the DFCO resume threshold to hide it: 1.50× idle still
  fell to 349 rpm without the correction, while the corrected original 1.25×
  policy remains at 479 rpm in the same Merlin trace.
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
  is void; one was, and was withdrawn. The GUI's own back-pressure warning was
  built on it too and was therefore a **false positive by construction** — a
  healthy LS3 peaks at 172 kPa against ~101 ambient while discharging freely, so
  the message latched on and never cleared. Use `exhaustBackPressureKpa`, the
  port mean damped over ~3 firing periods, which is what a manifold gauge reads.
  The local variable feeding the peak is still called `collectorPressureKpa`;
  it is not a collector value.
- **The GUI diagnostics panel had two permanent false alarms, and each had a
  different cause than it looked.** "Contre-pression excessive" was the peak/mean
  confusion above. "Limitée par l'adhérence" was a *sticky OR* across the
  driveline's mechanical sub-steps: one clipped sub-step out of five latched the
  indicator for the whole frame, and with a slip-velocity tyre spring at
  `normalForce * 7.5` N/(m/s) one clipped sub-step happens on every gearshift. It
  now requires a majority of sub-steps. `tractionLimited` is display-only — no
  physics reads it — so refining it is free.
- **A motorcycle has three reductions and the primary was missing.** Crank →
  clutch basket (1.6-2.0), gearbox, then chain. `TransmissionConfig` has no
  primary field, so it must be folded into `final_drive_ratio` — "everything
  outside the gearbox". `motorcycle_6_speed` shipped at `2.62 x 2.75 = 7.21` in
  1st where an MT-07 is `1.925 x 2.846 x 2.688 = 14.73`, i.e. every bike was
  geared ~1.6-2.0x too tall in every gear, halving wheel torque. Recognise it by
  arithmetic, not by feel: 1st gear reached **141 km/h at 9,000 rpm** (real: 69)
  and 6th reached 314 (real: ~189). Check any new vehicle the same way before
  believing a torque or acceleration complaint.
- **Longitudinal weight transfer is schema-5 vehicle physics, not a tuning
  gain.** `VehicleConfig` carries `drivenAxleLayout`, static driven-axle weight
  fraction, wheelbase and CG height. Every 1 ms mechanical sub-step evaluates
  `deltaFz = m*a*h/L`; positive acceleration unloads FWD, loads RWD, and AWD
  retains the total `m*g`. Keep the catalogue layout explicit and reproduce
  `EngineLab.VehicleDynamics` before changing tyre or chassis parameters.
- **Cell-centre primitives are not a profile at the shipped mesh.**
  `targetCellLengthM = 0.300` with `minimumCellsPerDuct = 1` gives a 760 mm
  primary three cells, and with the high-order reconstruction `rho*u` varies 2.4x
  along a *constant-area* duct in a state that is provably settled (bit-identical
  at two settle times) and globally conservative. Trust fluxes and boundaries,
  never cell centres. Reading that variation as "not converged" cost a retracted
  claim.
- **The exhaust coupling averages the cylinder state, then takes one flux from
  the average.** The interval is `min(125 us, 1/(16*firingFrequencyHz))` — the
  low-speed cap was halved from 250 us on 2026-07-28. Because
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

- **A compiled physical exhaust owns the user-facing exhaust controls.** The
  old Street/Open/Turbo/Long-tube/Moto selector and high-noise control are
  compatibility controls only. Do not re-enable them for a physical DAG unless
  the action changes real graph geometry or an explicitly authored downstream
  IR. An authored IR that cannot be decoded must stay visible as an error; field
  free is a valid explicit state, never a silent substitute for a requested WAV.
- **Independent aeroacoustic sources must not share a signed noise sample.**
  Compressor and turbine broadband models once used the same random sequence
  with opposite signs; equal geometry cancelled them exactly. Forced-induction
  sources now own deterministic independent streams. The total exhaust flow is
  partitioned by effective turbine/wastegate area and must sum back to the
  measured flow; never radiate the full flow through the turbine and then add a
  second `flow * opening` wastegate source. Frame-rate telemetry must also be
  reconstructed per audio sample before it modulates a blade-order tone.
- **An upshift torque cut must release against clutch slip, not only a timer.**
  Restoring WOT while the clutch is still pulling the crank into the next ratio
  makes two torques fight and hardens the exhaust-flow/audio transient,
  especially on a boosted engine. `DrivelineModel` now captures the first slip
  in the new ratio and releases the cut smoothly as that slip reaches the lock
  band. Validate both `EngineLab.AudioShiftTransientNA` and
  `EngineLab.AudioShiftTransientBoosted`: a pre-shift WOT percentile alone is
  not a valid click baseline for a turbo, because its intentional dump-valve
  jet raises broadband energy during the event.

## Two concrete traps that cost time here

- **Every engine selector in this repo is an unanchored SUBSTRING match, and the
  catalogue now contains names that contain each other.** `--engines Twin` matches
  three entries (`Big Twin-like 1.9 V2`, `Yamaha CP2 MT-07-like 689 Twin`,
  `MT-07-like 689 Twin Full System`) and the tools silently took the first. Same
  mechanism as `DynoSweepHarness`'s `containsCaseInsensitive`, which would have
  held a race-exhaust variant to the stock bike's rated torque had the variant
  been named "Yamaha CP2 ... Full System". `AbClipRenderer` now lists every match
  and refuses the ambiguity outright in `--compare`, where the whole point is that
  the two sides are the engines you meant. Assume any new selector has this bug
  until you check it.
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
