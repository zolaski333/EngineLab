# Physics, calibration, audio and performance rework — validation log

This is the durable engineering log for the rework started on 2026-07-26.
It records measurements as well as successful changes. Failed hypotheses are
kept because repeating an attractive but disproved fix wastes more time than
documenting it.

All quoted executable results use the MSVC Release build in
`out/build/windows-vs2022` unless stated otherwise. Thresholds were not relaxed
to make a change pass.

## Initial measured state

- Build: Release succeeded.
- Test suite: 17/19 passed.
  - `EngineLab.IdleStabilityRegression` failed because the Merlin stalled after
    a throttle blip.
  - `EngineLab.AudioRender` failed because the CP2 intake drove the monitor
    safety limiter.
- Yamaha CP2 dyno:
  - peak torque 71.07 Nm at 5,000 rpm;
  - 66.54 Nm at 6,500 rpm;
  - 55.15 kW at 8,500 rpm, 56.64 kW at 9,000 rpm and 57.16 kW at
    9,500 rpm;
  - pumping MEP reached -2.114 bar at 9,500 rpm.
- Yamaha CP2 audio:
  - maximum pre-limiter magnitude 1.313;
  - crest factor 17.78;
  - peak exhaust/intake/structure observer pressures
    40.2/314.4/4.9 Pa.
  - Muting only the intake reduced the pre-limiter peak to 0.320 and the crest
    factor to 5.16. This isolated the defect to the physical intake layer.
- Application-path performance:
  - audio callback p95 consumed approximately 22–57% of its 5.333 ms budget;
  - physics deadline overruns were K20 907/1440, 2JZ 990/1440,
    LS3 613/1440 and Merlin 522/1440;
  - worst physics lateness was 23–34 ms.

The dyno CSV writer also emitted 21 row fields under a 19-column header. Its
schema is now generated from fixed-size arrays and guarded by both a
`static_assert` and the fast `EngineLab.DynoSchema` CTest.

## Idle recovery and fuel adaptation

### Cause

The Merlin did leave deceleration fuel cut, but a single per-cylinder
closed-loop fuel trim was shared by idle and high load. Its 0.50-throttle blip
was rich enough to lower the same trim needed at idle from about 0.80 to about
0.56. The engine then crossed the idle catch with an empty port film and an
under-fuelled command.

Closed-loop adaptation was also allowed to observe cycles where fuel had been
explicitly cut. Such samples do not contain a meaningful commanded-mixture
error and must not train a fuel correction.

### Implemented model

- Closed-loop trim learning is disabled while fuel is cut or the requested
  fuel quantity is effectively zero.
- Low-load and high-load trim cells are independent.
- The command is continuously interpolated between the cells from 0.10 to
  0.25 physical throttle.
- The high-load cell is seeded from the learned low-load value on first
  tip-in. This prevents a discontinuity while preserving independent learning
  after that transition.
- Telemetry exposes requested/delivered fuel, delivery ratio, active trim,
  equivalence ratio, and DFCO latch/ramp state.

### Rejected hypotheses

1. **Raise the DFCO resume threshold by a fixed 300 rpm.** It improved the
   Merlin trace but changed the start-up DFCO sequence of low-idle engines.
   The Big Twin later stalled at steady idle and the radial stalled after the
   blip. The additive threshold was removed and the original proportional
   1.25-times-idle release restored.
2. **Freeze all transient lambda learning.** This also suppresses legitimate
   wetting-film adaptation during start-up and made starting worse. Only
   unobservable zero-fuel cycles are excluded.
3. **Use a fresh high-load trim of 1.0.** The radial's learned idle cell was
   about 0.89; a step to 1.0 over-fuelled its first tip-in before its slow
   crankshaft could accelerate. Seeding the new cell from the current
   low-load cell removed the discontinuity.

### Proof

`EngineLabIdleStabilityRegressionTests` passes all 13 catalogue engines. Every
engine has `caught=1`, `steadyStall=0` and `blipStall=0`. Selected results:

| Engine | target rpm | steady mean | min | max | stddev |
|---|---:|---:|---:|---:|---:|
| Big Twin | 760 | 762 | 726 | 816 | 17.5 |
| Merlin | 800 | 800 | 799 | 801 | 0.4 |
| Radial R5 | 640 | 637 | 545 | 693 | 29.0 |
| Yamaha CP2 | 1,400 | 1,400 | 1,366 | 1,437 | 15.2 |

`EngineLabCoreTests` also passes, including a synthetic 1,000 rpm/s coast test
which proves that fuel is enabled and has useful metering authority at idle.

## CP2 intake acoustic overload

### Diagnostic taps

The intake wave network now reports cumulative absolute pressure maxima at the
valve source, runners, plenum, airbox, mouth and 1 m radiation output. The
audio renderer transfers these measurements to atomics once per block; no
allocation or atomic operation was added to the per-sample network loop.

The failing CP2 render measured:

| Stage | peak pressure |
|---|---:|
| valve source | 26.15 kPa |
| runner waves | 37.50 kPa |
| plenum | 21.95 kPa |
| airbox | 18.93 kPa |
| mouth incident wave | 199.66 kPa |
| radiation at 1 m | 1.207 kPa |
| configured microphones | 324.7 Pa |

The source and cavity were not the overload. Energy accumulated in the
near-lossless linear open-end reflection until the mouth wave left the linear
acoustic validity range.

### Implemented model

The existing Padé radiation impedance remains unchanged at infinitesimal
amplitude. The intake mouth additionally enables the quasi-steady nonlinear
open-end resistance

`Z_nl / Z_c = 2 C_d / (3 pi) * |u| / c`,

with `C_d = 2` for the thin-wall unflanged termination represented by the
class. It is implemented as a positive series resistance in wave variables,
including an exact solution of the radiation IIR's direct feed-through. It is
not a sample clamp, limiter, output gain, or catalogue-specific coefficient.
With coefficient zero the old linear transfer is reproduced exactly.

Primary references:

- M. Atig et al., “On the nonlinear behavior of the open end of a tube,”
  *Comptes Rendus Mécanique* 332 (2004), 299–304,
  <https://doi.org/10.1016/j.crme.2004.02.008>.
- M. C. A. M. Peters et al., “Damping and reflection coefficient measurements
  for an open pipe at low Mach and low Helmholtz numbers,” *Journal of Fluid
  Mechanics* 256 (1993), 499–534,
  <https://doi.org/10.1017/S0022112093002861>.

### Proof

For the same CP2 simulation and unchanged valve-flow source:

| Metric | before | after |
|---|---:|---:|
| mouth pressure | 199.7 kPa | 17.8 kPa |
| microphone intake pressure | 324.7 Pa | 26.7 Pa |
| maximum pre-limiter magnitude | 1.334 | 0.218 |
| crest factor | 14.12 | 5.47 |
| limiter samples | non-zero failure condition | 0 |

The complete audio harness passes, including the 13-engine catalogue, long-run
stability, forced induction, true idle/rev/return, spectral differentiation
and physical decay checks. `EngineLabRealtimeRegressionTests` passes a
40 kPa sinusoidal stress case which proves that the nonlinear element remains
finite, removes reflected energy, and never returns more energy than arrives.

## Build-system observation

An unrestricted parallel Release build exhausted the MSVC compiler heap
(`C1060`) while compiling three very large LTO translation units at once.
Those exact targets (`EngineLabCoreTests`, `EngineLabAbClipRenderer`,
`EngineLabAudioAbHarness`) all compile and link with `--parallel 1`. This is a
build-memory scheduling limit, not a source error. Final build documentation
must recommend a bounded job count on ordinary machines.

## Open observations for subsequent phases

- CP2 torque magnitude is close to the intended real-engine envelope, but the
  simulated peak occurs roughly 1,500 rpm too early and high-rpm pumping loss
  is excessive.
- The audio callback itself meets its deadline, but physics production misses
  a large fraction of frame deadlines. Optimisation claims must be based on
  isolated, repeated before/after runs rather than the contended validation
  executions above.

## Radial bank topology and chamber flame calibration

### Radial root cause

Catalogue loading synthesized one zero-degree bank for every layout other than
V and flat engines. `MechanicalKinematics::bankAngleFor()` correctly treats an
explicit bank as authoritative, so the generated bank erased the radial
cylinders' authored 0/72/144/216/288-degree axes. All five pistons consequently
moved in phase although their valve and ignition schedules remained staggered
over 720 degrees. This explains both the implausible 17 Nm loaded result and the
loaded audio scenario that stopped: most cylinders exchanged gas on the wrong
physical stroke.

The catalogue now synthesizes one bank per radial cylinder, preserving the
authored spatial axis. Bank validation accepts both the signed convention used
by V/flat engines and the natural [0, 360] radial convention. A catalogue
regression checks every radial cylinder-to-bank mapping.

### In-cylinder model

Two physical head properties are now explicit combustion calibration inputs:

- chamber turbulence intensity relative to the mean-piston-speed closure;
- the number of independent ignition kernels.

Turbulence changes turbulent flame speed, while multiple ignition sites change
initial burned-kernel volume; they are deliberately not represented as fuel
energy, arbitrary torque multipliers, or extra spark advance. The shipped CP3
and CP4 heads use progressively stronger tumble. The large-bore radial uses its
realistic dual-plug topology. JSON/YAML round trips and range validation cover
both fields.

### Proof

The same radial catalogue sweep at 2,000 rpm changed from 16.99 Nm / 3.56 kW /
0.265 VE to 572.39 Nm / 119.86 kW / 0.909 VE. Its five resolved geometric TDC
angles are now 0.00, 73.18, 145.99, 214.01 and 286.82 degrees, and all five
cylinders trap useful fresh charge (1.36--1.67 g in the inspected cycle).
This is evidence of repaired mechanics and gas exchange, rather than a
catalogue torque correction.

At unchanged displacement, fuel and ignition tables, chamber calibration moved
the catalogue curves as follows:

| Engine | baseline peak torque | calibrated peak torque | baseline peak power | calibrated peak power |
|---|---:|---:|---:|---:|
| Yamaha CP3-like | 84.26 Nm | 86.98 Nm | 75.54 kW | 78.49 kW |
| Yamaha CP4-like | 84.37 Nm | 93.60 Nm | 77.21 kW | 87.32 kW |

The CP4 pressure trace changes consistently with faster combustion: burn
completion moves from about 60 to 29 crank degrees and peak pressure from about
48 bar at 10 degrees to 74.5 bar at 19.6 degrees. `EngineLabCoreTests` passes,
including monotonic tests for increased turbulence, two independent kernels,
serialization, validation, and radial catalogue topology.

## Variable-area quasi-1D ducts

### Implementation

Exhaust components now accept an optional `outlet_diameter_mm`; intake paths
accept `runner_plenum_diameter_mm`. Zero is backward-compatible constant area.
A non-zero second diameter defines a circular conical frustum (linear radius):
the exact frustum volume is used by inventory and topology compilation.

The finite-volume residual now uses the local area of every face and the exact
volume of every cell. Species, mass and energy remain conservative, while
momentum receives the quasi-1D pressure-wall source `p (A_R - A_L) / V`.
Friction, wall heat transfer, wall thermal capacity and source stability use
the local hydraulic diameter/volume. Network boundaries use the correct end
area instead of a mean area. The acoustic networks likewise scatter with the
area at the relevant endpoint.

During review, the port-fuel source exposed a genuine secondary bug: its
three-cell distribution divided every share by cell zero's volume. That is
correct only for a cylindrical mesh. It now divides each equal mass/energy
share by that cell's own volume, and the conservation test deliberately injects
into a tapered runner.

### Proof

- A stationary 4:1 taper remains at uniform pressure with relative density and
  energy errors below `3e-13` and momentum below `2e-10 kg/(m2 s)`.
- A moving 4:1 contraction closes every species inventory and total energy
  against its unequal boundary areas to `3e-10` relative.
- The exact conical volume is checked independently in the duct, exhaust-layout
  and intake-runner tests.
- The tapered port injection adds its requested fuel mass exactly once
  (`<1e-15 kg` absolute error) and closes sensible plus latent energy.
- The unchanged steady-runner benchmark still delivers `0.0352669 kg/s`
  against an independent isentropic `0.0366292 kg/s` reference (ratio
  `0.962808`), showing that backward-compatible cylindrical geometry did not
  acquire an artificial restriction.
- `EngineLabGasDynamicsTests`, `EngineLabCoreTests` and
  `EngineLabRealtimeRegressionTests` all pass. The desktop application also
  compiles with the new two-diameter editor.

## Compression ignition, flame ceiling and solver caches (`790f772`)

This commit shipped with an empty message body and no entry here. It is the
largest model change of the series, so it is recorded now, after the fact, from
its diff and from re-measurement on the committed state.

### What it contains

- `CompressionIgnitionModel`: Livengood-Wu induction integral over an Assanis
  pressure/temperature/equivalence-ratio delay correlation with an explicit
  cetane correction, then a rapid premixed fraction followed by a
  mixing-controlled diffusion burn. It consumes fuel and oxygen only through
  `ConservativeGasSystem`, so it keeps the same chemical invariants as spark
  combustion.
- Diesel fuelling: metered by an injected-quantity (smoke) map with the
  closed-loop trim explicitly disabled, and `normalizedLoad` taken from
  throttle/load rather than manifold pressure, because a quality-governed engine
  has no throttle-derived load.
- `engines/14_vw_2_0_tdi_like.engine.yaml` plus `road_diesel_en590`,
  `common_rail_diesel` and `diesel_low_speed` parts.
- **Flame speed ceiling.** The fixed 42 m/s clamp in
  `FlamePhysicsModel::turbulentFlameSpeedMps` clipped every high-speed pent-roof
  chamber to the same burn rate regardless of its authored tumble. It is now
  `0.18 * a(T_unburned)`, a deflagration Mach bound. Note for anyone
  investigating idle: at idle the laminar-plus-turbulent sum is around 5 m/s, so
  neither the old bound nor the new one is active there. This change cannot
  affect an idle, and that was verified before looking elsewhere.
- **Solver caches.** Immutable duct geometry (roots, powers, cell volumes,
  hydraulic diameters, roughness terms) was being recomputed per cell, per RK2
  stage, per substep; it is now prepared once at configuration. Wall heat
  transfer became transactional: the gas state and the wall state are validated
  together and committed only if the whole substep is physical.

### Measured effect

Curves, on the committed state, against manufacturer figures:

| Engine | sim torque | real | sim power | real |
|---|---:|---:|---:|---:|
| Yamaha CP2 | 65.9 Nm @6500 | 68 @6500 | 57.2 kW @9500 | 54 @8750 |
| Yamaha CP3 | 96.1 Nm @7000 | 93 @7000 | 89.3 kW @9000 | 87.5 @10000 |
| Yamaha CP4 | 111.9 Nm @9000 | 111 @9000 | 118.2 kW @10500 | 118 @11500 |
| VW 2.0 TDI | 341.6 Nm @2000 | 340 @1750-3000 | 111.2 kW @4000 | 110 @3500-4000 |

The CP4 was at 65% of rated power with its torque peak 3250 rpm early before
this pass. Two physical corrections produced that: the intake runner length had
counted only the visible bellmouth and omitted the head port, and the flame
ceiling above. Performance, median of repeated runs against `e624b9e`: CP2
-11.3/-7.5/-5.9%, LS3 -14.8/-18.9/-17.4%.

### Rejected during that pass, and worth not repeating

- A detailed 4-2-1 collector graph for the CP4. It raised back pressure from 166
  to over 210 kPa, nearly doubled the CP4's step cost, and degraded the curve.
  The equivalent collector measures better.
- Parallelising four cylinders. Eighty synchronisations per frame cost more than
  the four tasks return.
- One float re-association inside the fuel-limit precomputation. It was small
  enough to look harmless and was amplified by autoignition; reverting it
  restored the bench bit for bit.

## Idle robustness: a fragile attractor, not a regression (2026-07-26)

### Symptom

After `790f772`, `EngineLab.IdleStabilityRegression` failed on the Radial R5
(complete stall, 0 rpm, steady and after a blip) and the Big Twin (mean 753, min
576, sigma 57.7, drift 40.9 against a 760 target). Both passed at `e624b9e`.

### Cause

Not a model defect in `790f772`. Tracing both engines at `e624b9e` and at HEAD
and diffing line by line, the two runs first differ at t = 0.40 s (Big Twin,
`air_mg` 636.4 vs 636.5) and t = 0.50 s (radial, `air_mg` 3782.8 vs 3782.9) --
the last printed digit. That is the solver-cache work of `790f772`: algebraically
equivalent, not bit-identical. Everything after is amplification.

An idle that a one-part-in-a-million air-mass difference flips between settling
at 681 rpm and stalling is not a shippable idle, and it means every previous
green run on those engines was luck. `CLAUDE.md` already names this signature.
So the fix had to be robustness, not a hunt for the ULP.

The destructive mechanism, from the radial trace: the after-start air floor is
0.88 with no reference to the engine's idle target, so the radial flared to 1433
rpm against a 640 target. That crossed the deceleration-fuel-cut entry threshold
(1.65 x idle) 0.3 s after catch, all fuel was cut, cycle torque went from +209 to
-70 Nm, the engine fell back below the catch threshold, the starter re-engaged
and re-charged the floor to 0.88, and it repeated. While the floor owns the
actuator the idle anti-windup deliberately forbids the governor from integrating
down, so nothing could oppose it either.

### Rejected hypothesis

**Release the after-start floor in proportion to overspeed.** It fixes the two
worst engines (radial 0 -> 683 rpm, Big Twin sigma 57.7 -> 16.7) and is
physically defensible, but the floor is exactly what keeps a flaring engine
breathing: at 6.0/s it took the Audi I5 from a settled 780/746 rpm (sigma 9.3) to
a full stall. The flare is not the destructive event. Reverted, and the reason is
recorded in the code so it is not proposed again.

### Implemented

Deceleration fuel cut is an overrun function: it presumes a running, warmed
engine coasting down under a shut throttle. The after-start flare satisfies its
speed threshold while being the opposite condition -- the engine is accelerating
away from a catch with an empty port film. Production ECUs inhibit overrun cut
through the after-start phase for exactly this reason. The after-start air
schedule already *is* that phase, is already longer on a cold engine (which is
when a real inhibit lasts longest), and is already maintained upstream, so the
inhibit gates on it rather than introducing a second timer.

### Proof

All 14 catalogue engines pass, and every one is equal or better than its best
previously recorded state:

| Engine | at `e624b9e` (last all-green) | with the inhibit |
|---|---|---|
| Radial R5 | 681, sigma 14.5 | **684, sigma 5.0** |
| Audi I5 | 780, sigma 9.3 | **780, sigma 5.1** |
| Big Twin | 761, sigma 17.9 | **763, sigma 14.2** |
| Yamaha CP3 | 1303, sigma 16.1 | **1301, sigma 14.9** |

Non-vacuity: with the inhibit removed the radial stalls outright and the Big
Twin fails three assertions, which is the failure this fixes.

## The catalogue AFR gate could not read a diesel

`EngineLab.CatalogPhysics` failed only on the 2.0 TDI: mean AFR error 5.41
against a 2.5 ceiling, from AFR 22.41 measured versus a 16.99 "target".

The physics is right and the gate was wrong. A quality-governed engine has no
stoichiometric setpoint. What `EngineSimulator` publishes as a diesel's
`targetAirFuelRatio` is a smoke-limit **floor** (stoichiometric * 1.16), and its
fuel is metered by the injected-quantity map with the closed-loop trim disabled,
so whichever binds first the delivered mixture is normally *leaner* than the
floor. AFR 22.4 at the rated point is textbook, and it is the same calibration
that reproduces 340 Nm / 110 kW.

The gate now scores compression ignition one-sided: only running *richer* than
the smoke limit counts as an error. That is strictly tighter than the old
two-sided band on the rich side, which is where the real failure -- sooting past
the smoke limit -- lives. The tolerance was not widened.
