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
