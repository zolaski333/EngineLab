# Physical thermoacoustic architecture

This document describes the shipped implementation and its physical contract.
The default exhaust chain is no longer a voicing of presets: its source, its
propagation and its radiation derive from SI quantities produced by the
simulation. Any future change must preserve this separation.

## Active chain

```text
combustion and 0D chamber (GasCell)
        ↕ conservative Riemann flux at each valve
non-linear quasi-1D gas network (complete DAG, physical feedback band)
        → instantaneous SI boundary p, ρ, c, ṁ, CdA
decomposition into characteristics p+ / p−
        ↔ waveguides + admittance junctions
passive radiation load at each outlet
        → free-field pressure towards two microphones + r/c delay + directivity
        → explicitly provided measured IR, optional
```

There is no 0D runner/collector fallback in `EngineSimulator`. An invalid
physical topology prevents the simulator from being built. As soon as a valid SI
topology is compiled, the physical path owns the output from the first sample:
it propagates silence up to the first boundary, then stays locked. Procedural
voices can therefore neither appear during start-up nor come back after a later
telemetry loss.

## 1. Non-linear gas network

`EngineLabGasDynamics` carries, per finite volume:

- the mass of each species (`O₂`, inert, fuel, burned gas);
- the axial momentum;
- the total energy.

The kernel uses an HLLC flux with an HLLE safety fallback, a TVD
reconstruction, SSP-RK2 and a CFL step. A non-physical attempt is rejected and
retried with a smaller step; no mass or energy is created by a numerical floor.
Local losses, wall friction and heat transfer are declared source terms. The
engine network walls now have a finite heat capacity: the gas-metal exchange
conserves the combined energy and only external convection explicitly rejects
heat to the ambient.

A valve is not treated as the zero-thickness continuation of a tube. Its `CdA`
feeds a compressible isentropic nozzle law, subcritical or choked depending on
the pressure ratio. The composition and total enthalpy of the upstream side are
carried in both directions. The sonic flow has an independent analytical test;
this fix was necessary because an HLLC tube flux strongly underestimated the
blowdown of a cylinder reservoir.

`ExhaustNetworkLayout` compiles every authored component: tubes, catalysts,
mufflers, resonators and outlets become ducts; merges and splitters become
finite junction volumes. Length, volume, connection area, hydraulic diameter,
loss and discharge coefficient keep their unit and their owner. The order of
the boundary array does not change the result.

### Real-time scale separation

Resolving the whole audible spectrum with finite volumes would require tens of
thousands of updates per second per cell. The implementation adopts a physical
multirate decomposition:

- the non-linear mesh has a maximum cell length of 300 mm; it resolves the mean
  flow, the back-pressure and the firing fundamentals up to about 330–360 Hz in
  hot gas;
- a shorter component stays a single conservative control volume, with its
  exact volume, ports and losses; its audio delay belongs to the characteristic
  network;
- the macro boundary is integrated at least 16 times per firing period, with an
  absolute maximum window of 125 µs at low engine speed (see §14; the cap was
  halved from 250 µs on 2026-07-28);
- conservative state, chamber volume, valve `CdA` and outlet opening are
  integrated over time across each window;
- each macro exchange stays bidirectional and closes the species mass and
  energy balances exactly;
- the instantaneous Riemann flow is nonetheless observed at every mechanical
  sub-step so as not to decimate the audio excitation.

This is neither frame skipping nor a waveform cache. It is a partitioned
coupling of two bands whose validity domains are explicit.

## 2. Simulation → audio contract

For each cylinder, `CylinderPressureSample` publishes:

| Quantity | Unit | Convention |
|---|---:|---|
| chamber pressure | bar | absolute |
| runner pressure | kPa | absolute |
| mass flow | kg/s | positive cylinder → network, negative on reversion |
| density | kg/m³ | local network state |
| speed of sound | m/s | local network state |
| valve conductance | m² | geometric area × discharge coefficient |
| path index | — | compiled exhaust route |
| thermoacoustic validity | boolean | all the quantities above are physical |

The runtime publishes one frame per mechanical sub-step when audio is active.
The SPSC queues are bounded and the callback does not allocate.

## 3. Audible characteristic network

The renderer removes a slow average from the pressure and the flow, then builds
the plane characteristics from the measured boundary:

```text
Zc = ρ c / A
U′ = ṁ′ / ρ
p+ = 1/2 (p′ + Zc U′)
p− = 1/2 (p′ − Zc U′)
```

The port reflection is not derived from an opening preset. It comes from the
local linearisation of the orifice law around the mean flow. Runners and
collectors are bidirectional guides; the N-port junctions scatter the waves
according to the admittances `A/(ρc)`. Firing order, lengths, areas and
temperature therefore naturally determine the phase, the cross-talk between
cylinders and the resonances.

The characteristic high band is linear and passive. It propagates the audio
signal, but its high-frequency reflections are not fed back into the 0D
chamber; the non-linear low-band network stays the owner of the physical
back-pressure.

The valve flow now has two explicit spectral owners. The pressure/flow pair
from the network goes through a 4th-order Linkwitz–Riley low-pass at
`0.45 × coupling rate`. The instantaneous Riemann flow goes through the
complementary high-pass, computed at the mechanical rate. This second branch is
never paired with the slower pressure: it becomes a volume-velocity source at
the port, i.e. the antisymmetric characteristics `(+Zc U/2, -Zc U/2)`, then
goes through the same physical valve impedance as the runner waves. The two
filters sum coherently to an all-pass; there is therefore neither a doubled
band nor a hidden timbre gain. If the boundary is already published full-band
(`coupling rate = 0`), the complementary source is exactly zero.

This instantaneous branch is itself a signal sampled by the mechanical solver.
Two Butterworth low-pass sections therefore bound its reconstruction at
`0.42 × mechanical rate` (4th-order Linkwitz–Riley). Without this upper bound,
the interpolation images above the mechanical Nyquist were amplified by the
radiation derivative and produced isolated clicks — the crackle heard mostly on
the Merlin and the 2JZ. This filter removes no frequency the producer can
represent; it only forbids the audio from inventing a band the simulation never
sampled.

## 4. Radiation and calibration

The outlet is terminated by `UnflangedPipeRadiation`, a causal Padé (1,2)
approximation of the Levine–Schwinger solution fitted by Silva et al. The
filter returns the reflected pressure into the guide. The net volume velocity
at the mouth, then its acceleration, give the free-field monopole pressure.

Each outlet now publishes its position, its axis, its diameter and its
termination type (unflanged or flanged). `FreeFieldObserver` computes
separately the two outlet–microphone distances, the `r/c` delays, the `1/r`
decay and the frequency-dependent directivity tied to `ka`. A centred outlet can
legitimately stay almost mono; two separate outlets acquire their width through
their arrival times and not through invented panning.

Pascals have no universal mapping to dBFS: it necessarily depends on the
microphone and the preamp. The capture chain is therefore an explicit
calibration object (`AcousticMonitorCalibration`), with 20 µPa as the SPL
reference pressure and 144 dB SPL RMS at 0 dBFS by default. This choice gives
the headroom of a high-level engine recording; it can be changed independently
of the physics and of the listening volume. There is no hidden "realism" gain on
the physical exhaust bus, and the safety limiter stays at unity gain in the
validation scenarios.

References:

- [H. Levine and J. Schwinger, *On the Radiation of Sound from an Unflanged
  Circular Pipe*](https://doi.org/10.1103/PhysRev.73.383), Physical Review 73
  (1948), 383–406;
- [F. Silva et al., *Approximation formulae for the acoustic radiation impedance
  of a cylindrical pipe*](https://doi.org/10.1016/j.jsv.2008.11.008), Journal of
  Sound and Vibration 322 (2009), 255–263.

## 5. Impulse responses

Free field is the default. The application no longer loads a preset IR, a
generic IR or an IR synthesised from the geometry. A convolution is active only
if `exhaust_paths[].impulse_response` explicitly names a WAV. That IR must
represent a downstream measurement — cabin, room, microphone or a complete
identified system — and not replace missing gas dynamics.

## 6. What was deliberately removed from the physical path

Once the SI boundary is active, the exhaust no longer uses:

- blowdown or "crack" oscillators;
- random jet noise;
- flow jitter;
- muffler FDN;
- preset colouring, collector saturation or DAG audio transmission gain;
- an implicit or generated IR.

The historical code stays isolated for some compatibility harnesses without an
SI boundary. It is not mixed into the output delivered by `EngineRuntime`.

## 7. Physical compatibility — an honest answer

The previous physics was not enough for this qualitative leap. It took the
conservative quasi-1D network, finite cylinder reservoirs, bidirectional fluxes
at the valves, global junctions, signed flow and local SI states. The existing
0D combustion was structurally compatible: it already provides pressure,
energy, composition and volume at the boundary.

The corrected valve flow also revealed the one-cycle lag of port injection
during a fast rise in intake pressure. It was not hidden by enrichment.
`TransientChargeEstimator` keeps the last air mass actually trapped at intake
closing — hence the real filling and the engine's waves — then projects it by
the plenum's ideal-gas density ratio `p/T` alone. The oxygen already resolved in
the chamber stays a lower bound and the closed loop keeps its correcting role.

It is not an absolute validation, however. To correlate a real engine, cylinder
pressure, runner pressure, flow and temperature still need to be compared with
measurements, and combustion, heat transfer, valve coefficients and geometry
improved as needed.

The exhaust, the intake and the radiation of the block/heads are now physical
paths driven by the solver. Forced induction is solver-driven but stays
semi-empirical in its acoustic efficiency; valvetrain, starter and transmission
do not all have an identified radiating model yet. These limitations are not
replaced by any oscillator in a physical path that is already available.

Other explicit limitations: linear plane acoustics in the high band, mean-flow
correction of radiation not modelled, transverse modes and bend acoustics not
resolved, structural modes estimated as long as no NVH measurement is provided.

## 8. Code map

| Responsibility | Main files |
|---|---|
| finite volumes and thermodynamics | `src/gas-dynamics/*/FiniteVolumeDuct.*` |
| DAG compilation | `src/gas-dynamics/*/ExhaustNetworkLayout.*` |
| global coupling/junctions/valves | `src/gas-dynamics/*/ExhaustGasNetwork.*` |
| multirate orchestration and telemetry | `src/simulation/src/EngineSimulator.cpp` |
| physical PFI charge anticipation | `src/simulation/*/TransientChargeEstimator.hpp` |
| passive radiation | `src/audio/*/PipeRadiationModel.*` |
| stereo outlets and microphones | `src/audio/*/FreeFieldObserver.*` |
| intake network | `src/audio/*/AcousticIntakeNetwork.*` |
| structural radiation | `src/audio/*/StructuralModalRadiator.*` |
| forced-induction acoustics | `src/audio/*/ForcedInductionAcoustics.*` |
| Pa → dBFS calibration | `src/audio/*/AcousticMonitorCalibration.hpp` |
| duct steepening (finite amplitude) | `src/audio/*/NonlinearDuctAcoustics.hpp` |
| characteristics and rendering | `src/audio/*/RealtimeEngineAudio.*` |
| explicit IR loading | `src/app/src/MainComponent.cpp` |

Do not reintroduce a silent fallback if the network fails. A configuration
error must be observable; a resolution limit must feed
`solverResolutionLimited`.

## 9. Required validation

The tests cover in particular: uniform state, Sod shock tube, positivity,
species/energy conservation, single volume, propagation, direct interfaces,
boundary order, subcritical and sonic nozzle, blowdown/reversion, charge
projection by density, SPL calibration, passive radiation, determinism,
invariance to inherited presets/noises/gains, geometry and 48/96 kHz
invariance.

Before shipping:

```powershell
cmake --build out/build/windows-vs2022 --config Release
ctest --test-dir out/build/windows-vs2022 -C Release --output-on-failure
```

The LS3 harness must also stay under the 4.167 ms of the 240 Hz loop.

## 10. Measured state and known limitations

This section records what was **measured**, including what does not hold the
budget. It takes precedence over any earlier claim in this document.

### Performance (`EngineLabPhysicsPerfHarness`, Release, 3 runs)

| Engine | rpm | mean | p50 | p95 | max |
|---|---|---|---|---|---|
| LS3 V8 | 3630 | 2.38–2.44 ms | 2.37–2.42 | 2.55–2.74 | 2.79–3.27 |
| LS3 V8 | 5940 | 3.22–3.32 ms | 3.19–3.25 | 3.48–3.75 | 3.83–**5.09** |
| Merlin V12 | 1760 | 3.95–4.02 ms | 3.93–4.01 | **4.17–4.26** | 4.40–4.63 |
| Merlin V12 | 2880 | **4.94–5.08 ms** | 5.01–5.15 | **5.26–5.39** | 5.50–6.05 |

The LS3 holds the budget on average but its maximum reaches 5.09 ms, 22 % above
the rate. **The Merlin V12 exceeds the budget on average** at 2,880 rpm and its
p95 already exceeds it at 1,760 rpm: this engine does not sustain real time.
Do not quote the LS3 averages alone as proof of compliance.

### Audio path

The production voice is the sum of physical paths that can be measured
separately: complete exhaust network, intake network, structural modes and,
when present, solver-driven forced induction. The corresponding steps are
detailed in §20–23. The historical oscillator voices stay available only for
compatibility harnesses lacking a physical
configuration; the catalogue and `EngineRuntime` require zero samples from that path.

### Levels measured at the published observers

The monitor full scale stays fixed at 134 dB SPL. The render uses the microphone
position published by each engine; the power guard extrapolates that pressure to
one metre with the same `1/r` law before applying 90–130 dB SPL. It therefore
does not confuse a distant observer with a weak source. The harness additionally
requires the sum of the physical layers to stay under the limiter knee (`0.82`)
with a strictly unity safety gain. A calibration change or an AGC therefore
cannot let a badly sized engine through.

## 11. Coupling rate: what was measured, and why it did not move

The exhaust boundary that drives the waveguide is sampled at the multirate
coupling rate. Any spectral content above half that rate cannot come from the
boundary: it is a reconstruction image. `EngineState` now publishes
`exhaustCouplingFrequencyHz` and `exhaustNetworkSubstepFrequencyHz` so that the
comparison is direct instead of guessed, and the render harness reports them
per engine.

Two fixes were told apart, and only one was kept.

**Kept — boundary consistency.** Pressure, density and speed of sound were
reconstructed by blending two network nodes, while the mass flow came from the
most recent node alone. The characteristic decomposition `0.5 (p' ± Zc U')`
requires both to describe the same instant of the same field. The discrepancy
was `(1 − phase) (to.p − from.p)`: a sawtooth clocked at the coupling rate,
whose harmonics extend far above the coupling Nyquist. It was injected into the
source term, so no waveguide fix could reach it. Fixed by publishing two flows
with separate contracts. Zero CPU cost.

**Rejected — raising the rate.** Acoustically, it works. Measured by coupling
at every mechanical sub-step, all the peaks drop back below the coupling Nyquist
for the first time (674–1583 Hz, so modes that can really be represented), the
I4 high-band fraction falls from 3.8 % to 0.5 % and the V8's from 7.9 % to
0.8 %.

The budget forbids it. The LS3 goes from ~3.76 ms to ~4.55 ms for a 4.167 ms
frame at 3,630 rpm, i.e. from inside to outside; at 5,940 rpm it goes from 5.13
to 7.14 ms. The overhead is not the CFL sub-cycling — whose total number of
sub-steps depends on the physical time covered, not on the number of calls — but
the fixed work per coupling: boundary averaging over every cylinder and setting
up the advance. At stride 1 that fixed cost is paid at every sub-step.

An absolute bound on the rate was also tried, to spare engines with many
cylinders. It **degrades**: the V8 climbs back to 36.1 dB @ 5754 Hz, a
first-order image of its own coupling. The content of the blowdown puff does
follow the firing rate, so the per-firing-period rule is physically grounded and
a bound in absolute Hz breaks it. Do not retry without first addressing the
fixed cost per coupling.

The practical path is therefore to reduce that fixed cost, not to raise the
rate as is.

### Honest remaining limitation

The narrow peaks are not eliminated. After the consistency fix, the harness
still measures 16–32 dB above the local floor, worst case V8 at 31.9 dB @
3833 Hz, and several stay above the coupling Nyquist: there is imaging left
that this fix does not explain. The really physical band stays bounded at
~1.1–2 kHz by the coupling. It is a structural limitation, not a setting.

No non-regression test guards this fix. The only clean invariant considered —
"no energy above the coupling Nyquist" — is strictly false, since the valve
termination is non-linear and legitimately creates harmonics. An invented
threshold would have been worse than nothing.

## 12. The frame comb: the real origin of the "metallic" sound (resolved)

Section 11 left peaks "above the coupling Nyquist" unexplained. Their origin is
now established and fixed. The dominant peaks across the whole catalogue fell
on exact multiples of 240.07 Hz — the frame rate — identical from one engine to
another (4081 Hz on the V8 **and** the I2), with sidebands at each engine's
firing rate. Synchronous folding at 200 samples measured the frame-locked
component at ~−32 dB of the total RMS.

Attribution by elimination, each step measured: synthetic layers cut
(`--mute-combustion/-mechanical/-intake`) → persists; IR replaced by a Dirac →
persists; per-block refits frozen → persists; harness dyno smoothed at 2 Hz →
persists; render sub-blocks of 100 (`--audio-chunk`) → the comb does not follow
the block size. Forced conclusion: the comb enters through the telemetry — **the
network solution itself** was modulated at the frame rate.

Cause: the network drain was forced at the last sub-step of every frame. When
the number of sub-steps is not a multiple of the stride (V8: 49 sub-steps,
stride 2), the last averaging window is truncated — the same irregular pattern
repeated at 240 Hz. Four fixes, in order of elimination: an anti-imaging
reconstruction filter following the published rate; constant-delay
reconstruction (625 µs) over a ring of time-stamped nodes; 10 Hz ramps on the
delays driven by frame telemetry; and the decisive one, a **free-running network
integration grid** (member accumulators, drained by accumulated duration, never
again at the end of a frame).

Result in the factory configuration: no dominant peak left on the comb. I4
16.5 dB @ 2241 Hz (real mode), V8 34.6 dB @ 983 Hz (4th firing harmonic — engine
content), I2 12.1 dB @ 678 Hz, Radial 17.7 dB @ 523 Hz. High band 0.2–0.6 %
(against 3–15 % at the start of the work). Removing the degenerate window also
gives back ~20 % of CPU: LS3 3.73 → 2.89 ms at 3,630 rpm, 5.13 → 4.15 ms at
5,940 — back within budget.

Documented residue: the radial keeps a weak line at 32×240 Hz, its coupling grid
being genuinely commensurate with the frame (exactly 8 drains per frame); high
band energy 0.0 %. The remaining frame-locked component is concentrated at
240/480 Hz — the authentic response of the engine to per-frame commands — and
no longer in the treble.

## 13. "Muffled" and "all the same": what was measured, and the distinction that matters

Two separate listening complaints were confused at first, and they must be kept
apart because **their causes have nothing in common**:

- **"muffled"** — the top of the spectrum is missing. Real cause: the physical
  band was bounded by the coupling.
- **"all engines sound the same"** — per-engine character does not come
  through. This is **not** a bandwidth problem.

### The measurement that settled "all the same"

The spectral shape correlation (log spectrum, 80–6000 Hz) between engines is
0.55–0.74. The reflex is to conclude "they are too alike". That is wrong, and
the measurement that proves it is the **third-octave spread**: in each band, the
gap between the loudest and the quietest engine is **15 to 30 dB**. The engines
differ enormously band by band. The 0.55–0.74 correlation only captures the
**common trend** — everything rolls off towards the treble — which is physically
universal and correct. There is **no** common spurious resonance (the only
low-spread band, 160 Hz, is at 14 dB; everything else ≥ 17 dB). So: no
homogenisation bug to fix.

Trap to avoid: **the spectral shape correlation rises when every engine becomes
brighter "in the same way"**. Widening the band (§14) and adding steepening
(§15) each *raise* that correlation by ~0.02–0.03 (attribution isolated by A/B),
even though they *improve* the sound. It is a metric artefact, not a
regression: the metric penalises "everyone gained treble" even when that treble
is desirable. Do not "fix" this rise.

### What differentiation really is

Per-engine character lives mostly in the **firing pattern** (the envelope), not
in the stationary spectral shape. Two probes confirm it:

- **Envelope modulation spectrum at idle**: each engine has a distinct peak
  signature (K20 62.5 Hz, LS3 196 Hz, Hayabusa 82 Hz, Big Twin 20.5 Hz…). These
  exhaust "rhythms" are clearly distinct.
- **Crossplane V8 burble**: the sub-firing modulation energy relative to firing
  is **0.09 for the LS3** against **0.00** for the evenly firing I4s — the
  "angry" rumble from uneven firing per bank (180/270/180/90° intervals in each
  bank) is present.

Measured conclusion: the engines **are** differentiated; the muffling hid that
difference. Removing the muffling (§14, §15) makes the difference audible
without having to "force" an artificial differentiation — which would have been
a hack.

> **Update, 2026-09-22.** A blind listening test by the owner against real
> engines found that no engine is recognisable beyond its cylinder count (see
> `docs/journal.md`). The differentiation measured here is real, but it is not
> enough to identify a specific engine.

## 14. Doubling the physical band at low engine speed (coupling cap 500 → 250 → 125 µs)

> **Shipped state (2026-07-28).** The cap was halved a second time, from 250 to
> **125 µs**: `maximumLowSpeedExhaustCouplingSeconds.value_or(125.0e-6)` in
> `EngineSimulator::step`. The LS3 coupling rate goes from 3,760 to 11,280 Hz
> (physical Nyquist 1,880 → **5,640 Hz**) and the Merlin's from 3,480 to
> 6,960 Hz (1,740 → **3,480 Hz**). The "~2 kHz" figures in the original text
> below describe the 250 µs step and are kept as a history of the reasoning, not
> as the current state.

`EngineSimulator::step` bounds the coupling interval by
`maximumLowSpeedCouplingSeconds`. This cap only bites **where the
per-firing-period rule is slower than it**: at idle, at low engine speed, and
across the whole range of engines with few cylinders. That is exactly the regime
where the listener heard "muffled", because the coupling Nyquist was ~1 kHz
there and the anti-imaging reconstruction smoothed every blowdown front at the
same place for every engine. At 250 µs the Nyquist goes to ~2 kHz.

**Why this does not reopen the budget wall of §11.** §11 rejected *stride 1*
(coupling at every mechanical sub-step), which pays the fixed cost per coupling
at every sub-step and blows up the LS3 at high engine speed. The 250 µs cap is
different: at maximum engine speed, the per-firing-period rule already gives an
interval shorter than 250 µs (LS3 at 5,940 rpm: ~158 µs), so **the cap does not
bite where the budget is tight**. Measured: at 5,940 rpm the number of coupling
sub-steps is identical at 250 and 500 µs — the physical work at high engine
speed is the same. The cap only adds drains at idle/low engine speed, where it
stays ~3–5 % of the frame budget. An explicit measurement point at
`idle_rpm × 1.1` was added to `PhysicsPerfHarness` so that this region, where
the cap acts, is tracked independently of the load points.

## 15. Front steepening: finite-amplitude propagation in the ducts

`src/audio/include/enginelab/audio/NonlinearDuctAcoustics.hpp` adds the only
propagation non-linearity the physical path was missing. In-duct levels behind
a blowdown puff are of the order of a kilopascal (150–175 dB SPL): the local
speed of a simple wave there depends on amplitude (`dx/dt = c + β·u`,
`β = (γ+1)/2`). Crests catch up with troughs, fronts steepen, and this
steepening fills the harmonics above the telemetry band — the physical origin of
the "bark"/crackle of an open exhaust, the same mechanism as the brassiness of
brass instruments (Hirschberg 1996, Msallam 2000).

Implementation: an **amplitude-modulated delay read** on the existing waveguide
lines (runners and collector). A sample read at a nominal delay `D` is read
again at `D · delayScale(p')`, where `delayScale = 1/(1 + β·p'/(ρc²))`. A pure
time warp: it creates no energy (passive by construction), is exactly
transparent when `p' → 0`, and captures shocks implicitly through the
fractional interpolation. The medium (`ρc²`) comes from the gas state already
tracked per runner and per collector; the inherited runners publish `ρc² = 0`
and therefore read exactly at the nominal delay (bit-identical to the old
path).

**Non-vacuity and literature reference.** The `nonlinearDuctAcousticsRegression`
test (in `RealtimeRegressionTests.cpp`) checks what **first-order** theory
guarantees, not the simulator's output: the 2nd-harmonic growth law
`B₂/B₁ → σ/2` (leading term of the Fubini series), a present and decreasing
harmonic cascade, growth with level, passivity, and a linear control (medium
disabled → no distortion). **Documented trap**: the single-probe scheme is
first order; it reproduces the 2nd harmonic but under-generates the 3rd (~44 %
of Fubini, measured). Do not tighten the test to an exact Fubini match on the
3rd harmonic — this scheme cannot deliver it, and a threshold set on the
measured output would break the "references come from the literature" rule. An
exact match would require a shock-capturing characteristic solver, out of scope
for an audio-band effect.

Measured effect (limiter): the 1.5–4 kHz content jumps (Hayabusa 10.9 → 34.2 %,
K20 2.8 → 37.8 %, EJ25 1.5 → 25.0 %) and 4 kHz+ follows. Effect on
differentiation: neutral (§13).

## 16. Merlin V12 voice: short stacks instead of a long collector

`parts/exhausts.yaml`, `aircraft_manifold` preset: the 980 mm primaries in a
128 mm collector buried the blowdown crack in a low boom and cost ~3× the FV
cells (pushing the V12 out of budget). A real Merlin exhausts each cylinder
through its own short ejector stack (~150 mm) without a muffler. The corrected
geometry (150 mm primary, 0.02 restriction) is **more** faithful, not make-up.
Measured: the Merlin's 500–1500 Hz content roughly tripled, the highest
transient punch of the catalogue (3.13). The V12 stays the darkest engine —
partly physical for a 19.8 L turning at 3,800 rpm — but it now spits instead of
droning.

## 17. The muffler is acoustically inert on the physical path (measured — fixed in §19)

The most important finding for anyone wanting to differentiate road engines:
**`muffler_restriction` filters nothing in the shipped physical path.**

The physical branch of `RealtimeEngineAudio::render` (`sampleUsesPhysicalExhaust`,
`RealtimeEngineAudio.cpp`, "Collector -> outlet characteristic") chains exactly:
forward line → `DuctWallLoss` → steepening → `PipeRadiationModel` → return line
→ observer → calibration → `continue`. It reads neither `pathOpenness`, nor
`collectorReflection`, nor the FDN. The `continue` skips the whole inherited
branch, which is the *only* one to consume `openness` and the only one to call
`processMufflerFdn`.

Consequences, in order of importance:

1. **No muffler element exists in the waveguide.** The DAG `muffler` node
   (`ExhaustGraph.cpp`) only contributes a *length* (450 mm) and a pressure loss
   for the physics. Acoustically, every engine is a straight pipe ended by an
   unbaffled radiation load. No expansion chamber, no area discontinuity, no
   transmission loss.
2. **The FDN, for its part, is independent of the geometry.**
   `referenceFdnSamples` (1493/2111/2791/3557) is constant for every engine;
   only `presetFdngain_` varies, and it comes from the voicing table, not from
   the YAML. In any case it is not reached when telemetry is present.
3. **`openness` has a negligible dynamic range even where it acts.** Computed on
   the nine presets of `parts/exhausts.yaml`, `1/sqrt(1 + 1.35*K)` is 0.80 to
   0.87 for seven of them; only `aircraft_manifold` (0.99) and
   `radial_collector` (0.95) differ. Its uses are flat interpolations
   (`0.42 + 0.10*o`, `0.13 + 0.05*o`, `1.12 − 0.30*o`).

### The measurement that settles it

Two `EngineLabAbClipRenderer` renders with an identical seed were compared, with
only `muffler_restriction` changing: LS3 0.30 → 0.95 and K20 0.25 → 0.95, i.e.
from a sport exhaust to an almost blocked duct.

| engine | segment | max difference per third octave | overall difference |
|---|---|---|---|
| K20 | idle / rev-up / limiter | ≤ 0.52 dB | +0.02 / +0.04 / +0.07 dB |
| LS3 | idle / rev-up / limiter | ≤ 1.39 dB | −0.16 / −0.00 / +0.16 dB |

A real production muffler is 20 to 30 dB of transmission loss in the firing
band. The residue measured here is back-pressure travelling up through the
physics, plus the scheduling jitter of §18 — not acoustics.

**What this explains.** The engines judged successful by ear (Merlin V12,
flat-6, radial) are precisely those whose real exhaust *is* an almost open pipe:
the model happens to be right for them. Every road engine is rendered as a
straight pipe, hence the convergence towards a generic timbre. It is therefore
**not** a matter of wrong values in `parts/exhausts.yaml`: it is a component
missing from the model. Fixing the YAML figures can achieve nothing as long as
no transmission-loss element exists in the waveguide.

## 18. End-to-end real-time budget: audio is fine, physics overflows

Measured on an AMD Ryzen 7 8840U (8c/16t, recent laptop), Release build.

**Audio reminder** (`EngineLabAudioRenderHarness`, 256-sample block at 48 kHz,
5,333 µs budget, concurrent physics thread) — the callback is comfortable:

| engine | callback p95 | load |
|---|---|---|
| K20 I4 | 804 µs | 15 % |
| 2JZ I6 | 877 µs | 16 % |
| LS3 V8 | 1543 µs | 29 % |
| Merlin V12 | 1755 µs | 33 % |

**The physics loop, on the other hand, misses its deadlines.**
`EngineRuntime::run` runs at 240 Hz, i.e. 4,167 µs per step. The same harness
now reports the runtime overruns (`physicsOverruns`) while the audio renders:

| engine | late steps / 1440 | max lateness |
|---|---|---|
| 2JZ I6 | 21 (1.5 %) | 0.7 ms |
| K20 I4 | 134 (9 %) | 1.2 ms |
| Merlin V12 (3,100 rpm) | 162 (11 %) | 1.9 ms |
| **LS3 V8 (6,500 rpm)** | **498 (35 %)** | **17.5 ms** |

`EngineLabPhysicsPerfHarness` on the dyno, full load, median of p50 over three
runs (budget 4,167 µs/step):

| engine | idle | mid-range | high rpm |
|---|---|---|---|
| I2 | 7 % | 17 % | 25 % |
| V8 EL-50 | 24 % | 63 % | **96 %** |
| LS3 V8 | 26 % | 69 % | **99 %** |
| Merlin V12 | 49 % | 72 % | 77 % |

**Measurement correction, worth remembering.** A first version of this table
put the V12 at 158-164 % of the budget and called it out of budget at idle. That
was a CSV captured **before** the Merlin moved to short stacks (§16), which
halved its cost as the commit said. The V12 is not the critical case; the V8 at
high engine speed is. Do not compare a perf CSV across an exhaust geometry
change.

### Where the time goes (temporary phase probe, share of the frame)

| engine | cylinders | FV network | rest | FV sub-steps/frame |
|---|---|---|---|---|
| I2 | 53 % | 16 % | 31 % | 31.1 |
| LS3 V8 | 41 % | 29 % | 30 % | 31.5 |
| Merlin V12 | 20 % | **52 %** | 28 % | **60.7** |

Three readings:

1. **The case at the edge of the budget (V8 at high rpm) is dominated by the
   per-cylinder physics (41 %)**, already parallelised. The FV network is only
   29 % there.
2. **The V12 is the opposite**: the FV network takes half the frame, with
   almost twice the accepted sub-steps of the V8 although it turns half as fast.
   That is the mesh of the 12 runners plus the wide collector.
3. **A ~30 % "rest" exists on all three engines**, independently of their size:
   about 800 µs per frame on the V8. It is work outside the cylinders and the
   network (transmission, telemetry, event generation), and it is not
   parallelised. It is the least explored lead.

Nothing is *lost* (`droppedPressure=0`, `boundaryDropouts=0`): the telemetry
stream arrives late and irregularly, not truncated. But it is that stream, and
that stream alone, that drives the exhaust chain — the LS3 render is in fact not
reproducible from one run to the next (RMS 0.073 to 0.090 over three identical
renders), which the engines that hold their rate do not show.

**Key takeaway: the bottleneck is the physics thread, never the audio
callback.** Any optimisation aimed at the audio render misses the target; it is
the per-sub-step cost of `EngineSimulator::step` (and the number of sub-steps)
that must come down. The `SubstepParallel` pool is already correct — it only
spins between sub-steps of the same frame and goes to a condition variable
afterwards — so the lever is not there either.

## 19. The physical muffler: expansion chamber (correction of §17)

`src/audio/include/enginelab/audio/ExpansionChamberMuffler.hpp`. Two
Kelly-Lochbaum junctions and a bidirectional delay line on the
collector -> outlet duct. The transmission loss is Munjal's closed form,

    TL = 10 log10 [ 1 + 1/4 (m - 1/m)^2 sin^2(kL) ],   m = S_chamber / S_duct

and `expansionChamberMufflerRegression` drives the element with sines to check
the law, its pass bands, its passivity and its transparency. Non-vacuity
checked: flipping a junction sign makes the test fail.

**Transparency guarantee.** Without a configured chamber, `process` is an exact
pass-through. Checked in the shipped render, not just in the test: the Merlin
measures **+0.00 dB in every band and every segment**.

### Two fixes that only measurement found

- **The invented absorption was the real problem.** A first version dissipated
  in the chamber through a gain and a pole driven by `muffler_restriction`. That
  mapping was invented (the restriction is a pressure-loss coefficient, not an
  absorption coefficient) and, **in the collector <-> outlet feedback loop, a
  loss of 2 dB per pass compounds and collapses the low-frequency resonance**:
  the EJ25 lost 11 dB on its fundamental during a rev-up. Loudness normalisation
  then raised the clip and unmasked the render's pre-existing HF floor — 40 % of
  the energy above 4 kHz. The chamber did not manufacture hiss: it erased the
  engine. The element is now purely reactive; everything follows from two areas
  and a length.
  **Trap for the future: do not put broadband loss back into this loop without
  measuring the low end.**
- **False lead, noted so it is not repeated.** The first hypothesis was the
  delay applied as a step at the block (the trap documented in §12). Ramping was
  added — it is the invariant the runner and reflection delays already follow —
  but **measured, it changes nothing** on these clips. It is kept as prevention,
  not as a fix.

### Geometry and measured effect

Deliberately modest expansion ratios (peak TL 1.6 to 7.0 dB, not the 10-13 dB of
a big stock body): it is a **single** chamber, and a real multi-chamber muffler
is designed so that its dips avoid the firing orders. At stock depth, the model
dug a notch on the EJ25 fundamental and halved the top of the K20 and LS3
spectra.

At the limiter, 1.5-4 kHz content without -> with chamber: Hayabusa 41.9 ->
43.2 %, Big Twin 12.1 -> 13.1, K20 34.8 -> 28.3, LS3 11.2 -> 9.2. The
between-engine standard deviation per third octave rises at idle (8.01 ->
8.24 dB) and at the limiter (6.31 -> 6.48 dB). CPU cost: zero (LS3 p95 29.8 %
against 28.9 % before).

### HF floor re-audit — 2026-07-29

The suspicion above was replayed on the current Release binary, and it is no
longer reproducible. On a four-point governed rev-up, the EJ25 measures 1.2 % of
energy above 4 kHz at the top point, and 0.4 % at the previous point; at
mid-range, the short harness measures 0.5 %. The old 20.9 % figure therefore
described an earlier state of the renderer and must no longer drive a
calibration.

The audit of the turbo layer did find three structural defects independent of
level:

- compressor and turbine used the same noise sequence, one the negative of the
  other. With the same flow and geometry, the two sources cancelled exactly;
- power, shaft speed and flows arrived as 240 Hz telemetry steps, which produced
  amplitude discontinuities and their sidebands;
- the total flow was radiated once by the turbine, then again by the wastegate
  multiplied by its opening. An open wastegate therefore duplicated mass instead
  of sharing the flow between two parallel areas.

`ForcedInductionAcoustics` now has four independent deterministic generators,
reconstructs the telemetry per sample (5 ms; dump valve 0.75 ms on attack and
12 ms on release), and shares the flow according to
`A_turbine / (A_turbine + opening*A_wastegate)`. The sum of the two branches
stays exactly the measured flow. The turbine jet velocity uses the passage area
`turbine_flow_area_mm2`; the exducer diameter is still needed to declare a real
acoustic source.

The added test first failed on the exact cancellation, then passes along with
the continuity and conservation tests. On complete normalised clips, the change
stays targeted: before/after correlation 0.99821 on the 2JZ and 0.99803 on the
EJ25, aligned RMS difference 5.98 % and 6.28 %. Naturally aspirated engines are
bit-identical and the >4 kHz share stays practically unchanged (2JZ 0.701 ->
0.703 %, EJ25 0.326 -> 0.327 %): the fix removes source artefacts without
artificially brightening the whole engine.

## 20. Complete acoustic network compiled from the DAG

`AcousticExhaustNetwork` now compiles the exact `ExhaustGraph` into a real-time
wave network. Each duct keeps its length and area in a bidirectional line with
wall losses and finite-amplitude propagation; merges and splitters use N-port
scattering weighted by the admittances. Each outlet has its own radiation load
and its delay to the observer. A 4-into-1-into-2 topology therefore stays six
ducts and two outlets, instead of being reduced to a mean runner and a mean
collector.

The expansion chamber is also derived from its published geometry: its volume
and length give its internal area. `muffler_restriction` stays a mean-flow
pressure loss; it is deliberately not turned into a broadband acoustic gain
without a physical absorption law.

> **Correction (§25).** This section claimed that "the connection diameter keeps
> both real discontinuities". That was false: `connectionAreaM2` is computed by
> `ExhaustNetworkLayout` but was never read by `AcousticExhaustNetwork`, which
> only sees `flowAreaM2`. The two real discontinuities now come from the
> collector trunk, realised from the authored length of the junction — see §25.

All the memory for ducts, junctions, outlets and delays is reserved in
`prepare()`. `process()` does not allocate, and the render harness now rejects
a production engine if the complete DAG is not active. The historical reduced
network only survives as a compatibility path for producers and tests that do
not provide an `ExhaustGraph` yet.

Accepted limitation: the junctions are instantaneous acoustic scatterers,
without a lumped compliance of their own. Finite volumes that have a published
length do become ducts; later modelling a compact plenum without a length will
require a dedicated compliance element, not a voicing gain.

## 21. Modal radiation of the block and heads

`StructuralExcitationSample` publishes at every mechanical sub-step, per
cylinder and in SI units, the gas force on the piston, the inertia force of the
reciprocating assembly, their signed reaction at the bearing, the side thrust
and the reaction torque at the crankshaft. These quantities share exactly the
time stamp of the cylinder pressure; the renderer therefore interpolates them
without rebuilding a force from the engine speed or the audio level.

`StructuralModalRadiator` evolves 8 to 24 damped oscillators according to

    q'' + 2*zeta*omega*q' + omega^2*q = F_modal / m_modal

with an exact analytical transition for a force held over one sample. The
pressure at the published observer then comes from the power radiated by the
mode's RMS surface velocity, its area and the radiation efficiency of the
baffled piston. Going from antinode velocity to RMS velocity uses the mode shape
(beam/torsion or plate), with no hidden calibration gain. The production path no
longer contains the crankshaft sine, the noisy rattle or the shaped "piston
slap" that used to stand in for the structure.

Without a `structural_nvh` section, the frequencies are explicitly marked
`estimatedFamily`: block treated as a hollow shell, heads as thin plates,
dimensions inferred from bore, stroke, connecting rod and layout family. This
model is the best real-time compromise with the available data, but it must not
be presented as a manufacturer NVH correlation.

Schema 5 can now replace this set with `measured` or `calculatedGeometry`
modes. Each mode carries frequency, damping, modal mass, radiating area and
efficiency, RMS shape factor, drive type and signed participation per cylinder.
A traceable source is mandatory and there is no voicing gain. No engine of the
shipped catalogue claims a measurement yet: the absence of real data stays
visible at runtime. Format and protocol:
[`structural-nvh-configuration.md`](structural-nvh-configuration.md).

The tests require exact silence without force, finite physical parameters,
amplification at the computed resonance and decaying energy with positive
damping. `EngineLab.StructuralNvh` adds catalogue loading, round-trips,
excitation of the measured mode and provenance guards. The catalogue harness
also requires this path to be really active in the application wiring.

## 22. Complete intake and forced induction

`CylinderPressureSample` now carries the signed instantaneous mass flow of each
intake valve, the pressure, the density, the speed of sound and the conducting
area actually used by the solver. The published flow is the integral of the two
symmetric half-steps divided by the sub-step duration: the audio source and the
mass balance therefore describe exactly the same exchange, without rebuilding a
pulse from the engine speed.

`AcousticIntakeNetwork` compiles each intake path into individual bidirectional
runners, a WDF plenum compliance, a variable-admittance throttle restriction, an
optional airbox compliance, an inlet duct with thermo-viscous losses, then a
horn/mouth radiation load and the delay to the observer. Volumes, lengths,
diameters and areas come exclusively from `EngineConfig`. A zero value for the
airbox or the duct means the part is absent; it triggers no invented geometry.
The network only carries the acoustic perturbation: a 5 Hz moving average
removes the conserved flow, so that a perfectly stationary flow since `reset()`
produces exact silence.

`ForcedInductionAcoustics` uses the passing orders written in the
configuration: number of compressor and turbine blades, number of lobes of a
supercharger, shaft speed and drive ratio. The pressure of the lines comes from
the resolved shaft power and an explicit acoustic efficiency. The broadband
components of the compressor, the turbine, the wastegate and the dump valve
follow a jet law for a compact jet in U^8, centred on a Strouhal number of 0.2, with corrected
flow, exhaust flow and physical areas.

The four broadband sources have deterministic but independent noise sequences.
The exhaust flow is shared between turbine and wastegate in proportion to their
effective areas; it is never counted twice. The telemetry quantities are
smoothed at audio rate so that updates from the physics thread do not become a
240 Hz modulation.

The differential filter of each noise also computes its analytical variance: its
output is normalised to unity RMS before being multiplied by the RMS pressure
derived from the power. The old code compensated for the uniform noise but not
for the filter loss, then applied a peak pressure intended for a sine.

The dump valve is no longer an envelope triggered by closing the pedal. The
solver opens it when the pressure ratio between the compressor outlet reservoir
and the manifold exceeds the configured threshold; its flow is computed by a
compressible orifice law, subcritical or choked. Only that flow can drive its
radiation. The upstream reservoir is nonetheless still the zero-dimensional
reservoir of the boost model: there is no discretised boost duct or wheel CFD
yet.

The frequency of the blade/lobe orders and the thermodynamic powers are
computed. The blade counts, diameters and coefficients added to the "like"
engines of the catalogue are explicitly family estimates. Absolute
forced-induction levels therefore stay semi-empirical; a compressor map and
acoustic measurements specific to the turbo could replace these parameters
without changing the architecture. The tests require an exact passing
frequency, a pressure proportional to the square root of the power, silence
without power/flow, and the exact absence of wastegate or dump valve when its
physical flow is zero.

## 23. Physical outlets, directivity and stereo observer

Engine schema 4 adds, in SI units, a microphone pair, an optional local speed of
sound, and for each outlet its position, axis, diameter and unflanged/flanged
termination. YAML, JSON, catalogue, script and compiled graph carry these values
without implicit conversion. Older documents migrate to a documented free-field
geometry.

`FreeFieldObserver` is a causal operator per outlet and per channel. It reserves
its delay lines in `prepare()`, then applies:

- the exact geometric distance and the `r/c` arrival time;
- the spherical `1/r` decay;
- a directivity depending on the angle and on `ka`, split into low and high
  bands around `ka = 1`;
- the front/back behaviour specific to an unflanged or flanged termination.

Exhaust and intake therefore return a stereo pressure in pascals directly.
Structure and forced induction use the mean distance of the same microphone
pair as long as their configuration does not publish a complete source position
yet. A room or cabin IR stays an explicitly measured downstream element; free
field is always the default.

## 24. Catalogue validation and removal of the old paths

Release validation now renders **every engine in the catalogue**, not just a few
synthetic fixtures. For each render, it requires:

- real activation of the exhaust/intake networks and the modal radiation;
- zero lost events, boundaries or pressure samples;
- zero samples from the procedural compatibility path;
- finite values, no clipping plateau and non-impulsive dynamics;
- a safety limiter gain of exactly unity and a pre-limiter peak `< 0.82`;
- a plausible SPL at the published microphones and a bounded spectral balance.

The per-layer pressure observers (`exhaust`, `intake`, `structure`) and the
pre-limiter peak are read-only atomic measurements: they never act on the
render. The analytical tests also cover `1/r` decay, flanged back directivity,
rejection of images above the mechanical Nyquist, passivity and conservation of
the network elements.

The procedural path was not removed blindly: it stays isolated for a
compatibility API without a physical graph. In production, the presence of a
compiled graph takes ownership of the output away from it from the first
sample. This separation still allows an explicit A/B diagnosis without keeping
two competing voices in the shipped application.

## 25. Exhaust audit: the wall law, the collector trunk, the per-duct medium

An audit of the exhaust system alone, then a fix. Five defects, all measured
before and after. These are deliberate voicing changes.

### 25.1 `DuctWallLoss` did not apply the law it implements

The class derives the Kirchhoff-Rayleigh attenuation as `exp(-k*sqrt(f))` and
then approximated it with **a single pole fitted at 1 kHz**. A pole cannot
follow `sqrt(f)`: its slope runs off towards 6 dB/octave, whereas the target is
a very gentle tilt (0.2 to 2.5 dB across the whole audio band for a catalogue
duct). Measured against its own law, the shipped LS3 chain over-attenuated by
**1.0 dB at 2 kHz, 4.2 dB at 4 kHz and 11.1 dB at 8 kHz**, per single pass, in a
bidirectional network. Worse, the fitted pole fell between 0.31 and 0.67 for
*every* engine: the top of the band was shaped by the corner of the filter, not
by the geometry — exactly what flattens the differences between engines.

Replaced by a **one-pole / one-zero shelf**. Normalising the DC gain, the
squared magnitude reduces to a Möbius function of `s = sin^2(w/2)`:

```
|H(w)|^2 = (1 + Z s) / (1 + P s),   Z = 4z/(1-z)^2,  P = 4p/(1-p)^2
```

Fitting two points is therefore a **linear 2x2** system, and the warped
parameters invert in closed form, `z = (sqrt(1+Z) - 1)^2 / Z`. No iteration and
no search, and passivity reads `0 <= Z <= P`. Fitted at 2.5 and 15 kHz, the
shelf follows the exact law within **0.35 dB** from 80 Hz to Nyquist, against
11.9 dB at worst for the single pole. Beyond about 9.4 dB of loss per pass at
the low point, no first-order passive shelf joins the two points; `fit()` then
falls back to the pure pole. The longest and narrowest duct of the catalogue is
within 7 % of that threshold.

### 25.2 The missing mechanism: the plane-mode cutoff

Fixing 25.1 removed damping the network depended on without saying so: K20 and
Hayabusa hit the safety limiter. The missing mechanism is the one `DuctWallLoss`
already cited as an excuse to over-attenuate.

Above `f_c = 1.8412 c / (2 pi a)` a circular duct carries higher-order modes,
and every discontinuity there scatters plane-mode energy — where it no longer
follows the delay line. `DuctModeCutoff` applies a **4th-order Butterworth at
`f_c`, once per pass**. No adjustable depth: radius and speed of sound are the
only inputs. Order 4 because the section lives in the collector-outlet loop
where any per-pass loss accumulates (it is at 0.017 dB one octave below the
cutoff, where order 2 would be at 0.264 dB), and because modal onset really is
abrupt. The filter is a TPT state-variable filter, stable for any `g > 0`, so
per-sample interpolation of the cutoff is safe by construction.

The band is now set by the geometry, and the catalogue spans it: **2.1 to
2.9 kHz in an expansion chamber against 7 to 11 kHz in a primary**.

### 25.3 The collector did not exist in the waveguide

`ExhaustNetworkLayout` routed every `merge`/`splitter` to a junction without
ever reading its `lengthMm`, and `AcousticExhaustNetwork` treated a junction as
a point without extent. Yet the historical compiler authors a 120 mm collector
at collector diameter for **all ten engines of the catalogue**, and none of it
reached the audio: four primaries scattered directly into the muffler body.
Measured on the LS3, a primary saw a reflection of **-0.843 instead of
-0.693**, and the chamber inlet an expansion ratio of **2.19 instead of 3.49** —
a Munjal transmission loss of 2.4 dB where the geometry describes 5.5.

A bifurcation is a scattering point **and** a pipe. The authored length is
published as `CompiledExhaustJunction::trunkLengthM` and realised as a duct on
the side of the bifurcation that carries exactly one connection — the common
pipe of a collector is downstream of a merge, that of a dual outlet upstream of
a splitter. With more than one connection on both sides, the length cannot be
attributed to either side and the junction stays a point.

The finite-volume network is **deliberately unchanged**: it models a junction as
a well-mixed plenum and folds the extent into `volumeM3`, which is the right
lumped choice at the frequencies it resolves. A waveguide cannot, because length
is a delay there. Both discretisations now read the same authored geometry.

### 25.4 A single medium per path, taken at the hottest point

The renderer took one `(rho, c)` per path, built from the state **at the port**,
and applied it to every duct. The solver had always resolved temperature per
duct: `outletSamples()` was only read for mass accounting, and `ducts()` not at
all.

Measured on the K20 above 3,000 rpm, port state against per-duct state:

| element | c (m/s) | rho (kg/m3) |
|---|---:|---:|
| port (what every duct received) | 562.8 | 0.505 |
| primaries | 561.7 / 580.7 / 568.0 / 571.9 | |
| chamber | 554.0 | 0.444 |
| tail pipe | 545.8 | 0.459 |

This **corrects the audit's own estimate**, which assumed a much steeper
gradient. The delay error is about 3 % per duct and 6.4 % over the chain, not
20-25 % — i.e. ~20 Hz on the comb of a 400 mm chamber, not 165 Hz. The important
error was elsewhere: in the characteristic impedance `rho*c`, which sets the
scattering at the junctions — 284 at the port against 246 in the chamber and 250
at the outlet, hence **a 12 to 14 % error on every downstream scattering
coefficient**, compounded with the expansion ratio at the chamber inlet.

### 25.5 Boundary reconstruction: order 8

The anti-imaging filter was a 4th-order Linkwitz-Riley at 0.45x the coupling
rate, which its own docstring described as leaving the first image line only
28 dB down — above the 20 dB "metallic" threshold the harness itself uses.
Moved to order 8 with the corner raised to 0.47x:

| | 0.25x coupling | 0.5x coupling | 1.0x coupling |
|---|---:|---:|---:|
| LR4 at 0.45x | -0.79 dB | -8.0 dB | -28.1 dB |
| LR8 at 0.47x | -0.06 dB | -8.4 dB | -52.5 dB |

Better on both sides at once: a steeper filter can place its corner closer to
the band edge. The sum of the two halves stays all-pass, so neither physical
band needed retuning.

### 25.6 What is still not resolved, and why it is not gated

The harness now measures and **prints** `outOfBandResonance`, the prominence of
a narrow line above the coupling Nyquist. It is deliberately **not** turned into
a pass criterion.

The tempting argument — "above the coupling Nyquist the boundary carries no
information, so nothing there can be a mode" — is true of the *boundary* and
false of the *network*: the waveguide modes do not stop there, and the
complementary valve-flow source excites them. Measured on the reference
four-cylinder, cutting that source drops the 4107 Hz peak from **31.2 to
22.1 dB**: it is therefore neither purely an image nor purely a mode. And that
reference engine, in its default geometry without a chamber, is an open
straight exhaust — which really does resonate.

Moving the reconstruction to order 8 did not shift it, which **rules out** the
reconstruction path as the main cause. Remaining lead, not pursued here: the
port medium, the valve conductance and the lift are zero-order-held streams at
the coupling rate that **multiply** the boundary instead of adding to it, and
that the reconstruction filter therefore never sees.

What can soundly be gated is the filter itself, and its regression now requires
40 dB on the first image and 80 dB on the second (against 24 and 40).

### 25.7 Correction: `ExpansionChamberMuffler` is not dead code

The audit claimed that `if (useCompiledTopology) continue;` in
`RealtimeEngineAudio.cpp` made this element unreachable. That is too strong:
`useCompiledTopology = sampleUsesPhysicalExhaust && acousticExhaustNetwork_`,
and `sampleUsesPhysicalExhaust` is false as long as no cylinder has a valid
thermoacoustic boundary. The reduced path is therefore a real fallback, not dead
code. It remains that the shipped catalogue reports `legacySamples=0`
everywhere: the figures of §19 describe the fallback, not the shipped voice.

### 25.8 Shipped measurements

Comparison baseline: the state before this pass.

- Big Twin idle resonance **27.0 dB -> 15.7 dB**, and it is no longer a narrow
  line at 439 Hz. Exhaust tail floor **-70.0 -> -80.3 dB**.
- Every narrow peak above its engine's coupling Nyquist disappeared: LS3
  19.1 dB@4156 Hz -> 498 Hz, K20 18.3 dB@4128 -> 967 Hz, Flat-6 14.6 dB@4312 ->
  492 Hz. They really were out-of-model content.
- Differentiation improved on four of six catalogue pairs; V8 versus radial
  0.308 -> 0.228, I2 versus radial 0.679 -> 0.616.
- Brightness rises on engines with narrow ducts and falls on those with a wide
  chamber: it is the geometry doing the work.

### 25.9 Tests added

All non-vacuous (checked by deliberately breaking the code under test):

- `ductWallLossRegression` now bounds the fit error **across the whole band**
  instead of only at the point where the fit is exact by construction, and
  checks the difference equation against the analytical magnitude. A single
  pole misses the new bound by 1.6 to 6.6 dB on every catalogue duct.
- `ductModeCutoffRegression`, anchored on the textbook value: a 50 mm duct in
  air cuts off at 4 kHz.
- `branchTrunkDelayRegression` measures the causal onset at the outlet with and
  without a 400 mm trunk: **35 samples measured against 35.9 predicted**.
- `ductMediumRegression` requires that giving every duct the cold state exactly
  reproduces the network that is told the whole path is cold.
- `areaStepScatteringRegression` pins the scattering of an area step on the
  closed form `T(m) = 4m/(1+m)^2` — the low-frequency limit of Munjal's
  transmission loss — within 0.05 for m = 2, 4 and 9.

## 26. Turbulent source at the outlet

The physical network reconstructed the blowdown and the duct radiation, but
created no turbulent mixing source where the hot jet meets the outside air.
`ExhaustJetNoise` fills only that role:

- mean flow shared between the outlets by area;
- velocity `m_dot/(rho*A)` and power `K*rho*A*U^8/c^5`;
- spectral centre at `St = 0.2`;
- causal modulation by the audio volume flow of the termination;
- a subsonic bound and a `4.5 ×` bound on the instantaneous excursion;
- stereo observer, directivity, distance and IR identical to the outlet;
- no feedback into the gas solver.

The driving power multiplier `100` is an authored value, isolated from the jet's
own coefficient `1e-4`. It does not claim to be a measured physical constant. A
zero setting cuts the whole layer for sound and CPU comparisons.
