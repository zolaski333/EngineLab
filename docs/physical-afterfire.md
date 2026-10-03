# Physical afterfire in the exhaust

EngineLab's afterfire is neither a sample nor added noise. The ECU carries
packets of unburned fuel and oxygen into the quasi-1D network; a local reaction
then releases energy in the cell or junction where the conditions are met. The
pressure rise travels through the same acoustic DAG, the same collectors,
mufflers and outlets as the normal blowdown.

## ECU strategies

Engine schema 6 explicitly separates three intents:

- `clean_dfco`: clean fuel cut, no fuel retained;
- `continuous_anti_lag`: continuous fuel and exhaust ignition, meant for a
  rumble/anti-lag and not for isolated pops;
- `discrete_afterfire`: chopped fuel packets, meant to produce separate
  combustions after lifting off the throttle.

Example of a discrete calibration:

```yaml
schema_version: 8
engine:
  exhaust_afterfire:
    strategy: discrete_afterfire
    enabled: true
    ignition_temperature_k: 800
    reaction_time_constant_s: 0.002
    reaction_efficiency: 0.95
    overrun_fuel_fraction: 0.08
    overrun_minimum_rpm: 3000
    overrun_maximum_throttle: 0.02
    overrun_pulse_hz: 2.0
    overrun_pulse_duty: 0.08
    overrun_pulse_timing_variation: 0.40
    induction_time_s: 0.004
    induction_reference_pressure_kpa: 101.325
    induction_activation_temperature_k: 13340
    induction_pressure_exponent: 0.989
    induction_equivalence_ratio_exponent: -0.577
    induction_decay_time_s: 0.008
    minimum_equivalence_ratio: 0.45
    maximum_equivalence_ratio: 1.80
    quench_temperature_k: 520
```

The fuel fraction is a mean value. In pulsed mode, the ECU normalises it by the
duty cycle. The final profile keeps fraction and duty at 8 %: the instantaneous
command therefore reaches full normal injection during a 40 ms window, twice a
second, for only 8 % mean mass. The old 18 % / 35 % / 4 Hz profile stayed open
for 87.5 ms. It spanned many injection opportunities and measured a 76.6 %
chemical duty on the Twin, almost 100 % on the K20: it was a rumble close to
anti-lag, not a series of pops. A 25 ms window at 5 %, on the other hand, could
miss the injections of a twin and leave up to four seconds between reactions.

The `discrete_afterfire` cadence is no longer a perfectly periodic square wave.
`overrun_pulse_timing_variation` shifts the packet boundaries with a
deterministic low-repetition sequence. A value of 0.40 around 2 Hz bounds each
interval between 300 and 700 ms at the ECU level; the measured reaction front
may deviate slightly from it because of transport and induction. Each packet
keeps the same duty relative to its interval, so the prescribed mean mass is
preserved. A value of zero restores exactly the historical regular cadence.

## Physical conditions

A site burns only if all of the following hold:

1. fuel and oxygen coexist locally;
2. the local equivalence ratio is inside the flammability window;
3. the gas or the wall exceeds the ignition temperature;
4. this mixture stays active for the induction delay;
5. an established flame is not below the quench temperature.

Each cell and each junction has its own persistent induction and combustion
state. The reaction consumes the species conservatively and adds
`fuel_mass × LHV` to the energy. It publishes a bounded event holding the exact
node, the axial position, the energy, the duration, the density, the speed of
sound and the local area.

`induction_time_s` is the reference delay at the threshold, at the reference
pressure and at `phi=1`. Schema 8 integrates `dt/tau(T,p,phi)` with an
Arrhenius law whose coefficients are all exposed; schemas 1 to 7 migrate with
zero exponents and get exactly their flat duration back. The old computation
silently divided the time by `(T - T_ignition) / 450 K`: a delay written as
4 ms thus became 1.8 s just 1 K above the threshold. That hidden factor stays
removed. The wall/gas origin of the flame is stored at ignition; it is no
longer reinterpreted after the reaction itself has heated the gas.

The chemistry keeps its state as soon as a reaction strategy is authored,
including under load. That oxidises hydrocarbon traces as they travel instead
of artificially accumulating sixty seconds of fuel and then igniting that old
inventory on lift-off. This maintenance oxidation publishes neither afterfire
telemetry nor an acoustic source, though. Both outputs stay strictly reserved
for a DFCO that retains fuel or an active wet rev limiter; the loaded check
just before lift-off always measures zero acoustic events.

The compact acoustic source is derived from the released heat. The total
pressure jump is:

```text
Delta p = (gamma - 1) × Qdot / (A × c)
```

The network then splits it between its two travelling characteristics; each
outgoing wave therefore receives `(gamma - 1) × Qdot / (2 × A × c)`. The signal
at the solver rate is reconstructed by the same LR8 anti-imaging filter as the
physical boundaries. A two-pole 25 Hz DC blocker only removes the quasi-steady
heat already carried by the mean flow. The old high-pass around 3–6 kHz, by
contrast, removed almost all of a reaction lasting a few milliseconds.

The source is injected at the node and axial position of the reaction. There
is no "pop" oscillator, no sample, no imposed audio periodicity and no global
source artificially placed at the outlet. A last-resort 100 kPa bound protects
the linear network; every sample that would reach it is counted, shown and
invalidates the validation harnesses.

The reconstruction keeps nothing above ~3.8 kHz: the 1-D network does not
compute that band. `ReactionCrackSynthesiser` adds it to every reaction voice:

- deterministic white noise, LR4 high-passed at the same crossover point, then
  a −6 dB/octave slope (that of a steep front);
- an envelope locked to the voice's compact pressure jump: 0.1 ms attack,
  0.5 ms release;
- an RMS level equal to `defaultCrackRatio` (1.0) × that jump;
- nothing without a reaction, so an engine without afterfire stays
  bit-identical.

The 1.0 ratio is not calibrated. On 2JZ pops, extending the simulator's slope
would call for 0.8, and extending an ideal front for 2.5. At ratio 1.0, the
4-8 kHz band gains +3.9 dB during the pops. `EngineLabAfterfireHarness
--no-reaction-crack` renders the same trajectory without this layer.

Each voice now keeps the power `energy / duration` for the exact duration of
the reaction step. The old render held it twice as long with a 0.5 ms minimum:
it duplicated the energy of short events and glued neighbouring kernels into a
continuous wave.

## Using it in AUDIO HQ

Afterfire is a per-engine option, disabled by default in the catalogue except
for the engine that authors it. The `AFTERFIRE  (pops and bangs on overrun)`
switch in the `REAL-TIME MIX` group turns it on or off and applies it at once,
without going through `APPLY LIVE`. It stays in sync with the box in the
physics group. If it is refused (dyno run in progress), it returns to the state
that is really active. For a profile that was left clean, enabling it installs
the discrete profile described below. An authored calibration is kept.

After each application, the editor shows a warning if the retained fuel is
below the chemistry's lean limit (`minimumEquivalenceRatio`, 0.45). This check
is `afterfireRetainedChargeBelowLeanLimit()` in `EngineTypes.hpp`. Retained
equivalence: `fraction` in continuous mode, `fraction / duty` (capped at 1) in
discrete mode. Measured on the 2JZ:

- 18 % continuous burns 0.0 % of the fuel;
- the same mass in packets burns 89 %.

Such a setting only dumps raw fuel.

The `APPLY LIVE` button now sends the calibration to the simulation thread
through a mailbox. It no longer rebuilds `EngineRuntime` and therefore keeps
engine speed, phase, gas inventories and wall temperatures. Applying is refused
during a dyno pull so as not to alter a measurement in progress.

`AUDIBLE DEMO` selects `discrete_afterfire`, a compact 2 ms reaction, 2 Hz
nominal, 8 % duty, 40 % timing variation and 8 % mean fuel. The plain toggle
also completes a calibration left at zero and installs the schema 8
thermochemical induction correlation. An already authored anti-lag or
afterfire calibration stays unchanged.

For a direct test:

1. select `Audio Physics Lab 689 Twin`;
2. enable `AUDIBLE DEMO`;
3. hold the engine above 3,000 rpm with more than 20 % throttle;
4. release completely.

The telemetry shows the strategy, the ECU blockers, the kW, the mg/s, the
number of reacting volumes and the lost events. Zero power is still a possible
physical result: cold line, mixture outside the window, clean DFCO or strategy
not armed.

## Measured proof of the acoustic path (product profile, 23 August)

The final check uses the lab Twin as catalogued, 60 s of warm-up, a loaded
interval just before lift-off and 8 s of overrun. The two renders below have a
strictly identical engine trajectory, chemistry and energy; the control only
removes the copy of the reaction events to the audio network.

| Case | Reactions before lift-off | Fuel burned | Heat events | Peak power | Audio peak | P99.9 | Crest |
|---|---:|---:|---:|---:|---:|---:|---:|
| reaction events not injected | 0 | 89.373 mg | 10 | 15.068 kW | 0.04764 | 0.03310 | 4.75 |
| reaction events injected | 0 | 89.373 mg | 10 | 15.068 kW | 0.05835 | 0.03349 | 5.73 |

Subtracting the WAVs gives a reaction contribution with a 0.03415 peak. Its
median RMS over 5 ms windows is exactly zero, then reaches 0.01229 on the pops:
it is no longer a continuous rise in volume. The ten isolated audio fronts are
460 to 1,090 ms apart. Their energy splits as 15.35 % between 20–120 Hz,
64.60 % between 120–500 Hz, 19.65 % between 500 Hz–2 kHz and 0.39 % between
2–8 kHz. The 1,479 finite-volume events carry 5,419.634 J; the maximum compact
jump is 80.902 kPa, without reaching the 100 kPa bound.

The active render uses 13.5 % of the mean budget of a 200-sample block and
15.9 % at p99, against 13.1 % and 14.2 % in the control. The simulation uses
20.6 % of the 4.167 ms step on average and 24.4 % at p99. None of the 1,920
blocks or steps overran.

These figures validate the software path and that the model is not empty. The
2 ms calibration remains an engineering estimate for the lab engine, not an
identification from a recording or an instrumented bench.

The detailed write-ups of the schema 8 thermochemical induction and of the
level/profile correction are in the docs archive (git tag
`archive/docs-2026-09`): `afterfire-induction-implementation-2026-08-22.md` and
`audio-level-afterfire-correction-2026-08-23.md`.
