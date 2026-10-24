# Physical afterfire in the exhaust

EngineLab's afterfire is neither a sample nor added noise. Unburned fuel and
oxygen leave the cylinder into the quasi-1D network; a local reaction then
releases energy in the cell or junction where the conditions are met. The
pressure rise travels through the same acoustic DAG, the same collectors,
mufflers and outlets as the normal blowdown.

**Status (2026-10-24, `docs/journal.md`):** no rendered overrun yet reaches the
real pops measured on three recordings. Their 1 ms peaks stand 16-22 dB over
the running median; the best profile reaches 11-12 dB, against 8.5 dB without
pockets. The heat-to-pressure path is not what holds it back: the overrun's
heat is a continuous afterburn (100 % duty), every late-spark cycle burning its
own charge, so no unburnt charge collects to burn at once. Measured on the
voices, a 16 dB pop needs about 220 J released within 1 ms, the fuel of four
or five overrun cycles; misfires make the burn intermittent but each slug
(~50 J) still lights alone, at the port, on the next fired cycle's gas. No
element holds that fuel: the muffler is tailpipe air (phi ~0.03, at most
50 J), and a primary holds at most ~230 J, too lean to light, which would give
~90 J in its first millisecond if it did. That 220 J includes the silencer's
loss between the primaries and the outlet, which is the next thing to measure.

## Backfires are a listening switch, not an engine property

Backfires (overrun afterfire and the wet rev limiter) are turned on only from
the Audio workshop, for the engine running, and are off whenever an engine is
loaded. `disableBackfires()` (`EngineTypes.hpp`) clears the strategy, the
`enabled` flag and `ignition.limiter_keeps_fuel`. It is applied:

- when the application switches engine (catalogue, saved, designed or built);
- when an engine is saved or read back (`SavedEngines.cpp`);
- in the "modified" comparison (`EngineBaseline`), so the switch is never an
  edit.

An edit of the running engine keeps them: `keepBackfires()` gives every edit
(the 3-D view's bore or stroke, a designer, the JSON editor, an A/B switch, a
script reload) the running engine's switch and calibration, because those hold
copies made before the workshop's last change. Only another engine resets them.

The calibration under the switch (temperatures, times, spark angle, pocket
mixing) is part of the engine and is saved. Engine files can still carry the
activation keys and a script `ignition.limiter_keeps_fuel` (the serializers
read them, and the `EngineLabAfterfireHarness` measures them), but the
application drops them.

## ECU strategies

- `clean_dfco`: clean fuel cut, no fuel retained (the default);
- `late_spark_overrun`: what the Audio workshop installs. On a shut throttle the
  ECU keeps the normal charge and fires it at `overrun_spark_advance_deg`
  (negative is after top dead centre, -30 by default in the workshop), past the
  map's -10 deg floor. The flame is still burning when the exhaust valve opens,
  and the late, hot, partly burnt charge goes down the pipe.

The retained-fuel strategies (`continuous_anti_lag`, `discrete_afterfire`, with
`overrun_fuel_fraction` and the `overrun_pulse_*` chopper) were removed on
2026-10-24: the discrete rhythm was the chopper's 2 Hz, not the physics', and
the retained charge, mixed into 0.25 m cells, never burned on the lab Twin. A
file that still names them reads as `clean_dfco`, and the old keys are ignored.

## Unburned pockets

A production four-stroke cell is 0.25-0.30 m long. Mixed into it, a cylinder's
unburnt charge (equivalence ratio about 1.2) and the air the tailpipe draws in
average to 0.3-0.45, below the 0.45 lean limit: nothing burns, cell by cell.
`pocket_mixing_time_s` (zero: off) keeps that charge, at exhaust valve opening,
as one pocket per cylinder (`ExhaustGasNetwork::emitUnburnedPocket`, at most 32
per network) with its own composition and temperature:

- it is carried at the local gas velocity, from duct to duct and through
  junctions, and leaves at the outlet;
- its temperature relaxes to the host cell's over the mixing time, and after
  three of them it has mixed into the cells;
- it lights from its own temperature (the same Livengood-Wu integral as a
  cell) or from a burning host cell, and is checked against the flammability
  window on its own equivalence ratio;
- it burns at the reaction time constant, out of the fuel and oxygen its host
  cell actually holds, so it adds no mass; the released heat is a source like
  any other.

Pockets are emitted only while an overrun afterfire or a wet limiter is
active, so a loaded engine is bit-identical with or without them.

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
zero exponents and get exactly their flat duration back. The wall/gas origin of
the flame is stored at ignition; it is not reinterpreted after the reaction
itself has heated the gas.

The chemistry keeps its state as soon as a reaction strategy is authored,
including under load. That oxidises hydrocarbon traces as they travel instead
of accumulating sixty seconds of fuel and igniting that old inventory on
lift-off. This maintenance oxidation publishes neither afterfire telemetry nor
an acoustic source: both stay reserved for an active overrun afterfire or wet
rev limiter, and the loaded check just before lift-off measures zero acoustic
events.

## The acoustic source

The compact acoustic source is derived from the released heat. The total
pressure jump is:

```text
Delta p = (gamma - 1) × Qdot / (A × c)
```

The network then splits it between its two travelling characteristics; each
outgoing wave therefore receives `(gamma - 1) × Qdot / (2 × A × c)`. The signal
at the solver rate is reconstructed by the same LR8 anti-imaging filter as the
physical boundaries. A two-pole 25 Hz DC blocker only removes the quasi-steady
heat already carried by the mean flow.

The source is injected at the node and axial position of the reaction. There
is no "pop" oscillator, no sample, no imposed audio periodicity and no global
source placed at the outlet. A last-resort 100 kPa bound protects the linear
network; every sample that would reach it is counted, shown and invalidates
the validation harnesses. Each voice keeps the power `energy / duration` for
the exact duration of the reaction step.

The reconstruction keeps nothing above ~3.8 kHz: the 1-D network does not
compute that band. `ReactionCrackSynthesiser` adds it to every reaction voice
(white noise high-passed at the crossover, a -6 dB/octave slope, an envelope
locked to the voice's jump: 0.1 ms attack, 0.5 ms release, at `defaultCrackRatio`
1.0 × that jump, uncalibrated). `EngineLabAfterfireHarness --no-reaction-crack`
renders the same trajectory without it.

Measured on the lab Twin with pockets (journal 2026-10-24): the reaction voices
change the overrun's 1 ms peaks by nothing measurable (11.3 dB with them,
11.6 dB without). What the pockets add reaches the observer through the
finite-volume gas itself.

## Using it in the Audio workshop

The `AFTERFIRE  (pops and bangs on overrun)` switch in the `REAL-TIME MIX`
group turns backfires on or off and applies at once, without `APPLY LIVE`. It
stays in sync with the box in the physics group, and returns to the live state
if it is refused (dyno run in progress). The wet limiter has its own box in the
physics group. Both are off whenever an engine is loaded.

The physics group exposes the ignition temperature, the reaction time, the
efficiency, the overrun spark angle and the pocket mixing time. On an engine
that never authored an afterfire calibration (900 K, 10 ms, no pockets),
turning backfires on installs the measured profile: 800 K, 2 ms, -30 deg,
10 ms pockets, and the schema 8 induction correlation in place of a flat timer.
An authored calibration is kept. `AUDIBLE DEMO` installs the same profile with
the wet limiter and a 6 % cycle variation.

`APPLY LIVE` sends the calibration to the simulation thread through a mailbox.
It keeps engine speed, phase, gas inventories and wall temperatures, and is
refused during a dyno pull.

For a direct test:

1. select `Audio Physics Lab 689 Twin`;
2. turn `AFTERFIRE` on (or `AUDIBLE DEMO`);
3. hold the engine above 3,000 rpm with more than 20 % throttle;
4. release completely.

The telemetry shows the strategy, the ECU blockers, the kW, the mg/s, the
number of reacting volumes and the lost events. Zero power is still a possible
physical result: cold line, mixture outside the window, strategy not armed.

`EngineLabAfterfireHarness --backfires` measures what the switches do, on the
engine's own calibration. `--late-spark DEG` and `--pocket-mix-ms MS` change
one factor; `scripts/pop_metrics.py` counts pops in the render the same way as
in a recording.

The 2026-08 write-ups of the schema 8 induction and of the discrete-slug level
correction are in the docs archive (git tag `archive/docs-2026-09`):
`afterfire-induction-implementation-2026-08-22.md` and
`audio-level-afterfire-correction-2026-08-23.md`.
