# ECU tuner and hot reload

The **ECU** button opens a map editor modelled on calibration tools: table
selection, engine speed/load axes, coloured cells, highlighted operating point,
limits and active revision.

This tuner acts on the EngineLab simulation only. Its files and values must
never be flashed into the ECU of a real vehicle.

## Parameters that are actually live

The set created with each engine contains:

| Identifier | Shape | Effect |
|---|---|---|
| `fuel.target_afr` | 2D table engine speed × load | bilinearly interpolated AFR target; on a spark engine in petrol AFR (lambda × 14.7), metered at that lambda on any fuel (E85 at 9.81, methanol at 6.47); the ECU tuner shows and takes it in the fuel's own AFR (a cell of 13.2 reads 5.81 on methanol) |
| `ignition.advance_deg` | 2D table engine speed × load | absolute advance before dynamic corrections; on a diesel, the start of injection (deg BTDC), no earlier than the injection part's window |
| `idle.target_rpm` | scalar | idle speed the idle governor holds (not on a carburettor) |
| `limits.rev_rpm` | scalar | rev limiter threshold |
| `limits.launch_rpm` | scalar | launch control (two-step) rev limit; 0 = off |
| `knock.retard_deg_per_intensity` | scalar | knock control gain (spark-ignition only) |
| `boost.target_gauge_kpa` | curve by engine speed | boost the wastegate controller holds (turbocharged only) |
| `limits.overboost_kpa` | scalar | overboost cut: margin past the boost target where the fuel is cut; 0 = off, the default (turbocharged only) |
| `vtec.enabled` | scalar | VTEC on (1) or held on the low cam (0); only with VTEC cams fitted |
| `vtec.engage_rpm` | scalar | speed a whole cycle must stay above for the high cam to engage |
| `vtec.hysteresis_rpm` | scalar | how far under the engagement a whole cycle must fall to drop back |
| `vtec.min_throttle_percent` | scalar | throttle below which the high cam is not used |

Load is currently `MAP / ambient_pressure`, clamped between 0 and 4. The
default tables cover 0 to 400 %, with extra points above 100 % for
forced-induction engines. Coordinates outside the axes are clamped to the first
or last cell.

The AFR slider in the main window is an explicit trim from -3 to +3 AFR,
neutral at 0. The spark slider adds a trim in degrees to the table. Cranking,
temperature, transient enrichment, knock and rev-limiter logic apply after
that.

Knock control works as a production ECU's does: while the knock intensity is
above 0.1 (about 1 bar of oscillation, where measurements start calling a
cycle knocking) it pulls the spark back fast, 20 x the excess per second as a
share of its authority: a heavy event (0.5, decaying at 45/s) pulls about 1.3°,
within the 1-3° production controls take per knocking cycle. In a clean stretch it
gives the advance back at 1°/s. The retard therefore settles at the knock-limited
advance instead of needing knock to hold it. Its authority, the most it may
pull, is `knock.retard_deg_per_intensity` (12° by default; the key keeps the
name of the proportional control it replaced), read with
`knockRetardGainDegrees()`. Past that much retard the knock is no longer held.
Zero turns it off: the knock then runs free. Diesels have no entry.

A map is calibrated against that limit, as a calibrator does on the dyno, by
`ignition.knock_limit_curve` in the engine file: the spark the knock control
settles at with the throttle wide open, at six speeds, and the manifold
pressure it was measured at (`calibrateKnockLimit`, in the runtime). The ECU
holds the map under it in proportion to how far the manifold pressure has
climbed from the ambient (a boosted engine) or 0.7 of that pressure (one that
is not) to the measured one, so part load keeps its map; the knock control
trims what is left. The engine designer measures it for every spark-ignition
design when it is built; `EngineLabKnockLimitCalibration --filter <engine>
[--verify]` measures a catalogue engine and prints the curve.

Launch control (`limits.launch_rpm`, off by default) moves the limiter's cut
down to that speed while the vehicle stands in gear (under 5 km/h) with the
throttle past half, so a launch takes up the clutch from there; it lets go
as soon as the vehicle rolls, in neutral and on the dyno. The limiter cuts as
it does at the rev limit (the engine's limiter style), and the engine's rev
limit, which over-rev damage follows, does not change.

On a turbocharged engine, `boost.target_gauge_kpa` is the boost the
wastegate controller holds, in kPa above the atmosphere, by engine speed. It
owns the boost: the default table is flat at the configuration's
`wastegate_pressure_ratio` (the same gauge boost at any altitude, as before),
and the 3-D view's "Wastegate opens at" moves the whole table by the same
step. Where the turbo cannot make the target (low engine speed), the table is
a ceiling it does not reach. Saved tables without this entry and
superchargers fall back on the configuration's ratio.

The overboost cut (`limits.overboost_kpa`, 0 to 1,000 kPa) is a production
ECU's protection: while the manifold's gauge pressure runs that margin past
the boost target at the engine's speed, the fuel is cut, and it comes back
once the boost falls under half the margin. Zero, the default, turns it off:
the boost then goes wherever the turbocharger takes it. 40 kPa catches a
runaway (a fuel whose energy per kilogram of oxygen is far past a real
one's, a wastegate that cannot hold) and never a held boost, which the
wastegate keeps within 6 kPa; but the catalogue 2JZ's spool-up on the dyno
overshoots by more than 40 kPa for about a second, so a cut on by default
would have changed it (journal 2026-11-11). Saved tables older than it get
it at its default when they load (`addMissingDefaultSettings`), so the
tuner lists it.

The canonical bounds are shared by validation, the generated metadata and the
ECU: 9 to 30 for AFR (petrol AFR, lambda 0.61 to 2.04; a generated map
stays at 10.5 or leaner; tables saved under the former 18 are widened when
they load, values untouched; what still fires that lean is the combustion's
call: its lean misfires start at lambda 1.60 and take most cycles by 1.73,
Kolodziej et al. SAE 2017-01-0671, Shiomoto et al. SAE 780235), -10° to 55° for advance, 150 to 25,000 rpm for the
rev limiter, 50 to 3,000 rpm for the idle target, 0 to 125 g per cycle for a
diesel's fuel, 0 to 40° per unit of knock for the knock gain, 0 to 510 kPa for
the boost target (a pressure ratio of 6, the configuration's ceiling). A cell accepted by the tuner is therefore no longer silently
re-clamped to a different range. The dynamic corrections stay bounded to the
same domain after the trims are applied.

### Active limits

`activeRevLimitRpm()` (`SimpleEcuModel.hpp`) is the rev limit the ECU applies:
`limits.rev_rpm` when the calibration has it, else the authored
`ignition.rev_limit_rpm`. The engine state publishes it as `activeRevLimitRpm`,
next to `activeLimits`, a bitmask of `ActiveLimit` saying what holds the engine
back this frame: the rev limiter, knock retard (at least 0.5°), coolant
protection (above 108 °C), a diesel's smoke or quantity ceiling, an open
wastegate (boost held at its target), a saturated injector (open for at
least 99 % of its window), launch control, the overboost cut, and a
turbocharger past its top speed line (or burst). It is telemetry: nothing
reads it back.

Two things follow the active rev limit rather than the authored one. The
dyno's sweep stops at 0.95 of it (and 300 rpm under it); an untuned engine
keeps its validated ceiling, under the lower of redline and limiter. Over-rev
damage starts 2 % past it: under its own power the engine stays at its
limiter, and only a driven engine (a missed downshift) goes past. The redline
is a display mark. The piston-acceleration term (above 5,500 g) still caps a
limiter raised too far.

### Idle target

`idle.target_rpm` ("Idle target", 300-3,000 rpm) is the speed the idle
governor holds with the throttle shut: the idle-air opening on a spark
engine, the idle fuel on a diesel, four- or two-stroke. The default is the
engine's `idle_rpm`, so an engine runs exactly as before until it is edited;
saved tables without it fall back on `idle_rpm` (`activeIdleTargetRpm()`).
The idle air is sized for `idle_rpm`: above it the governor may open past the
bypass's full travel, two bypasses more per `idle_rpm` of raise, as an
electronic throttle cracks its plate, and keeps that extra air through a blip.
Measured at 1.6 x and 2 x `idle_rpm` on every catalogue engine
(`IdleStabilityRegression`, journal 2026-10-28): the settled idle within 2 %
of the target. Below `idle_rpm` the governor can only shut the bypass: an
engine whose closed-throttle leak alone outruns the target (the Hayabusa, the
Radial and the CP4 already do at their own) idles where its leak takes it.
The overrun fuel cut and the dashpot follow the target; the clutch, the
gearbox and the starter keep `idle_rpm`. A carburetted engine has
no governor and no such entry: its idle stop screw sets its idle
(`idle_screw_turns`, the 3-D view's carburettor).

### VTEC

An engine with VTEC cams (a second, longer and higher, cam lobe per valve,
`variable_profile_enabled` and `high_*` in the engine file) carries four VTEC
settings in its tables, and the tuner's top row a **VTEC ON / VTEC OFF**
switch. An engine without them shows **FIT VTEC CAMS** instead (not on a
two-stroke); clicking a valve in the 3-D view does the same with "VTEC
cams", where the high cam's duration and lift are edited, and takes them off
with "No VTEC cams". Fitted, the high cam is the low one plus 24 deg and
1.0 mm on the intake and plus 22 deg and 0.6 mm on the exhaust (a rotary's
staged ports: the intake only), stretched from the low cam's lift profile;
the engagement is two thirds of the rev limit, at 55 % throttle, as the
catalogue K20A's. Everything is taken live: no restart, no file to edit.

The ECU engages the high cam when a whole cycle runs at or above
`vtec.engage_rpm` with the throttle at `vtec.min_throttle_percent` or more,
and drops back when a whole cycle runs under the engagement less
`vtec.hysteresis_rpm`, or when the throttle falls under its minimum. Every
switch, the ECU's or a cam fitted or taken off, lands on a base circle of
that cylinder (`EngineLab.ValveProfileTransition`); cams fitted live start on
the low cam and engage on a cycle completed after the fit. Defaults are the
cams' own figures (`switch_rpm`, `switch_throttle`, no hysteresis), so the
K20A runs as before until they are edited. Every value of the range is
taken: an engagement under idle runs the high cam at idle, one past the
limiter never engages, a hysteresis past the engagement never drops back by
speed, 100 % throttle means the pedal floored (the plate follows the pedal
exponentially, so the ECU reads 99.9 %). A high cam smaller than the low is
allowed. The VTEC light beside the tachometer is lit while the valves run
the high cam, an outline on the low cam, struck through when the ECU holds
it off; the side panel says which cam runs and on how many cylinders.

### Anti-lag

A turbocharged petrol engine's tables carry `antilag.enabled` (0, off),
`antilag.mode` (0 rally, 1 launch), `antilag.braking_kept_percent` (50),
`antilag.aggressiveness_percent` (70), `antilag.min_rpm` (0.45 x redline),
`antilag.max_rpm` (0.90 x redline), `antilag.max_time_s` (4),
`antilag.egt_limit_c` (1,050), `antilag.air_percent` (40),
`antilag.spark_atdc_deg` (25), `antilag.lambda` (0.90) and
`antilag.cut_percent` (33); a zero exit is no exit. The tuner shows them as
one page, the Equipment window as two sliders (braking kept, aggressiveness)
with the rest folded under *Advanced*, and Drive has its switch.

Armed, the exhaust's reaction model runs, its rich limit widened with the
gas temperature (Zabetakis: 1.80 x (1 + 7.21e-4 (T - 298 K)), about 2.3 at
700 K; US Bureau of Mines Bulletin 627). While it works the ECU fires the
spark `spark_atdc_deg` after top dead centre, targets `lambda`, cuts the
spark of at least `cut_percent` of the firings with their fuel kept, keeps
the overrun fuel cut and overrun afterfire off and does not learn lambda.
Each cylinder cuts its own share of its own cycles, offset along the firing
order by the golden ratio, so every cylinder fires in turn (a pattern counted
over the engine's firings locked a fixed set of cylinders: at 90 % a 2JZ lit
only cylinders 1, 3 and 2, an I5 only cylinder 1). A four-stroke is fuelled
for the air its valve delivered, as a speed-density ECU fuels: the oxygen
trapped at intake close counts a cut cycle's retained charge as new air
(2JZ: 720 mg trapped for 560 delivered), and it ran AFR 10.4 for 13.2 asked
(now 13.5).

- **Rally** works with the driver's throttle at or under 5 %. Its cut and
  air are a feedback, not a fixed setting: the ECU models the engine's drag
  as its friction plus a shut throttle's pumping, 0.65 bar over the
  displacement (Heywood fig. 13-12, 0.5-0.9 bar), and holds the brake torque
  (filtered 50 ms) at `-braking_kept x drag` by a PI (1.5/s and 0.3 per drag
  of error) on an authority from 0 to 1 (`EngineState::antiLagAuthority`).
  Its lower half opens the throttle from 0 to `aggressiveness x air_percent`
  with 90 % of the firings cut (`ecuLimits::antiLagMaximumCutFraction`); its
  upper half keeps it open and gives firings back down to `cut_percent`. A
  lift starts at the middle: all the air, 90 % cut. Like a rally system it
  holds its air and kills the torque with cut firings, whose charges burn in
  the manifold; 2JZ lifted 2 s at 5,000 rpm, 50 % kept, no exhaust limit:
  143 kPa in the manifold over the last second, the turbo at 95 % of its
  loaded speed (16 kPa and 53 % disarmed), 1,160 C at the turbine inlet.
  100 % kept brakes as without it; 0 % holds the speed.
- **Launch** works only while launch control holds the engine (foot down,
  standing): the limiter decides fuel and spark, anti-lag adds its late spark,
  mixture and cut.

It lets go (`EngineState::antiLagBlockers`, one `AntiLagBlocker` bit per
reason) under `min_rpm` or over `max_rpm`; in neutral or with the clutch out
longer than 1 s; after `max_time_s` on one lift, until the throttle passes
30 % again (rally) or launch control lets go (launch); over `egt_limit_c`,
working again 50 C below it; at the rev limiter (rally), on the overboost cut
or a broken engine. The limit reads `EngineState::turbineInletTemperatureC`,
a probe at the turbine inlet (0.5 s), where the cut charges burn; the
cylinders' mean read 420-570 C and never let go. 1,050 C is Garrett's
G-Series rating (stainless housing, Mar-M wheel); the 2JZ reads ~1,020 C
after a full-boost pull, and with 0-50 % kept reaches 1,050 C within ~1 s of
a lift, so it lets go and comes back about every 0.5 s. When rally anti-lag
lets go with the pedal shut, the overrun fuel cut starts at once: the plate
it held closes over ~0.3 s, and the boosted manifold fired normal charges
(2JZ: +530-850 rpm on a lift).

### Idle and fuel-cut strategies (not tunable)

- **Fuel cut on overrun**: below 2 % throttle and above 1.65 × the idle
  target; fuel resumes below 1.5 × idle, ramped in at 3/s, or earlier when
  the engine falls fast: 0.45 s of its fall ahead of idle, at most 1.55 ×
  idle (a heavy engine's port film refills before it reaches idle). A rotary cuts
  above 2.5 × idle and resumes below 2.25 ×: its face waits 1,080° of the
  shaft for its first fire and its light shaft coasts down twice as fast.
  A two-stroke keeps its fuel on overrun.
- **After-start phase**: right after a start the fuel cut is inhibited while
  the start air opening decays. Opening the throttle past 20 % lifts that
  inhibit until the next start (lambda learning keeps its start state): a blip
  from a fresh start coasts down on a fuel cut, not fuelled through an
  idle-air opening.
- **Idle dashpot**: an idle-air opening that softens the return to idle. It is
  held only while the engine is still coasting down (losing at least 5 % of
  the idle speed per second) above 1.3 × idle; otherwise it decays, so an
  engine cannot hang at 1.4-1.5 × idle.
- **Lambda learning** is suspended for 2 s after a fuel-cut resume: the
  resume's transient mixture would otherwise be learnt as a steady error and
  lean the idle out.

The framework defines reserved keys for future volumetric-efficiency, VVT/VVL,
boost or wastegate maps. They are not instantiated in the default calibration
and are not consumed by the current ECU.

## Editing blocks of cells

Click a cell to select it; shift-click selects the block between it and the
last clicked cell; ctrl-click adds or removes one cell. The tool row then acts
on the whole selection at once:

- `-` / `+`: one step, ten units of the last displayed digit (0.1 for a table
  shown with two decimals, 10 rpm for a whole-number limit);
- `-5 %` / `+5 %`: scales the cells;
- `SMOOTH`: each cell becomes the mean of itself and its four neighbours
  (selected or not), read before the pass so the order does not matter;
- `INTERPOLATE`: fills the selection's rectangle bilinearly from its four
  corner cells (a curve: linearly between its two ends).

Results are clamped to the entry's hard limits. Every published change, typed
or from a tool, goes on an undo stack (200 steps); `UNDO` and `REDO` publish
the previous or next version of that entry live, switching to its table if
another one is shown. The history is dropped when it no longer applies (an
engine change removed the entry, or the store rejected the version). The
operations are in `calibration::applyCellOperation`, tested by
`EngineLab.CalibrationEditing`. The selector shows each entry's display name
with its id. Single-value settings that share a prefix with another one (the
id before its first dot: `antilag.*`, `vtec.*`, `limits.*`) are one page
instead, *Anti-lag settings (antilag.*)*: one row each, its name and unit,
its value, its description as the tooltip, the switch (`.enabled`) first. A
row typed is published at once, and the tools act on the selected rows
(`EcuTunerComponent::pagesOf`).

## Transactional publishing

The window edits a `CalibrationDraft`. When a cell loses focus or Enter is
pressed:

1. the text is converted to a finite number;
2. the complete draft is validated;
3. `expectedRevision` prevents overwriting a concurrent change;
4. a new immutable `CalibrationSnapshot` is published with an atomic swap.

The ECU loads this pointer once at the start of each external simulation frame.
It therefore sees either the complete old calibration or the new one, never a
mix of cells in the middle of the sub-steps. The reader does not take the
writers' mutex. A per-reader epoch holds old snapshots until the simulation has
acknowledged the new revision; their destruction thus stays on the publishing
thread. `atomic<shared_ptr>` is however not assumed to be lock-free on every
standard library. An exceeded limit, an invalid axis or a conflict keeps the
previous revision and reloads the editor from the active source.

This publishing does not replace the runtime: engine speed, temperatures,
combustion cycle, fuel film and transmission carry on without a reset.

## Loading, saving and watching

**LOAD** accepts a `.ecu.json` or `.json` file of at most 2 MiB, validates it
and publishes it in one transaction. **SAVE** writes the current snapshot. In
both cases, that path becomes watched. Tables saved before a setting existed
(VTEC, the knock limit...) load with that setting's default for the engine
running, on LOAD and on every reload of the watched file. A `.ecu.json` dropped
on the main window loads the same way, asking first when the running tables
hold edits not saved.

As long as the tuner window exists, its timer checks the file ten times per
second. A valid external save publishes a new revision; malformed or
out-of-range JSON shows the error and leaves the last valid revision in
service.

The watcher does not follow a file before a first load or save. Structural
reloads triggered by the live script, the JSON editor or the exhaust designer
keep the same store, the window and its watcher. Explicitly choosing or
importing another engine closes the window and creates that engine's default
set, unless that engine brings its own: a saved engine, an engine left with
unsaved edits this session, or an imported engine file with its tables inside
(or, from 0.9.1 or earlier, a `.ecu.json` file beside it; see "Saved engines
and session edits" in `architecture.md`).

## Calibration JSON format

The current schema is `1`. A calibration is a `scalar`, a `curve_1d` or a
`table_2d`. This reduced example shows a 2 × 2 AFR table and the rev limiter:

```json
{
  "schema_version": 1,
  "name": "Example calibration",
  "description": "Minimal map",
  "calibrations": [
    {
      "kind": "table_2d",
      "metadata": {
        "id": "fuel.target_afr",
        "display_name": "Target AFR",
        "description": "Engine speed and load",
        "unit": "afr",
        "display_precision": 2,
        "live_editable": true,
        "limits": { "minimum": 10.5, "maximum": 30.0 }
      },
      "x_axis": {
        "id": "rpm",
        "display_name": "Engine speed",
        "quantity": "engine_speed",
        "unit": "rpm",
        "breakpoints": [1000, 6000]
      },
      "y_axis": {
        "id": "load",
        "display_name": "Load",
        "quantity": "normalized_load",
        "unit": "ratio",
        "breakpoints": [0.0, 2.0]
      },
      "values": [14.7, 14.2, 13.8, 12.8]
    },
    {
      "kind": "scalar",
      "metadata": {
        "id": "limits.rev_rpm",
        "display_name": "Rev limiter",
        "description": "Engine speed threshold",
        "unit": "rpm",
        "display_precision": 0,
        "live_editable": true,
        "limits": { "minimum": 600, "maximum": 25000 }
      },
      "value": 7200
    }
  ]
}
```

In a 2D table, `values` is laid out by rows: every X-axis value for the first Y
point, then every value for the second Y point, and so on. The number of values
must therefore be `x_count × y_count`.

Axes must hold finite, strictly increasing numbers. Their unit must match their
quantity. For the known ECU keys, the contract is stricter, to avoid a map that
is valid but ignored: engine speed as `engine_speed/rpm`, then load as
`normalized_load/ratio` for a 2D table. Limits are hard: a single out-of-range
cell invalidates the whole document.

## C++ API

`CalibrationStore` is deliberately generic. Readers can request a scalar, sample
a linear curve or a bilinear table with typed `AxisCoordinate`s. An
incompatible unit or quantity returns no value rather than an implicit
conversion.

`CalibrationJson::parse` produces a draft, `publishJson` combines parsing,
validation and publishing, and `makeDraft` starts again from a snapshot. The
`EngineLab.Calibration` and `EngineLab.EcuCalibration` tests cover dimensions,
interpolation, limits, revision conflicts and concurrent reads.

## Limitations and next extensions

- no axis editing or visual comparison of two revisions yet;
- the datalogger writes a CSV file but the tuner shows no history trace of
  the active point, and there is no table auto-tune;
- no wastegate duty table: the boost target drives the controller, whose
  gains are fixed;
- the per-cylinder lambda correction compensates for fuel transport, but there
  is no detailed model yet of sensors, short/long-term trims, OBD strategies or
  torque management comparable to a production ECU;
- an explicit switch to another engine does not automatically transpose the
  maps onto new axes; it starts with that engine's values.
