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
| `fuel.target_afr` | 2D table engine speed × load | bilinearly interpolated AFR target |
| `ignition.advance_deg` | 2D table engine speed × load | absolute advance before dynamic corrections; on a diesel, the start of injection (deg BTDC), no earlier than the injection part's window |
| `limits.rev_rpm` | scalar | rev limiter threshold |
| `limits.launch_rpm` | scalar | launch control (two-step) rev limit; 0 = off |
| `knock.retard_deg_per_intensity` | scalar | knock control gain (spark-ignition only) |
| `boost.target_gauge_kpa` | curve by engine speed | boost the wastegate controller holds (turbocharged only) |

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

The canonical bounds are shared by validation, the generated metadata and the
ECU: 10.5 to 18 for AFR, -10° to 55° for advance, and 600 to 25,000 rpm for the
rev limiter, 0 to 40° per unit of knock for the knock gain, 0 to 250 kPa for
the boost target. A cell accepted by the tuner is therefore no longer silently
re-clamped to a different range. The dynamic corrections stay bounded to the
same domain after the trims are applied.

### Active limits

`activeRevLimitRpm()` (`SimpleEcuModel.hpp`) is the rev limit the ECU applies:
`limits.rev_rpm` when the calibration has it, else the authored
`ignition.rev_limit_rpm`. The engine state publishes it as `activeRevLimitRpm`,
next to `activeLimits`, a bitmask of `ActiveLimit` saying what holds the engine
back this frame: the rev limiter, knock retard (at least 0.5°), coolant
protection (above 108 °C), a diesel's smoke or quantity ceiling, an open
wastegate (boost held at its target) and a saturated injector (open for at
least 99 % of its window). It is telemetry: nothing reads it back.

Two things follow the active rev limit rather than the authored one. The
dyno's sweep stops at 0.95 of it (and 300 rpm under it); an untuned engine
keeps its validated ceiling, under the lower of redline and limiter. Over-rev
damage starts 2 % past it: under its own power the engine stays at its
limiter, and only a driven engine (a missed downshift) goes past. The redline
is a display mark. The piston-acceleration term (above 5,500 g) still caps a
limiter raised too far.

### Idle and fuel-cut strategies (not tunable)

- **Fuel cut on overrun**: below 2 % throttle and above 1.65 × the idle
  target; fuel resumes below 1.5 × idle, ramped in at 3/s.
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
with its id.

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
both cases, that path becomes watched.

As long as the tuner window exists, its timer checks the file ten times per
second. A valid external save publishes a new revision; malformed or
out-of-range JSON shows the error and leaves the last valid revision in
service.

The watcher does not follow a file before a first load or save. Structural
reloads triggered by the live script, the JSON editor or the exhaust designer
keep the same store, the window and its watcher. Explicitly choosing or
importing another engine closes the window and creates that engine's default
set, unless that engine brings its own: a saved engine, an engine left with
unsaved edits this session, or an imported JSON engine with a `.ecu.json` file
beside it (see "Saved engines and session edits" in `architecture.md`).

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
        "limits": { "minimum": 10.5, "maximum": 18.0 }
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
