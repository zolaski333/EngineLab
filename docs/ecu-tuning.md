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
| `ignition.advance_deg` | 2D table engine speed × load | absolute advance before dynamic corrections |
| `limits.rev_rpm` | scalar | rev limiter threshold |

Load is currently `MAP / ambient_pressure`, clamped between 0 and 4. The
default tables cover 0 to 400 %, with extra points above 100 % for
forced-induction engines. Coordinates outside the axes are clamped to the first
or last cell.

The AFR slider in the main window is an explicit trim from -3 to +3 AFR,
neutral at 0. The spark slider adds a trim in degrees to the table. Cranking,
temperature, transient enrichment, knock and rev-limiter logic apply after
that.

The canonical bounds are shared by validation, the generated metadata and the
ECU: 10.5 to 18 for AFR, -10° to 55° for advance, and 600 to 25,000 rpm for the
rev limiter. A cell accepted by the tuner is therefore no longer silently
re-clamped to a different range. The dynamic corrections stay bounded to the
same domain after the trims are applied.

The framework defines reserved keys for future volumetric-efficiency, VVT/VVL,
boost or wastegate maps. They are not instantiated in the default calibration
and are not consumed by the current ECU.

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
set.

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

- no axis editing, multi-cell selection, smoothing, undo/redo or visual
  comparison of two revisions yet;
- no synchronised datalogger, history trace of the active point or table
  auto-tune;
- only AFR, advance and rev limiter have a runtime effect;
- the per-cylinder lambda correction compensates for fuel transport, but there
  is no detailed model yet of sensors, short/long-term trims, OBD strategies or
  torque management comparable to a production ECU;
- an explicit switch to another engine does not automatically transpose the
  maps onto new axes; it starts with that engine's values.
