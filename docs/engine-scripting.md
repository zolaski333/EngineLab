# `.els` engine scripts

The EngineLab DSL is a safe declarative language that compiles to an
`EngineConfig`. It is used to pick a base and override values with explicit
units. The extensions recognised by the application are `.els` and `.engine`.

This language is not compatible with ES2D `.mr` files and does not yet try to
reproduce all of their expressiveness.

## Minimal complete example

```text
# Comments start with # or //.
preset inline_four
name "Street Turbo I4"

let runner = 30 cm + 20 mm
let primary = 48 cm

set idle_rpm = 900 rpm
set engine.redline_rpm = 7200 rpm
set ignition.rev_limit_rpm = 7350 rpm
set intake.runner_length_mm = runner
set intake.runner_diameter_mm = 42 mm
set exhaust.primary_length_mm = primary
set exhaust.outlet_diameter_mm = 70 mm

set forced_induction.enabled = true
set forced_induction.type = turbo
set forced_induction.pressure_ratio = 1.35
set cylinder.all.bore_mm = 86 mm
set cylinder.2.ignition_offset_deg = -2 deg

ignition clear
ignition point 800 rpm, 10 deg
ignition point 3500 rpm, 29 deg
ignition point 7350 rpm, 28 deg
```

The repository also contains `examples/street-turbo.els`.

## Choosing a base

Without a base instruction, the compiler starts from an inline four. The
built-in presets are:

- `inline_two`/`i2`, `inline_four`/`i4`, `inline_five`/`i5`;
- `v6`, `v8`;
- `flat_six`/`boxer_six`;
- `radial_five`.

A complete existing engine can replace that base:

```text
base "../engines/my-engine.yaml"
name "Track variant"
set engine.redline_rpm = 8200 rpm
```

`base` accepts a v1 or v2 engine JSON or YAML. A v1 base is migrated in memory
and the compiled result uses the current v2 schema. The path is relative to the
file that holds the instruction.

`include` reads another script in the same variable and configuration context:

```text
include "shared/intake.els"
include "shared/track-ignition.els"
```

Instruction order matters: a `preset` or `base` met later replaces the
configuration accumulated so far.

## Variables, expressions and units

`let` defines a variable once. Names are case-insensitive. Expressions accept
`+`, `-`, `*`, `/`, parentheses, unary signs and `pi`.

Addition and subtraction require the same dimension. Multiplication only
accepts one dimensional factor and one dimensionless factor. Dividing by a
value of the same dimension produces a ratio; arbitrary compound units are not
inferred.

Recognised units:

| Quantity | Symbols |
|---|---|
| ratio | `ratio`, `%`, `percent`, `pct` |
| length / volume | `mm`, `cm`, `m` / `l`, `ml`, `cc`, `cm3` |
| mass | `mg`, `g`, `kg` |
| pressure / temperature | `pa`, `kpa`, `bar` / `c`, `degc`, `celsius` |
| angle / time | `deg`, `degree`, `rad` / `us`, `ms`, `s`, `sec` |
| frequency / engine speed | `hz`, `khz` / `rpm` |
| area | `mm2`, `cm2`, `m2` |
| mass flow | `mg_s`, `mgps`, `mg_per_s`, `g_s`, `kg_s` |
| specific energy | `j_kg`, `kj_kg` |
| velocity / force | `mm_s`, `m_s`, `mps` / `n` |
| friction / inertia | `ns_m` / `kg_m2` |
| power / torque | `w`, `kw` / `nm` |

For example `20 MPa` is rejected because `MPa` is not in this list; write
`200 bar` or `20000 kpa`.

## Settable properties

`set idle_rpm = ...` is a shorthand for `set engine.idle_rpm = ...`. The
families currently supported cover:

- engine: idle, mechanical rev limiter, inertia, friction, octane, ambient,
  cooling, bank angle, layout and name;
- global intake and exhaust geometry;
- rev limiter and spark curve;
- injection mode, window, rail, flow, film and cooling;
- physical cycle-to-cycle combustion variability and physical afterfire;
- solver frequency, sub-steps and resolution;
- forced-induction enable and main parameters;
- per-cylinder geometry, masses, friction, journal, bank and attenuation.

A `cylinder.all` target changes every cylinder. A numeric target uses the
cylinder identifier, never its position in the array:

```text
set cylinder.all.compression_ratio = 10.5 ratio
set cylinder.7.exhaust_primary_length_mm = 620 mm
set cylinder.7.connecting_rod_type = articulated
```

An unknown property is an error; it is not ignored. The executable reference
list lives in the tables of `src/scripting/src/EngineScriptCompiler.cpp`.

The [`physical-audio-lab.els`](../examples/physical-audio-lab.els) example
enables a moderate combustion spread and the afterfire reaction. The latter
schedules no pops: without unburned fuel, oxygen and hot enough gas in the
exhaust, it correctly stays silent.

## Diagnostics and validation

Each error carries an `ESxxx` code, the file, the line and the column. The
compiler checks in particular:

- incompatible units, division by zero and non-finite results;
- missing cylinder identifier and unknown property;
- include cycles and a maximum depth of 32 files;
- sources larger than 2 MiB;
- reading and decoding a base file;
- normalisation and complete final validation of `EngineConfig`.

The current runtime ultimately rejects engines that are not four-stroke petrol
engines, even though the reserved symbols `two_stroke` and `diesel` exist in
the parser in preparation for a future extension.

## Hot reload in the application

After importing a `.els` or `.engine` file, a worker watches every dependency
roughly every 250 ms. A save triggers a compilation off the UI thread:

- if it succeeds, a new immutable revision is published and the application
  builds a new runtime;
- if it fails, the revision and the pointer to the last valid configuration
  are kept, and the diagnostics are shown.

The watcher stays active after a successful reconfiguration. It also detects
changes to an `include` or to the JSON/YAML loaded by `base`.

This hot reload is structural. It avoids restarting the application, but it
replaces the simulated engine and resets engine speed, temperatures,
combustion, transmission and audio. The current `CalibrationStore` is shared
with the new runtime: live edits and the watched `.ecu.json` file stay active
without going through a reload. To change AFR, advance or the rev limiter
without resetting the dynamic states, use the [ECU tuner](ecu-tuning.md)
directly.

## Limitations compared with ES2D `.mr`

The language cannot yet declare new node types, user functions, loops,
conditions, part collections or a complete topology from scratch. It overrides
a preset or a canonical configuration. That is more bounded and easier to
validate, but much less expressive than the Piranha/`.mr` ecosystem of ES2D.
