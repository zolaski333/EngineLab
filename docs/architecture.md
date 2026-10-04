# Architecture

EngineLab separates the physical models, their orchestration, real time and
presentation. The computation libraries stay in standard C++; JUCE is confined
to audio and the desktop application.

## Modules and dependencies

```text
foundation
  ├── calibration ───────────────┐
  ├── events                     ├──► ecu
  ├── physics                    │
  ├── exhaust ──► events         │
  ├── serialization              │
  ├── diagnostics                │
  └── render                     │
                                 ▼
events + physics + ecu + exhaust ──► simulation
                                        │
                                        ▼
                                     runtime ──► audio (JUCE/DSP)

foundation + serialization ──► scripting
catalog + diagnostics + serialization + scripting
       + calibration + render + runtime + audio ──► app (JUCE)
```

| Target | Responsibility |
|---|---|
| `EngineLabFoundation` | Configuration and state types, validation, normalisation and a generic SPSC queue |
| `EngineLabCalibration` | Typed axes/units, scalars/curves/tables, validation, immutable snapshots and ECU JSON |
| `EngineLabEvents` | Firing events and intra-cycle pressure frames |
| `EngineLabPhysics` | Conservative gas, kinematics, P·dV, injection, flame, knock, valvetrain and Helmholtz intake |
| `EngineLabEcu` | Map interpolation, enrichments, corrections and rev limiter |
| `EngineLabExhaust` | Topology compilation, routes, aggregated gas properties and acoustic metadata |
| `EngineLabSimulation` | Deterministic orchestration of the models and state integration; no threads |
| `EngineLabRuntime` | Simulation thread, transmission/vehicle, dyno, snapshots and bridge to audio |
| `EngineLabAudio` | SI exhaust characteristics, passive radiation, explicit convolution and non-exhaust layers |
| `EngineLabSerialization` | JSON/YAML round-trip in engine schema 2 and migration of v1 files |
| `EngineLabScripting` | Safe compilation of the `.els` DSL and a background dependency watcher |
| `EngineLabCatalog` | Loading of the shipped engines and parts |
| `EngineLabDiagnostics` | Explanatory diagnostics for the UI |
| `EngineLabRender` | API-independent 3D scene, fixed snapshots and interpolation |
| `EngineLabApp` | JUCE composition, files, main window panels, ECU tuner, exhaust designer, audio workshop |

## State ownership and threads

`EngineSimulator` owns the physical states and advances only when its caller
invokes `step`. `EngineRuntime` is its application-level owner and runs the
simulation on a `std::jthread`. It publishes a protected `EngineState` for the
UI, slow telemetry through atomics and two SPSC streams: discrete events and
continuous pressure frames.

The JUCE callback consumes these streams. It compiles no script, reads no file
and takes no mutex. Impulse-response decoding, allocations and DSP preparation
happen before rendering.

`EngineScriptHotReloader` owns a separate worker. It watches the root script,
its `include`s and its optional `base`, compiles off the UI thread, then
publishes an immutable state. The UI replaces the runtime only after a complete
compilation and validation.

The calibration watcher is lighter: the ECU window timer detects a file change,
parses and validates the document, then calls `CalibrationStore::publish`. ECU
readers obtain the snapshot with an atomic load at the start of an external
frame and never see a partial draft. A per-reader epoch keeps replaced
snapshots alive until they are acknowledged; their destruction therefore
happens on the publishing thread, not on the simulation thread. This strategy
avoids a reader mutex but does not assume that `atomic<shared_ptr>` is
lock-free in hardware.

## Main window

`MainComponent` owns the runtime, the audio device and the files. It no longer
paints anything itself. Once per 30 Hz tick it copies everything the screen
shows into a `DashboardModel`: the engine state, the render snapshot, the
telemetry ring, dyno runs, faults, runtime counters and the audio mix. It then
asks each panel to refresh. Panels read the model and never touch the runtime.
User actions come back as callbacks that `MainComponent` wires in its
constructor.

| Panel | Area |
|---|---|
| `TopBar` | engine picker, run state, Exhaust / ECU / Audio windows, `⋯` menu |
| `ControlPanel` | ignition, starter, dyno, throttle column and presets, load and trims |
| `EngineViewport` | layer tabs and the 2D cutaway (zoom, pan, recentre) |
| `ReadoutStrip` | tachometer and six live readouts |
| `SidePanel` | Dyno, Telemetry, Audio and Diagnostics tabs |
| `StatusBar` | fault summary, real-time factor, simulation speed, counters |

`Theme` holds the colour tokens, fonts and the `ui::LookAndFeel` installed for
the whole application, so the tool windows and alerts share it. `Widgets` holds
the buttons, segmented controls and slider rows the panels are built from. The
main component keeps the keyboard: no button or slider takes focus, and a
mouse wheel a child does not use goes up to the main component's mouse-wheel
handler (`mouseWheelMove`).

## Transactional boundaries

### ECU calibration

An editor works on a `CalibrationDraft`, which may be temporarily invalid.
`publish` validates the whole and optionally checks `expectedRevision`. On
success, a new `CalibrationSnapshot` is swapped in atomically. On error or
conflict, the active pointer and its revision do not change.

This operation recreates neither `EngineRuntime` nor `EngineSimulator`. It
suits the parameters the ECU reads at every evaluation: AFR, advance and rev
limiter in the current integration.

### Structural configuration

JSON, YAML and `.els` all produce a complete `EngineConfig`. The decoders accept
engine schemas 1 and 2; normalisation migrates v1 to v2 in memory, and every new
export writes v2. Normalisation makes explicit topologies authoritative, then
validation checks identifiers, ranges, cylinder assignments, solver frequency
and acyclic graphs. An accepted configuration is used to build a new runtime.
The old one is stopped only after its replacement could be created.

The replacement is safe but is not a state migration: engine speed,
temperatures, fuel films, combustion phases, transmission and audio queues
restart from the initial state. For structural reloads of the same engine (live
script, JSON editor, exhaust designer), the new runtime reuses the same
`CalibrationStore`: maps, tuner and ECU watcher survive. Explicitly choosing or
importing another engine creates its default set.

## Physical contracts

- `GasCell` conserves species, internal energy, volume and 2D momentum.
  `ConservativeGasSystem` is the only transfer point between cells.
- `MechanicalKinematics` provides positions, velocities, accelerations and
  lever arms, including master/articulated-rod geometry; simulation and render
  scene thus share the same reference.
- `FlamePhysicsModel` computes delay, speed and progress without depending on
  the UI or audio. Its current geometric closure is a cylindrical effective
  volume `πr²h`, not a resolved 3D flame geometry.
- `IndicatedWorkModel` integrates the signed P·dV loops as telemetry. The
  crankshaft receives the instantaneous torque from the pressure, not a
  reinjected mean torque.
- `ExhaustGraph` turns a configuration into a bounded DAG, then
  `ExhaustNetworkLayout` discretises each component into quasi-1D ducts and
  finite junctions. `ExhaustGasNetwork` conserves species, momentum and energy,
  including at valves and outlets. The renderer receives a signed SI boundary
  and propagates its high band through passive characteristics, separately for
  each path.

## 3D rendering groundwork

The `render` module depends on neither JUCE nor OpenGL. `RenderSnapshotBuilder`
converts `EngineConfig` and `EngineState` into parts with a stable identifier,
parent, XYZ transform, activity and temperature. Cylinder stations are placed
along Z, while the solver kinematics stay resolved in their XY plane. A fixed
array caps the number of published parts at 256.

`RenderSnapshotInterpolator` keeps two frames and interpolates the transforms,
including angles along the shortest path. `IEngineRenderer` only defines
`prepare`, `resize`, `render` and `release` with a neutral camera and viewport.

This groundwork makes an OpenGL integration possible without touching the
physics, but it is not a renderer yet. Still to do:

- write the backend and manage the OpenGL context;
- define meshes, materials, lighting, shaders and a resource cache;
- choose the synchronisation with the render thread;
- wire the camera and interaction to the backend;
- extend the scene to collectors, intakes, accessories and effects;
- replace or adapt the JUCE 2D view, which still reads the simulation state
  directly despite producing a `RenderSnapshot` in parallel.

## Extension rule

A two-stroke or diesel engine, a torsional crankshaft, a 1D acoustic solver or
an OpenGL backend must come in through an explicit model or interface. They
must not bypass validation, the conservation invariants or the real-time
boundaries.
