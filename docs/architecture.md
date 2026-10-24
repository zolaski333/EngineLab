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
| `EngineLabSimulation` | Deterministic orchestration of the models and state integration; a bit-identical worker pool for the gas networks |
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

Inside a step, `EngineSimulator` may borrow a small `CylinderWorkerPool` (on
engines with three cylinders or more, and on a two-stroke twin: at most half
the reported hardware threads, two workers, three from ten cylinders).
Per-runner intakes advance their runners on it; the exhaust network and a
joint intake manifold use it through `ExhaustGasNetwork::setParallelFor` for
each duct's share of an RK stage, handed out as one contiguous run of ducts
per thread: its update, cache, wall heat, interior residual and the face
fluxes of its ports, outlets and the interfaces it owns (the upstream duct's).
What couples ducts (junctions, restrictions, applying the fluxes, the shared
plenum) stays serial in a fixed order, so the result is bit-identical
whatever the worker count, including none
(`EngineLab.GasDynamics` checks a reversed dispatch order). A two-stroke's
exhaust is a `PartitionedExhaustNetwork` instead: its unconnected pipes are
networks of their own, each with its own CFL step, and one dispatch per
advance runs one pipe per claim (`EngineLab.PartitionedExhaustNetwork` checks
each pipe against a network of that pipe alone). A four-stroke's stays one
network. From eight
cylinders up, where each sub-step reads a frozen copy of the shared plenum,
the per-cylinder pass (`processCylinder`) and the end-of-sub-step publish of
each cylinder's state also run on the pool, one cylinder per item: it writes only that cylinder's state, and its engine-wide events
(spark and misfire counts) are applied in cylinder order afterwards
(`EngineLab.JointIntakeManifold` checks 0 against 3 workers on the LS3 and the
Merlin). Two-strokes keep it inline.

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
| `ControlPanel` | ignition, starter, dyno, throttle column and presets, Drive section (speedometer, trip computer, gears, brake, clutch, cruise control, each with its key), trims |
| `EngineViewport` | the GPU 3-D engine view with its overlays, or the 2-D cutaway |
| `ReadoutStrip` | tachometer and six live readouts |
| `SidePanel` | Dyno, Telemetry, Audio and Diagnostics tabs |
| `StatusBar` | fault summary, real-time factor, simulation speed, counters |

`Theme` holds the colour tokens, fonts and the `ui::LookAndFeel` installed for
the whole application, so the tool windows and alerts share it. `Widgets` holds
the buttons, segmented controls and slider rows the panels are built from.

**Display units.** The simulation keeps one internal unit per quantity (m/s,
kPa absolute, N m, kW, °C, mm, cm³, kg, g/s, L/100 km). `units::` in
`foundation/Units.hpp` lists the units offered for each quantity (exact
definitions: international mile and pound, US and imperial gallons, standard
gravity, mechanical hp, metric PS) and four presets: Metric (the default, the
simulator's own readouts), Metric bar/PS, US and UK. `ui::UnitFormat`, held in
the `DashboardModel`, formats the readouts the panels show; a pressure can be
shown absolute or gauge (relative to the engine's ambient air), and a pressure
that is already a difference (oil, rail, a duct against ambient) is never
offset again. The choice is in `⋯ › Units` and is saved in `units.json` next
to `keybindings.json`. The physics debug list of the Telemetry tab stays in
the internal units on purpose: it is the engineering view.

The main component keeps the keyboard: no button or slider takes focus, and a
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

### Saved engines and session edits

The engine menu lists the catalogue, then *My engines*: the files of
`%APPDATA%/EngineLab/engines` (`loadSavedEngines`, `SavedEngines.hpp`). A saved
engine is two files sharing a stem: `<name>.json`, the configuration as
*Export engine* writes it, and `<name>.ecu.json`, its ECU tables. Each is
written to a temporary file renamed over the old one, the tables first. A
damaged table file fails the whole read: the engine never runs on other tables
than its own. The JSON names the voicing the engine speaks with
(`audio_voicing`: family and key); the voicing itself is read back from the
catalogue's voicing files (`restoreAudioVoicing`).

Leaving an edited engine for another one keeps its edits for the session:
`MainComponent` stores its configuration and its `CalibrationStore`, and
choosing it again builds the runtime from them. *Return* reloads the engine
from disk and drops them. "Edited" is measured against the engine as it was
loaded (`EngineBaseline`): the configuration through its JSON encoding, the
tables through the JSON of their entries, without the store's revision. A dot
on the engine picker shows it; the menu marks the engines that hold edits.

*Save engine* writes a saved engine over its file; on a catalogue engine it
asks for a name (*Save engine as…*). A catalogue name is refused, since the
menu tells engines apart by name. Saving does not restart the engine:
`EngineRuntime::renameEngine` gives the running engine its new name, which a
dyno run reads when it starts. *Delete saved engine…* moves both files to the
recycle bin; the running engine carries on, unsaved. *Export engine* in JSON
writes the tables beside the engine the same way, and *Import engine* reads
them back when they are there.

### New engine

*New engine…* (the menu) designs an engine from nothing: no base engine.
`designEngine()` (`catalog/EngineDesign.hpp`) takes an `EngineSpec` (cycle,
fuel, use, layout, cylinder count, banks, bore and stroke or a displacement,
induction and boost) and derives everything else by rules fitted to the
catalogue engines, which were authored from their makers' figures: redline
from the use's mean piston speed, rods, pistons, compression, cams, intake and
exhaust sizes for the peak-power speed, plenum, throttles, injectors, ignition
template, gearing for the top speed the estimated power reaches. Each value is
returned as a `DesignValue` with its rule; any can be set by key
(`overrides`), and what derives from it follows. A rule that would fall
outside the simulator's limits (`validateEngineConfig()`: on a 50-250 cc
engine the flywheel, final drive, plenum, piston and throttle) is held at the
limit (`minimumRotatingInertiaKgM2` and the others, `EngineTypes.hpp`), and its
rule says so; a value set by hand outside them is refused by name. The
closed-throttle leak follows the throttle bore (0.6 mm2 at 60 mm and over).

The crank: banks at the layout's angles (a W's as given), cylinders shared
over them in id order, one row per pin position along the crank. A V's or a
W's cylinders of a row share one pin, as on the real engines (`crank_pins`,
`rowPinCrank()`, `CrankBalance.hpp`): a 90 deg twin fires 270-450, a V8, a
72 deg V10 and a 60 deg V12 fire evenly. Where a shared pin cannot fire six
cylinders or more evenly (a 60 or 90 deg V6, a W16), *automatic* splits the
pins of a row by the least angle that does (`splitPinCrank()`: 60 and 30 deg);
`shared` keeps them (a 90 deg V6 fires 90-150), `separate` gives each cylinder
its own pin, placed for the best balance (`evenFiringOrders()`), as inline and
flat engines have. A firing order given is kept, on shared pins when they
allow it; crank offsets give an uneven crank (a 270 deg twin), the cylinders
they put on one pin sharing it. A radial fires from its single throw. The
`crank_pins` value says which, and the firing intervals. A two-stroke gets one expansion chamber per cylinder, tuned to its
peak-power speed. Silencers, gearboxes and vehicles come from the family parts
(`parts/`); a designed engine has the default voicing only.

An engine file may hold `design:` (the spec, same keys) instead of `engine:`;
the catalogue loads it through the same function, so every harness runs a
designed engine. The window (`EngineDesignerWindow`) shows the derived values
and their rules as the spec changes, and names it from its cycle, fuel,
layout and cylinder count until a name is typed; *Advanced* adds firing order, crank
pins, crank offsets, W bank angles and setting any value by hand. *Build and run* first
runs the design off-screen for a few seconds with the live engine paused
(`measureRealtimeCapacity()`, `runtime/RealtimeCapacityProbe.hpp`: the
physics-only factor at the realtime harness's speed; the engine is cranked
with the throttle shut until it runs, at most 5 s simulated, then held on the
dyno, since a carburetted two-stroke never catches at full throttle) and asks before building
below `realtimeCapacityMargin` (1.14: the sound and the 3-D view cost 3-12 %
on top). Cost grows with cylinders and speed: a 12-cylinder sport car is at
the limit on the reference machine, a quad-turbo W16 at half. Then it saves the
engine under *My engines*. `EngineLab.EngineDesign` checks it.

### Live exhaust changes

An edit of `exhaust` and `exhaustPaths` only (the inspector's steppers, the
Exhaust editor) that keeps the network's topology (same ducts, junctions,
ports and outlets) is taken by the running engine:

1. `EngineRuntime::applyLiveExhaust`, on the caller's thread, builds the new
   graph and gas network (`EngineSimulator::buildLiveExhaustNetwork`), which
   is the costly part, and leaves them in a mutex-guarded mailbox.
2. The simulation thread takes it between two frames: the new network adopts
   the running one's state (`ExhaustGasNetwork::adoptStateFrom`), the
   simulator swaps it in, the runtime swaps its graph (the simulator holds a
   reference to it) and republishes the exhaust's static telemetry.
3. The app hands the audio its own graph built from the same configuration.
   `RealtimeEngineAudio::replaceExhaustGraph` builds and prepares the new
   acoustic network off the audio thread; the audio thread feeds it silently
   for 150 ms while its pipes fill, then crossfades over 30 ms. The retired
   network is freed by the app's timer, never on the audio thread.

`adoptStateFrom` keeps the intensive state where it was: equal meshes copy
cell for cell, different meshes take overlap-weighted means of densities,
momentum, energy and wall temperature. A resized pipe therefore holds more or
less gas at the same pressure and temperature. The wall temperature is the
state that matters: its time constant is tens of seconds, while the gas
refills in milliseconds (an exhaust swapped with a fresh network shows no
engine-speed step, but a cold wall). Anything else (another topology, the
dyno running, an audio network that fails to prepare) restarts the runtime
as above.

The same path remeshes the running exhaust: the view settings' *Exhaust
waves* choice (`ViewSettings::fineExhaustWaves`) passes 180 mm instead of the
production 360 mm as the target cell length, and every later restart of the
runtime keeps it. It never coarsens a mesh: a two-stroke's production cells
are already 100 mm (`EngineSimulator::productionExhaustCellLengthM`) and stay
so. 360 mm cells show 29 to 61 % less pressure swing than a
45 mm reference, 180 mm within 5 % on most engines; the simulation then has
13 to 20 % less headroom. The audio does not depend on it.

### Live bore, stroke and rod changes

`resizeCylinders` (foundation) gives every cylinder one bore and stroke and
the crank journals half the stroke as throw. An engine authored by
compression ratio keeps it; one authored by chamber geometry keeps its
piston-to-deck clearance and its compression ratio follows.
`resizeConnectingRods` gives every cylinder one rod length (at least 1.2
strokes; a radial's articulated rods are refused) and keeps the compression
ratio: one authored by chamber geometry keeps its piston-to-deck clearance,
its deck moving by the added length. `setCompressionRatio` gives every
cylinder one compression ratio (5 to 20): an engine authored by ratio takes
it as is; one authored by chamber geometry keeps its head chamber, gasket and
piston and moves its deck (a decked or shimmed block), refused when the
crown would have to rise past the gasket.
`EngineRuntime::applyLiveCylinderResize` hands the sizes to the simulation
thread, where `EngineSimulator::beginCylinderResize` applies them one crank
journal at a time, at the gas-exchange TDC (phase 360) of the first of its
cylinders to reach it: the chamber then holds its clearance volume and both
valves are near their crossing. The gas keeps its mass through the volume
step (`setVolumeAdiabatic`). With a ramp, each such TDC takes the size
interpolated at that moment until the target. A stopped engine takes the
target at once. Changed at once, the first cycle of each cylinder burns fuel
metered for the old one (one lean cycle on a larger engine); a ramp spreads
that over many small steps.

What was derived from the geometry follows it: the kinematics reference is
rebuilt after each change; the runtime republishes displacement, bore and
stroke to the audio, gives the driveline the new engine inertia and, outside
a dyno run, re-sizes the dyno absorber. Everything else reads the
configuration at every step. Not updated until a restart: the audio's
structural modes (`StructuralModalRadiator`, built from the mean bore and
stroke). Cylinder count, layout and fuel stay restart-only.

### Live injector, turbo, limiter, afterfire, flywheel, air and gearbox changes

The simulator reads `injection`, `forcedInduction`, `ignition`,
`exhaustAfterfire`, the crankshafts' inertia and the ambient pressure and
temperature only inside `step()`, and caches nothing derived from them but
the exhaust outlet's ambient reservoir, which it rebuilds, so
`EngineSimulator::replaceSettings` swaps them between two frames (the
crankshafts field by field: their layout stays). The pipes' outer walls keep
the outside temperature they were built with until a restart.
`EngineRuntime::applyLiveSettings` posts them the same way as an exhaust,
then gives the driveline the new engine inertia, gearbox and vehicle
(`DrivelineModel::replaceVehicle`, same gear count: the clutch, wheel and
vehicle states carry on) and the audio the new ambient. Refused, for a
restart: another injection mode, a forced induction added, removed or of
another type, another blade or lobe count, another number of gears, another crankshaft count, ratio,
phase or position (`settingsReplaceable`). The ECU tables (advance, AFR, rev
limit) are live through the ECU tuner, and override `ignition`'s curve and
limit whenever the store holds them. The sound's turbo layer takes the new
configuration at the start of an audio block
(`RealtimeEngineAudio::replaceForcedInduction`, `ForcedInductionAcoustics::
updateConfig`); its rotor and filter states carry on.

### Live camshaft changes

The valve train reads the camshafts (the engine's and each bank's) at every
sub-step and keeps only its phaser state, so an edit of the camshafts that
keeps the banks, and variable profiles and continuous valve control switched
on or off alike (`camshaftsReplaceable`), is taken live.
`EngineRuntime::applyLiveCamshafts` posts it like the other groups;
`EngineSimulator::beginCamshaftChange` swaps in the lift profiles, flow curves
and continuous control at once and glides durations, lifts, centrelines, flow
coefficients and the profile switch from their running values over the ramp
(`advanceCamshaftChange`, at every sub-step), or takes them at the next
sub-step without one. A valve open at that moment changes its lift on the
spot; the ramp spreads that.

### Live intake changes

An edit of the intake's sizes (runner length and diameter, per path or per
cylinder, plenum volume, throttle bore) that keeps its topology (same paths
and cylinders, throttle count, airbox or none: `intakeReplaceable`) is taken
like an exhaust. `EngineRuntime::applyLiveIntake` builds the new runner
networks (`EngineSimulator::buildLiveIntake`) off the simulation thread;
`replaceIntake` has them adopt the running state, swaps them in, and resizes
each plenum keeping its pressure, temperature and velocity
(`GasCell::resizeKeepingState`); the throttle area is read at every step.
The sound's intake network is rebuilt from the configuration and handed over
like the exhaust's (`RealtimeEngineAudio::replaceIntake`: 150 ms filling
silently, 30 ms crossfade).

### Which edits are live

`engineEditScope` (serialization) compares a running and an edited
configuration, both normalised, through their JSON, and sorts the difference
into the groups above: exhaust, cylinder sizes, settings, intake, camshafts,
or `other`.
`MainComponent::applyEngineEdit` takes each group live in turn and restarts
only for `other` or a group the running engine refuses; cylinder sizes and
camshafts take its ramp (the inspector's choice, 3 s from the JSON editor).
The JSON editor and the 3-D view's inspector go through it.

The top bar's A/B button (`MainComponent::switchAbVersion`, key 7) keeps the
version not running in `abVersions_` and hands the other one to
`applyEngineEdit` with no ramp, so the switch is heard at once. Each version
also keeps its ECU tables (a calibration snapshot), published back into the
running store when it is switched to; the ECU tuner reloads on the new
revision. Switching engine or
reloading the engine file clears both versions.

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

## 3-D engine view

The engine view is drawn by the GPU through OpenGL 3.2 (`juce_opengl`). Three
pieces split the work:

| Piece | Module | Role |
|---|---|---|
| `render::EngineModel3D` | `render`, no JUCE | meshes built from `EngineConfig` (mm, +Y up, +Z along the crank) and a pose per crank angle |
| `render::layoutExhaust` / `layoutIntake` | `render`, no JUCE | ducts laid out from the configured exhaust graphs and intake paths |
| `render::relaxRoutes` / `checkRoutes` | `render`, no JUCE | pipes bent clear of each other and of the engine; what a real system could not do, counted |
| `render::CrankClock` | `render`, no JUCE | the crank angle to draw at any display time |
| `render::GasFieldView` / `bindGasField` | `render`, no JUCE | the solver's gas field mapped onto the drawn ducts, as wave colours and wall temperatures |
| `ui::EngineSceneRenderer` | `app` | shaders, buffers, multisampling and motion blur |

Every moving part is placed with `evaluateCylinderKinematics()`, the function
the simulator uses, so inline, V, flat, radial and articulated-rod engines are
drawn as they are simulated. `EngineLab.EngineModel3D` checks every catalogue
engine at 72 angles: rods keep their length, wrist pins stay on the cylinder
axis, crowns stay below the deck. More than two banks (a W) are drawn bank by
bank like a V, a head per bank, banks next in angle interleaved half a pitch
along the crank as a VR's are; only a radial is drawn as one. On a designed
W16 (`EngineLabSceneExport --catalog-root`) that took duct clashes from 58 to
15 and ducts through the engine from 70 to 8 (journal 2026-10-19).
Rods on one crank journal are drawn on one pin; journals of their own
pointing the same way share a pin within a row only (a V's banks sit 0.45
pitch apart, rows 0.55), so a V12's two middle throws stay two.

Threads:

- the message thread copies the crank angle, engine speed, simulation rate
  (time scale × real-time factor, 0 when paused) and per-cylinder burn
  strength at 30 Hz into a mutex-protected block;
- the OpenGL thread runs `CrankClock`: in real time it extrapolates the last
  sample with the engine speed and acceleration and pulls back onto each new
  sample (time constant 80 ms, jump beyond 90°), so the picture turns at the
  simulated speed between samples. In slow motion it integrates rpm × factor
  instead;
- a pacing thread triggers frames at the chosen cap (30 to 240 fps). It also
  triggers the overlay repaints, so each tick renders one frame.
  "Unlimited" repaints continuously.

`EngineSceneRenderer` writes every part's transform and material into a buffer
texture and draws each material group with one call. Each frame goes into a 4×
multisampled buffer; with motion blur, the scene is drawn every 12° the crank
sweeps during half a frame (12 sub-frames at most) and the sub-frames are
averaged in a half-float buffer. Overlays are JUCE child components painted
over the OpenGL frame. View settings live in `view.json` next to the key
bindings. If the context cannot be created or a shader fails, the view falls
back to the 2-D cutaway and says why.

The older `RenderSnapshotBuilder` / `IEngineRenderer` groundwork is not used by
this view.

### Ducts from the configuration

`DuctLayout3D` draws what the solver simulates, not a generic manifold:

- **Exhaust.** Each path's component graph (or, without one, the graph the
  editor compiles from the scalar geometry) is drawn component by component,
  at its authored length and diameter; mufflers, catalysts, resonators and
  merges get the diameter of their authored volume. Pipes fed by a port, or
  feeding a junction with several inputs, are *routed* (below). Junctions and
  bodies sit on a trunk that runs along +Z (towards the flywheel), placed out
  from their ports and no higher than the crankshaft; a splitter spreads its
  branches, an X sends each bank back to its side. A collector's mouth is wide
  enough for its inputs side by side (they sit on a circle of radius
  (r + gap/2) / sin(π/N)). A collector fed by primaries sits behind the last
  port when every primary still reaches it (long tubes), else as far back as
  the shortest allows; the junctions of one path that would overlap (the two
  Y pieces of a 4-2-1) sit side by side across the trunk. An outlet is the open end of the pipe before it: with no authored
  length it is only a short collar with a rolled lip, and the inside of an
  open opaque tube is shaded dark, so the opening reads as a hole.
- **Intake.** Each runner has its length and taper, from the port to a
  plenum box of the authored volume (a drum round the crank axis on a
  radial). Upstream follow the throttle bores (on the plenum end, or on its
  face when there are several), the airbox volume, the inlet duct and its
  bellmouth, in the order the air crosses them.

**Routing.** `relaxRoutes` bends every routed pipe of the engine together.
Each is a chain of equal segments with straight ends held fixed: 0.8 diameter
out of its port, one diameter into its junction, along the trunk (the inputs
of a collector arrive side by side, as on a real one). Contacts with the other
pipes, with the intake ducts, and with the engine's solids (block, heads,
crankcase, sump, liners, pulley, flywheel, plenum and airbox boxes) are
projected out, then bends are held at 1.25 diameters or more and segments at
their length, until nothing moves (200 contact passes at most). Each input of
a collector takes the slot that makes the straight joins shortest in total.
Intake runners are routed the same way, clear of their plenum. The shapes are
invented; only their length, diameter and taper are the configuration's.

**Route check.** `checkRoutes` measures what a physical system could not do:
two ducts through each other, a duct through itself, a duct through the
engine, a centreline bent tighter than one diameter (circle through the
points half a diameter either side), a pipe drawn more than 3 % longer than
authored. Joints are not clashes: near an end that plugs into a port, a box
or another duct, within two radii. Over the 16 catalogue engines, before the
router: 107 clashes, 51 self-clashes, 0 through the engine, 259 tight bends,
6 stretched; with it: 7, 4, 0, 48, 7. `EngineLab.EngineModel3D` keeps every
count from growing. A pipe that must span more than its length is drawn
longer: the four 140 mm pipes of the LS3's X, the end primary of the I5, and
the two end primaries of the 2JZ, whose 430 mm cannot reach a collector they
must enter along its axis. `EngineLabSceneExport` writes the scene and the
issues as JSON, to look at them outside the app.

**Turbocharger.** The gas solver's turbine is a restriction face on the
outgoing face of the first junction where all of a path's cylinders have met
(see [simulation-model.md](simulation-model.md#forced-induction)). The view
draws one turbo per unit at the same place (`TurboPlacement`, `turbos()`):
each unit on the collector of the first path that drives it, the path's own
unit or its index modulo the count, as the simulator assigns them. A unit no
path drives gets no gas and is not drawn; two units on a flat six's two banks
are both drawn, each clickable with its own inspector (`EngineLab.EngineModel3D`
checks it). Gas bends out of the collector, enters the volute
tangentially and leaves the exducer along the trunk, where the path resumes;
the shaft runs along the crankshaft, the compressor on the far side of the
bearing housing, far enough out to clear the collector. Wheel sizes come from
the inducer and exducer diameters (about 70 % and 85 % of the wheels), blade
counts from the configuration (7 and 10 when unset). The wheels turn at the
simulated shaft speed, slowed with the crank, motion-blurred over the same
sub-frames. Housings are translucent in X-ray. The gas field shows the
manifold at turbine inlet pressure and the downpipe at turbine outlet
pressure. A turbo's charge piping is not drawn.

**Supercharger.** The solver has no compressor in its network either: the
supercharger raises the intake pressure by its ratio, geared to the crank.
The view draws it (`SuperchargerPlacement`) between the airbox and the
throttle of the first intake path with a single throttle mouth, beside the
throttle on the side away from the engine: the airbox or the inlet duct
feeds its eye along the throttle axis, a charge pipe runs from its volute
back to the throttle. Wheel size from the inducer diameter (an eye 1.2 times
the throttle when unset), blade count from the configuration. The impeller
turns at the drive ratio times the displayed crank angle.

Engine views frame the engine with its ports, runners, plenum and primaries;
the *Exhaust* view frames the whole system.

### Gas field on the ducts

The exhaust and the intake runners and plenums are coloured by the gas
solver's own state, not by an animation:

- **Capture.** The view asks `EngineRuntime::requestGasField()` for the field at
  the crank angle on screen. The simulator tracks that angle and copies the
  state (`GasFieldSnapshot`: up to 16 cells per exhaust duct and junction,
  intake runner and plenum) between two sub-steps when its crank crosses it,
  so in slow motion or frozen the picture is the most recent cycle at the
  displayed angle. In real time the view takes the latest field instead. A
  request that waits 0.25 s of simulated time (a stopped engine) is served at
  the end of a frame. The copy reads conservative states only, so the
  simulation is unchanged (`EngineLab.EngineModel3D` compares two
  simulators bit for bit); the runtime publishes it through a `try_lock`
  mailbox and stops capturing a quarter of a second after the view stops
  asking (the 2-D cutaway never asks).
- **Mapping.** `bindGasField` matches each drawn duct to its solver element by
  component id, or, on a path compiled from the scalar geometry, by node type
  and cylinder. Resonators with no outlet are acoustic side branches the gas
  solver does not carry; they keep their metal colour. Each duct vertex has a
  station (0 at the inlet, 1 at the outlet) and the shader interpolates
  between cell centres.
- **Resolution.** The real-time mesh has cells of about 0.36 m on a
  four-stroke (0.10 m on a two-stroke), so a four-stroke primary shows one to
  three cells. The view shows what the solver resolves, not the
  audio band, which the characteristic network carries separately. Measured
  against a 45 mm mesh at full load: in a primary it shows 29 to 61 % less
  peak-to-peak; 180 mm closes most of it but costs 13 to 20 % of real-time
  capacity (the LS3 drops to about 1.0), so the production mesh stays.
- **Colours.** The wave, not the pressure: each cell's departure from its
  own running mean (an exponential average over 0.5 s of simulated time, so a
  slowed or frozen view keeps it). Against ambient, a turbocharged manifold
  would read as one colour, at turbine inlet pressure. The prototype's scale:
  violet below the mean, grey at the mean,
  orange then pale yellow above, compressed with a 0.75 power. The scale is
  the strongest departure anywhere, decaying by 6 % per snapshot and never
  below 2 kPa; the legend prints it. Intake and exhaust share it, so their
  relative strength reads true: at idle and at full load (60 % of redline)
  the intake's peak is 0.6 to 4.8 times the exhaust's over the catalogue.
  The plenum is one lumped volume, so it shows one colour. A transient (a rev-limiter cut, a
  throttle step) shows as the whole exhaust above or below its mean until the
  mean catches up. The inspector gives the pressure against ambient.
- **Display mode.** *Pressure waves* in the settings menu (saved in
  `view.json`) chooses what the All layer shows: *Hidden* (the default:
  plain metal, nothing flickers), *Pulsation strength* or *Live pulses* (the
  wave above). The strength is each cell's root-mean-square departure,
  its square averaged twice over 0.5 s (once leaves a 5 % ripple at an idle's
  cycle rate, a visible shimmer), on a scale that is the strongest anywhere, never below
  1 kPa and decaying by 2 % per snapshot, from grey (still) to orange and pale
  yellow; it shows where the gas pulses hardest and holds still while the
  engine runs steadily. The Gas flow layer shows the strength when the mode
  is *Hidden*; the other layers show no waves.
- **Camera microphone.** *Microphone on the camera* (settings menu, *Sound*,
  saved in `view.json` as `microphoneOnCamera`, on by default) hears the engine from where the
  camera is heading: `EngineViewport::cameraMicrophones()` maps the eye and
  its right vector through `render::ListenerFrame`, and `MainComponent`
  passes them to `RealtimeEngineAudio::setMicrophones` when they change or a
  new renderer plays. The drawn openings (`EngineViewport::soundSources()`,
  from `render::ListenerFrame::sources`, rebuilt with each 3-D model) go to
  `RealtimeEngineAudio::setSoundSources` in the same way, so the exhaust is
  heard from the drawn tailpipe and the intake from its drawn mouth (see
  `realtime-audio.md`, *Camera microphone*).
- **Heat.** Each cell's wall temperature (the solver's finite-capacity wall)
  makes the steel glow from the Draper point, 798 K, to orange-yellow at
  1,300 K, in every layer.
- **Afterfire.** A flame at every outlet, lit by
  `1 − exp(−heat release / 10 kW)` of the afterfire at the captured instant.

Clicking a part opens an inspector: the part's authored geometry and, for a
duct, the solver's pressure range, gas and wall temperatures and cell count
at the displayed angle. The pick is a ray cast on the CPU against the posed
meshes of the visible layer; X-ray shells let the click through.

For a duct the inspector also draws an oscilloscope: the pressure of the
clicked cell (the cell nearest the click along the centreline) over one
engine cycle, against ambient. The view asks `EngineRuntime::requestGasProbe()`
for that cell; the simulator records it at every sub-step into 360 crank
bins (2° on a four-stroke), each bin keeping the latest cycle that crossed
it, and the runtime publishes the trace through a `try_lock` mailbox once per
frame. It reads the same cell the gas field shows (runners reversed to
gas-flow order) from conservative states only, so the simulation is
unchanged; it stops 60 frames after the view stops asking. A marker shows the
bin last written. Its sub-step resolution is what the snapshot cannot give:
a capture is one instant per crank angle.

An exhaust component can be resized from the inspector: steppers for its
length (10 mm, 1 mm with Shift) and diameter (1 mm, 0.1 mm), or a click on
the value to type it. Enter applies the typed value at once (clamped to the
stepper limits), Escape or a click elsewhere cancels it, and both hand the
keyboard back to the engine controls. `resizeExhaustComponent` (exhaust
library, no JUCE) writes the edit into the configuration: the component of
the path's network or, on a path authored as scalar geometry, the scalar
field behind it (the primaries share one size; a collector and an outlet
have no length; a silencer is sized by its body). It refuses an edit that
would change which components the path has (a silencer no wider than its
pipes stops being one) and, on the only path, mirrors the geometry into
`EngineConfig::exhaust`, which normalisation copies back and the solver
reads. A stepper press reaches the running engine 250 ms after the last one
(a live exhaust change, see above); *Reset* goes back to the size the part
had when it was selected. The camera stays and the part stays selected.

A cylinder liner or piston offers the engine's bore and stroke the same way
(1 mm, 0.1 mm with Shift), and its compression ratio (0.5, 0.1 with Shift;
only the values edited are applied, so a chamber-geometry engine's ratio
still follows its bore and stroke), with a choice between a 3 s ramp (the default) and
the next cycle (a live bore and stroke change, see above). The view shows the target
size at once. A connecting rod offers the rods' length the same way (5 mm,
0.5 mm with Shift).

The other parts edit the same way, each through `MainComponent::applyEngineEdit`
(see *Which edits are live*):

- an intake runner (length and diameter, its own if the cylinder has one,
  else the path's, which every runner on it shares), a plenum (volume) and a
  throttle (bore): `resizeIntakePart` (foundation), which mirrors a single
  path into `EngineConfig::intake` and the legacy fields. On a path metered
  by a carburettor the throttle is drawn as one (the slide's tower above the
  bore, the float bowl under it) and named *Carburettor*: it also offers the
  main and pilot jets as stamped flow numbers, the needle clip (1 richest to
  5) and the air screw's turns out (`tuneCarburettor`, mirrored the same
  way), which the running engine takes as an intake edit;
- the exhaust ports of a cylinder, and a junction, when the engine has
  another exhaust path that can join this one: a `choice` between
  *Separate* and *Joined* that restarts the engine with
  `joinExhaustPaths` (exhaust library). Each cylinder of the next path gets
  a copy of this path's first primary run (its components up to the first
  junction several pipes meet at), feeding that junction; the other path's
  pipes, silencers and outlet go, and banks that fed it feed this one. A
  boxer six's two 3-into-1 banks so become one 6-into-1; the collector keeps
  its size. Refused (not offered) when the primary meets no other pipe
  before its outlet, or when a turbocharger would be left without a path;
- the injectors, on each drawn injector (`PartRole::injector`: in the runner
  out of the head for port injection, in the chamber's roof on the intake side
  for direct injection), and also on the intake ports or the combustion: flow
  in g/min (the configuration keeps mg/s) and fuel pressure in bar, plus a
  diesel's start of injection in degrees before firing TDC (the window keeps
  its length and moves with it), with the window, live duty cycle, fuel per cycle and, for port injection, the wall
  film. The side panel's Telemetry tab has a *Fuel injection* section whose
  *Show in 3-D* button selects cylinder 1's injector
  (`EngineViewport::showInjectors`, back to the All layer if the current one
  hides it);
- a turbo: the wastegate pressure ratio, the rated ratio keeping its
  distance to it, and the turbine flow area; a supercharger: its pressure
  and drive ratios. A turbo and every exhaust component also offer a
  `choice` between *No turbo* and *Turbo*: it fits `sizedTurbocharger()` or
  takes the turbo off, which restarts the engine; the view then selects the
  new turbo (`Reselect::role`), since the parts have changed;
- the cylinder block: the air around the engine, as an altitude (the
  standard atmosphere's pressure there) and a temperature, and a switch
  between regulated and realistic engine temperatures (`thermal.regulated`;
  the inspector's edit `choice`, taken at once; its rows show the coolant
  temperature and, realistic, when the radiator fan is on);
- the gearbox (`PartRole::gearbox`, drawn on every engine with two gears or
  more): the clutch shift time and the quickshifter's ignition cut in ms,
  and the shift style as a `choice` (manual, quickshifter, quickshifter and
  blipper, dual clutch; see *Shift styles* in `simulation-model.md`). Its
  rows show the engaged gear, its ratio, the input shaft speed, the clutch
  slip and the road speed. A car's box sits in line behind the flywheel and
  its clutch disc, the layshaft below; a motorcycle's (vehicle under 500 kg)
  behind the crank, driven against it by the primary gear, with its clutch
  pack and sprocket. Each pair has the teeth of its ratio (within 6 %) and
  turns at that tooth ratio of the input shaft, whose angle the view
  integrates from the engine speed less the clutch slip (the crank's in
  neutral), never wrapped; a blue collar slides against the engaged gear
  and turns the output shaft with it. Its parts sit out of the engine
  framing and out of the ducts' solids, so neither the camera views nor the
  routing change;
- the crankshaft: the inertia of the crank and flywheel (every shaft scaled
  alike), with the total including the rods;
- an intake or exhaust valve: its cam's duration and lift; the cylinder
  heads: the intake and exhaust centrelines, with the lobe separation and
  the overlap. Every camshaft moves by the same amount, a variable profile's
  high cam too, over the 3 s ramp or at the next cycle.

Two controls at the bottom right slow things down, and they are different.
*0.25x / 0.5x / 1x* is the simulation's time scale
(`EngineRuntime::setTimeScale`, also on the time keys and the wheel): the
physics runs slower, and the sound with it. *Live / Strobe 1:50 / Strobe
1:250 / Freeze* is a stroboscope: only the picture slows down, showing the
most recent cycle at the displayed angle, while the engine and its sound run
on. A click in the 3-D view, like Enter or Escape in a text field (the dyno's
run name, a typed size), gives the keyboard back to the engine controls.

## Extension rule

A new engine cycle or fuel (as the two-stroke and the diesel did), a
torsional crankshaft or an OpenGL backend must come in through an explicit
model or interface. They
must not bypass validation, the conservation invariants or the real-time
boundaries.
