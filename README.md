# EngineLab

**A four-stroke internal combustion engine simulator and real-time audio
synthesiser**, written in C++20 with JUCE.

EngineLab simulates engine thermodynamics cycle by cycle — induction,
combustion, exhaust — and makes that simulation *audible*: the sound comes
neither from samples nor from a bank of oscillators, but from the pressure
actually computed at the valves, propagated through a quasi-1-D duct network out
to the tailpipe.

Design an engine, draw its exhaust system, put it on the dyno, listen to it.

> ⚠️ EngineLab is a simulator built for exploration and listening. It is not
> validated thermodynamic analysis software, not an engine test bench, and not a
> calibration tool for a real vehicle.

---

## What EngineLab does

### Physics

- **Control-volume gas network** coupled to slider-crank kinematics, injection,
  flame propagation, torque derived from cylinder pressure, friction and pumping
  losses, turbocharging, and the driveline all the way to the vehicle.
- **Conservative quasi-1-D intake and exhaust**: every duct is meshed and
  solved, with signed SI mass flow at the valves, characteristic waveguides, a
  thermal wall model, and a passive radiation load at the outlet.
- **Physical exhaust elements**: primaries, collectors, junctions, resonators,
  expansion chambers (Munjal), porous packing (Delany-Bazley), catalysts, and
  multiple outlets.
- **Emergent cycle-to-cycle variability**: the coupling between gas dynamics,
  fuel film, wave action and the ECU makes consecutive cycles genuinely
  different, with no authored random dispersion.
- **Exhaust afterfire** fed by unburned fuel on overrun and ignited by the pipe
  wall — which has to heat up first, exactly as on a real engine.
- **Vehicle dynamics**: gearbox, clutch, and longitudinal load transfer driven by
  wheelbase, centre-of-gravity height and driven-axle layout.

### Audio

- **The exhaust path is driven entirely by simulated pressure.** No preset,
  noise source or blowdown oscillator is mixed into that physical path.
- Separate, individually soloable layers for intake, mechanical noise, the
  starter and forced induction (compressor, turbine, wastegate), each with its
  own aeroacoustic sources.
- **Structural NVH modes** of the head and block, either estimated per engine
  family or configured from measured, sourced data.
- The real-time audio callback performs **no allocation, no locking and no file
  access**; physics telemetry crosses SPSC queues.
- **Declarative, hot-reloadable voicing** (`voicing/*.yaml`): the mix is data,
  not code.

### Tooling

- **A catalogue of 16 engines** — naturally aspirated and turbocharged I4s, a V8,
  a flat-six, a supercharged V12, motorcycle twins and triples, an inline five,
  a TDI diesel, a five-cylinder radial — all in readable, editable YAML.
- **ÉCHAP. PRO**, a validated graph editor for exhaust systems: it rejects
  cycles, incomplete branches and inconsistent cardinalities, and reports the
  solver cost of an unusual geometry before applying it.
- **ECU tuner**: AFR and spark tables plus the rev limiter, applied live through
  a transactional snapshot — the engine keeps running and keeps its thermal
  state.
- **`.els` DSL**, declarative and unit-typed, with automatic watching of the
  script, its includes and its base file; an invalid save keeps the last valid
  configuration running and surfaces the diagnostics.
- **AUDIO HQ**: mute/solo, JSON scenarios, and WAV export at 48/96/192 kHz in
  24-bit PCM or 32-bit float, with optional stems.
- **Automatic dyno**, stepped or as a continuous ramp, with history, curves and
  CSV export.

---

## Quick start

Windows prerequisites: Visual Studio 2022 (or 2026) with the C++ desktop
workload, CMake 3.24 or newer, and Git. JUCE, nlohmann-json and yaml-cpp are
fetched by CMake at pinned revisions.

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --target EngineLabApp
```

On Visual Studio 2026, use `-G "Visual Studio 18 2026"`. The executable lands in
`build/src/app/EngineLabApp_artefacts/Release/EngineLab.exe`.

To build and run the full validation suite (29 test suites):

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Details on the tests, the deterministic harnesses and sanitizer builds are in
[docs/phase-0-1-2.md](docs/phase-0-1-2.md).

### Controls

Key bindings are editable from the **TOUCHES** button; `keybindings.json`
rejects unknown actions, duplicates and reserved shortcuts.

| Input | Default action |
|---|---|
| hold `A` / `S` | ignition / starter |
| `Q`, `W`, `E`, `R` | throttle 1 %, 10 %, 20 %, 100 % |
| `D` / `H` | automatic dyno / hold engine speed |
| `P` / `Tab` | pause / next screen |
| `1` to `5` | time scale 0.25×, 0.5×, 1×, 2×, 4× |
| up / down / left arrow | upshift, downshift, wheel brake |
| hold `Y` or `Shift` | declutch; `T` / `U` adjust the setpoint |
| `;` | next exhaust acoustic preset |
| wheel / drag / double-click | zoom, pan and recentre the engine view |

Holding `G`, `Z`, `X`, `C`, `V`, `B`, `J`, `K`, `L`, `O`, `N` or `Space` while
scrolling adjusts, respectively, the speed hold, volume, convolution, noise
bands, mix layers, simulation speed and fine throttle.

---

## Building and modifying an engine

Three levels are deliberately kept separate, from the most structural to the
lightest:

1. **JSON / YAML** describes the whole engine structure. Applying it replaces the
   simulation instance and resets its dynamic state.
2. **An `.els` script** picks a preset or a base file, then applies unit-typed
   modifications. The application watches the file and hot-reloads it.
3. **The ECU tuner** publishes a calibration without replacing the runtime: a
   validated cell becomes visible to the ECU on the next evaluation, with the
   engine still running.

The distinction matters. An ECU hot reload keeps the engine, its speed and its
thermal states; changing displacement, topology or geometry requires a new
instance and starts from its initial state. Replacements coming from the live
script, the JSON editor and the exhaust designer do however reuse the same ECU
store, so the maps and the tuner window stay active.

**ÉCHAP. PRO** works on a copy of the engine: generate a starting network, add
and configure its components, connect the nodes, assign every cylinder, then
**VALIDER ET APPLIQUER** (refused during a dyno run) and **EXPORTER** to JSON or
YAML.

Two example scripts are ready to import: `examples/street-turbo.els` and
`examples/physical-audio-lab.els`.

---

## Measure, don't guess

EngineLab ships a set of deterministic harnesses used as instruments, not only
as regression tests: each makes a physical quantity observable and comparable
against the literature.

| Harness | What it measures |
|---|---|
| `AudioRenderHarness` | renders the real real-time path offline: RMS, crest, DC, per-band spectral balance, exhaust-chain RT60 |
| `GeometrySensitivityHarness` | does changing the exhaust change the sound? — timbre per third octave, one factor changed at a time |
| `RealtimeBudgetHarness` | realtime factor: simulated seconds produced per wall-clock second, on the real runtime thread |
| `CombustionPhasingTests` | LPP, CA10-50-90 and IMEP across an rpm sweep |
| `PhysicsPerfHarness` | gas-exchange trace at crank-degree resolution, on both sides of the valve |
| `CyclicVariabilityHarness` | COV(IMEP) per cylinder, at held engine speed |
| `DynoSweepHarness` / `UserDynoHarness` | torque and power curves, CSV export |
| `AfterfireHarness` | the shape of overrun heat release, and its real effect on the audio |
| `IntakeDuctBench` | bit-exact fingerprint of the duct solver: proves an optimisation is not a physics change |

Reference figures come from engine and DSP literature, never from the
simulator's own current output.

---

## Documentation

The documentation under `docs/` is currently written in French.

**Models**

- [Simulation model](docs/simulation-model.md)
- [Overall architecture](docs/architecture.md)
- [Physical thermoacoustic architecture](docs/thermoacoustic-architecture.md)
- [Real-time audio architecture](docs/realtime-audio.md)
- [Cycle-to-cycle physical variability](docs/combustion-variability.md)
- [Physical exhaust afterfire](docs/physical-afterfire.md)
- [Structural NVH mode configuration](docs/structural-nvh-configuration.md)

**Guides**

- [The EngineLab DSL](docs/engine-scripting.md)
- [ECU tuner and file format](docs/ecu-tuning.md)
- [Custom exhaust systems](docs/custom-exhaust.md)
- [Declarative audio voicing](docs/audio-voicing.md)
- [Diagnostic exports, stems and order maps](docs/audio-diagnostics.md)
- [Offline HQ audio rendering](docs/audio-lot5-offline-hq-2026-07-29.md)
- [Audio workshop and honest controls](docs/audio-lot6-workshop-2026-07-29.md)

**Measurement logs** — measurements, hypotheses tested, and hypotheses
*refuted*:

- [Physics audit](docs/physics-audit.md)
- [Rework validation log](docs/rework-validation-log.md)
- [Final validation, 1 August 2026](docs/final-validation-2026-08-01.md)

`CLAUDE.md` collects the traps of this repository learned the hard way: what was
measured, what was refuted, and why some obvious-looking "fixes" are in fact
regressions. It is probably the single most useful file to read before touching
the code.

---

## Known limitations

- four-stroke only; petrol and direct-injection diesel still rely on global,
  semi-empirical models;
- 0-D cylinder chambers and low-band quasi-1-D networks — this is not 3-D CFD;
- audible propagation is linear, through characteristics aggregated per path:
  transverse modes, 3-D bends and mean-flow radiation correction are not
  resolved;
- global chemistry, and semi-empirical flame, knock and heat-transfer models;
- at most eight audio exhaust paths; branches are preserved in the gas solver,
  but their outlets do not yet have independent 3-D audio positions;
- the exhaust designer has no drag-and-drop, no undo/redo and no IR picker in
  the interface — the IR remains editable in JSON/YAML;
- catalogue structural modes stay estimated per engine family until sourced
  measurements are supplied; validation on a real bench with multiple
  microphones is still missing;
- **no 3-D backend.** The `render` module prepares the transforms, stable
  identifiers, scene bounds, snapshot interpolation and the `IEngineRenderer`
  interface, but the current view is direct 2-D JUCE rendering. See
  [the architecture document](docs/architecture.md#préparation-du-rendu-3d);
- no OBD diagnostics aimed at a real ECU;
- large engines (V8, V12) remain expensive for the physics thread. The
  application displays its realtime factor so that cost is visible rather than
  silently endured.

---

## Contributing

Contributions are welcome — see [CONTRIBUTING.md](CONTRIBUTING.md) and the
[code of conduct](CODE_OF_CONDUCT.md). Two rules are worth knowing before
opening a PR:

- the project builds with **zero warnings** (`/WX`), and a green PR means a full
  green `ctest`;
- this repository trusts measurement over code reading: if a change touches
  physics or audio, include the output of the matching harness.

Note that the user interface and the documents under `docs/` are in French,
while the code, its comments and this README are in English.

---

## Acknowledgements

EngineLab owes its existence to **[Engine Sim
2D](https://github.com/ange-yaghi/engine-sim)** by **AngeTheGreat** (Ange
Yaghi): that project is what showed a simulated engine could be *heard*, and it
is the direct inspiration for this one.

The exhaust impulse responses shipped in `assets/ir/` are in fact taken from it
under the MIT license; per-file provenance is documented in
[assets/ir/README.md](assets/ir/README.md) and the full license text is
reproduced in [assets/ir/LICENSE-es2d.txt](assets/ir/LICENSE-es2d.txt).
Copyright © 2022 Ange Yaghi.

The corpus of real recordings used for A/B listening tests is made of CC0
sources, credited in
[references/real-engine-audio/manifest.json](references/real-engine-audio/manifest.json).

Dependencies: [JUCE](https://juce.com/),
[nlohmann/json](https://github.com/nlohmann/json),
[yaml-cpp](https://github.com/jbeder/yaml-cpp).

---

## License

Distributed under the MIT license — see [LICENSE.md](LICENSE.md).
