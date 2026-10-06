# Changelog

All notable changes to EngineLab are listed here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/) (pre-1.0: minor versions may break
file formats, always with an in-memory migration).

## [Unreleased]

### Changed

- New main window. A top bar holds the engine picker, the run state and the
  Exhaust, ECU and Audio windows. Engine controls and a vertical throttle with
  1/10/20/100 % presets are on the left. The engine view sits in the middle with
  layer tabs, and the tachometer and six live readouts run below it. A tabbed
  side panel shows the dyno, telemetry, audio mix and diagnostics, and a status
  bar shows faults and real-time counters. The old full-screen pages
  (load simulation, mixer, oscilloscope, physics debug) are now side-panel
  tabs, and `Tab` cycles through them.
- Edit JSON, import/export, dyno CSV export, key bindings and full screen moved
  to the `⋯` menu in the top bar.
- One dark theme for every window, with key caps shown on the controls
  they trigger. Inter and JetBrains Mono are embedded, so the interface looks
  the same on every machine.

### Added

- 3-D engine view rendered on the GPU, replacing the 2-D cutaway: pistons,
  rods, crank throws, valves and flames follow the simulator's kinematics and
  turn at the simulated engine speed. X-ray or solid block, four layers,
  front / side / three-quarter views, an orbit camera, real time, 1:50, 1:250
  or frozen playback, and a per-cylinder cycle panel. The settings menu offers
  a 30 to 240 fps cap or unlimited, VSync, motion blur and 4× anti-aliasing.
  The 2-D cutaway stays available there, and is used automatically without
  OpenGL 3.2.
- The 3-D view lays out the exhaust and intake from the engine's
  configuration: every component of the exhaust graph with its length,
  diameter and volume, and the runners, plenum, throttle bores, airbox and
  inlet duct of each intake path. A new *Exhaust* camera view frames the whole
  system. An outlet is drawn as the open end of its pipe (a short collar
  with a rolled lip), not as a closed can.
- The exhaust and intake show the gas solver's pressure waves at the crank
  angle on screen (in slow motion, the latest cycle at that angle): violet below each
  cell's running mean, grey at it, orange to pale yellow above, with a legend
  giving the scale. Exhaust walls glow red above 798 K, and an afterfire lights a
  flame at the outlets.
- Click a part in the 3-D view to inspect it: its authored dimensions and, for
  an exhaust duct, the solver's pressures and temperatures in it. For an
  exhaust or intake duct, an oscilloscope traces the pressure of the clicked
  cell over the engine cycle, recorded at every solver step.
- Resize an exhaust component from the 3-D view: length and diameter
  steppers in the part inspector. Each press is heard a quarter of a second
  later, and Reset goes back to the size the part had when it was selected.
- Exhaust changes no longer restart the engine. A resize from the inspector or
  an Exhaust editor change that keeps the same pipes and junctions reaches the
  running engine: speed, load, temperatures and the ECU carry on, the gas stays
  where it was, and the sound crossfades to the new exhaust in 0.2 s. A change
  of topology (adding or removing a component) still restarts the engine.
- *Exhaust waves* in the 3-D view's settings: *Fine* remeshes the running
  exhaust with 180 mm cells instead of 360 mm, so the gas field and the
  oscilloscope show the waves' real shape, at 13 to 20 % less simulation
  headroom. The engine keeps running through the change.
- *Pressure waves* in the 3-D view's settings: *Hidden* (the new default,
  nothing flickers), *Pulsation strength*, a steady map of how strongly each
  part of the exhaust and intake pulses, or *Live pulses*, the moving waves.
  The Gas flow layer shows the strength unless *Live pulses* is chosen.
- Slow the simulation itself from the 3-D view: *0.25x* and *0.5x* slow the
  physics and the sound together. The picture-only slow motion is now labelled
  as a stroboscope (*Strobe 1:50*, *Strobe 1:250*, *Freeze*).
- Type a size in the part inspector: click the value, type, Enter applies.
  The steppers stay for small steps.
- Save an engine as it is. *Save engine* in the `⋯` menu writes the running
  engine with every setting (exhaust, intake, cylinders, turbo…) and its ECU
  tables; it is then listed under *My engines* in the engine menu, kept in
  `%APPDATA%/EngineLab/engines`. Saving does not restart the engine. *Delete
  saved engine…* moves it to the recycle bin.
- Switching to another engine no longer loses your edits: an edited engine
  keeps them for the session, and the engine menu marks it. A dot on the
  engine picker shows that the running engine differs from the one loaded;
  *Return* reloads it as it was.
- *Export engine* in JSON writes the ECU tables beside the engine
  (`<name>.ecu.json`), and *Import engine* reads them back. An exported engine
  keeps its voicing.
- Injector and turbo settings edited in the JSON editor no longer restart the
  engine: it takes them while running, and its turbo sound follows. A change
  of injection mode, of turbo type or of blade count still restarts it.
- The intake's sizes too: runner length and diameter, plenum volume and
  throttle bore are taken while running, sound included. Adding or removing
  a path, a throttle or the airbox still restarts.
- Change the bore and stroke while the engine runs: click a cylinder liner or
  a piston in the 3-D view. The size glides over 3 s by default, or each crank
  throw takes it at its next gas-exchange TDC. The number of cylinders, the
  layout and the fuel still need a restart.
- Exhaust and intake pipes in the 3-D view are routed like real ones: they no
  longer pass through each other, the engine or the intake, bend no tighter
  than 1.25 diameters and keep their authored length. A collector is wide
  enough for its primaries, which enter it side by side along its axis; long
  primaries meet behind the last cylinder, and the two Y pieces of a 4-2-1 sit
  side by side.
- Turbocharged engines show their turbo in the 3-D view, bolted to the
  collector: the exhaust enters the turbine volute and leaves through the
  exducer, and both wheels turn at the simulated shaft speed. Click it for the
  shaft speed, pressure ratio, wastegate opening, turbine and compressor power,
  and how the solver models it.
- The Merlin's supercharger is drawn between its airbox and its throttle, with
  the charge pipe to the throttle; its impeller turns geared to the crank.
  Click it for the drive ratio, impeller speed, pressure ratio and power.

### Fixed

- A stopped engine's intake no longer churns. From rest, every engine rang
  its intake runners to ±14 to ±111 kPa within a second, which the 3-D view
  showed as an intake flashing as if the engine ran. Behind a stopped crank
  the valves now pass no gas and the runners stand at their plenum's
  pressure. The sound of a running engine is unchanged.
- The EJ25's four primaries now meet in one collector feeding its one turbo,
  then one silencer and outlet. Its two banks used to be separate exhausts,
  one of which bypassed the turbo in the 3-D view. The sound changes (about
  2 dB quieter, darker, mono); the turbo no longer overshoots its boost at
  the top of the range (402 instead of 462 N.m at 6,000 rpm, closer to the
  WRX STI's rating) and spools later at 2,000 rpm (313 instead of 359 N.m).
- The dyno's run name no longer keeps the keyboard: Enter, Escape or a click in
  the 3-D view hands it back to the throttle and the other engine keys.

## [0.2.0] — 2026-10-04

### Added

- Prebuilt Windows release archive (`EngineLab-<version>-win64.zip`) with the
  Visual C++ runtime included, built and published by GitHub Actions on every
  version tag.
- Physical exhaust afterfire with discrete fuel packets, a thermochemical
  ignition delay (engine schema 8), a > 4 kHz crack layer driven by the reaction,
  a mixer switch and a warning when the retained fuel is below the lean limit.
- Directional X-pipe (`crossover`, engine schema 9), passive side-branch
  resonators, tapered ducts, homogenised catalyst substrates and packed mufflers.
- Cycle-authoritative dyno measurement, dense validated ramps and persistent
  dyno sessions.
- Afterfire harness, geometry sensitivity harness and blind A/B clip renderer
  with per-condition reference windows.

### Changed

- The whole project is now in English: user interface, diagnostics, tool
  output and documentation.
- Idle and starting were reworked: port-injection pulse sized
  once per cycle, fuel pushed back past the throttle kept in the airbox, start
  fuel reserve limited to cranking.
- AFR, dyno metrology, intake continuity and DFCO recovery fixes.
- Turbo shaft energy is conserved; output-chain diagnostics report honestly.
- Repository reorganised around a written [vision](VISION.md); the dated
  measurement history moved out of the tree to the `archive/docs-2026-09` tag.

### Fixed

- Stalls on a standing start caused by flooding.
- Port-injection ratchet at idle.
- Labels with a middle dot (dyno axes, ECU tuner status, gauges) showed `Â·`.

## [0.1.0] — 2026-08-06

First public pre-release.
