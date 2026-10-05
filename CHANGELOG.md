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
- The exhaust shows the gas solver's pressure waves at the crank angle on
  screen (in slow motion, the latest cycle at that angle): violet below each
  cell's running mean, grey at it, orange to pale yellow above, with a legend
  giving the scale. Exhaust walls glow red above 798 K, and an afterfire lights a
  flame at the outlets.
- Click a part in the 3-D view to inspect it: its authored dimensions and, for
  an exhaust duct, the solver's pressures and temperatures in it.
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
