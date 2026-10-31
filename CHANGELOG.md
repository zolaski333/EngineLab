# Changelog

All notable changes to EngineLab are listed here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/) (pre-1.0: minor versions may break
file formats, always with an in-memory migration).

## [Unreleased]

## [0.7.7] — 2026-10-31

### Added

- **A rotary can be sized by its rotor.** The engine designer shows the
  rotor's radius, eccentricity and width beside the displacement, as bore and
  stroke beside a piston engine's: give either. The displacement is the whole
  engine's as Mazda counts it (a 13B is 1.3 L), so 8 L on 4 rotors makes four
  rotors of 2 L.

### Fixed

- **A designed rotary has one exhaust per rotor.** With the automatic exhaust
  (the designer's default) each face had its own primary, three exhausts a
  rotor, in the simulation as well as on screen. A rotor's faces now open onto
  its one port, as on the catalogue's 13B. Designed rotaries saved before keep
  the exhaust they were saved with.
- **A rotary's exhaust is drawn from its ports.** The exhaust designer shows
  one source a rotor; in the 3-D view each rotor's pipe leaves from its port,
  where it ran through a junction beside the engine and looped back.
- **A rotary no longer crashes the application.** The cycle panel of the 3-D
  view counted a face's 1,080 deg in 180 deg strokes and read past its four
  stroke names; a face's strokes are now 270 deg each.
- **Launch control works.** The ECU was never told the gear or the road
  speed, so it never armed. Set `limits.launch_rpm` in the ECU, stand in gear
  and open the throttle past half: the engine holds that speed until the car
  passes 5 km/h.
- **Saving no longer writes over another engine.** After *Save engine as*, an
  edit from the 3-D view, the exhaust designer or the A/B switch gave the
  engine its old name back, and the next *Save* replaced the engine it was
  saved from. Edits no longer rename the engine (the JSON editor included:
  *Save engine as* names it). An imported file named like an engine of the
  menu now runs as "<name> (imported)".
- **An engine with an H-pipe loads again.** A saved or catalogue engine whose
  exhaust has a balance pipe was refused on load ("crossover port must be 0 or
  1") and dropped from *My engines*. Changing a merge or splitter's type in the
  exhaust designer keeps its balance pipe.
- **The H-pipe is drawn as an H.** In the exhaust designer the balance pipe
  stands between the two pipes it joins instead of pushing one side two
  columns back; in the 3-D view it leaves and meets each pipe across its
  flank, and the pipes keep their lanes (the right one ran down the middle).

## [0.7.6] — 2026-10-30

### Added

- **A rotary's intake ports can open in stages.** The switched cam profile
  (*Profile switch*, `switch_rpm`) now works on a rotary, as a Renesis opens
  its auxiliary ports near 6,300 rpm. It switched only on a piston engine
  before. The catalogue 13B is unchanged.

## [0.7.5] — 2026-10-30

### Fixed

- **A dyno start could still cancel itself on Linux.** 0.7.4 closed the
  race on Windows; one start in a few thousand still ended at once on Linux.

## [0.7.4] — 2026-10-30

### Changed

- **The intake can be turned up further.** Its level in the side panel, the
  Audio workshop and a voicing file now goes to 4 (about +13 dB over the
  default), where it stopped at 2: a stock airbox sits well under the
  exhaust, and listeners who want the induction roar can now hear it. The
  default mix is unchanged.

### Fixed

- **Starting the dyno could cancel the run at once.** About one start in
  a thousand ended immediately with no point, as if it had been stopped.

## [0.7.3] — 2026-10-30

### Changed

- **A diesel's premixed burn follows its ignition delay.** It was a fixed
  share of the fuel (8 % on the TDI); it is now the fuel that has mixed with
  the air by the time it autoignites, so a long delay (idle, a cold engine,
  low cetane) burns harder at first. The TDI's idle clatters 1-4 dB louder;
  full load barely moves (torque within 0.15 %). A file can keep the fixed
  share with `compression_ignition_mixed_premix: false`.

### Fixed

- **The dyno holds a light engine steady.** Its brake swung light shafts
  (Hayabusa, CP3, CP4, the 13B) by 15-61 rpm about a held point and could
  stall a CP2 it caught at low speed; every catalogue point now holds within
  9 rpm, and torque moves within 1.3 %.

## [0.7.2] — 2026-10-30

### Fixed

- **A very big designed turbo engine reaches its boost.** The designer gave
  every turbocharger a 130,000 rpm design speed, whatever its wheel: a
  162 mm marine wheel ran 1,100 m/s and never finished spooling (1.34 of
  3.0 bar on a 193 L six). The design speed now comes from the tip speed the
  boost needs, so the six reaches 2.96 bar in 3-4 s.
- **A designed diesel's injector is sized on its injection window**, not on
  the whole cycle as before: the marine six ran at lambda 3.0 and a quarter
  of its torque (10.7 kN.m, now 38.9 kN.m). The catalogue's diesels keep
  their own injectors.
- **The rev limiter's bands scale with a slow engine's limit.** On a 750 rpm
  limit the fixed 220 rpm band cut the fuel from 530 rpm. Engines limited
  above 2,450 rpm (the whole catalogue) are unchanged.
- **A big designed diesel gets a crank of its size** (inertia scaled by the
  cylinder's size), so it no longer hunts under the dyno's brake.
- The dyno instruments hold an engine whose idle is below 550 rpm: their
  starter released at 550 rpm whatever the engine.

### Changed

- **The intake's airbox and plenum are volumes with a length.** Above its
  first mode an airbox behaves as an expansion chamber, and it was modelled
  as a point volume that swallowed 16 to 35 dB of the intake between 1.6 and
  4.5 kHz. It now loses what a chamber of its size does (1 to 9 dB): the
  induction's upper notes come through an airbox. The master moves by under
  1 dB on most catalogue engines (the I5 and the TDI the most): with a stock
  airbox the intake stays well under the exhaust, as on a real car.

### Added

- `IntakeTransferHarness`: the intake's acoustic transfer, frequency by
  frequency, with and without its airbox and filter.
- `TurboDownstreamAuthorityHarness --spool <rpm>`: the turbo's spool second
  by second at a held speed.

## [0.7.1] — 2026-10-30

The application is 0.7.0's. The release builds of 0.6.0 and 0.7.0 failed one
test under GCC (the designer's size test: its dyno absorber swung the 25 cc
single, not the engine), so neither was published; 0.7.1 is their release.

### Fixed

- The designer's size test holds its engines with a stable absorber, and the
  CI names each failed test in a public annotation.

## [0.7.0] — 2026-10-30

### Added

- **Rotary (Wankel) engines.** `cycle: rotary` runs one to ten rotors, each
  with its generating radius, eccentricity and width. A rotor's three faces
  each run a 1,080-degree cycle of the eccentric shaft: the face's volume
  follows the rotor, its gas pushes the eccentric, its heat leaves through
  the long, thin chamber, and its side ports open and shut as the flank
  sweeps them. A rotor's faces share one intake runner and one exhaust port.
- **A 13B-class twin-rotor in the catalogue**
  (`engines/18_mazda_13b_renesis_like.engine.yaml`), after the RX-8's
  high-power Renesis. It makes 233 N.m at 5,500 rpm and 169 kW at 8,500,
  against Mazda's 211 N.m and 184 kW. Its port timings and sizes are
  estimates, not Mazda's drawings.
- **The engine designer designs rotaries.** *Rotary (Wankel)* takes a rotor
  count and Mazda's displacement (one face a rotor, 1.3 L for a 13B), and
  sizes the rotor, the ports, the two plugs a face, the ignition, the
  redline and the exhaust from it.
- **The 3-D view draws a rotary as one**: each rotor turns at a third of the
  shaft's speed on its eccentric, inside its epitrochoid housing, and each
  face's flame rides its flank. Clicking a rotor shows its size and its
  face's pressure.

## [0.6.0] — 2026-10-29

### Added

- **The exhaust is set in the engine designer, before the engine is built.**
  The designer has three pages: Engine, Intake & fuel, Exhaust. The Exhaust
  page chooses the system (the family's, single, dual, dual with an H-pipe or
  an X-pipe) and the silencer (none, a packed box, a straight-through one),
  and every length and bore along it: primaries, collector, mid pipe, split,
  crossover, balance pipe, silencer body, tailpipe and tip. An empty field
  shows its derived value in grey; a typed one is kept. A factory system's
  published dimensions go straight in, with no node editing afterwards.
- **An H-pipe.** Two pipes joined by a balance pipe at right angles, which
  carries whichever side pulses higher. In a custom exhaust it is a side
  branch, port `2` of a merge or a splitter.
- **The fuel system is a choice, on either cycle.** Port injection, direct
  injection, or a carburettor, on a four-stroke or a two-stroke (a diesel
  injects directly). A direct-injected petrol four injects on its intake
  stroke; on a designed K20 it makes 5 % more torque than port injection, from
  the charge the evaporating fuel cools. A carburetted four-stroke gets a
  car's fixed-venturi carburettor with an accelerator pump and a starter
  circuit, all sized from the engine; it starts, idles, and pulls from idle at
  part throttle.
- **A supercharger is a machine, not a fraction of a target.** A Roots has a
  displacement, a leak and a bypass valve the manifold's vacuum opens, so it
  boosts more as it spins faster and does no work at cruise; a centrifugal one
  reads its wheel's map. The designer offers both (*Centrifugal supercharger*).
- **The intake can be an open trumpet.** The Intake & fuel page chooses an
  airbox, a pod filter (the element on the throttle's mouth, no box) or open
  velocity stacks (a bellmouth, no filter).
- **An airbox on several throttles breathes through all of them.** With no
  snorkel authored, the box radiated through one throttle's bore; it now
  radiates through the bores' combined area. A Hayabusa's intake is 12 dB
  louder, a CP2's 6 dB; engines with one throttle or a snorkel are unchanged.
  A designed car's airbox now has a snorkel and a lip (designer values).
- **Compressor surge, off by default.** `compressor_surge` (or *Surge* in the
  turbocharger inspector) resolves the duct between wheel and throttle as
  Greitzer's model: a throttle shut on boost with no blow-off valve makes the
  air blow back through the wheel in bursts, about 13 per second on a 2JZ. The
  inspector also sets the blow-off valve's area. The reversal is in the
  physics; it is barely audible yet (the turbo's jet noise is tiny at that
  flow).
- **An engine of any size.** The designer and the configuration no longer
  stop at 20 litres and 50 cc: bores and strokes run from 5 to 1,000 mm, and
  every length, volume, mass and inertia scales with them (a 193 L marine six,
  a 71 L 28-cylinder radial and a 25 cc single design and run). The smallest
  engine is 10 cc, where the carburettor and the parts stop scaling. The dyno
  absorbs up to 10 MN.m (it stopped at 10 kN.m), the ECU's limits follow (a
  diesel's fuel to 125 g a cycle, idle from 50 rpm, boost to a pressure ratio
  of 6), and the 3-D view's exhaust editor takes 4-3,000 mm bores and 30 m
  pipes. The 32-cylinder ceiling stays.

- **The idle speed is set from the ECU page.** `idle.target_rpm` ("Idle
  target", 50-3,000 rpm) is the speed the idle governor holds with the
  throttle shut, on every injected engine, petrol or diesel, four- or
  two-stroke. It starts at the engine's `idle_rpm`, so nothing moves until it
  is changed, and it is saved with the engine's tables. Above `idle_rpm` the
  governor opens past its idle valve, as an electronic throttle cracks its
  plate: every catalogue engine holds twice its idle within 2 %. Below it, an
  engine whose closed-throttle leak alone outruns the target idles there.
- **A carburettor's idle is set with its idle stop screw.** A carburettor has
  no idle governor any more: `idle_screw_turns` (0-8 turns in, 1.5 nominal,
  "Idle screw" on the 3-D view's carburettor) holds the slide open and the
  engine idles wherever that air takes it. Turning it in raises the idle.

### Fixed

- **A designed engine reloads as it was designed.** Its closed-throttle leak
  was not saved, so a designed engine reopened from *My engines* ran on the
  0.6 mm2 default instead of its own (a designed 350 cc two-stroke twin,
  1,300 rpm target, idled about 2,360 rpm instead of 1,800).
- **The tachometer's red zone follows the ECU's rev limit.** It started at
  the authored `redline_rpm` whatever the tuner said; once `limits.rev_rpm` is
  moved, the red zone (and the RPM trace's line) starts there.
- **Two turbochargers spool faster than one.** Splitting a turbocharger into
  several (the designer's "Turbochargers" count) shrank the wheels' flow but
  kept the shaft speed and scaled the inertia with the flow alone, so a pair
  spooled exactly as the single did. Each unit is now the single scaled to its
  share of the air at the same tip speed: wheels 1/sqrt(2) the size, sqrt(2)
  faster, inertia as the fifth power of the size. A designed I4's pair reaches
  90 % boost in 0.43 s against the single's 0.52 s, with the same boost and
  torque.

### Changed

- **A diesel injects when its map says.** The ECU's advance is a diesel's
  start of injection (deg BTDC, editable in the ECU tuner), where it used to
  inject from a fixed 20 deg BTDC whatever the map. The ignition delay now
  runs from that injection, at Assanis et al.'s published value, and the burn
  loses heat to the walls as a diesel's does (Woschni's combustion term). The
  TDI's full-load firing peak falls from 231 to 156 bar, its fuel
  consumption from 171 to 191-210 g/kWh (a real one: ~195-210), and its torque
  at 2,000 rpm from 347 to 308 N.m (rated 320). A saved diesel keeps the
  ignition delay scale it was saved with: 3.0 was the old default, 1.0 the
  published delay (about 3 deg later combustion, under 1 % of torque).

## [0.5.0] — 2026-10-28

### Added

- **A Linux x64 release archive** (`EngineLab-<version>-Linux.tar.gz`) beside
  the Windows one; a release is published only when both platforms pass.

- **The spark is calibrated against knock.** `ignition.knock_limit_curve`
  holds the map under the knock-limited spark measured on the dyno at full
  load, scaled with the manifold pressure so part load keeps its map. The
  engine designer measures it for every spark-ignition design when it builds
  it (a few more seconds), and `EngineLabKnockLimitCalibration` measures a
  catalogue engine. The I5 now holds its 2.5 bar without the knock control
  pulling 8-12 degrees behind its map; its torque stays above the maker's.

### Changed

- **The simulation's worker threads follow the processors Windows or Linux
  lets EngineLab use.** Limited to two processors (process affinity), large
  engines ran four times slower than they can (a 2JZ at 0.3 x real time,
  now 1.3).
- **A two-stroke loses Blair's friction, not a four-stroke's.** It carried a
  valvetrain's and plain bearings' friction, 2.8 times what a rolling-bearing
  motorcycle two-stroke loses: the RD350 gains about 5 N.m at 9,000 rpm and
  designed two-strokes 13-17 %. Four-strokes are unchanged.
- The designer no longer lists valve, header or injector rows for a
  two-stroke: the model never read them.
- A two-stroke's idle air no longer stays open through a closed-throttle
  coast-down, and the RD350's idle air is refitted to the new friction (it
  idled at 1,436 rpm instead of 1,300). Its coast from the limiter to idle
  is slow, about 12 s: it has little to brake it.

### Fixed

- **High boost no longer makes the boost and the mixture hunt.** With the
  wastegate set past about 2.7-3.0 x, the boost control slammed the gate open
  and shut some 13 times a second: the boost swung 3.0 to 3.7 and lambda 0.58
  to 0.96 under load, whatever the intake, exhaust or injectors. The controller
  now widens its band with the boost it holds; every catalogue engine is
  unchanged.

## [0.4.2] — 2026-10-25

The tags `v0.4.0` and `v0.4.1` exist but neither was published. 0.4.0's
release build ran the hosted runner's compiler out of heap (C1060), which the
release workflow, unlike CI, did not resume on one build node; 0.4.1's tests
failed on a full V8 dyno sweep that the runner, sharing its processors with a
second test, could not finish in time (the first entry under *Fixed*). 0.4.2
is the first release with everything listed here.

### Highlights

- **Design an engine from nothing**: *New engine…* asks what it is and derives
  every other value, each with the rule behind it, and checks that this
  computer keeps it in real time before building it.
- **Two-strokes behave and sound like two-strokes**: scavenging follows the
  fresh charge's front, a slide carburettor, a power valve, an idle that
  misfires on its diluted charge, and one pipe per thread.
- **Engines knock**: the end gas autoignites, and the knock control holds the
  limit in steps as a production one does.
- **Backfires pop**, switched on in the Audio workshop: an overrun spark cut
  whose unburnt charge the next fired cycle lights in the pipe.
- **Linux x64** builds and passes the tests, beside Windows.
- More edits on the engine itself: compression ratio, port timing, the
  carburettor's jets, the clutch, two exhaust paths joined into one.

### Added

- **A two-stroke can breathe through a cylinder port**: the designer's
  *Intake* choice keeps the crankcase reed (the default) or puts the
  carburettor on a port in the barrel that the piston skirt opens around top
  dead centre, with or without a reed behind it.
- The 3-D view draws a two-stroke as one is built: a round crankcase without a
  sump, the exhaust leaving from its window in the barrel, the transfers in the
  barrel flanks, the carburettor where the intake is, and a power valve whose
  blade drops as it shuts. Click the exhaust port to fit or remove a power
  valve and to set its timing and speeds.
- The carburettor's jets are edited as the hole a Dellorto or Keihin jet is
  stamped with (hundredths of a millimetre), with the Mikuni flow number
  beside it; a new carburettor bore scales the main jet so the mixture holds.
- `EngineLabSceneExport --two-stroke-intake` draws every two-stroke with
  either intake.
- **Two-strokes can have a power valve** (`two_stroke.power_valve_*`): it
  lowers the exhaust port at low speed and lifts as the pipe comes in. The
  designer fits one to sport two-strokes; a designed 500 cc twin gains 20-39 %
  of torque between 3,250 and 5,750 rpm and keeps its top end.
- **Clicking a two-stroke's head or ports edits the port timing** (exhaust
  and transfer opening, with the boost port following the transfers) and the
  port widths; the engine restarts with them.
- **A carburettor is drawn and tuned in the 3-D view**: on a carburetted
  engine the throttle body becomes a carburettor (slide tower, float bowl),
  and clicking it offers its main and pilot jets, needle clip and air screw,
  taken by the running engine.
- **The compression ratio is edited by clicking the engine**: a cylinder
  liner, piston or head offers it beside the bore and stroke, on any engine
  (a two-stroke's included, and engines whose cylinders differ in size), and
  the running engine takes it (at once or over the ramp).
- **The clutch's capacity can be edited** on the gearbox, live: a boosted
  engine no longer has to slip on the designer's estimate.
- **Two exhaust paths can be joined into one**: the exhaust ports or a
  junction offer *Joined*, which gives the other path's cylinders a copy of
  this path's primary into its collector (a boxer six's two banks become a
  6-into-1) and restarts the engine.
- **New engine… designs an engine from nothing**: say what it is (four- or
  two-stroke, petrol or diesel, what it is built for, inline, V, flat, radial
  or W, any number of cylinders, bore and stroke or a displacement, natural,
  turbocharged or supercharged) and every other value follows, each shown with
  the rule behind it. *Advanced* sets the firing order, an uneven crank, a W's
  bank angles or any value by hand. An engine file can hold such a design.
  Before building, the design runs for a few seconds to check that this
  computer keeps it in real time, and asks first if it would not.
- `EngineLabDynoSweepHarness --advance-offset deg` shifts the whole ignition
  curve, to see how far an engine is from its best-torque timing.
- The 3-D view draws engines with more than two banks (a W16, a broad-arrow
  W12) as W engines, bank by bank, instead of as radials.
- `EngineLabSceneExport --catalog-root` exports another catalogue folder.
- `EngineLabRealtimeBudgetHarness --designer-probe` runs the designer's
  real-time check on each engine.
- **EngineLab builds and passes its tests on Linux x64.**
  `cmake --preset linux-gcc-release` (GCC 13 or newer) configures and builds
  the whole project with warnings as errors, and `ctest --preset
  linux-gcc-release` runs all 92 tests green; `linux-clang-release` does the
  same with Clang 18. Every push now compiles and tests both platforms.
  Windows stays the reference, and [docs/platforms.md](docs/platforms.md) says
  why a figure measured on one is not comparable with the other:
  floating-point contraction is pinned off so a Linux result matches MSVC's
  non-contracting `/fp:precise`, an unprivileged Linux run cannot raise the
  physics thread's priority, file names are converted through each platform's
  native encoding, the oversized `EngineRuntime` test fixtures moved off the
  stack because Linux has no link-time stack size, and the one test that
  builds real JUCE components runs under a window manager on `xvfb-run`.
  No Linux archive is published yet, and the GUI has not been reviewed on a
  Linux screen.
- **Two-stroke engines get a carburettor.** The designer fits a slide
  carburettor to every two-stroke it builds, one per cylinder, with jets sized
  from the engine (an RD350-sized design gets a 287 main and a 41 pilot; the
  real bike runs 260-290 and 30-40). The main and pilot circuits each run on
  their own depression, the needle clip and the air screw do what they do on a
  real one, and the fuel enters the crankcase with the air. Engine files carry
  it as `carburettor:` under `intake:` and under each intake path. A
  carburetted two-stroke runs richer than the injector did (lambda 0.74-0.84
  against 0.88-0.94) and makes 4-6 % less peak torque. The catalogue RD350
  keeps its injector.

### Fixed

- **An engine of three cylinders or more no longer crawls on a busy or small
  computer.** Its physics thread waited for its helper threads by spinning
  without ever giving the processor up, so whenever the threads outnumbered
  the free processors each step cost a whole scheduler time slice: a full V8
  dyno sweep took 61 s on two processors instead of 7, and never finished on
  one. It now hands the processor over (41 s on two); on the reference
  machine the real-time factor is unchanged within its noise.
- A backfire's pressure wave is no longer partly deleted in the exhaust:
  strong pulses were read too early by the pipes' wave-steepening and lost
  4-9 dB depending on where they burned, and a 100 kPa safety bound clipped
  every lit slug by about 3 dB. Without backfires the sound is unchanged, to
  the bit.
- `EngineLabAfterfireHarness` names an override outside the engine's limits
  (a `--late-spark` past +10 deg, a `--spark-cut` above 0.9) and exits,
  instead of crashing.
- **A two-stroke idles like one**: its idle charge is mostly burned gas left
  from the last cycle, and it now misfires on it, lighting about one
  revolution in three as a real one does, instead of firing every revolution
  like a four-stroke. Four-stroke idles and the default sound are unchanged.
- `EngineLabRealtimeBudgetHarness --free-run` measures designed two-strokes
  again: it cranked them at the dyno's full throttle, where they never
  caught, and every one read "hold not stable".
- **A two-stroke's intake is drawn as one is built**: per cylinder the
  transfer duct inside the barrel, then out of it a reed block, a short
  manifold and the carburettor a few centimetres from the reeds, and a rubber
  boot into one airbox all the carburettors share (above a V's valley). The
  carburettors were hidden inside per-crankcase boxes and could not be
  clicked; the crankcase is now the block's only.
- **A V motorcycle engine's gearbox no longer goes through the rear bank**
  (a designed V-twin or V4, and the catalogue Big Twin's, 16.6 mm inside a
  barrel): the case turns down behind the crankshaft until it clears the
  barrels.
- **The designer's real-time check failed every carburetted two-stroke**
  ("did not hold 5000 rpm"): it cranked the engine at full throttle, where a
  carburettor cannot start it. It now cranks with the throttle shut until the
  engine runs.
- The designer names the engine from what it is (*Two-stroke Inline Twin*)
  until a name is typed, instead of keeping *Inline Seven*, and its table's
  columns are headed *Quantity* and *Value*.
- A carburetted engine no longer draws, or offers to edit, injectors it does
  not have.
- The diagnostics say what a slow machine report needs first: the audio
  device's rate and the rate the sound is synthesised at, the audio callback's
  load (over 100 % it drops samples), and whether the physics thread is held to
  the performance cores and opted out of power throttling.
- An audio device set to 96 or 192 kHz (common on gaming headsets and external
  DACs) no longer costs two or four times the sound of a 48 kHz one: the engine
  is synthesised at 48 kHz, the rate it is voiced at, and upsampled to the
  device. At 192 kHz the sound took 95 % of the audio thread's time and dropped
  samples on a 2JZ; it now takes 22 %, as at 48 kHz. A 44.1 or 48 kHz device
  is unchanged.
- On a processor with performance and efficiency cores (Intel 12th generation
  and later), the physics thread and its workers are held to the performance
  cores, and the process opts out of the power throttling Windows applies to
  an app it judges in the background, which costs 41 % of the real-time
  factor on the reference machine. Not yet measured on a hybrid processor.
- A designed V or W engine shares one crank pin between the cylinders of a
  row, as real ones do; each had its own pin. A 90 deg V-twin now fires
  270-450 and a 45 deg one 315-405. Where a shared pin cannot fire six
  cylinders or more evenly (a 60 or 90 deg V6), the pins are split by the
  least angle that does. *Advanced* > *Crank pins* (`crank_pins` in a design
  file) forces shared pins or one per cylinder.
- The 3-D view drew two neighbouring crank throws pointing the same way as
  one (a V12's middle pair, a boxer four's).
- Small engines (50-250 cc) can be designed: the values the rules gave
  under the simulator's limits (flywheel, final drive, plenum, piston,
  throttle) are held at the limit, each saying so. A refused engine now names
  the value at fault and its range, instead of one generic line. A small
  throttle leaks less when closed, so small engines idle at their speed.
- The designer's real-time check no longer fails single-cylinder engines,
  whose speed swings within each cycle, nor engines the dyno cannot pull up to
  the measuring speed (it measures where they settle). When the check cannot
  finish, the box says so instead of "Not real time".
- A designed two-stroke's throttle (now its carburettor) is sized from the
  cylinder it feeds instead of from the whole engine, which gave identical
  125 cc cylinders a 16.6 mm bore on a single and 33.2 mm on a four. Singles gain
  18-29 % of peak torque; triples, fours and V4s lose the 6-9 % their oversized
  bores gave them; twins are unchanged.
- **Engines knock.** A boost and an advance that would destroy a real engine
  now knock: the end gas the flame has not reached is followed as its own
  zone, compressed from the seal, against an autoignition delay fitted on
  reference fuels in a knocking engine (Douaud-Eyzat), and the knock's
  intensity is the pressure that gas adds. The delay's prefactor is
  calibrated so that the catalogue's own spark maps sit at their knock limit,
  as a production map does: 10 deg of advance over a map knocks, and a 2JZ
  run that way at full load with knock control off knocks heavily. The knock
  control now works as a production one: it pulls the spark back in steps of
  about a degree while the engine knocks and gives it back at 1 deg/s, so an
  engine settles at its knock limit instead of sitting in light knock. The
  Audi I5 gets the intercooler its real engine has.
- **Every turbocharger is drawn.** With two or more units the 3-D view drew
  only the first; now each unit an exhaust path drives sits on that path's
  collector (a flat six's two banks get one each), and clicking one opens its
  own inspector, titled with its number and its path.
- A two-stroke saved as JSON keeps its scavenging loop length (it came back
  at the default).
- A two-stroke twin no longer fails to hold 3,500, 7,000 and 7,500 rpm on the
  dyno: the brake answered the crankshaft speed sampled at the end of each
  240 Hz frame, where a firing rate near 240 Hz (or 120) aliases into a slow
  wander. The dyno now holds the speed averaged over the frame.
- A designed carburetted two-stroke idles: its pilot jet was sized for about
  eight times the air a two-stroke idles on, so twins and triples flooded and
  stalled. The pilot is now sized from the air the idle path passes and
  follows it as the idle air opens, and each carburettor's idle bypass goes
  with the cylinder it feeds. Designed 1-4 cylinder two-strokes idle at their
  1,300 rpm; an RD350-sized design keeps a 35 pilot.
- A two-stroke's idle no longer hunts: the idle governor runs at half the
  gain on two-strokes, whose idle air passes the crankcase first. The RD350
  held 1,175-1,398 rpm with a 5 s period; now 1,284-1,325.
- Boost no longer burns the charge faster: the flame's turbulence follows the
  piston, not the intake pressure, so a boosted cylinder burns like a full
  unboosted one.

### Changed

- The build compiles JUCE once instead of once per executable: a clean
  Linux build takes 6 minutes instead of 15 on four cores.
- Engines with eight cylinders or more simulate their cylinders in parallel,
  with exactly the same result: 22-33 % more real-time headroom (a designed
  6 L V12 at 7,500 rpm 0.71 -> 0.93, the LS3 1.17 -> 1.43, the Merlin
  1.40 -> 1.86 on a 6-core desktop). `EngineLabRealtimeBudgetHarness
  --inline-cylinders` (and `--serial-publish`) keep the old serial passes for
  comparison.
- Two-stroke scavenging follows the fresh charge's front as it crosses the
  cylinder, integrated through the transfer window, instead of a fixed
  short-circuit share. A two-stroke held at one speed no longer hunts between
  4,500 and 6,250 rpm. Designed two-strokes' torque curves are smoother and
  2-14 % stronger; the catalogue RD350's curve now rises steadily to its peak
  instead of dipping at 5,750 rpm, at 12 % less mean torque, and it sounds
  different.
- Two-strokes simulate faster: each cylinder's expansion chamber is solved on
  its own thread, and a twin now uses a second thread. On a 6-core desktop,
  free-running, a designed twin goes from 1.53 to about 2.1 times real time,
  a triple from 1.30 to about 1.65, a V4 from 0.97 (slow motion) to about
  1.25. Torque moves by less than 1 %. Four-strokes are unchanged, to the bit.
  `EngineLabDynoSweepHarness` and `EngineLabRealtimeBudgetHarness
  --exhaust-whole` keep a two-stroke's exhaust as one network for comparison.
- **Backfires are switched on in the Audio workshop only**, for the engine
  running, and every engine loads with them off: a catalogue engine, a saved
  one, a script or a designed engine never turns on the overrun afterfire or
  the wet limiter, and switching them is not an edit to save. The *Audio
  Physics Lab 689 Twin* no longer pops or fires a wet limiter until you switch
  them on. Editing the running engine (a bigger bore, a designer, the JSON
  editor, a script reload) keeps them as the workshop left them; only another
  engine turns them off.
- **The *AFTERFIRE* switch pops like a real overrun**: instead of chopping
  retained fuel at 2 Hz, the ECU cuts the spark of most overrun cycles (70 %
  by default, fuel kept) and fires the others at 10 degrees, and the next
  fired cycle's exhaust lights the unburnt charge waiting in the pipe. On the
  *Audio Physics Lab 689 Twin* at 3,400 rpm that gives about 8 pops a second,
  16.6 dB over the engine, against 15.8 for a recorded bike. The physics group
  shows the spark angle, the pocket mixing time and the spark cut instead of
  the overrun fuel and the pop rate, duty and jitter. High revs (above about
  6,000 rpm) still pop rarely.

### Removed

- The old *New engine…* that changed the size of an existing engine: the
  designer replaces it.
- The `continuous_anti_lag` and `discrete_afterfire` afterfire strategies, with
  `overrun_fuel_fraction` and the `overrun_pulse_*` chopper: a file naming them
  loads with a clean fuel cut, and its old keys are ignored. A script that sets
  `exhaust_afterfire.enabled` or `exhaust_afterfire.overrun_fuel_fraction` is
  now rejected (unknown property). `EngineLabAfterfireHarness` loses
  `--force-demo`, `--fuel-fraction` and the `--pulse-*` flags.

## [0.3.1] — 2026-10-18

The tag `v0.3.0` exists but 0.3.0 was never published: its release build
failed the tests on the intermittent click fixed below (the Merlin's). 0.3.1
is the first release with everything listed here.

### Highlights

- **A new main window and a GPU 3-D engine view** in X-ray: pistons, valves,
  gearbox, turbo and routed pipes placed by the simulator's own kinematics,
  with the gas solver's pressure waves drawn on the exhaust and intake, and
  a microphone that follows the camera.
- **Live editing**: click a part to change it while the engine runs (exhaust
  and intake sizes, bore, stroke, rods, cams, flywheel, injectors, turbo,
  gearbox, the air), an **A/B** button to compare two versions by ear, and
  *Save engine* to keep the result.
- **Two-stroke engines**, with a Yamaha RD350LC-like twin in the catalogue.
- **Forced induction**: one to four turbochargers on a compressor map, an
  intercooler, an ECU boost table, and a turbo that can be fitted to any
  engine.
- **Driving**: a Drive section, automatic, dual-clutch, sequential and CVT
  gearboxes, cruise control, launch control, a progressive brake, display
  units and a datalogger.
- **New engine…** builds an engine from a running one; the ECU tuner edits
  blocks of cells with undo.

### Added

- Two-stroke engines (`cycle: two_stroke`, with a `two_stroke:` block for the
  port timings and widths and the crankcase compression): piston ports,
  crankcase pumping through a reed, one firing per revolution, fuelled as a
  carburettor is (no overrun fuel cut); the 3-D view and the inspector show
  its ports instead of valves and cams, and the engine language sets the ports
  (`two_stroke.*` keys). New catalogue
  engine: a Yamaha RD350LC-like twin with expansion chambers. Two-strokes
  run a finer real-time exhaust mesh (100 mm cells) so the chambers can
  work, reconstructed across their short cones; its load is its delivery
  ratio, not its crankcase pressure, and its exhaust is coupled to the
  cylinder every step from bottom dead centre to the transfers' closing.
  Known gap: it makes 29-33 N.m at 8,000 rpm (maker 37-40) and 34 kW at
  9,000 (maker 34.5 kW at 8,500); 25 mm cells, not real time, make 36.
- Launch control (`limits.launch_rpm` in the ECU, off by default): standing
  in gear with the throttle past half, the limiter holds the engine at the
  launch speed until the vehicle rolls.
- A datalogger (`Log` in the top bar, key `9`) records the main channels and
  the active limits to a CSV file while you drive or pull.
- New engine… (menu): build an engine from any engine that runs, with
  another bore, stroke, compression, redline or aspiration (natural or 1-4
  turbochargers). Injectors, throttles, plenums and clutch follow the new
  size; the result is saved under My engines.
- The turbo inspector (3-D view) sets the intercooler, the number of
  turbochargers and which one each exhaust path drives, and shows each
  turbocharger's speed, air share and map position.
- The ECU tuner edits blocks of cells: select with click, shift-click and
  ctrl-click, then step, scale by 5 %, smooth or interpolate them, with undo
  and redo. Its table list shows readable names.
- The Engine panel says what holds the engine back now: rev limiter, launch
  control, knock retard, hot coolant, boost at target, smoke limit...
- Turbo engines can have several turbochargers (`turbocharger_count`, up
  to 4), each exhaust path driving the one it names. A turbo fed by fewer
  cylinders spins slower and makes less of the air, as on a real engine.
  The compressor now follows a map: its boost falls as the flow rises, so
  spool at low engine speed changes a little (within 2.5 % at 2,000 rpm).
- New `intercooler_effectiveness` (default none). Catalogue engines do
  not use one: with it they make about 20-30 % more torque than the real
  engines, so treat its effect as relative.
- Peak firing pressure is tracked at every physics step and shown against
  the engine's structural limit (side panel); above it the engine is damaged
  and a critical diagnostic says so. The limit is `peak_pressure_limit_bar`,
  by default 120 bar naturally aspirated, 140 boosted, 200 for a diesel.
- A diesel's fuel pressure now changes its combustion: higher rail pressure
  mixes faster and burns harder. Its start of injection can be moved live
  from the injector in the 3-D view.
- Knock control is an ECU setting: `knock.retard_deg_per_intensity`, the
  spark retard per unit of knock (12 deg as before; 0 turns it off). Turned
  off, the 2JZ at full load runs 4 deg more advance and 2 % more torque at a
  higher knock. Knock still does not punish an over-advanced map as a real
  engine would (journal).
- CVT gearboxes: a car's belt CVT that creeps like a dual clutch, and a
  scooter's with a centrifugal clutch that idles free. The variator holds the
  engine at a speed set by the pedal (low when cruising, near peak power flat
  out). Pick `car_cvt` or `scooter_cvt`, or switch any gearbox to CVT from its
  inspector; the gear shows D.
- Display units: speed, distance, fuel consumption (L/100 km, km/L, MPG US
  or UK), fuel volume, pressure (kPa, bar, psi, absolute or gauge), torque,
  power, temperature, length, displacement, mass and air flow, chosen one by
  one or from a preset (Metric, Metric bar/PS, US, UK) in the `⋯` menu, under
  **Units**. Every readout, the dyno, the inspector and its edit fields
  follow them; the choice is kept in `units.json`.
- A **Drive** section in the left column: speedometer, gear, instant
  consumption (litres per hour when standing), a trip computer with its own
  reset, and buttons for gear down / up, brake, clutch and cruise control,
  each showing its key.
- An automatic gearbox for any engine (`8`, or the **Auto** button): it
  changes up early with a light foot and near the limiter flat out, kicks
  down when you floor it at low revs, and comes back to first when you stop.
  Changing gear yourself takes it back to manual; selecting first from
  neutral does not. `transmission.automatic_upshift_rpm` and
  `automatic_downshift_rpm` now default to 0, which derives them from the
  engine (before: fixed 6,400 and 1,650 rpm for every engine).
- Cruise control (right arrow, or the button): holds the road speed from
  30 km/h; the pedal overtakes it; the brake, the clutch, neutral or falling
  far under the set speed cancel it.
- The brake is progressive: holding the key (or the button) builds the
  pressure over about a second (three with `Space` held) and releasing it
  lets go at once.
- The part inspector edits any number of fields, shows each in your units,
  and says whether a change applies live or restarts the engine. The
  gearbox is editable from it: clutch and hand-over times, ignition cut,
  final drive and every gear ratio.
- Gearboxes can now shift the way real ones do (`transmission.shift_style`,
  changeable live in the engine file). `manual` is the driver's lift and
  clutch, as before. `sequential` is a motorcycle or race dog box: an upshift
  through a quickshifter cuts the ignition for a set time
  (`quickshifter_cut_s`) with the clutch left in, and an auto-blipper revs
  the engine to the lower gear's speed on a downshift. `dual_clutch` hands the
  drive from one clutch to the other with no torque gap. The CP2 engines now
  shift up through a quickshifter (65 ms cut): the drive comes back in about
  20 ms instead of 55 ms through the clutch. Every other engine is unchanged.
- The 3-D view draws the gearbox: a car's in line behind the flywheel, a
  motorcycle's behind the crank with its clutch pack and sprocket. Each
  gear pair turns at its own ratio, and a blue collar slides against the
  engaged gear. Click it to see the gear, the ratio, the clutch slip and the
  road speed, to change the shift style (manual, quickshifter, quickshifter
  and blipper, dual clutch) and to set the shift time and the
  quickshifter's ignition cut; the running engine takes them.
- A turbo can be fitted to any engine: click an exhaust part (or the turbo)
  and switch *No turbo* to *Turbo*. It is sized for the engine (its
  displacement, redline and compression: about 0.5 bar on a high-compression
  engine, more on a low one), sits on the collector, and the engine
  restarts. A CP2 goes from 54 to 80 kW. The ignition map stays the engine's,
  so it may knock lightly at boost. On engines without a collector (LS3,
  Big Twin) the turbo works but is not drawn, and the Big Twin overboosts.
- An **A/B** button in the top bar (key 7) keeps two versions of the
  running engine. Switch to B, change the cams, the exhaust or the air, then
  switch back and forth to hear the difference at once. Each version keeps
  its own edits and its own ECU tables. Parts that cannot change live
  (the firing order, for example) restart the engine. Choosing another
  engine or reloading its file starts again from A.
- Gearbox and vehicle edits in the JSON editor (ratios, final drive, mass,
  tyres, clutch) are taken while the engine runs, without a restart, as long
  as the number of gears stays.
- Change the air while the engine runs: click the cylinder block and set
  the altitude and the temperature. At 2,000 m and 35 degC the CP2 loses
  27 % of its torque, the turbocharged 2JZ 20 %: its wastegate holds the
  same boost above the outside pressure, as a real one does.
- Change the camshafts while the engine runs: click an intake or exhaust
  valve for its cam's duration and lift, or the cylinder heads for the cam
  timing (intake and exhaust centrelines, with the overlap shown). The
  valves glide to the new cams over 3 s, or take them at the next cycle.
  Switching a variable profile (VTEC) on or off still restarts the engine.
- Change the flywheel while the engine runs: click the crankshaft in the
  3-D view and set its inertia (crank and flywheel). A lighter one revs up
  and down faster. Ignition limiter and afterfire settings edited in the
  JSON editor are also taken live now, without a restart.
- Change the connecting rods' length while the engine runs: click a rod in
  the 3-D view. The compression ratio stays (the deck moves with the rod),
  and the change glides or lands at each throw's gas-exchange TDC, like the
  bore and stroke. An engine with articulated rods (the radial) keeps its
  rods.
- Individual throttle bodies: `individual_throttles: true` in an engine's
  intake puts one plate in each runner instead of one ahead of the plenum,
  at `throttle_distance_from_valve_mm`. The plate is a real restriction in
  the 1-D runner: choked when nearly shut, free when wide open. No catalogue
  engine uses it yet, and the intake sound and 3-D view still show a single
  throttle.
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
- *Microphone on the camera* in the 3-D view's settings (on by default):
  the sound is heard from where the camera is, and follows it smoothly as it
  moves. The exhaust is heard from its drawn tailpipe, the intake from its
  drawn mouth and the turbo from where it is drawn, no longer from the middle
  of the engine; switched off, the usual listener comes back.
- In the 3-D view, click an intake runner, the plenum or the throttle to
  resize it; the intake ports (or the combustion, for direct injection) for
  the injectors' flow and fuel pressure; the turbo for its wastegate and
  turbine; the supercharger for its pressure and drive ratios. Each edit is
  taken by the running engine.
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
- The injectors are drawn in the 3-D view, one per cylinder: in the runner
  for port injection, in the head for direct injection. A click shows their
  window, duty cycle, fuel per cycle and wall film, and edits their flow and
  fuel pressure. The side panel's Telemetry tab gains a *Fuel injection*
  section with a *Show in 3-D* button.
- Ctrl+S saves the engine and Ctrl+Shift+S saves it under another name; S
  with Ctrl held no longer cranks the starter.
- Quitting (the window's close button or Escape) with edits not saved asks
  first and names the engines that would lose them.

### Changed

- The boost of a turbo engine now lives in the ECU: a boost target table by
  engine speed (`boost.target_gauge_kpa`), flat at the engine's wastegate
  setting by default, so a tuner can taper or raise boost through the rev
  range. The 3-D view's wastegate setting moves the whole table.
- Exhaust collectors no longer sound muffled. A collector without an
  authored volume behaved as a 2-litre chamber, which cut 0.5-5 kHz by up
  to 19 dB on most engines; it now holds only what its pipe sweeps. Engines
  that author a collector box (MT-09, MT-10) are unchanged. Power moves by
  at most 2.4 %. Resizing a collector box in the 3-D view now scales its
  volume.
- Motorcycles accelerate like motorcycles: their wheels, chain and clutch
  had a car's inertia (the equivalent of 230 kg more bike in first). The
  MT-07 now does 0-100 km/h in 3.9 s (published 3.85 s, before 5.9 s). The
  most powerful bikes are now quicker than real ones, which wheelie: there
  is no wheelie limit yet.
- The dual-clutch gearbox no longer uses the clutch key: it launches on its
  own clutch, creeps at idle (stops on the brake) and never stalls. It also
  refuses a downshift that would over-rev the engine, as a real one does;
  manual and sequential gearboxes still let you do it.
- A dual-clutch upshift under load now retards the spark (up to 20 deg,
  `transmission.dual_clutch_upshift_retard_deg`) while the clutch pulls the
  engine down, as production dual clutches do: the push after the shift is
  shorter. Its effect on the sound has not been listened to yet.
- The exhaust now radiates into the surrounding air rather than into its own
  hot gas, as a real tailpipe does. Hot engines get louder relative to cool
  ones (+2.5 to +11.7 dB near redline, the CP2 least), and the exhaust stands
  out more against the jet and intake noise. The monitor's full scale moves
  from 134 to 140 dB SPL so the loudest engines stay clear of the safety
  limiter; the CP2 keeps roughly its old level.
- The engine runs warm from the start: coolant and oil are held at 90 and
  100 degC, as on a dyno cell, so every run and every A/B comparison is made
  at one temperature (before, every engine started at 22 degC and warmed up
  over many minutes). Click the cylinder block to switch to realistic
  temperatures, which start cold and warm up. The sound barely moves (within
  0.6 dB across the catalogue).
- Realistic temperatures now behave like a real engine: it warms up in
  minutes rather than never (a CP2 idling in still air reaches 80 degC in
  about 18 min; on the dyno it is warm after one or two passes), the
  thermostat holds it near 90 degC, a fan comes on at 100 degC, and riding
  cools it. On the dyno a cell fan blows on it, so repeated passes level off
  instead of climbing. The block's inspector shows when the fan is on.
  Air-cooled engines (Aircooled, Big Twin, Radial) have no radiator fan; the
  Aircooled's blower follows the engine speed, and air-cooled engines hold
  their oil near 90 degC through an oil thermostat (the Big Twin no longer
  stays near 40 degC on the dyno).
- The turbocharged cars' exhausts have their real length (about 3.4 m from
  port to tip, like the other cars): a 2.2 m mid pipe behind the turbine.
  Spool and power stay within a few per cent. The sound changes: 2JZ about
  6 dB louder and darker, I5 3 dB louder, EJ25 2 dB and TDI 4 dB quieter and
  brighter.
- The mechanical sound layer is off by default on every engine; its fader
  still turns it back on. The mix gets quieter where it was loud (I5 and 2JZ
  about -5.5 dB, Hayabusa -2.8 dB) and less bright. The starter keeps its own
  fixed level, so it is heard on every engine while cranking.
- The turbocharger's turbine sits in the exhaust gas network, on the
  collector, instead of narrowing the tailpipes: every pulse crosses it, it
  takes its work from the gas, and the downpipe carries cooler gas at the
  turbine outlet pressure. The old model overstated the turbine's power, so
  the catalogue turbine areas are re-fitted to the rated curves (2JZ 600,
  EJ25 500, I5 450, TDI 280 mm2). The turbo engines sound different: the
  EJ25 2.7 dB quieter and darker, the TDI 3 dB louder, the I5 1.3 dB louder.
- The exhaust and the intake manifold spread their pipes over the
  simulation's worker threads, with bit-identical results. At 95 % of the
  limiter with audio, every catalogue engine now runs above real time: the
  LS3 at 1.07 (0.95 before, 1.05 with the 3-D view open) and the Merlin at
  1.40.
- Intake valves are a characteristic boundary on the runner: the trapped
  charge no longer depends on the solver's internal time step (it lost up to
  11 % before). The cylinders exchange with the intake network every 330 us,
  with the network running ahead of them, instead of on every sub-step. The
  LS3 and the Merlin are back above real time (1.08 and 1.15 at 5,000 /
  3,040 rpm, from 0.78 and 0.80). Torque curves and the sound move: most
  engines by less than 4 % in level, the 2JZ -15 %, the EJ25 -11 % and the
  Hayabusa +19 %. The Merlin no longer drives its limiter at steady load
  (pre-limiter peak 1.51 -> 0.66 of full scale).
- Exhaust junctions take the same explicit step rule as the pipe cells
  around them. A compact X crossover no longer halves the step of the whole
  exhaust: the LS3 runs 13 % faster near its limiter, with the same torque
  and sound. The Exhaust editor's cost estimate uses the same rule.
- Catalogue exhausts run their real length from port to tip: 3.0-3.5 m
  under the naturally aspirated front-engined cars, 1.6 m behind the
  rear-engined flat-six and 1.2-1.5 m on the motorcycles. They used to end
  0.9-1.3 m from the port. The turbocharged cars keep theirs: the solver's
  turbine then sat at the outlet, and a longer pipe ahead of it slowed the
  spool.
  The manufacturer torque and power points move by less than 5 %. Exhaust
  presets and paths gain `midpipe_length_mm` and `midpipe_diameter_mm`.
- The two pipes leaving an X crossover turn away from each other at once in
  the 3-D view instead of running through each other.
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

### Removed

- The *Manual load* slider: the dyno and the vehicle load the engine.

### Fixed

- No more full-scale clicks in the exhaust when the physics thread falls
  briefly behind on a busy machine (heard intermittently on the Merlin): a
  late frame's sound is now played across the gap instead of being skipped.
- A stalled engine left in gear no longer rolls the car backwards: its
  friction was passed through the clutch as a drive torque.
- A coasting car now comes to rest: below 1 cm/s it used to creep on for ever.
- A two-stroke engine file that leaves out the crank offsets now spaces its
  cylinders over 360 degrees. Before, a twin got 0 and 360 degrees: both
  cylinders fired together.
- The "injectors saturated" limit no longer lights up on a healthy engine at
  full load: it is now judged on the last completed cycle, not on the part of
  the window elapsed so far.
- The dyno now sweeps up to the rev limiter you set in the ECU tuner. Before,
  it always stopped under the engine's original limiter, whatever the tune.
- Over-rev damage now starts just past the active rev limiter, not at the
  redline: an engine on its limiter takes no damage, and a downshift that
  throws it past the limiter does. The redline is only a mark on the
  tachometer.
- Holding the clutch while changing gear now makes the shift yours: the gear
  changes at once and the auto-blipper, quickshifter and torque cut stay out
  of it. Before, the blipper opened the throttle with the clutch pulled.
- A throttle blip no longer risks stalling the engine on its way back to
  idle. Warm, the 2JZ fell to about 520 rpm (target 760) and the Audi I5,
  Big Twin and Aircooled nearly as low; blipped in the first seconds after a
  start, the LS3, the Merlin and the Radial stalled. Every catalogue engine
  now stays above about 0.8 x its idle speed. The idle also comes back down
  after a blip: the EJ25, CP4 and TDI could hang at 1.4-1.5 x idle.
- Dyno passes no longer cost power for good. Every engine knocks lightly at
  full throttle, and that light knock was counted as damage: each pass took
  0.3-0.5 % of power that never came back. Only severe knock damages the
  engine now. Known limit: warm, the EJ25, the I5 and the Big Twin can stall
  on a full-throttle launch straight from idle (about 1 launch in 5 for the
  EJ25); give them some revs before letting the clutch out.
- The TDI no longer surges at full throttle from 2,500 rpm (torque swinging
  by up to 32 % four times a second, which looked like misfires on the
  dyno). Its boost controller oscillated once the turbine sat on the
  collector; every turbo now uses a proportional-integral boost controller
  and holds its wastegate pressure exactly.
- The CP4 no longer drops below real time on the limiter or when the throttle
  is shut after it (0.73-0.80 on lift-off before, 18 ms late at worst). During
  valve overlap, the intake side drained the chamber at the previous window's
  rate of hot exhaust gas while it was refilling with cold charge, down to
  0 K, and the solver retried its step thousands of times per second. Every
  catalogue engine now stays at or above 0.99 through a limiter blip. Torque
  curves move by 1.1 % at most, the sound by less than 3 % in level.
- No more isolated clicks in the exhaust sound of the Merlin at full load and
  of the Radial at its catalogue point. The high-band valve source switched on
  for a few milliseconds whenever one crank-angle sub-step was shorter than
  the others, then off again; it now follows the slowest recent sub-step rate.
  The level of every catalogue engine is unchanged.
- Torque curves no longer swing from one 50 rpm step to the next because of
  the solver. The intake runners amplified their own waves, the cylinders
  exchanged gas with the plenum up to 400 µs late, and the crank step could
  overrun its angle as the engine accelerated. Adjacent steps now differ by
  2-3.5 % on the K20 (8.8-11.4 % before) and about 2 % on the CP2. The 2JZ
  gives 416 N.m at 4,000 rpm (369 before; Toyota: 427), and the Hayabusa's
  maximum falls from 178 to 153 N.m (Suzuki: 138). The dyno discards a whole
  720° cycle when the solver faulted during it. Cost: the LS3 and the Merlin
  no longer run in real time on the development machine (about 0.78 and
  0.80 of real time, against 1.14 and 1.45 before).
- The K20's VTEC switches once per camshaft revolution on a common base
  circle, instead of on the instantaneous engine speed with its valves open
  (up to 106 switches per second per cylinder around 5,800 rpm).
- The exhaust is coupled to the cylinders at least every 125 µs, as
  intended. The interval could run half a sub-step past it, and switch
  between one and two sub-steps from one frame to the next. The level of
  some engines moves (CP3 +16 %, CP4 -19 % in-duct pressure, flat-six +10 %,
  LS3 -4 %); the K20 needs 7 % more CPU.
- The 2JZ's turbo holds its ~0.9 bar to the limiter. Its wastegate was too
  small to bypass the exhaust at high speed: fully open from 5,500 rpm, the
  boost ran to 246 kPa at 6,000 rpm (and 297 kPa at 6,500). It now holds
  190-194 kPa, and the rated power falls from 275 to 259 kW (Toyota: 239 kW).
  The sound changes only above about 5,000 rpm under load.
- The `⋯` menu showed `â€¦` instead of an ellipsis (Save engine…, Import
  engine…) and an impulse response load error `Â·`; a test now refuses such
  text in the sources.
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
