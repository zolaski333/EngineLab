# Changelog

All notable changes to EngineLab are listed here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/) (pre-1.0: minor versions may break
file formats, always with an in-memory migration).

## [Unreleased]

### Added

- Two-stroke engines (`cycle: two_stroke`, with a `two_stroke:` block for the
  port timings and widths and the crankcase compression): piston ports,
  crankcase pumping through a reed, one firing per revolution, fuelled as a
  carburettor is (no overrun fuel cut); the 3-D view and the inspector show
  its ports instead of valves and cams. New catalogue
  engine: a Yamaha RD350LC-like twin with expansion chambers. Known gap: at
  the real-time exhaust mesh the expansion chambers barely work, so it makes
  about a quarter of the real bike's torque.
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
