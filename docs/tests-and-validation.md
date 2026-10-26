# Tests and validation

This page describes the current reliability groundwork: what the test suite
checks, how to run it, and what it does not prove.

## Structural fixes in place

Configuration normalisation treats explicit topologies as authoritative. The
old global properties must not silently overwrite crankshafts, intake paths or
exhaust paths that are already described. The V8 and flat-six presets were
made consistent again with their journals, offsets and path volumes.

The kinematics share one computed reference and cover conventional, master and
articulated connecting rods. TDC positions and chamber volumes account for the
geometry actually configured. Friction at very low speed keeps a resisting
sign.

The gas network uses:

- a signed directional dynamic pressure;
- the real characteristic areas of volumes and restrictions;
- a transfer bounded by pressure equilibrium;
- a simultaneous solution of the flows during valve overlap;
- checked conservation of species, energy and both momentum components.

Events and audio frames are produced at the real sub-steps, with their
simulation time. A reset clears old samples. Per-cycle telemetry comes from the
observed cycle crossings, not from an estimate frozen on the duration of a UI
call.

The runtime and audio were also hardened around bounded queues, the
producer/consumer clock, publishing the engaged gear, ending a dyno run that
does not converge and swapping impulse responses across threads. The continuous
signal stays tied to its exhaust path and the DSP constants follow the real
sample rate.

Finally, the comparison harness explicitly enables pressure-frame production.
Its WAVs can no longer pass while being entirely silent.

## CTest suite

With the tests and harnesses enabled (the default):

```powershell
cmake --preset windows-vs2022
cmake --build --preset windows-release
ctest --preset windows-release
```

Every command on this page uses the Windows tree, which is the reference for
every baseline here. The same three commands work on Linux with the
`linux-gcc-release` preset, and a Linux harness binary sits at
`out/build/linux-gcc-release/tools/<name>` with no `Release` folder and no
`.exe`. A figure measured there is not comparable with one measured on
Windows: [platforms.md](platforms.md) says why.

The main registered tests are:

| Test | Main subject |
|---|---|
| `EngineLab.Core` | simulation integration, gas, combustion, transmission, serialisation, catalogue and basic audio |
| `EngineLab.PhysicsRegression` | authoritative topology, presets, signed dynamic pressure, flows and kinematics |
| `EngineLab.Calibration` | types, interpolation, validation, transactions, concurrency, JSON and file watcher |
| `EngineLab.EcuCalibration` | immediate application of the tables and the rev limiter by the ECU |
| `EngineLab.CalibrationEditing` | block edits of ECU cells: rectangles, add, scale, clamp to the hard limits, smooth, bilinear interpolation, the click step |
| `EngineLab.EngineDesign` | crank balance gives the textbook free forces and couples of inline 3/4/5/6 and cross-plane V8 cranks and a 270 deg twin's 270/450 intervals; ranked inline orders put a balanced six and the flat-plane four first; a designed I7 has the best balanced order, a derived redline with its rule, one exhaust and a final drive; a set value wins and what follows it moves, an unknown key and a missing size are refused, a displacement alone sizes it; V10, W16 (even, given, no angle: spread over 120 deg; two banks at one angle refused), radial (fires from its throw, even count uneven, 30 L refused), diesel, two-stroke (expansion chamber per cylinder, one throttle) and 270 deg twin designs; a `design:` file loads, an unknown key refused; the designed I7 idles within 30 % of its target and makes 10-15 bar BMEP at 60 % of its redline; the real-time probe measures a factor at the harness speed and can be cancelled |
| `EngineLab.LaunchControl` | off by default; at standstill in gear with the throttle past half the limiter cuts at the launch speed and flags it; rolling, part throttle or neutral let go |
| `EngineLab.DataLogger` | CSV header and rows, the interval skip, road speed and limits by name, the log time going on after an engine restart |
| `EngineLab.ShiftStyle` | shift styles on a real engine, ECU and driveline: every catalogue motorcycle's rotating inertia in first, referred to the road, is under 20 % of its mass (fails with the car defaults: 234 kg on 260); a CP2 quickshifter upshift cuts the ignition for its cut length (one frame of rounding), drops the spark, keeps the drive gap under 0.6 of a clutch shift's and ends at the new gear's speed; a 2JZ dual-clutch upshift has no torque gap, and at 85 % of redline its spark retard (20 deg) takes the advance at least 8 deg under the unretarded run and shortens the push by more than 2 % (fails without the retard: the two runs are identical); a CP2 blipped downshift engages at more than 0.95 of the new gear's speed and slips under 0.3 of an unblipped one; the catalogue CP2 is sequential with a quickshifter; the style, shifter and blipper survive YAML and JSON, an unknown style is refused; a CP2 changed down from third with the clutch held out for 1 s engages at once with no blip (fails before: full blip over 42 frames); a dual-clutch 2JZ in first, pedal and throttle at zero, creeps at 1.76 m/s, stops on the brake and never falls under 0.6 x idle; held in second at 75 % of redline it refuses first (stays in second), a manual box takes it |
| `EngineLab.DriverCommand` | driver-command arbitration: the driver's throttle alone, lifted by a torque cut, opened past by a blipper but never closed by it, a non-finite pedal closed, a quickshifter's ignition cut passed through; the engine controls carry throttle, cut, brake, coupling torque, inertia and the cooling air of a reversing vehicle; `advanceVehicle` drives a CP2 driveline bit-identically to a direct `advance` over 10 s |
| `EngineLab.ActiveLimits` | the active rev limit is the authored one untuned and the tuned one after a publish; the ECU reports the rev limiter at the tuned limit, knock retard from 0.5° (not below), coolant protection above 108 °C, nothing on a warm engine below its limit; a free-revving CP2 reports its limiter and no diesel or boost limit, and a CP2 tuned to 6,000 rpm publishes and holds it; a TDI held at 2,500 rpm at full load reports a diesel fuel ceiling and its wastegate, the boost bit following the wastegate on every frame; a CP2 motored between its redline and its limiter takes no damage (1.25e-5 before), motored at 7,000 rpm past a limiter tuned to 6,000 it is damaged; the runtime dyno sweeps to 9,500 rpm untuned, 6,650 with a 7,000 rpm limiter and 11,400 with a 12,000 rpm one; a CP2 held at 6,000 rpm at full load does not report saturated injectors, with 2 g/s injectors it does |
| `EngineLab.Units` | display units: known values (60 mph = 26.8224 m/s, one psi, one pound-foot, one hp and PS, 212 °F, a 350 cu in V8, one US and one imperial gallon...), 10 L/100 km = 23.52 MPG US and 28.25 MPG UK, a fuel cut as infinite economy, a gauge pressure against the ambient and its "(g)" symbol, every unit round-trips, unique ids, unknown ids and indexes change nothing, the four presets |
| `EngineLab.CruiseControl` | the cruise control on a toy car (1,000 kg, drag 150 N + 0.4 v²): holds 100 km/h within 0.3 km/h and up a climb within 0.5, finds the steady throttle, engages at it without a surge, cancels on the brake, the clutch, neutral and a speed far under the set one and then leaves the pedal alone, refuses under 30 km/h, lets the pedal overtake it and returns without winding up |
| `EngineLab.ShiftSchedule` | automatic shift points: the upshift point rises with the pedal and stays under the limiter, a gear the schedule lands in never sends it back the other way (no hunting) over a sweep of pedals, gears and speeds, a kickdown lands under the upshift point, flat out at 1,800 rpm in sixth it drops at least two gears; a dual-clutch 2JZ driven flat out changes up between 0.85 and 1.01 x its limiter, at a 25 % pedal under 0.55 x, and braked to a stop it is back in first without stalling |
| `EngineLab.Cvt` | a CVT on a toy engine of known torque: the variator holds the engine within 5 % of the speed it schedules from the pedal while the ratio climbs, with no step over 20 rpm per frame (no clutch chatter from the moving ratio), for a wet and a centrifugal start clutch at 30 % and full pedal; the ratio moves at a bounded rate and returns to its lowest when uncoupled; a wet clutch creeps, a centrifugal one idles free; braking to a stop never stalls and ends at the lowest ratio; the selector only knows D, N and R. |
| `EngineLab.TwoStroke` | two-stroke ports and crankcase: a port is shut above its edge and fully open at bottom dead centre, the crankcase volume is largest at top dead centre by its compression ratio; a diesel, transfers opening with the exhaust and an injection angle past 360 deg are refused; the catalogue RD350LC-like twin idles above 600 rpm after 4 s and, held at 8,000 rpm at full load, fires once per cylinder per revolution near stoichiometric with positive torque under the maker's (about 33 against 37-40 N.m at the 100 mm production mesh: journal 2026-10-16), within 5 % of its torque with the exhaust coupled every substep (failed before the scavenging coupling window: 29.2 against 33.6 N.m); its load is its delivery ratio, not its crankcase pressure: the ECU's advance at 8,000 rpm does not move between a crankcase at 0.8 and 1.07 x ambient, and at idle (crankcase above 0.6 x ambient) the load stays under 0.4 and within 0.05 of the delivery ratio (failed before: 17.65 against 19.8 deg, idle load 0.77); a JSON or YAML file without crank offsets spaces the twin's cylinders 180 deg apart; a cylinder intake port is covered by the skirt beyond its angle either side of top dead centre, kept by both file formats, refused below 30 deg, delivers less than a crankcase reed at 8,000 rpm and, without its reed, blows back at 3,000 rpm (its trace: `--cylinder-port <deg BTDC>` beside the trace mode) |
| `EngineLab.PeakPressure` | the default peak-pressure limits follow fuel and induction (120/140/200 bar) and an authored one wins; a K20 held at full load fires between 30 bar and its limit, its tracked cycle peak not below any per-frame sample, and takes no damage; with a limit at 70 % of its peak it is damaged (> 0.02 in 8 s) and diagnosed; the TDI at half its rail pressure, injector capacity held, fires more than 2 bar lower. |
| `EngineLab.Turbocharger` | the compressor map holds its design point, its head falls with flow, surges a little above design head at no flow, adds none past choke, and keeps a quarter of the head at half speed and half flow; two identical compressors in parallel hold the ratio of one twice the size, half the flow each, and a slower one passes less; an I4 on two half-size turbochargers matches one double within 1 % (boost, torque, shaft speed); a V8 with 2 + 6 cylinders per turbine spins the small one below 0.8 x the other, with under 40 % of the air; changing the turbocharger count keeps the total size (per-unit flow, areas, inertia and bearing loss) and resets path assignments that no longer exist; a 0.7 intercooler makes more than 8 % more torque at the same boost; the ECU boost table defaults flat at the wastegate setting, holds it, and a table 20 kPa lower holds less boost and less torque; an EJ25 and a 2JZ with the wastegate at 3.2 (rated ratio moved with it, as the editor does), held at 5,400 rpm at full load, hold it within 0.05 at under 0.03 standard deviation and lambda within 0.08 (before the band scaled with the target: 3.28 +- 0.12, lambda 0.71-0.88). |
| `EngineLab.KnockControl` | the ECU retards the spark by the calibrated gain times the knock (4.8 deg at knock 0.4 by default, none at gain 0, 10 deg at gain 25) and reports a knock retard only when it retards; a diesel has no knock entry; the 2JZ held at full load with knock control off runs more than 2 deg more advance and knocks more than 0.05 harder than with it on. |
| `EngineLab.TurboFit` | a turbo fitted by `sizedTurbocharger()`: the laws hold on the four catalogue turbos (turbine area within 10 %, inducer within 15 %, full boost at 35-55 % of the redline); every engine without forced induction takes it and stays valid, and it comes off again; a lower compression takes more boost; the CP2 at 7,000 rpm WOT holds the wastegate boost, gains torque between 0.9 and 1.35 x the charge density ratio, stays rich (lambda 0.78-0.95) and does not knock heavily |
| `EngineLab.IdleStabilityRegression` | every catalogue engine idles at its target and comes back from throttle blips without stalling: a 1.2 s full-throttle blip after the idle settles (lowest speed at least 0.78 × idle, settled within 0.85-1.15 × idle) and a 0.5 s one 6 s after the start (lowest at least 0.78 × idle) |
| `EngineLab.GasDynamics` | the exhaust finite-volume network; among others, a 400 mm pipe authored as two 200 mm components carries a pulse as one pipe when the network reconstructs across their interface |
| `EngineLab.Exhaust` | DAG validation, series/parallel losses, aggregated flow properties, routes, migration and JSON/YAML round-trips; a collector without an authored body holds only what its trunk sweeps (solver and editable network), and resizing an authored collector scales its body |
| `EngineLab.ExhaustSimulation` | effect of the authored K on pressure/flow/torque, independence from the legacy geometry |
| `EngineLab.Scripting` | units, diagnostics, base/include, cycles and keeping the last valid script; the `two_stroke.*` port keys |
| `EngineLab.CatalogRoundTrip` | every catalogue engine encoded, decoded and encoded again in JSON and YAML gives the same text: no field is written without being read back |
| `EngineLab.VehicleDynamics` | tyre friction circle and load transfer for front, rear and all-wheel drive, forward and reverse; catalogue layouts, persistence and diagnostics; a stalled engine left in gear does not move the car; a car coasting in neutral comes to rest |
| `EngineLab.RenderSnapshot` | bounded scene, 3D layout and angle interpolation |
| `EngineLab.SavedEngine` | saving an engine as it is: every catalogue engine, with bores 1 mm smaller and runners 40 mm longer, saved and read back runs bit for bit as before (2.5 s from cranking to 70 % throttle) and re-encodes to the same JSON; every catalogue engine saved and loaded renders the same audio through the shipping path, heard through its own voicing; tables retarded by 6° are saved, read back and run bit for bit; the "edited" baseline (the ECU's default tables are not an edit, retarded tables and resized cylinders are); a damaged table file fails the read; saving without tables removes stale ones; safe file names, catalogue and empty names refused; the folder listing sorted, without table files, a broken file reported; a running engine renamed without a restart names the next dyno run |
| `EngineLab.EngineModel3D` | 3-D meshes and poses of every catalogue engine (rod length, wrist pin on axis, crown below deck), the ducts laid out from the configuration (every exhaust component drawn once, an outlet with no authored length drawn as an open end, no pipe shorter than authored, runners at their length, plenum and airbox volumes, a two-stroke's transfer ducts included and its one shared airbox; on every catalogue two-stroke, with a crankcase reed and with a cylinder port, the exhaust leaving from its window more than a quarter stroke below the deck (it left from the head before), the transfer windows in the barrel flanks, the carburettor low behind the crankcase or high on the barrel back, and a power valve's blade dropping when it shuts), a motorcycle gearbox clear of the barrels on catalogue motorcycles and on designed V2 45 and 90 deg, V4 and inline-four engines, four- and two-stroke (the catalogue Big Twin's case stood 16.6 mm inside a barrel before), the route check over the catalogue (clashes, self-clashes, engine clashes, tight bends and stretched pipes no more than measured), the router on two crossing pipes and a box (they part, keep their length, bend no tighter than one diameter; a pipe that fits stays put), a tube with flat open ends, the turbo (drawn exactly on the four catalogue turbochargers, fed by its collector, on the path every cylinder exhausts into, the exhaust resuming at its outlet, clear of the engine, two wheels turning about the shaft and nothing else), the supercharger (drawn on the Merlin only, the airbox or inlet duct feeding its eye, its charge pipe ending on the throttle, clear of the engine, the impeller turning about its axis at the drive ratio times the crank), the gearbox (drawn on every catalogue engine with gears, one pair per gear with its ratio's teeth within 6 %, the teeth of every pair in mesh at any input angle, the collar against the engaged gear and between first and second in neutral, the output shaft turning with the engaged gear), the gas field (every drawn exhaust component, runner and plenum bound to its solver element; capture within one sub-step of the requested angle; capturing leaves the simulation bit-identical; a wave over a 200 kPa mean still shows both colours, and every intake runner and plenum shows its wave below ambient on the same scale; the pulsation strength holds still while the live wave moves, warm where the gas pulses and grey where it does not; a never-started intake stays at ambient and, once a running CP2 or LS3 is switched off and stops, its runners stand at their plenum's pressure), the oscilloscope probe (a cycle fills every crank bin, every element and cell reads what the gas field shows, the simulation stays bit-identical), the exhaust resize from the inspector (the size shown is the one drawn; on every engine and on a scalar-geometry copy, a component 80 mm longer and 4 mm wider is valid and drawn so (the width checked on pipes), the others unchanged; a length on a collector or an outlet, a 4 mm diameter and a silencer narrowed to its outlet are refused), the injectors (one per cylinder, named with its cylinder by a click, its nozzle ending at its tip, on the intake side; a port injector out of the head spraying at the intake valves (a two-stroke's on its reed manifold, spraying through the reeds), a direct one in the chamber's roof within the bore spraying into the cylinder; both kinds checked), the display crank clock (steady, accelerating, slow motion, freeze, pause) and the camera microphone's frame (distances from the origin kept, up stays up, a camera 4 m behind the drawn tailpipe on the first outlet's acoustic axis for at least 12 catalogue engines) and its sound sources (on every catalogue engine and a scalar-geometry copy, every compiled exhaust outlet finds its drawn tip and axis, every intake path one drawn mouth away from the origin, an open inlet duct sounding from its open end, a drawn turbo or supercharger its place), and the Aircooled's two banks joined into one 6-into-1 (six primaries meeting at one collector, every drawn piece on the kept path, bound to its solver, idling; a path joined to itself, a twin-turbo's paths and the Big Twin's separate pipes refused) |
| `EngineLab.RealtimeRegression` | timing, audio rate, buffers, path isolation and DAG transmission; the starter keeps its level whatever the mechanical fader |
| `EngineLab.LiveSettings` | live injector and turbo changes: taking the running settings leaves the 2JZ and CP2 bit-identical; another injection mode, forced-induction type or blade count is refused; a 2JZ wastegate cut to 1.42 (boost ratio 1.94 -> 1.68) and CP2 injectors of 120 g/min (torque 60 -> 46 Nm at 5,900 rpm) taken at 6 s settle within 10 % of the effect of an engine built that way, without a stall; the runtime refuses an invalid or other machine and keeps its configuration; edits are sorted into exhaust, cylinders, settings and other; the audio turbo layer takes the same settings bit-identically, hears an edited turbo, refuses another machine or a sound that would start or stop |
| `EngineLab.LiveIntake` | live intake sizes: a plenum resized keeps its pressure and temperature; swapping in the running intake leaves CP2 and LS3 bit-identical; another airbox, throttle count or path layout is refused; CP2, K20A and LS3 held at 60 % of redline take runners +120 mm, plenums x1.6 and throttles +6 mm at 6 s, settle within 25 % of the effect of an engine built that way (CP2 60 -> 78 Nm), without a stall and with no frame-to-frame speed step beyond 1.5 times the engine's own plus the shift between the two equilibria; every catalogue engine keeps idling through a smaller change; a designed two-stroke's carburettor jetting is read and written where the engine meters it (an injected engine's refused, a clip out of range refused), and a main jet 1.5 times larger fitted live richens full throttle; a Mikuni 500 reads as a 173 hole and a hole reads back its flow number, a carburettor bored 20 % larger takes a main jet 44 % larger (failed before: unchanged) and an authored venturi keeps its jet; edits are sorted, an edit of one of two paths included; the intake sound's swap of the same intake stays 60 dB below the engine during and 90 dB after, a resized one is heard without a sample step beyond the engine's own |
| `EngineLab.LiveTuning` | live tuning: a K20A driveline in first gear at 2,000 rpm takes a final drive x1.3 and +200 kg at 1 s and settles at the road speed of one built with it (3.08 against 3.09 m/s, 4.00 untouched), another gear count refused; CP2 and 2JZ held at 60 % of redline take the air of 2,000 m (79.5 kPa) at 35 degC at 6 s and settle within 15 % of the effect of an engine built there (CP2 70.8 -> 51.6 Nm, 2JZ 455 -> 362 Nm), an ambient edit being a settings edit; the 2JZ held at 60 % of redline keeps its gauge boost within 8 % at 79.5 kPa (90.2 kPa both); a regulated CP2 stays at 90 degC through 10 s of full load, from the start and once regulated while running, an unregulated one starts at 22 degC and warms up, the switch being a settings edit; the realistic thermal model of the CP2 warms past 80 degC at idle within 30 min (9.8), its fan holding it at 100 degC in still air, overheats at rated heat standing, holds below 100 degC at 30 m/s and stops its fan at 95 degC, the Aircooled having no fan; the 2JZ held at full load for 8 s knocks lightly (mean 0.34) and takes no damage; a flywheel edit is a settings edit; another crankshaft ratio is refused, by the simulator and the runtime; CP2 and 2JZ given a flywheel twice as heavy at idle (3 s) free-rev from 4 s to 75 % of redline within 15 % of the effect of an engine built with it (CP2 0.525 -> 0.94 s), still idling; taking the running camshafts leaves CP2 and K20A bit-identical, at once and over a ramp; a cam edit is a camshaft edit, a profile switch added is refused by the simulator and the runtime; CP2 and K20A held at 60 % of redline take cams 40 degrees shorter with 30 % less lift at 6 s, at once and over 2 s, and settle within 15 % of the effect of an engine built with them (CP2 70.8 -> 55.2 Nm, K20A 223 -> 142 Nm), the duration half-way at half the ramp |
| `EngineLab.LiveCylinderResize` | live bore, stroke and rod changes: resizing to the running size leaves the simulation bit-identical (at once and over a ramp); each journal of a K20A changes at its own gas-exchange TDC; CP2, K20A and LS3 held at 60 % of redline, +3 mm bore and stroke at once and over 2 s, settle to the brake torque of an engine built that way, and resized back to the original's, the cycle torque leaving the band of its cycle-to-cycle variability for at most one cycle at once (the cycle fuelled for the old cylinder) and none over the ramp, and the bore half-way at half the ramp; every catalogue engine keeps running through a 2 mm change at idle (smaller for the Merlin, at the 20 litre ceiling); CP2 and K20A rods 0.3 stroke longer keep the compression ratio and, at once or over 2 s, settle to the brake torque of an engine built with them (within a quarter of the effect or 0.5 %), the rod half-way at half the ramp; a radial's articulated rods are refused; a K20A's compression ratio raised 1.5 while held settles to the brake torque of an engine built with it, above the untouched one's, and a chamber-geometry copy takes a higher ratio by lowering its deck while a ratio its chamber cannot reach is refused, changing nothing |
| `EngineLab.LiveChange` | live exhaust changes: swapping in the same exhaust leaves the simulation bit-identical and the audio more than 60 dB below the engine during the crossfade (90 dB after); a resized exhaust (CP2, K20A, LS3, held at 60 % of redline) settles to the brake torque of an engine built with it, with no speed step beyond the second before and the walls as hot as before; the running exhaust remeshed from 360 to 180 mm has finer cells and settles to the brake torque of an engine started with them (on the LS3, where the mesh moves it, closer to that than to the 360 mm one); a resized exhaust in the audio changes the sound with no larger sample step than before |
| `EngineLab.CameraMicrophone` | the camera microphone: a microphone moved to its own place is bit-identical; twice as far is 6.02 dB quieter for a tone, the CP2's block, exhaust and intake and the 2JZ's turbo; a glide shifts the pitch by 2 % at most and does not click; an immediate move jumps; a microphone in the source is kept 0.25 m away; at the authored positions or switched off the CP2 renders bit for bit the default sound, and switched off after 3 s twice as far it is bit for bit the default sound again within 2.5 s; an exhaust and an intake swapped in live are heard from the moved microphones; an opening moved to its own place is bit-identical, moved halfway to the listener it is louder as 1/r says, moved back bit-identical again; the CP2's outlets and mouths placed 1 m before the microphones (after they moved) are about 12 dB louder, bit for bit the default sound when the microphones are not moved or switched off, and kept by a live swap; the 2JZ's turbo placed halfway is heard at its 1/r level |
| `EngineLab.CatalogPhysics` | deterministic scenarios and loaded holds governed by a PI brake across the whole catalogue |
| `EngineLab.CatalogBoostHold` | every catalogue turbocharger, swept at wide-open throttle every 1,000 rpm, keeps its pressure ratio within 0.11 of its wastegate ratio, and its held brake torque within a 15 % swing (the TDI's boost controller swung it 32 % before it had an integral term) |
| `EngineLab.AudioRender` | multi-engine renders at normalised engine speed, finiteness, dynamics, limiter ceiling and long-run stability |
| `EngineLab.SourceEncoding` | no source file under `src`, `tools` or `tests` holds UTF-8 text decoded as Windows-1252 and saved again (`â€¦` for an ellipsis, `Â·` for a middle dot) |

`ctest --preset windows-release -N` lists the complete set.

`EngineLab.AudioRender` only emits a warning when spectral signatures are too
close; that threshold is not yet a failure cause. This nuance is documented on
purpose: the test prevents invalid or crushed outputs, but does not prove good
perceptual differentiation.

## Comparison harness

The deterministic harness produces, for I4, V8, V-twin and radial:

- a CSV trace per scenario;
- a mono WAV derived from the intra-cycle pressures;
- `summary.csv` with engine speed, torque, AFR, fuel, pressure, IMEP, runner
  resonance, audio RMS and peak;
- an operating gate on every engine of the catalogue.

```powershell
cmake --build out/build/windows-vs2022 --config Release --target EngineLabComparisonHarness
out\build\windows-vs2022\tools\Release\EngineLabComparisonHarness.exe `
  --output comparison-output `
  --catalog-root .
```

The WAV must have non-zero energy and peak. The catalogue thresholds check
start/engine speed, AFR tracking and FMEP on a loaded hold at 65 % of the rev
limiter, as well as the spool of a configured turbo.

To compare with a reference EngineLab result:

```powershell
out\build\windows-vs2022\tools\Release\EngineLabComparisonHarness.exe `
  --output comparison-candidate `
  --catalog-root . `
  --reference comparison-baseline\summary.csv
```

Tolerances are relative: 5 % for mean engine speed and AFR, 2 % for resonance
frequency, 10 % for torque, fuel, pressure and IMEP. The WAV metrics are
recorded and checked non-zero, but do not take part in this numerical reference
comparison yet.

The complete JUCE renderer can be run separately:

```powershell
cmake --build out/build/windows-vs2022 --config Release --target EngineLabAudioRenderHarness
out\build\windows-vs2022\tools\Release\EngineLabAudioRenderHarness.exe --output audio-render-output
```

## Drive-scenario harness

`EngineLabDriveScenarioHarness` plays a timeline of driver inputs (pedals with
ramps, gear lever, starter, "upshift at N rpm") against one catalogue engine
through the product's own coupling (`advanceVehicle`). It writes one CSV row
per public frame (`--csv`) and prints the engine-to-road ratio measured in each
gear (clutch locked, no shift, no wheelspin) against the authored one, 0-100
km/h and 0-60 mph from first motion, top speed, who held the throttle and how
long each active limit held. `--shift-style`, `--quickshifter`, `--blipper` and
`--automatic` override the engine's gearbox so one factor changes at a time;
`--catalog-root` reads engines from another folder (holding `engines/` and
`parts/`), so a variant runs without touching the catalogue.
Built-in scenarios: `pull` (launch, upshift at 90 % of the ceiling through
every gear) and `downshift-clutch` (two downshifts with the clutch pulled). A
scenario file is JSON; its format is in the source header.

```powershell
out\build\windows-vs2022\tools\Release\EngineLabDriveScenarioHarness.exe `
  --engine "CP2 MT-07" --builtin pull --csv cp2-pull.csv
```

## Instruments

Every tool under `tools/` is built with the project (target
`EngineLab<Name>`, in `out\build\windows-vs2022\tools\Release`). Each source
file's header says which question it answers and lists its options.

| Tool | Question it answers |
|---|---|
| `AbClipRenderer` | listening clips, each segment (idle, rev-up...) at its own loudness, for blind A/B against a recording or another engine |
| `AfterfireHarness` | the shape of the overrun heat release in the exhaust (use `--liftoff-rpm`) |
| `AudioAbHarness` | the real-time audio path rendered offline over an idle-to-redline sweep of every catalogue engine, with the same metrics as `AudioRenderHarness` |
| `AudioRenderHarness` | the real-time audio path rendered offline: RMS, crest, DC, spectral bands, exhaust RT60 |
| `ComparisonHarness` | deterministic scenarios on the catalogue with an operating gate, compared against a reference `summary.csv` (above) |
| `CyclicVariabilityHarness` | COV(IMEP) per cylinder at a held engine speed |
| `DesignBench` | designs engines from a spec file exactly as the designer window does, prints or exports every derived value, and writes them as a catalogue `--catalog-root` can sweep: the only way to measure what a user builds rather than what the catalogue authored |
| `DieselControlHarness` | does the editable injected-quantity curve reach a diesel's fuel and torque? |
| `DriveScenarioHarness` | a driving complaint as a timeline of pedals and gears (above) |
| `DynoCurveHarness` | torque at established points, integrated over whole cycles once the engine has settled; writes per-cycle data and a provenance file next to its CSV |
| `DynoRuntimeCurveHarness` | the application's own dyno run, captured from the runtime's public snapshots |
| `DynoSweepHarness` | torque and power over an engine-speed sweep, CSV export; `--advance-offset deg` shifts the whole ignition curve. The sweep is warm-started, so each point inherits its predecessor's state: `--settle s` is what separates a curve's shape from that memory, and `--step` cannot (it moves the grid, the first point and the path at once). `--sample s` widens the averaging window, `--exhaust-cell-mm` overrides the exhaust mesh (a convergence knob, not a setting). `--crank-degrees deg` and `--coupling-us us` resize the mechanical substep and the exhaust flush clock, which is how a gas-dynamic feature is told from a sampling artefact: a real one stays where it is when either changes; `--frame-hz` moves the 240 Hz outer frame the absorber and controls step at, so a feature locked to the speed where the firing rate meets it is told from the engine's own; `--exhaust-whole` keeps a two-stroke's pipes one network, one CFL step for all (the partitioned default moves torque by under 1 %); `--pipe-temperature` prints the first exhaust path's mean gas temperature per element to stderr (`pipe_temperature_c <rpm> <element>:<C>`) |
| `ExhaustFlowBench` | an authored exhaust under a steady mass flow, engine removed: is the back-pressure the pipes' or the pulses'? |
| `ExhaustNetworkCostBench` | does the same geometry cost more solver time when authored as a designer network? |
| `GeometrySensitivityHarness` | does an exhaust change alter the timbre (`shape`) or only the level? |
| `IntakeDuctBench` | bit-exact checksum of the duct solver: unchanged means the physics is untouched |
| `LoadedAccelerationHarness` | a wide-open-throttle pull held in one gear, without shifts, to attribute a torque collapse |
| `OfflineAudioExporter` | high-quality WAV render with stems ([offline rendering](offline-hq-rendering.md)) |
| `PhysicsPerfHarness` | gas-exchange trace at crank-degree resolution; `--trace 1 --idle` is a real idle; `--idle-firing S [--settle S]` idles every engine (or `--filter`, under `--catalog-root`) and prints the share of sparks that lit, the longest misfire run, the diluent at the spark and the firing pattern |
| `RealtimeBudgetHarness` | the real-time factor on the runtime thread (`--free-run` for capacity); back-to-back comparisons only. `--designer-probe` runs the engine designer's own check (`RealtimeCapacityProbe`) on each engine; `--power-throttled` runs it under EcoQoS, as Windows may treat a background app, instead of opting out (`ThreadScheduling.hpp`); `--with-audio --audio-rate HZ` adds the audio consumer at a device rate, synthesised at 48 kHz and upsampled as in the application unless `--native-rate-audio`. It cranks at a shut throttle before the dyno takes the engine (at full throttle a designed two-stroke never caught). `--exhaust-whole` keeps a two-stroke's exhaust one network (A/B) |
| `SceneExport` | the 3-D scene of the catalogue (or `--catalog-root` another, e.g. designs) as JSON, with the route check issues per engine; `--two-stroke-intake crankcase_reed\|cylinder_port` draws every two-stroke with that intake |
| `ShiftTransientHarness` | the exhaust mass-flow step (the "crack") of a clutchless full-throttle upshift, and with `--audio-output` its sound |
| `ThermalHarness` | coolant and oil temperatures over repeated dyno passes, tied to the power lost from pass to pass |
| `TurboDownstreamAuthorityHarness` | does the exhaust after the turbine change a turbocharged engine's power? |
| `UserDynoHarness` | the dyno started as a user does, from an engine already running above the first point: a preparation phase, no stall, points from the low end |

`WasapiLoopbackRecorder.cpp` is not part of the build: a standalone recorder of
the sound card's output (compile line in its header), used to measure another
program's sound with the same analysis.

## Sanitizers and static analysis

In a dedicated build tree:

```powershell
cmake -S . -B build-asan -G "Visual Studio 17 2022" -A x64 `
  -DENGINELAB_ENABLE_SANITIZERS=ON
cmake --build build-asan --config Debug
ctest --test-dir build-asan -C Debug --output-on-failure
```

MSVC enables AddressSanitizer; GCC/Clang toolchains enable AddressSanitizer and
UndefinedBehaviorSanitizer. Another tree can enable
`-DENGINELAB_ENABLE_CLANG_TIDY=ON` if `clang-tidy` is available.

Sanitizer builds are slower, especially `EngineLab.Core` and the harnesses. An
error-free run does not replace dedicated race analysis or a long campaign on
several audio devices.

## What these checks prove — and do not prove

They prove numerical invariants, round-trips, atomic transactions, bounded
outputs and expected trends on the shipped scenarios. They make regressions
visible.

They do not prove the accuracy of a real torque curve, the fidelity of a
specific exhaust, stability on every possible configuration or that an engine
sounds like the real one. Those claims need external data, A/B protocols and
physical references that the repository does not hold today.
