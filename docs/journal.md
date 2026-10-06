# Measurement journal

Short entries (10 lines maximum), most recent first. An entry says what was
measured, how, and what was not. Anything that becomes a lasting rule goes into
`.claude/CLAUDE.md`. Earlier history lives in the `archive/docs-2026-09` git tag
(`docs/archive/`).

## 2026-10-06 — Camera microphone

- `EngineLab.CameraMicrophone`: twice as far, -6.02 dB for a tone, the CP2
  block (-6.02), exhaust (-6.02), intake (-5.99), 2JZ turbo (-6.02); pitch
  979.98-1000 Hz during a 1 kHz glide; largest sample step while moving 0.0049
  against 0.0128 before; a live exhaust/intake swap heard at -6.00/-6.00 dB.
- Switched off, the CP2 stayed 35 dB off its default sound: the float glide
  stalls 0.3 sample short (step < half an ulp). A linear tail lands it: bit
  for bit again 1.25 s after switching off. Mutations caught: 9 of 10, the
  survivor equivalent. Off by default; not listened to or exercised on screen.

## 2026-10-06 — Inspector edits for intake, injectors and boost

- `resizeIntakePart`: read and written where the engine keeps them (own
  runner or the path's), mirrored on a single path (without it, normalising
  puts the old runner back), refused out of range; the runtime takes them.
  Mutations caught: no mirror, own runner ignored.
- The turbine flow area is read at every step (outlet restriction) and by
  the turbo sound per sample: a live edit is not stale. Not exercised on
  screen: the app starts; no click in its window from here.

## 2026-10-06 — Intake sizes while running

- `EngineLab.LiveIntake`, 60 % of redline, change at 6 s (runners +120 mm,
  plenum x1.6, throttle +6 mm): torque built/swapped/untouched CP2
  78.4/78.6/60.3, K20A 223.6/223.6/229.1, LS3 685.6/685.7/628.7 Nm.
- The stiff dyno moves the speed to its new equilibrium within a few frames
  (LS3: 28 rpm): a first "no step" bound (1.5 x the engine's own) failed on
  that legitimate shift; the bound now adds it.
- Mutations caught: runners without their state, plenum not resized, resize
  not keeping its state, scope blind to a path's own size (needed a two-path
  CP2: no catalogue engine has two), audio swap not begun, no warm-up.

## 2026-10-06 — Injectors and turbo while running

- `EngineLab.LiveSettings`, held at 60 % of redline, change at 6 s, mean of
  the last 4 s. 2JZ wastegate 1.89 -> 1.42: boost ratio 1.94 -> 1.68 (built)
  and 1.69 (swapped). Its brake torque moved 1 % only: not limited by boost.
- CP2 injectors: x0.45 and 4-8 g/s change nothing (its own are 20 g/s);
  2 g/s each gives 60 -> 46 Nm, swapped 45.8.
- Mutations caught: no turbo swap, no injection swap, audio update not taken,
  scope blind to the turbo, runtime without its check, layer taking another
  compressor. The app starts; the JSON editor path not exercised on screen.

## 2026-10-06 — Saving an engine as it is

- `EngineLab.SavedEngine`: the 16 catalogue engines, edited (bore -1 mm,
  runners +40 mm), saved and read back run bit-identical over 2.5 s
  (cranking to 70 % throttle); tables retarded 6° likewise.
- The JSON did not name the voicing: a copy lost it (K20, voiced monitor:
  rms 0.0341 instead of 0.0363; the physical reference monitor ignores the
  voicing). Family and key are now in the JSON; the 16 render the same master
  rms and peak once saved.
- Mutations caught: runner length not decoded, tables not read, voicing not
  restored, revision in the "edited" test, rename not applied.
- Not checked: the menu, dialogs and picker dot on screen (no screen here);
  the app starts and closes with the change.

## 2026-10-06 — EJ25: one exhaust path into its one turbo

- Two paths (one per bank) left cylinders 2/4 bypassing the drawn turbo; the
  solver's single turbine already took the whole flow. Now one path (4-1).
- Dyno sweep, WOT, before/after: 2,000 rpm 359/313 N.m (boost 185/151 kPa);
  2,500-5,000 rpm -2 to +3 %; 5,500-6,000 rpm 453-462/402-404 N.m, boost
  236-257/211 kPa (the overshoot past the 2.05 wastegate is gone).
- CatalogReference: 4,000 rpm -6.4/-7.1 %, 6,000 rpm power +10.6/+7.0 %.
- AudioRenderHarness: rms 0.0341/0.0265 (-2.2 dB), bands 39/60 -> 63/37 %,
  main resonance 350 Hz -> 3,986 Hz, mono (LRcorr 1.0). Not listened to.
- DFCO resume (declared rich regression): peak AFR error 31.6 -> 35.1 %,
  misfires 3 -> 1; the test envelope went from 0.35 to 0.40.

## 2026-10-06 — A stopped engine's intake rang by itself

- From a uniform ambient start, ignition off, every engine's runners reached
  ±14 to ±111 kPa in a second (CP2 ±14, LS3 ±54, Radial ±111). Two causes,
  each measured alone: an open valve at zero pressure difference (the
  quasi-steady law has unbounded gain), and, valves shut, the explicit
  plenum coupling (growth ~3.7x per 400 us flush; ±30-70 kPa at 400 us,
  ±5-45 at 100 us, ±0.05-0.25 at 20 us). Mesh 30 mm and RK2: no better.
- Fix: |rpm| < 20 without starter seals the valves and settles the runners
  at plenum pressure. AudioRenderHarness deterministic renders bit-identical.
- Not measured: whether the same coupling adds noise to a running intake.

## 2026-10-05 — Merlin supercharger drawn; cost of the gas view

- `EngineLab.EngineModel3D`: drawn on the Merlin only, eye fed by the airbox,
  charge pipe on the throttle, clear of the engine, impeller at ratio x crank.
  Route counts unchanged. Mutation caught: impeller frozen.
- Budget harness `--gas-view` (field + probe asked at 60 Hz), free-run, 60 %
  redline, best of 4 alternated, off/on: CP2 3.28/3.24, LS3 1.31/1.30, K20A
  1.92/1.92. Within the ±4 % noise: no measurable simulation-thread cost.
- Not measured: GPU and message thread; not seen in the app (the scene
  viewer does not draw in a hidden pane).

## 2026-10-05 — Fine exhaust mesh on demand

- The view's *Fine* setting remeshes the running exhaust from 360 to 180 mm.
  Held at 60 % redline, remeshed at 6 s: LS3 625.94 Nm, against 625.80 for an
  engine started at 180 mm and 629.19 at 360 mm; CP2 60.16/60.33/60.29 (the
  mesh barely moves its torque, so it only proves nothing breaks). Cells:
  CP2 6 -> 11, LS3 26 -> 44. No stall.
- Mutation caught: the live build ignoring the requested cell length.
- Not tested: the menu itself, and the cost in the app (see 2026-10-05 wave
  resolution entry: 13-20 % less headroom).

## 2026-10-05 — Live bore and stroke changes

- CP2/K20A/LS3 at 60 % redline, +3 mm bore and stroke at 5 s: settled brake
  torque within 0.3 % of an engine built that way (CP2 67.80 at once, 67.77
  over 2 s, 67.89 built); resized back, within 0.06 % of untouched.
- At once, one cycle leaves the variability band (CP2 58.9 Nm, AFR 14.8
  against 13.3): its fuel was metered for the old cylinder. Then ~69 Nm,
  drifting to 68 over seconds as the walls warm. Over a ramp: no cycle out.
- 16 engines, 2 mm at 3 s of idle: none stalls (2JZ dips to 625 rpm, idle 760).
- Mutations caught: no TDC wait, no ramp, no kinematics rebuild (74.4 Nm).
- Not tested: the runtime's telemetry, inertia and dyno re-sizing.

## 2026-10-05 — Live exhaust changes

- Gas: an identical exhaust adopted continues bit-identically; 360 to 45 mm
  conserves mass, energy and wall energy per duct. Physics, CP2/K20A/LS3 held
  at 60 % redline, +500 mm and x0.70 diameter swapped at 6 s: settled brake
  torque within 0.2 % of an engine built with it (CP2 56.64/56.64 Nm, 60.26
  untouched). No speed step either way: a fresh network refills in ms. What
  the state transfer keeps is the wall (CP2 332 K at 6 s, < 3 % change).
- Audio: identical swap -75 dB during the crossfade, -114 dB after. 60 ms of
  warm-up left -33 dB, 150 ms -74 dB. Outlet jet noise is seeded: disabled.
- Mutations caught: no wall copy, nearest-cell remap, no adoption, no fade.

## 2026-10-05 — Resizing the exhaust from the 3-D view

- `EngineLab.EngineModel3D`, 16 engines and a scalar-geometry copy of the
  K20A: the resized component is drawn 80 mm longer (a pipe also 4 mm
  wider), every other component unchanged, the configuration valid.
- Found by the test: on a single scalar path the edit did not reach the
  drawing (normalisation copies `exhaust` over the path); fixed by mirroring.
- Mutation: without the silencer guard, the test fails (a silencer narrowed
  to its outlet is accepted).
- Not done: no live preview before Apply; the intake is not editable.

## 2026-10-05 — Wave resolution of the exhaust mesh

- First primary, cell at 1/4 of its length, full load held at 60 % of
  redline, probe at every sub-step averaged over 12 × 1/8 s, against 45 mm:
  360 mm (production) shows 29-61 % less peak-to-peak (CP2 89/127 kPa, LS3
  37/85, K20A 36/94, Hayabusa 51/96), RMS shape error 27-76 %. 180 mm: within
  5 % on CP2, LS3, Hayabusa; the K20A needs 90 mm (cells sampled up to 1/8 of
  the length off the reference point).
- Cost, budget harness free-run, best of 4 alternated, 360/180/90 mm: CP2
  3.28/2.85/2.39, LS3 1.31/1.05/0.73, K20A 1.92/1.67/1.32. Nothing changed.

## 2026-10-05 — Oscilloscope in the part inspector

- `EngineLab.EngineModel3D` (first catalogue engine): a probed cycle fills
  all 360 bins; on every element the probe reads the captured cell exactly;
  rpm and crank angle bit-identical to an unprobed simulator.
- Mutation: runner cells not reversed to gas-flow order fails (element 8).
- Not measured: its cost on the budget harness. It only runs while a duct
  is inspected: one cell read per sub-step.

## 2026-10-05 — Intake waves in the 3-D view

- Premise checked first: is the intake wave too weak for the exhaust's
  scale? Peak departure from each cell's mean over 1 s, 16 engines, idle and
  full load held at 60 % of redline: intake/exhaust 0.57-4.8. Refuted: one
  shared scale, no separate intake scale.
- Runners and plenums now take the wave colours. Mutation: the old
  exhaust-only filter fails `EngineLab.EngineModel3D`.
- Not done: the plenum is lumped (one colour); the airbox, throttle bores and
  inlet duct have no solver cells and stay plain.

## 2026-10-05 — Turbocharger in the 3-D view

- Drawn on the 4 catalogue turbos (2JZ, EJ25, I5, TDI), after the collector
  where all of a path's primaries meet. Route check unchanged (7, 4, 0, 48,
  7); the turbo's solids clear the engine's on all four.
- Mutations: the trunk not resuming at the turbine outlet, a wheel that does
  not turn: both fail `EngineLab.EngineModel3D`.
- Not done: the solver's turbine is a restriction at the outlets, so the gas
  field shows the downpipe at turbine inlet pressure. No charge piping, no
  supercharger (Merlin).

## 2026-10-05 — Pipe routing in the 3-D view

- Route check (`checkRoutes`), 16 catalogue engines. Old layout: 107 clashes,
  51 self-clashes, 0 through the engine, 259 bends < 1 D, 6 stretched. Router
  and placement: 7, 4, 0, 48, 7. Gated at these counts.
- A first checker gave 161 clashes: inner segments bulged past a flat end by
  their radius (a 65 mm collector reached 50 mm out of its mouth). Fixed.
- Stiffness (pull to straight) tried at 0.1-0.5: more bends (97-244 vs 87):
  with the length held it acts as tension. Removed.
- Not done: the 2JZ six-into-one stays a dense knot; +1 stretched pipe there.
  Layout cost: export of 16 engines 1.3 → 4.1 s.

## 2026-10-04 — Gas field on the 3-D view

- `EngineLab.EngineModel3D`, 16 engines: every drawn exhaust component,
  runner and plenum bound to its solver element; capture within one sub-step
  of the asked angle (fails without the guard), simulation bit-identical;
  waves over a 200 kPa mean show (fails against ambient: scale 204 kPa).
- Against ambient the 2JZ exhaust was one orange (the turbine is the 1-D
  network's restricted outlet): colours show the departure from a mean.
- Budget harness as below, alternated: with 1.896-1.923 (7 runs), without
  1.919-1.956 (4). At most ~2 %, the ranges overlap.
- Not observed: the wall glow (798 K not reached in a 15 s run), the
  afterfire flame.

## 2026-10-04 — Exhaust and intake laid out from the configuration

- `EngineLab.EngineModel3D`, 16 catalogue engines: every exhaust component is
  drawn once; no pipe is drawn shorter than authored; runners within 3 % of
  their length; plenum ≥ and airbox = authored volume (mesh volume).
- 6 of 72 exhaust pipes are drawn > 3 % longer than authored: the LS3 X
  (140 mm pipes crossing ~400 mm between banks, ×2.9-3.1), 2JZ cylinder 6
  primary (+16 %), I5 cylinder 5 primary (+6 %). Gated at 6.
- Budget harness as in the entry below, same hour, 4 runs each: with the
  ducts 1.93-1.95, without 1.91-1.93. No measurable cost.
- Not measured: whether the invented routing matches any real engine.

## 2026-10-04 — Cost of the 3-D engine view

- `EngineLabRealtimeBudgetHarness --filter 2JZ --free-run --rpm 4000`, 10 s,
  run while the app drove its own 2JZ at the rev limiter; i5-10600, GTX 1660
  Super, 60 fps cap. Alternated, 2 to 4 runs each: app closed 2.08-2.10, 2-D
  1.98-1.99, 3-D without blur 1.94-1.95, 3-D blur every 6° (≤ 24 sub-frames)
  1.89, every 12° (≤ 12) 1.93-1.95. Kept 12°.
- Frame submission costs 0.07-0.2 ms on the OpenGL thread; the rest is the
  driver. Overruns at the limiter exist in 2-D too.
- Not measured: integrated GPUs, other drivers, frame-time jitter.

## 2026-09-23 — Injection and idle

- Port-injection pulse recomputed every sub-step: ratchet (CP2: AFR 10.3, trim
  pinned at 0.55). Pulse sized once per cycle: 15/15 at φ 1.02-1.04 at idle.
- Fuel pushed back past the throttle was destroyed (CP2 start: 592/1,192 mg);
  kept in the airbox and drawn back in: 25 s start 14/14 except Aircooled.
- **Declared, accepted**: rich DFCO recovery ~0.2 s, peak 16-32 % (HEAD 9-17 %),
  3 EJ25 misfires; LS3 stalls at the 4 s start (fuel stored in the plenum).
- CP2 idle sound: +5.3 dB, darker (−17 dB at 1-2 kHz). Realism unknown.
- Not measured: remaining trim biases (predicted air +17 % CP2, efficiency 0.77).

## 2026-09-23 — Fixes after the afterfire audit

- Hill-start stall: ECU reserve ×2.12 kept ~14 s after start on top of the
  X-tau; limited to the starter. At 4 s: 8 → 13/14 road engines; at 25 s:
  13/14 unchanged. Declared stalls: 2JZ ≤ 8 s (lean, crushed by the clutch),
  Aircooled at 25 s. Accepted by the owner: 2JZ 5,600 rpm +14.81 →
  +15.11 % (16 % tolerance), gear change launched after 20 s of idle.
- Crack layer (noise > 4 kHz driven by the reaction), ratio 1.0 uncalibrated,
  clamped between 0.8 and 2.5: +3.9 dB in 4-8 kHz during the 2JZ pops.
- **CP2 (standard and Full System): the discrete profile burns nothing** (0 % of
  200 mg, wall 390-424 °C); at 650 K instead of 800 K: 18 %. Not fixed.

## 2026-09-23 — Afterfire audit

- **Regression from `d70e8b8`**: both CP2s and the LS3 stall at launch
  (1st gear, full throttle, 0.9 s clutch) in `EngineLabAfterfireHarness`; at
  `75f64fa` the same lab Twin launches and reaches 4,284 rpm. No test caught
  it. `--trace` now prints the trajectory when arming fails.
- Audio front of a backfire: 0.2-0.8 ms (median Twin 0.46, 2JZ 0.61),
  duration 2-4 ms. Energy: centroid 450-790 Hz, 0.2-0.5 % between 4 and 8 kHz,
  0 above (LR8 cutoff at 0.47 × coupling rate). ON/OFF peak: -3 to +10 dB.
- Continuous fuel at 18 %: 0.0 % burned (equivalence 0.18 < 0.45). Same mass in
  packets: 89 % burned (2JZ). Only engine 16 authors the afterfire.

## 2026-09-22 — Project reorganisation

- Blind listening by the owner against real engines: **no engine is
  recognisable beyond its cylinder count**. The rhythm (firing order) is right;
  the timbre resembles no specific engine.
- The afterfire does not sound like a backfire, despite a dozen commits.
- The `references/real-engine-audio/` corpus cannot measure the gap: 10 of 10
  recordings have an unknown engine speed, 7 are proxies, and none is a CP2.
  There has never been a measurable sound target.
- New direction: see `VISION.md`. Pilot engine CP2 (MT-07, Arrow), recorded by
  the owner.

## 2026-09-22 — Commit of the 27 August audit work

Work in progress (34 files: AFR, dyno, intake smoothing, DFCO recovery)
committed after a warning-free Release build and 44/44 green tests. Two points
to know:

- DFCO recovery goes from 1.25× to **1.50× idle**, which an earlier note
  advised against (1.50× alone still stalled without the fuel-film fix). Both
  fixes coexist. Revisit only if an idle or recovery problem comes back.
- A common mechanical loss term above 4,000 rpm was added to pull the top end
  back into line: it is a compensation, not a model.
- 27 August audit: LS3 at 1.151× real time in free-run, Merlin at 88 % of an
  audio block at P99, on the reference machine, without the UI open.
