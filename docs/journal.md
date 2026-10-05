# Measurement journal

Short entries (10 lines maximum), most recent first. An entry says what was
measured, how, and what was not. Anything that becomes a lasting rule goes into
`.claude/CLAUDE.md`. Earlier history lives in the `archive/docs-2026-09` git tag
(`docs/archive/`).

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
