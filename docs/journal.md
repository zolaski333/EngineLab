# Measurement journal

Short entries (10 lines maximum), most recent first. An entry says what was
measured, how, and what was not. Anything that becomes a lasting rule goes into
`.claude/CLAUDE.md`. Earlier history lives in the `archive/docs-2026-09` git tag
(`docs/archive/`).

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
