# Measurement journal

Short entries (10 lines maximum), most recent first. An entry says what was
measured, how, and what was not. Anything that becomes a lasting rule goes into
`.claude/CLAUDE.md`. Earlier history lives in the `archive/docs-2026-09` git tag
(`docs/archive/`).

## 2026-10-22 — Knock: the end gas, a stepped control, Woschni's combustion term

- End gas as its own zone from the seal, isentropic with real cp(T); Douaud-Eyzat delay; Livengood-Wu from the spark; intensity = the constant-volume pressure the unburned share adds, over 30 bar. ECU knock control now steps (20 x excess/s of a 12 deg authority, ~1.3 deg per heavy event, back at 1 deg/s).
- 2JZ at full load, control off: knock 0.83 mean, damage 0.0065; on: 0.08 mean at 4.7 deg, no damage. Catalogue WOT: NA engines at most 0.10, 2JZ <= 0.08, EJ25 0.40 at 5,000 rpm.
- It took three more changes: the flame's load term capped at 1 (boost burned a charge up to 36 % faster); intercoolers 0.7 on 2JZ/EJ25/I5; Woschni's combustion term and the 4V/B liner (TDI BSFC 171/181/199 -> 212/203/215 g/kWh at 2/3/4,000 rpm; petrol WOT 227-310). TDI throat 280 -> 220 mm2: 285 N.m at 2,000 rpm.
- Maker gate (before the TDI refit): all pass but the TDI at 2,000 rpm; 2JZ +9.6 %, EJ25 +12.8 %, I5 +7.0 % torque.
- **Open: the model over-knocks under boost.** Implied knock-limited spark vs calibration practice: ~6 deg early NA, ~11 at 1.9 bar, ~17 at 2.45. The I5 knocks 0.78-0.90 at every rpm with the authority saturated, 0.56 at 2,000 rpm at the -10 deg floor; at 2.3/2.2/2.1 bar still heavy (2.1: 0.47-0.83); direct injection (end gas -15 K) and CR 10 neither cure it.
- Refuted: Kalghatgi's octane index (the zone is 150-200 K colder than his K fit, boost gains 1-3 octane); an ECU boost cut on knock (I5 155 kPa / 363 N.m).
- Leads: the intensity scale (constant-volume rise vs MAPO, a load-scaled threshold); Douaud-Eyzat used at 90-100 bar onsets, beyond its fit; the DI preset going lean above 5,000 rpm (lambda 1.07-1.18); the RD350's 437-1,582 g/kWh.
- Not measured: the sound before/after; CatalogReference and CatalogBoostHold after the TDI refit; the full ctest.

## 2026-10-22 — Two-stroke: a carburettor, and the bore it sits in

- Added: a slide carburettor per intake path (`CarburettorModel`), two circuits on their own depressions -- the main on the venturi's 1/2 rho V^2, the pilot on the bowl-to-crankcase drop -- with jets in their stamped numbers (cc/min under 1 m of head). Its fuel enters the crankcase with the air; the injector is commanded nothing on a carburetted path. The designer fits one to every two-stroke and sizes the jets from the engine: an RD350-sized design gets #287 main / #41 pilot against the bike's 260-290 / 30-40, fitted to neither.
- **A bore bug, found in passing.** A two-stroke's carburettor was sized from the whole engine's displacement and fitted to each cylinder, so `t2_single_125` (16.6 mm) and `t2_four_500` (33.2 mm) were four times apart in area for identical 125 cc cylinders. Now sized per cylinder, with the factor doubled (0.16 -> 0.32) so the RD350-calibrated twin stays at 26.186 mm: every 125 cc cylinder gets 23.45 mm, every 250 cc one 29.60 mm, and both twins are bit-identical by construction.
- Three-way sweep, one factor between each pair (A committed, B bore fix, C B plus carburettor), seven designed two-strokes, 250 rpm steps, 8 s settle. Arm A reproduces the previous entry's roughness to the digit, and the four-stroke control is bit-identical across all three.
- Bore fix, peak torque: the singles gain (125 cc +29 %, 250 cc +18 %) and the multis lose what the oversized bores gave them (triple -6 %, four -9 %, V4 -8 %). Roughness moves both ways (125 cc 1.76 -> 1.40, four 0.97 -> 1.60).
- Carburettor: lambda 0.88-0.94 -> 0.74-0.84, falling steadily with rpm (twin 0.837 at 2,000 -> 0.768 at 9,000). That is the plain jet richening with flow, i.e. the deliberately absent air corrector, now measured. Peak torque -4 to -6 % and roughness +0.13 to +0.35 points on all seven. The 3,500/7,000 rpm holding band is where it was in all three arms (t2_twin_500 at 7,000: 5.7 / 5.7 / 5.8 rpm), so neither factor touches it.
- **Three refutations of that roughness, all mine.** (1) The pilot: shut to #1 it moves lambda 0.768 -> 0.784, 2 % of the fuel. (2) The rich mixture: the main jet x0.90 brings lambda to 0.89-0.94, the injector's own, and the roughness stays (twin 1.91 -> 1.94, triple 2.16 -> 2.38) with 2.5-4 % of torque still missing. (3) Mixture noise: lambda is *smoother* with the carburettor on six of seven (0.11-0.16 % against 0.13-0.17 %), an order of magnitude below torque's.
- So at equal lambda it is putting the fuel in the crankcase rather than the transfer duct that costs 2.5-4 % of torque and ~0.3 points of roughness. Unverified candidate: fuel vapour changes how the crankcase pumps (the triple's delivered-VE roughness rises 1.85 -> 2.34 with it). The larger part, the 1.4-2.1 points the injector arm already had against a four-stroke's 0.32, is not the fuel system.
- A gauge bug of mine fixed: `fuel_g_s` counted the injector only and read 0.000 with a carburettor fitted. Rerun on the fixed binary it is the only column that moves (now 1.4-4.0 g/s on the twin against the injector's 1.6-3.9).
- Not measured: the sound; a real carburetted two-stroke's curve or AFR trace; the catalogue RD350 on a carburettor (it keeps its injector so the reference engine does not move); reed and power valves.

## 2026-10-21 — Two-stroke: integrating the scavenging front, and what the numerics do not own

- The fix for the hunting the one-shot sample caused: the front's progress is integrated every sub-step, `frontProgress += jetVelocity x dt / loopLength`, instead of a transit time predicted from the jet velocity at the instant the transfers crack open. Three-way sweep (model off / one-shot / integral), eight designed engines plus the catalogue RD350, 250 rpm steps, 8 s settle.
- **The hunting is gone.** Mean `held_rpm_stddev` over 4,500-6,250 rpm, one-shot against integral: RD350 5.06 -> 0.48 (model off 0.43), t2_twin_347 4.27 -> 0.49, t2_triple_750 3.31 -> 0.40, t2_v4_1000 3.17 -> 0.33, t2_four_500 1.67 -> 0.26, t2_single_125 3.27 -> 0.72. Worst single point 10.05 -> 2.50 rpm. The integral sits on the model-off floor everywhere.
- **And the curve is smoother than with the model off**, which neither earlier variant managed. Roughness off / one-shot / integral: RD350 5.74 / 6.34 / **1.44**, t2_v4_1000 2.23 / 4.97 / **1.92**, t2_single_250 2.62 / 4.01 / **1.95**, t2_twin_500 2.71 / 3.77 / **1.91**, t2_four_500 1.13 / 2.52 / **0.97**. It beats the shipped model on all eight two-strokes and model-off on seven of eight (t2_single_125 1.76 against 1.72 is a tie).
- Torque: the designed engines gain 2-14 % and keep it (t2_twin_500 +7 %, t2_v4_1000 +9-14 %, t2_single_250 +8 %). The catalogue RD350 is the exception and **loses 12 % of mean torque** (29.68 -> 26.77 N.m): trapped VE falls below 8,000 rpm and rises above it, so the hole at 5,750 rpm disappears and the curve becomes a monotone rise. That is a deliberate behaviour and sound change on a catalogue engine, peakier in the direction a tuned two-stroke should be, but it is not validated against a real RD350's curve.
- Two controls, both held: a ratio-0 two-stroke run is **bit-identical** to the shipped ratio-0 run, and the four-stroke from the same derivation is **bit-identical across all three variants** (roughness 0.32, peak 78.9 N.m @ 4,251). So the rewrite reaches only the scavenging window.
- **The 3,500 / 7,000 / 7,500 rpm holding failures are the plant, not the numerics.** Five runs of t2_twin_500 with the model off, one factor each: `held_rpm_stddev` at 7,000 rpm reads 5.62 (settle 8), 6.16 (settle 16), 5.93 (step 125), 5.81 (crank 1 deg), 5.57 (coupling 60 us). Halving the step also finds 5.29 rpm at 7,376, so it is a band. The brake swing there is not the speed error: 5.6 rpm x the 0.166 N.m/rpm P gain is 0.9 N.m against 14.7 N.m measured, so the engine's own cycle-averaged torque swings about 15 % at a steady hold and the absorber only follows it.
- Two convergence costs the same probe exposed, for the performance pass: **the 2 deg crank step is 10.5 % off at 2,000 rpm** (35.70 -> 31.94 N.m at 1 deg, 6.6 % at 2,250, under 1.5 % from 4,000 up), and **the 125 us exhaust flush clock costs 2.5-4.7 % of torque around the power peak** (6,000 rpm 40.40 -> 42.28 at 60 us). Neither is a bug; both are what the real-time budget buys.
- **Refuted: the 3,500/7,000 rpm band is not the exhaust pipe's.** Same engine at 0.87x, 1.00x and 1.15x of the designer's 947.5 mm tuned length, 125 rpm steps. The torque peak moves as it should (7,375 -> 7,000 -> 6,000 rpm), so the length took effect, but the scatter stays at 6,997/7,000/7,000, 7,375/7,375/7,373 and 3,500/3,502/3,499 rpm. It is locked to crank speed, with 3,500 and 7,000 exactly 2:1, and survives pipe length, settle, step, crank step and flush clock. Untested candidates: the intake side (runner, crankcase, throttle plenum -- the one part held fixed in every probe), and the constant-power accessory term faded in over exactly 3,500-4,500 rpm at `EngineSimulator.cpp:5000`, whose torque falls as 1/omega and is therefore negative damping.
- Not measured: the RD350's changed curve against a real one; the sound; the intake-side probe for that band; a carburettor; MBT calibration.

## 2026-10-21 — Two-stroke: the mean is smooth while the engine is not, and the scavenging model is half the chaos

- Every "is the curve chaotic?" judgement so far read `torque_nm`, the mean over the sample window. `brake_nm_min`/`brake_nm_max` and `held_rpm_stddev` are accumulated only after the settle, so they say whether the settled engine **held**. Brake-torque spread is the wrong one of the two: a twin's within-cycle crank ripple inflates it (the four-stroke control reads 26 % spread at 3,500 rpm with 4.1 rpm of filtered-speed scatter, which is a healthy engine). `held_rpm_stddev` already has the ripple filtered out by the absorber's own 8 rad/s filter, so it is the holding metric.
- **Refuted, mine:** "3,500 and 7,000 rpm are bad on every engine, four-strokes included, so they are a fixed-rpm artefact of the harness." Measured on the filtered speed, the four-strokes are clean at both (0.88-1.14 rpm at 7,000, against 5.5-8.4 for the two-strokes) and 3,250-3,500 is the scavenging model, not the plant. Only the 7,000 rpm point survives as plant: RD350 reads 8.08 rpm with the model off and 8.39 with it on, same torque and same VE, while its neighbour at 7,250 is the cleanest point of the sweep (`rpm_stddev` 2.5 against 44). Not the absorber (no rpm-locked structure in `DynoAbsorberController`) and not an ignition node (the designed map is flat 20 deg from 1,720 to 9,030).
- One factor over the eight designed engines, `scavenge_loop_length_ratio` 1.8 against 0, same binary, 250 rpm steps, 8 s settle. The four-stroke control is **identical to the digit** (roughness 0.32 %, spread 7.3 %, peak 78.9 N.m @ 4,251 both), so the knob reaches two-strokes only.
- The model **doubles** mean-curve roughness on every two-stroke: 2.52-4.97 % against 1.13-2.71 % with it off (+1.06 to +2.73 points). A four-stroke from the same derivation is 0.32 %, so the model owns about half of the designed two-stroke's roughness and the geometry owns the other half.
- Its whole effect is one band. Outside 4,500-6,250 rpm torque and VE are unchanged to three decimals (±0.5 %); inside it torque rises 12-27 % and `held_rpm_stddev` goes from 0.2-0.9 rpm to 3-9.6. That band is the pipe's antiresonance (t2_triple_750 VE 0.453 -> 0.399 -> 0.568 with the model off), and the model fills it: 0.399 -> 0.505 at 4,750 rpm, 46.3 -> 59.0 N.m, scatter 0.44 -> 9.55 rpm.
- Which says what the instability **is**, not just that it exists: only in the trough does the front stop short of the exhaust port, so `frontProgress` sits on the steep middle of the ramp where a small change of crankcase-to-cylinder pressure swings trapping hard, and the cycle-to-cycle loop through the exhaust pipe closes there. Elsewhere the ramp is saturated and insensitive. So the shipped one-shot sample is a sufficient cause but may not be the only one, and the integral-front rewrite has to be measured against the holding columns, not the mean.
- Not measured: whether integrating removes the hunting or only halves it; the ramp width as a second factor; the 7,000 rpm point's mechanism; anything on sound.

## 2026-10-21 — Two-stroke: the product's own dyno is not where the chaos is

- The complaint is about the product's dyno, not the sweep, so `EngineLabDynoRuntimeCurveHarness --mode ramp` was run on the shipped build (25 mm clamp restored, idle host) for the catalogue RD350 and a CP2 four-stroke control. RD350: 145 points, 1,800-9,000 rpm. CP2: 155 points, 1,800-9,500.
- **Refuted, mine:** "a ramp measures while accelerating and `DynoQualityGate` accepts 150 rpm of speed error and 3x the ramp rate, so the two-stroke's accepted points scatter." **Every point of both runs is valid and `quality_reasons` is 0 on all 300.** The gate rejects nothing, so it is not filtering a scatter into existence and not letting one through by a loose tolerance either.
- The two-stroke *is* measurably noisier in the product's own instrument, but by a margin nobody would call chaotic: second-difference roughness **0.71 % against the four-stroke's 0.09 %**, per-point torque variance **3.28 against 0.105 N·m²** (31x), max step 1.65 against 0.54 N·m. Window rpm spread and accepted cycles per point are the same for both (129/119 rpm, 11.8/12.3 cycles), so the controller is holding the two-stroke as well as the four-stroke.
- So neither instrument reproduces the complaint on the **catalogue** engine: sweep 1.58 %, ramp 0.71 %. What the owner's words actually say is "whatever 2T engines **I design**" — the designer's derivation, not the hand-authored RD350, and no two-stroke measurement this session had touched it.
- `EngineLabDesignBench` was written for exactly that: it runs `designCatalogueEngine` over a spec file and writes the results as a catalogue the sweep can measure, so a designed engine is now a first-class measurement subject. Eight designs (125-1,000 cc, 1-4 cylinders, inline and V, plus a four-stroke control from the same derivation) came out with nothing clamped — tuned lengths 840-1,055 mm inside the [500, 1800] range, redlines 8,600-10,800 from the 17.5 m/s piston-speed rule.
- Not measured yet: the sweeps of those eight, running. The ramp also stops at its accepted ceiling (9,025 rpm here), so its 36.52 N·m peak at 8,900 is a transient reading below the sweep's 41.25 at 9,751 and the two numbers are not comparable.

## 2026-10-21 — Two-stroke: the mesh converges at 12 mm, and production is 18 % below it

- Peak torque against exhaust cell length (probe-r18, step 250 / settle 8): **23.20@6,252 / 34.32@9,251 / 39.10@9,751 / 41.25@9,751 / 41.67@9,751 / 41.52@9,751** at 200 / 100 / 50 / 25 / 12 / 6 mm. It **converges at about 12 mm** — 12 against 6 mm differ by 1.47 % of mean torque and −0.36 % on the peak. Production for a two-stroke is 100 mm (`productionExhaustCellLengthM`, confirmed bit-identical to an explicit 100 mm run), so **production sits 17.6 % below the model's own converged answer with its peak 500 rpm low**. 25 mm is within 1 % of converged.
- Torque rising monotonically with refinement is numerical diffusion smearing the blowdown front, which weakens the reflected plugging wave. **Roughness moves the other way** — 2.55 / 3.41 / 6.19 / 6.34 / 7.08 / 7.59 % from 200 mm to 6 mm — so a coarse mesh *smooths* the curve. Mesh aliasing is therefore **not** the "chaotic curves" complaint; production gives the smoothest curve of the set.
- **Trap, and it cost a measurement of mine:** `--exhaust-cell-mm` saturates. `buildPhysicalExhaustNetwork` clamps the target to [25, 600] mm, so a 12 mm request became 25 mm and the two runs came out **bit-identical on all 42 rows**. The clamp was lifted to 4 mm locally for the study above and **restored before anything else was measured**; nothing of it is committed. 4 mm is unreachable anyway — the per-duct cap saturates silently at 75 cells against a limit of 64.
- Separating the two defects this leaves: at convergence the **magnitude is right** (41.5 N·m against the maker's 40.2, +3 %) but the **peak is 1,750 rpm high** (9,751 against 8,000). So geometry/wave timing owns the position and the mesh owns the level. The implied wave speed at the measured peak is `12 x 0.940 x 9,751 / 196` = 561 m/s where the gas there supports 510 — the model's return is about 10 % early, which is the next single factor for position.
- The level defect is **a real-time budget problem, not a physics one**: the code comment on `productionExhaustCellLengthM` records that 100 mm is the finest mesh that ran in real time on the reference machine and that 75 and 50 mm did not (journal 2026-10-15). Two-stroke torque accuracy is therefore gated by solver speed, which makes the performance pass the unlock for two-stroke accuracy rather than a separate errand.
- Not measured: the owner's actual instrument. The sweep is 1.58 % jitter at this step and settle, so the "chaotic curves" complaint lives in `DynoMode::continuousRamp`; `EngineLabDynoRuntimeCurveHarness --mode ramp` on the two-stroke against a four-stroke control is next, on an idle host with the shipped clamp restored.


## 2026-10-21 — Two-stroke: the exhaust chamber owns the curve's shape

- **Both torque humps are the pipe's**, measured by scaling the tuned section alone (stinger, muffler and outlet held, so back-pressure does not move), 42 rows each: at x1.088 the pair 7,245 / 9,751 becomes **6,750 / 9,001** — 9,001 against 8,964 predicted by 1/L, inside one 250 rpm step — and at x1.300 the two merge into one peak, 54.99 @ 7,001, collapsing violently after it (15.39 by 8,251). So the **8,252 trough between them, exactly where the real engine peaks, is the chamber's own wave structure.** Magnitude at the stock 940 mm is right (41.25 against the maker's 40.2) and the longer pipes are overpowered (+33 % at x1.300): lengthening is not the fix.
- **Refuted, mine:** "the designer's `a = 500 m/s` (`EngineDesign.cpp:706`) is too slow for the solver's pipe, so every designed two-stroke peaks high." The 469 °C behind that came from the wrong operating point. Measured egt across 7,000-9,000 is 345-365 °C ⇒ c = 486-493 m/s, so the authored 940 mm tunes to **8,436-8,570 rpm** — exactly the 8,500 the engine file intends. Blair's figure and the designer's rule are both right.
- **Refuted, mine, within the hour:** "there is one resonance, and the 9,751 hump is trapping overtaking falling delivery, not a pipe feature." The length probes move it as 1/L. The error was reading the delivery hump off `max(delivered_ve)`, which selects the lugged 2,000 rpm through-flow (1.43) rather than the mid-range resonance. Trapping *does* rise monotonically 0.448 → 0.843 and that is still unphysical at the top, but it is not what places the peak.
- A real expansion chamber gives **one broad power band**. Two sharp humps with a trough between, both scaling with length, is the diffuser's rarefaction and the baffle's compression arriving as separate sharp events instead of blending. The quasi-1D cone itself is genuine (`DuctGeometry::faceAreaM2` lerps the radius per face, so a taper is a varying-area duct, not a point step).
- Next single factor, running: the **mesh**, already suspect at 34.00 / 36.26 / 38.18 N·m for 100 / 50 / 25 mm with 25 mm called not converged. 50 and 12 mm on the identical catalogue, step 250 / settle 8. If the humps merge and the trough fills as the mesh refines, the structure is discretisation; if they sharpen, it is the model's physics.
- Not measured: whether the real engine's single band is reproducible at any mesh, and any listening on these geometries.

## 2026-10-21 — Two-stroke: a scavenging model that fills one hole and cannot touch the other

- The cylinder was perfect mixing *by construction* (two orifices onto one well-mixed cell), so trapping had no engine-speed term. Replaced by a loop-scavenging law (`TwoStrokeScavenging.hpp`): a per-window **state** whose displacement phase lasts until the fresh front crosses the cylinder, raced against the port window — the front needs `scavenge_loop_length_ratio` x bore over the transfer jet's velocity, the window lasts `duration / (6 x rpm)`, and the breakdown then establishes over one further transit time. Two measured times, no fitted constant.
- The bias goes on the **composition the exhaust port offers the 1-D network**, not on the port areas. The network reports back exactly the species masses it took and the cylinder is debited the same ones, so species, mass and energy conserve for any composition presented. The energy density is passed through untouched: pressure and temperature drive the flow, and this model says what the gas leaving *is*, not how much leaves.
- **One-binary A/B** (`scavenge_loop_length_ratio` 0 vs 1.8, same build, 42 rows, step 250 / settle 8 / 25 mm): mean **+2.22 %**, max **+26.2 % at 5,752**, concentrated in 5,001-5,752. It fills the antiresonance measured there: **23.05 → 29.09**. Below 3,251 and above 6,998 the curve moves less than 1 % (at 2,000 rpm, −0.34 %). Peak torque is **unchanged: 41.23 → 41.25, both at 9,751** — so it does not address the owner's complaint, and the entry above says what does.
- Band-limited for a reason, not by a tuning: the displacing share is `transit/window` = 0.10 at 2,000 rpm (so mixing covers 80 % of the window and there is almost no bias — which is why the low end is untouched) and 0.48 at 9,751, where trapping is *already* 0.83 and the `min(shortfall, residual)` cap binds. You cannot displace with gas the cylinder no longer holds. The effect lands where both a long displacement phase and spare residual exist.
- Controls, all on this build: four-stroke **provably untouched** (EJ25/CP2/LS3 torque and egt identical to 0.0000, audio render bit-identical on every four-stroke — structural, since a four-stroke never opens a window and its boundary returns `networkStateForGasCell` on the helper's first line); a free idle is **neutral** (1,312.4 rpm mean with the loop against 1,305.5 perfectly mixed, spread 38.3 against 39.2, target 1,300 — the 752 and 1,134 rpm idles reported earlier today were the refuted window-width variant); `ctest` 89/89 green. The only sound that moved is the RD350's own (resonance 17.3 dB@1312 Hz → 24.9 dB@335 Hz, rms 0.0079→0.0108), unlistened.
- **Refuted, mine:** "a delivery ratio above one is an internal contradiction." It is not. The crankcase's clearance volume is 2.86 swept volumes and is partly emptied and refilled every cycle, so its ceiling is ~3.9, not ~1.0. A wide-open two-stroke at 2,000 rpm really does blow air straight through — that is why it is filthy there — so `delivered_ve` 1.44 *with* trapping 0.45 was the sound pair all along.
- **Refuted, mine:** ramping the breakdown over the **remaining window**. It over-trapped the low end (48.8 N·m at 2,000 rpm, trapped ve 1.088 with the pipe at only 114 kPa, so no plugging to justify it). At 2,000 rpm that window is 10.7 ms and 90 % of it was given a gentle ramp where a through-flowing cylinder is fully mixed or worse. The transit-time width measured above is the one kept.
- A bug of mine, caught before it reached a measurement: `transfer_port_open_deg` is the angle the piston uncovers the port, not the duration (`2 x (180 - angle)`). A 5 % short window; now `portOpenDurationDegrees`, asserted with the ordering it fixes (exhaust 82° opens first and runs 196°, transfers 116° run 128°). A *larger* opening angle is a *shorter* window.
- Not measured: a descending sweep against the wall-heating confound, and whether the exhaust chemistry refuted earlier today behaves differently on top of this.

## 2026-10-21 — Two-stroke: the pipe never burns, and un-gating it is not the fix

- The RD350's chaotic curve was measured before it was touched: at step 250 / settle 8 the harness curve is **1.58 % jitter**, not chaos (the 5.04 % at step 500 was the coarse grid). The owner's complaint therefore lives in `DynoMode::continuousRamp`, a different instrument, not in the sweep. Settling was real but minor: settle 2→8 cut mean |held_rpm_drift| 3.1→0.3 rpm and moved the curve 1-4 % with its shape untouched.
- Swept past the production ceiling (probe catalogue, redline 13000, 25 mm mesh), the curve is **two lobes, not a ramp**: a flat 25-29 N·m shelf below 5500 with no resonance at all, an antiresonance at 5752 (23.1), lobe A to 36.5 @ 7250, a dip at 8252 — *where the real engine peaks* — lobe B to 41.2 @ 9751, then a cliff to ~9.7 by 11,000. Magnitude is about right (maker: 40.2 @ 8000); position is ~1750 rpm high and the shape has a dip on the real peak.
- Mesh: the expansion chamber does work and the production 100 mm mesh blunts it. Peak torque 34.00 (100 mm) / 36.26 (50) / 38.18 (25) at 9000 — **11 % lost at production mesh, and 25 mm is still not converged**. The `delivered_ve` step at 5750→6000 sharpens monotonically (+0.029/+0.053/+0.089).
- Scavenging is **perfect mixing by construction, with no rpm term**: both ports are orifices onto one well-mixed cell (`EngineSimulator.cpp:3349-3363`), so trapping is a function of delivery ratio and nothing else (0.448→0.807 as `delivered_ve` falls 1.441→0.825). `grep -rn scaveng src/` finds comments only. A real loop-scavenged engine loses charge two ways, both worsening with speed; the model has neither.
- At matched 100 mm mesh the two-stroke pipe runs **300-330 K colder** than a four-stroke's (egt 177/319/469 vs 508/680/778 °C at 2000/5000/9000), where a real two-stroke runs *hotter*. Cause found: exhaust oxidation was gated on the afterfire strategy, so an engine on the default `cleanDfco` never burned anything in its pipe, discarding 36-49 kW of chemical power (four-stroke control: 0.2-0.6 kW). `egt_c` is mesh-independent on a four-stroke (±6 K) but moves 105 K on the two-stroke.
- **Un-gating it is refuted, on three counts.** Peak torque 41.23 @ 9751 → **36.45 @ 7248**; egt at 9000 went *down* (364→357), not up. Four-stroke controls moved (EJ25 −1.54 N·m @ 2500). Four-stroke *sound* moved materially (EL-09 I2 resonance 6281→609 Hz, Aircooled peak 0.4795→0.3620), which `CLAUDE.md` forbids outright. Reverted, never committed.
- Why it failed is an **ordering result**: torque collapses while trapping *rises* (0.81→0.92), the signature of trapped burned product rather than fresh mixture — `ve/delivered_ve` counts trapped mass, not trapped *combustible* mass. Burning the short-circuit at the port mouth lets the plugging pulse stuff oxygen-free gas back in. The slug is 25-55 % of the fuel, which is itself the scavenging defect. So scavenging comes first; the chemistry is re-tested on top of it.
- Control worth keeping: `EngineLabAudioRenderHarness` is **deterministic run-to-run on one binary** (two runs byte-identical on every engine). That had never been checked here, and it is what makes a sound A/B trustworthy.
- Refuted along the way: a warming pipe wall as the cause of the climbing top end (settle 2 vs 8 = 87 s vs 261 s of thermal integration moved it the *wrong* way); "pmep=0 on a two-stroke is a bug" (deliberate — the crankcase pumps, `:3384`). Not measured: the sound of a two-stroke, any descending sweep, and the 10,250 cliff — which **disappears** under the reverted change, so it is composition-sensitive.

## 2026-10-21 — Linux CI: the GUI wrapper trusted `DISPLAY`

- The port's first CI run failed where every local run passed: `EngineLab.AudioWorkshop` in **0.49 s**, the same `BadAtom` the wrapper exists to prevent, with the other 88 of 89 green and no timeout. The cause is in the wrapper, not the test: it reused an already-set `DISPLAY`, and the hosted Linux runner exports one with **no window manager** behind it. WSLg does run one, which is exactly why the local runs all passed — the entry below says both branches were exercised, and they were, but neither was this case.
- It now asks the display instead of trusting the variable: `_NET_SUPPORTED` on the root window, absent on a bare `Xvfb` and present once a manager claims the screen. Measured on Ubuntu 24.04, four conditions, a bare `Xvfb :77` standing in for the runner — `DISPLAY` set to it passes (0.25 s, own server started), no `DISPLAY` passes, a real managed desktop passes and is reused with no nested server (0.08 s), and the binary run straight at the bare server still **fails**, which is what makes the first a fix and not a tautology.
- The probe has to read xprop's **output**: it exits 0 for a property that does not exist and prints `no such atom on any window`. Testing the status, as the first version did, made the check answer yes on a bare server — and is also why the wait for matchbox never waited. Not a timing bug; a shell one.
- Both CI test steps now pass `--output-on-failure`. Without it a job log names the failing test and says nothing else, and a job log is the only record a CI-only failure leaves.
- Confirmed on the pushed commit: **both CI jobs green**, Windows (MSVC, Release) and Linux (GCC, Release), run 39538121420. A fresh clone of the pre-fix commit in WSL also clears `EngineLab.AudioWorkshop` with `DISPLAY` unset, so the xvfb branch is doing the work and not an artefact of this tree.
- One failure in that fresh-clone suite was **self-inflicted and is not a Linux defect**: `EngineLab.Core` failed at 85.68 s while a dyno sweep saturated the Windows host (WSL shares its CPUs), and passes alone twice at 40.4 / 40.1 s. CI's own 4-core runner passed it on the same commit. It does expose a real fragility, though — `tests/CoreTests.cpp` drives a live `EngineRuntime` against wall-clock `steady_clock` deadlines (15 s, 40 s, 10 s) that nothing scales by machine load, so `ctest --parallel 2` on few cores is passing on margin.
- Not measured: whether the hosted runner's `DISPLAY` points at a live server or a dead one (the fix does not care — absent `_NET_SUPPORTED` either way); and the full fresh-clone suite on an idle host, so "89/89 on Linux" is still a claim about this tree.

## 2026-10-21 — Linux: the port, measured

- Scope: `linux-gcc-release` (GCC 13.3, Ubuntu 24.04 under WSL2, 8 CPUs) configures, builds with `-Werror` and runs the suite; `linux-clang-release` (Clang 18.1.3) does the same; CI builds and tests both platforms. Windows stays the reference and `docs/platforms.md` says what differs. Not done: no Linux archive, and the GUI has never been looked at on a real Linux screen.
- Warnings: the first build gave 522, of which ~400 were JUCE's own module units inheriting our flag set (they arrive as interface sources, so they compile *into* our targets), not our code. EngineLab's own 125 came from 5 root causes: omitted aggregate tails on types without member initialisers, shadowed locals, `size_t`->`double`, a `/*` inside a block comment, one unused variable. JUCE's units are quieted per source; its *headers* needed `INTERFACE_SYSTEM_INCLUDE_DIRECTORIES`, because no per-source property reaches a header and one `juce_UMPUtils.h` line broke 14 of our translation units.
- Three things the compiler cannot see. `EngineRuntime` (~7.4 MB) on the stack: Linux has no link-time stack size (the kernel ignores `PT_GNU_STACK`'s), so 4 test fixtures moved to `make_unique`, which is the house rule anyway. The physics thread's priority: POSIX grants `SCHED_RR` only to a privileged process, so the attempt is made and its failure ignored, and an unprivileged Linux run can report a lower real-time factor for the same engine. File names: `std::filesystem::path` reads a narrow string through the active code page on Windows and a wide one through the locale on Linux, so `pathOf`/`fileOf` use wide on Windows, UTF-8 on Linux.
- `-ffp-contract=off` for GCC and Clang: both default to `fast` and fuse `a * b + c`, MSVC under `/fp:precise` does not, and whether a given expression gets fused is a code-generation decision. Predictability was chosen over the FMA because the tests assert tight tolerances. Cross-platform bit-exactness is unattainable regardless (glibc vs MSVC `libm`, `long double` 16 bytes vs 8), so `EngineLabIntakeDuctBench`'s checksum stays a within-platform instrument.
- Two GCC-only diagnostics, neither a bug. `-Winterference-size` fires 34 times on `CylinderWorkerPool.hpp`: suppressed at the declaration, since EngineLab is one binary per build and nothing crosses an ABI, while the per-architecture value is what the padding wants. `-Warray-bounds` on `ExhaustGraph::process` is `std::sort`'s long-range branch (insertion-sort threshold 16) on a 4-element array: the sort length is now clamped against `size()`, which costs nothing at run time. MSVC reports neither.
- Clang is not redundant with GCC: its first run found **26** things GCC accepted (plus 12 in yaml-cpp's headers). 22 were `-Wsign-conversion`, which Clang's `-Wconversion` pulls in and GCC's does not in C++ -- all an `int` counter or rpm bin subscripting a `std::array`/`std::vector`, now `std::size_t` converted once *after* the guard that rejects a negative, since the index is cast from a `double`. 1 was a real conformance error (a captureless lambda reaching a local `constexpr std::string_view`, which comparing against odr-uses; GCC and MSVC both accept it). 3 were dead declarations, including a `SidePanel` member the constructor stored and no method read. yaml-cpp's headers needed JUCE's SYSTEM treatment, so that is now one `enginelab_vendor_headers_are_system` over all three dependencies.
- `EngineLab.AudioWorkshop` died in 0.4 s with `BadAtom` on `X_ChangeProperty`, atom 0: JUCE 8 reads the `_NET_WM_*` atoms with `only_if_exists` (`Atoms::getIfExists`) and, with no window manager ever having interned them, changes a property with atom `None`. Depth 24 and `+extension RANDR GLX` changed nothing; it passes unchanged with a window manager running. `scripts/run-gui-test.sh` is now what `ctest` invokes: it starts Xvfb and matchbox and waits for `_NET_SUPPORTED`, or uses an existing `DISPLAY`. Both branches were run on purpose, because WSL sets `DISPLAY=:0` (WSLg) and a plain `ctest` here therefore exercises only the desktop one.
- ctest: **89/89 on both compilers**, 937 s under GCC and 933 s under Clang (`--parallel 2`), against the ~15 min CLAUDE.md records for Windows. The first full run was not like that: 87 min, with four tests timed out (IntakeTuning, AudioRender, LoadedAccelerationCatalogDfcoResume, StandingStartSettledIdle). None is stuck -- a clean run takes 134/92/109/82 s for those four, inside their *original* limits, and the slowest test in the suite is IdleStabilityRegression at 220 s. In between, AudioRender alone measured 369 s. Same binaries, same flags: wall times here are not reproducible between runs, so no Linux/Windows ratio is claimed, and the 4x scaling of all 31 limits (off Windows only, one place) is slack for a CI runner with fewer cores, not a measured factor. Windows keeps "nothing printed after `Start N:` means stuck" exact.
- Not measured: the GUI or the audio device on a real Linux desktop; the real-time factor on Linux against Windows (scheduling differs by construction, so the comparison would not mean anything); aarch64; macOS.

## 2026-10-20 — Cylinders in parallel on the resident workers (lead 1)

- Audit of `processCylinder` from 8 cylinders up (shared volumes already frozen): it writes only its cylinder's arrays, scratch and runner duct, except 4 engine-wide events (spark count, misfire count + last misfire AFR/time, "solver resolution limited"). These now go through the scratch and are applied in cylinder order after the pass. Two-strokes stay inline (reed fill per intake path); a joint manifold must give each cylinder its own runner duct (checked each step).
- Dispatch: the existing intake pool, one more barrier per sub-step. `EngineSimulatorOptions::parallelCylinderPass` / harness `--inline-cylinders` keep it inline for A/B.
- Bit-identity: `EngineLab.JointIntakeManifold` now also runs the Merlin and compares net torque, spark and misfire counts besides speed and pressures, every frame, 0 vs 3 workers: identical on LS3 and Merlin.
- A/B in the same binary, `--relative-rpm 0.95 --free-run --seconds 15`, 4 alternating runs, warm-up ignored, best: designed V12 0.711 -> 0.877 (x1.23), LS3 1.171 -> 1.368 (x1.17), Merlin 1.400 -> 1.755 (x1.25), 2JZ control (6 cyl, untouched) 1.453 / 1.454. Spread within a variant <= 0.04. Machine: no build/test running; instantaneous load 2-90 % (the harness itself).
- Then, one factor: the end-of-sub-step per-cylinder publish (state + kinematics, writes only its index) on the workers too, A/B against `--serial-publish`, same protocol: V12 0.878 -> 0.933 (x1.06), LS3 1.369 -> 1.427, Merlin 1.770 -> 1.855, 2JZ 1.455 / 1.450. Both together: x1.31 / x1.22 / x1.33 against the inline pass, near the x1.37/1.30/1.39 estimate. The designed V12 stays below 1.0 (0.93).
- Not measured: with audio or the 3-D view; on a machine with fewer cores than this i5-10600 (6 cores, 12 threads; the pool needs at least 4 hardware threads).

## 2026-10-20 — Where a big engine's physics thread goes (profile only, nothing changed)

- How: rdtsc counters (temporary patch, not committed; they cost 1-3 % against the plain binary) + an unelevated stack sampler on Release with symbols; `EngineLabRealtimeBudgetHarness --relative-rpm 0.95 --free-run`, this i5-10600, idle machine, 4 alternating runs each. Factor quoted is the plain binary's best; times are the best instrumented run.
- Designed V12 sport 6.0 L at 7,505 rpm: 0.72. Needs 24,000 sub-steps per sim s (1.88 deg each), reaches 16,900 per wall s, 59.0 us each: cylinders (`processCylinder`) 31 % (1.54 us per cylinder), exhaust network 31 %, intake runners 14 %, serial rest 24 %, incl. per-cylinder loops (end-of-step kinematics 5 %, valve-plane publish 3 %). LS3 6,270 rpm: 1.18, 42.4 us, 29/34/12/24 %. Merlin 2,900: 1.39, 77.9 us, 34/27/21/18 %, 2.19 us per cylinder: its 107 us sub-step exceeds the 100 us blow-by sub-rate, so blow-by runs every sub-step (10 % of the thread; V12 design and LS3 4 %).
- Pool: 72,000-102,000 barriers per wall s (V12 design 1.3 intake + 3 exhaust per sub-step), 2.1-5.2 us each; the master waits 0.2-0.8 us after its own share, 2.6-5.7 % of wall. Workers 20-24 % busy and never park: already resident across sub-steps. These dispatches run at 3.1x on 4 threads (2.7x on 3); empty-dispatch floor 0.30-0.51 us (pool bench).
- Lead 1, cylinders (and their per-cylinder loops) in parallel on these workers, at that 3.1x: V12 design x1.37 (0.72 -> ~0.99; x1.43 if ideal), LS3 x1.30 (-> ~1.5), Merlin x1.39 (-> ~1.9). In Aug 2026 a cylinder cost ~0.6 us; now 1.5-2.2. Not verified: that `processCylinder` writes only cylinder-private state.
- Lead 2, coarse steps while valves shut and nothing burns: 25-29 % of cylinder calls, and such a call costs as much as an open one (V12 1.49 vs 1.52 us). Ceiling x1.07-1.10 at 4x coarser, x1.10-1.13 at most; x1.05 after lead 1.
- Not measured: either lead built; with audio or the 3-D view. Tables and tools: side-session notes `physics-profile-2026-10-20.md`.

## 2026-10-19 — Catalogue Vs: the LS3 and the Merlin share no pin either

- Catalogue V8 and V12 cylinders are generated: TDC from the firing order, odd ids on the -half bank. A row shares a pin only if its two TDCs differ by the bank angle (mod 360). Checked by hand from the files, not changed.
- LS3 (1-8-7-2-6-5-4-3): rows differ by 270, so pins pair 1-8, 3-6, 4-5, 2-7 across rows. With odd ids on the +45 bank every row shares (pins 315/225/45/135, the cross-plane): the GM order assumes the other rotation sense. That fix moves no TDC, but swaps the cylinders' stereo pan.
- Merlin (1-8-5-10-3-7-6-11-2-9-4-12): no bank side shares a pin; the real order 1A 6B 4A 3B 2A 5B 6A 1B 3A 4B 5A 2B with A odd, B even is 1-12-7-6-3-10-11-2-5-8-9-4, which shares all six. That fix changes the firing sequence over its 12 stubs.
- Both are sound changes to catalogue engines: left for the owner. Big Twin (explicit pins): shared, right.

## 2026-10-19 — V cranks: one pin per row

- Owner report: a designed V gives each piston its own crank pin. Measured (SceneExport, throws drawn): V2 90/45 2, V4 4, V6 60/90 6, V8 4 (by luck), V10 72 7, V12 60 5. The even-firing search placed pins freely.
- Fix: a V's or W's row shares a pin (`rowPinCrank()`, every pin set on a 360/rows grid, evenest then best balanced). After: V2 1 (270-450, 315-405), V4 2 (90-180-270-180), V8 4, V10 5, V12 6, all even. 60/90 deg V6 and W16: shared cannot fire evenly, so split pins (60, 30, 5 deg), as the real even-firing V6s; `crank_pins: shared` gives the odd-fire V6 (90-150).
- 3-D view: neighbouring rows with pins alike were merged into one throw (the V12's 6th); now only within a row. Catalogue: EJ25 3 -> 4 throws (right), LS3 7 -> 8, Merlin 11 -> 12: their own data pairs no cylinders on a pin (next entry).
- 8 V designs: idle on target (V4 1,392 for 1,300), designer probe passes except the 6 L V12 at 0.99. Back to back with the old binary, 3 runs each: V12 1.005-1.030 old, 0.993-1.013 new, I6 control 1.74-1.81 both: borderline before, not caused.
- Not measured: the sound of the V-twins now firing 270-450 (designed engines only; the catalogue's are unchanged).

## 2026-10-19 — Small engines: refused, then idling high

- Owner report: a 200 cc single cannot be designed. 31 designs (singles 50-250 cc x 5 uses, small twins/four, 2T, diesel): 29 refused, all with the generic validator line. The validator now names the value: final drive 8.0-20.2 over the limit of 8 (a motorcycle's ratio includes the primary: ~10 on a 125), inertia 0.002-0.009 under 0.01; under 0.1 L also plenum, piston mass (< 50 g), throttle (< 15 mm), and 50 cc failed a 0.05 L check by round-off.
- Fix: final drive limit 15; the designer holds every such value at the simulator's limit and says so in its rule. 31/31 design.
- Idle, then: 50 cc 2,042 rpm for 1,350. One factor, the per-bore closed-throttle leak 0.6 -> 0.1 mm2: 1,329 (cruiser 1,196 -> 800, sport car 1,385 -> 901). Rule: 0.6 mm2 x bore / 60 mm below 60 mm. 15 rebuilt designs: idle unchanged (CP2 1,350 -> 1,349, RD350 1,316 -> 1,311). 26/31 small designs within 30 % of target.
- Not fixed: sport motorcycles <= 125 cc idle ~2,000 for 1,300. One factor each on the 125: leak 0.05 1,859, bypass 0.5 2,009, inertia x3 2,029, bore/stroke 1.15 1,761, road cams 1,361: the sport cams' overlap on a small cylinder. No rule from size is backed by data (the CP4 design, 250 cc a cylinder, idles on target). Diesel single 0.3 L 1,106 for 800; 50 cc two-stroke 3 Nm (BMEP ~4 bar).
- Full load: 125 road single 14 Nm, 11.4 kW at 9,000 (BMEP 14 bar); 50 cc 5 Nm; 250 twin 27 Nm.

## 2026-10-19 — The designer's real-time check failed engines that run fine

- Owner report: some designs fail "the 5,000 rpm test" yet run well. New `EngineLabRealtimeBudgetHarness --designer-probe` runs the designer's own check. Before: 3/3 singles (450-650 cc) and the Merlin and the radial (catalogue) "did not hold" in 30 s; 14 other designs and 15 catalogue engines passed.
- Singles: held on the mean (5,000 rpm), but one polled sample swings 4,711-5,436 rpm (trace at 2 deg: +3.7 / -5.7 % on the 500 cc); the check wanted every sample within 3 % for 0.5 s. A test-binary single passed by poll timing, so the regression test is a synthetic +-5 % ripple (fails on the old rule).
- Merlin, radial: the dyno cannot pull a full-load engine up to 95 % of the limiter; they settle 5 % short (2,900 / 3,040, 2,151 / 2,280 rpm).
- Rule now: mean speed (1/3 s filter, as the harness) within 3 % for 0.5 s, or settled (< 0.5 % in 1 s) within 15 %, measured where it settled. After: 17/17 designs and 17/17 catalogue engines measured in 1.9-4.4 s; factors as before within run-to-run noise.
- Not measured: the check from the app with the 3-D view and audio running.

## 2026-10-19 — Builder phase 6: W engines in 3-D, and a W with no angle

- A W designed with no bank angle had every bank at 0 deg: four cylinders stacked per row. The phase 4 W16 figures were that engine. Rule now: banks spread over 120 deg (at most 90 between neighbours; 3 banks 60 deg, 4 banks 40 deg); two banks at one angle are refused. Corrected W16: physics-only 0.533 (stacked 0.537, so cost is per cylinder, not geometry), still INVALID with audio; idle 901 / 900 rpm.
- 3-D view: banks > 2 were drawn as a radial. Now bank by bank, banks next in angle interleaved half a pitch. Measured with SceneExport (`--catalog-root`, new) and RouteCheck, one change at a time on the W16: stacked banks 213 duct clashes / 236 through the engine; angles fixed 58 / 70; plus the W drawing 15 / 8. Deepest liner overlap 96 -> 57 -> 30 mm (V12 design: 29). Catalogue and 15 rebuilt designs: totals identical before/after (9/4/0/48/7, 16/7/6/40/13).
- Not measured: the view in the app (screen access declined; front/side projections of the exported meshes only), a W's exhaust stubs from the inner banks (they point at the outer head).

## 2026-10-19 — Builder phase 4: spark advance, real-time check, idle

- Advance: DynoSweep `--advance-offset` -6/-3/0/+3/+6/+9 deg on the 15 rebuilt designs, `--step 1000`. Most still gain at +9 (2JZ, EJ25, CP3/4, Hayabusa, I5, flat six: +2 to +9 %); K20A, Harley, RD350 peak at -3..+3; the TDI ignores it. No MBT auto-calibration: designs already exceed the maker's power (2JZ 312 vs 206 kW, flat six 289 vs 200) and nothing here limits advance by knock, so "best torque" would walk away from the datasheets.
- Real time, 7 harder designs (free-run, audio + gas view): I7 1.51, radial 9 1.60, I10 1.16, V8 diesel 0.99, V12 sport 0.95, two-stroke four 0.93, quad-turbo W16 INVALID (physics 0.54). V12 physics only 1.00: exhaust mesh 600 mm +3 %, intake mesh 0; it is per-cylinder work, rising with rpm.
- The whole chain ran at 0.88-0.97 of the physics-only factor (19 designs). The designer now runs a design for ~3 s (`measureRealtimeCapacity`, same speed as the harness: 5,000 rpm, at most 95 % of the limiter) with the live engine paused, and asks before building under 1.14 physics-only. The I7 probe read 1.598 against the harness's 1.599.
- Idle, 22 designs, PhysicsPerfHarness `--freerev 0 --seconds 25`, last 8 s: 21 within 7 % of target, no misfire. Two-stroke four 2,070 rpm for 1,300, misfiring: one factor at a time, bypass 0 2,203; idle target 1,600 2,475; one throttle body 1,433, no misfire. Rule: a two-stroke gets one throttle (the catalogue RD350's carburettor-to-crankcase convention). RD350 design idle 1,393 -> 1,304 (sd 42 -> 16), WOT -2 to -5 % (34.4 kW at 9,000, maker 35).
- Not measured: the designer window on screen (access declined), the probe inside the app against the harness, catalogue engines' advance sensitivity, transients.

## 2026-10-19 — Builder phases 1-3: the catalogue rebuilt from datasheets alone

- `designEngine()` (catalog): cycle, fuel, use, layout, cylinders, bore x stroke (or litres), induction -> a whole engine; each value derived by a rule fitted to the catalogue, or set by key (its rule kept). A file may hold `design:` instead of `engine:`, so every harness runs designed engines.
- Rebuild: the 15 distinct catalogue engines as datasheet specs only (size, layout, use, boost, firing order or crank offsets). Dyno `--step 1000`, peak torque design / catalogue 0.88-1.18 (median 1.04): K20A 244/236, LS3 718/645, EJ25 372/424, CP4 104/116, Harley 212/188, flat six 406/344 N.m.
- Power first ran high where the derived redline was (sport car 23.5 m/s: 2JZ 8,200 rpm vs 7,200, 349 vs 276 kW). Refit: sport car 22.0 m/s, boosted petrol 1.5 lower. Then 2JZ 312, EJ25 267, I5 297, flat six 289 kW (catalogue 276/235/283/203). Not refitted further: 15 engines are the whole fitting set.
- Two-stroke on the four-stroke 4-2-1 part: RD350 design 13.5 kW at 9,000 rpm. With a generated expansion chamber per cylinder (tuned length 500 x exhaust duration / 12 n, Blair; RD350LC proportions): 36.1 kW (catalogue 34.2, Yamaha 35).
- Crank: V/flat/W banks, W bank angles as given (W16 +-37.5/+-52.5); a radial's TDCs follow its single throw (an even-count radial cannot fire evenly). With no order given, the best balanced one is searched.
- Correction to 2026-10-18: the W16 had no banks, so it loaded as one bank at 0 deg (a 16 in line); its idle and real-time figures are not a W's.
- Real time (free-run, audio + gas view, one run each): 13 designs 1.26 (LS3) to 2.88 (CP2); catalogue 1.20-2.83. Merlin and radial designs INVALID (leveler, limiter, clamp): outlet jet 1,640 / 920 Pa vs 1.5 Pa. Not the voicing (catalogue Merlin's: same); the stack lip, 108 mm from "1.1 x collector" vs the part's 140. Lip 2.7 x stack, radial on `radial_collector`: valid, 1.29 / 1.91, jet 0.3 / 0.7 Pa.
- Not measured: sound of any design against its catalogue twin, idle of all 15, transients.

## 2026-10-18 — Builder phase 0: eight engines written with no base

- Question: what breaks when an engine is described from nothing? Eight files in `tools/torture-engines/engines/` (I7 petrol, single, flat twin, V10 72 deg, V16, I7 diesel = TDI x 7/4, W16 as four `bank_offset_deg` banks, two-stroke triple = RD350 x 3/2). Catalogue root = that folder plus `parts/` and `voicing/default.yaml` copied beside `engines/`.
- All eight load, validate and hold their idle 20 s (PhysicsPerfHarness `--freerev 0`): mean at target within 1 % except W16 711 vs 750 (672-735). The triple misfires at idle (0.55 at start) on the four-stroke 4-2-1 part.
- Dyno (`--step 1000`), peak N.m and BMEP: I7 329 (14.7 bar), single 72 (13.8), flat twin 142 (14.2), V10 518 at 3,000 (13.5), V16 869 (13.7), I7 diesel 611 (22.3). Generic parts give upper-edge NA BMEP (typical 11-13.5); the V10's peak at 3,000 rpm on a 9,000 rpm engine shows parts not matched to it.
- Real time (free-run, audio + gas view, one run each, not back-to-back): I7 1.29, single 4.03, flat twin 2.50, I7 diesel 1.22, triple 1.59, V10 0.96 (below the hard limit), V16 and W16 invalid (40,000 dropped pressure samples); V16 without audio 0.65 (exhaust substeps 28 kHz).
- So the format already takes any piston layout up to 32 cylinders; what a from-zero builder lacks is matched parts (cams, pipes, plenum for the target rpm) and a real-time cost known before the engine is delivered.
- New `CrankBalance` (foundation): free forces and couples, firing intervals, ranked even-firing inline orders. EngineLab.EngineBuilder checks the textbook I3/I4/I5/I6 and cross-plane V8 (primary couple sqrt 10) values. Best I7 orders: 1-3-5-7-6-4-2 and its mirror (couples 0.27 / 0.30 m r w^2 a); the torture I7's 1-4-7-3-6-2-5 was not among the top four.
- Not measured: sound of any of them, the 3-D view (W16 would be drawn as a radial: banks > 2), V/W axial positions (assumed one pitch per row).

## 2026-10-17 — The Merlin's runtime spike was a late frame stamped in the past

- `EngineLab.AudioRender` and `AudioRuntimeDiscontinuity` (Merlin, live runtime) failed CI on 6a1d505. Locally 2 of 40 runs fail alone, 15 of 50 with two runs in parallel (preLimiter up to 8.9, observer up to 3 kPa).
- Per-block trace (harness instrumentation, not committed): every spike follows 20-37 ms in which the renderer's rpm does not move, i.e. one physics frame preempted while computing. Its samples were stamped from the frame's start, so all of them arrived behind the renderer's 20 ms look-ahead; the renderer stepped through them in one audio sample, and the exhaust boundary's jump radiated as a one-block, full-scale click. Not the rev limiter (one spike at 1,696 rpm), not the jet nor reactions (0).
- Fix: a frame published more than 10 ms after its start is stamped from 10 ms before publication (`MonotonicPublicationTimeline`); the renderer then interpolates across the gap. A frame in time keeps its stamps, and offline renders do not use the runtime.
- Same stress, 50 runs: 0 failures, preLimiter max 0.42 (median 0.35, as alone). Same overrun band (400-699): 12 of 34 failed before, 0 of 12 after. dropped pressure samples 0 both.
- Not measured: a long run on a machine below real time, and the audibility of the held boundary during the gap (a short freeze instead of a click).

## 2026-10-16 — Two-stroke exhaust coupled every substep from BDC to transfer close

- Where 100 mm lost (RD350, 8,000 rpm, single-point dyno, one factor each): first the cylinder/exhaust coupling, not the cells. Production couples every 117 us (3 substeps, exchange applied as a lump): 27.6 N.m; every substep 31.3; 25 mm coupled at 40 us 35.9. 78 us gains nothing over 117 (not explained).
- Fine coupling inside one crank window only: blowdown 28.0, 115-180 deg (first half of scavenging) 26.8, whole scavenging 115-245 30.8, BDC to transfer close (180-245) 31.1. There the cylinder pressure is a few-kPa balance between two open ports, and it sets the trapped charge. Finer transfer (intake) coupling: 26.0-27.4, not a lever.
- Kept: a two-stroke couples every substep while a cylinder is between 180 deg and 360 - transfer open. Dyno sweep (N.m, coarse -> window, every substep): 6,000 19.2 -> 22.3 (21.3), 7,000 24.6 -> 28.3 (27.9), 8,000 26.0 -> 28.9 (29.2), 9,000 29.5 -> 33.6 (34.5); 3,000-5,000 within 1.4 %.
- Real time (free-run, audio + gas view, 9,025 rpm, 4 alternated rounds): window 1.057-1.068 (36.0 N.m, 34.0 kW), coarse 1.164-1.177 (31.7, 30.0), LS3 control 1.107-1.123. Whole scavenging window 0.968-0.985 and every substep (cpl 40 us, dropped samples) are not real time.
- EngineLab.TwoStroke: production within 5 % of every-substep at 8,000 rpm (33.0 vs 33.6; fails before: 29.2). Gates: idle 1,300 std 14.7, WOT blip min 1,133; launch at 4/25 s min 704/574. Four-stroke dyno and IntakeDuctBench bit-identical. ctest 89/89.
- Render `--catalogue-filter RD350`: RMS 0.0077 -> 0.0079, bands 0.4/77.8/21.8 -> 1.0/72.6/26.5 %, brightness 0.48 -> 0.50, crest 6.7 -> 9.3, max 6,902 -> 7,232 rpm.
- Left: ~35-36 N.m converged vs 37-40.2 (maker). Not measured: any recording, scavenging efficiency against literature, friction.

## 2026-10-16 — RD350 combustion: maker's timing, crankcase no longer read as load

- Burn phasing from traces (CA10/50/90, `burn.py` on PhysicsPerfHarness `--trace 8000`): with the maker's fixed 20 deg BTDC (CDI, 2.0 mm, workshop manual as quoted by owners) and the default flame, CA50 sat at 28.5 deg ATDC, far past MBT (8-10); the catalogue's 13 deg gave 29.0.
- Cause 1: the two-stroke "MAP" is its crankcase (75-87 kPa at idle, ~107 at WOT), read as load: the flame saw an idle at 0.77 load and the ECU took 2 deg off at WOT for "boost". Load is now the delivery ratio, in the simulator and the ECU (EngineLab.TwoStroke, fails before: 17.65 vs 19.8 deg, idle load 0.77).
- Cause 2, a calibration, not a measurement: `chamber_turbulence_intensity_ratio` 2.0 (squish head) puts CA50 at 9.2 deg ATDC with 20 deg (CA10/90 -4.0/17.6, peak 51.5 bar; 1.0: 28.5, 1.75: 12.1). 1.75 with the MAP load failed the idle gate alone.
- Single geometry factors, earlier (12.3 N.m state, 8,000 rpm): crankcase compression 1.50, exhaust port 50 mm wide, transfers 115 mm, throttle 36 mm, runner 50 mm gave 10.3-13.7; none kept. The port timing matches two forum port maps that agree (exhaust 193-209 deg, transfers 125-130 deg duration); it was not swept. Port widths were not checked.
- RD350 dyno 100 mm (N.m, before -> after): 3,000 21.1->23.1, 5,000 19.1->21.9, 8,000 21.9->26.0, 9,000 22.6->29.5 (27.8 kW). 25 mm: 8,000 30.1->34.1, 9,000 26.8->37.2 (35.1 kW; maker 34.5 kW at 8,500, 37-40.2 N.m at 8,000).
- Gates: idle 1,300 std 14.7 drift -0.8, WOT blip min 1,122 settled 1,257, early blip 1,128; launch at 4/25 s min 704/574. Four-stroke dyno and IntakeDuctBench bit-identical.
- Render `--catalogue-filter RD350` (both entries of the day): RMS 0.0086 -> 0.0077 (-1.0 dB), bands 1.9/68.0/30.2 -> 0.4/77.8/21.8 %, brightness 0.51 -> 0.48, max 6,241 -> 6,902 rpm. Not measured: any recording, the real burn duration, 12.5 mm with this setup.

## 2026-10-16 — Two-stroke exhaust reconstructed across interfaces; where 100 mm loses

- The chambers are short cones: at 100 mm most are 2-3 cells, and duct end cells were first order, so a whole cone was. Reconstructing across every open duct-to-duct interface (on for two-strokes only), RD350 at 8,000 rpm: 100 mm 12.1->21.9 N.m, 50 mm 17.9->27.1, 25 mm 22.3->30.1; 18 mm 30.9, 12.5 mm 31.3 (converged near 31-32 with the timing of the day). New EngineLab.GasDynamics case: a 400 mm pipe authored as two 200 mm components carries a pulse as one pipe.
- What 100 mm still loses against 12.5 mm (one factor at a time): the time step ~2.2 N.m, the port cells ~1.8, the rest ~5.6. The first cell behind a port holds ~126 cm3, about the cylinder's volume, so the blowdown lingers in a plenum.
- Not kept (8,000 rpm, 100/25 mm): third-order kappa = 1/3 faces 21.6/29.8; one-sided slopes at open ends 22.5/30.4; the characteristic valve 22.7/28.2 (gains at 100, loses at 25: mesh-dependent). They stay as diagnostics.
- Real time (free-run, audio + gas view, 9,025 rpm, 4 alternated rounds after a warm-up): RD350 reconstructed 1.149-1.170, first-order ends 1.287-1.302, LS3 control 1.081-1.114. The reconstruction un-fuses RK2's second stage on two-strokes.
- Not measured: a solver fast enough for 25 mm in real time (multirate per duct is the lead), any recording.

## 2026-10-16 — Two-stroke fuelled from its reed fill; two-strokes now at 100 mm cells

- Lean launch (RD350, 100 mm, launch after a 25 s idle, traced every 12.5 ms): the pulse (sprayed 250-355°) was sized from the air across the transfers latched at TDC, i.e. for the event before last, and that transfer air itself lags the crankcase fill by one event (mg per event from idle: fill 81, 328, 287; transfers 18, 127, 192). Five lean misfires before the clutch bit.
- A, latch the transfer air at transfer close: min 389/592 rpm (launch at 25/4 s), but stalls a 12 s launch and at 90 mm; at 360 mm the early WOT blip falls to 614 (gate 1,014). Not kept.
- B, kept: the reference is the reed inflow into the crankcase since the last transfer close, latched at that close. 100 mm: launches at 4-30 s all survive (min 430-653, stall 220); 90/110/125 mm pass; 80 mm stalls at 25 s on top of its known idle hunt. At 360 mm B also fails the early blip (626): it only stands with the finer mesh, rich early idle (below).
- Two-stroke production mesh is now 100 mm (`EngineSimulator::productionExhaustCellLengthM`), in EngineRuntime, the app's fine view (never coarser) and the designer's cost. IdleStability RD350: mean 1,322, std 28.8, drift -42.7, WOT blip min 1,251, settled 1,433 (gate 1,495; 1,258 before); early blip 1,372. Standing start at 4/25 s: min 629/486.
- RD350 dyno (N.m, before -> after): 2,000 15.3->20.5, 3,000 15.3->22.4, 5,000 13.9->15.7, 8,000 9.8->12.3, 9,000 8.2->9.7 (maker 37-40). Four-stroke dyno sweep and IntakeDuctBench checksums bit-identical. Render `--catalogue-filter RD350`: RMS 0.0056 -> 0.0086 (+3.7 dB), bands 1.6/87.0/11.4 -> 1.9/68.0/30.2 %, brightness 0.38 -> 0.51.
- Left: B coasts lean after a WOT blip (AFR 18-25, misfire 0.6-0.9; reed fill ~3 mg vs 4-7 mg of oxygen-based delivery, not explained), hence the settle near its gate. While cranking the trim winds up to 2.2 and acts as a choke (early idle rich). Not measured: the real-time factor with B (no added cost expected; 1.22-1.26 is the 2026-10-15 figure), any recording.

## 2026-10-16 — Aircooled soft limiting at 7,030 rpm: duct steepening into an open pipe

- RealtimeBudgetHarness `--with-audio --relative-rpm 0.95`: the Aircooled is INVALID on soft limiting with or without `--gas-view` (3 runs without it: 12-22 samples). Its pre-limiter peak is 1.54-1.80 against 0.09-0.43 on the 16 other engines, and its exhaust observer reads 590-712 Pa against 26-140. AudioRenderHarness does not reproduce it: its catalogue point never passes 6,103 rpm (pre-limiter 0.45). GeometrySensitivityHarness held at 7,030 rpm does: pre-limiter 2.20, 1,237 soft-limited samples.
- Signature: positive spikes 2-5 samples wide (99 % positive), spaced by one bank's firing interval and not aligned to blocks. Onset is near 5,500 rpm: 16 of 206 cycles over 0.82 at 6,200 rpm, 122 of 234 at 7,030.
- One factor each at 7,030 rpm (offline, with diagnostic switches that are not committed). Exhaust reactions: 0 events. Reaction crack off: unchanged. Valve-flow high band off: 2.04. NonlinearDuctAcoustics off: pre-limiter 0.26, observer 883 -> 62 Pa, RMS -4.8 dB, no soft limiting. A guard against reading backwards in the warp: 1.83 only.
- Cause: at these in-duct amplitudes the steepening warp sits on its ±30 % clamp, so each long duct turns the blowdown front into a jump of up to ~0.23 x its transit time (~13 samples on a 720 mm primary). The flat-six is the only road exhaust without a silencer, so its open outlet radiates that step as an impulse. Steepening stays: it carries every engine's 1.5-4 kHz content (2026-07-21).
- Silencer as one factor (scratch catalogue, `flat6_equal_length` + 130 x 380 mm chamber). Packed (20,000 Pa s/m2, 30 mm, 0.30): pre-limiter 0.29, no soft limiting, RMS 0.112 -> 0.051 (LS3 0.060). Realtime, 2 runs: softLimit=0 in both (one run INVALID on 42 late events, a timing failure, not a level one). Dry chamber alone: 1.42. Torque -0.4 to -2.1 % at 3,000-7,000 rpm (every row UNSETTLED_TIMEOUT).
- Not changed: the flat-six keeps no chamber, as decided on 2026-07-21 ("near-open pipe in reality"). Yet the preset's own comment says the silencer sits behind the engine, and a stock 964 has a rear silencer (not checked against a drawing). This is the owner's call. Not measured: whether the spike height depends on the sample rate (exporter at 48 vs 96 kHz was inconclusive), or any recording.

## 2026-10-15 — Two-stroke exhaust stays at 360 mm: 50 mm is not real time, 100 mm stalls the launch

- Profile (RD350, 9,025 rpm, 50 mm): the exhaust FV network is most of the physics thread; interfaces were solved serially. Each interface's Riemann solve now runs in its upstream duct's pool share: +4.5 % on a pooled run (min of 4 alternated, 1.262 -> 1.319), bit-identical (four-stroke dyno sweep, IntakeDuctBench, RD350 standing starts unchanged).
- MUSCL-Hancock (diagnostic `--exhaust-hancock`, one stage per substep): cheaper, but amplified a sealed duct up to 479x above Courant 0.3 (centred wall pressure on closed faces; a reflected Riemann pressure fixes it, test fails without it). That wall on partly open faces cost 43 % torque: not kept. At 75 mm it made 8.3 N.m at 4,000 rpm (RK2 17.8): not production. CFL 0.9: refuted.
- Real time (free-run, audio + gas view, 9,025 rpm; LS3 1.09 the same hour): RK2 50 mm 0.94-1.05 even with 2-3 pool workers (one invalid run); 75 mm + 2 workers 1.37; 100 mm without pool 1.22-1.26. A pool on the twin gave paced spikes of 25-34 ms and late audio events, even at 360 mm: not kept.
- RD350 torque, 360 -> 100 mm (N.m): 3,000 15.2->22.3, 5,000 13.8->15.5, 8,000 9.6->11.9, 9,000 9.5->10.4 (maker 37-40.2 at 8,000; 25 mm 22.7, not converged). Steady WOT torque at 1,500 rpm also rises (15.9 -> 20.1).
- Blockers, so production stays 360 mm: at 75 and 50 mm the idle hunts (std 90-138 rpm; 1-2 mm2 pilot bypass did not settle it, not explained). At 100 mm idle is fine but the launch from a settled idle stalls: the tip-in goes lean (AFR 47-50) before the clutch bites. At 360 mm the same lean tip-in (AFR 36-39) already dips it to 370 rpm (stall 220); 150 mm survives at 228.
- Not measured: a real RD350 recording; the two-stroke tip-in fuelling itself (the next lever for a finer mesh); the pool's paced spikes on other engines.

## 2026-10-15 — Every catalogue engine through the drive harness

- `EngineLabDriveScenarioHarness --builtin pull|downshift-clutch|cruise` on all 17 engines. Cruise holds 100 km/h within 0.06-0.96 km/h mean, 2.1 max; no false "injectors saturated" left. Merlin and Radial sit on a test stand (no road vehicle): they do not move, as intended.
- Bug: the Audi I5-like stalls on the `cruise` launch (60 % throttle, clutch let out linearly over 0.9 s), and the stalled car then rolled BACKWARDS at a steady 0.33 km/h, brake off, flat road. The stopped crank's friction (-6.7 Nm) went through the locked clutch as a drive torque (-71 Nm at the wheel) while the crank was held at zero. A crank at rest now passes no resistive torque.
- Second bug under it: rolling resistance switched off below 1 cm/s, so a car crossing that line crept on at that speed for ever (0.036 km/h here). It now fades with the speed below 1 cm/s while the car slows; a car pulling away keeps none there, as before. Fading it on launches too moved the 2JZ dual-clutch push by 2 % (EngineLab.ShiftStyle, push 375 -> 369 ms without retard) and failed its retard check: not kept. Both bugs in EngineLab.VehicleDynamics, failing before; dyno bit-identical.
- Not a bug, not changed: the stall itself. The I5's 900 Nm clutch asks 200 Nm at 0.2 s of a 0.9 s linear release, more than the engine makes at 700 rpm and 60 %; it also stalls from a settled idle (launch at 8 s). The scenario has no clutch modulation; a real driver's launch was not measured. The RD350 never reaches 100 km/h in `cruise` (known torque gap).

## 2026-10-15 — "Injectors saturated" was read mid-window

- The `injectors_saturated` limit read the injector's open time over the window elapsed SO FAR: a frame ending while the injector was still open read 100 %. Now judged on the last completed cycle. EngineLab.ActiveLimits: CP2 held at 6,000 rpm at full load, own injectors: no (was yes); 2 g/s injectors: yes.
- Correction to "Drive-scenario harness: first runs on the CP2" below: the "99 % of their window for 9.3 s" was this artefact. CP2 `pull` now: 0 s saturated, 0-100 km/h 3.97 s (5.86 s before the stage-E motorcycle inertias), top speed 202 km/h. A real MT-07 is quoted near 4 s; not measured here. Physics untouched (flag only).

## 2026-10-15 — Two-stroke fuelling: delivered air, no overrun cut

- The RD350 was fuelled for its trapped oxygen-equivalent air (~60 mg, mostly retained charge) while ~9 mg crossed the transfers: idle AFR 3.7, trim on its 0.55 floor, 50 % misfires, stalled in the standing start. Fuelled on last cycle's delivered air: idle AFR 13.5, trim 1.01. Four-stroke dyno bit-identical.
- IdleStability: idle 1,320 rpm (target 1,300, bypass 3 mm2; 6 mm2 gave 1,421) but 1,747 rpm 12 s after a WOT blip. Trace (`--trace --wot`): the overrun fuel cut cycled between 1.50 and 1.65 x idle for 4 s, each resume firing a clean scavenged charge (+5 N.m). No overrun cut on two-strokes: 1,413-1,430 rpm, passes.
- Fuelled overrun then taught the trim the film drain (0.64 at a tip-in, AFR 25, 141 misfire events in 2 s). Holding learning only during overrun just delayed the crash to the 0.55 floor; holding it through overrun and 2 s after: 4 events, lean peak 62 %. The DFCO-resume test reports two-strokes as n/a.
- Not compared with a real carburettor's transient; the remaining tip-in lean is the one-cycle lag of a delivered-air reference.

## 2026-10-15 — Two-stroke engines; the expansion chamber needs a finer mesh

- Piston ports, crankcase pumping (path plenum = crankcase volume, reed throttle), 360-degree cylinder phase in a 720-degree crank domain. Four-stroke catalogue dyno bit-identical. Catalogue: RD350LC-like twin (maker 37-40.2 N.m at 8,000 rpm).
- The maker's 6.2:1 is a trapped ratio; geometric 11.2:1 is used (single cylinder: 40 -> 54 J per cycle).
- Brake torque at 8,000 rpm against exhaust cell length: 360 mm 8.5 N.m, 100 mm 12.3, 75 mm 15.3, 50 mm 19.1, 25 mm 23.4 (stinger 22 mm). Single cylinder, trapped fresh charge 73 -> 136 mg from 360 to 25 mm: the chamber's suction and plugging waves need cells of 50 mm or less.
- Real-time capacity (free run, no audio) at 9,500 rpm: 100 mm 1.41, 75 mm 1.29, 50 mm 0.76, 25 mm 0.29; paced with audio, 100 mm already overruns. So the catalogue keeps the 360 mm production mesh: 10.4 N.m at 8,000 rpm with a 26 mm stinger (0.65 x header), a quarter of the maker's. Not compensated by combustion or friction.
- Second gap: combustion is late (CA50 ~30 deg ATDC at 8,000 rpm); not addressed. Outlet size and silencer restriction are inert. Ports, chambers and crankcase ratio are estimatedFamily; not compared with a recording.

## 2026-10-15 — Several turbochargers, a compressor map, an intercooler

- N turbochargers (1-4), each exhaust path's turbine face on its own shaft, the compressors solved together on one charge. EngineLab.Turbocharger: an I4 on two half-size units = one double (boost, torque within 0.2 %); a V8 with 2 + 6 cylinders per turbine: 113 vs 174 krpm, 22 % of the air through the small one, both at the common boost.
- Compressor head now from a scaled speed-line map (Moraal-Kolmanovsky, design point = wastegate ratio at the engine's flow at 0.7 x redline). Dyno (catalogue): above 2,000 rpm unchanged (wastegate holds the ratio); at 2,000 rpm EJ25 +2.5 %, I5 -1.8 %, 2JZ +0.8 %, TDI -0.5 %.
- REFUTED, not kept: an efficiency island (peak at the design point, -0.24 (phi-1)^2 - 0.25 (N-1)^2) costs at 2,000 rpm 2JZ -8.6 %, EJ25 -10.6 %, I5 -19.3 % (397 -> 320 N.m, real 480), TDI -15.9 % (347 -> 291, real 320); phi only (similarity): -7 to -16 %. Peak power unchanged. The low end is spool-limited and was already short (I5); the constant efficiency compensates a weak low-speed turbine. A turbine efficiency against U/C was not tried: its peak is the fitted value, so it can only remove more.
- Intercooler (`intercooler_effectiveness`, default 0): with 0.7 the petrol turbos make 19-29 % more torque, ABOVE their makers' figures (2JZ 445 -> 524 at 4,000 rpm, real 427; EJ25 411 -> 506, real 393; I5 473 -> 594, real 500); the TDI is fuel-limited (+3 %). IMEP per unit of trapped charge is 14.5-15.2 bar on these vs 16.4 on the K20: the missing intercooler hides a ~20 % optimism at boost (lead, unverified: knock intensity saturates, so no knock-limited retard). The catalogue keeps none authored.

## 2026-10-15 — Peak firing pressure and its limit; diesel rail pressure

- Peak pressure is now tracked per cylinder at every sub-step and published per cycle (the dyno's `peak_cyl_bar` sampled once per frame). DynoSweepHarness, whole catalogue, maximum over each 1,000 rpm row at full load: K20 116.8 bar, LS3 84.6, CP2 73.6-80.3, Hayabusa 77.1, CP4 87.9; 2JZ 107.1, EJ25 138.3, Audi I5 175.2 (3,000 rpm), TDI 231.5 (2,000 rpm).
- Pmax/IMEP at those rows: 5.1-7.3 on petrol engines, 9.9 on the TDI. Real production limits are about 120 (NA), 140 (boosted) and 200 bar (diesel): the I5 and the TDI fire above them, so their full-load phasing is probably earlier than a real calibration. Not addressed; both carry a limit 10 % over their stock peak (195, 255 bar).
- Damage above the limit grows as the square of the excess (0.05/s at 100 % over). EngineLab.PeakPressure: K20 held at full load (106.5 bar mean, 0 damage); with a limit at 70 % of that, 0.069 damage in 8 s. Stock dyno torque bit-identical.
- Diesel mixing time x sqrt(rated / rail pressure) (Chmela-Orthaber jet velocity). TDI held at full load, injector capacity held: 198.4 bar at 2,000 bar rail, 177.0 at 1,000. Stock rail = rated pressure, so bit-identical at stock. Not compared with a measured pressure trace or a noise recording.

## 2026-10-15 — The default collector was a 2 L chamber

- Owner complaint: a short collector sounds muffled on every engine. GeometrySensitivityHarness, collector volume -> swept only: CP2 +10 to +19 dB over 0.5-5 kHz, TDI +8 to +12 dB at 1.2-3 kHz. Cause: `collectorVolumeLitres` defaulted to 2.0 L, lumped as a compliance at the branch (a pre-chamber the geometry does not have), and the catalogue floored it at 0.05 L.
- Now 0 = no body: the collector holds what its trunk sweeps (legacy graph, editable network, catalogue builder). An authored body (CP3 1.7 L, CP4 2.0 L) is kept and scales with the collector on resize.
- First attempt collapsed the CFL (2JZ 64.8 -> 9.3 mm): the catalogue's 0.05 L floor, not the swept stiffness. Final: CFL identical on all 10 engines measured (CP2 44.4, 2JZ 64.8, Hayabusa 44.4 ...).
- Sound vs before (default render, 2 s skip): shape CP2 7.2, Hayabusa 12.3 (+5.8 dB level), TDI 6.9, I5 6.7, K20 4.1, 2JZ 3.8, EJ25 3.6; tilt brighter (CP2 -2.80 -> -1.25, Hayabusa -0.06 -> +3.43 dB/oct). LS3, Harley, CP4: bit-identical.
- Dyno (whole catalogue, 1,000 rpm steps): peak power -1.9 to +2.4 %, torque rows within -4.9/+3.0 %.
- Not listened to; not compared with a recording. The real stock MT-07 and Hayabusa have an under-engine box that is not authored here (could be, as on CP3/CP4).

## 2026-10-15 — Knock control as a calibration; knock intensity saturates

- The ECU's knock retard (`knockLevel x 12` deg) is now the calibration `knock.retard_deg_per_intensity` (default 12, 0 = off). EngineLab.KnockControl, 2JZ held at 0.6 x redline, full load, last 6 of 12 s: on: knock 0.35, advance 6.3 deg, 455.6 N.m, damage 0; off: knock 0.43, advance 10.5 deg, 465.4 N.m, damage 3e-6.
- Same rig with the spark trimmed +5/+10/+20 deg: knock 0.42/0.47/0.47 (on), 0.47/0.47/0.47 (off); torque still +2 to +5 %; damage at most 4.4e-4 after 12 s (0.5 "damaged" would take hours). The intensity (`severity x sqrt(unburned)` at auto-ignition, mean-chamber conditions) saturates just above the 0.45 "severe" threshold whatever the advance.
- So knock consequences were NOT strengthened: a damage rate or a heat-transfer penalty tuned on this flat scale would be calibrated on a non-physical signal (onset changes are refuted, see the rules). An over-advanced map is not punished as a real engine would; next lead, unverified: the intensity's dependence on advance, not its onset.
- Not compared with a real knock trace; a real ECU retards in steps per event and recovers slowly, not modelled.

## 2026-10-15 — CVT, car and scooter

- A variator inside the clutch model: ratio interpolated over the 1 ms steps, the stick constraint follows its rate and takes slip out over 20 ms. Fixed-ratio gearboxes bit-for-bit unchanged (start = end ratio, added terms zero).
- Toy engine (EngineLab.Cvt, 200 Nm, 1,300 kg): the scheduled speed is held within 0.0 %, no step over 0.0 rpm/frame; 30 % pedal 2,510 rpm (centrifugal 2,717), flat out 6,120; launches and braked stops never under 0.85 x idle; the scooter's centrifugal clutch idles free, the car's creeps.
- K20 switched to CVT (its 1st-6th range), DriveScenarioHarness `--builtin pull`: held 7,396 rpm from 6 s to top speed, slip 0-1 rpm; 0-100 km/h 5.80 s (its manual 6.19 s), 235.5 km/h. `--builtin cruise`: 100 km/h held, mean error 0.59 km/h.
- Not compared with a real CVT's engine-speed trace; the 1.5 s ratio rate, the pedal schedule and both parts' ranges are typical values, not a unit's. No scooter engine in the catalogue yet.

## 2026-10-15 — Motorcycle rotating inertias were a car's

- The three motorcycle gearboxes inherited the car defaults (wheels 3.6, differential 0.16, input 0.08 kg m^2): 234 kg of equivalent mass in first on a 260 kg bike. Now wheels 1.2 (cruiser 1.6), chain 0.02 (0.03), clutch basket 0.004 (0.008) referred through the primary: 23 kg (28).
- DriveScenarioHarness `--builtin pull`, 0-100 km/h before -> after: CP2 5.86 -> 3.91 s (published 3.85 s, motostatz), CP3 4.17 -> 2.75, CP4 3.53 -> 2.38, Hayabusa 2.90 -> 1.95, Big Twin 4.52 -> 3.92. Top speeds unchanged (+/-0.6 km/h).
- CP3, CP4 and Hayabusa are now quicker than published (about 3.0-3.5 s): a real one is wheelie- and grip-limited and the model has neither by default. Next gap, not addressed.
- Car gearbox input inertias (0.07-0.12 kg m^2) look high too; not changed, not measured.

## 2026-10-15 — Dual-clutch upshift spark retard (timeboxed)

- Retard of up to 20 deg while the oncoming clutch pulls the engine down, released with the slip (DriveScenarioHarness `--dct-retard`, 2JZ dual clutch, flat-out pull, 0 vs 20 deg). The advance reaches the ECU's -10 deg floor on every loaded upshift (control: 4-7 deg).
- Sync time: 237.5 -> 225, 216.7 -> 208.3, 216.7 -> 204.2, 237.5 -> 200, 308 -> 238 ms (gears 1-2 to 5-6). Peak wheel torque unchanged (6,471, 4,634, 3,592, 2,742, 2,137 vs 2,151 Nm): the clutch sits at its 760 Nm capacity through the pull-down either way, so the retard only shortens the push (-4 % impulse at 85 % of redline, EngineLab.ShiftStyle).
- EGT peak +1 to +3 degC: the retard window (0.2 s) is shorter than the EGT lag.
- Kept (no catalogue engine uses a dual clutch, default sound unchanged). Not listened to: whether the late burn gives the "DSG fart" is the owner's ear. No production retard calibration found to compare.

## 2026-10-15 — Cruise control and the automatic shift schedule, driven

- `DriveScenarioHarness --builtin cruise --automatic 1` (60 % pedal, set at 100 km/h, brake at 40 s). First version: the 2JZ ran to 116 km/h, because the integral started at the 60 % accelerating throttle and bled at 0.03/s per m/s. Added: past the set speed and still gaining, the throttle falls at 0.5/s (and the mirror case). Toy-car test: peak 111.1 -> 103.0 km/h.
- After, the gearbox shifting itself (the scenario's scripted rpm upshifts are now skipped under `--automatic`): peak 106.7 (2JZ), 107.9 (CP2), 101.6 (TDI), 108.6 km/h (LS3); then mean |error| 0.67 / 0.05 / 0.22 / 0.87, max 1.42 / 0.70 / 0.40 / 2.05 km/h. Gains are choices, not fits to a real controller.
- Upshifts at 60 % pedal, one gear at a time: 2JZ 4,601-4,768 rpm, CP2 5,994-6,032, TDI 3,263-3,427, LS3 4,068-4,507; under cruise the LS3 lifts up to sixth at 2,308. Dual-clutch 2JZ test: flat out 6,581-6,769 rpm (limit 7,000), 25 % pedal 2,660; braked from sixth to a stop, back in first, lowest 629 rpm.
- Reference: MT-07 0-100 km/h 3.85 s (MotoStatz), 0-60 mph 3.80 s (MCN 2014); EngineLab CP2 5.86-6.35 s. Not changed yet (plan E5).

## 2026-10-15 — Standing starts after the driver's clutch took over the shift

- `EngineLab.StandingStartSettledIdle` failed after B1: the I5 stalled at 25 s. Cause, one factor: a gear selected with the clutch held now engages at once, so the scripted 0.9 s pedal ramp loaded the engine from t = 0 (before, the timed selection held it out for 0.18 s). I5, 8 launch times: 2 stalls, lowest 241-294 rpm; ramp held at 0 for the first 0.18 s: 0 stalls, 383-462 rpm.
- Fix in the scripted driver, not the driveline: the pedal stays down for the gearbox's shift duration, then ramps over 0.9 s. EJ25, Aircooled, I5, Big Twin over 8 launch times each: 32/32 launch, lowest 595 rpm (Big Twin). The whole catalogue launches at 4 and 25 s, LS3 included: the three declared stalls are removed.
- Not measured: a human's real clutch timing; the launch is gentler than before, so these tests no longer cover a clutch dumped from the first frame.

## 2026-10-15 — Drive-scenario harness: first runs on the CP2

- New `EngineLabDriveScenarioHarness`, through the product's coupling (`advanceVehicle`). The shift, loaded-acceleration and afterfire harnesses now use it too: before, they skipped the cooling air, and two of them skipped the quickshifter cut and the blip. Their ctest gates still pass.
- CP2 `pull` (launch, upshift at 9,000 rpm): measured engine/road ratio within +0.13 to +0.37 % of the authored ratio in every gear (clutch slip, tyre). Manual gearbox: same within 0.4 % (gear 3 -0.05 %). **"The quickshifter changes the gear spacing" does not reproduce on the catalogue CP2.** Still open: an engine or setting the owner used.
- Same run: 0-100 km/h 5.86 s from first motion (manual 5.85), top speed 204 km/h. Injectors open for 99 % or more of their window for 9.3 s of the 45 s. Knock retard of 0.5° or more for 3.6 s. Nothing compared with a real MT-07 yet.
- CP2 `downshift-clutch` with `--blipper 1`: the blipper opened the throttle for 93 frames (0.39 s) with the clutch pulled. Bug confirmed, to be fixed in B1.
- Not measured: any other engine; whether the 5.86 s is a driveline-inertia problem (plan E).

## 2026-10-14 — Blind listening of the valve noise and the IR, and the fine-mesh question

- Owner, blind, one trial per pair, CP2 Full System clips (`--valve-noise`, `--impulse-response`). Valve turbulence noise: no preference at idle; on the rev the control was preferred ("deeper, less muffled"). It stays off.
- Stairwell IR at the default convolution 0.45: no difference heard at idle or on the rev, as its 22 dB margin under the dry sound predicts. Free field stays the default. Not tested: a fully wet chain.
- Earlier blind listening, recorded here: the owner preferred the rev without the output stage (`--output-stage`). Not adopted.
- A private mesh ablation of the other simulator (owner's captures, kept out of the repo) found its default mesh not converged in the 1-2 kHz band, and no sign that a finer mesh is what the owner hears. The 4 cm lockstep prototype is not pursued.
- Same analysis on EngineLab CP2 at 3,148 rpm, steady part only (a render starts with a ~5,500 rpm transient 15 dB louder: skip the first 4 s): energy above 2 kHz -50 dB of total, tilt -9.8 dB/oct, crest 10.8 dB, cycle-synchronous share 93.7 %.
- Merlin spike: `EngineLab.AudioRender` (runtime wiring) failed 4 times in ~17 runs on this branch: Merlin at 3,106 instead of 3,040 rpm, observer up to 4,092 Pa. On `main` (c423400) its 2nd run failed the same way (3,090 rpm, preLimiter 1.53). Pre-existing; cause found 2026-10-17.
- Not measured: anything against a real CP2 recording, which remains the arbiter.

## 2026-10-13 — Valve turbulence noise, and the first IR clips

- The other simulator's turbulent valve-area modulation (4 RBJ low-pass sections, f_c = clamp(0.35 v/d, 100, 3,500) Hz, depth 0.3, collapse on fast opening) re-expressed as `ValveTurbulenceNoise`: a Norton mass-flow source beside each exhaust valve's high-band source, own noise per valve. Its tests reproduce the analysis's measured deviations (1.0 / 3.0 / 5.7 %) and fail with 2 sections or a 10x smaller collapse. Off by default; `--valve-noise-ab`.
- Seven points: level -0.03 to +0.04 dB; shape 1.05-3.47 dB from valleys filled between harmonics (worst band +4.0 to +9.4 dB, anywhere from 25 Hz to 2 kHz); crest +0.2 to +1.2 dB; cycle-to-cycle COV up (LS3 1.2 -> 5.0 %, CP2 Full System 4,000 rpm 0.8 -> 1.8 %); periodicity -0.07 at most (LS3); tilt within 0.9 dB/oct.
- Blind clips, CP2 Full System (`AbClipRenderer --valve-noise`): the noise is 21-22 dB under the clip (correlation 0.996). Offline renders are deterministic (two controls bit-identical), so that is the noise alone.
- IR: no catalogue engine authors one, so the application plays in free field. The owner's stairwell IR (provenance unknown, not in the repo) on every path at the default convolution 0.45 is 22 dB under the dry sound; its 0.6 s cut is 60 dB from the 3 s file. Clips: `--impulse-response`.
- Not measured: realtime cost with the noise on (per-sample trigonometry per valve), intake valves, any listening result.

## 2026-10-13 — Ambient far-field density by default, monitor full scale 140 dB

- Owner-approved sound change: the outlets now radiate with the live ambient air density (p / R T from the runtime's ambient state); `setOutletFarFieldDensity(0)` restores the duct medium. Catalogue at 88 % of redline: +2.5 to +11.7 dB, the CP2 least (coolest gas).
- At the old 134 dB full scale the CP3 (~210 Pa peak) and the Merlin drove the safety limiter. Full scale moved to 140 dB (282.8 Pa peak): every catalogue engine stays under the 0.82 knee except the Aircooled, whose spikes (crest ~24 dB) were there before (29 soft-limited samples and 1 clamp at 7,166 rpm, against 377 / 32 before the change). The CP2 keeps roughly its old on-screen level.
- `EngineLab.AudioRuntimeDiscontinuity` (Merlin, 80 ms stall) failed in 2 of ~26 runs: the engine took another trajectory (3,116 instead of 3,040 rpm) and the observer peaked at 454 Pa (preLimiter 1.63; 117 Pa and 0.35 otherwise). Physics does not read the audio, and the duct density is at most 4.9 dB under ambient there (measured, Merlin 3,000 rpm), so the old calibration would have put that peak at ~1.85: a pre-existing intermittent spike, not caused by this change; cause found 2026-10-17.
- Not measured: absolute SPL against a calibrated CP2 recording (the MT-07's stationary-noise homologation figure is the cheapest check).

## 2026-10-13 — Pressure-release outlet versus the Levine-Schwinger load

- The other simulator terminates its pipes with an imposed pressure and no radiation load (R = -1 at every frequency). New switch `setOutletPressureRelease`, off by default (renders byte-identical); `GeometrySensitivityHarness --pressure-release-ab`, same seven points.
- Level: -0.04 to +0.42 dB. Shape 0.47-3.45 dB. Worst band: +7 to +10 dB at 5-10 kHz on CP2 Full System, K20 and LS3; +1.5 to +2.5 dB at 1.25-2 kHz on CP2, Hayabusa and 2JZ. The tailpipe keeps the resonances that the load would radiate away. Crest -0.1 to +0.9 dB, tilt up to +0.9 dB/oct, periodicity unchanged except the 2JZ (0.77 -> 0.72).
- Verdict: it moves the sound away from the other simulator's character (crest rises), and only above the plane-mode cutoff, where a 1-D pipe means nothing physical. Kept off; only a real CP2 recording could argue otherwise.

## 2026-10-13 — Outlet far field: duct density versus ambient air

- The outlet monopole is p = rho dQ/dt / (4 pi r) with rho = the path's duct medium (hot gas). Outside the mouth the field is in ambient air and Q is continuous across the interface, so the textbook rho is the ambient one. New switch `setOutletFarFieldDensity`, off by default (renders byte-identical); `GeometrySensitivityHarness --radiation-density-ab 1.2`.
- Same seven points as the output-stage entry, same render otherwise. Exhaust level: CP2 Full System +4.0 dB at 4,000 rpm and +4.5 at 7,000; CP2 +4.4; K20 +8.2; 2JZ +9.4; LS3 +9.9; Hayabusa +10.0. That implies a duct density of 0.4-0.75 kg/m3, i.e. a mean path medium of ~490 K on the CP2 and 740-930 K on the others. Leveler never engaged.
- Shape 0.9-4.1 dB, but not from the physics of the main band: on the CP2 at 6,000 rpm, 50 Hz-1.6 kHz moves as one block (+-1 dB), and above 2 kHz it rises 1.5-4.4 dB less. That is the outlet jet noise (own jet density, not switched) taking a smaller share. Crest, periodicity and tilt are unchanged within 0.6 dB, 0.01 and 0.35 dB/oct.
- At equal loudness, then, this is mostly a balance change against the jet noise, plus a level that grows with gas temperature. It does not explain the other simulator's character. Not changed by default: it is a sound change and needs the owner's go-ahead. The absolute SPL is still unchecked against any calibrated CP2 recording.

## 2026-10-13 — How much character an output stage alone gives

- Spec from a static analysis of another simulator, whose sound the owner prefers: after its room convolution it ends in a level-normalising gain (5 Hz envelope, 4x attack, 1 s hold, gain <= 10) into a tanh, on by default. Its August 2026 measurement gave crest 7.4 dB, periodicity 0.92, tilt -4.6 dB/oct (archive). Re-expressed as `AutoGainSoftClipper` (off the default path).
- Step 0, today's EngineLab at steady wide-open throttle (`GeometrySensitivityHarness`): periodicity 0.77-0.98, crest 7.8-14.5 dB, tilt -4.9 to +2.3 dB/oct. The August EngineLab figures (crest 18-25, periodicity 0.38-0.50) are stale.
- Step 1, `--output-stage-ab`, same render with the stage at +40 dB drive (at 0 dB the gain sits on the x10 cap, so the stage is just a gain; shape 0.09-1.72 dB). Crest raw -> staged: CP2 Full System 4,000 rpm 11.2 -> 6.2, 7,000 rpm 14.5 -> 7.0; CP2 6,000 11.3 -> 6.3; K20 6,000 9.5 -> 6.1; LS3 4,500 11.3 -> 6.3; Hayabusa 7,000 7.8 -> 5.4; 2JZ 4,500 11.1 -> 6.4. Band shape moves 1.3-3.4 dB; periodicity and tilt are almost unchanged.
- So the other simulator's low crest factor can come from its output stage alone, not from its gas dynamics. Blind clips: `EngineLabAbClipRenderer --output-stage "Full System"`.
- Not measured: any listening result, either simulator against the owner's CP2 recordings, transients (idle, rev) beyond the clips, or the other steps of that spec (room response, outlet radiation, fine mesh).

## 2026-10-13 — A turbo fitted to any engine

- `sizedTurbocharger()`: laws fitted to the four catalogue turbos (turbine 0.028 mm² per L per redline rpm, within 10 %; inducer within 15 %). Boost by compression: 0.5 bar from 10.5:1, 0.7 from 9.5, 0.9 below.
- `EngineLabDynoSweepHarness --add-turbo`, 1,000 rpm steps, every engine without forced induction: peak power +38 to +85 % (CP2 54 -> 80 kW, K20A 166 -> 229, LS3 315 -> 440, Aircooled 198 -> 368 at 0.7 bar), lambda 0.84-0.90, boost at target except the Big Twin (1.79 for 1.66: no collector, so the older outlet turbine, whose work is overstated).
- `EngineLabThermalHarness --add-turbo`, CP2, 3 passes: 61.5 -> 92.1 kW, knock mean 0.30 (heavy 0 %), no damage; the ECU retards from 30.5 to 24 deg.
- `EngineLabRealtimeBudgetHarness --free-run --relative-rpm 0.8`, alternated: a fitted turbo costs 0-5 %; lowest LS3 1.24 (1.27 without).
- `EngineLab.TurboFit`: CP2 at 7,000 rpm, torque x1.41 for a charge density x1.28, lambda 0.88, peak knock 0.41.
- Not measured: spool against a real kit, any real turbo conversion's curve. The LS3 and Big Twin turbines are not drawn; the Aircooled's second bank turbine is not drawn.

## 2026-10-13 — Shift styles: quickshifter, blipper, dual clutch

- `EngineLab.ShiftStyle`: real engine, ECU and driveline coupled as the runtime couples them; 1 -> 2 at WOT, 70 % of redline. Gap = time the wheel torque stays below half the new gear's.
- CP2 manual (lift + clutch, 0.18 s): gap 54 ms, synchronised at 204 ms. Quickshifter (65 ms ignition cut, clutch in): cut 62.5 ms (240 Hz frames), spark really off, gap 21 ms, synchronised at 67 ms, 0.3 % off the gear's speed.
- 2JZ manual: gap 62 ms, sync 379 ms. Dual clutch: no gap (wheel torque never below the new gear's), sync 200 ms; no torque reduction during the inertia phase, so the wheel gets a push.
- CP2 3 -> 2 on a shut throttle: blipped, engaged at 0.988 of the gear's speed, slip 35 rpm; through the clutch, 0.756 and 1,517 rpm. Disabling the styles fails the test.
- Not measured: any real quickshift or dual-clutch shift trace; the 15 ms dog change is a guess. Wet cut pops depend on the CP2 afterfire (nothing burns at 800 K).

## 2026-10-13 — Stalls after a blip: three ECU causes

- `EngineLabPhysicsPerfHarness --freerev 1 --release S --settle S` (new `--settle`, fuel and air columns). 2JZ, warm, WOT blip then release: down to 516-548 rpm (760 target). After the fuel-cut resume the charge reference ran 20-30 % above the trapped air for ~1.5 s (AFR 10-12); lambda learning took the idle trim 0.81 -> 0.57 in 2 s, then the refilled manifold ran AFR 17.4, 30 % misfires. Learning now holds 2 s after a resume.
- That exposed a hung idle: the dashpot was held above 1.3 x idle, so a fuelled EJ25 sat at 1,200 rpm on its air for good (CP4 1,814 and TDI 1,161-1,288 already did in the base). Held now only while coasting down (losing > 5 % of idle per second).
- Blipped 6 s after the start the fuel cut was inhibited (after-start phase): the 2JZ coasted from 6,800 rpm fuelled at lambda 0.27, 90 % misfires; LS3, Merlin, Radial stalled. Opening the throttle past 0.2 now lifts that inhibit, as for afterfire engines; ending the whole start phase also stopped its lambda learning and moved the default renders (EL-20 dyno 3,820 -> 4,843 rpm). Renders now unchanged except the Big Twin rev return (771 -> 776 rpm).
- Catalogue, WOT 0.5 / 1.2 s after 6 or 30 s of idle: base 6 stalls, worst warm 0.67 x idle; all three: no stall, lowest 0.83 x idle (Aircooled), idle back. Each change alone left a stall (Radial, LS3). `EngineLab.IdleStabilityRegression` gained these blips; base fails on 12 engines.
- Not reproduced in the application itself, nor with a gear engaged.

## 2026-10-13 — Knock model checked: not physical, not changed

- Trace of every auto-ignition, one dyno pass: 2JZ 2,029, K20A 1,877, CP2 875 events, at 50-95 % burned. The integral reads the mean chamber temperature (1,850-3,070 K, clamped to 1,500) where the end gas is the unburned charge, isentropic from the flame start: 830-1,200 K. The delay fit is 2-5 x slower than Douaud-Eyzat (18 vs 8.2 ms at 850 K, 20 bar, RON 95); the two errors offset.
- Isentropic end gas alone: no knock on any engine (peak power -0.7 to +1.1 %), none even at RON 80. Plus Douaud-Eyzat: CP2 clean at 98, light knock from 95 (realistic); 2JZ and K20A knock 0.33-0.44 whatever the octane, still 0.2-0.36 with an integral knock control at its 12 deg stop: their end gas (1,000-1,200 K) is hotter than measured engines at knock onset (~850-950 K). The charge temperature is the next suspect, unverified.
- Kept as it was: light knock on every engine at WOT (mean 0.12-0.37), no damage since the earlier fix, held by a proportional retard (`knockLevel * 12`).

## 2026-10-13 — Air-cooled oil thermostat; why turbo engines overheated

- Big Twin ran at 34-47 degC on the dyno: its fins were always fully open, so its temperature scaled with load. Air-cooled engines now get a tenth of their fin cooling always and the rest through an oil thermostat (85 -> 100 degC). Realistic, cell fan, 5 passes: Big Twin 42 -> 83 degC, 74 after 60 s of idle, 89.6 on a ride; Aircooled 48 -> 90, ride 87. `EngineLab.LiveTuning`: Big Twin cruise 87.5, flat out at 30 m/s 97.3 degC.
- The owner's overheating (turbo engines, hot by pass 5, big loss at 8-10) reproduced with the former free model from 3 min of warm-up: 2JZ +8 degC per pass, 32.7 -> 100.3 degC at pass 10 (EJ25 90.7, I5 99.8): heading for the 108 degC derate and 115 degC damage. New model, same run: 2JZ, EJ25, I5 level off at 92.9-93.5 degC (oil 105-110).
- Power still drifts -1.2 to -1.9 % over 10 passes. Probe at 80 % of each ramp: boost, AFR, advance and knock constant; exhaust wall 154 -> 608 degC (2JZ), 143 -> 538 (CP2), still climbing; VE -0.9 %, torque -1.2 %. Heat soak of the pipes, physical. Harness renders all start with cold pipes.
- Not compared with any recorded air-cooled temperature or dyno-session power trace.

## 2026-10-13 — Realistic temperatures (EngineThermalModel)

- The former free model stalled cold: a car-sized 180 kJ/K on every engine and radiator air through a closed thermostat (CP2 36.8 degC after 5 passes). Now: capacities from displacement, a thermostat (82-94 degC), air of the vehicle's speed (12 m/s of cell fan on the dyno), a fan on at 100 / off at 95 degC, all friction heat into the oil, heat shares 24 % / 5 % (Heywood, coolant 17-26 %). Sizes are plausible, not measured.
- `EngineLabThermalHarness --realistic`. CP2 in still air, idle: 42 / 59 / 73 / 85 degC at 5 / 10 / 15 / 20 min (simulated idle ~2.8 kW released). With the cell fan, 5 passes from cold: CP2 81 then 94 degC (oil 113), K20A 62 -> 94, LS3 49 -> 91; 60 s of idle after: ~80; a 60 s ride at 25 m/s, half redline, quarter throttle: 86-87 degC. Peak power within -1.8 % (CP2 pass 2, run-to-run spread; no damage).
- Air-cooled Big Twin with the cell fan: only 34 -> 47 degC over the passes, 29 degC after 60 s of idle: probably too cool; its fins are sized like a radiator. Not changed.
- Not compared with any recorded warm-up or temperature trace. Default (regulated) behaviour and sound unchanged.

## 2026-10-13 — Warm standing starts: three engines with no margin

- `EngineLab.StandingStartSettledIdle` (launch at 25 s, 1st gear, WOT, clutch over 0.9 s) failed once engines ran regulated at 90 degC: the EJ25 stalled. New `--unregulated-thermal` flag in `EngineLabLoadedAccelerationHarness` for the one-factor comparison.
- EJ25, 41 launch times from 6 to 34 s: 9 stalls warm, 0 free (cold, ~25-30 degC). Catalogue, 8 launch times each: warm EJ25 0/8, Audi I5 1/8, Big Twin 2/8; free 0 everywhere. All three already sagged to 240-440 rpm cold.
- Trace (EJ25, 25 s): the first cycle after the tip-in gets 67 % of its metered fuel (the late part of the extended pulse stays in the port), AFR ~18 cold and ~19.7 warm; at 19.7 the misfire chance is 45 %, two misfires in a row and the clutch drags it under 220 rpm. Min AFR after: ~10 cold, ~11.2 warm, consistent with the lost warm-up enrichment (x1.09-1.12). Not proven to be the only difference.
- Declared as a known stall (EJ25), not compensated. Stalls are the next step, with the owner's scenarios; an asynchronous tip-in pulse is refuted cold and would need new evidence warm.

## 2026-10-13 — Light knock no longer wears the engine

- `EngineLabThermalHarness` now prints knock per pass. At WOT every engine knocks lightly through the whole pass, held there by the ECU's proportional retard (`knockLevel * 12` deg): mean CP2 0.14, 2JZ 0.27, LS3 0.37; above the 0.45 "severe" threshold 0 % of the pass (LS3 0.2 %).
- Damage counted that light knock linearly: +0.003 to +0.005 per pass, a permanent 0.82 x damage efficiency loss. Now only knock above 0.45 counts, scaled so full knock keeps its rate. 2 passes: damage 0.006 -> 0 (CP2), 0.011 -> 0 (LS3). `EngineLab.LiveTuning`: 2JZ at full load 8 s, knock mean 0.34, damage 0 (0.0029 without the fix).
- Sound: within the first pass the old damage reached 0.003, at most -0.02 dB by efficiency (computed, not rendered). The ECU's knock control itself is unchanged: a real one retards in steps and holds, it does not sit at constant light knock. Not measured against a real knock trace.

## 2026-10-13 — Engine temperature over a dyno session; regulated by default

- New `EngineLabThermalHarness`: idle, 5 app-like dyno passes (entry to ceiling, 500 rpm/s), 20 s of idle between, 120 s of idle after. Deterministic.
- Before, every engine started at 22 degC (car-sized thermal mass, 180 kJ/degC, on all but Merlin and Radial). After 5 passes: CP2 36.8, K20A 54.2, 2JZ 65.2, LS3 77.7 degC (highest of 16); 120 s of idle then cools 0 to 2.9 degC. The owner's "excessive after 5 passes" is NOT reproduced from a cold start; harness renders were all of a cold engine.
- Power over 5 passes: -0.0 to -1.2 % (Lab twin -4.3 %), cold and warm alike. It follows `damage`, +0.003 to +0.006 per pass from end-gas knock at WOT (`knockLevel * throttle * 0.0012`), which lowers efficiency by 0.82 x damage for good. Fixed the same day (light knock no longer counts).
- Now `thermal.regulated` (default): coolant 90, oil 100 degC from the start. Sound, cold -> warm: CP2 at 6,000 rpm +0.32 dB, shape 0.57 dB; AudioRenderHarness catalogue within +0.24 / -0.62 dB (EJ25), brightness within 0.009. Not compared with a recording.

## 2026-10-13 — Wastegate holds a gauge boost

- The boost controller held `wastegate_pressure_ratio` as a ratio to ambient: up high it lost as much manifold pressure as an unblown engine. It now holds the sea-level gauge boost, like a spring against the atmosphere. Bit-identical at 101.325 kPa (the added term is exactly zero).
- `EngineLab.LiveTuning`, 2JZ held at 60 % of redline, pressure only: gauge boost 90.19 kPa at sea level, 70.76 -> 90.19 kPa at 79.5 kPa; brake torque at 79.5 kPa 358.5 -> 382.8 Nm (sea level 457.4). The new check fails without the fix.
- 2,000 m and 35 degC: 2JZ 455 -> 339 Nm before, 455 -> 362 Nm now (-20 %). Not compared with a measured altitude curve; turbo overspeed up high is not limited.

## 2026-10-13 — Do the cams change the sound?

- Owner report: editing valve lift/duration changes the engine but not the sound. `EngineLabGeometrySensitivityHarness --cam-duration-delta D --cam-lift-scale S --wav-out`, then `--wav-reference`, CP2 held at 6,000 rpm, 0.85 throttle, every cam (low and high profile).
- Cams -40 deg and lift x0.7 (CP2 torque -22 % in LiveTuning): level -0.74 dB, shape 4.07 dB, worst band -11.8 dB at 315 Hz. Cams +40 deg, lift x1.2: -0.69 dB, shape 2.38 dB, worst -9.1 dB at 200 Hz.
- Same run, exhaust variants for scale: no muffler body 2.70, long primaries 4.08, independent pipes 12.0 dB of shape. The cams do reach the sound, as much as a modest exhaust change; the level barely follows the torque.
- Not measured: idle (a long-duration cam's lope), light load, any real recording of a cam change. The harness's default `CP2` filter had become ambiguous (two CP2 engines); it is now `Yamaha CP2`.

## 2026-10-12 — Live flywheel, camshafts and air

- `EngineLab.LiveTuning`, deterministic, built / swapped / untouched engines. Flywheel x2 taken at idle: free rev to 75 % of redline CP2 0.938 / 0.925 / 0.525 s, 2JZ 1.404 / 1.421 / 0.842 s.
- Cams 40 deg shorter, 30 % less lift, taken at 6 s at 60 % of redline: CP2 55.16 / 55.14 (55.13 over 2 s) / 70.84 Nm, K20A 142.2 / 142.1 / 222.6 Nm. Each fails without its copy into the running configuration.
- Air of 2,000 m (79.5 kPa) and 35 degC: CP2 51.6 / 51.5 / 70.8 Nm (-27 %), 2JZ 338.7 / 339.3 / 454.7 Nm (-25 %). The turbo's boost target is a pressure ratio to ambient, so it does not hold manifold pressure up high as a gauge-referenced wastegate (about -11 % MAP) or an absolute-MAP controller would. Not changed; not compared with a measured altitude curve.
- Gearbox: K20A driveline, first gear, engine held at 2,000 rpm, final drive x1.3 at 1 s: 3.093 / 3.085 / 3.997 m/s.
- Not done: the pipes' outer walls keep the built ambient until a restart; the 3-D edits were not driven by hand.

## 2026-10-12 — Turbo mid pipes, behind the turbine

- `turbo_downpipe` (2JZ, I5, TDI) and the EJ25's `compact_sport` get the K20A's 2.2 m mid pipe: routes ~1.2 -> ~3.4 m. Same binary, catalogue copies with and without, deterministic harnesses.
- Spool t90 (`TurboDownstreamAuthorityHarness --spool`), 3,000 rpm: 2JZ 2.263 -> 2.321, EJ25 2.358 -> 2.392, I5 1.979 -> 1.887, TDI 0.979 -> 0.979 s; 2,000 rpm within 2 %. `DynoSweepHarness`: within 1 % from 3,000 rpm up (top end -0.6 to -1 %, back pressure +4 to +7 kPa), EJ25 -2.4 % and I5 +8.5 % at 2,000. CatalogReference: every gap within 0.5 point (2JZ power +11.8 -> +11.4 %, I5 -6.4 -> -6.7 %). CatalogBoostHold passes (swing <= 6.7 %).
- Realtime free-run, alternated, minimum: default point 2JZ 1.67 -> 1.79, EJ25 2.08 -> 1.83, I5 1.91 -> 1.86, TDI 2.24 -> 2.03, K20A control 1.72 -> 1.79; 95 % of redline with audio 2JZ 1.49 -> 1.35, EJ25 1.64 -> 1.60, I5 1.56 -> 1.44, TDI 2.11 -> 1.88. `--runtime-blip`: lowest 0.992, worst lateness 1.8 ms (without: 0.994, 8.6 ms).
- Sound, `AudioRenderHarness` catalogue point: RMS 2JZ +5.8 dB (brightness 0.138 -> 0.086), I5 +3.1, EJ25 -2.1 (0.096 -> 0.132), TDI -4.0 dB (0.080 -> 0.127); K20A control identical. One held point each: not a level over the range, and not compared with any recording.
- `TurboDownstreamAuthority` (tailpipe sweep 45-95 mm, 2JZ): behind the mid pipe 76 -> 95 mm reads +0.5 kPa (tolerance 0.5 -> 1.0), and the raised-target stress holds 3.42 even at 45 mm (gate 0.003), so its authority is now the gate (0.003 -> 0.27) when boost cannot rise. A turbo-back sweep (mid pipe at the tip's bore) was rejected: a 95 mm pipe between 64 mm sections raises back pressure, the K20A control's too (101.0 -> 107.4 kPa).

## 2026-10-12 — Mechanical layer off by default

- Owner's listening: the layer adds nothing and an unpleasant sound; always switched off. Every voicing `mechanical_gain` (0.66-1.18; CP2s already 0) and the code defaults (0.70) -> 0; the fader stays. The starter leaves the mechanical bus for a fixed 0.70 (`EngineLab.RealtimeRegression`: starter RMS 0.0484 at fader 0 and 2; before, 0 at fader 0).
- `AudioRenderHarness` catalogue point, old binary + old voicing vs new: RMS -5.7 dB I5, -5.3 2JZ, -2.8 Hayabusa, -1.2 LS3, -0.9 K20A, -0.8 Merlin, -0.4 EJ25, -0.3 Radial and APL, -0.2 CP4, 0 to +0.1 Big Twin, Aircooled, CP3, TDI, CP2s. Brightness falls on every engine that changes (2JZ 0.177 -> 0.138, K20A 0.180 -> 0.081): at these points the layer was mostly upper band, not low. The starter is now audible while cranking where the voicing had 0 (CP2 peak 0.095 -> 0.115 in the first 1.3 s).
- The layer was compensating: the synthetic EL-R5 radial goes from 48 % to 0.9 % of its energy above 250 Hz without it (catalogue Radial 5 % either way). The synthetic fixtures keep 0.70 so the harness still checks the renderer with every layer.
- Not compared with any recording. Not measured: what the owner hears as "low" (the harness shows the layer bright at these points).

## 2026-10-12 — Live connecting-rod length

- `resizeConnectingRods` moves the deck with the rod, so the clearance volume and the compression ratio stay; the live resize now carries the rod (the kinematics already read it live). `LiveCylinderResize`, rod + 0.3 stroke, 60 % of redline, brake torque over the last 3 s: CP2 115 -> 135.6 mm, built 71.50, at once 71.80, over 2 s 71.75, untouched 70.74 N.m; K20A 139 -> 164.8 mm, built 223.96, at once 223.93, ramp 223.92, untouched 222.73. Without the simulator carrying the rod: 70.74 (fails). The longer rod gains ~0.5-1 % at that point; not compared with any measurement.
- Not done: the rod keeps its mass (a real longer rod is heavier); the structural radiator keeps the built rod length until a restart. The 3-D edit was not driven by hand.

## 2026-10-12 — TDI boost controller limit cycle at full load

- Owner's report: the TDI "misfires" at WOT from 2,500 rpm, the dyno curve full of quick dips. `DynoSweepHarness`: held brake torque swing 13 % at 2,500, 32 % at 3,000 rpm (base before the turbine move: 3-7 %); other turbos within 9 %. No misfire: `PhysicsPerfHarness --trace 3000 --watch` (new turbo columns) shows a 4 Hz cycle, gate 0 -> 0.45, back pressure 148 -> 239 kPa, turbine 2 -> 16 kW, cycle torque 322 -> 352 N.m.
- Cause: the proportional gate (fully open 0.08 above its ratio) against a turbine that now loses its inlet pressure as soon as the gate opens: loop gain ~30 on a 4e-5 kg.m2 shaft. A gate lag of 0.1 / 0.3 s made it worse (swing 302-358 / 292-356 N.m); band 0.25 alone settled it (339-346) but droops.
- Fix: PI, band 0.25, integral 0.5 s, anti-windup. Swing <= 6 % on all four turbos (TDI 4 %); boost exactly at the gate ratio (2JZ 1.890, EJ25 2.050, I5 2.450, TDI 2.250); spool t90 within 1 %; TurboDownstreamAuthority boost 1.890 at every outlet. Sound vs the previous build: RMS -4 to +1 %. CatalogBoostHold now fails a 15 % swing (P gate: 30.5 %).

## 2026-10-12 — Turbine on the collector face, in the gas network

- The turbine is a restriction face on the collector's outgoing face (orifice from the collector's stagnation state; turbine share of the flow gives up `eta cp T0 (1 - (p_out/p0)^k)`, removed from the downpipe's energy; no jet on the receiving side). `EngineLab.RestrictionFace`: choked junction face -0.2 % vs isentropic, work 37.7 kJ/kg under its 37.9 ceiling, energy closed to 3e-12, cooling 33.9 K vs w/cp 37.5 K, half share 0.50 of the work. With the jet kept on the receiving side: work over the ceiling, half share 0.66.
- At the old areas (700 mm2) the spool at 3,000 rpm (`TurboDownstreamAuthorityHarness --spool`, t90) went 1.09 -> 3.60 s (2JZ), 1.69 -> 5.29 (EJ25), 1.98 -> 5.55 (I5): the old outlet model drove the shaft from the peak exhaust pressure, and the areas had been fitted to it. Re-fitted to the rated curves (`DynoSweepHarness`), not measured: 2JZ 600, EJ25 500, I5 450, TDI 320 -> 280. t90 then 2.25 / 2.34 / 1.98 / 0.98 s (TDI 0.89 before); CatalogReference and CatalogBoostHold pass.
- Authority harness: outlet monotone tolerance 0.05 -> 0.5 kPa (pulse-resolved outlet), boost hold within the gate's 0.08 proportional band, shaft balance within 2 % of turbine power (was 0.26 kW at the 45 mm stress point).
- Realtime free-run, 4 alternated rounds, minimum: 2JZ 1.61 -> 1.60, EJ25 1.92 -> 1.91, I5 1.77 -> 1.83, TDI 2.18 -> 2.25, K20A control 1.82 -> 1.80; at 95 % of redline 1.59-2.34 -> 1.59-2.27. `--runtime-blip` with audio: lowest 0.992, worst lateness 4.7 ms (TDI lift-off).
- Sound (`AudioRenderHarness` catalogue point, rpm floats with torque so it is not held): RMS 2JZ -1.6 %, EJ25 -27 %, I5 +16 %, TDI +42 %; low-band share 2JZ 14 -> 45 %, EJ25 43 -> 84 %. The turbine move alone (old areas) gave -0.5 / -16 / +33 / +50 %. Not compared with any recording.
- Not done: turbo mid pipes (the downpipe is now cold side); several paths each with a turbine (supported, no catalogue engine uses it).

## 2026-10-12 — Individual throttles as a restriction face in the runner

- New 1-D primitive: a plate on a duct-to-duct face (isentropic orifice fed by the approaching cell's stagnation state; jet re-expanding against the loaded face; the supplying side keeps its own momentum). `EngineLab.RestrictionFace`: choked flow -0.2 % vs isentropic (-1.4 % with the throat momentum on the supplying side), full bore +0.003 % vs no plate, shut 0, mass to 1e-14.
- CP2 with `individual_throttles: true` only (plate 66 mm from the valve, same bores and Cd 0.72, plenum fed through the bores fully open). `DynoSweepHarness`: VE -2 to -8 % above 3,500 rpm; torque -10 to -19 % with the ECU reading the cell behind the plate (62-70 kPa at WOT), -5 to -10 % reading the mean of the valve-side duct (83-87 kPa), kept. The peak stays at 3,500-4,000 rpm: no help for the shape gap.
- Idle (`PhysicsPerfHarness --idle`, 20 s): 1,399 rpm (1,383-1,419) vs 1,408 (1,389-1,439). Free-run at 5,000 rpm: 2.66 vs 3.18 (more cells). Catalogue WAVs byte-identical (no engine uses it).
- Not done: the CP2 is not switched; the audio intake network and the 3-D view still draw one plenum throttle; the ECU still reads a pressure, not the throttle angle.

## 2026-10-12 — Mechanical layer over the exhaust above 1 kHz, catalogue-wide

- Owner's listening: the CP2 sounds better with the mechanical slider at 0. `GeometrySensitivityHarness` (WOT held, runtime default mechanical gain 0.70, voicing not applied), mechanical stem minus exhaust stem per third octave:
- CP2 3,500 rpm: over the exhaust from 1 kHz up, by 20-44 dB at 1.25-5 kHz (12 bands of 28); 7,000 rpm: up to 10 dB at 2-5 kHz (4 bands). CP4 5,000: 9-18 dB from 1.6 kHz (14 bands). 2JZ 3,500: 24-47 dB from 1 kHz (22 bands).
- Reference (literature, not a recording): on a 125 cc motocross bike at full load the silencer is predominant from 500 Hz to 3 kHz, the cylinder a substantial contribution at 1-2 kHz (Astrup, IOA). A constant gain cannot match both CP2 speeds: the excess falls 30 dB from 3,500 to 7,000 rpm, so the layer's speed scaling is suspect, not only its level.
- Changed: CP2 and CP2 full-system voicing `mechanical_gain` 0.82 / 0.78 -> 0 (linear gain: the mix loses exactly the mechanical stem). `AudioRenderHarness` CP2 catalogue point (5,430 rpm): RMS 0.0150 -> 0.0146 (-0.23 dB), physics byte-identical. Not changed: the layer itself, the other engines.

## 2026-10-12 — CP4 below real time at the limiter and on lift-off: overlap drain

- Owner's report. `PhysicsPerfHarness --freerev 1 --release 4` (new step_ms, intake accepted/rejected columns): limiter 3.87 ms per 4.17 ms frame, lift-off 4.23 ms; 102k / 94k rejected intake substeps in 3.3 s / 1 s.
- Cause: during overlap the intake network subtracted the exhaust outflow handed over from the previous window (hot burned gas, ~6-10e4 W) from a chamber refilling with cold charge: energy to 0 K, every attempt rejected. Now the outflow leaves at its mass rate relative to the content, with the chamber's own composition and enthalpy, and caps the step. `EngineLab.IntakeMovingBoundary` fails without it (549 rejections, 5.6e-10 K).
- Second, separate: ring blow-by applied every 100 us instead of every sub-step (12 % of a CP4 limiter step); idle and cruise sub-steps are longer, bit-identical WAVs on 7 engines.
- Runtime with audio, `AudioRenderHarness --runtime-blip` (WOT 2-5.5 s, shut to 9 s), 3 alternated runs: lowest realtime factor on lift-off 0.73-0.80 -> 0.995, worst lateness 18 ms -> 1.8 ms. Whole catalogue after: >= 0.992 in both phases, worst 5.6 ms (2JZ).
- Effect: torque <= 1.1 % (CP4 6,750 rpm), peak 124.0 -> 123.2 kW; WAV level within 3 %, bands within 4 points. The two changes were not separated on the curves.
- Still counted: ~320 of 840 CP4 WOT frames late by under one frame (sub-frame jitter, absorbed).

## 2026-10-12 — Bike WOT torque shapes against Cycle World dyno traces

- `DynoSweepHarness` (250 rpm) against `tests/reference-data/measured-curves/` (rear-wheel), each curve divided by its own peak. Peak rpm, sim vs real: CP3 7,250 vs 7,000; CP4 9,000 vs 9,250; Big Twin 3,500 vs 3,000; CP2 4,000 vs 6,250. Shape RMS: CP2 7.5 %, CP3 6.6 %, CP4 12.7 %, Big Twin 7.1 %.
- Common to the three Yamahas: too much torque below the peak relative to it (CP2 +10 % at 2-3k, CP3 +11 % at 5k, CP4 +15 to +22 % at 3-6k, where the real CP4 has a flat 5-6k hole). Big Twin: -15 % at 5,000.
- CP2: VE is nearly flat (0.915 at 2,000, 0.988 at 7,000); gross IMEP per unit VE flat within 2 % from 4,000 to 8,000; FMEP 0.74 -> 2.63 bar and PMEP up to 1.2 bar at 9,000. The torque falls from 4,000 because friction and pumping grow while VE barely rises. Crank peak 74 Nm vs 67-68 Nm rated.
- One factor, CP2 intake runner 120 -> 240 / 360 mm: peak moves to 5,500 (VE 1.08 / 1.14 at 6,500 / 5,500) but 8-9k collapses (-11 / -29 % vs the trace); shape RMS 8.1 / 15.4 %. The CP2 full-system exhaust (a twin's 42 mm primaries) changes nothing (6.9 %).
- Not found: a single cause. Not measured: real CP2 VE, ignition and lambda at WOT; no catalogue value changed.

## 2026-10-12 — Individual throttle bodies as a small cell per cylinder: refuted

- Tried on the CP2 (real hardware: one 38 mm body per cylinder) with configuration only: two intake paths, each one throttle into a 0.05 L cell (the volume behind the plate), then the 120 mm runner. `DynoSweepHarness`, WOT, 500 rpm steps.
- Torque vs today's shared 1 L plenum: -3 % at 2,000 rpm, -15 % at 4,000, -30 % at 6,500 (48 vs 70 Nm), -54 % at 9,500; VE 0.98 -> 0.74 at 6,000.
- One factor, today's topology with the shared plenum at 0.1 L: -7 % at 4,000, -16 % at 6,000, -29 % at 9,000. The loss is the orifice dumping a pulsating jet into a small well-mixed cell (peak velocity ~3x the mean, loss as v^2). A real open plate sits in a continuous bore and recovers that pressure.
- So an ITB needs the plate inside the 1-D runner (a restriction face between two ducts, choked at idle), with the runner mouth on the airbox. Not built: it moves the MAP signal, the idle bypass and the ECU load the plenum carries today.

## 2026-10-12 — CP2 exhaust lengths: header and silencer do interact

- Owner's report: the sound is only right when header AND silencer are both very long. `GeometrySensitivityHarness --factorial primary|midpipe` (x2 each), CP2 held at 3,500 and 7,000 rpm, third octaves, interaction LL-LS-SL+SS.
- Null (same plan at x1.001): interaction RMS 0.7 dB (3,500) and 1.1 dB (7,000). Primary x silencer: 5.2 / 5.6 dB on the mix, 6.0 / 6.9 dB on the exhaust bus; long-both +13 to +18 dB at 250 Hz where neither alone does it. Midpipe x silencer: 2.1 / 4.3 dB. So the model is non-additive, mostly below 1 kHz; the header length matters more than the midpipe. Caveat: the brake holds each cell within ±130 rpm, the null within ±10.
- At 3,500 rpm the mechanical layer is 20-44 dB above the exhaust from 1 kHz up (12 of 28 bands): the mix does not move there whatever the geometry (pipe x2 changes the exhaust bus by +8 dB at 1.6 kHz, the mix by 0.0). At 7,000 rpm only 4 bands are masked.
- Removing the silencer changes the unweighted RMS by +0.07 dB (3,500) and +0.29 dB (7,000); the exhaust bus rises up to +9 dB at 1.6 kHz. Not compared with a measured insertion loss (none in hand, and the RMS is dominated by < 200 Hz).
- Not measured: whether the real CP2's mechanical noise sits that high over an Arrow can; it decides between "mechanical too loud" and "exhaust too dull above 1 kHz".

## 2026-10-11 — Gas networks on the worker pool, bit-identical

- Each duct's share of an RK stage (update, cache, wall heat, interior residual, its ports' and outlets' face fluxes) runs on the existing worker pool; coupling stays serial in a fixed order. Euler stage and second-stage pass fused (3 barriers per step). WAVs of all 16 catalogue engines byte-identical to the serial build; `EngineLab.GasDynamics` reverses the dispatch order (a misplaced plenum sum fails it with 6 runners, not with 3).
- Free-run, 0.95 x limiter, alternated, 2-3 runs each: Merlin 1.15 (0 workers) -> 1.47; LS3 1.01 -> 1.12. Fusion alone +2-3 %. Duct interiors only (no boundaries) gave Merlin +5-8 %, LS3 nothing.
- With audio, whole catalogue, same hour vs main: worst LS3 1.07 (main 0.95); Merlin 1.40 (1.35). Twins (no pool) 1.98-2.06 vs 2.10-2.22, not investigated. LS3 with audio + 3-D view: 1.05-1.06 (main 0.94-0.96 or invalid, dropped pressure).
- Runtime scenario (`--runtime-filter`, limiter at 50 % throttle), overruns of 1,440: Merlin 115-117 (main 59-112), LS3 267-306 (main 530).
- Not tried: residual of the next step fused into the candidate pass (one barrier fewer except on each call's last step).

## 2026-10-11 — Merlin and Radial clicks: the valve-flow band toggled

- Corrects the 2026-10-09 entry "Merlin runtime gate still red, not from the intake": the cause was in the audio. The complementary valve-flow band is fitted to the published mechanical rate; single short crank sub-steps spiked it to 12-20 kHz for a few ms. On the Merlin the floor (~9.8 kHz) leaves the band empty, so it switched on ~30 times in 8 s with stale state: clicks up to 3x full scale.
- Fix: follow the slowest recent rate (down at once, up over 0.25 s) and unprime a closed band. Offline Merlin peak 3.15 -> 0.45, crest 32 -> 5.0; runtime gate passes (pre-limiter 0.42-0.46). Catalogue RMS unchanged (Radial -1.1 %), Radial window crest 8.91 -> 1.81.
- Refuted: re-tuning the reconstruction as a state-variable filter instead of DF1 sections (-65 dB difference, clicks unchanged).

## 2026-10-09 — Exhaust junction step: the duct cells' convention

- A junction's explicit bound was C V / sum(A (|u|+c)); a duct cell's is C dx / (|u|+c) = 2 C V / sum(A (|u|+c)). The LS3's derived X crossover (2 A d, four 80 mm ports) took half its pipes' step and bound the whole exhaust (~42 us vs ~84 us).
- LS3, one factor (junction bound x0.5 / x1 / x2): torque at 3,000 / 4,500 / 6,000 rpm within 0.15 %; catalogue sound octave bands within 0.2 dB, spectral cosine 0.9987.
- Realtime, LS3 free-run at 0.9 of the limiter (5,940 rpm), alternated: 0.915 / 0.905 -> 1.031 / 1.043. At the limiter (6,270) it was 0.886 before; not remeasured.
- Closed X fixture (`EngineLab.ExhaustJunctionStep`): passive at the new bound and at twice it.

## 2026-10-09 — Intake valve: characteristic boundary; forward coupling 330 us

- Root cause of the intake bias: the quasi-steady valve nozzle fed by the first cell's average made the trapped charge depend on the internal Courant number (single runner + moving piston, Courant 0.8 vs 0.08: -6.5 % at 1,300 mm2, -11 % at 2,000 mm2). Ruled out before: MUSCL limiter, finite plenum, wall heat, valve linearisation, forward coupling alone.
- Characteristic boundary (face on the interior wave curve, mass flux = valve nozzle): 0.07-0.09 % at the same points. Joint manifold then accurate at Courant 0.8; the coupling interval sets the cost.
- Forward coupling with the exhaust overlap drain, K20 vs exchange every substep: 150 us -0.49 % worst, 250 -0.79, 330 -1.26, 400 -1.48 %. Default 330 us.
- Front-loaded delivery (6,500: -2.28 % unchanged) and trajectory-resolved delivery (-0.78 vs -0.79 %): no effect, removed.
- Warm-started secant: 4.3 residual evaluations per solve (was 8.4). Live intake swap now copies these warm starts (bit-identity test caught it).
- Forward intervals counted their substeps at their opening size; over 2,500 <-> 6,500 rpm ramps the network's clock drifted beyond one interval from the chambers'. Each interval now runs to the chambers' expected time from the network's own clock.
- Realtime, free-run, 5,000 rpm (Merlin 3,040): LS3 1.08, Merlin 1.15 (were 0.78 / 0.80). At --relative-rpm 0.9 every catalogue engine > 1.1 except LS3 0.918 at 5,940 rpm: its exhaust step is bound by the 4-port crossover junction (derived ~0.8 L, ~42 us vs ducts ~84 us).
- Sound, AudioRenderHarness catalogue point, `--intake-legacy` (nozzle, every substep) vs new: RMS within ±4 % on 12 of 16; 2JZ -15 %, EJ25 -11 %, Hayabusa +19 % (brightness 0.176 -> 0.143). Merlin pre-limiter 1.51 -> 0.66: its catalogue gate (0.82) now passes. Not judged by ear; full 50-rpm curves not rerun.

## 2026-10-09 — Exhaust coupling cap; LS3 realtime and Merlin clicks still open

- The exhaust flushed on the sub-step nearest its 125 us cap: up to half a sub-step past it, and with crank-angle sub-steps the stride flipped 1 <-> 2 per frame (Merlin: 82 us sub-steps, 164 us windows). Now the last sub-step within the cap; default I4 at a 160 us cap: 941 frames over -> 0.
- AudioRenderHarness, catalogue: most engines < 5 %; CP3 rms +16 %, CP4 in-duct Pa -19 %, flat-six +10 %, EL-09 Pa +12 %, LS3 -4 %. Not judged by ear.
- Realtime, free-run, min of 2 back-to-back: all ±1.5 % except K20A 1.21 -> 1.12 (its 62-83 us sub-steps now couple every sub-step).
- LS3 0.78 vs main 1.13-1.14, Merlin 0.80 vs 1.45: structural. The joint intake network advances every runner on both half-steps (main advanced closed runners once per sub-step). Profiling found ~1 % recoverable without changing results; main's intake workers gained ~1 % too.
- Merlin over the limiter (catalogue pre-limiter 1.49-1.51, gate 0.82): sparse ~5 kHz clicks clipped in the exhaust-dry stem (1.34-1.82 s), same RMS as main; late steady state is quieter (master RMS 0.030 vs 0.044). Ruled out: coupling cadence (fix leaves 1.51), exhaust reactions, firing voices, port state, low-pressure EVOs.
- Reverting only the inflow law: 0.79; only the joint network (per-runner): 0.76; neither: 1.66. An interaction, not one cause. main's own build also fails the non-deterministic runtime Merlin gate.

## 2026-10-08 — Curve spikes: intake numerics, crank step, VTEC switching

- Intake runners on MUSCL/Forward Euler amplified a closed perturbation 823-962x; SSP-RK2 holds the gain at 1. The inflow boundary had a singular small-signal admittance, and the finite plenum was not advanced in the RK stages: both fixed, each with a passivity fixture.
- A 400 us delayed cylinder/plenum exchange moved trapped mass +4.6 % and work -7 % (isolated); exchange now on both half-steps, one joint network per path.
- The crank step was sized from the initial speed and could overrun its angle; it now follows the travelled angle, and a fault rejects the whole 720-degree dyno cycle.
- K20 VTEC switched on instantaneous rpm, valves open, 106 times/s/cylinder: torque CV 3.48 % at 5,800 rpm. Requested per shaft cycle and switched on a common base circle: 0.14 %.
- Adjacent 50-rpm torque steps: K20 8.8-11.4 % -> 1.9-3.5 %, CP2 2.6-5.8 % -> 1.8-2.1 %. Against rated points (100-rpm sweep): 2JZ 4,000 rpm 369 -> 416 Nm (427), Hayabusa max 178 -> 153 Nm (138).
- Refutes the 2026-10-07 "not numerical": that oracle changed mesh and integrator together, on a stale binary.
- Not fixed: Merlin audio over the limiter (pre-limiter 2.7x full scale), LS3 realtime 0.75-0.87. 931 of 942 sweep points are thermally unsettled. Threaded/reuse prototypes in `out/evidence` were never built.

## 2026-10-07 — Hayabusa and 2JZ curve swings come from the intake runner waves

- A rated peak torque is its curve's maximum. Sweep maxima over it: Hayabusa +29 % (178 Nm at 7,500, VE 1.26
  where 138 Nm needs ~1.03), EJ25 +22 %, Big Twin +16 %, the rest ≤ +13 %.
- Hayabusa, one factor each: intake oracle mesh (30 mm, RK2) keeps VE 1.17-1.24 at 7,500-10,000, so not
  numerical; plenum momentum fully dissipated: inert; 8 L plenum: MAP 107.8 -> 103.4 kPa, peak 178 -> 164 Nm,
  but it stands in for throttles the model can only put upstream of it. CR 11.0 (the 1999 engine's; 12.5 is
  2008's): -2 % at 7,000 but VE at 9,500 1.14 -> 1.22. Not applied. Its WOT misfire sits on the VE jump at 7,000.
- 2JZ dip (369 Nm at 4,000 for 427): boost held at 192 kPa, charge over plenum density 0.62-0.81. Well-mixed
  junctions and 650 mm primaries: inert; runners 330 -> 450 mm move the peak 5,500 -> 4,750. Runner waves swing
  ±20 % where real curves move 5-8 %. Next: their damping, against a measured full curve.

## 2026-10-07 — Catalogue exhausts at their real length

- Routes were 0.9-1.3 m (LS3 2.07). `midpipe_length_mm` gives the K20A 3.5 m, LS3 3.0 (realtime: 3.73 m cost
  8 % and one INVALID run, median 1.06 -> 1.02 at 3.0), flat-six 1.57, bikes 1.22-1.49. CatalogReference moves ≤ 4.4 %.
- Turbos refuted: the turbine is the outlet restriction, so the mid pipe sat on its hot side. 2.6 m slowed
  the spool (90 % of the step at 3,000 rpm) 1.30 -> 1.72 s (2JZ), 2.47 -> 3.53 s (I5), EJ25 and TDI alike;
  the 2JZ no longer reached its shift point. 2JZ, EJ25, I5 and TDI keep their old length.
- AudioRenderHarness: level -1.2 dB (LS3) to +2.5 dB (flat-six); the share below 250 Hz falls (CP2 64 -> 47 %,
  Arrow 64 -> 41 %, Lab 53 -> 23 %). Not judged by ear yet.
- Open: the CP3 and CP4 misfire once on the DFCO resume (first charge, CP3 AFR 9.05); the CP2's live remesh
  lands 37 % short of a fine-built engine, already before this change.

## 2026-10-06 — 2JZ boost ran past its wastegate

- DynoSweepHarness: at 520 mm2 (the default gate) the 2JZ's gate is fully
  open from 5,500 rpm and the pressure ratio keeps rising: MAP 202 kPa at
  5,000, 246 at 6,000, 297 at 6,500. EJ25, I5 and TDI hold theirs.
- Turbine inlet at the mean back pressure instead of the port peak: 252 kPa
  at 6,500, not a fix; not kept. Gate 900 mm2 (a 34 mm port): 190-194 kPa
  from 3,000 to 6,500; 1,300 mm2 gives the same curve.
- CatalogReference: power 275.2 -> 259.2 kW (+15.2 -> +8.4 %), torque 371.3 ->
  373.2 N.m at 4,000 (-12.6 %). Audio harness (holds ~3,700 rpm): RMS and peak
  equal, crest 6.33 -> 6.84. New test CatalogBoostHold (fails at 520 mm2: 2.47).

## 2026-10-06 — DFCO resume: the EJ25's peak is lean, not rich

- LoadedAccelerationHarness now splits the resume error: rich 16-30 %, lean
  5-35 %. The EJ25's 35.1 % (which loosened the envelope to 0.40) is its first
  post-cut cycle, lean: first four charges 219 mg metered, 120 mg in the
  chambers (162 needed); the dry runner and film kept the rest.
- The rich side is the resume trim: it starts at the fourth root of the
  learned high-load cell (EJ25 0.92 against 0.73 settled), ~3 cycles to fall.
- One factor each, catalogue max: no prime 51 % lean; start sqrt 41 %; full
  cell 58 % lean; unity 30 % but Hayabusa/CP3 misfire; 150 mm footprint 34 %
  but 2JZ 23 misfires; both 40 % (EJ25) and 26 misfires. Kept: rich 0.35, lean 0.40.

## 2026-10-06 — Injectors drawn, Ctrl+S, quit warning, no manual load

- `EngineLab.EngineModel3D`: one injector per cylinder on the 16 catalogue
  engines (port and direct), named with its cylinder by a click, port in the
  runner spraying at the valves, direct in the roof spraying down.
  Mutations caught: 6 of 6.
- `EngineLab.SourceEncoding` fails on the `⋯` menu text corrupted in 8af95bd
  (UTF-8 read as Latin-1) and passes on the 258 repaired files.
- Not exercised on screen: the inspector, the side panel's *Show in 3-D*,
  Ctrl+S against the starter, the quit dialog (no screen here).

## 2026-10-06 — Sound sources at their drawn places

- With the camera microphone on, the exhaust and intake still radiated from
  the origin (their authored, schematic place). Now each outlet sounds from
  its drawn tip and each intake path from its drawn mouth, the turbo from its
  compressor. `EngineLab.EngineModel3D`: every compiled outlet of the 16
  catalogue engines and a scalar copy finds its drawn tip and axis.
- `EngineLab.CameraMicrophone`: CP2 openings 1 m before the microphones
  +11.9 dB exhaust, +11.9 dB intake; 2JZ turbo halfway +6.00 dB (1/r 6.00);
  switched off, bit for bit again at 7.11 s. Mutations caught: 13 of 13.
  Not listened to in the application.

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
