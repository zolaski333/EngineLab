# Completed subreview notes

These preserve source traces, asset checks, refutations and unpromoted leads. The canonical finding count/classification is in [audit.md](../audit.md); these notes are not additional independent findings.


---

Original scratch report: `out/audit2/physics/findings.md`

# Physics / simulation / gas dynamics review

All 199 files assigned across this audit wave were fully read (64,474 inventory lines): 119 source/header/internal-test/CMake files, 24 engine and parts data files, 33 top-level test files, two model documents, and 21 reference-data files. `coverage.json` records each path, SHA256, line count, notes and validation linkage, with no unreviewed assigned files. No production source or build configuration was edited.

The bounded probes executed after the root agent released the baseline CPU window. `prepare_probes.py` extracts the current throttle helper and knot-selection block verbatim; its manifest records the source and header SHA256. `build_probe.ps1` recompiles the current gas ownership sources with MSVC 14.44 x64, C++20, /O2 /MD and VS Release dependencies. `build_auxiliary.ps1` recompiles current SimpleEcuModel.cpp for the direct ECU witness; persistence/script decoding links the freshly completed authoritative VS Release libraries. The alias failure runs only in a separate child process with a ten-second bound and Windows fault dialogs disabled. Logs and complete provenance are under this directory; `verification.json` identifies the exact witnesses and hashes. These are component/synthetic-input results, not new manufacturer, audible or real-time measurements.

Seven new findings are recorded: PG-01, PG-02, PG-04, PG-05, PG-06, PG-07 and PG-08. PG-03 extends existing PH-06 and is excluded from the new count. The airbox lead remains unpromoted.

## PG-01 — Exhaust boundary history is shorter than its fixed reconstruction delay

- Priority P1 for sound-model integrity; source-proven temporal-coverage failure, runtime/audio magnitude not yet measured.
- Sources: `src/simulation/include/enginelab/simulation/EngineSimulator.hpp:541-544`; `src/simulation/src/EngineSimulator.cpp:2030-2063` (fine scavenging cadence), `:5897-5928` (selection), `:5963-5965` (overwrite).
- Trigger: exhaust flushes faster than 11,200 Hz for enough calls to replace the eight history slots. Seven inter-knot intervals then cover less than the fixed 625 microsecond delay. A two-stroke's production fine scavenging window explicitly flushes every mechanical substep; this condition therefore does not require an unsupported cylinder count or a diagnostic override. Large high-speed four-strokes can also exceed the history capacity.
- Proof: the search reaches the oldest valid slot without finding a knot at or before the target. `before` and `after` then refer to that same oldest slot. The zero interval forces phase 1, publishing the oldest value unchanged until the next flush replaces it. The published boundary consequently uses the oldest retained state with a delay that depends on the current coupling interval, despite the explicit constant-delay contract. If there are mechanical publications between flushes this value is held between them; when every mechanical substep flushes (the fine-scavenging trigger), the oldest state also advances every publication, so downstream interpolation can preserve a linear ramp despite the wrong delay. At a 30 microsecond flush interval the history spans 210 microseconds, 415 microseconds short of its requested delay.
- Impact: the pressure, flow and medium pair no longer follows the fixed-delay linear reconstruction in these supported operating windows. It changes delay when the cadence enters/leaves the fine window. Coupling-rate holds require a publication cadence faster than the flush cadence and are not asserted for the every-mechanical-substep fine window itself. No claim is made about the audible dB magnitude or a blind listening outcome.
- Reproduction: the source-extracted `RingProbe` feeds a linear timestamp ramp through 125 and 30 microsecond histories. `component_probe.log` reports `fine_delay_us=209.99999999999753`, `target_delay_us=625`, `max_error_us=415.00000000000261`. The separate between-knot control reports output delta 0 for a 10 us input advance; it shows the hold only when publications exceed flushes and is not the every-mechanical-flush fine-scavenging trigger itself.
- Confidence: high in the history/selection defect; unmeasured perceptual effect.

## PG-02 — Wet nitrous fuel is omitted from fuel consumption

- Priority P2; source-proven bookkeeping omission.
- Sources: `src/simulation/src/EngineSimulator.cpp:2696-2709`, `:6228-6236`, `:6284-6285`.
- Trigger: a fitted wet kit actively sprays. Its supplementary gasoline is injected directly into the intake plenums at the nitrous flow and configured kit lambda.
- Proof: the wet-kit branch calls `injectFuelMoles` but does not add that quantity to a metered-fuel accumulator. At the only consumption commit, `totalMeteredFuelMassKg` contains cylinder injector metering plus carburettor metering only. The kit fuel therefore burns and contributes torque without entering `fuelFlowGramsPerSecond`, `fuelConsumedGrams` or `fuelConsumedLitres`. The BSFC denominator includes its power while the numerator omits its fuel.
- Impact: reported fuel use and BSFC undercount fuel while the wet kit sprays; dry-kit supplemental fuel follows the normal injector and does not have this same omission. End-to-end magnitude not measured.
- Confidence: high; complete supply-to-counter ownership trace.

## PH-06 extension (PG-03) — Live ambient edits leave intake and exhaust wall surroundings at the old temperature

- Priority P2; source-proven state propagation omission, related to but broader than the existing PH-06 clutch finding.
- Sources: `src/simulation/src/EngineSimulator.cpp:1112`, `:1279`, `:1634-1652`; `src/gas-dynamics/src/ExhaustGasNetwork.cpp:843-845`; `src/gas-dynamics/src/FiniteVolumeDuct.cpp:872-879`.
- Trigger: edit the ambient temperature through the supported live settings path while retaining the existing intake and exhaust networks.
- Proof: the networks copy the construction ambient into `externalTemperatureK` and then into each duct's geometry. Every subsequent dynamic wall heat exchange reads that stored geometry value. `replaceSettings` updates `config_` and the exhaust's atmospheric reservoir, but neither the existing network config nor any duct geometry is updated. No per-step ambient argument reaches the wall heat operation.
- Impact: inlet/outlet gas boundaries and the engine thermal model follow the new atmosphere while the pipe/runner metal continues rejecting heat to the construction atmosphere. The mismatch remains after a normal `reset`, which resets gas and metal temperatures but retains duct geometry. No long thermal transient was measured.
- Confidence: high; complete ownership trace. This evidence extends existing PH-06 and is excluded from the new finding count.

## PG-04 — The internal reordered-duct test executes one ascending chunk

- Priority P2 validation limitation; source-proven fixture defect.
- Sources: `src/gas-dynamics/tests/IntakeNetworkTests.cpp:343-350`, `:385-386`; `src/gas-dynamics/include/enginelab/gasdynamics/ExhaustGasNetwork.hpp:510-515`; `src/gas-dynamics/src/ExhaustGasNetwork.cpp:2075-2079`, `:2095`.
- Trigger: the ordinary `testPerDuctPassIsOrderIndependent` test run.
- Proof: `{ &ReversedExecutor::run, &executor }` leaves `ParallelFor::threads` at its default 1. Chunk construction therefore makes exactly one chunk. The reverse executor reverses only that one chunk index; the chunk callback still visits all six ducts in ascending order. The `dispatches > 0` assertion proves that a callback ran, not that the duct order differed.
- Impact: the named test can pass when per-duct execution order affects the solve; its two runs use the same duct order. This does not itself establish a production race or order-dependent result.
- Reproduction: `component_probe.log` records `completed=1 dispatches=3 maximum_dispatched_chunks=1 ducts=6` with the same two-field executor initializer. The callback is exercised, but has only one chunk to reverse.
- Confidence: high.

## PG-05 — Remeshing a tapered duct does not preserve its inventory

- Priority P2; confirmed geometric weighting discrepancy and public-API executable reproduction.
- Sources: `src/gas-dynamics/src/ExhaustGasNetwork.cpp:1088-1100`, `:1128-1154`; `src/gas-dynamics/src/FiniteVolumeDuct.cpp:594-628`, `:798-813`; `src/gas-dynamics/tests/ExhaustGasNetworkTests.cpp:1308`.
- Trigger: `adoptStateFrom` remaps a nonuniform tapered duct between different cell counts, including a supported live exhaust edit that changes mesh count.
- Proof: remapping averages conserved densities by normalized axial overlap only. The extensive inventory later integrates those densities over the physical frustum cell volumes. In a taper, the portions either side of a source-cell boundary have different volumes, so their equal-length weights are not their inventory weights. The existing mesh-conservation test explicitly skips variable-area ducts. A two-cell 4-to-1 area contraction with different density/energy in its two cells remapped to three cells is a bounded public-API witness.
- Impact: species mass and total gas energy can change on a remesh even when the authored geometry itself is identical. The broader resize operation deliberately changes inventory with physical volume; this witness holds the geometry fixed and isolates the remap.
- Reproduction: `component_probe.log` records mass `0.0036623684972229772` kg before and `0.0037112000771859508` kg after this two-to-three-cell adoption. Both mass and total energy increase by `0.013333333333333419` (1.33333333333334%). Both layouts configure successfully and adoption returns true; geometry is held fixed.
- Confidence: high; the source discrepancy and bounded numerical inventory change agree with an independent frustum-volume calculation.

## Candidate not promoted — Returned airbox fuel adds to a fresh-air inflow

`throttleFlowKeepingReversedFuel` at `EngineSimulator.cpp:397-444` first draws fresh atmospheric composition for the full mass inflow, then injects the fuel fraction returned from its upstream slug without removing the fresh mass it replaces. This appears to add fresh oxygen/inert material when the slug is drawn, rather than restoring the slug's full composition. The extracted helper compared equal starting plenums with/without a fuel-bearing slug: fresh-boundary inflow was 1.3774354573929063e-7 kg, returned fuel 1.5734445229799169e-9 kg, and extra plenum mass 1.5734445230021141e-9 kg, with unchanged oxygen. This confirms the code path but does not establish the intended total open-boundary inventory contract. The upstream atmosphere is open; no new registered defect is inferred from that arithmetic alone.

## False leads and scoped limits

- Rotary face injection initially appeared to share a duct across parallel cylinder workers. The simulator explicitly detects shared runner addresses and disables that parallel pass; no defect claimed.
- Reverse-phase crossing helpers initially appeared to manufacture events on kickback. The current crank integrator clamps angular velocity nonnegative, so this is not a reachable current-production reversal path; no finding.
- Known two-stroke mesh deficits, the 13B power exception, mean-gas approximations, accepted combustion compensations, and first-pass PH-01 through PH-06 were not recounted as independent new physics defects.
- The audit did not run new manufacturer curves, acoustic renders, blind listening, numerical convergence sweeps, long thermal scenarios or realtime benchmarks. Root owns full build/suite evidence separately.


## PG-06 — Overrun backfires re-enable fuel during an overboost cut

- Priority P2; confirmed command-priority defect with a direct ECU witness.
- Sources: `src/ecu/src/SimpleEcuModel.cpp:583-585`, `:733-742`, `:788-789`, `:1018-1024`; the explicitly protected anti-lag branch is at `:914-915`.
- Trigger: a turbocharged gasoline engine with a nonzero overboost margin, late-spark overrun enabled, a closed throttle above the overrun speed, and manifold pressure still above the overboost threshold. This includes the transient just after lifting off a boosted run.
- Proof: `fuelEnabled` is false on overboost, but `overrunAfterfireActive` does not include the overboost protection gate. The final fuel command ORs that afterfire boolean with the protected normal command and consequently becomes true. Telemetry still publishes `ActiveLimit::overboostCut`, so it reports a fuel cut while the output continues metering the full overrun charge.
- Impact: the explicitly enabled boost protection is bypassed while this listening option is active. Whether a particular engine produces a sustained boost excursion depends on charge/shaft dynamics and was not measured.
- Reproduction: `ecu_probe.cpp` validates a normalized turbo/afterfire configuration, then compares an identical 4,000 rpm, 300 kPa closed-throttle input with clean DFCO and late-spark overrun, using a published 40 kPa overboost setting. `ecu_probe.log` reports `overboost_active=1 afterfire_active=1 afterfire_fuel=1 clean_fuel=0`.
- Confidence: high in the command defect; no sustained engine trajectory claimed.

## PG-07 — A YAML alias cycle in embedded ECU data recursively exhausts the stack

- Priority P1; reproduced through the public saved-file decoder in an isolated child process.
- Sources: `src/catalog/src/SavedEngines.cpp:52-82` (`jsonOf`), `:219-225` (conversion before calibration validation).
- Trigger: append `ecu_calibration: &loop [*loop]` at document root to an otherwise valid YAML engine. This is a small alias graph rather than a deeply nested textual document.
- Proof: the engine serializer does not consume the calibration member. The subsequent YAML-to-JSON conversion visits every sequence child recursively without an identity/cycle guard. An alias node refers to the original sequence, so the conversion has no terminating leaf and never reaches the calibration validator. A standard C++ exception handler does not turn Windows stack exhaustion into a decoder error.
- Impact: importing this file can terminate the application instead of retaining the running engine and reporting malformed ECU data. This is distinct from first-pass DR-03's engine-script expression recursion.
- Reproduction: `persistence_probe.cpp alias` first decodes an ordinary generated YAML engine, appends the one-line alias and invokes public `decodeEngineFile`. `alias_probe.log` reports `plain_yaml_valid=1`, reaches `entering_alias_conversion`, and exits with code `3221225725`, hexadecimal `C00000FD` (Windows stack overflow). The child does not return a decoder error.
- Confidence: high; current source and public-decoder child failure agree.

## PG-08 — Scripted listener distance keeps millimetres in a metre field

- Priority P2; source-proven typed-unit conversion error.
- Sources: `src/scripting/src/EngineScriptCompiler.cpp:64-65` (canonical millimetre length), `:402-409` (microphone and listener setters), `:1020-1024` (literal unit conversion); `src/foundation/src/EngineTypes.cpp:939-942` (distance range).
- Trigger: set `acoustic_observer.listening_distance_m` in an engine script using any length unit.
- Proof: lengths are evaluated in millimetres. Every adjacent microphone-coordinate setter multiplies its argument by 0.001 before storing metres, but the listening-distance setter stores it directly. `4 m` becomes 4,000 metres and fails the 200-metre validation ceiling; `100 mm` becomes an accepted 100 metres instead of 0.1 metre.
- Impact: ordinary metre-valued scene distances reject valid scripts; shorter typed lengths can publish an observer distance 1,000 times the requested one, changing propagation delay and attenuation. The nonzero supported range is 0.5-200 m; its millimetre representation is 500-200,000 and therefore always exceeds the erroneous 200-metre ceiling. Conversely, the 100 mm witness is shorter than the intended minimum but is admitted as 100 m.
- Reproduction: the normal mode of `persistence_probe.cpp` compiles both examples. `persistence_probe.log` reports `four_metres_accepted=0` with final engine validation failure, then `hundred_millimetres_accepted=1 stored_metres=100`.
- Confidence: high; complete literal-to-storage ownership trace.

Additional false lead: EngineEditScope classifies the whole crankshaft vector with settings, but `EngineSimulator::settingsReplaceable` rejects layout/id/ratio/phase/position edits before runtime publication. No accepted stale mechanical topology was established.

## Review coverage and documentation observations

The 33 assigned test files were checked for fixture validity, finite-state gates, false non-vacuity, timing/settling, conservation, manufacturer-versus-numerical references and accepted exceptions. No generic missing-test assertion is counted as a production defect. Current forward joint-intake tests distinguish the historical independent-runner predictor diagnostics; the old approximation was not promoted as a current default-path bug. The 21 reference-data files were checked internally, without independent publisher verification: all 46 manufacturer target rows have positive finite numbers and unique target keys; measured torque curves have strictly ascending rpm and lb-ft/Nm columns agree within 0.05 Nm rounding. Named assumed valve/lift/flow/load inputs remain assumptions.

For consolidation into existing V08 only: `docs/simulation-model.md:88-90`, `:110-113` and `:1060-1061` describe a 0D/Helmholtz-only intake, while the current production owner builds joint finite-volume manifolds; `:918-930` describes the older AMT throttle-only/no-ignition-cut policy. The newer `docs/thermoacoustic-architecture.md:900-975` explicitly describes the current forward joint manifold and labels old partitioned controls as diagnostics. `tests/reference-data/catalog-manufacturer-sources.md:39` calls the CP2 wheel-curve peak 61.6 Nm; its traced CSV peaks at 59.1 Nm (43.6 lb-ft), while the detailed provenance table correctly states 43.6 lb-ft. Historical performance and acoustic tables were not treated as fresh measurements.

Additional refutation from cross-review: physical `ExhaustGraph` retains a cone as one varying-area node, and `ExhaustNetworkLayout` emits one duct for that node. Only the audio compiler tessellates it. The suspected physical PressureProbeHarness cone-segment mapping defect is therefore refuted; audio-only segmentation findings belong to the audio reviewer.

Cross-module source contract: audio reviewer A2-08 separately records the fixed 625 us delay present only on the low-band network pair before complementary LR summation with current-time raw flow. This remains distinct from PG-01 (which violates that fixed delay when history is too short). A2-08 is owned and counted by the audio report only; its controlled frequency witness does not establish a catalogue engine spectrum error.


---

Original scratch report: `out/audit2/audio/findings.md`

# Audio/events/assets audit, second pass

Status: every assigned tracked file was fully read; 83 paths recorded in `coverage.json`. Binary files were structurally decoded and checked, not treated as semantic source. The bounded actual-source/header probe compiled against the authoritative Windows VS Release libraries and exited 0 after root released the CPU window. Exact output is in `probe-output.txt`; `probe-validation.md` records scope. No fixes, sound adjustments, tracked source edits, or build configuration changes were made.

## New findings under verification

### A2-01 — Cancelling a stem export leaves a Windows-locked partial and blocks the next export

- Priority: P2 — failed cancellation cleanup blocks an ordinary retry.
- Sources: `src/audio/src/OfflineAudioExporter.cpp:620-629`, `:763-768`, `:1015-1022`; public cancellation contract `src/audio/include/enginelab/audio/OfflineAudioExporter.hpp:166`.
- Trigger: export with `writeStems=true`, cancel from a progress callback after rendering begins.
- Evidence: the normal failure lambda closes `orderMapStream`; the cancellation branch only aborts WAV writers and calls `removePartialFiles` while the CSV stream remains open. On Windows an ordinary open output stream prevents file deletion. Stream destruction happens only after the attempted cleanup; cleanup ignores errors. Preflight then rejects the leftover `engine-order-map.csv.partial`.
- Impact: a cancelled export can require manual output-directory cleanup before retry. Separate from UA-05 overwrite destruction and UA-06 unguarded diagnostic overwrite.
- Actual reproduction: the public exporter returned cancelled=true, left `engine-order-map.csv.partial`, then rejected retry with `Stale partial export exists`. Windows Release probe exited 0; see `probe-output.txt`.
- Confidence: high, source and actual public-API reproduction.

### A2-02 — Forced-induction sound continues while paused

- Priority: P2 — audible paused-state behavior contradicts the runtime state.
- Sources: `src/runtime/src/EngineRuntime.cpp:228-305`, `src/audio/src/RealtimeEngineAudio.cpp:876-879`, `:2024-2045`, `src/audio/src/ForcedInductionAcoustics.cpp:191-192`, `:257-259`.
- Trigger: pause while a turbocharger/supercharger is producing audible sound.
- Evidence: paused publication zeroes RPM/throttle/time scale, but keeps the forced-induction shaft speed, compressor/turbine power and flows. Renderer maps zero time scale back to `acousticTimeScale=1`, and sends that to the forced-induction source. The source's explicit zero-time-scale silence path is therefore unreachable through pause. MainComponent's callback still calls render while paused.
- Impact: a continuous turbine/blower tone and flow noise remain after simulation freeze instead of ringing down.
- Actual reproduction: a synthetic paused frame through the actual renderer published timeScale 0 and shaft 12000 rpm, yet the late forced-induction stem RMS remained 0.000506531 after 400×256 samples (last 200 blocks measured). No simulation thread or live device was used.
- Confidence: high source trace and actual renderer result. Catalogue audible level and listening not measured.

### A2-03 — Closing a valve preserves an undamped offset in the reflection filter

- Priority: P2 — dynamic termination state violates the stated passive acoustic behavior; full-render magnitude remains unmeasured.
- Sources: `src/audio/include/enginelab/audio/ValvePortTermination.hpp:62-65`, `:98-106`, `:143-151`; production callers `src/audio/src/AcousticExhaustNetwork.cpp:1088-1102`, `src/audio/src/AcousticIntakeNetwork.cpp:848-855`.
- Trigger: a previously open valve closes while reflection-filter state has `previousOutput != previousInput`, an ordinary dynamic state.
- Evidence: the rigid coefficients `{1,-1,-1}` give `y[n]=x[n]-x[n-1]+y[n-1]`. This cancels to identity only with matching history. Switching from the open filter retains its old history, so a zero incident signal thereafter produces a constant nonzero output indefinitely. A finite initial excitation can therefore produce unbounded cumulative reflected-wave energy in the isolated termination.
- Impact: nonphysical source/DC state is fed into both intake and exhaust wave networks on valve closure. Effect on complete engine spectra is not quantified; no claim about a specific audible artifact.
- Actual reproduction: a 1000 Pa impulse with the open coefficients produced 928.171875 Pa; after switching to closed coefficients and processing 48000 zero inputs, the last output remained -71.8281403 Pa and the accumulated tail square-sum was 247645523 Pa²-samples.
- Confidence: high algebraic/source evidence and actual-header execution. Full-render spectrum remains unmeasured.

### A2-04 — The jet-noise renderer bypasses its own subsonic velocity cap

- Priority: P2 — rendered source power disagrees with its configured capped physical law.
- Sources: `src/audio/src/ExhaustJetNoise.cpp:66-80`, `:96-110`, `:143-164`, `:191-207`; production call `src/audio/src/AcousticExhaustNetwork.cpp:1438-1444`.
- Trigger: configured positive outlet flow whose `mdot/(rho*A)` exceeds `0.95*c`.
- Evidence: public target spectrum and target RMS use `limitedJetVelocityMps`; configured volume velocity and inverse area retain raw values. `process` multiplies them and uses the uncapped fourth power for emitted pressure, capped only at 100 kPa. At Mach 2 the pressure exceeds the stated target by `(2/0.95)^4` (approximately 25.9 dB).
- Impact: accepted high-flow/small-outlet inputs can emit materially more broadband sound than their reported subsonic model. Full-engine reachability and incidence remain to be measured; the production network forwards raw finite positive outlet telemetry without the cap.
- Actual reproduction: the actual class at 48 kHz with Mach 2 input reported target RMS 10.304111 Pa while 96000 settled output samples measured 202.919295 Pa RMS, +25.886256 dB over target.
- Confidence: high source trace and class-level execution. Full-engine incidence remains unmeasured.

### A2-05 — Intake tapered runner propagation gains power between unequal endpoint admittances

- Priority: P2 — a supported passive component has an active one-way transfer.
- Sources: `src/audio/src/AcousticIntakeNetwork.cpp:275-280`, `:623-627`, `:711-713`, `:825-828`, `:913-920`.
- Trigger: intake runner with `runnerPlenumDiameterMm` different from the valve-end runner diameter, under a steady finite medium.
- Evidence: one delay line transports pressure unchanged, apart from wall loss. The valve-end impedance uses inlet area while plenum scattering uses outlet area. Acoustic traveling-wave power is proportional to `A*p^2/(rho*c)`, so the same pressure acquires `Aout/Ain` power in one direction and its reciprocal in the other. A diameter ratio 2 gives a factor 4 (6.02 dB) before wall loss. This is the identical mechanism explicitly documented and removed from the exhaust implementation at `src/audio/src/AcousticExhaustNetwork.cpp:342-354`; that removal does not cover intake.
- Impact: passive conical intake hardware is represented by an active one-way transfer; it can bias physical intake source levels and spectral comparisons. No catalogue-wide amplitude or listening claim made.
- Confidence: high source/algebraic evidence; no full-network energy probe run yet.

### A2-06 — Authored-component pressure taps address only the first taper section

- Priority: P2 — diagnostic pressure recordings are taken at a different position from their label.
- Sources: public request `src/audio/include/enginelab/audio/OfflineAudioExporter.hpp:65-74`, propagation `src/audio/src/OfflineAudioExporter.cpp:742-751`, network selection `src/audio/src/AcousticExhaustNetwork.cpp:1928-1937`.
- Trigger: export a pressure tap on a tapered authored component compiled into several acoustic sections, with nonzero axial fraction.
- Evidence: each section shares the component's graph node ID. `setPressureTap` selects the first matching duct and stores the requested fraction directly. It does not map the fraction across the component's several sections, unlike `injectReactionPressure`, which explicitly resolves section count and local fraction. For a four-section cone, fraction 1 is the end of its first quarter instead of the authored outlet.
- Impact: diagnostic recordings can be labelled with the wrong physical position, undermining comparisons with measured pipe pressure. Default render is unaffected.
- Confidence: high source trace; no reference-measurement conclusion inferred.

## Deduplicated findings and rejected/unconfirmed leads

- Offline ECU calibration omission is already UA-01; destructive publication and diagnostic tap overwrite are already UA-05/UA-06.
- Muting/soloing still works in physical-reference mode: `layerGain` preserves an explicit zero. Positive level editing switching into capture mode is deliberate.
- Cached physical-intake/forced-induction availability does not change after a successful live replacement: replacement rejects transitions that start/stop acoustic availability. App handling of rejected replacements is outside this wave.
- Offline relative IR asset roots pass `.string()` to JUCE's UTF-8 string constructor. Actual Windows reproduction copied a valid shipped WAV into an `ir-é` asset directory; the render chain reported 1 authored/0 loaded IR and a file-not-found warning with the corrupted directory. This extends existing DR-05's path encoding family rather than being counted independently here.
- Pass-by allocates history for an approaching source at 100 m/s but accepts 0.8 times sound speed. At sufficiently distant/high-speed run positions retarded delay is clamped. Reachability through supported run speed controls is not established; no confirmed finding yet.
- The live network-owner access was subsequently traced through actual UI/callback paths and independently confirmed by the app reviewer; it is recorded as source-proven A2-07 below, with no race-detector or crash claim.
- NonlinearDuctAcoustics claims unconditional passivity for an approximate amplitude-dependent delay warp; an independent closed perturbation/energy test has not been run. No confirmed sound defect inferred.
- Extremely large public loop duration/RPM values can exceed numeric/memory bounds; UI/catalogue reachability not established, so not counted as an ordinary-user bug.
- Realtime event drain, stems, upsampling remainder, per-path IR routing, source counts, reaction complements, directional crossover and limiter paths were read in full. No extra finding established from those reads alone.

## Asset checks

All six WAVs parse as mono 44.1 kHz PCM16 with consistent RIFF/chunk/data sizes and finite decoded samples. `exhaust_default.wav` and `exhaust_street.wav` are byte-identical as locally documented. Exact hashes, frame counts, peaks and chunks are in `assets.json`. Local README and license attribute each file to Ange Yaghi's engine-sim under MIT; upstream binary identity/license provenance was not independently verified. README's statement that these are automatically selected default/preset kernels differs from current renderer behavior, which installs no fabricated default IR. This is a documentation drift observation, not proof of a rendering defect.

## Validation boundary

The isolated probe compiled against the authoritative Windows VS Release libraries and ran after root released the CPU window. It exercised actual public APIs/headers for A2-01/02/03/04/08, RT2-01 and the existing DR-05 extension. The production zero-match CLI trigger for TL2-03 also returned 0 with a header-only result. A2-05/06/07 and T2-01 remain source-proven without standalone full-path execution; TL2-01/02/04 additionally have bounded textual/algebraic witnesses. No real recording comparison, blind listening, sound change, heavy engine sweep, race detector or live audio device was used. Root owns authoritative full-suite evidence.

### A2-07 — Live replacement reads the network owner while the audio thread swaps it

- Priority: P2 — ordinary overlapping live edits contain unsynchronized C++ object access; no crash prevalence is established.
- Sources: `src/audio/src/RealtimeEngineAudio.cpp:152-155`, `:281-289`, `:315-320`, `:375-381`; caller paths `src/app/src/MainComponent.cpp:700`, `:707-722`, `:1854-1855`.
- Trigger: submit another live intake or exhaust edit while the callback completes the crossfade from a previous edit. Both successful live application paths keep the device running; `getNextAudioBlock` calls the same renderer concurrently.
- Evidence: the message thread checks `acousticIntakeNetwork_` or `acousticExhaustNetwork_` through the non-atomic `unique_ptr::operator bool`. The audio thread writes that same `unique_ptr` through `std::swap` in `completeIntakeSwap` or `completeExhaustSwap`. The atomic incoming/retired handoffs transfer other objects; they do not synchronize the message-thread current-owner read with this write. In particular, the read can occur after `collectRetiredExhaustNetworks` has observed an empty retired slot and before completion publishes its retired object.
- Impact: supported overlapping live edits have a C++ data race and therefore undefined behavior. This is independent of the previously recorded driveline-config race DR04. Both network pointers normally remain nonnull, so a particular wrong branch, crash or audible discontinuity has not been established.
- Confidence: high from the concrete UI and callback paths; independently source-confirmed by the app reviewer. No ThreadSanitizer run, observed crash or prevalence claim.

### A2-08 — Complementary exhaust bands are summed with different source times

- Priority: P2 — the physical crossover violates its explicit coherent-summation timing contract.
- Sources: fixed low-band delay `src/simulation/include/enginelab/simulation/EngineSimulator.hpp:541-544`, knot reconstruction and separate instantaneous flow `src/simulation/src/EngineSimulator.cpp:5897-5949`, common publication `:6678-6686`; audio interpolation/filter/sum `src/audio/src/RealtimeEngineAudio.cpp:1730-1755`, `:1803-1840`, source consolidation `:1860-1867`, common propagation `:2115-2119`; contract `docs/thermoacoustic-architecture.md:149-158`.
- Trigger: the instantaneous complementary valve-flow band is active while the low-band network boundary is reconstructed with its normal 625 microsecond delay. Adequate ring capacity does not prevent this mismatch; this is distinct from PG-01.
- Evidence: the network pressure/acoustic-flow pair describes simulation time t-625us, while the instantaneous mass flow describes t. The renderer interpolates both at the same publication time, passes them through nominally matching LR low/high filters, and adds their characteristics immediately. `ValveFlowAcousticSource` contains only high-pass and upper reconstruction-filter state; it has no alignment delay. `outgoingMeasured`/`incomingMeasured` are consolidated into one `cylinderExhaustPulse` before the common acoustic network, so no downstream branch-specific compensation remains possible.
- Controlled counterexample: the exact current coefficient formulas at 48 kHz audio, 40 kHz mechanical source and 5 kHz coupling have their crossover at 2350 Hz. Eight knots span 1.4 ms, comfortably exceeding 625 us. With equal-amplitude coherent sinusoidal branch inputs, the ideal LR pair sums to magnitude 1; retaining the production upper reconstruction filter gives 0.978735 when aligned and 0.300074 when only the low branch receives 625 us delay, a 10.2687 dB difference attributable to that relative delay alone. `crossover-delay-math.py/.json` records this frequency-domain calculation. The actual-header probe then reproduced aligned RMS 0.692070052 versus delayed RMS 0.212184695, a -10.268720 dB difference after settling.
- Impact and limits: matching LR slopes do not ensure coherent spectral ownership when their inputs have different source times. The controlled witness establishes the semantic timing mismatch and the unsupported all-pass inference. Real finite-volume and instantaneous flows also differ in bandwidth/amplitude, so this does not assert a 10.27 dB catalogue error, an audible outcome, or a complete engine transfer function.
- Confidence: high from complete publication-to-consolidation ownership, coefficient math and actual-header execution. No full-engine transfer or spectrum measurement.



---

Original scratch report: `out/audit2/audio/runtime-findings.md`

# Runtime/diagnostics audit continuation

Every tracked file under `src/runtime/` and `src/diagnostics/` was fully read after the audio wave. Coverage and hashes are in `runtime-coverage.json`. No tracked files were modified. Root's build/full-suite CPU reservation remained in force during these reads.

## RT2-01 — Changing engine during a datalog makes the recorded time jump backwards

- Priority: P2 — a continuous user recording loses monotonic time across a supported engine replacement.
- Sources: `src/runtime/src/DataLogger.cpp:57-71`; sentinel initialization `:52`; application reachability independently checked by the app reviewer at `src/app/src/MainComponent.cpp:440-529`, `:2181`.
- Trigger: start a log after an engine has already run, then replace/reload the engine while the log remains open. DataLogger is an application-lifetime member; replacement restarts the simulator clock without stopping the logger.
- Evidence: push simulation times100,101,0,0.02,0.04 with the default0.02s minimum interval. After the rewind the intended retained origin is `0-(101-100)-.02=-1.02`. The next push interprets that negative origin as the `startSeconds_<0` initialization sentinel and replaces it. The actual Windows Release public-class probe recorded CSV times0.000,1.000,1.020,0.000,0.020; see `probe-output.txt` and `probe-run-20261116-1642/rewound-datalog.csv`.
- Impact: one continuous CSV loses monotonic elapsed time and conflates pre/post-reload periods. Downstream time-series alignment and duration calculations become wrong. Separate from existing DR-09 same-second filename reuse.
- Confidence: high source/reachability evidence and actual public-class reproduction. No running application interaction performed.

## Rejected or unconfirmed leads

- `finishDynoSession` archives and clears `currentRun_` together under `dynoMutex_`; completed runtime runs are not retained as live runs. No finding from this hypothesis.
- A pass-by held gear disables automatic shifting and restores it when the scripted run ends or is cancelled. `PassByDriver`'s own automatic-gear short circuit therefore does not defeat this requested gear ceiling.
- `DrivelineModel::advance(0,...)` with unlimited grip reaches a 0/0 acceleration calculation, but `EngineRuntime` deliberately skips driveline advance when paused and only calls it for positive simulation dt. No ordinary application trigger established; not counted.
- Existing PH-01 reverse power transmission, PH-04 fixed aero density, PH-06 stale clutch cooling atmosphere, and DR-04 live gearbox/config races were recognized and not duplicated.
- Knock-limit sweep labels points with target RPM after allowing actual RPM down to half the target; a valid engine/input that produces a materially wrong retained calibration has not been demonstrated. Measurement-quality lead only.
- Lifecycle/start-stop epoch reuse remains an unconfirmed source timing lead. The live audio unique_ptr ownership checks were later traced through actual UI/callback paths and recorded separately as source-proven A2-07 in `findings.md`; no race-detector result or observed crash is asserted.
- Dyno estimator/gate, brake-cycle continuity, hold/stepped/ramp sequencing, immutable ECU snapshot pinning, archive IDs, load governor, free-run capacity probe, gas-field mailboxes, cruise/shift/CVT/pass-by controllers and queue timestamp remapping were read in full. No additional confirmed defect established from those reads.

## Validation boundary

No standalone compile, simulation or test ran during root's CPU reservation. After release, the shared isolated probe compiled against authoritative Windows VS Release libraries and reproduced RT2-01 through the public DataLogger API. No remedies are proposed here.


---

Original scratch report: `out/audit2/audio/test-findings.md`

# Test oracle review

The 16 explicitly assigned test sources were fully read, including every line of `CoreTests.cpp` and `RealtimeRegressionTests.cpp`. Paths, hashes and per-file oracle/limitation notes are in `test-coverage.json`. This is source review; root owns the authoritative baseline execution. Bounded component-probe results are documented separately in `probe-validation.md`.

## T2-01 — The maximum-mix quality test bypasses the boosted mixer settings it claims to validate

- Priority: P2 — the named regression guard does not exercise its claimed maximum authored mix.
- Sources: `tests/RealtimeRegressionTests.cpp:2006-2086`, assertions `:2121-2144`; default mode `src/runtime/include/enginelab/runtime/EngineRuntime.hpp:111`; renderer mode branch `src/audio/src/RealtimeEngineAudio.cpp:1024-1057`.
- Trigger: run the existing `outputQualityRegression` test. `renderMaximumMix` constructs default `RealtimeAudioState`, sets high-frequency gain 2.5, noise gains 1.5 and source gains 2.0, and never switches `monitorMode` to `captureVoiced`.
- Evidence: the default is `physicalReference`. In that mode the renderer replaces high/low gains with 1, high/low authored noise with 0, and every positive source gain with 1. Consequently its finite/full-scale/flat-top/DC assertions execute without the authored boosts/noise claimed by the fixture and assertion messages. Volume 2 and dense source events still stress the physical-reference output, so the test is not wholly vacuous.
- Impact: a passing maximum-mixer test supplies no evidence that the authored maximum gains/noise satisfy those quality bounds. A regression confined to capture-voiced combinations can pass this guard. No claim that the shipping application currently clips is made.
- Confidence: high, from complete fixture and renderer source. No modified-test experiment was run during the root CPU reservation.

## Related missing triggers, not separate findings

- The offline cancellation test cancels at preflight with stems disabled; it never reaches cancellation with the open order-map CSV (A2-01).
- The valve-port reflection test computes frequency response for stationary coefficients and checks finite guards; it does not process an open-to-closed coefficient transition with retained state (A2-03).
- Exhaust jet-noise RMS assertions use a subsonic fixture and doubled subsonic flow; they do not compare process output to capped target power above the velocity limit (A2-04).
- Intake regression authors taper but checks nonzero/finite/reset/mean-flow behavior, not forward acoustic energy across unequal end areas (A2-05).
- The complementary crossover's all-pass assertion at `RealtimeRegressionTests.cpp:3237-3249` sums the low/high coefficient responses with identical phase; it does not include the simulator's low-band-only625us source delay (A2-08).
- DataLogger rewind fixture recovers a positive origin and does not exercise the subsequent negative-origin sentinel reset (RT2-01).
- Nonlinear duct passivity is checked for selected periodic sine inputs. The implementation's general passivity statement is not established by those cases; no arbitrary-signal counterexample has yet been demonstrated.

## Rejected cross-module lead

Whole-crankshaft copy in the edit classifier and runtime settings request does not establish stale accepted kinematics: `EngineSimulator::settingsReplaceable:1601-1610` rejects count, ID, rotation ratio, phase and positions before `EngineRuntime::applyLiveSettings` publishes the change. `replaceSettings` updates only inertia, friction and mass fields. No finding retained from this hypothesis.


---

Original scratch report: `out/audit2/audio/tool-findings.md`

# Tool audit continuation

All 34 assigned tool files were fully read (every tracked `tools/` file except the six explicitly reviewed by root). `tool-coverage.json` records SHA256 and per-path notes; unreviewed is empty. No build or simulation ran during root's baseline reservation. After CPU release, the bounded zero-match production CLI reproduced TL2-03 with no simulation selected; other tool findings retain textual/algebraic verification.

## TL2-01 — Cyclic variability silently discards the zero/negative-work cycles that create roughness

- Priority: P2 — the roughness measurement and its validation gate discard diagnostically essential cycles.
- Sources: `tools/CyclicVariabilityHarness.cpp:68-96` (`CylinderSeries`), collection in `measure`, final per-cylinder COV aggregation and literature gate in main.
- Trigger: a measured cylinder produces zero or negative net indicated work on some cycles (e.g. misfire/light-load). `observe` records the changed lastWork then drops work<=0 before adding to sum/sumSquares/count. Those are valid COV(IMEP) observations; only the final mean denominator must be positive.
- Evidence: for 20 alternating 0J/100J cycles, the actual filtering algorithm retains ten identical100J values and reports COV0. The complete20-sample series has mean50J and sample COVsqrt(20/19)=1.025978352. This is an algebraic reproducer of the owning function, not a simulation-derived oracle. Negative-work cycles have the same exclusion. Additionally, detecting a new cycle only through a changed scalar work value cannot observe consecutive exactly equal completed cycles; this further alters weighting.
- Impact: the measurement/gate can understate rough combustion and misrepresent the intended literature comparison. Depending on retained sample count and bands it can fail for the wrong reason or pass an otherwise rough series. Catalogue-wide prevalence has not been measured.
- Confidence: high for the deterministic measurement defect. Ordinary model misfires are source-supported; no specific catalogue failing run is asserted.

## Rejected lead — Physical pressure-probe component coordinates

Cross-review refuted the proposed extension of audio finding A2-06 to `PressureProbeHarness`. `ExhaustGraph.cpp:395-430` emits one graph node per authored component, retaining both inlet and outlet diameter. `src/gas-dynamics/src/ExhaustNetworkLayout.cpp:198-228` then emits one varying-area physical duct per graph node. Unlike the audio compiler, the physical layout does not tessellate a cone into several same-ID ducts. The first matching physical gas-field element therefore spans the whole component, and the tool's whole-length fraction is correct. A2-06 is limited to the independently tessellated audio network.

## TL2-02 — Every drive-scenario CSV row contains five unnamed fields

- Priority: P2 — ordinary exported telemetry has a malformed named-column schema.
- Sources: `tools/DriveScenarioHarness.cpp:353-357` header, `:493-506` rows.
- Trigger: use `--csv` for any nonempty normal driving scenario.
- Evidence: header has28 names and ends at `egt_c`. The row format emits33 values, with lambda, AFR validity, MAP, wastegate opening and injector duty appended after the28 named fields. There are no corresponding header names. This is directly countable from the complete format string and argument list.
- Impact: output is not a rectangular named dataset; dictionary readers expose the last5 values under an anonymous/extra-field key, and consumers requiring header/row agreement reject it. The five diagnostically useful quantities cannot be selected by their names. First28 fields retain their positions.
- Confidence: high source evidence. A full scenario was not executed during the baseline reservation; no claim of physics failure follows from the CSV defect.

## TL2-03 — A manufacturer-reference check reports success after selecting no engines or points

- Priority: P2 — a mistyped validation selector can produce an empty successful check.
- Sources: `tools/DynoSweepHarness.cpp:567-691` reference-validation selection and final unmatched check; main returns its result in reference mode.
- Trigger: supply a valid nonempty manufacturer reference CSV and a `--filter` string matching neither the catalogue engine names nor reference engine filters.
- Evidence: `passed` begins true. The catalogue loop skips every engine. The final unmatched-reference loop skips every point whose engine filter is unrelated to the requested filter. No point executes and nothing sets passed=false. The function returns true, so the process reports successful exit with only a CSV header.
- Impact: a mistyped selector can turn an intended reference-validation run into a successful empty check. This is distinct from first-pass V07, which describes a designed-engine performance comparison intentionally wired to report-only mode.
- Actual reproduction: `EngineLabDynoSweepHarness.exe --catalog-root . --reference-file tests/reference-data/catalog-manufacturer-points.csv --filter audit-no-engine-match-20261116` returned exit0 and only the reference-results CSV header; exact output is in `zero-match-cli.txt`. No engine simulation was selected.
- Confidence: high source evidence and production CLI reproduction.

## TL2-04 — The purported X-only exhaust control also removes 1.21 metres from each LS3 bank

- Priority: P2 — the named single-factor validation changes major geometry in addition to crossover coupling.
- Sources: `tools/GeometrySensitivityHarness.cpp:768-815` (`twin-4into1-no-x`); current `engines/03_gm_ls3_like.engine.yaml:75-88` and connection list.
- Trigger: compare the shipped LS3 reference against the named `twin-4into1-no-x` geometry variant.
- Evidence: the variant retains only cylinder primary IDs, two bank merge IDs, muffler IDs and outlet IDs. It deletes all other components, then connects each bank merge directly to a muffler. The current LS3 has140mm pipes211/212 before the X and1070mm pipes311/312 after it. Those four pipes are also deleted. A bank route changes from690+620+140+1070+400+80=3000mm to690+620+400+80=1790mm. The X itself has zero authored length.
- Impact: a comparison advertised as bypassing only the common X section changes each route by1210mm (40.3percent), plus its mean-flow volume, propagation delay and losses. Differences cannot be attributed to crossover coupling. The two renders still compare actual configured networks, but the named single-factor interpretation is false.
- Confidence: high from the complete tool rewrite and current authored graph. No listening/performance magnitude is asserted; no heavy render is needed to establish changed geometry.


---

Original scratch report: `out/audit2/audio/docs-findings.md`

# Remaining audio/exhaust document review

Both assigned documents were fully read. `docs-coverage.json` records their current hashes, complete line counts and source linkage. These observations extend existing audit V08 (documentation drift); they are not additional independent runtime findings.

## V08 extension — Exhaust designer apply behavior describes the former restart-only path

`docs/custom-exhaust.md:65-68` states that every valid application replaces the runtime and resets engine speed/thermal state. Current designer application reaches `MainComponent::applyExhaustEdit` (`src/app/src/MainComponent.cpp:1289-1294`, `:707-722`). A compatible edit first calls `EngineRuntime::applyLiveExhaust` (`src/runtime/src/EngineRuntime.cpp:500-539`) and publishes a replacement gas network; the simulation thread consumes it through `consumeLiveExhaustChange` (`:808-824`). The audio network is separately crossfaded through `replaceExhaustGraph`. This successful live path does not rebuild the runtime or reset engine speed. A fallback restart remains for incompatible changes, and the dyno restriction remains enforced. This is a user-facing behavior mismatch, not evidence of a reset defect.

## V08 extension — Realtime flow diagram omits the third production queue

`docs/realtime-audio.md:11-16` describes two SPSC queues as the simulation-to-callback flow. Current runtime declares `ExhaustAcousticQueue` in addition to event and cylinder-pressure queues (`src/runtime/include/enginelab/runtime/EngineRuntime.hpp:30-37`); it forwards the simulator's finite-volume medium/outlet/reaction stream at `src/runtime/src/EngineRuntime.cpp:1605-1617`. The renderer consumes that third queue at `src/audio/src/RealtimeEngineAudio.cpp:954-967`. The omission hides the ownership and timing path for distributed duct media, outlet boundaries and chemical reaction events. No additional queue-correctness defect follows from the documentation omission.

## Reviewed limits

The custom-exhaust document correctly distinguishes one varying-area physical gas duct from the audio cone's up-to-four sections (`:513-517`), and this distinction refutes the earlier physical PressureProbe extension of A2-06. Schema 9 agrees with `currentEngineSchemaVersion`; free-field behavior without an explicit IR agrees with current loading. Runtime-budget rejection claims agree with the harness's explicit `renderOverBudget` and other counter gate. Dated numerical measurements and source citations were read as historical evidence; no new performance run, upstream-source verification or listening assessment was performed.


---

Original scratch report: `out/audit2/app/findings.md`

# Application audit, second pass

All 55 tracked app files have been fully read as text, or (the five third-party TTF resources) inspected through fontTools table/name metadata and the matching complete bundled OFL notices. The additional complete render/exhaust/test/history scopes contain 24/14/29/2 tracked paths, respectively; their manifests record all 124 current hashes. No tracked source changed. The baseline CPU reservation was respected; after release, bounded standalone model/component probes were compiled against current source/Release objects and libraries and executed. No live app-control tool, whole-engine sweep, real recording or listening test was used by this subreview. Existing audit.md findings are not repeated below.

## Finding status and count

There are **19 counted findings: one P1, thirteen P2 and five P3**. All are supported by current source; the matrix distinguishes executed corroboration from source-only paths. Additional leads and coverage observations are excluded from this count. TEST-01 and TEST-02 count independently because their implemented acceptance conditions demonstrably permit outcomes excluded by their stated oracle. Mathematical witnesses do not assert an actual current engine regression or differing saved render.

| ID | Severity | Current evidence | Verification limits |
| --- | --- | --- | --- |
| APP-01 | P1 | Complete save-name/restart/selection source proof | Integrated sequence not executed; no user file overwritten |
| APP-02 | P2 | Actual designer: true CFL cost ratio 5.971, UI says unchanged | No complete-engine cost measurement |
| APP-03 | P2 | Live-edit refresh omissions and cached export request source proof | Integrated export sequence not executed |
| APP-04 | P2 | Actual workshop: B's 0.03 COV replaced by A's 0.11 | No running engine or audible comparison |
| APP-05 | P2 | Focus-preserving Escape plus later commit source proof | Focused component event sequence not executed |
| APP-06 | P2 | Debounced update and captured build configuration source proof | Timed designer/worker sequence not executed |
| APP-07 | P3 | Actual K20A component PNG and button bounds | No full app scrolling interaction |
| APP-08 | P2 | Actual CrankClock and source shaft formula disagree | No GL execution |
| APP-09 | P2 | Actual TopBar at 1280 px: A/Log/Exhaust overlap | No live app interaction |
| APP-10 | P2 | Actual 652 px ReadoutStrip PNG clips ordinary numbers | No full app interaction |
| APP-11 | P3 | Actual minimum PassByWindow: Level has zero height | Default window fits |
| APP-12 | P2 | Actual PartInspector plus unchanged checked owning source: vector bound failure | Instrumented standalone failure; no normal whole-app crash asserted |
| APP-13 | P3 | Actual second-choice inspector PNG obscures note | Controlled component; real cam/carb branches independently traced |
| RENDER-01 | P2 | Actual model plus synthetic graph field binds midpipe to primary node | No live pressure-wave display run |
| EXHAUST-01 | P2 | Actual public removePath: valid before/after but mappings reverse | No simulation or sound comparison |
| EXHAUST-02 | P2 | Actual valid scalar/editable graphs have different routes | No power or sound measurement |
| RENDER-02 | P3 | Actual model ratio 0.1 renders 9/41 = 0.219512 | No physical speed-error claim |
| TEST-01 | P3 | Both acceptance predicates plus mathematical counterexample | Current engine incidence is not asserted or required for the oracle defect |
| TEST-02 | P2 | Sample-identity claim reduced to two scalar statistics | Actual differing saved renders are not asserted or required for this validation limit |

## APP-01 — An exhaust designer opened before Save As can restore the original engine identity

**P1, source proof; integrated GUI sequence not executed.** Open the Exhaust designer for saved engine A, Save As B in the main window, then apply a valid topology-changing exhaust edit from that still-open designer. DesignerContent owns the whole working EngineConfig, including its original name (ExhaustDesignerWindow.cpp:522-524,569-582,2087). Main's callback directly calls applyExhaustEdit (MainComponent.cpp:1289-1294), whose restart fallback passes that complete old configuration to applyConfig (708-709). Save As updates config_.name/runtime and the preset selection without updating that window (1793-1814). applyConfig reselects by the restored name (523), and the next Save writes the selected preset's file (1698-1705). **Impact:** a user can be returned from B to A and subsequently overwrite A with B's intended changes. The general name-preservation rule at581-589 is bypassed on this callback. This is distinct from the filename collision DR-01 and the excluded by-value reference-alias suspicion. No real user file was overwritten in this audit.

## APP-02 — The successful exhaust apply erases the baseline before calculating its cost warning

**P2, actual component reproduced.** On a successful designer apply, applying_ remains true while its synchronous callback runs (ExhaustDesignerWindow.cpp:2084-2092). Both Main's live exhaust path (MainComponent.cpp:724) and full replacement path (488) synchronously call designer.setConfig(config_), which overwrites working_ and baseline_ with the same applied config (ExhaustDesignerWindow.cpp:576-582). solverCostAdvisory subsequently compares their minimum CFL lengths (2043-2065). The standalone actual designer callback replicated that synchronous setConfig: original minimum CFL length 59.71 mm, applied 10 mm, ratio 5.971; its actual status read "Limiting CFL length 10.0 mm versus 10.0 mm before (solver cost unchanged)." **Impact:** the relative solver-cost warning misses an edit that raises the implied global substep cost. No complete-engine running cost was measured. Evidence: component-output.log.

## APP-03 — An open Audio Workshop exports stale live-edited engine geometry

**P2, source proof; integrated exporter interception not executed.** Open Audio Workshop, change exhaust/intake/cylinder/turbo settings live elsewhere, then export without reopening the workshop. Main's live edit paths (MainComponent.cpp:646-745) update config_, the runtime and 3-D view but do not update the workshop's engine copy. WorkshopContent stores an EngineConfig and copies that cached engine_ into both offline export requests (AudioWorkshopWindow.cpp startExport/startLoopPack). Full engine replacement and reopening Audio Workshop do refresh it (MainComponent.cpp:513-517,1148-1152). **Impact:** the resulting render can use the pre-edit exhaust/intake/geometry while the running engine and picture use the edit. This is separate from UA-01's lost ECU tables.

## APP-04 — Engine switching leaves Audio Workshop physics controls on the preceding engine

**P2, actual component reproduced.** WorkshopContent constructor fills the physical sliders through setPhysicsInternal(audioPhysicsSettingsFor(engine_)). Its later setEngine replaces engine_, availability and the small afterfire switch, but never fills the detailed physical sliders (AudioWorkshopWindow.cpp setEngine). Applying physics, or clicking the immediate mixer afterfire switch, constructs all AudioPhysicsSettings from those retained sliders and sends them to the new running engine. The actual window was constructed with engine A's cycle-variation COV 0.11, switched through public setEngine to B with COV 0.03, and its actual APPLY LIVE callback emitted 0.11. **Impact:** physics settings from A can be written into B while its label correctly identifies B. A full replacement's setMix refreshes only audio faders. Evidence: component-output.log; no running engine or sound comparison.

## APP-05 — Escape in a vehicle numeric field does not cancel its pending value

**P2, source proof; focused-component sequence not executed.** VehiclePanel's Escape callback clears shownOnce_ and calls refresh. refresh deliberately skips every focused text editor, then marks shownOnce_ true again. The editor therefore retains the typed value; its later onFocusLost callback commits it through onEdit (SidePanel.cpp VehiclePanel constructor, commit and refresh). **Impact:** pressing Escape followed by clicking elsewhere can apply the vehicle change the user attempted to cancel. This path requires the editor to retain keyboard focus across Escape, as its callback does; it has not been exercised against the main app.

## APP-06 — Build can use the previous designer state during its 250 ms text debounce

**P2, source proof; timed designer/worker sequence not executed.** Text fields queue redesign after 250 ms (EngineDesignerWindow.cpp addField and field-specific callbacks). build() immediately copies designed_.config, with no redesign before the copy, and checkRealtimeThenBuild captures that config by value into its worker. **Trigger:** type a final name/count/dimension change and click Build before that timer runs. **Impact:** the engine eventually built and saved can be the preceding design even if the delayed preview subsequently shows the new values. A capacity/knock sweep was not run just to exercise this UI race.

## APP-07 — VTEC adds a telemetry row without increasing its layout or content height

**P3, actual component snapshot and bounds confirmed.** engineRows adds a VTEC row when cams are fitted, producing 14 rows instead of 13. TelemetryPanel's preferredHeight and resized still use the constant engineRowCount=13 (SidePanel.cpp:98-99,163-166,185-198). paint consumes the actual list (253-259). The actual K20A panel's preferred height is 1136 px; Fuel/Show in 3-D start at y624, 24 px above the actual Fuel injection heading and overlap the last engine row by 6 px. The final Hold speed row gets an empty rectangle after the fixed content area is consumed and is wholly absent from the snapshot. **Impact:** controls are displaced and one telemetry row cannot be reached by scrolling the measured content. Evidence: component-output.log and [K20A telemetry PNG](app/telemetry-vtec.png).

## APP-08 — Strobe playback keeps turbo and gearbox shafts moving while paused

**P2, source-confirmed arithmetic with actual clock corroboration.** EngineViewport sets Shared.rate to zero when RuntimeHealth.paused is true, but keeps sampled shaft RPM values (EngineViewport.cpp:3059,3066,3070). renderOpenGL has no outer pause gate: it advances the clock then separately integrates each shaft. In strobe mode it multiplies shaft RPM only by playbackFactor, omitting rate (3263-3269), whereas CrankClock::displayDegreesPerSecond always multiplies rate by the selected factor (CrankClock.cpp:53-54). At 3000 rpm and factor 1/250, the actual clock produced 0 degrees/s while paused and 18 degrees/s at rate 0.25; the viewport's shaft formula is 72 degrees/s for both. **Impact:** paused strobe crank and shafts disagree, and fractional simulation speed gives incorrect shaft-to-crank speed. Evidence: model-output.log. No OpenGL run was used.

## APP-09 — The minimum-width toolbar places A/B, Log and Exhaust on top of one another

**P2, actual component bounds and render confirmed.** MainWindow supports 1280 px width (src/app/src/Main.cpp:36), and MainComponent gives TopBar the entire width (MainComponent.cpp:2266-2269). TopBar reserves fixed brand/picker/status widths, then consumes its remaining rectangle from the right with fixed button widths (TopBar.cpp:215-231). At 1280, actual A, Log and Exhaust bounds are respectively [606,10,33,34], [599,10,47,34] and [600,10,93,34]. **Impact:** Exhaust hides A/B and almost all of Log while placing their hit rectangles underneath it. Pointer interaction was not executed. At 1440 the large occlusion is gone; A/Log still overlap by 2 px. Evidence: component-output.log, [1280 toolbar PNG](app/topbar-1280.png), [1440 comparison PNG](app/topbar-1440.png).

## APP-10 — Primary numerical readouts are clipped at the supported main-window minimum

**P2, actual component render confirmed.** At Main's minimum width 1280, its fixed 248 px controls and 380 px side panel leave ReadoutStrip 652 px (MainComponent.cpp:2266-2280; ControlPanel.hpp:91; SidePanel.hpp:120). ReadoutStrip retains a minimum 200 px tachometer plus six cells, fixed insets and 26/34 px numeric fonts (ReadoutStrip.cpp:19-41,48-109,114-123). The actual 652x92 render with ordinary state values rpm 3250, torque 180 Nm, power 62 kW, exhaust 722 C visibly clips the RPM's last digit, several numeric/unit strings and labels. **Impact:** the prominent values can be read as incomplete numbers at a supported size; units and context also disappear. Evidence: component_probe.cpp state setup and [minimum readout PNG](app/readouts-minimum.png). No physics-value error or full app manipulation is asserted.

## APP-11 — The Pass-by minimum height removes the microphone level control

**P3, actual window bounds and component render confirmed.** PassByWindow accepts height 560 (PassByWindow.cpp:401-403), while its fixed rows consume more vertical space (290-342). Resizing the actual standalone window to 760x560 gives content [0,0,760,560], Ground [18,518,150,28] and Level [18,546,150,0]; the corresponding Level segmented control also has zero height. **Impact:** Real/Normalised microphone-level selection disappears at an accepted window size. The actual default window is 762x628 and fits; this is specifically the supported minimum, not a default-size defect. Evidence: component-output.log, [minimum PNG](app/passby-window-minimum.png), [default PNG](app/passby-window-default.png).

## APP-12 — A last-ratio click can read beyond the resized inspector values

**P2, actual component reproduced under vector bounds instrumentation.** PartInspector::beginTyping validates index against pending_, calls sendEdit synchronously, then reads pending_[index] without revalidating (EngineViewport.cpp:898-903). sendEdit invokes its owner's callback (590-596), which can synchronously replace its fields through setEdit (599-629). A forward-gear-count reduction rebuilds the inspector with fewer ratio fields (2630-2642,2683-2695), and its public stepper debounces for 250 ms (701-713). **Trigger:** reduce Forward gears with its stepper, then click the old last gear's numerical value within that interval. The controlled actual component used eight fields, queued count 3→2, and its owner callback synchronously set the resulting seven fields. The original click resumed and current, unchanged EngineViewport.cpp hit MSVC14.44's supported hardened-vector check: "vector subscript out of range", exit86. **Impact:** the UI reads outside the logical vector after a supported reentrant field-list change. Evidence: part-reentry-output.log, part_probe.cpp, checked_vector.hpp and build-part-probe.ps1. This proves the instrumented bounds failure; it is not an observed crash of the normal whole app, and no arbitrary memory disclosure is claimed.

## APP-13 — A second inspector choice draws the authored note beneath the buttons

**P3, actual component render confirmed; real owner branches traced.** editHeight and resized reserve two button rows when secondChoice is present (EngineViewport.cpp:755-784), but paintEdit skips only one before drawing note/status text (788-827). Actual cam edits always supply secondChoice plus a note (2304-2316); the carburettor branch also supplies both (2796-2807,2840-2842). The controlled actual component rendered the second choice at y128, first choice/Reset at y158, and its note starting on that same y158 row; the controls obscure its first line. **Impact:** cam/jetting instructions and edit-status text can be covered by the controls they describe. Evidence: part-layout-output.log and [second-choice inspector PNG](app/inspector-second-choice.png). The snapshot is an isolated representative edit; no whole-app cam interaction was performed.

## RENDER-01 — Scalar midpipe gas colouring binds the first primary's field

**P2, actual model plus synthetic graph field reproduced.** In bindGasField's scalar exhaust branch, a pipe receives a cylinder ID only when its editable component is directly cylinder-mapped. A midpipe has no such mapping, so the final predicate accepts the first same-path pipe element (GasFieldView.cpp:64-80). ExhaustGraph's scalar compiler emits cylinder primary pipes before its downstream midpipe. The actual model of a supported scalar configuration bound its 200 mm midpipe (component6) to graph node1, the first 777 mm primary, and returned that primary's deliberately unique synthetic pressure 100777. **Impact:** downstream pressure/temperature colouring and hover probes can describe a primary rather than their own solver element. Authored network paths use component IDs and avoid this branch. Evidence: model-output.log/model_probe.cpp; no live wave display was run.

## EXHAUST-01 — Removing a path can silently reassign the remaining turbochargers

**P2, actual validated-config reproduction confirmed.** ExhaustPathTopologyEditor::removePath erases a source path and commits the shortened vector without resolving default turbocharger=-1 assignments (ExhaustPathTopologyEditor.cpp:613-653). Those defaults mean path index modulo turbochargerCount. The public API accepted a valid three-path/two-turbo configuration [path1→0,path2→1,path3→0], removed path1 into path3 without error, and produced a still-valid [path2→0,path3→1]. **Impact:** both retained paths' turbine assignments reverse when a preceding path is deleted. joinExhaustPaths explicitly preserves these default assignments before erasing (ExhaustComponentResize.cpp:236-244); removePath does not. Evidence: model-output.log; no physical/sound effect measured.

## EXHAUST-02 — Converting a supported scalar exhaust to editable geometry changes its model

**P2, actual valid-graph comparison confirmed.** The direct scalar compiler respects CylinderConfig.exhaustPrimaryLengthMm (ExhaustGraph.cpp:546-562), includes the legacy collector trunk (576), gives an unconfigured muffler a 450 mm body (612-614), and emits a 180 mm outlet (636). makeEditableExhaustNetwork instead uses only path.geometry.primaryLengthMm for every cylinder (LegacyExhaustNetwork.cpp:31-37), sets a collector with explicit volume to zero length (45-46), omits a non-expanding/unconfigured muffler (93-96), and gives the outlet zero length (132-134). The normalizer does not materialize scalar paths; JSON/YAML/script/default configurations can retain this supported representation, although current catalog engines already have authored networks. The actual before/after graphs were both valid with no diagnostics: cylinder1 route 1727→600 mm and others 1350→600 mm on conversion alone, with corresponding graph-delay changes. **Impact:** a conversion described as neutral changes the modeled routes without an authored geometry edit. Evidence: model-output.log. No audibility or power claim; existing neutral tests compare selected properties rather than complete routes.

## RENDER-02 — Valid low gearbox ratios are drawn with a different gear ratio

**P3, actual model reproduced.** EngineModel3D clamps every ratio used to size gear pairs to at least 0.2 (EngineModel3D.cpp:1112-1113,1204), while configuration/edit bounds accept ratios down to 0.05. Tooth counts and output rotation derive from that clamp. An accepted custom gearbox {1.0,0.1} produced actual displayed ratios 1.0 and 9/41=0.219512195122. **Impact:** the second gear's displayed meshing/output ratio differs from its configured 0.1. Evidence: model-output.log. No effect on simulated vehicle speed is asserted.

## TEST-01 — The idle slow-coast exception accepts a high, flat recovery

**P3, source-confirmed false-acceptance condition; no current engine regression asserted.** The regression documents its RD350 exception as applying only while the final sample remains below the final two-second mean, so a high hanging speed still fails (tests/IdleStabilityRegressionTests.cpp:505-513). recoverFromBlip returns the minimum over the entire recovery, not its final sample (57-78), and both normal and raised-idle gates instead compare that minimum with the final mean (591-595,630-634). **Reproduction by the actual predicate:** for an allowlisted name and target 1,000 rpm, a recovery that dips to 900 then rises to a flat final 2 s at 2,000 has minimum 900 and final mean 2,000. The minimum exceeds the separate 780 rpm stall threshold, slowCoastAllowed becomes true, and the above-1,150 rpm return check is bypassed. Its final sample equals the mean, so it fails the documented coasting condition. **Impact:** this gate can report success for the named engine hanging high after a blip. This counterexample establishes the oracle defect; it does not claim that the current RD350 simulation follows it or that other independent checks fail.

## TEST-02 — Saved-audio sample identity is accepted from only RMS and peak

**P2, source-confirmed validation limitation with mathematical false acceptance; no differing saved render asserted.** checkSavedSound explicitly describes the reloaded shipping render as sample-for-sample identity (tests/SavedEngineTests.cpp:401-402) and reports that the saved engines render the same audio (427). Its sound helper discards the waveform and returns only {masterRms,masterPeak} (373-398); the sole audio comparison is equality of that pair (421-425). **Reproduction of the information loss:** equal-length sample arrays [1,-1,0,0] and [1,0,-1,0] differ at two sample positions but both yield RMS sqrt(0.5) and peak 1, so this comparison accepts them. Any sample permutation has the same two summary statistics. **Impact:** this regression cannot establish the exact saved-audio identity it reports and can accept timing, phase or channel-content changes that preserve these aggregate values. No current original/reloaded engine export is claimed to differ.

## Additional source leads, not counted as confirmed findings

- showDynoStats_ is toggled by the mapped action but has no reader; the key has no visible effect in this source.
- The EngineDesigner numeric helpers accept repeated decimal points and use getDoubleValue rather than complete-token parsing; firing-order values are cast to uint32 without a whole-number check. Parser behavior has not yet been executed.
- The 2-D fallback has no rotary branch. A rotary configuration runs through the non-radial piston/rod drawing and regards it as valved because only two-strokes suppress valves. No fallback screenshot is yet attached.
- The former PartInspector and minimum-layout leads were executed and either counted narrowly as APP-09 through APP-13 or limited by the measured default-window comparison.

## Checked/refuted leads

- Completed dyno runs are appended and then cleared from currentRun_ in EngineRuntime::finishDynoSession; the suspected persistent duplicate after hiding/deleting an archived curve is refuted (cross-checked with runtime reviewer).
- Key-bindings modal ownership was checked against JUCE SafePointer deletion after callbacks; the suspected double deletion from a callback reset is refuted.

- Cached audio-path availability remains valid after successful live replaceForcedInduction/replaceIntake: an actual machine/path-presence change returns false and Main rebuilds the renderer (cross-checked with audio reviewer).
- Mute/solo zeros work in physical-reference mode: RealtimeEngineAudio preserves requested<=0 as zero and normalizes only positive gains (cross-checked with audio reviewer).
- FuelWindow does merge other current engine edits while it holds pending fuel edits; its pending working-copy path was read and is not described as blanket stale-copy loss.
- No font copyright/license inconsistency was found in the inspected embedded metadata and bundled notices. Redistribution packaging is outside app/CMake's ownership and is reviewed at the root.

## Test-review limits and validation observations

All 29 files in the exact app-tests partition were fully read. Hashes are in tests-coverage.json. Their baseline execution belongs to the root; the bounded model/component and mathematical witnesses described here were executed independently after its CPU release.

- TEST-02 separately counts SavedEngineTests.cpp's sample-identity claim being decided from only RMS and peak. It is not evidence that current saved audio differs.
- EngineModel3DTests.cpp checks uniqueness of gas-field bindings for catalog engines, which already have authored networks. Its scalar copy at the end checks resize and source placement but not gas binding; the scalar-midpipe lookup above is outside that exercised branch.
- AudioWorkshopTests.cpp exercises setEngine on the same engine with changed availability flags; it does not switch between engines with different physics settings. Its exhaust apply callback stores the configuration without MainComponent's synchronous setConfig response, so the relative-cost baseline replacement above is not exercised.
- TEST-01 separately counts the idle slow-coast predicate's concrete false-acceptance condition. No present RD350 failure is inferred from the predicate alone.
- The selected live remesh check explicitly leaves the CP2's residual long-settling discrepancy as an open question and applies the effect-relative comparison to LS3 only. The baseline result therefore does not establish remesh equivalence for every engine.

## Historical documentation cross-check

Both docs/journal.md (2,009 lines) and CHANGELOG.md (1,554 lines) were fully read; history-coverage.json records their current hashes. They are historical provenance, not measurements rerun by this audit. The journal's dated reversals resolve many earlier hypotheses: the two-stroke hold band became a frame-sampling alias; the Merlin click received separate high-band and delayed-publication fixes; the first intake oracle was explicitly retracted as a stale binary; constant-gamma, knock and friction conclusions were revised after later experiments. Those early symptoms are not counted again as current defects.

The October 30 journal and 0.7.7 changelog describe engine-name restoration after Save As as fixed through general edit-name preservation. APP-01 identifies the current direct exhaust callback that bypasses that preservation; it is supported by current source, not only the older report. The histories also preserve numerous named, unmeasured limitations (real hardware/listening, ambient/rod state, scalar/original paths and provisional empirical fits); none has been promoted here to a fresh measurement.


---

Original scratch report: `out/audit2/audio/root-cross-review.md`

# Independent source-only review of root findings

Reviewed `root-findings.md` RT-01 through RT-08 and PKG-01/02 against the actual probe scripts/logs, targeted current owners, registered reference fixture and first-pass audit. No additional executable, heavy computation or production edit was performed.

## Precise objection

RT-07 spells the YAML fixture in Markdown with two backslashes before `n`, whereas the actual YAML fixture contains one backslash escape. The latter decodes to a newline and reproduces the invalid JSON; the former would decode to a literal backslash plus `n`. The finding should identify the accepted name as containing a newline. Its source mechanism, successful import/export and strict JSON rejection are confirmed. Calling it an accepted name is more precise than calling it an ordinary name.

## Supported findings and limits

- RT-01: `pop_metrics.analyse` returns three NaN medians for the ordinary no-pop witness; the CLI uses the same default `json.dumps` behavior. The probe invokes the unmodified function and the CLI's serialization expression, rather than claiming a separate CLI subprocess. P2 is reasonable for the advertised machine-readable output.
- RT-02: self modes enter `variant_mode`; the heading distinguishes the same engine's two conditions, but the shared body explicitly says two catalogue engines. The report correctly limits the defect to explanatory text. P3 is appropriate.
- RT-03: current theme font and colors support the recorded arithmetic and the selected small-text contrast benchmark. The report properly avoids a legal-compliance or measured-user-acuity claim. P3 is appropriate; this is a benchmark assessment, not evidence that a specific person failed to read the labels.
- RT-04: the 22 recorded rows contain exactly20 PASS and2 FAIL; both failed rows have successful hold gates and match the table. The spec deliberately leaves cam/flow/duct values to generic designer inference, and its provenance explicitly omits unsupported details. The report's present caveat correctly identifies an unmet project fixture target, not a fully specified real-engine prediction or isolated physics cause. First-pass V07 reports the report-only enforcement mechanism and explicitly asserts no new measured values; the new numerical result is therefore distinct evidence. P2 is defensible as a project plausibility target, subject to retaining those fixture limits.
- RT-05: the pressure-tap component's `stoul` is unguarded, with no consumed-length validation. The child exits0xC0000409 before loading an engine; the report correctly limits this to the CLI. P2 is appropriate.
- RT-06: `SceneExport` never checks the index stream before returning0. The directory-as-index/no-match witness isolates this failure without a model build. The report does not overclaim a separately reproduced scene-file failure. P2 is appropriate.
- RT-07: mechanism and actual invalid JSON are supported, subject to the escaping clarification above. P2 is appropriate for accepted-input serialization.
- RT-08: positive duration validation permits a zero-length buffer; both `.back()` calls follow without a size check. The recorded zero-sample crash and successful .001s control use the same component. P2 is appropriate for a bounded accepted-input tool crash.
- PKG-01: current CPack configuration selects the EngineLab component, matching the staged install. The seven staged notice files and current install declarations lack the two full dependency texts; the root report distinguishes this artifact fact from a legal judgment. I did not independently browse or certify upstream licensing in this source-only cross-review. P2 is reasonable.
- PKG-02: both shipped tools use the compile-time absolute source root. `loadEngineCatalog` directly checks that root's engines directory and has no executable-adjacent fallback; the exporter returns1 for no engines. The staged sentinel/default/explicit-root control proves the default ignores packaged catalogue edits. The restored staged K20A hash matches its pre-probe entry in `package-files.json`. The foreign-machine failure remains correctly conditional/source-proven, not a second-machine experiment. P2 is appropriate.

## Fresh scene log

All18 catalogue entries appear. Totals agree with individual rows:4 duct clashes (2JZ),4 self clashes (LS3 two, Hayabusa one, CP4 one),0 engine clashes,46 bend reports and6 stretch reports (2JZ two, LS3 four). These update evidence for existing R01/R02/R03; they do not establish18 independent visual inspections, audible errors, or new finding categories. No live visual confirmation was performed in this cross-review.
