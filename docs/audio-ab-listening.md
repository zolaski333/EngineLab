# Blind A/B listening protocol

A spectral measurement does not prove that a sound is realistic. This protocol
produces blind pairs between EngineLab's real real-time audio path and
recordings of real engines, with matched duration and loudness.

## Instrument

The `EngineLabAbClipRenderer` target is defined by
[`tools/AbClipRenderer.cpp`](../tools/AbClipRenderer.cpp).

```powershell
cmake --build out/build/windows-vs2022 --config Release `
  --target EngineLabAbClipRenderer -- /m:1 /nr:false
```

For each engine, the tool:

1. renders `RealtimeEngineAudio` with the cylinder pressure telemetry, the
   physical exhaust graph and the default voicing;
2. follows `start -> held idle -> rev-up to the limiter -> cut -> engine
   braking`;
3. decodes the WAV, AIFF, FLAC or OGG reference with JUCE;
4. resamples it to 48 kHz if needed;
5. selects the window defined by the manifest and trims both sides to the same
   duration;
6. applies the same 20 ms fades;
7. matches each side by ITU-R BS.1770 integrated loudness (K-weighting,
   absolute gate -70 LUFS, relative gate -10 LU);
8. if a side would exceed a 0.98 peak, attenuates **both sides by the same
   factor**. Loudness equality is thus kept without clipping;
9. randomises the A/B position with a reproducible seed.

The comparison therefore reveals the side neither by its duration, nor by its
volume, nor by clipping added during normalisation.

## Real CC0 corpus

The versioned manifest is
[`references/real-engine-audio/manifest.json`](../references/real-engine-audio/manifest.json).
Its schema 2 makes the following mandatory: provenance, redistribution rights,
SHA-256, nature of the take, operating conditions, RPM status, microphone,
asset quality and the real strength of the match. An unknown datum stays
literally `unknown` or `null`: the corpus does not turn a guess into a
measurement.

The script only downloads `redistributable` entries automatically, rejects
absolute paths or paths leaving the corpus, then checks the SHA-256 before
replacing the final file. The assets stay ignored by Git.

```powershell
powershell -ExecutionPolicy Bypass `
  -File references/real-engine-audio/fetch-corpus.ps1

out/build/windows-vs2022/tools/Release/EngineLabAbClipRenderer.exe `
  --reference-manifest references/real-engine-audio/manifest.json `
  --validate-manifest
```

The second command reads the ten files back, recomputes their SHA-256 and
actually decodes them with JUCE. The renderer repeats this integrity check
before every A/B render.

| Key | Real recording | Known conditions | Declared match |
|---|---|---|---|
| 2JZ | Toyota Supra turbo on a dyno, editboy23 | loaded sweep | platform proxy, exact engine not stated |
| LS3 | unknown V8 in binaural, overmedium | idle, blips | architecture proxy |
| Hayabusa | Suzuki GSX1300, Heigh-hoo | start, idle, pull-away | exact platform |
| Big Twin | Harley-Davidson, allencote | start, idle, blips | family proxy |
| EJ25 | 2003 Subaru Impreza WRX turbo-back, ulose2piranha | idle, blips | boxer/turbo proxy |
| K20 | 2012 Honda Civic, thepodcastdoctor | start, idle, blips | naturally aspirated I4 proxy, engine not identified |
| Merlin | Spitfire Merlin on a static stand, squashy555 | static run without propeller | exact platform, load not matched |
| Aircooled | Porsche 911 on the street, mharo | street pass-by | platform proxy, generation unknown |
| CP3 | Yamaha MT-09/FZ-09 on track, richwise | pass-by under load | exact platform |
| Radial | radial-engined biplane, craigsmith archive | in-flight pass-by | archive radial proxy, cylinder count unknown |

These levels are deliberately conservative. A Supra not documented as a 2JZ, a
2003 WRX not documented as an EJ25 or a 911 of unknown generation does not
become an exact reference through mere resemblance.

## Complete run — 2026-07-29

Command:

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAbClipRenderer.exe `
  --output out/validation/listening-corpus-v2-2026-07-29 `
  --engines "2JZ,LS3,Hayabusa,Big Twin,EJ25,K20,Merlin,Aircooled,CP3,Radial" `
  --reference-manifest references/real-engine-audio/manifest.json `
  --require-references `
  --seed 20260729
```

Result:

| Pair | Duration | EngineLab LUFS | Reference LUFS | Max peak | Match |
|---|---:|---:|---:|---:|---|
| 2JZ / Supra | 10.300 s | -20.000 | -20.000 | 0.708 | platform proxy |
| LS3 / V8 | 8.530 s | -20.000 | -20.000 | 0.467 | architecture proxy |
| Hayabusa | 10.300 s | -20.000 | -20.000 | 0.601 | exact platform |
| Big Twin / Harley | 10.300 s | -20.000 | -20.000 | 0.743 | family proxy |
| EJ25 / WRX | 10.300 s | -21.649 | -21.649 | 0.980 | family proxy |
| K20 / Civic | 10.300 s | -20.000 | -20.000 | 0.626 | architecture proxy |
| Merlin | 10.300 s | -20.000 | -20.000 | 0.284 | exact platform |
| Flat-6 / 911 | 10.004 s | -20.000 | -20.000 | 0.674 | platform proxy |
| CP3 / MT-09 | 10.300 s | -20.441 | -20.441 | 0.980 | exact platform |
| Radial / biplane | 10.300 s | -20.000 | -20.000 | 0.807 | architecture proxy |

EJ25 and CP3 hit the common peak ceiling. Both sides of each pair are then
attenuated by the same factor: loudness stays equal, favouring neither EngineLab
nor the reference.

All twenty WAVs were written. For each pair, A and B have exactly the same size.
The complete key, including the schema 2 metadata, lives outside the folder
handed to the listeners: `listening-key.json` at the root of the output folder.

## Layout and key

- The blind files are `clips/pair_N_<segment>_A.wav` and
  `clips/pair_N_<segment>_B.wav` — one pair per (engine, condition).
- `clips/INDEX.md` comes with the clips and names the **condition** of each
  pair, never the engine or the side: a listener must know whether they are
  hearing an idle or a rev-up, and must not know that pair 7 is a Merlin.
- The position of the tested side is drawn with `--seed`. If every pair lands
  on the same side, the draw is repeated.
- `listening-key.json` stays outside `clips/`. It holds the mode, the engine
  name, the segment, the sides (`subject_side` / `control_side`), the duration,
  the LUFS before/after, the peaks, the loudness of the whole trajectory, the
  level error removed, and for the control side: its nature (`control.kind`),
  provenance, licence, SHA-256, original sample rate, conditions, RPM status,
  microphone, asset quality, match quality and `window_condition_matched`.
- `--require-references` fails before any render if a file is missing, if its
  SHA-256 differs or if it cannot be decoded.
- `--validate-manifest` checks the metadata, integrity and decoding of the whole
  corpus without running the simulation.
- A reference given manually with `--ref key=path` is marked
  `MANUAL / UNVERIFIED` in the key: the tool does not invent a provenance.

## Human procedure

1. Use the same neutral headphones or the same monitors and stop touching the
   volume.
2. Give only the `clips/` folder to the listeners.
3. For each pair, score separately:
   - realism of A and B from 1 to 5;
   - preference for A and B from 1 to 5;
   - forced choice "most realistic" A/B;
   - forced choice "preferred" A/B;
   - free comment.
4. Ideally use at least five listeners and a different seed per session.
5. Then score with the key. A qualitative result is only announced after these
   listening sessions; the build and the metrics alone do not mean "better
   sound".

## Response sheets and scoring

`scripts/listening.py` (standard library only) handles both ends.

```powershell
# 1. make the blank sheets (never reads the key)
python scripts/listening.py sheets --pack out/validation/listening-pilot --listeners 5

# 2. once filled in, score them
python scripts/listening.py report --pack out/validation/listening-pilot
```

`sheets` writes `responses/listener_NN.csv`, one row per pair, with the columns
`pair, condition, realism_A, realism_B, preference_A, preference_B,
most_realistic, preferred, comment`. The `condition` column is **pre-filled**
from the key so the listener knows whether they are scoring an idle or a
rev-up; it reveals neither the engine nor the side, and `report` never reads it
back.

The renderer writes `clips/INDEX.md` (generated: conditions, durations, whether
a control side exists, and the instruction not to touch the volume between A
and B). For a real listening session, add an `INSTRUCTIONS.md` by hand next to
it. Both travel with the clips; the key stays at the root of the pack and must
not be distributed.

`report` reads `listening-key.json`, maps each A/B answer back to the tested or
the control side through `subject_side`, and writes `listening-report.md` and
`listening-report.json`. It **refuses to score** (exit code 2) a sheet that is
incomplete, outside the 1-5 scale or whose forced choice is not `A`/`B`, and
lists the faulty rows; `--allow-incomplete` scores the valid rows and declares
how many were discarded. A missing answer is never guessed.

Two safeguards added on 2026-07-30, both found while exercising the segmented
mode:

- **A pair without a control side is no longer scored.** It only has one file
  on disk, so a listener who scored the other side scored silence. Those
  judgements are excluded from every statistic and listed separately with the
  reason — it is the corpus that is missing, not the listener. Previously the
  report **crashed** on this case (`KeyError` on `reference`), which was at
  least visible; averaging it would have been less so.
- **The number of proxies is counted from the key.** The sentence "seven
  references out of ten are proxies" was hard-coded in the generator: it
  becomes false as soon as the corpus changes, and a stale warning is worse
  than none, because it is read as current.

The report gives, per question and per family: EngineLab's choice rate, a
**95 % Wilson confidence interval** (correct at small n, unlike the normal
approximation — exactly the regime of a five-listener pilot), an **exact
binomial sign test**, the mean realism and preference of both sides, and all
comments grouped by pair.

## What this test decides — and what it does not

Two comparisons that are easily confused must be told apart.

| | Against what | What it is for | Threshold |
|---|---|---|---|
| **A. Diagnostic** | EngineLab versus **real recording** | rank the families by deficit, to know where to put the effort | no pass threshold |
| **B. Non-regression** | candidate versus **EngineLab baseline** | accept or reject a timbre change | ≥ 65 % preference |

The current pack is case **A**. The 65 % threshold belongs to case **B** and
**does not apply to it**: seven references out of ten are proxies, and none has
a matched engine-speed trajectory or microphone position. Losing against a real
recording is expected, not a failure.

What can legitimately be drawn from it:

1. **The ranking by realism gap.** This is the main output. The family with the
   largest deficit is the next work target.
2. **The realism / preference gap.** A family judged unrealistic but well liked
   does not have the same problem as a family judged wrong *and* unpleasant.
3. **Comments grouped by layer.** "Too smooth" and "metallic" do not point at
   the same stage of the chain: the stems then let you check which one.
4. **A dated starting point** that a future version can be compared with.

What cannot be drawn from it:

- any claim of superiority over ES2D — no runnable comparison exists, its
  submodules are empty;
- any ranking between two families whose confidence intervals overlap: with
  five listeners, a 1/5 difference on a forced choice is not a result;
- any conclusion about a family whose reference is a weak proxy, if the comment
  does not say *why*.

With five listeners and ten pairs, a global effect shows if it is clear-cut; a
per-family difference stays indicative. It is a pilot — it serves to steer the
work and break in the protocol, not to publish a figure.

## One clip per engine biased the test — split on 2026-07-30

The first human pass reported an idle that was "too quiet". Measurement showed
that it was not the audio engine:

- the gap between idle and the limiter is **14 to 19 dB** depending on the
  engine;
- the BS.1770 integrated loudness of a clip holding both necessarily locks onto
  the loud part;
- the idle therefore lands around **−39 to −45 dBFS** in the delivered file;
- while the corpus references are often **idle-only** takes, and therefore
  normalised *on* the idle.

The comparison was biased by construction, and the bias grows with the idle
fix: a real idle at 950 rpm is quieter than an engine pulled against a brake at
1,892, so the gap **worsened from 4.4 to 5.3 dB** when the idle was made
correct.

**The fix is in the protocol, not in the acoustics.** Raising the idle level to
compensate must not be done — that would manufacture what the project refuses
elsewhere. `AbClipRenderer` therefore renders **a single trajectory** and
**splits** it into independently normalised segments (`listeningSegments`):

| Segment | Window in the trajectory | Duration | Normalised on |
|---|---|---:|---|
| `idle` | the **last 3.5** seconds of the idle hold | 3.50 s | its own content |
| `rev` | rev-up, limiter and lift-off, in one piece | 6.80 s | its own content |

The idle window is taken at the **end** of the hold because the post-start
flare occupies the beginning: a window that catches it measures a flare, not an
idle. The rev-up stays as **a single** clip because it is a single gesture and
its internal dynamics are precisely what a listener judges — the split
therefore raises nothing *within* a clip.

The bias, measured (2026-07-30, `--engines "Yamaha CP2"`):

| | own loudness | presented before | presented after |
|---|---:|---:|---:|
| whole trajectory | −21.55 LUFS | −20.00 | — |
| `idle` segment | −44.89 LUFS | ≈ **−43.3** | −20.00 |
| `rev` segment | −20.30 LUFS | ≈ −18.8 | −20.00 |

That is **23.3 dB** of presentation error removed from the CP2 idle, and
18.2 dB from the Full System one. The key publishes this figure per pair
(`level_error_removed_db`) along with the loudness of the whole trajectory
(`whole_trajectory_lufs`), so that the correction stays auditable.

**The render itself did not change**: the whole trajectory measured −21.5486
LUFS before the split and −21.55 after, on the same engine. It is a measurement
correction, not a voicing change.

Side benefit: the listener's two complaints become separable. "Too bright" is
judged on the `rev` clip, "the idle sounds like nothing" on the `idle` clip, and
a comment no longer has to cover two engine speeds at once.

### A reference must be matched in condition, or it is not matched

A real recording carries its idle and its rev-up at **different offsets** — the
five takes of the local corpus are 8.5 to 47 s long — so a single
`clip_start_seconds` cannot match both conditions. Manifest schema **3** adds:

```json
"segment_windows": {
  "idle": { "clip_start_seconds": 2.0 },
  "rev":  { "clip_start_seconds": 20.0 }
}
```

The rules are deliberately strict:

- a segment **missing** from `segment_windows` gets **no** reference, and the
  reason is published in the key (`control_error`). Setting a simulated idle
  against a take under load would produce a confident verdict on nothing —
  exactly the confounder that made the first pass unreadable;
- a take too short to carry two conditions declares only one. The LS3 idle
  (8.5 s file) therefore has no reference, and this **shows** instead of being
  substituted;
- schema 2 is still decoded and checked by `--validate-manifest`, but no longer
  matches anything: it carries no per-condition window.
  `--allow-unmatched-reference-window` forces the match for a rough check,
  marks the pair `[WINDOW NOT CONDITION-MATCHED]` in the output and
  `window_condition_matched: false` in the key, and the report then refuses to
  draw a publishable verdict from it;
- `--require-references` fails **before** any render if a window is missing.

## A/B between two catalogue engines (`--compare`)

Case **B** of the table above — judging a variant — had no tool: two engines
could only be compared by ear in the application, without loudness matching or
blinding. `--compare <baseline>,<candidate>` renders both engines on the
**same** trajectory, cuts the **same** windows, matches each side to the same
loudness and draws the A/B assignment.

```powershell
EngineLabAbClipRenderer --compare "Yamaha CP2,Full System" `
    --output out/validation/listening-cp2-variant
```

- The key carries `mode: "variant"`, `baseline_engine` and `candidate_engine`,
  and `control.kind` is `enginelab-baseline`. `scripts/listening.py` changes its
  labels accordingly: it cannot announce "EngineLab beat a real recording" on a
  test where no recording takes part.
- The reference options are **rejected** in this mode instead of being ignored:
  a run that silently dropped `--reference-manifest` would look like a
  comparison against reality in the shell history.
- An ambiguous filter is **rejected** here (`Twin` matches three catalogue
  engines), and only flagged in reference mode. It is the same substring trap as
  the one documented in `engines/15_cp2_full_system_like.engine.yaml`.
- The **65 %** threshold applies to this mode, not to the diagnostic mode.

## Remaining limitations

- The gestures of the real recordings do not follow the EngineLab trajectory
  exactly. The engine character is judged, not a cycle-by-cycle alignment.
- Seven references out of ten are explicitly labelled proxies. Even the three
  exact platforms have neither a matched RPM trajectory nor a matched
  microphone position. Instrumented takes will be needed for a model-by-model
  validation.
- The HQ previews are OGG or MP3 transcodes of the cited takes. For a final
  study, replace the paths with the original downloaded WAV/AIFF files, fill in
  the take parameters and keep their new SHA-256.
- No human preference is recorded in the repository yet. The corpus finally
  makes the test possible; it does not replace the listeners.
- `es2d` stays unusable locally because its submodules are empty; no runnable
  comparison against it is claimed.
