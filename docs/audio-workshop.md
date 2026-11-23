# Audio workshop and truthful controls

## What it does

**Listen > Audio workshop…** in the top bar opens a separate workshop without stopping the engine.
This window brings together:

- the real-time mix faders;
- the physical afterfire switch, applied immediately and the only way to turn
  backfires on; every engine loads with them off (see
  [physical-afterfire.md](physical-afterfire.md));
- `MUTE` and `SOLO` for the four source families;
- a choice of 48/96/192 kHz and 24-bit PCM / 32-bit float;
- master only, or master plus six stems;
- the built-in `showcase` profile or a JSON scenario chosen by the user;
- an offline render on a separate thread;
- progress, cancellation, opening the output folder and a final report;
- a **loop pack for games** (below);
- the engine identity and the state of the physical graph.

Rendering stays available during a simulation or a dyno run: it works on a
consistent copy of the configuration and does not borrow the audio thread.
Closing the window during a render does not destroy the job. Quitting the
application requests cancellation, then waits for the thread cleanly.

Every run creates a time-stamped sub-folder. The application therefore never
silently reuses an old `master.wav`.

## Truthful controls

The old `MIXER / AUDIO` screen showed several parameters that could no longer
act once the physical graphs were compiled. That was not a DSP defect: the
corresponding procedural voices are deliberately removed. The defect was the
interface, which kept presenting those parameters as active.

The workshop and the shortcuts now apply the following table:

| Control | Physical graph | Behaviour |
|---|---:|---|
| Master | all | active |
| Measured IR return | only if an IR was loaded | disabled and forced to zero without an IR |
| Master treble | all | active |
| Legacy intake noise | only without an intake graph | disabled with the quasi-1D graph |
| Legacy exhaust noise | only without an exhaust graph | disabled with the thermoacoustic graph |
| Direct combustion | only without an exhaust graph | disabled: cylinder pressure already drives the exhaust |
| Exhaust | all | active, dry and IR return |
| Intake + forced induction | all | active |
| Structure / mechanical | all | active |

The legacy keys `X`, `V`, `B` and `J` no longer change an inaudible value in the
`N/A` cases. The main screen says so as well, instead of showing a misleading
gauge.

The visible faders form the **base mix**. Mute/solo produces a separate
**effective mix** sent to the audio thread and to the HQ render. A solo thus
never destroys another fader's value; turning the solo off restores the
previous mix.

## Dry/IR semantics

The IR control is called **measured IR return**, not "convolution":

- the dry signal is not attenuated;
- the 0..1 value adds up to 50 % of measured return, following the existing
  routing of `RealtimeEngineAudio`;
- without a valid IR file, the control is disabled and the effective mix is
  zero;
- the HQ export lets you inspect `stem_exhaust_dry.wav` and
  `stem_exhaust_ir.wav` separately.

There is therefore no hidden room preset and no fake 100 % wet.

## Loop pack for games

`Loop pack for games...` renders seamless loops for a game's sample-based
engine sound, BeamNG's `soundConfig` and `soundConfigExhaust` for instance. It
is off until clicked and changes nothing else. The step selector (250, 500 or
1,000 rpm) and `+ OGG` sit beside it; the command line gives the same pack
with `EngineLabOfflineAudioExporter --loop-pack [--loop-step <rpm>]
[--loop-seconds <s>] [--ogg]`.

- **Points**: the idle, then every step from a step above it, up to 0.95 x
  the lower of the redline and the rev limit (the dyno's ceiling).
- **On load**: full throttle, held at the point by the dyno's absorber
  (`DynoAbsorberController`). **Off load**: throttle shut, the shaft driven at
  the point by a motor, as the wheels drive an engine on the overrun, so the
  ECU's fuel cut and the overrun sound are the engine's own. The motor only
  drives, never brakes: where an engine runs above the point on its own, the
  loop is at its own speed (the file says which). At the idle the engine idles
  on its own, the motor a floor at 0.9 x idle that keeps a wandering
  two-stroke idle from stalling.
- The engine is started as the audio harness starts every catalogue engine:
  1.1 s of starter at a fifth of throttle.
- A point is recorded once settled: its speed within 1.5 % of the target and
  steady to 0.4 %, its manifold pressure steady to 1 kPa, over two half
  seconds. After 12 s it is recorded anyway and the report says so.
- **Loops**: a whole number of the engine's cycles (720 crank degrees, 360 on
  a two-stroke, 1,080 on a rotary), at least four, about 2 s. The end is
  cross-faded into the start over whole cycles (50 ms at least), so the last
  sample runs into the first as any two consecutive samples do. The loop's
  mean is removed: over whole cycles it is its DC.
- **Layers**: `engine` (combustion + intake + forced induction + mechanical,
  what radiates from the engine) and `exhaust` (dry exhaust + IR return, what
  leaves the tailpipe). Together they are the premaster of the HQ export.
- **Gain**: one for the whole pack, so the loudest file peaks at -1 dBFS and
  every other keeps its level relative to it. The loops are the sources
  before the master tone shelf, saturation, AGC and limiter: the game applies
  its own.
- **Files**: `onload_03000rpm_engine.wav`, `offload_03000rpm_exhaust.wav`...,
  48 kHz mono 24-bit WAV with a `smpl` chunk marking the whole file as one
  loop, an optional Ogg Vorbis copy (lossy: the WAV is the exact loop), and
  `loop-pack.json`: each loop's measured mean speed (the speed to declare for
  it, not the requested one), cycles, manifold pressure, levels, and the
  **cylinders to declare**, the firings per 720 degrees: the real count on a
  four-stroke, twice it on a two-stroke, two per rotor on a rotary.

Not written: BeamNG's `sfxBlend2D` blend file. Its format is not documented,
and a guessed one would be worse than none.

## Automated checks

The `EngineLab.AudioWorkshop` test builds the real JUCE window and checks:

- three sizes, from the 980×620 minimum to 1600×900;
- every visible component has non-zero dimensions;
- no component extends outside the window;
- nine faders are present;
- the four faders with no effect in the physical case without an IR are
  disabled;
- four MUTE/SOLO pairs exist and direct combustion is unavailable;
- the HQ export action is visible;
- the loop pack action is enabled and inside the window;
- solo routing keeps the exhaust and its IR return, then cuts intake and
  structure;
- `MUTE` takes priority over `SOLO`;
- returning to compatibility mode re-enables the controls that are really
  available.

Command:

```powershell
cmake --build out\build\windows-vs2022 --config Release `
  --target EngineLabApp EngineLabAudioWorkshopTests `
  --parallel 1 -- /nr:false /m:1 /v:minimal

ctest --test-dir out\build\windows-vs2022 -C Release `
  -R "^EngineLab\.AudioWorkshop$" --output-on-failure
```

Result:

```text
Audio workshop: layout, truthful availability, mute/solo routing
and HQ export controls PASS
```

## Deliberate limitation

The forced-induction bus has its own offline stem, but still shares the intake
real-time fader. Splitting it in the runtime would change the historical
contract of the harnesses that use "mute intake" to isolate the exhaust. That
split can be made in a dedicated change, with an A/B and an update of every
isolator.
