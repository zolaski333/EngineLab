# Audio workshop and truthful controls

## What it does

The **AUDIO HQ** button opens a separate workshop without stopping the engine.
This window brings together:

- the real-time mix faders;
- the physical afterfire switch, applied immediately (see
  [physical-afterfire.md](physical-afterfire.md));
- `MUTE` and `SOLO` for the four source families;
- a choice of 48/96/192 kHz and 24-bit PCM / 32-bit float;
- master only, or master plus six stems;
- the built-in `showcase` profile or a JSON scenario chosen by the user;
- an offline render on a separate thread;
- progress, cancellation, opening the output folder and a final report;
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
