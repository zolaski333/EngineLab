# Real-time audio

The mix delivered by `EngineRuntime` assembles renderers in SI units for the
exhaust, the intake and the modal radiation of the block/heads. Forced
induction is driven by the solver, but its acoustic efficiency is still
semi-empirical. The complete physical description and its limits are in
[thermoacoustic-architecture.md](thermoacoustic-architecture.md).

## Simulation → callback flow

Three SPSC queues separate the responsibilities:

- `CylinderPressureSample` carries, at every sub-step, the chamber pressure and
  the exhaust boundary (`p`, `ṁ`, `ρ`, `c`, `CdA`, path, validity);
- `ExhaustAcousticSample` carries, each time the finite-volume network
  advances, what only changes then: the ducts' density and speed of sound,
  the outlets' flow and the exhaust's reaction heat, kept out of the per-step
  sample so that kilobytes are not copied on every sub-step;
- `FiringEvent` carries the combustion phasing, misfires, knock and the sources
  not tied to the physical exhaust.

Flows are signed, in kg/s. A negative value represents a real reversion from
the network back into the cylinder. Each physics frame maps its simulation
times onto a wall-clock window that starts when the frame starts, windows never
overlap, and the renderer reads them at least 20 ms behind the producer; a slow
software PLL compensates for the drift between the producer clock and the
sound card clock without abruptly moving events. A frame published more than
10 ms after its start (preempted while computing) starts its window 10 ms
before publication instead, so its samples still arrive ahead of the renderer:
otherwise they were all skipped at once and the exhaust boundary's jump was
heard as a click.

When a valid thermoacoustic graph is compiled, it owns the output from the
first sample and propagates silence up to the first boundary. If the telemetry
later becomes invalid, the physical guides drain naturally; the historical
synthesiser appears neither at start-up nor after a producer loss.

## Physical source

The renderer interpolates the adaptive frames monotonically. A slow average
separates the operating state from the acoustic perturbations, then:

```text
Zc = ρc/A
U′ = ṁ′/ρ
p+ = (p′ + ZcU′)/2
p− = (p′ - ZcU′)/2
```

The valve's differential resistance is derived from the orifice law around the
current flow; it sets the port reflection coefficient. The characteristics are
then propagated through forward/backward lines whose delays come from the
physical lengths and the local speed of sound. Collector junctions use the
admittances `A/(ρc)` and keep the cross-talk between cylinders of the same
path.

The historical parameters `exhaustPreset`, low/high-frequency noise, DAG audio
transmission, FDN, jitter, collector saturation and blowdown voices take no
part in this computation. Regression tests check that invariance.

## Outlet, radiation and level

Each outlet ends in a passive causal load of a free or flanged circular
opening. The reflected wave returns to the collector; the volume-velocity
acceleration gives a reference monopole pressure. `FreeFieldObserver` then
propagates separately to the left and right microphones with exact distance,
`1/r` decay, `r/c` delay and `ka`-dependent directivity.

### Camera microphone

*Microphone on the camera* (view settings, on by default) moves the pair to
the 3-D view's camera. `RealtimeEngineAudio::setMicrophones` publishes the
positions lock-free (a sequence lock); the audio thread takes them at the next
block and every spatialised layer moves: each exhaust outlet and intake inlet
from its own position (`FreeFieldObserver::moveMicrophones`), the turbo and
the structure by the pair's mean distance. Combustion and the legacy layers
do not move.

A moved microphone glides, with a 0.2 s time constant, its delay changing by
at most 0.02 sample per sample: a pitch shift of 2 % at most, never a step.
The glide ends in a short linear tail that lands on the target exactly; a
plain exponential step in float stalls once it falls below half an ulp (a
560-sample delay stopped 0.3 sample short, leaving the sound 35 dB off the
default one after switching off).
It stays 0.25 to 40 m from each source (closer, a 1/r point source is no
model of a pipe mouth). Switched off, the pair glides back to the authored
positions and the sound becomes the default one again. An unmoved
microphone runs the exact arithmetic it always has.

The view maps its frame (millimetres, +Y up, +Z along the crankshaft) to the
sound's with `render::ListenerFrame`: up stays up, distances from the origin
are kept, and the frame is turned about the vertical so that the drawn
tailpipe points along the first outlet's acoustic axis.

### Trackside microphone

*Trackside microphone* (view settings, *Sound*; off at every start, not
saved) hears the car from beside a lane as it drives past, ISO 362's
microphone: 7.5 m from the lane's centre line, 1.2 m up by default
(`PassByMicrophone`, `RealtimeEngineAudio::setPassByEnabled`; the place, the
ground and the level from `setPassByGeometry`). The master as
the engine's own observer hears it, after the voicing and before the master
gain and limiter, is summed to two points 0.5 m up that move along the lane
at the vehicle's speed: the exhaust's share (the dry tailpipe and its room
response) at the tailpipe, the wheelbase behind the engine (an estimate for a
front-engined car; a mid-engined car or a motorcycle has it shorter), and the
rest (intake, combustion, structure, turbo) at the engine. So the intake
leads and the exhaust follows, 0.1 s later at 108 km/h on a 3 m car. The car
moves (`RealtimeAudioState::vehicleSpeedMps`, smoothed over
20 ms). Alone it starts in front of the microphone, drives to 150 m past it,
and comes back from 150 m before it, the last 30 m each way faded out; set
the cruise control and it passes again and again. On the dyno or standing
still it stays where it is. During a pass-by run (`RealtimeAudioState::
passByPhase` not 0) the microphone is on whatever the menu says and the car
is the run's (`passByPositionM`, followed between frames at the car's speed
and pulled toward each new figure over 0.5 s; a jump over 50 m, a restart,
fades out, moves and fades in), scaled by the run's fade (`passBySoundGain`).
The delay line holds a car 2.1 km away (`maximumRangeM`).

Each of the two microphones (0.18 m apart along the lane, the left one
upstream) gets two paths, the direct one and the ground's image. Each path
reads a delay line at its
retarded time: the sound heard now left the car tau ago from x - v tau, so
(c^2 - v^2) tau^2 + 2 x v tau - (x^2 + D^2) = 0, which gives the moving
source's own Doppler shift exactly for a speed constant over tau (to 80 % of
the speed of sound). Its level is the observer's distance over the emission
distance (6 dB per doubling); *Normalised* holds it at the closest
approach's, so a far car keeps its timbre (delay, air, ground) at a level
one can hear. The image path is a real reflection: the ground's spherical-
wave coefficient's magnitude at the comb's first notch, so its notches stay
where a hard ground puts them.

What a delay and a real gain cannot carry goes on each microphone after the
sum, redesigned every 256 samples from where the car is: ISO 9613-1's air
absorption (20 C, 70 %, 101.325 kPa; the exact formula) as four one-pole
low-passes set where the air takes 20 dB, and the porous ground's
reflection: Delany-Bazley's impedance (flow resistivity: asphalt 30,000,
gravel 3,000, grass 200, snow 20 kPa s/m2, Embleton, Piercy & Daigle 1983)
in the spherical-wave coefficient Q = Rp + (1 - Rp) F(w) (the Faddeeva
function by Weideman's 32 terms). Both become the levels of a 29-band
third-octave graphic equaliser (25 Hz-16 kHz, peaking filters at Q 4.3, the
bands' interaction inverted, each held to -36..+12 dB) that takes what the
low-passes and the real reflection left: within 0.3 dB of ISO from 10 m to
1 km and 0.1 dB of the exact ground at band centres (`EngineLab.
PassByMicrophone`). Not modelled: the notches' shift on a soft ground
between band centres, turbulence, refraction. Switched on, it waits until
its delay line holds the car's sound,
then cross-fades in over 50 ms; switched off it cross-fades out and lets go.
Off, the master is untouched (`EngineLab.PassByMicrophone`). While it is on,
the camera's microphones go back to their authored place. Not modelled: wind,
a temperature gradient, the car's own directivity, the tyres and the wind
noise of the car.

The authored sources are schematic (outlets and intake mouths near the
origin). While the microphones are on the camera, every opening radiates from
where the 3-D view draws it instead: `render::ListenerFrame::sources` gives
each exhaust outlet's tip and each intake path's mouth (bellmouth, airbox
inlet, throttle mouths or supercharger eye), with the direction out of it, and
the turbo's compressor or the supercharger. `RealtimeEngineAudio::
setSoundSources` hands them to the audio thread (a pointer swapped at the
next block, the old one freed by `collectRetiredExhaustNetworks`), and
`AcousticExhaustNetwork::placeOutlets` / `AcousticIntakeNetwork::placeMouths`
move each `FreeFieldObserver`'s source (`moveSource`), matched by path index
and, for an exhaust outlet, its component id. The source glides like a
microphone. The turbo is heard at the pair's mean distance from its drawn
place, the structure from the origin. Switched off, every source goes back to
its authored place, and the sound is the default one again bit for bit.

The camera is usually within a metre or two of the engine, against 4 m for
the authored listener, so the level and the balance between layers change;
the master's safety gain and limiter catch what is too loud.

The signal stays in pascals until the monitoring conversion. Since dBFS
describes an electrical/digital chain and not a universal pressure, the full
scale SPL of the microphone/preamp is published explicitly in
`RealtimeAudioState::acousticFullScaleSplDb`. By default:

```text
282.843 Pa peak = 200 Pa RMS = 140 dB SPL = 0 dBFS
```

The conversion always uses the 20 µPa acoustic reference. Changing this
calibration simulates the gain/headroom of the capture chain; it changes
neither the computed physical pressure, nor propagation, nor listening volume.
The observed SI pressure peak and the leveler activity are exposed separately
to the validation harnesses.

The 3D positions/axes of the outlets and the microphone pair are published by
engine schema 4. Stereo therefore comes from physical path differences, not
from panning. A measured stereo IR can add the room or cabin downstream.

## Impulse responses

`RealtimeConvolutionBank` holds up to eight partitioned convolutions,
preallocated outside the callback. The application loads an IR only if the path
explicitly declares `impulse_response` in JSON/YAML. Without a declared file,
the result is the computed free field.

There is no longer automatic selection by preset, a generic default IR or an IR
synthesised from lengths and restrictions. An explicit IR is treated as a
measurement of downstream propagation; it must not double the tube response
that is already simulated.

The application now makes this contract visible:

- when a physical exhaust DAG is compiled, the historical
  `Street / Open / Turbo / Long tube / Moto` selector is replaced by
  **PHYSICAL GRAPH** and disabled; changing the sound goes through the exhaust
  designer's geometry, not through a procedural preset with no effect;
- the inherited high-frequency noise control is flagged as not applicable and
  no longer moves in physical mode;
- an IR that is explicitly declared but missing, empty, corrupt or unreadable
  produces an error naming the path. The mixer shows the number of IRs actually
  loaded. The free field stays usable, but it is no longer a silent fallback.

Decoding outside the callback is centralised in `ImpulseResponseLoader`. The
tests open a shipped IR and explicitly reject the missing-file and corrupt-WAV
cases.

## Other layers

`AcousticIntakeNetwork` propagates the valve flows through the runners, the
plenum, the throttle and the air inlet. `StructuralModalRadiator` receives gas
forces, inertias and bearing reactions in SI. Forced induction uses shaft
power, blade/lobe orders and wastegate/dump-valve flows; its absolute levels
stay semi-empirical until a component measurement replaces them. Its noise
sources are independent, its telemetry steps are reconstructed at audio rate
and the total flow is conserved when it splits between turbine and wastegate.
The broadband filter now keeps the RMS pressure derived from its configured
power. See also §21–24 of the architecture document.

For the estimated structural modes, the longitudinal participation follows the
explicit cylinder order within each bank. The previous modulo on the global
index distorted V and flat engines; the reference I4 stays bit-identical.

## Diagnostic stems

`RealtimeEngineAudio::renderWithStems` can observe six pre-master stereo buses
without feeding them back into the output:

- combustion;
- dry exhaust;
- exhaust IR return;
- intake;
- forced induction;
- mechanical/structure.

The taps sit before the common shelf, the DC blocker, the reconstruction
filter, the volume, the leveler and the limiter. Dry exhaust and IR stay
separate so as not to confuse the engine source with the room or cabin. The
buffers are supplied by the caller and filled without allocation.

The high-quality user render reuses this path in `OfflineAudioExporter`:
48/96/192 kHz, 24-bit PCM or 32-bit float, JSON scenario, master and stems,
manifest and cancellation. See [offline-hq-rendering.md](offline-hq-rendering.md).
The **Audio workshop**, its mute/solo controls and the explicit neutralisation
of legacy settings with no effect are documented in
[audio-workshop.md](audio-workshop.md).

Targeted export:

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAudioRenderHarness.exe `
  --stems K20 `
  --output out/validation/audio-stems-k20
```

Release proof of 29 July 2026:

| K20 stem | RMS | Peak |
|---|---:|---:|
| direct combustion | 0.000000 | 0.000000 |
| dry exhaust | 0.030013 | 0.316187 |
| exhaust IR | 0.003014 | 0.015993 |
| intake | 0.030953 | 0.522270 |
| forced induction | 0.000000 | 0.000000 |
| mechanical/structure | 0.027226 | 0.114039 |

Both zeros are expected: the K20 is naturally aspirated and, as soon as the
physical boundary takes over, cylinder pressure drives the exhaust and the
structure instead of being doubled by a direct procedural voice. The
`EngineLab.RealtimeRegression` test renders two deterministic instances, one
with stems and one without, then requires **bit-exact equality of every master
sample**. It also checks the buffer bounds and the silence of inactive buses.

## Turbulence at the exhaust outlet

`ExhaustJetNoise` adds a causal broadband source at the outlet, derived from
the flow, density, speed of sound and diameter already solved. Its power
follows the subsonic `U^8` law in the ambient air (`rho0`, `c0`) and its
spectral centre `St = 0.2`. A silencer's flow noise (`MufflerFlowNoise`,
Green and Smith's `u^4.5` law, pink) can leave by the same outlet and observer,
but is off by default since 2026-11-19: on silenced engines it buried the note
under 10-35 dB of hiss at 4-8 kHz (`setMufflerFlowNoiseEnabled`, harness
`--muffler-flow-noise`). The audio
modulation comes from the volume flow of the radiation load; it is never fed
back into the gas network.

The source goes through the same stereo observer and the same IR as the outlet.
It is deterministic and does no allocation in the callback. Both laws are
sourced (`docs/thermoacoustic-architecture.md` section 26); neither has been
judged against a recording.

A/B sound check:

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAudioRenderHarness.exe `
  --exhaust-jet-comparison "LS3" --output out/validation/jet-ls3
```

A/B CPU check, same binary:

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --filter Merlin --warmup 3 --seconds 10 --free-run --with-audio `
  --disable-exhaust-jet-noise
```

## Callback contract

`RealtimeEngineAudio::render`:

- does no allocation, I/O, logging or locking;
- honours `startSample` and `sampleCount` exactly;
- allocates guides, observer buffers and convolutions in `prepare()`;
- sizes the lines from the published geometry and counts any truncation;
- neutralises denormals;
- stays invariant at 48, 96 and 192 kHz for the tested physical constants.

After the mix, a user shelf, a DC blocker, a reconstruction filter, a slow
safety leveler and an oversampled soft limiter protect the device. These stages
do not serve to create the exhaust signature. At normal voicing, the leveler
must stay at unity gain; its counters are observable by the harnesses.

## Tests

`EngineLab.RealtimeRegression` covers in particular:

- passivity and causality of radiation;
- determinism of the SI render;
- the `U^8` law, Strouhal, RMS level and silence at zero flow of the outlet
  jet;
- the influence of the flow sign;
- silence of a stationary SI boundary;
- invariance to the inherited presets, events and noises;
- dependence on tailpipe lengths;
- delay consistency at 48 and 96 kHz;
- exactness of the SPL ↔ pressure ↔ dBFS conversion;
- no re-activation of the procedural path after the physical lock.

The capacity bench can also run the renderer concurrently with the real runtime
thread:

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --catalog-root . --free-run --rpm 7000 --warmup 3 --seconds 6 `
  --with-audio --audio-rate 48000 --audio-block 256
```

In `--free-run`, the consumer follows the accelerated simulated time and
compares each render duration with the block's real deadline. It rejects queue
losses, invalid boundaries, the historical fallback, non-finite output,
over-budget callbacks and leveler intervention.

`EngineLab.Core` additionally checks that a finite SI boundary is published at
every mechanical sub-step despite the multirate coupling of the non-linear
network. It also rejects compressor/turbine cancellation, a power discontinuity
at the telemetry boundary and any turbine/wastegate flow duplication.
`EngineLab.RealtimeRegression` keeps the tests for blade order, power root,
silence without flow and radiation of the valves that are actually open.

The Release transient suite is separate from the stationary render:

- `EngineLab.AudioTransients` renders the production chain during a start, a
  settled idle, a blip, the return to idle, a throttle lift/reapply under boost
  with a dump valve, then an entry into the rev limiter's cut zone;
- `EngineLab.AudioShiftTransientNA` and
  `EngineLab.AudioShiftTransientBoosted` render a complete clutchless WOT gear
  change, check the clutch lock-up and write the proof WAVs;
- each case rejects non-finite values, event or pressure losses, voice
  stealing, the physical fallback, the leveler or limiter used as a band-aid,
  and an isolated discontinuity.

The click detector compares a step with the local distribution of steps during
the same event. Comparing an active dump valve with the pre-transient WOT
plateau alone is invalid: the expected broadband jet noise rises precisely
during the shift and would fail a continuous signal.

## Known limitations

- eight paths and 32 cylinders maximum;
- plane, linear acoustics for the audio band;
- no mean-flow correction of radiation;
- estimated structural modes when no sourced NVH section is provided;
- no real NVH measurement shipped in the catalogue to date, although the
  `measured` path is now configurable;
- forced-induction acoustic efficiency still semi-empirical;
- real multi-microphone correlation still to be done engine by engine.
