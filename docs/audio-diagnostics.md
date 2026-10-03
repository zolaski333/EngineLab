# Audio diagnostic exports

The Windows package also ships `tools/EngineLabOfflineAudioExporter.exe`. From
the extracted root, a directly usable export is obtained with:

```powershell
.\tools\EngineLabOfflineAudioExporter.exe --catalog-root . --engine K20 --output .\exports\k20 --format float32 --stems
```

When the stems option is enabled, the offline export writes:

- `master.wav`;
- six source buses (`stem_combustion`, `stem_exhaust_dry`,
  `stem_exhaust_ir`, `stem_intake`, `stem_forced_induction`,
  `stem_mechanical`);
- `premaster.wav`, the sum of the six buses in the order above;
- `master_processing_delta.wav`, the difference between the shipped master and
  the premaster;
- `engine-order-map.csv`.

Two extra diagnostic sub-stems break down the dry bus without being added a
second time to the premaster:

- `stem_exhaust_pressure_wave.wav` holds the wave radiated by the network
  (blowdown, reflections and any afterfire);
- `stem_exhaust_jet.wav` holds only the outlet turbulence, after the authored
  jet gain.

Their sum rebuilds `stem_exhaust_dry.wav`. The manifest publishes the float
error in `exhaust_dry_decomposition.sum_to_exhaust_dry_max_abs_error`. This
split lets you decide by ear whether excess treble comes from the pressure
source or the jet noise, without changing the master.

The auditable relation is:

`master = premaster + master_processing_delta`

The schema 4 manifest records the maximum absolute error of both
reconstructions in the float domain, before WAV encoding. In PCM24, each file is
quantised independently: a reconstruction read back from the WAVs may therefore
differ by a few LSB, which is a representation limit and not a missing audio
source. Use `float32` for numerical analysis without that ambiguity.

The `path_diagnostics` block also separates the four stages that can shape the
output: voicing saturation, slow AGC, 2x soft limiter and last-resort clamp.
`maximum_post_limiter_sample_magnitude` is exactly the sample peak at the host
rate after the soft limiter and just before the final clamp. It is deliberately
not called "true peak": no standardised inter-sample meter runs in the
real-time callback.

The order map uses 1/12 s windows with 50 % overlap, a Hann window and a
Goertzel evaluation at orders 0.5 to 24 in steps of 0.5. Each row holds the
centre time, the measured mean engine speed, the order, its frequency and its
level in dBFS. The engine speed comes from the simulation during the render;
it is not estimated from the sound.

The manifest's `audio_physics` block distinguishes configuration from measured
activity: authored COV/correlation, afterfire enabled, wet rev limiter, minimum
and maximum cycle multipliers, number of variation samples, maximum afterfire
heat, fuel mass actually burned and number of porous mufflers. This avoids
concluding that a ticked box produced an effect when the line was cold or no
unburned fuel was available.
