# Declarative audio voicing

Voicing is EngineLab's non-physical monitoring layer. It changes neither
combustion, cylinder pressures, gas dynamics nor torque. Its purpose is to
allow fast listening work without recompiling and without hiding a physical
correction inside an arbitrary gain.

## Layered resolution

The catalogue merges, in this order:

1. `voicing/default.yaml`;
2. `voicing/families/<normalised_family>.yaml`, if present;
3. `voicing/engines/<normalised_engine_key>.yaml`, if present.

A layer only replaces the fields it declares. The family comes from the
`family` field of the engine file. The engine key comes from the file name
without its last extension, normalised to lower case with `_` (for example
`11_yamaha_cp2_mt07_like.engine.yaml` becomes
`11_yamaha_cp2_mt07_like_engine.yaml`).

The schema is strict. An unknown key, a non-finite or out-of-range value, or an
unknown saturation placement rejects the whole new snapshot. In the
application, the last valid voicing then stays active and the error is shown.
Files are polled once per second on the UI thread; the audio callback does no
I/O and only reads lock-free atomics.

## Schema 1 fields

```yaml
schema_version: 1
voicing:
  volume: 1.0                  # 0..2
  convolution: 0.45           # 0..1
  high_frequency_gain: 1.0    # 0.2..2.5
  low_frequency_gain: 1.0     # 0.2..2.5
  low_frequency_noise: 0.35   # 0..1.5
  high_frequency_noise: 0.35  # 0..1.5
  combustion_gain: 1.0        # 0..2
  exhaust_gain: 1.0           # 0..2
  intake_gain: 0.85           # 0..2
  mechanical_gain: 0.70       # 0..2
  stereo_width: 1.0           # 0 mono, 1 neutral, 2 wide
  outlet_jet_gain: 1.0        # 0..2, physical jet noise only
  saturation_drive: 0.0       # 0 exact bypass, then 0..4
  saturation_placement: post_shelf # pre_shelf or post_shelf
```

Saturation has unity small-signal gain and its `0.0` bypass is exact. Stereo
width uses mid/side processing only when it differs from `1.0`. The jet gain
acts on the separate jet pressure component published by the exhaust acoustic
network.

## Catalogue profiles and instant A/B

All sixteen shipped engines now have an override under `voicing/engines/`.
These profiles are `estimatedFamily` presentations: they bring out the
character produced by pressure, topology and geometry, but are not presented as
measured microphone equalisations. The CC0 takes whose engine speed, load or
microphone geometry are unknown only serve to frame the expected character.

In **AUDIO HQ**, **VOICING CATALOGUE** recalls the engine's full override and
**NEUTRAL** restores every schema value, not just the nine visible faders. The
change is instant and does not restart the engine. Moving a fader now keeps
`low_frequency_gain`, `stereo_width`, `outlet_jet_gain`, `saturation_drive` and
`saturation_placement`; previously they were silently reset to their defaults
by the partial rebuild of the mix.

Changing engine while the workshop is open also reloads its catalogue profile
and its physical availabilities. The A/B is therefore no longer exposed to a
mix belonging to the previous engine.

## Default non-regression

The shipped file reproduces exactly the old compiled constants. On the K20A
`showcase` scenario, 48 kHz float32, master only, the render before and after
the introduction of the catalogue gives the same SHA-256:

`851B708DC02385D7A141746484002EED63B140425324FF778E10FF93648AE221`

This bit-exact equality is the baseline condition: any future audible voicing
must be an explicit override and go through a blind A/B listening test.
