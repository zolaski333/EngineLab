# Measured or computed structural NVH modes

## Principle

Mechanical radiation is always driven by the SI forces published by the
solver: gas force on the piston, axial and lateral bearing reactions, and the
crankshaft reaction torque. An NVH configuration therefore does not replace the
engine with "musical" oscillators; it only replaces the reduced mode set that
turns those forces into surface velocity and then into radiated pressure.

Without a `structural_nvh` section, EngineLab keeps its shell/beam/plate family
model and publishes the `estimatedFamily` provenance. None of the shipped
engines currently claims a manufacturer NVH measurement. For this fallback,
each cylinder's longitudinal coordinate comes from its position in
`banks[].cylinderIds`, never from its global index. That distinction is
essential for V and flat configurations whose storage alternates banks.

## Schema 5

Example of the shape, deliberately illustrative and **not usable as the
calibration of a real engine**:

```yaml
structural_nvh:
  provenance: measured
  source: "modal report, identifiable mounting and revision"
  modes:
    - name: "block vertical bending 1"
      drive: bearing_axial
      frequency_hz: 1234.5
      damping_ratio: 0.025
      modal_mass_kg: 4.2
      radiating_area_m2: 0.18
      radiation_efficiency: 0.42
      surface_velocity_rms_scale: 0.50
      torque_radius_m: 0.06
      cylinder_participation: [1.0, -0.65, 0.65, -1.0]
```

Accepted values:

- `provenance`: `estimated_family`, `calculated_geometry` or `measured`;
- `drive`: `head_gas`, `bearing_axial`, `bearing_lateral` or `torsion`;
- a signed participation, normalised at the antinode, per cylinder and in the
  order of `engine.cylinders`;
- frequency, damping, modal mass, radiating area, radiation efficiency, RMS
  shape factor and equivalent torque radius in SI units.

`torque_radius_m` is only consumed by a `torsion` mode, but it is still
persisted so that every mode shares a fixed schema. There is no arbitrary gain
field.

## Conditions for declaring `measured`

A usable campaign must at least keep:

1. the mounting, the impact/excitation points and the accelerometers or the
   vibrometer field;
2. the FRFs that gave the frequency and damping;
3. the normalisation convention of the mode shape and a modal mass consistent
   with it;
4. the computation or measurement of the radiating area/efficiency;
5. the identifier of the report or dataset in `source`.

Validation rejects:

- a measured provenance without modes;
- a mode set without a source;
- more than 64 modes;
- a shape that does not have exactly one value per cylinder;
- a zero, non-finite or out-of-`[-1, 1]` shape;
- non-physical or out-of-band parameters.

This discipline prevents turning an estimate that pleases the ear into fake
manufacturer data.

## Automated check

`EngineLab.StructuralNvh` builds a temporary mini-catalogue holding a synthetic
bench mode at 1,234.5 Hz. The test proves:

- loading through the same YAML decoder as the production catalogue;
- provenance and source reachable at runtime;
- exact transmission of frequency, damping, mass, area and efficiency;
- JSON/YAML round-trips;
- a finite, non-zero response to the bearing force at resonance;
- rejection of a fake measured provenance and of an incomplete shape;
- the `estimatedFamily` fallback kept when the section is missing.

Command:

```powershell
ctest --test-dir out/build/windows-vs2022 -C Release `
  -R '^EngineLab\.StructuralNvh$' --output-on-failure -V
```
