# Designing a custom exhaust

Each `ExhaustPathConfig` can hold an optional `graph`. This graph describes a
directed acyclic network of components between the cylinders and one or more
outlets. It is serialised as JSON and YAML and compiled by `ExhaustGraph` into
routes used by the historical diagnostic tools. In production,
`ExhaustNetworkLayout` keeps every component in the quasi-1D gas solver and
`AcousticExhaustNetwork` compiles the same DAG into bidirectional guides for the
high band. The network is therefore never reduced globally to a single
restriction or a single acoustic tube per path.

The **Exhaust** button in the top bar opens a graphical designer for the most common
operations. JSON/YAML stays the persistence format and still allows the
structural changes the interface does not cover. A `.els` script can use this
file as its `base`, but the DSL does not create the graph nodes itself yet.

## Using the designer

The window loads a working copy: adding, removing or editing a component does
not touch the running engine until the copy is applied.

1. Pick a path in the list. **+ PATH** extracts the chosen cylinder with its
   graph branch; **- PATH** moves its cylinders to a destination before
   deletion. The **MOVE** selector reassigns a cylinder between two paths
   without creating a duplicate. The IR stays in the engine file.
2. If the path has no graph yet, select **GENERATE FROM LEGACY**. One primary
   per cylinder, a junction, a muffler and an outlet are created from
   `geometry`.
3. Add the components, then edit their type, ID, dimensions, restriction,
   resonance, gain and discharge coefficient. The canvas uses automatic layout,
   and clicking a node selects the matching component.
4. Create the directed connections in flow order and assign each cylinder to
   its first component. On an X-pipe, the designer assigns the first free port
   `0`, then `1`, and shows that port in the list. Deleting then recreating both
   links swaps the pairing.
5. Select **VALIDATE AND APPLY**. The complete validation described below runs
   before any change to the engine.
6. Use **EXPORT** in the main window to save the applied configuration as JSON
   or YAML.

An error keeps the copy for correction and leaves the current runtime
unchanged. A valid application is a structural change: it replaces the runtime,
resets engine speed and thermal states, and is not allowed during a dyno run.
It is therefore not the no-reset hot reload of the ECU tuner.

## YAML example

The following block replaces `engine.exhaust_paths` for an I4 whose cylinders
have the identifiers 1 to 4:

```yaml
exhaust_paths:
  - id: 1
    cylinder_ids: [1, 2, 3, 4]
    impulse_response: "assets/ir/exhaust_default.wav"
    audio_volume: 1.0
    geometry:
      primary_length_mm: 480
      primary_diameter_mm: 42
      collector_diameter_mm: 60
      muffler_restriction: 0.25
      outlet_diameter_mm: 70
      collector_volume_l: 2.5
      outlet_discharge_coefficient: 0.78
    graph:
      components:
        - { id: 101, type: pipe,     length_mm: 480, diameter_mm: 42, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 102, type: pipe,     length_mm: 480, diameter_mm: 42, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 103, type: pipe,     length_mm: 480, diameter_mm: 42, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 104, type: pipe,     length_mm: 480, diameter_mm: 42, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 201, type: merge,    length_mm: 0,   diameter_mm: 60, restriction: 0.03, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 301, type: catalyst, length_mm: 180, diameter_mm: 60, outlet_diameter_mm: 68, restriction: 0.18, catalyst_cell_density_cpsi: 400, catalyst_open_area_ratio: 0.80, catalyst_substrate_volumetric_heat_capacity_j_m3_k: 2000000, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 401, type: muffler,  length_mm: 520, diameter_mm: 65, volume_l: 8.0, restriction: 0.20, resonance_hz: 95, acoustic_gain: 0.82 }
        - { id: 501, type: outlet,   length_mm: 120, diameter_mm: 70, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00, discharge_coefficient: 0.78 }
      cylinder_connections:
        - { cylinder_id: 1, to_component_id: 101 }
        - { cylinder_id: 2, to_component_id: 102 }
        - { cylinder_id: 3, to_component_id: 103 }
        - { cylinder_id: 4, to_component_id: 104 }
      connections:
        - { from_component_id: 101, to_component_id: 201 }
        - { from_component_id: 102, to_component_id: 201 }
        - { from_component_id: 103, to_component_id: 201 }
        - { from_component_id: 104, to_component_id: 201 }
        - { from_component_id: 201, to_component_id: 301 }
        - { from_component_id: 301, to_component_id: 401 }
        - { from_component_id: 401, to_component_id: 501 }
```

The same keys exist in JSON under `engine.exhaust_paths[].graph`.

### Measured porous packing

The designer exposes the three fields below when a `muffler` component is
selected. **DEMO PACKING** fills in 24,000 Pa·s/m², 35 mm and 0.28;
**BYPASS PACKING** sets all three fields back to zero. An update is a
structural change and restarts the engine when the graph is applied.

A `muffler` component can describe its absorption with three measurements
independent of the pressure loss:

- `packing_flow_resistivity_pa_s_m2`, the flow resistivity of the porous
  material;
- `packing_thickness_mm`, the radial thickness of the packing;
- `perforated_open_area_ratio`, the open fraction of the perforated tube,
  between 0 and 1.

All three fields must be strictly positive together and are only valid on a
`muffler`. A packed muffler then explicitly distinguishes two geometries:

- `diameter_mm` and `outlet_diameter_mm` describe the perforated core that
  carries the mean flow and the propagation delay;
- `volume_l` describes the gross volume of the outer body. It must be strictly
  greater than the volume swept by the core. The difference is the closed
  annulus filled with material, never an extra flow area.

Without them, the filter compiles to an exact identity: EngineLab does not
invent absorption from `restriction`, `acoustic_gain` or the body volume. When
they are provided, the surface impedance follows Delany-Bazley and the
propagation loss stays passive. The open fraction also couples part of the
annular volume to both ends of the core as passive acoustic compliance.
Changing the body volume therefore changes the transfer without widening the
gas duct or adding a cell. These values must come from the material data sheet
or a measurement of the muffler.

This model is deliberately reduced-order: it represents the dominant
compression mode of the annular volume and the loss of the material. It does
not simulate the holes one by one, nor their inertance, because the graph does
not yet hold the perforation diameter or the sheet thickness. Adding those
effects without that data would be hidden sound tuning, not a physical
correction.

The same three fields exist on a scalar geometry without `graph`, under the
names `muffler_packing_flow_resistivity_pa_s_m2`,
`muffler_packing_thickness_mm` and `muffler_perforated_open_area_ratio`. They
are copied into the real muffler node during compilation or **GENERATE FROM
LEGACY**. `cp2_full_system` and `cp2_absorptive_lab` share the same geometry
and the same restriction, but different estimated material properties for a
controlled A/B. EngineLab never infers them from a muffler's name.

### Homogenised catalyst monolith

A `catalyst` component can describe its cellular substrate explicitly with
three fields:

- `catalyst_cell_density_cpsi`, cell density per square inch;
- `catalyst_open_area_ratio`, the fraction of frontal area that is really
  open;
- `catalyst_substrate_volumetric_heat_capacity_j_m3_k`, heat capacity of the
  solid per volume of occupied substrate.

All three values must be zero together or positive together. Zero keeps the
historical catalyst exactly: an ordinary duct carrying the length, the diameter
and the `restriction` coefficient. When the substrate is provided, validation
accepts 25 to 5,000 cpsi and an open area of 0.05 to 0.99, and requires at least
one cell pitch to fit in the housing diameter. The designer offers 400 cpsi, 0.80
and 2.0 MJ/m³/K as an editable starting point; these numbers are neither
inferred from the engine name nor applied to older catalogues.

The model assumes square channels. With `N` in cpsi and `phi` as the open
fraction:

```text
pitch      = 0.0254 / sqrt(N)
width      = pitch * sqrt(phi)
channel Dh = width
flow area  = housing area * phi
```

The complete bundle stays **a single quasi-1D duct**. The open area sets the
flow admittance, and the hydraulic diameter of one channel sets the distributed
friction and the thermo-viscous acoustic losses. The number of gas solver cells
and the number of audio lines therefore stay identical to the bypass, whatever
the cpsi value. Each axial cell also aggregates the wetted area of all
channels, the heat capacity of the solid and that of the metal shell into a
single thermal state. This model does not claim to compute the emission
chemistry, internal radial conduction or each real channel.

## Component types

| Type | Compiled role |
|---|---|
| `pipe` | length, diameter, geometric loss and quarter-wave resonance |
| `merge` | gathers at least two inlets into one outlet |
| `splitter` | splits one inlet into at least two branches |
| `resonator` | inline duct if it has an outlet; closed acoustic side branch if it has none |
| `muffler` | inline chamber/duct and, if provided, distributed porous packing |
| `catalyst` | physical duct and, if provided, a passive homogenised cellular substrate |
| `outlet` | ends a route and applies diameter/discharge coefficient |
| `crossover` | compact X-pipe with two paired inlets/two outlets and a passive acoustic matrix |

Each component has:

- `id`, unique within the path;
- `length_mm`, `diameter_mm`, and optionally `outlet_diameter_mm` and
  `volume_l`;
- `restriction`, an additional dimensionless loss coefficient;
- `resonance_hz`, the reference tuning of a terminal `resonator`; zero keeps its
  geometric length;
- `acoustic_gain`, kept for the historical fallback audio render, but never
  applied to the passive physical waveguide;
- `discharge_coefficient`, mainly used by the outlet;
- `crossover_coupling`, only on `crossover`, the cross-power amplitude `k`
  between 0 and 1.

The final restriction adds the computed geometric loss and `restriction`. It
changes the computed flow and pressure, so it can indirectly change the
physical acoustic source; it is not yet a complex acoustic impedance. Changing
only `acoustic_gain` does not change the production physical network. That
field only acts if the render has to use its old reconstructed fallback path.

### Directional X-pipe

An X-pipe is not a `merge` followed by a `splitter`. That old notation forms an
ideal common plenum: a wave from bank 0 and the same wave from bank 1 become
exactly identical downstream. The schema 9 `crossover` type therefore keeps
four explicit ports:

```yaml
- { id: 200, type: crossover, length_mm: 0, diameter_mm: 80,
    volume_l: 0, restriction: 0.06, crossover_coupling: 0.30 }
# inlet from bank 0 / bank 1
- { from_component_id: 110, to_component_id: 200, to_port: 0 }
- { from_component_id: 120, to_component_id: 200, to_port: 1 }
# straight-through outlet 0 / 1
- { from_component_id: 200, from_port: 0, to_component_id: 210 }
- { from_component_id: 200, from_port: 1, to_component_id: 220 }
```

Each port must be connected once, by a finite duct component. The X itself
requires `length_mm: 0` and `volume_l: 0`: the really measurable lengths before
and after the intersection belong to the four neighbouring tubes, which avoids
a hidden or double-counted length. A direct connection to a cylinder, merge,
splitter or another crossover is rejected.

In power-normalised pressure coordinates `q=sqrt(Y)*p`, with `t=sqrt(1-k²)`,
the port order being inlet 0, inlet 1, outlet 0, outlet 1:

```text
q'_inlet0  =  t q_outlet0 + k q_outlet1
q'_inlet1  = -k q_outlet0 + t q_outlet1
q'_outlet0 =  t q_inlet0  - k q_inlet1
q'_outlet1 =  k q_inlet0  + t q_inlet1
```

The matrix is real, orthogonal and reciprocal: it conserves `sum(Y p²)` exactly
and adds no gain. `k=0` gives two separate straight passages; `k=1` swaps the
outlets. The sign of the cross mode preserves the modal parity, instead of
summing four in-phase pressures in a common node.

The low-band gas solver keeps a single well-mixed volume for the intersection.
Without an authored volume, it derives `2 A d`, i.e. two tube sections one
diameter long. This avoids underestimating a four-port component by half and
keeps its CFL length `V/sum(A_port)` at `d/2`. The audio cost stays four mixes
per sample, with no extra voice, allocation, cell or delay line in the X itself.
The four `sqrt(Y)` are only recomputed when the target gas state is published at
the start of a block, then interpolated per sample; they are not needlessly
recomputed at 48 kHz.

`crossover_coupling` must be a measured calibration, or an estimate labelled as
such. The current model is compact and frequency-independent: it does not
infer the coupling from an angle, an overlap length or a 3D geometry that the
schema does not hold yet.

### Terminal resonator

A `resonator` connected from a component, with no outlet and no cylinder
mapping, is a sealed side branch. It adds no mean-flow route. The audio adds a
bidirectional line with a `+1` pressure reflection at its end. A positive
`volume_l` turns that end into a passive compliant cavity; zero gives a rigid
quarter-wave branch.

With `resonance_hz = 0`, `length_mm` is the acoustic length. A positive
frequency replaces that length, at the graph's reference temperature, with
`c/(4f)`. The simulated medium then keeps varying the speed of sound and
therefore the tuning. There is no added oscillator and no corrective filter. If
a cavity is also configured, the entered frequency tunes the neck length and
not the final resonance of the neck-cavity assembly.

The outlet's `discharge_coefficient` is a flow contraction coefficient. It is
applied once, as the effective outlet area `A·Cd`, and therefore no longer
enters the restriction as a `1/Cd²` factor: that double counting reduced the
same effective area a second time and underestimated the outlet flow.

### Variable cross-section

`diameter_mm` is the diameter at the component inlet. `outlet_diameter_mm` is
optional: zero keeps a constant area, while a non-zero value describes a
conical fitting whose radius varies linearly. The implicit volume is that of
the truncated cone,

```text
V = pi L (r_inlet² + r_inlet r_outlet + r_outlet²) / 3
```

and not an arbitrary average of the diameters. The quasi-1D mesh uses the exact
area of each face, weights the conservative fluxes by those areas and adds the
geometric term `p dA/dx` to the momentum equation. A uniform pressure at rest
therefore stays an exact equilibrium. The inlet and outlet areas are also kept
all the way to the acoustic network so that each junction uses its own
admittance `A/(rho c)`.

The graphical designer exposes both diameters. Older documents without
`outlet_diameter_mm` keep exactly their historical cylindrical duct.

## Connection rules

Validation enforces:

- one to eight paths, each cylinder assigned exactly once;
- 1 to 256 components and at most 1,024 connections per graph;
- at most 4,096 expanded routes between cylinders and outlets;
- at most eight terminal `resonator`s across the whole engine, an explicit
  bound on the number of extra acoustic delay lines;
- a unique component ID and unique edges without self-loops;
- a unique mapping for each cylinder of the path;
- one inlet and one outlet for `pipe`, `muffler`, `catalyst` and an inline
  `resonator`;
- exactly one incoming component connection, no cylinder mapping and no outlet
  for a `resonator` used as a side branch;
- at least two inlets and exactly one outlet for `merge`;
- exactly one inlet and at least two outlets for `splitter`;
- at least one inlet and no outlet for `outlet`;
- no cycle, no unreachable component and every flow branch ending in an outlet
  (terminal `resonator`s are the only sealed leaves).

An error fails the whole import; the application never runs a partially valid
network.

## Compiler diagnostics

`ExhaustGraph::makeForEngine` always returns a usable graph, even when it
receives a topology that application validation would have rejected: the gas
solver and the audio must stay safe in every circumstance. But every fallback
it takes discards part of the author's intent, and it now reports it instead of
silently substituting a default. `ExhaustGraph::diagnostics()` is empty when the
topology was compiled as written.

| Diagnostic | Meaning | `relatedId` |
|---|---|---|
| `topologyRejected` | a cylinder is not covered exactly once; the written paths were replaced by a single generated path | the faulty cylinder |
| `routeLimitReached` | the 4,096-route limit is reached; later routes are not compiled | the current cylinder |
| `unresolvedRestriction` | a route has no finite equivalent restriction (dangling branch or cycle) and was assigned the maximum | the cylinder concerned |
| `nodeIdSpaceExhausted` | the space of generated IDs is exhausted | 0 |
| `acousticBranchLimitReached` | more than eight acoustic branches were provided without going through validation; the extra ones are omitted | first omitted component |

`topologyRejected` is the one to watch: it makes a whole custom exhaust
disappear in favour of a generic collector. By ear, that is indistinguishable
from a design that is merely disappointing.

## Non-finite values

A non-finite field (NaN, infinity) from a malformed file no longer produces the
worst possible result. A non-finite `restriction` falls back to "no additional
restriction" and leaves the geometric loss alone, instead of assigning the
maximum and muzzling the line with no symptom. A non-finite `acoustic_gain` is
treated as silence, never as maximum gain.

## Series, branches and routes

For pressure loss, common components are in series. The downstream branches of
a splitter are combined in parallel with:

```text
K_parallel = 1 / (Σ 1 / √K_branch)²
```

The equivalent restriction of each cylinder contributes to the physical
conductance of the path and to the attenuation of the event. Every
cylinder-to-outlet route is also enumerated. Its delay uses `c = √(γRT)` at a
reference exhaust temperature (ambient + 405 °C by default, or a temperature
explicitly given to the compiler). Each route keeps up to eight modes: the
route's odd quarter-wave modes, and local resonators and mufflers. Close modes
are merged and ranked by energy, instead of simply keeping the highest frequency
encountered.

At a split, energy is shared according to the approximate downstream admittance
`A / √(1 + K_downstream)`. The branch amplitude receives the square root of that
share of energy. Gain products are accumulated in the log domain and clamped to
8 so as to stay finite even on a large DAG.

For an authored DAG, `ExhaustNetworkLayout` keeps every physical component:

- tubes, catalysts, mufflers, resonators and outlets become quasi-1D ducts with
  length, volume, area, hydraulic diameter and loss;
- merges, splitters and crossovers become finite junction volumes for the gas;
  the crossover keeps its acoustic ports separately;
- each valve and each outlet keeps its area and discharge coefficient;
- the interfaces share a single Riemann flux, so a branch can create neither
  mass nor energy depending on iteration order.

The low-band solver directly computes pressure, temperature, composition, flow
and reversion in each component. The old aggregated conductances and the
analytic back-pressure closure are no longer used by `EngineSimulator`.

For physical audio, the complete DAG provides the lengths and areas of the
characteristic guides. The historical metrics `audio_volume`,
`sound_attenuation`, component gains, preset modes and `FiringEvent`
transmission do not colour the SI boundary. Firing order, pressure, flow,
temperature and geometry are enough to produce the acoustic characteristics.

Each component of finite length becomes a bidirectional delay line. Direct
interfaces, merges and splitters use passive admittance scattering; a crossover
uses its passive, matched four-port matrix. Trunk lengths carried by a branch
stay ducts, and each outlet keeps its own radiation load, position and axis. A
4-1 and a 4-2-1 are therefore not reduced to the same path as soon as their
geometries differ. The causal and analytical proofs are in
`tests/RealtimeRegressionTests.cpp`.

## Multiple paths

A V or a flat engine can declare two `exhaust_paths`, each with its cylinders,
its graph and, optionally, an explicit measured IR. Gas and audio keep these
paths separate. Runtime indices follow the array order, while the authored IDs
stay the references for banks and serialisation.

## Compatibility with existing configurations

`graph` is optional. Without it, EngineLab compiles the old geometry fields
into primaries, merge, muffler and outlet. Engine files of schemas 1 to 8 stay
readable and are migrated in memory to schema 9. Every new JSON/YAML export
carries `schema_version: 9`. Schema 8 adds the explicitly parameterised
afterfire induction correlation; schema 9 adds the `crossover` type,
`crossover_coupling`, `from_port` and `to_port`. A document older than schema 8
receives zero exponents and keeps its flat timer exactly. The historical
catalogue files deliberately stay migration fixtures; the absence of the three
substrate fields keeps the exact bypass.

Even with a graph, the `geometry` block stays useful: it provides the fallback
values needed for the physical compilation of an old configuration. No WAV now
means free field; no IR is generated.

## What the graph does not simulate

The non-linear solver does resolve each component, but only in the band needed
for real-time flow and back-pressure. The audio high band keeps the DAG, but
stays linear and plane. It does not resolve transverse modes, 3D bends or the
complete mean-flow correction of radiation. The simple termination directivity
and the independent outlet positions/axes are resolved towards a common
microphone pair; the model is neither a 3D acoustic field nor a room
simulation. The compact X is not yet frequency-dispersive, and the DAG still
forbids the loop needed for a literal H-pipe; both limitations are explicit.

The guide's high-frequency reflections do not return into the 0D cylinder; the
physical feedback is provided by the non-linear low-band network. IRs are
loaded only through an explicit `impulse_response` and must represent a
downstream measurement. See
[thermoacoustic-architecture.md](thermoacoustic-architecture.md) for the
equations, invariants and owning files.
