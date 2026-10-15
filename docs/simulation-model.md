# Simulation model

EngineLab is a real-time control-volume simulator. Its equations are designed
to stay consistent and observable within an interactive budget; they are
neither a CFD model nor a performance certification.

## Integration and angular resolution

The public `EngineSimulator::step(dt, controls)` call is split into sub-steps.
The requested rate is the maximum of the minimum mechanical rate, including
`gasSubsteps`, and the rate required to honour `maximumCrankDegreesPerStep` at
the current engine speed. It stays bounded by `maximumMechanicalFrequencyHz`.

Validation rejects a configuration whose maximum rate could not hold the
promised angular resolution at the rev limiter. The solver publishes the
angular step actually reached and any overshoot; it does not hide an
under-resolution behind a nominal rate.

## Kinematics, pressure and torque

`MechanicalKinematics` solves the slider-crank geometry from the normalised
crankshafts and journals. Conventional, master and articulated rods share the
same API for position, velocity, acceleration and lever arm. Deck height,
compression height, pin offset and articulated-journal radius take part in the
TDC position when provided.

At every sub-step, each cylinder's pressure acts on the piston area and the
exact lever arm. Gas, reciprocating, friction, starter, load and transmission
forces form the net torque, then:

```text
angular acceleration = net torque / equivalent inertia
```

Running torque comes from this resolved pressure. No empirical mean torque is
mixed into the crankshaft dynamics.

In parallel, `IndicatedWorkModel` integrates the signed loop
`∮(P_cylinder - P_ambient)dV` with the trapezoidal rule. At each cycle change,
it publishes indicated work, IMEP, indicated power and equivalent mean torque.
This reading is for diagnostics and balances, without becoming a second torque
source.

Piston/liner friction uses a Stribeck law per cylinder: breakaway force,
Coulomb, low-speed transition and a viscous term. The direction is defined even
near zero velocity, which avoids a friction that would artificially accelerate
the piston.

## Conservative gas network

The atmosphere, plenums, runners, cylinders, primaries and collectors are
`GasCell`s. A cell stores the amount of each species, the internal energy, the
volume, an orientation, a characteristic area and a 2D momentum.

Restrictions use their real area and a discharge coefficient. The flow becomes
choked when the pressure ratio reaches the sonic condition; otherwise it
follows the subsonic isentropic relation. The directional dynamic pressure is
signed: momentum directed towards the restriction raises its effective total
pressure, opposing momentum lowers it.

A transfer simultaneously carries:

- oxygen, inert gas, fuel vapour and burned products;
- mass and momentum on both axes;
- stagnation enthalpy and macroscopic kinetic energy.

An equilibrium search bounds the transfer before the non-physical reversal of
the gradient. Dissipating momentum beyond the speed of sound converts the
kinetic energy back into heat. The effective properties `Cv`, `gamma`, molar
mass and speed of sound depend on the composition.

Temperature and pressure stay derived from all the conserved internal energy,
including beyond the usual thermal range of the engine: no display ceiling can
hide an energy reserve in a cell.

During valve overlap, intake and exhaust are evaluated from the same starting
state and then applied together. This transaction prevents an arbitrary call
order from changing the gradient seen by the second valve.

The model nonetheless stays 0D per volume. Momentum provides directional
inertia; it does not turn a runner into a meshed tube where a wave propagates
spatially.

A stopped crank (below 20 rpm, starter off) passes no gas through its valves,
and the 1-D intake runners stand at their plenum's pressure instead of
advancing (`ExhaustGasNetwork::settleAtRest`). Held at equilibrium, an open
valve's quasi-steady law and the runners' explicit plenum coupling both grew
roundoff into tens of kPa (see `docs/journal.md`, 2026-10-06).

## Valvetrain and intake

`ValveTrainModel` evaluates the profile of the cylinder's bank. Lift profiles
and lift/discharge-coefficient curves are sampled and interpolated. An
RPM/load calibration can continuously drive intake advance, exhaust advance and
the lift multiplier; the actuators follow their target at a configurable
response rate. The switched high profile is kept for older configurations.

`HelmholtzRunnerModel` attaches a damped mode to each runner. Its frequency
depends on the area, the length, the volume and the local speed of sound. The
pressure of this mode changes the admittance of the conservative restriction:
it creates neither mass nor a torque bonus independent of filling.

## Injection and mixture

Injector flow depends on its nominal capacity and the square root of the
pressure differential. For port injection, `rail_pressure_bar` is the
differential pressure regulated against the manifold: flow therefore does not
collapse under boost. For direct injection, it is an absolute rail pressure and
the instantaneous cylinder back-pressure is subtracted.

- With direct injection, fuel goes into the cylinder and its latent heat cools
  the charge according to the configured efficiency.
- With port injection, a fraction joins a persistent liquid film on the port.
  Its evaporation depends on temperature and a time constant.

The commanded mass, the film, the vapour available at spark and the fuel
actually consumed stay separate. A window that is too short, an undersized
injector or a slow film therefore reduce the fuel actually burned.

With port injection, the pulse is sized once per cycle, at the first sub-step
of the window: the charge's need, minus the film available before spark
(X-tau) and the fuel of a chamber that misfired, divided by the share of the
fresh pulse that is available. The port vapour is not credited: it is a
stationary reservoir, not fuel for the next charge.

Gas pushed back past the throttle is kept in the airbox (`airbox_volume_l`) and
drawn back in first. Without an airbox, the fuel it carries is lost.

The reported AFR and lambda come from the trapped species. A per-cylinder
corrector learns the transport losses of the previous cycle. Its bandwidth
depends on the cycle duration and, with port injection, on the film's
vaporisation constant; this avoids lambda hunting on big slow engines. The ECU
tables set the AFR target and the advance as a function of engine speed and a
normalised load; acceleration enrichment, cold start, temperature, knock and
rev limiter apply after that.

## Ignition, flame and knock

A spark is scheduled per cylinder. Before the kernel is born, an ignition delay
depends on pressure, temperature, equivalence ratio and residuals. The laminar
speed follows a Metghalchi-Keck type correlation; a closure based on mean
piston speed adds turbulence, while dilution reduces speed and efficiency.

The current progression geometry is a cylindrical effective volume:

```text
V_burned = π × radial_travel² × axial_travel
```

Both travels are bounded by the bore radius and the equivalent instantaneous
chamber height. It is neither an ellipsoidal front nor a resolved 3D surface.
The geometric fraction drives an absolute number of moles to react; the
released energy uses the fuel's LHV and stoichiometry.

Knock uses a Livengood-Wu integral on the end gas. When its threshold is
reached, a share of the remainder really auto-ignites in the cell, raises the
pressure and feeds the telemetry. The ECU then pulls advance in proportion to
the knock level, so at WOT engines sit at a light, constant knock (0.1-0.4)
rather than stepping back and holding as a real knock control does. Only knock
above 0.45 (the diagnostics' "severe") adds damage, which lowers efficiency for
good. This global correlation is not multi-species chemical kinetics.

## Forced induction

The turbocharger follows a power balance: turbine minus compressor and bearing
losses, integrated with the shaft inertia. The wastegate (or variable vanes)
follows a proportional-integral boost controller that holds
`wastegate_pressure_ratio` exactly: fully open 0.25 above it on the
proportional term alone, integral time 0.5 s, frozen while the gate is
saturated. Like an actuator spring set against the atmosphere, it holds a
gauge boost: the ratio is the one at sea level (101.325 kPa), and in
thinner air the ratio it allows rises so the gauge boost stays the same. The supercharger uses a separate closure, without claiming to
model a complete compressor map.

The turbine lives in the exhaust gas network, as a restriction face (the same
primitive as an individual throttle) on the single outgoing face of the first
junction where all of a path's cylinders have met. The face opens to the
turbine plus open wastegate area, times the path's share of the cylinders.
Every pulse crosses it as an isentropic orifice fed by the collector's
stagnation state; the turbine's share of the flow (its area over the total)
gives up `eta * cp * T0 * (1 - (p_out/p0)^((gamma-1)/gamma))` per kilogram,
taken from the downpipe's energy, so the downpipe receives cooled gas and the
shaft the work. The receiving side gets no jet momentum: the turbine took it,
and the wastegate jet is dissipated in the housing. `EngineLab.RestrictionFace`
checks the choked flow, the work against its isentropic ceiling, the energy
balance and the cooling. The manifold therefore sits at turbine inlet
pressure and the downpipe at turbine outlet pressure, pulse by pulse.

If a path has no such junction (a custom graph without a collector), the
older model applies: every outlet is narrowed by the turbine and wastegate
areas, and the shaft is driven from the peak exhaust pressure with a
quasi-steady expansion. That expansion overstated the work, so the catalogue
turbine areas were re-fitted when the turbine moved (2026-10-12, see
`docs/journal.md`).

### Fitting a turbo to any engine

`sizedTurbocharger()` (foundation, `TurboSizing.hpp`) sizes a turbocharger for
an engine that has none, from scaling laws fitted to the four catalogue
turbochargers (turbine area 0.028 mm² per litre and per redline rpm, full
boost at 45 % of the redline, inducer from the square root of the air flow,
rotor inertia from the inducer). The boost follows the compression ratio, as
on a street conversion: 0.5 bar from 10.5:1, 0.7 from 9.5:1, 0.9 below, 1.3 on
a diesel. The engine keeps its own ignition map, so a high-compression engine
may knock at boost. The 3-D view's inspector fits it (or takes it off) from any
exhaust component or from the turbo, and the engine restarts. Every engine
with a collector gets the turbine face described above, one per path. An
engine without one (the LS3's X-pipe, the Big Twin's two pipes) falls back to
the older outlet model, whose shaft work is overstated, and the view does not
draw it.

## Exhaust

Each `ExhaustPathConfig` can hold a custom DAG. The physical compiler keeps
tubes, resonators, mufflers, catalysts and outlets component by component as
quasi-1D ducts; merges and splitters become finite junction volumes. Interfaces,
valves and outlets exchange a common bidirectional Riemann flux. Pressures,
temperatures, composition, flow and back-pressure therefore come from the
conservative network and not from an equivalent 0D throat or collector.
Without a graph, the historical geometry is first expanded into a compatible
physical DAG. See [custom-exhaust.md](custom-exhaust.md).

Cylinders use Woschni's instantaneous convective correlation rather than a
constant conductance. Intake ducts and every cell of the exhaust network have a
metal wall with finite heat capacity. Internal exchange is integrated
analytically and conserves gas + wall energy; only the explicitly accounted
external convection rejects energy to the ambient. The model therefore never
clamps EGT to hide excess energy.

The thermodynamic network and the audio renderer deliberately use two scales:
a non-linear low-band mesh for flow/back-pressure, and a linear characteristic
network for audible propagation. The instantaneous SI flow links the two at
every mechanical sub-step. The outlet load is a passive radiation model; an IR
is used only if it is explicitly provided. See
[thermoacoustic-architecture.md](thermoacoustic-architecture.md).

Intake runners can also define `runner_plenum_diameter_mm`. The historical
`runner_diameter_mm` then designates the valve side and the new field the
plenum side; zero keeps a constant area. Intake and exhaust share the same
conservative conical discretisation (exact volume, local areas, friction and
heat exchange computed with the local hydraulic diameter).

`individual_throttles: true` puts one plate in each runner instead of one
ahead of the plenum (motorcycle and racing throttle bodies). The plenum is then
the airbox the runner mouths draw from, fed through the bores held fully open.
Each runner becomes two 1-D ducts meeting at a restriction face, placed
`throttle_distance_from_valve_mm` from the valve (zero: 55 % of the runner, a
default not measured on any engine). The face is an isentropic orifice fed by
the approaching cell's stagnation state, choked when it must be; the jet
re-expands into the receiving duct against the pressure-loaded rest of the
face, so the plate costs nothing fully open and its loss is the Borda-Carnot
expansion otherwise. Each plate takes its share of the throttle, leakage and
idle bypass area. The manifold pressure the ECU reads is the mean over the
valve-side ducts (all their cells, all runners) through a 30 ms first-order
tapping manifold; the cell right behind the plate reads the jet's low pressure
and costs 10-19 % torque through a false load reading. A small 0-D
cell per cylinder was tried first and refuted (`.claude/CLAUDE.md`).

## Engine temperatures

By default the thermal model is **regulated** (`thermal.regulated`): coolant
and oil sit at `regulated_coolant_c` (90 degC) and `regulated_oil_c`
(100 degC) from the start and never move, as a dyno cell's conditioning unit
holds them. Every run, A/B switch and harness measurement is then made on a
warm engine at one temperature, the state the CP2 recording protocol asks
for. Unregulated (realistic, `EngineThermalModel`), both start at the
ambient temperature. The coolant capacity stands for what a closed thermostat
circulates (the water jacket and the metal around it), the oil exchanges heat
with it through the block, and they take shares of the heat released by
combustion (24 % and 5 % by default; Heywood gives 17-26 % to the coolant at
full load) while the oil takes all the friction heat. A liquid-cooled
engine sheds heat through a radiator its thermostat opens from 82 to 94 degC,
in the air of its speed (`EngineControls::coolingAirSpeedMps`: the vehicle's
speed, or 12 m/s of dyno cell fan on the dyno) and of an electric fan, on at
100 and off at 95 degC. An air-cooled one (`liquid_cooled: false`) sheds it
through its fins, in the air of its speed and of its blower if it is driven by
the crank (`engine_driven_fan`): a tenth of their design cooling always, the
rest through an oil thermostat opening from 85 to 100 degC (the 911's opens its
front cooler near 83 degC); its temperature stands for the oil's, as its
gauge reads it. Unauthored sizes (`0`) follow the displacement: 14 kJ/K per
litre plus 3 for the coolant side, a radiator that rejects the heat of the
estimated rated power (9 kW per litre per 1,000 rpm of redline, more with
boost) and its friction at 30 m/s and 65 K above the air. These are sizes for a plausible engine,
not measured ones. The ECU enriches and retards a cold or hot engine from the
coolant temperature. The switch is a settings edit, taken live.

## Transmission and vehicle

`DrivelineModel` owns the clutch, gearbox shaft, differential, driven wheel and
vehicle. Reverse, neutral and forward gears share a state machine with
declutching, shifting, re-engagement and torque reduction.

The clutch is a dry brake with a stick/slip law (Karnopp), not a viscous
coupling. Outside the `clutch_lock_speed_rpm` lock window, it slips and
transmits its full capacity opposing the slip (kinetic friction, independent of
the magnitude): that is what launches the vehicle and heats the disc. Inside
the window, it sticks: crankshaft and input shaft form a single body, and their
common acceleration is solved from both inertias, the engine's own torque and
the reflected road load; the exact torque that holds synchronism follows from
it. Since this is the solution of the constraint and not a steep slope, the
reaction cannot overshoot over one step, so an engaged clutch holds the engine
torque with a few rpm of residual slip instead of slipping endlessly. The
capacity — bounded, then degraded by heating and fade — always limits it:
beyond it, the clutch breaks away and slips. The wheel stays a degree of
freedom separate from vehicle speed. Its slip generates a longitudinal force
bounded by grip. The normal load on the driven axle responds to traction and to
the quasi-static transfer `m*a*h/L`; forward acceleration loads a rear-wheel
drive, unloads a front-wheel drive and keeps the total weight available for
all-wheel drive. Drag, rolling resistance and brakes then dissipate the energy.
The runtime sub-samples this coupling at 1 ms.

### Shift styles

`transmission.shift_style` says how a change between two forward gears is
made; neutral and reverse are always selected the manual way. It is a settings
edit, taken live.

- `manual` (default): the driver lifts (`shift_torque_cut_fraction` of the
  throttle) and dips the clutch over `shift_duration_s`; after the timer the
  torque comes back with the clutch slip, not the clock.
- `sequential`: a dog box, clutch left in. With `quickshifter` (default on),
  an upshift cuts the ignition for `quickshifter_cut_s` (default 65 ms); the
  ECU cuts as its rev limiter does (spark, and fuel too unless
  `ignition.limiter_keeps_fuel`, in which case the unburnt charge reaches the
  exhaust). The dogs change after 15 ms (drum travel, not measured) and the
  clutch pulls the engine down to the new gear's speed while it makes no
  torque; a cut too short leaves the engine firing into the slip. With
  `auto_blipper` (default on), a downshift frees the dogs and opens the
  throttle (wide open until the last 15 % of the climb) until the engine
  reaches 98 % of the new gear's speed, at most 0.3 s, then engages. Without
  either device, that shift is made the manual way.
- `dual_clutch`: an upshift hands the drive to the oncoming clutch at once,
  with no torque gap. Its pressure carries the engine's torque plus what pulls
  the engine down to the new speed over `shift_duration_s`; the engine's torque
  is not reduced meanwhile, so the wheel feels that inertia as a push. A
  downshift is blipped as above. The clutch pedal is not connected: the
  gearbox runs its own launch clutch. At idle, foot off, it holds a creep
  torque (25 Nm, or 5 % of the clutch's capacity if lower), released on the
  brake. With the throttle open it closes with the engine's speed, fully at a
  launch speed that rises with the throttle to 38 % of the redline. It opens
  before the engine stalls, stays shut once in sync above 1.2 × idle, and
  opens again below 1.1 × idle. It fills in 0.10 s and dumps in 0.03 s (not
  measured). A downshift that would land the engine above the active rev
  limit goes only as far as the lowest gear that does not; a manual or
  sequential box lets the driver over-rev.

Whatever the style, a shift started with the driver's clutch held out (pedal
below half) is the driver's own: the gear changes at once and no device
(torque cut, quickshifter, blipper) acts. Pulling the clutch during a blip
ends the blip.

There is no dog-box engagement noise and no torque converter.

### Automatic shifting

With `transmission.automatic_shifting` (switched live with the **Auto**
button or `8`), `ShiftSchedule` chooses the gear from the driver's pedal, not
the engine's throttle, so a blip or a torque cut does not move the points.
The upshift point runs from max(2.2 × idle, 0.3 × limit) foot-light to
0.94 × the active rev limit flat out (`automatic_upshift_rpm` replaces the
latter when set), along pedal^1.5; the downshift point from 1.3 × idle
(`automatic_downshift_rpm`) to 0.55 × the full-throttle upshift, which is the
kickdown. An upshift must land above 1.15 × the downshift point and a
downshift under 0.9 × the upshift point, so a gear it lands in is kept; a
kickdown goes down several gears while the gear reached still lugs. It waits
0.8 s after any shift and never acts while the driver holds the clutch. The
shape is a choice, not a fit to a production schedule. A change between two
forward gears asked by the driver switches the gearbox back to manual;
selecting first from neutral does not.

### Driver commands

`DriverCommandArbiter` (`src/runtime`) is the one place where the driver's
pedals and the driveline's own devices become engine commands. Each public
frame, `advanceVehicle()` gives the driveline the clutch the arbiter chooses
(the driver's pedal; a dual clutch ignores it), advances it on the previous frame's engine state,
then sets the engine throttle to `max(driver x torque cut, blip)` and passes a
quickshifter's ignition cut through. It also records who held the throttle
(`driver`, `shiftTorqueCut` or `blipper`). `EngineRuntime`, `EngineLab.ShiftStyle`
and the shift, loaded-acceleration, afterfire and drive-scenario harnesses all
step the vehicle through it, so they cannot drift from the product. The dyno
bypasses it: the bench, not a driver, holds the engine.

The cruise control (`CruiseControl`) is a driver-side device: a PI on road
speed (0.08 per m/s, integral 0.03) whose throttle joins the driver's pedal,
the higher one winning, before the arbiter. It engages from 30 km/h at the
throttle in use (no surge); past the set speed and still gaining, or under it
and still losing, that throttle moves at 0.5/s, so a set pressed while
accelerating hard overshoots by a few km/h only. The brake, the clutch,
neutral or falling under 0.6 × the set speed cancel it. The runtime merges it once per frame, so
the driveline and the arbiter see the same inputs.

In vehicle mode, the runtime's external load (`EngineRuntime::setLoad`; the
application keeps it at zero since it no longer has a *Manual load* control)
is a longitudinal resisting force; it reaches the crankshaft only through the
wheel, gearbox and clutch. In dyno mode, the
brake acts directly on the crankshaft and the vehicle is decoupled. The two
paths are never applied at the same time.

The balances publish stored energy, dissipated energy and residual. The model
includes no suspension/pitch dynamics, no ABS, no detailed synchronisers and no
full Pacejka; load transfer is a quasi-static longitudinal equilibrium.

## Limitations and interpretation

- four-stroke petrol only, despite the presence of types reserved for future
  extensions;
- 0D gas volumes and an aggregated Helmholtz mode, without CFD or meshed 1D
  acoustics on the intake side;
- semi-empirical global reaction, turbulence, walls, blow-by, film and knock;
- no spray, temperature field or 3D flame front;
- multiple crankshafts linked by rigid ratios, without torsion of their own;
- catalogue parameters not certified by a flow bench or an engine dyno;
- power and torque useful for internal comparison, not for an engineering or
  tuning decision on a real vehicle.

The tests check invariants, finiteness, trends and regressions. They do not
replace experimental calibration.
