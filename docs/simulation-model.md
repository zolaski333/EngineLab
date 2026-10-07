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
pressure and feeds the telemetry. The ECU then pulls advance. This global
correlation is not multi-species chemical kinetics.

## Forced induction

The turbocharger follows a power balance: turbine minus compressor and bearing
losses, integrated with the shaft inertia. The turbine and wastegate areas
influence the manifold flow and therefore back-pressure, spool and pressure
ratio. The supercharger uses a separate closure, without claiming to model a
complete compressor map.

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
