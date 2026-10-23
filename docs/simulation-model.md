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

### Two-stroke engines

`cycle: two_stroke` gives a piston-ported, crankcase-scavenged petrol engine
(reed or piston-port intake, one firing per cylinder per revolution). The
crank angle stays a 720-degree domain for every engine, so the render, the
crank clock and the gas field are unchanged; each cylinder's thermodynamic
phase runs on 360 degrees (`EngineSimulator::cylinderCycleDegrees()`), and
spark, injection, the trapped-charge latch and the cycle telemetry follow it.

- **Ports, not valves** (`TwoStrokePorts.hpp`): the exhaust and transfer
  ports open when the slider-crank piston uncovers their top edge
  (`two_stroke.exhaust_port_open_deg`, `transfer_port_open_deg` after top
  dead centre) and are fully open at bottom dead centre. The open area is
  width x uncovered height x `port_discharge_coefficient`; widths default to
  0.65 and 1.30 x bore. No cam, no valve lift (published lifts are zero).
- **Crankcase pumping**: each cylinder has its own intake path (validation
  enforces one cylinder per path). That path's plenum is the crankcase: its
  volume follows the piston's underside every substep, adiabatically, between
  `crankcase_compression_ratio` x its bottom-dead-centre volume and that
  volume. The throttle is the carburettor bore with a reed check valve (no
  flow out of the crankcase), and the runner is the transfer duct. The piston's
  underside sees the crankcase pressure, not the ambient.
- **Trapping**: the charge is trapped when the transfers close (symmetric
  about bottom dead centre); short-circuit losses come out of the same 1-D
  exhaust flow that scavenges the cylinder. There is no pumping loop in the
  cylinder: all of its work is gross work.
- **Scavenging** (`TwoStrokeScavenging.hpp`): one well-mixed cylinder cell with
  two ports onto it *is* perfect mixing, so trapping came out a function of
  delivery ratio with no engine speed in it anywhere (0.448 at 2,000 rpm to
  0.807 at 9,000, straight along that relation). A real loop-scavenged cylinder
  displaces first: the transfers aim the charge up the far wall, it loops over
  the head and pushes burned gas out ahead of it, and only once that front
  reaches the exhaust port does fresh charge start escaping. Those are the
  classical bounds -- displacement `SE = min(DR, 1)`, mixing
  `SE = 1 - exp(-DR)` -- and the simulator sat on the lower one.
  What places an engine between them is a race between the front's travel and
  the port window. The front's progress is *integrated* every sub-step,
  `frontProgress += jetVelocity x dt / loopLength`, with the loop length
  `scavenge_loop_length_ratio` x bore and the jet velocity read from the
  crankcase-to-cylinder pressure difference; displacement lasts until the
  progress reaches 1 and mixing is established over one further loop length.
  The window lasts `port_open_duration / (6 x rpm)`, so a faster engine's front
  covers less of the loop and the displacement phase owns more of the window --
  which is why a two-stroke scavenges well near its tuned speed and is dirty
  when lugged. No pressure across the ports means no loop: the front stays
  where it is and the cylinder mixes as it used to.
  Integrating rather than predicting a transit time is not a refinement. An
  earlier version sampled the jet velocity once, at the instant the transfers
  cracked open, and in the 5,000-6,000 rpm band that instant is where the
  exhaust pipe's returning wave sits: the loop closed through it (strong jet,
  more trapped, more exhaust energy, stronger wave, different pressure at the
  next opening) and the RD350's brake torque stopped holding, 13.3 to 37.6 N.m
  around a 25.6 N.m mean at 5,999 rpm against 6 % spread with the model off
  (journal 2026-10-21). A front does not travel at one instant's orifice
  velocity in any case.
  The bias is applied to the *composition* the exhaust port offers the 1-D
  network, not to the port areas: the network reports back exactly the species
  masses it took and the cylinder is debited those same masses, so species,
  mass and energy conserve for any composition presented. Only the composition
  moves -- the energy density is passed through untouched, because pressure and
  temperature drive the flow and this model says what the gas leaving is, not
  how much of it leaves. A cylinder holding less burned gas than the fresh
  charge being held back (cranking, the first cycles) has nothing to displace
  with, and the bias reduces itself to what the residual can cover. A loop
  length of zero restores the perfectly mixed cylinder exactly, and a
  four-stroke never opens a window, so its exhaust boundary is bit-for-bit
  unchanged.
  Measured, one binary with only the loop length changed: the curve moves
  +2.2 % in the mean and +26 % at 5,752 rpm, where it fills an antiresonance
  (23.1 -> 29.1 N.m), and less than 1 % below 3,250 or above 7,000. The band is
  not a tuning. Below it the displacing share is a tenth of the window, so
  there is almost no bias; above it trapping is already 0.83 and the cap on
  what the residual can cover binds. It therefore does **not** address torque
  climbing to the rev limiter: that is trapping which never falls, and the
  displacing share is monotonic in engine speed by construction.
- **Carburettor**: the fuel system a two-stroke actually has, and what the
  designer fits to every two-stroke it builds
  (`IntakeConfig::carburettor`, `CarburettorModel`). Nothing is commanded: the
  jets are orifices into a float bowl at the pressure outside the engine, and
  the mixture is whatever depression the air draws through them. The fuel goes
  into the crankcase with the air that metered it, in the bore, so the lag to
  the cylinder is the crankcase's own residence time rather than an injector
  command's guess at it.

  Two circuits run in parallel and are never blended, because each is driven by
  a depression the other does not see. The main circuit has the venturi's own,
  which goes as the square of the throat velocity and is worth tens of pascals
  at idle -- less than the lift of the jet above the bowl, so it passes nothing
  there at all. The pilot circuit has the whole drop from the bowl to the
  crankcase, largest with the slide shut and gone at full throttle, where the
  crankcase pumps at about ambient. Each fades out where the other takes over
  and no crossover term is written anywhere. Between them the needle taper
  meters, riding with the slide over the first three quarters of the opening,
  which is what the clip position moves.

  Jets carry the number stamped on them: cc/min of test fluid under one metre
  of head, so flow is the number times the root of the head available. The
  designer sizes them from the engine and not from a table -- the fuel peak
  power needs at AFR 12.8 over the root of the venturi head there, and the
  fuel the idle air needs at AFR 13.5 over the root of the crankcase head. The
  idle air is what the idle path passes at that head (the slide's leak and the
  bypass at the idle governor's neutral opening), and as the ECU opens or
  shuts that path, standing in for the slide's idle stop, the pilot follows it
  the way progression holes do: the idle mixture holds through cranking and
  idle. On an RD350-sized design the main comes out at 287 against the bike's
  260-290. The main jet is very nearly its circuit's only restriction so its
  number is used directly; the pilot's is not, because the emulsion passage,
  the air screw's bleed and the outlet all throttle it further, and
  `pilotCircuitFactor` is that lumped restriction (0.026, the one value fitted
  to a real pair: an RD350's 35 pilot feeding its idle air at AFR 13.5 on about
  20 kPa). A pilot sized for an idle delivery ratio of 0.35, about eight times
  what the model's RD350 idles on, flooded every designed twin.

  Two things a real carburettor has are deliberately absent, both of which act
  at the top of the range: there is no emulsion tube or air corrector, so the
  main circuit keeps the plain jet's richening with flow instead of being held
  flat, and the air density at the throat does not fall with the depression.
  Neither is stood in for by a factor, because the first is a two-phase effect
  in the passage that no single algebraic term represents honestly. They are
  measured gaps, not calibrations.
- **Fuelling** (an engine configured without a carburettor): the ECU meters for
  the air that crossed the reed into the crankcase during its last fill
  (latched when the transfers close, just before the pulse), which is the same
  air a carburettor would have metered; not for the trapped oxygen (most of a
  two-stroke's trapped charge is retained from earlier cycles), nor for the air
  across the transfers, which lags the fill by an event on a tip-in and left
  the RD350 lean and stalling. There is no overrun fuel cut (a cut scavenges
  the cylinder clean, and each resume fires a full charge), and lambda learning
  is held through a closed-throttle overrun and for 2 s after it.
- **Load**: a two-stroke's "manifold" is its crankcase, which pumps about
  ambient at any throttle (75-87 kPa at idle), so its load is the delivery
  ratio of its last cycle, not the manifold pressure; the ECU's load axis
  reads the same (an alpha-N calibration is the closest real input). Read as
  a four-stroke MAP, the crankcase made the ECU retard a full-load RD350 by
  about 2 deg, and read an idle as a 0.77 load.
- **Exhaust reconstruction**: an expansion chamber is a chain of short cones,
  most of them two or three cells at the two-stroke mesh. A two-stroke's
  exhaust reconstructs across every open duct-to-duct interface
  (`ExhaustGasNetworkConfig::reconstructAcrossInterfaces`), so two components
  of one section are one pipe to the solver; a four-stroke keeps first-order
  duct ends.
- **Exhaust coupling**: the exhaust network is coupled to the cylinders at
  intervals of up to 125 us (117 us, three mechanical substeps, for the RD350
  at 8,000 rpm), and the exchange reaches the cylinder as one lump. From
  bottom dead centre to the transfers' closing, a two-stroke's cylinder
  pressure is a few-kPa balance between two open ports that sets the trapped
  charge, so while any cylinder is in that window the exhaust is coupled every
  substep (`EngineSimulatorOptions::exhaustFineCouplingWhileScavenging`).
  Coupled coarsely there, the RD350 lost 10-14 % of its torque above
  6,000 rpm; coupling every substep over the whole cycle is not real time.
- **Compression ratio**: the model takes the geometric ratio. A Japanese
  maker's figure (6.2:1 for the RD350LC) is usually the trapped one, counted
  from the exhaust port's closing.
- **Limits**: petrol only, naturally aspirated, one cylinder per intake path,
  no individual throttles, injection angles inside the 360-degree cycle. There
  is no oil (premix) in the charge. An engine configured with an injector
  rather than a carburettor keeps the injector's own limit: the fuel reaches
  the cylinder about an event before the crankcase would deliver a
  carburettor's mixture, so a tip-in runs rich (AFR 9-10) for a few events. The
  expansion chamber's return wave needs fine exhaust cells: a two-stroke's
  production mesh is 100 mm
  (`EngineSimulator::productionExhaustCellLengthM`, 360 mm for a four-stroke;
  the view's fine mode never coarsens it). The catalogue RD350LC-like twin
  makes 29-33 N.m at 8,000 rpm (dyno sweep and held test) and 34 kW at
  9,025 rpm in real time, against the maker's 37-40 N.m and 34.5 kW; 25 mm
  cells coupled every substep give 35.9 N.m. What is left of the mesh gap is
  mostly the first exhaust cell behind the port, which holds about a
  cylinder's volume and acts as a plenum. 75-80 mm make its idle hunt
  (journal 2026-10-15/16).
  The script language sets the cycle but not the port timings,
  which take their defaults (exhaust 92 deg, transfers 118 deg).

## Injection and mixture

Injector flow depends on its nominal capacity and the square root of the
pressure differential. For port injection, `rail_pressure_bar` is the
differential pressure regulated against the manifold: flow therefore does not
collapse under boost. For direct injection, it is an absolute rail pressure and
the instantaneous cylinder back-pressure is subtracted.

- With direct injection, fuel goes into the cylinder and its latent heat cools
  the charge according to the configured efficiency.
- On a diesel, the mixing-controlled burn (and the premixed burn, a fixed
  fraction of its time) is calibrated at the injector's rated pressure and
  scales with sqrt(rated / rail pressure): the jet's speed, which drives the
  mixing, goes as the square root of the pressure drop (Chmela and Orthaber).
  Halving the TDI's rail pressure at full load lowers its firing peak from 198
  to 177 bar with the injector capacity held.
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
piston speed adds turbulence, while dilution reduces speed and efficiency. The
turbulence term grows with load up to a full cylinder and no further: the
piston stirs the charge, not the intake pressure, so boost does not burn it
faster.

The current progression geometry is a cylindrical effective volume:

```text
V_burned = π × radial_travel² × axial_travel
```

Both travels are bounded by the bore radius and the equivalent instantaneous
chamber height. It is neither an ellipsoidal front nor a resolved 3D surface.
The geometric fraction drives an absolute number of moles to react; the
released energy uses the fuel's LHV and stoichiometry.

Knock follows the end gas, the charge the flame has not reached yet. From the
seal it is its own zone, compressed isentropically by the cylinder pressure
with real heat capacities (air, products and gasoline vapour, fitted to the
ideal-gas tables). Its autoignition delay is Douaud and Eyzat's, fitted on
primary reference fuels in a knocking engine, with their prefactor doubled:

```text
tau = 35.4 ms x (ON/100)^3.402 x p[atm]^-1.7 x exp(3800 / T_unburned)
```

The prefactor is the constant users of this correlation recalibrate to their
engine. With Douaud and Eyzat's 17.68 ms every catalogue engine with a knock
limit, naturally aspirated ones included, held 4-13 deg of knock retard behind
its own spark map, where a production map sits 0-3 deg from its limit; with
35.4 ms the maps hold 0-5 deg and 10 deg of advance over a map knocks.

A Livengood-Wu integral of dt/tau runs from the spark. When it reaches 1 while
the flame burns, everything the flame has not reached autoignites at once and
really raises the pressure in the cell. The pressure that gas adds burning at
constant volume, (gamma - 1) x unburned share x charge energy / V, is the scale
of the oscillation it starts; the knock intensity is that over 30 bar. The
ECU's knock control retards in steps and gives the advance back slowly
([ecu-tuning.md](ecu-tuning.md)), so at full load an engine settles at its
knock limit. Only knock above 0.45 (the diagnostics' "severe") adds damage,
which lowers efficiency for good.

The correlation is global, not multi-species kinetics, and it over-predicts
knock under boost: the Audi I5 knocks heavily at its maker's 2.5 bar even at
the ECU's -10 deg floor, where the real engine (10:1, direct injection) holds
it on 98 RON (journal 2026-10-22).

Each cylinder's firing peak is tracked at every physics sub-step and published
once per cycle, when the cylinder opens into its gas exchange
(`EngineState::peakCylinderPressureBar`, the highest over the cylinders). Above
the engine's structural limit (`peak_pressure_limit_bar`; zero takes 120 bar
naturally aspirated, 140 boosted, 200 for a diesel, the upper end of production
designs) it adds damage as the square of the excess: 10 % over takes about
17 minutes to "damaged", 30 % about 2 minutes. The simulated full-load peaks of
the boosted catalogue engines ran above real ones (Audi I5 175 bar before the
knock control held it, TDI 231 bar), so those two carry their own limit 10 %
over that stock peak.

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

The turbocharger's compressor head comes from a generic map scaled to its
design point (Moraal and Kolmanovsky, SAE 1999-01-0908): the head coefficient
against the flow coefficient, both relative to the design point, flat towards
surge and falling to zero head at 1.5 times the design flow coefficient
(choke). The design point is the wastegate ratio at
`compressor_design_mass_flow_kg_s`, by default 100 m/s through an authored
inducer, else the engine's air flow at 70 % of the redline. Since the
wastegate holds the boost, the map only shows while the turbo spools. The
compressor efficiency stays the authored constant: an efficiency island was
measured further from the makers' torque figures (journal 2026-10-15).

`turbocharger_count` (1 to 4) fits identical turbochargers, each with the
areas, inertia and design flow authored for one. Each exhaust path's turbine
face drives the one its `turbocharger` index names (by default the path's
index modulo the count), and the face's share of that turbine is its share of
that turbocharger's cylinders. The shafts run independently; their
compressors feed one charge and are solved together every 0.25 ms: the common
pressure ratio at which the flows their maps pass at their own speeds add up
to the engine's draw. A turbocharger fed by fewer cylinders spins slower and
passes less of the air, without any special rule.

`intercooler_effectiveness` (0 to 0.95, default 0) takes that share of the
compressor's temperature rise back out of the charge. The 2JZ, EJ25 and I5
author 0.7, as their real engines have one. Before the end gas could knock it
gave them 19-29 % more torque than their makers' figures (journal 2026-10-15);
now the knock control spends that margin and they land 7-13 % over (journal
2026-10-22).

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
constant conductance, its motored term only: the combustion term would bring
the TDI's 171 g/kWh at 2,000 rpm to a real diesel's 212, but it stalls lugged
and launched engines the tests hold (journal 2026-10-23). Intake ducts and every
cell of the exhaust network have a
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
  the engine down to the new speed over `shift_duration_s`. Under drive, the
  ECU retards the spark by up to `dual_clutch_upshift_retard_deg` (default
  20°, taken off the mapped advance within its -10° floor) while the clutch
  pulls the engine down, and gives it back with the same speed-matched
  release as a manual shift's torque cut. The engine, making less torque,
  reaches the new speed sooner; the clutch's torque does not drop where it is
  already at its capacity (the 2JZ at 70-85 % of redline), so the push keeps
  its peak and is only shorter (journal). Production calibrations are not
  published: the 20° is a choice. A downshift is blipped as above. The clutch pedal is not connected: the
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

### Continuously variable transmission

`shift_style: cvt` makes the gearbox a belt variator whose ratio runs anywhere
between the first gear ratio (lowest) and the last (tallest); the selector
has drive, neutral and reverse only (the UI shows D). `VariatorControl` holds
the engine at a speed scheduled from the driver's pedal: max(1.5 × idle,
0.25 × limit) foot-light, 0.85 × the active rev limit flat out, along
pedal^1.5, never under a centrifugal clutch's lock speed. The ratio moves
toward the one that gives that speed at the current road speed, at most the
whole range in 1.5 s (not measured), and goes back to its lowest whenever the
start clutch is not holding. A car's or a scooter's mechanical balance
(hydraulics; roller weights, spring and torque cam) is not modelled, only the
engine speed it holds.

The moving ratio lives inside the clutch model: through a frame the ratio is
interpolated over the 1 ms mechanical steps, and the stick constraint makes
the crank follow the ratio's change (rate × wheel speed) and takes any slip
out over 20 ms, so the clutch stays locked as the ratio moves. Gearboxes with
fixed ratios are bit-for-bit unchanged.

It starts on its own clutch; the pedal does not reach it. A wet start clutch
(default) is the dual clutch's launch clutch: it creeps at idle and stops on
the brake. With `centrifugal_clutch: true` (a scooter's), the clutch is open
at idle, bites at `centrifugal_clutch_engage_rpm` (0: 1.7 × idle) and is shut
at 1.35 × that, a smooth step between. There is no torque converter. The
parts `car_cvt` (2.5-0.4, final 5.0) and `scooter_cvt` (2.6-0.85, final 9.4)
are typical ranges, not a specific unit's. Any gearbox can be switched to a
CVT live from the gearbox inspector; the variator takes the gear's ratio.

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

- two-stroke engines lose about a tenth of their full-load torque at the
  100 mm real-time exhaust mesh against 25 mm cells (see *Two-stroke
  engines*);
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
