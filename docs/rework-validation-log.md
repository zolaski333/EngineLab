# Physics, calibration, audio and performance rework — validation log

This is the durable engineering log for the rework started on 2026-07-26.
It records measurements as well as successful changes. Failed hypotheses are
kept because repeating an attractive but disproved fix wastes more time than
documenting it.

All quoted executable results use the MSVC Release build in
`out/build/windows-vs2022` unless stated otherwise. Thresholds were not relaxed
to make a change pass.

## Initial measured state

- Build: Release succeeded.
- Test suite: 17/19 passed.
  - `EngineLab.IdleStabilityRegression` failed because the Merlin stalled after
    a throttle blip.
  - `EngineLab.AudioRender` failed because the CP2 intake drove the monitor
    safety limiter.
- Yamaha CP2 dyno:
  - peak torque 71.07 Nm at 5,000 rpm;
  - 66.54 Nm at 6,500 rpm;
  - 55.15 kW at 8,500 rpm, 56.64 kW at 9,000 rpm and 57.16 kW at
    9,500 rpm;
  - pumping MEP reached -2.114 bar at 9,500 rpm.
- Yamaha CP2 audio:
  - maximum pre-limiter magnitude 1.313;
  - crest factor 17.78;
  - peak exhaust/intake/structure observer pressures
    40.2/314.4/4.9 Pa.
  - Muting only the intake reduced the pre-limiter peak to 0.320 and the crest
    factor to 5.16. This isolated the defect to the physical intake layer.
- Application-path performance:
  - audio callback p95 consumed approximately 22–57% of its 5.333 ms budget;
  - physics deadline overruns were K20 907/1440, 2JZ 990/1440,
    LS3 613/1440 and Merlin 522/1440;
  - worst physics lateness was 23–34 ms.

The dyno CSV writer also emitted 21 row fields under a 19-column header. Its
schema is now generated from fixed-size arrays and guarded by both a
`static_assert` and the fast `EngineLab.DynoSchema` CTest.

## Idle recovery and fuel adaptation

### Cause

The Merlin did leave deceleration fuel cut, but a single per-cylinder
closed-loop fuel trim was shared by idle and high load. Its 0.50-throttle blip
was rich enough to lower the same trim needed at idle from about 0.80 to about
0.56. The engine then crossed the idle catch with an empty port film and an
under-fuelled command.

Closed-loop adaptation was also allowed to observe cycles where fuel had been
explicitly cut. Such samples do not contain a meaningful commanded-mixture
error and must not train a fuel correction.

### Implemented model

- Closed-loop trim learning is disabled while fuel is cut or the requested
  fuel quantity is effectively zero.
- Low-load and high-load trim cells are independent.
- The command is continuously interpolated between the cells from 0.10 to
  0.25 physical throttle.
- The high-load cell is seeded from the learned low-load value on first
  tip-in. This prevents a discontinuity while preserving independent learning
  after that transition.
- Telemetry exposes requested/delivered fuel, delivery ratio, active trim,
  equivalence ratio, and DFCO latch/ramp state.

### Rejected hypotheses

1. **Raise the DFCO resume threshold by a fixed 300 rpm.** It improved the
   Merlin trace but changed the start-up DFCO sequence of low-idle engines.
   The Big Twin later stalled at steady idle and the radial stalled after the
   blip. The additive threshold was removed and the original proportional
   1.25-times-idle release restored.
2. **Freeze all transient lambda learning.** This also suppresses legitimate
   wetting-film adaptation during start-up and made starting worse. Only
   unobservable zero-fuel cycles are excluded.
3. **Use a fresh high-load trim of 1.0.** The radial's learned idle cell was
   about 0.89; a step to 1.0 over-fuelled its first tip-in before its slow
   crankshaft could accelerate. Seeding the new cell from the current
   low-load cell removed the discontinuity.

### Proof

`EngineLabIdleStabilityRegressionTests` passes all 13 catalogue engines. Every
engine has `caught=1`, `steadyStall=0` and `blipStall=0`. Selected results:

| Engine | target rpm | steady mean | min | max | stddev |
|---|---:|---:|---:|---:|---:|
| Big Twin | 760 | 762 | 726 | 816 | 17.5 |
| Merlin | 800 | 800 | 799 | 801 | 0.4 |
| Radial R5 | 640 | 637 | 545 | 693 | 29.0 |
| Yamaha CP2 | 1,400 | 1,400 | 1,366 | 1,437 | 15.2 |

`EngineLabCoreTests` also passes, including a synthetic 1,000 rpm/s coast test
which proves that fuel is enabled and has useful metering authority at idle.

## CP2 intake acoustic overload

### Diagnostic taps

The intake wave network now reports cumulative absolute pressure maxima at the
valve source, runners, plenum, airbox, mouth and 1 m radiation output. The
audio renderer transfers these measurements to atomics once per block; no
allocation or atomic operation was added to the per-sample network loop.

The failing CP2 render measured:

| Stage | peak pressure |
|---|---:|
| valve source | 26.15 kPa |
| runner waves | 37.50 kPa |
| plenum | 21.95 kPa |
| airbox | 18.93 kPa |
| mouth incident wave | 199.66 kPa |
| radiation at 1 m | 1.207 kPa |
| configured microphones | 324.7 Pa |

The source and cavity were not the overload. Energy accumulated in the
near-lossless linear open-end reflection until the mouth wave left the linear
acoustic validity range.

### Implemented model

The existing Padé radiation impedance remains unchanged at infinitesimal
amplitude. The intake mouth additionally enables the quasi-steady nonlinear
open-end resistance

`Z_nl / Z_c = 2 C_d / (3 pi) * |u| / c`,

with `C_d = 2` for the thin-wall unflanged termination represented by the
class. It is implemented as a positive series resistance in wave variables,
including an exact solution of the radiation IIR's direct feed-through. It is
not a sample clamp, limiter, output gain, or catalogue-specific coefficient.
With coefficient zero the old linear transfer is reproduced exactly.

Primary references:

- M. Atig et al., “On the nonlinear behavior of the open end of a tube,”
  *Comptes Rendus Mécanique* 332 (2004), 299–304,
  <https://doi.org/10.1016/j.crme.2004.02.008>.
- M. C. A. M. Peters et al., “Damping and reflection coefficient measurements
  for an open pipe at low Mach and low Helmholtz numbers,” *Journal of Fluid
  Mechanics* 256 (1993), 499–534,
  <https://doi.org/10.1017/S0022112093002861>.

### Proof

For the same CP2 simulation and unchanged valve-flow source:

| Metric | before | after |
|---|---:|---:|
| mouth pressure | 199.7 kPa | 17.8 kPa |
| microphone intake pressure | 324.7 Pa | 26.7 Pa |
| maximum pre-limiter magnitude | 1.334 | 0.218 |
| crest factor | 14.12 | 5.47 |
| limiter samples | non-zero failure condition | 0 |

The complete audio harness passes, including the 13-engine catalogue, long-run
stability, forced induction, true idle/rev/return, spectral differentiation
and physical decay checks. `EngineLabRealtimeRegressionTests` passes a
40 kPa sinusoidal stress case which proves that the nonlinear element remains
finite, removes reflected energy, and never returns more energy than arrives.

## Build-system observation

An unrestricted parallel Release build exhausted the MSVC compiler heap
(`C1060`) while compiling three very large LTO translation units at once.
Those exact targets (`EngineLabCoreTests`, `EngineLabAbClipRenderer`,
`EngineLabAudioAbHarness`) all compile and link with `--parallel 1`. This is a
build-memory scheduling limit, not a source error. Final build documentation
must recommend a bounded job count on ordinary machines.

## Open observations for subsequent phases

- CP2 torque magnitude is close to the intended real-engine envelope, but the
  simulated peak occurs roughly 1,500 rpm too early and high-rpm pumping loss
  is excessive.
- The audio callback itself meets its deadline, but physics production misses
  a large fraction of frame deadlines. Optimisation claims must be based on
  isolated, repeated before/after runs rather than the contended validation
  executions above.

## Radial bank topology and chamber flame calibration

### Radial root cause

Catalogue loading synthesized one zero-degree bank for every layout other than
V and flat engines. `MechanicalKinematics::bankAngleFor()` correctly treats an
explicit bank as authoritative, so the generated bank erased the radial
cylinders' authored 0/72/144/216/288-degree axes. All five pistons consequently
moved in phase although their valve and ignition schedules remained staggered
over 720 degrees. This explains both the implausible 17 Nm loaded result and the
loaded audio scenario that stopped: most cylinders exchanged gas on the wrong
physical stroke.

The catalogue now synthesizes one bank per radial cylinder, preserving the
authored spatial axis. Bank validation accepts both the signed convention used
by V/flat engines and the natural [0, 360] radial convention. A catalogue
regression checks every radial cylinder-to-bank mapping.

### In-cylinder model

Two physical head properties are now explicit combustion calibration inputs:

- chamber turbulence intensity relative to the mean-piston-speed closure;
- the number of independent ignition kernels.

Turbulence changes turbulent flame speed, while multiple ignition sites change
initial burned-kernel volume; they are deliberately not represented as fuel
energy, arbitrary torque multipliers, or extra spark advance. The shipped CP3
and CP4 heads use progressively stronger tumble. The large-bore radial uses its
realistic dual-plug topology. JSON/YAML round trips and range validation cover
both fields.

### Proof

The same radial catalogue sweep at 2,000 rpm changed from 16.99 Nm / 3.56 kW /
0.265 VE to 572.39 Nm / 119.86 kW / 0.909 VE. Its five resolved geometric TDC
angles are now 0.00, 73.18, 145.99, 214.01 and 286.82 degrees, and all five
cylinders trap useful fresh charge (1.36--1.67 g in the inspected cycle).
This is evidence of repaired mechanics and gas exchange, rather than a
catalogue torque correction.

At unchanged displacement, fuel and ignition tables, chamber calibration moved
the catalogue curves as follows:

| Engine | baseline peak torque | calibrated peak torque | baseline peak power | calibrated peak power |
|---|---:|---:|---:|---:|
| Yamaha CP3-like | 84.26 Nm | 86.98 Nm | 75.54 kW | 78.49 kW |
| Yamaha CP4-like | 84.37 Nm | 93.60 Nm | 77.21 kW | 87.32 kW |

The CP4 pressure trace changes consistently with faster combustion: burn
completion moves from about 60 to 29 crank degrees and peak pressure from about
48 bar at 10 degrees to 74.5 bar at 19.6 degrees. `EngineLabCoreTests` passes,
including monotonic tests for increased turbulence, two independent kernels,
serialization, validation, and radial catalogue topology.

## Variable-area quasi-1D ducts

### Implementation

Exhaust components now accept an optional `outlet_diameter_mm`; intake paths
accept `runner_plenum_diameter_mm`. Zero is backward-compatible constant area.
A non-zero second diameter defines a circular conical frustum (linear radius):
the exact frustum volume is used by inventory and topology compilation.

The finite-volume residual now uses the local area of every face and the exact
volume of every cell. Species, mass and energy remain conservative, while
momentum receives the quasi-1D pressure-wall source `p (A_R - A_L) / V`.
Friction, wall heat transfer, wall thermal capacity and source stability use
the local hydraulic diameter/volume. Network boundaries use the correct end
area instead of a mean area. The acoustic networks likewise scatter with the
area at the relevant endpoint.

During review, the port-fuel source exposed a genuine secondary bug: its
three-cell distribution divided every share by cell zero's volume. That is
correct only for a cylindrical mesh. It now divides each equal mass/energy
share by that cell's own volume, and the conservation test deliberately injects
into a tapered runner.

### Proof

- A stationary 4:1 taper remains at uniform pressure with relative density and
  energy errors below `3e-13` and momentum below `2e-10 kg/(m2 s)`.
- A moving 4:1 contraction closes every species inventory and total energy
  against its unequal boundary areas to `3e-10` relative.
- The exact conical volume is checked independently in the duct, exhaust-layout
  and intake-runner tests.
- The tapered port injection adds its requested fuel mass exactly once
  (`<1e-15 kg` absolute error) and closes sensible plus latent energy.
- The unchanged steady-runner benchmark still delivers `0.0352669 kg/s`
  against an independent isentropic `0.0366292 kg/s` reference (ratio
  `0.962808`), showing that backward-compatible cylindrical geometry did not
  acquire an artificial restriction.
- `EngineLabGasDynamicsTests`, `EngineLabCoreTests` and
  `EngineLabRealtimeRegressionTests` all pass. The desktop application also
  compiles with the new two-diameter editor.

## Compression ignition, flame ceiling and solver caches (`790f772`)

This commit shipped with an empty message body and no entry here. It is the
largest model change of the series, so it is recorded now, after the fact, from
its diff and from re-measurement on the committed state.

### What it contains

- `CompressionIgnitionModel`: Livengood-Wu induction integral over an Assanis
  pressure/temperature/equivalence-ratio delay correlation with an explicit
  cetane correction, then a rapid premixed fraction followed by a
  mixing-controlled diffusion burn. It consumes fuel and oxygen only through
  `ConservativeGasSystem`, so it keeps the same chemical invariants as spark
  combustion.
- Diesel fuelling: metered by an injected-quantity (smoke) map with the
  closed-loop trim explicitly disabled, and `normalizedLoad` taken from
  throttle/load rather than manifold pressure, because a quality-governed engine
  has no throttle-derived load.
- `engines/14_vw_2_0_tdi_like.engine.yaml` plus `road_diesel_en590`,
  `common_rail_diesel` and `diesel_low_speed` parts.
- **Flame speed ceiling.** The fixed 42 m/s clamp in
  `FlamePhysicsModel::turbulentFlameSpeedMps` clipped every high-speed pent-roof
  chamber to the same burn rate regardless of its authored tumble. It is now
  `0.18 * a(T_unburned)`, a deflagration Mach bound. Note for anyone
  investigating idle: at idle the laminar-plus-turbulent sum is around 5 m/s, so
  neither the old bound nor the new one is active there. This change cannot
  affect an idle, and that was verified before looking elsewhere.
- **Solver caches.** Immutable duct geometry (roots, powers, cell volumes,
  hydraulic diameters, roughness terms) was being recomputed per cell, per RK2
  stage, per substep; it is now prepared once at configuration. Wall heat
  transfer became transactional: the gas state and the wall state are validated
  together and committed only if the whole substep is physical.

### Measured effect

Curves, on the committed state, against manufacturer figures:

| Engine | sim torque | real | sim power | real |
|---|---:|---:|---:|---:|
| Yamaha CP2 | 65.9 Nm @6500 | 68 @6500 | 57.2 kW @9500 | 54 @8750 |
| Yamaha CP3 | 96.1 Nm @7000 | 93 @7000 | 89.3 kW @9000 | 87.5 @10000 |
| Yamaha CP4 | 111.9 Nm @9000 | 111 @9000 | 118.2 kW @10500 | 118 @11500 |
| VW 2.0 TDI | 341.6 Nm @2000 | 340 @1750-3000 | 111.2 kW @4000 | 110 @3500-4000 |

The CP4 was at 65% of rated power with its torque peak 3250 rpm early before
this pass. Two physical corrections produced that: the intake runner length had
counted only the visible bellmouth and omitted the head port, and the flame
ceiling above. Performance, median of repeated runs against `e624b9e`: CP2
-11.3/-7.5/-5.9%, LS3 -14.8/-18.9/-17.4%.

### Rejected during that pass, and worth not repeating

- A detailed 4-2-1 collector graph for the CP4. It raised back pressure from 166
  to over 210 kPa, nearly doubled the CP4's step cost, and degraded the curve.
  The equivalent collector measures better.
- Parallelising four cylinders. Eighty synchronisations per frame cost more than
  the four tasks return.
- One float re-association inside the fuel-limit precomputation. It was small
  enough to look harmless and was amplified by autoignition; reverting it
  restored the bench bit for bit.

## Idle robustness: a fragile attractor, not a regression (2026-07-26)

### Symptom

After `790f772`, `EngineLab.IdleStabilityRegression` failed on the Radial R5
(complete stall, 0 rpm, steady and after a blip) and the Big Twin (mean 753, min
576, sigma 57.7, drift 40.9 against a 760 target). Both passed at `e624b9e`.

### Cause

Not a model defect in `790f772`. Tracing both engines at `e624b9e` and at HEAD
and diffing line by line, the two runs first differ at t = 0.40 s (Big Twin,
`air_mg` 636.4 vs 636.5) and t = 0.50 s (radial, `air_mg` 3782.8 vs 3782.9) --
the last printed digit. That is the solver-cache work of `790f772`: algebraically
equivalent, not bit-identical. Everything after is amplification.

An idle that a one-part-in-a-million air-mass difference flips between settling
at 681 rpm and stalling is not a shippable idle, and it means every previous
green run on those engines was luck. `CLAUDE.md` already names this signature.
So the fix had to be robustness, not a hunt for the ULP.

The destructive mechanism, from the radial trace: the after-start air floor is
0.88 with no reference to the engine's idle target, so the radial flared to 1433
rpm against a 640 target. That crossed the deceleration-fuel-cut entry threshold
(1.65 x idle) 0.3 s after catch, all fuel was cut, cycle torque went from +209 to
-70 Nm, the engine fell back below the catch threshold, the starter re-engaged
and re-charged the floor to 0.88, and it repeated. While the floor owns the
actuator the idle anti-windup deliberately forbids the governor from integrating
down, so nothing could oppose it either.

### Rejected hypothesis

**Release the after-start floor in proportion to overspeed.** It fixes the two
worst engines (radial 0 -> 683 rpm, Big Twin sigma 57.7 -> 16.7) and is
physically defensible, but the floor is exactly what keeps a flaring engine
breathing: at 6.0/s it took the Audi I5 from a settled 780/746 rpm (sigma 9.3) to
a full stall. The flare is not the destructive event. Reverted, and the reason is
recorded in the code so it is not proposed again.

### Implemented

Deceleration fuel cut is an overrun function: it presumes a running, warmed
engine coasting down under a shut throttle. The after-start flare satisfies its
speed threshold while being the opposite condition -- the engine is accelerating
away from a catch with an empty port film. Production ECUs inhibit overrun cut
through the after-start phase for exactly this reason. The after-start air
schedule already *is* that phase, is already longer on a cold engine (which is
when a real inhibit lasts longest), and is already maintained upstream, so the
inhibit gates on it rather than introducing a second timer.

### Proof

All 14 catalogue engines pass, and every one is equal or better than its best
previously recorded state:

| Engine | at `e624b9e` (last all-green) | with the inhibit |
|---|---|---|
| Radial R5 | 681, sigma 14.5 | **684, sigma 5.0** |
| Audi I5 | 780, sigma 9.3 | **780, sigma 5.1** |
| Big Twin | 761, sigma 17.9 | **763, sigma 14.2** |
| Yamaha CP3 | 1303, sigma 16.1 | **1301, sigma 14.9** |

Non-vacuity: with the inhibit removed the radial stalls outright and the Big
Twin fails three assertions, which is the failure this fixes.

## The catalogue AFR gate could not read a diesel

`EngineLab.CatalogPhysics` failed only on the 2.0 TDI: mean AFR error 5.41
against a 2.5 ceiling, from AFR 22.41 measured versus a 16.99 "target".

The physics is right and the gate was wrong. A quality-governed engine has no
stoichiometric setpoint. What `EngineSimulator` publishes as a diesel's
`targetAirFuelRatio` is a smoke-limit **floor** (stoichiometric * 1.16), and its
fuel is metered by the injected-quantity map with the closed-loop trim disabled,
so whichever binds first the delivered mixture is normally *leaner* than the
floor. AFR 22.4 at the rated point is textbook, and it is the same calibration
that reproduces 340 Nm / 110 kW.

The gate now scores compression ignition one-sided: only running *richer* than
the smoke limit counts as an error. That is strictly tighter than the old
two-sided band on the rich side, which is where the real failure -- sooting past
the smoke limit -- lives. The tolerance was not widened.

## 2026-07-27 — Le son des gros moteurs était un problème de thread physique

Entrée courte : le détail complet, les tableaux et les hypothèses réfutées sont
dans `docs/physics-audit.md`, section « Le simulateur tournait au ralenti, et le
son en découlait ».

Point de départ : plainte utilisateur sur l'audio (« plus il y a de cylindres,
moins ça va », Merlin V12 et LS3 V8 quasi muets, décalage entrées/effet), avec la
consigne explicite de ne pas corriger la génération du son avant d'avoir examiné
tout ce qui la précède. Aucun des défauts trouvés n'est dans la chaîne audio.

- **Nouvel instrument** `EngineLabRealtimeBudgetHarness` : facteur temps réel
  (secondes simulées produites par seconde murale) mesuré sur le vrai thread
  `EngineRuntime`. Le LS3 produisait 0.266, le Merlin 0.286. La boucle avance un
  pas fixe de 1/240 s sans accumulateur et abandonne sa dette au-delà de quatre
  pas de retard : la surcharge ne saute pas une trame, elle met tout au ralenti.
- **Fork-join par cylindre retiré** : net loss, quatre variantes mesurées, toutes
  perdantes. LS3 0.266 -> 0.343, Merlin 0.286 -> 0.396, **bit-identique**.
  Une hypothèse intermédiaire (réveil `notify_all` des non-participants) a été
  implémentée, mesurée, et **réfutée** : elle dégradait le résultat.
- **Profil de coût remplacé** : l'admission 1-D vaut 75-84 % du sous-pas, la
  physique cylindre 5-10 %. L'ancien 41/29/30 de `CLAUDE.md` est obsolète.
- **Deux fausses alarmes GUI corrigées** : contre-pression (un pic lu comme une
  moyenne ; nouveau champ `exhaustBackPressureKpa`) et adhérence (OU collant sur
  les sous-pas). Ce qui alerte encore est vrai et pointe le pompage excessif déjà
  documenté comme non résolu.
- **Moto sans réduction primaire** : 7.21 en première au lieu de 14.73, donc
  couple à la roue divisé par deux dans tous les rapports. Corrigé, contre-vérifié
  sur la vitesse de pointe.

Suite complète verte (19 tests ; le test du fork-join disparaît avec lui).

**Non résolu et assumé** : après retrait du fork-join le facteur temps réel reste
à 0.33-0.43 sur tous les moteurs de 4 cylindres et plus. Le fork-join ne valait
qu'un tiers du déficit du V8. Tant que le coût de l'admission 1-D n'est pas
réduit, le son des gros moteurs restera affamé, et aucune correction de la chaîne
audio ne peut compenser ça.

## 2026-07-27 — Passer le temps réel : l'admission 1-D en concurrence

- **L'instrument saturait.** Le facteur temps réel se lit sur une boucle qui dort
  jusqu'à son échéance : il plafonne à 1,0 et ne peut pas distinguer 3× de marge
  d'un équilibre exact. `--free-run` retire ce sommeil et la même mesure devient
  une capacité. Sur cette échelle le départ était LS3 0,473 et Merlin 0,401.
- **Avance des runners d'admission mise en concurrence.** Ce n'est pas un retour
  sur le retrait du fork-join : celui-ci distribuait `processCylinder` (~0,6 µs
  par cylindre), celle-ci distribue l'avance 1-D (~5 µs). Neuf moteurs sur
  quatorze passent maintenant le temps réel, contre quatre.
- **Le plénum partagé était la vraie difficulté, pas le threading.** Quatre
  schémas mesurés et rejetés contre l'invariant « un moteur à l'arrêt se
  stabilise à l'ambiant » : Jacobi −0,62 kPa (mauvais point fixe, pas relaxation
  lente), escalier retardé ±0,17 kPa (cycle limite), point milieu −0,095 kPa,
  escalier prédit une étape −0,011 kPa. Retenu : escalier prédit à l'instant
  courant, prédicteur Heun, deux tours de point fixe, +0,0011 kPa.
- **Interblocage fermé dans le pool, et il a coûté cher à trouver.** La barrière
  comptait les *items* restants ; elle compte maintenant les *workers* ayant
  acquitté la génération. Un worker qui a observé une génération mais n'est pas
  encore entré dans la file n'est pas observable par le maître : celui-ci peut
  finir le job, publier le suivant, et ce worker entre alors dans la file du
  *nouveau* job, décrémentant un compteur que le maître va réécrire. Le compte
  reste définitivement trop haut et le maître tourne indéfiniment.
  `EngineLab.CombustionPhasing` est resté bloqué **8 h 52** sur un test de
  7,81 s. Aucune garde du type « un worker est sur le point d'entrer » ne peut
  fermer ça — la fenêtre est entre deux instructions du worker lui-même.
  Signature à reconnaître : **un seul cœur à 100 %, tous les autres garés**,
  donc temps CPU du processus ≈ temps mural quel que soit le nombre de threads.
  L'acquittement par génération supprime aussi la course sur `body_`/`context_`
  identifiée plus tôt (un appel via le mauvais type), et ne coûte rien : la
  suppression du décrément atomique par item paie l'arête supplémentaire
  (LS3 0,73 → 0,76, 2JZ 0,88 → 0,91).
- **Un `ctest` en arrière-plan qui a imprimé `Start 10:` et plus rien est
  bloqué, pas lent.** Le vérifier avec `Get-Process ... | Select CPU,StartTime`,
  pas en attendant.
- **Trois impasses documentées** : maillage runner plus grossier (×1,26 sur le
  V8 mais −22,3 % de couple sur le Merlin), `/arch:AVX` et `/arch:AVX2` (tous
  deux **plus lents**, le solveur est limité par la latence de chaînes de
  divisions), suppression des divisions inutilisées de `recoverPrimitive` (bruit).
- **Piège d'analyse** : `EngineLab.Core` met ~2 minutes quand il PASSE et ~8
  secondes quand il échoue — un `require` interrompt le test. Une durée courte
  dans un relevé ctest n'est donc pas une bonne nouvelle.

**Non résolu et assumé** : le V8, le V12, les six-cylindres, l'I5 et le K20A
restent sous le temps réel. Le levier suivant est le modèle thermique de paroi
du conduit, avec une constante de temps en secondes intégrée ~38 000 fois par
seconde.

## 2026-07-27 (suite) — Sous-cadencer l'échange de paroi du conduit

- **L'attribution « parois 28 % » était trompeuse.** Mesurée au banc bit-exact,
  par cellule et par sous-étape : chaîne du coefficient (Sutherland, Reynolds,
  Nusselt de Gnielinski) **6,2 %**, échange lui-même (`expm1`/`exp`, ~8
  divisions) **11,0 %**, et le reste (**~10,8 %**) est une passe complète de
  `recoverPrimitiveStates` qui n'existe que parce que l'étape de paroi modifie
  l'énergie des cellules entre la récupération et `prepareStateCache`.
  Sous-cadencer l'échange rapporte donc plus que l'arithmétique qu'il évite,
  puisqu'il évite aussi cette passe.
- **`std::pow(x, 2.0)` est bien un vrai appel libm ici**, dans le facteur de
  frottement : le remplacer par `x*x` vaut 5 % du solveur entier et les six
  configurations du banc rendent un checksum **inchangé**. Le commentaire du
  source qui affirmait le contraire est corrigé.
- **Sous-cadençage retenu : 150 µs.** Ce n'est pas la constante de temps
  thermique (~69 ms) qui fixe le plancher, c'est l'échantillonnage du
  coefficient, qui suit l'écoulement : le temps de séjour d'une cellule L/u vaut
  ~600 µs à 50 m/s dans une maille de 30 mm. 150 µs garde quatre échantillons
  dans le séjour le plus rapide et une trentaine sur une levée d'admission à
  7 000 tr/min. Coût du solveur de conduit : 530 → 402 ns/cellule/sous-étape,
  **−24 %**.
- **Un sous-cadençage auto-déclenché se désynchronise derrière une barrière.**
  Chaque réseau décidant à partir de son propre historique CFL, le pic de paroi
  tombe sur une répartition différente pour chaque cylindre, et une barrière
  coûte le *maximum*, pas la moyenne. La cadence est donc pilotée par
  `EngineSimulator` (`requestWallHeatUpdate`), déclenchée sur le **second**
  demi-pas — le seul où tous les cylindres sont distribués. Vaut ~3 % de plus
  sur le V8.
- **Piège de mesure, et il a failli faire annuler le changement.** Le facteur
  temps réel du même commit `c03b6d3` a lu LS3 0,762 le matin et 0,618 deux
  heures plus tard, après une série de builds et de suites de tests : 20 % de
  dérive machine, dans un seul sens, sur tous les moteurs. Comparé à la table du
  matin, ce travail ressemblait à une perte de 13 %. **Seule une mesure
  entrelacée est valable** : les deux binaires côte à côte, alternés. Ainsi
  mesuré, l'« après » gagne dans 14 paires sur 15 — CP2 +35 %, LS3 +21 %,
  Merlin +9 %, K20A +9 %, Audi I5 +6 %.
- **Physique déplacée, balayage complet, lignes contaminées exclues** : couple
  entre −0,17 % et +0,42 % de moyenne par moteur (pire point +2,62 %), VE à
  ±0,06 % (pire −0,94 %), et **EGT — la grandeur que ce modèle gouverne
  directement — entre +0,01 % et +0,14 % de moyenne, pire point +1,04 %**.
  Suite complète verte, `OverrunThermalRegression` et `IdleStabilityRegression`
  comprises.

**Non résolu et assumé** : les valeurs absolues de capacité de cette session ne
valent rien, la machine ayant dérivé de 20 % pendant les mesures. Seuls les
rapports entrelacés ci-dessus sont fiables. Une table absolue devra être reprise
sur une machine reposée avant toute affirmation du type « N moteurs sur 14
passent le temps réel ».

## 2026-07-28 — Reprise sur 6 cœurs / 12 threads

La reprise complète, les tables et les commandes reproductibles sont consignées
dans [`validation-2026-07-28.md`](validation-2026-07-28.md).

- le catalogue complet passe le plancher 1,10× temps réel à 90 % du régime
  limite (pire LS3 1,102 ; Merlin 1,105) ;
- l'admission réduite reste à 13,435 % de l'oracle sur le cas runner 2× et à
  4,022 % sur le moteur standard ;
- le plafond 5 workers est retenu après A/B contrebalancé, chemin applicatif et
  témoin CP2 ;
- le couplage échappement 125 µs restaure une Nyquist physique de 3,48 à
  5,64 kHz sur les deux cas lourds mesurés ;
- 14/14 points constructeur, 14/14 ralentis, 20/20 tests et le smoke test
  Release passent.

**Non résolu et assumé** : PMEP haut régime 1,480 bar à 6 000 tr/min pour une
cible 0,750 ; références constructeur encore absentes pour les moteurs
génériques/scalés ; marge de capacité brute d'environ 10 %, pas 15 %, sur les
deux cas les plus lourds.

## 2026-07-28 — Correction physique du collecteur d'échappement

- Les ablations à 6 000 tr/min ont réfuté le maillage fin, le silencieux, la
  sortie atmosphérique et le volume de collecteur comme causes racines.
- Une remise du réseau à l'ambiante a séparé deux contributions : mémoire de
  pression aval et capacité soupape/port.
- La cause aval confirmée était l'annulation forcée de la quantité de mouvement
  dans chaque jonction 0-D. La correction conserve le résidu axial après
  réaction de pression statique des parois.
- A/B I4 : PMEP 1,475 → 1,046 bar ; pression échappement moyenne
  169,230 → 111,953 kPa.
- Banc stationnaire K20 : vitesse de collecteur 0 → 128,851 m/s, débit
  0,139 → 0,295 kg/s, `K/K_géométrique` 5,549 → 1,494, sans écart de bilan
  masse/énergie.
- Le contrôle historique est conservé avec `--well-mixed-junctions`.
- Après réancrage local de la turbulence du CP2, les points constructeur passent
  14/14. Le résidu PMEP haut régime reste honnêtement ouvert : oracle
  0,990 bar à 6 000 tr/min pour une cible de 0,750.
- Le rendu A/B LS3 atteint une similarité spectrale de 0,937184 entre collecteur
  dirigé et contrôle mélangé, avec zéro fallback, dropout, pression perdue ou
  limiteur. Les deux WAV sont conservés pour l'écoute humaine.
- Le transitoire Big Twin démarrage–ralenti–coup de gaz–retour passe également :
  1 337 → 4 036 → 780 tr/min, sans perte ni limiteur.

## 2026-07-28 — Protection de charge

- Admission 500 µs refusée : aucune marge CPU LS3 mesurable.
- Reconstruction de plénum à un tour refusée : meilleur facteur identique
  (1,030×) sur six essais par variante, CP2 utilisé comme témoin nul.
- Garde retenue : cadence thermique des parois d'admission 150 → 600 µs après
  six échéances consécutives manquées ; retour après 480 trames confortables.
- Jamais active pendant un dyno, une pause ou un banc `--free-run`.
- Physique protégée : écart oracle maximal 14,158 %, déplacement direct de VE
  maximal 7,69 %, quatre critères d'accord d'admission verts.
- Chemin applicatif LS3 : 607 → 582 retards sur 1 440, callback p95
  2 986,5 → 2 963,9 µs, zéro pression perdue/fallback. L'adaptatif s'active une
  fois et termine à 597 retards, comme attendu après sa fenêtre de confirmation.

## 2026-07-28 — Fermeture admission, pool et son

### Maillage admission retenu

Le maillage ciblé à 75 mm n'était pas le dernier point sûr. Une comparaison
contrebalancée 75/95 mm, six valeurs par variante et un CP2 sans workers comme
témoin nul, a donné :

| Cas | 75 mm, moyenne | 95 mm, moyenne | Écart |
|---|---:|---:|---:|
| CP2 témoin nul | 2,5467 | 2,5580 | +0,4 % |
| LS3 | 0,9910 | 1,0498 | +5,9 % |
| Merlin | 0,9920 | 1,0818 | +9,1 % |

La production passe donc à **95 mm**. Contre l'oracle 30 mm/RK2/couplage à
chaque sous-pas, l'écart VE maximal n'est plus que 4,460 % ; pic VE 1,127 à
5 840 tr/min, VE à 6 000 tr/min 1,120 et déplacement du pic avec runner doublé
26,478 %. Le mode thermique protégé à 600 µs reste à 4,916 %. Le candidat
120 mm a été construit et refusé : 16,580 % d'écart sur le runner doublé à
3 000 tr/min, au-delà de la limite de 15 %.

### Pool adapté à six cœurs

Après le changement de maillage, les plafonds 2/3/4 workers ont été remesurés
sur LS3 et Merlin, avec six passages contrebalancés et le Big Twin comme témoin
zéro-worker :

| Cas | 2 workers | 3 workers | 4 workers |
|---|---:|---:|---:|
| LS3, moyenne | **1,1160** | 1,1045 | 1,0805 |
| Merlin, moyenne | 1,1095 | **1,1222** | 1,1032 |
| Big Twin, moyenne | 3,5168 | 3,5395 | 3,5238 |

La politique automatique est donc 2 workers pour 3 à 9 cylindres et 3 workers
à partir de 10 cylindres, toujours bornée par le matériel et `cylindres - 1`.
Les overrides explicites de banc restent prioritaires. Elle évite de
souscrire quatre ou cinq threads de calcul admission sur une machine qui doit
également servir le thread physique, l'audio et l'interface.

Le catalogue production de ce lot est entièrement au-dessus du temps réel :
pire facteur **1,113×** sur le Merlin, puis 1,121× sur le LS3, sans overrun en
mode capacité. Après le dernier correctif de film, une tâche Windows
`CompatTelRunner` a contaminé la répétition pleine (Merlin 1,082×), mais le lot
alterné final trouve un meilleur temps à **1,147×** tandis que le CP2 témoin
zéro-worker s'effondre de 2,574× à 1,894×. Selon le protocole imposé, la
capacité intrinsèque observée est donc 1,147× ; 1,082× documente honnêtement le
cas où d'autres processus occupent déjà la machine.

### Validation audio finale

Le rendu complet conserve le réseau physique, la topologie complète, la
structure modale et l'admission ondulatoire sur les 14 moteurs. Aucun moteur ne
produit de fallback, dropout de frontière, pression perdue, événement tardif
ou échantillon limité. Le maximum de similarité spectrale tombe à **0,642**.
Dans le chemin applicatif final exécuté sous cette charge externe, le p95
callback vaut 1 837,3 µs sur K20, 2 268,2 µs sur 2JZ, 2 887,5 µs sur LS3 et
3 662,0 µs sur Merlin pour un budget de 5 333,3 µs. Le garde-fou s'active sur
K20, LS3 et Merlin sans décimer la télémétrie acoustique.

## 2026-07-28 — Reprise DFCO : bilan du film neuf

La suite Release finale a révélé un dernier défaut réel sur le Merlin après un
coup de gaz. Relever le seuil de reprise DFCO jusqu'à 1,60 fois le ralenti et
accélérer la rampe ne le corrigeait pas. Même une reprise à 1,50 fois, donc plus
précoce que la production d'origine, descendait à 349 tr/min.

La cause était dans l'interface contrôleur/injecteur. `requestedFuelMoles`
désigne la masse nécessaire dans la charge piégée, tandis que l'injecteur
commande un liquide dont la fraction `X` mouille le port. Le code soustrayait
bien le film déjà présent et disponible avant l'étincelle, mais commandait le
déficit brut pour le pulse neuf. Après DFCO, film vide, cela imposait une
première charge pauvre de la fraction humide indisponible.

La commande port tient maintenant compte de sa disponibilité physique :

```text
disponible = (1 - X) + X * fraction_du_film_évaporée_avant_étincelle
masse_liquide_commandée = déficit_de_charge / disponible
```

Le seuil proportionnel 1,25 et la rampe 3/s d'origine sont conservés. Avec ce
réglage moins favorable en temps que l'expérience 1,50, le même trace Merlin
reste encore à 479 tr/min au point où l'expérience non compensée était déjà à
349 tr/min, au-dessus du plancher de test de 360 tr/min. `EngineLab.Core`,
`EngineLab.IdleStabilityRegression` sur les 14 moteurs et
`EngineLab.CatalogReference` passent ensemble ; aucune calibration Merlin n'a
été ajoutée.

La fermeture complète donne **20/20 tests en 514,43 s**, un rendu audio final
séparé vert sous charge, puis un lancement caché de `EngineLab.exe` resté vivant
cinq secondes. Les hash et journaux sont consignés dans
`docs/validation-2026-07-28.md`.

## 2026-07-29 — Réaudit et correction de la couche turbo

Le balayage Release actuel réfute l'ancien plancher aigu EJ25 : énergie au-dessus
de 4 kHz de 1,2 % au point haut et 0,4 % au point précédent, contre 20,9 % dans
un ancien rendu. Aucun assombrissement opportuniste n'a donc été appliqué.

Un test analytique ajouté avant le correctif a en revanche échoué :
`equal compressor and turbine broadband sources must not cancel`. Deux sources
égales s'annulaient exactement parce qu'elles filtraient la même suite de bruit
avec des signes opposés. Deux autres défauts ont été corrigés dans le même
contrat physique : marches de puissance/vitesse à 240 Hz, et débit total compté
dans la turbine puis une seconde fois dans la wastegate.

Après correction :

- bruit déterministe indépendant par compresseur, turbine, wastegate et dump ;
- interpolation temporelle 5 ms, avec dump 0,75 ms attaque / 12 ms relâchement ;
- partage conservatif par aires : avec 700/350 mm² et wastegate ouverte,
  0,12 kg/s devient 0,08 + 0,04 kg/s ;
- `EngineLab.Core` et `EngineLab.RealtimeRegression` : 2/2 en 51,77 s ;
- rendu court EJ25 : 0 perte/fallback/limiteur, pic 0,3737 contre 0,3751 avant ;
- A/B complet : différence RMS alignée 5,98 % sur 2JZ, 6,28 % sur EJ25 ;
  les trois moteurs atmosphériques témoins sont bit-identiques ;
- part >4 kHz quasi inchangée : 2JZ 0,701 -> 0,703 %, EJ25
  0,326 -> 0,327 %.

Les artefacts de source sont retirés sans élargissement artificiel du spectre ni
retouche des calibrations moteur.
