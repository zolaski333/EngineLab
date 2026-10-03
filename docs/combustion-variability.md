# Cycle-to-cycle combustion variability

EngineLab can apply a cycle-to-cycle spread to the physical combustion path. It
acts on the turbulence/flame speed of a spark-ignition engine and on the mixing
time of a diesel. Cylinder pressure, indicated work, exhaust flow and finally
the sound therefore all see the same event. It is not noise added in the
renderer.

Two `combustion_calibration` fields drive it:

```yaml
combustion_calibration:
  cycle_variation_cov: 0.04
  cycle_variation_correlation: 0.55
```

- `cycle_variation_cov` is the standard deviation of the combustion speed
  multiplier. The valid range is 0 to 0.20. Zero, the default, is an exact
  bypass that does not even advance the pseudo-random generator.
- `cycle_variation_correlation` is the AR(1) correlation between two
  consecutive cycles of the same cylinder, from 0 to 0.98.

The draw is deterministic, clamped between 0.55 and 1.45, and uses an
independent random stream per cylinder. It does not change the sequence of
real misfires. `CylinderState::combustionCycleMultiplier` publishes the active
value for measurements and exports.

The production engines deliberately stay uncalibrated in the catalogue. The
variant explicitly named `Audio Physics Lab 689 Twin` is a teaching exception:
its COV of 0.06 is an estimated listening setting, declared as such, not a
manufacturer figure. For a calibrated engine, a value must be chosen from a
series of measured cycles (IMEP or cylinder pressure), not to artificially
manufacture an irregular idle. As a starting order of magnitude for an A/B
listening test, 0.02 to 0.05 suits a stable warm engine; higher values must
match a lean, diluted or unstable condition that the simulation also explains
physically.

**AUDIO HQ** publishes the minimum and maximum multipliers seen across the
cylinders live. The **BYPASS** button sets the COV back to zero, which keeps the
bit-exact bypass and allows a comparison without any hidden random draw.

## What the catalogue actually delivers (measured 2 August 2026)

`cycle_variation_cov` is 0 on fifteen of the sixteen engines, and it would be
natural to conclude that their cycles repeat. **That is wrong, and it has been
measured.**

`EngineLabCyclicVariabilityHarness` measures the COV of indicated work per cycle
and per cylinder -- i.e. COV(IMEP) -- each cylinder around its own mean. On the
shipped catalogue, with no authored spread at all:

- part load, held engine speed: **1.7 to 7.0 %**;
- full load, held engine speed: **0.9 to 12.9 %**.

The spread comes from the dynamic coupling of the gases, the wall film, duct
waves and the ECU. The simulator is deterministic -- two runs are identical --
but it is not periodic. Any proposal to add variability must therefore start
from this table, not from the zero field.

Three practical consequences:

1. A physical closure driven by dilution was prototyped then **removed**: at
   this harness's `light` condition, the burned-product fraction at spark is
   0.004 to 0.026, below its 0.06 threshold, so its term was zero there.
   **Note two corrections from 2026-08-02**: this field is not a residual gas
   fraction but a mole fraction of PRODUCTS, i.e. 0.266 × RGF; and at free idle
   it is 0.063 to 0.142, so above the threshold. The claim "zero across the
   whole catalogue" is withdrawn. See `physics-audit.md` in the docs archive
   (git tag `archive/docs-2026-09`), section on the trapped dilution not being
   too low.
2. The order is **reversed** on seven engines, which spread more at full load
   than at part load. It is not the absorber: the `dN%` column shows the engine
   speed held to 0.10-0.48 % while the work varies by 5 to 13 %.
3. A **free idle is not a valid instrument** for this question: its COV(IMEP)
   is dominated by governor hunting (90 % measured on the radial). Hold the
   engine speed, or use `EngineLab.IdleStabilityRegression`, which answers a
   different question.

The harness is deliberately **not** registered as a test: its reference bands
come from the literature and nine engines fall outside them today. Turning it
into a gate now would force widening the bands to the current behaviour, which
would destroy their value.

```
out/build/windows-vs2022/tools/Release/EngineLabCyclicVariabilityHarness.exe
```

`--cov X` forces the authored spread for an A/B within a single session and a
single binary; `--filter NAME` restricts the catalogue; `--enforce` turns the
bands into a gate, once they can be met.
