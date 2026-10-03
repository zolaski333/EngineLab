# EngineLab documentation

Each page describes how one part of EngineLab works today. They are kept up to
date when the code changes; measurement results go into the
[journal](journal.md).

## Project

- [Vision](../VISION.md) — the intent of the project and how success is judged.
- [Measurement journal](journal.md) — dated measurement results, most recent
  first.
- [Recording protocol for the pilot engine](recording-protocol-cp2.md) — how the
  CP2 reference recordings are made.
- [Changelog](../CHANGELOG.md) — user-visible changes per release.

## Models

- [Architecture](architecture.md) — modules, threads, ownership and
  transactional boundaries.
- [Simulation model](simulation-model.md) — kinematics, gas network,
  injection, combustion, knock, forced induction and driveline.
- [Physical thermoacoustic architecture](thermoacoustic-architecture.md) — the
  exhaust sound chain, from the valve to the microphone, with its measurements.
- [Real-time audio](realtime-audio.md) — the callback contract, layers, stems
  and tests.
- [Cycle-to-cycle combustion variability](combustion-variability.md)
- [Physical afterfire](physical-afterfire.md)
- [Structural NVH configuration](structural-nvh-configuration.md)

## Guides

- [Custom exhaust systems](custom-exhaust.md) — the exhaust graph and the
  EXHAUST PRO designer.
- [`.els` engine scripts](engine-scripting.md)
- [ECU tuner and calibration format](ecu-tuning.md)
- [Audio workshop](audio-workshop.md) — the AUDIO HQ window.
- [Declarative audio voicing](audio-voicing.md)
- [Offline high-quality rendering](offline-hq-rendering.md)
- [Audio diagnostic exports](audio-diagnostics.md) — stems and order maps.
- [Blind A/B listening protocol](audio-ab-listening.md)
- [Tests and validation](tests-and-validation.md)

## History

The detailed measurement history up to September 2026 (audits, validation
logs, refuted hypotheses, in French) is kept out of the tree, in the git tag
`archive/docs-2026-09`:

```bash
git ls-tree --name-only archive/docs-2026-09 docs/archive/
git show archive/docs-2026-09:docs/archive/physics-audit.md
```
