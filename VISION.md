# EngineLab — Vision

This document sets the intent of the project. It rarely changes, and only by
decision of the owner. Everything else (code, docs, priorities) must conform to
it.

## What EngineLab must be

A four-stroke engine simulator **whose sound is as close to reality as
possible**, for any engine the user builds.

1. **Sound first.** Someone who knows the real engine should recognise it by
   ear: not just "a parallel twin", but "a CP2". The method is open: physics,
   synthesis, or both. What counts is the audible result, not the elegance of
   the model.
2. **Physics that is broadly right.** Plausible torque and power (within about
   ±15 % of the manufacturer figures), credible reactions to throttle, gear and
   exhaust. Physics serves the sound and the feel; it is not an end in itself.
3. **Real time on a mid-range PC.** Reference machine: Intel i5-10600
   (6 cores / 12 threads), 16 GB of RAM.

## Fundamental constraint: no per-engine recordings

An engine built by the user (a W16, for example) must sound realistic **from
its description alone**: geometry, firing order, intake, exhaust. The user must
never have to supply a recording of their engine.

- Real recordings are used to **calibrate the generic model** during
  development. They are never a required ingredient of the render.
- **Generic** sound elements, not specific to one engine, are allowed in the
  render: a mechanical noise texture per family, the typical crack of a
  backfire, the response of a listening environment, and so on.

## Success criterion

The sound is judged **against real recordings at a known engine speed**, never
against the simulator's own output.

- **Objective:** timbre distance between simulation and reality at the same
  engine speed and load, measured by a dedicated tool.
- **Subjective:** blind listening.

Milestones:

1. **Pilot engine: Yamaha CP2 (MT-07), Arrow exhaust.** Recordings made by the
   owner on their own motorcycle, at a known engine speed.
2. **Generalisation:** a second, very different engine must improve **without
   specific tuning**. That is the proof that calibrating on the CP2 improved
   the generic model rather than just imitating one motorcycle.

## Non-goals

- A 1:1 reproduction of a real engine.
- Physics for its own sake: a more faithful model that changes nothing audible
  or felt is not a priority.
- A thermodynamic analysis tool or a calibration tool for real vehicles.
- Adding a component (packed muffler, catalyst, resonator…) without a measured
  audible gap that justifies it.

## Decision rule

Before any work, answer one question:

> **Which measured gap between simulation and reality does this work reduce?**

Without an answer, the work waits. Exceptions: a bug that breaks the program,
and real-time performance on the reference machine.
