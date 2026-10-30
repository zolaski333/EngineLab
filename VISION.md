# EngineLab — Vision

This document sets the intent of the project. It rarely changes, and only by
decision of the owner. Everything else (code, docs, priorities) must conform to
it.

## What EngineLab must be

A four-stroke, two-stroke and rotary engine simulator **whose sound is as close to
reality as possible**, for any engine the user builds.

*Amended 2026-10-15 by the owner's decision: two-stroke engines are in scope
(piston-ported, crankcase-scavenged). A two-stroke follows the same rules as a
four-stroke: plausible figures against the maker's, real time, no per-engine
recording.*

*Amended 2026-10-26 by the owner's decision: there is no pilot engine any
more (the CP2 milestone is withdrawn), and EngineLab is open source, built for
many users and many computers, not for one.*

*Amended 2026-10-29 by the owner's decision (the owner's request of that day
for a rotary engine): Wankel rotary engines are in scope, with the same rules
as a piston engine: plausible figures against the maker's, real time, no
per-engine recording.*

1. **Sound first.** Someone who knows the real engine should recognise it by
   ear: not just "a parallel twin", but that engine. The method is open: physics,
   synthesis, or both. What counts is the audible result, not the elegance of
   the model.
2. **Physics that is broadly right.** Plausible torque and power (within about
   ±15 % of the manufacturer figures), credible reactions to throttle, gear and
   exhaust. Physics serves the sound and the feel; it is not an end in itself.
3. **Real time on a mid-range PC.** Reference machine: Intel i5-10600
   (6 cores / 12 threads), 16 GB of RAM. The whole simulation (physics,
   sound, threads) must use the computer it runs on as fully as it can, so a
   smaller PC runs more engines and a bigger one runs bigger engines. Nothing
   a user may build is capped to fit the reference machine: an engine that
   needs a faster PC is allowed.

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

There is no pilot engine. The recordings are whatever real engines can be
measured at a known engine speed: freely licensed recordings (CC0, CC BY) and
recordings contributors share. Two rules keep calibration generic:

1. **One recording never tunes the render.** A change calibrated on one
   engine's recording is kept only if it does not move the others away from
   their own recordings.
2. **Generalisation is the proof:** an engine that was not used to calibrate a
   change must improve, or at least not regress, **without specific tuning**.

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
