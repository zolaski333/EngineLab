# Contributing to EngineLab

Thank you for considering a contribution. EngineLab is a small project with a
strict method; this page explains it so that your time is well spent.

By taking part you agree to the [code of conduct](CODE_OF_CONDUCT.md).

## Before you start

Read [VISION.md](VISION.md). EngineLab's goal is a sound that is as close as
possible to real engines, judged against real recordings. Every change should
be able to answer one question: *which measured gap between simulation and
reality does it reduce?* Bug fixes and real-time performance are the
exceptions.

For anything larger than a small fix, open an issue first so the approach can be
agreed before you write the code.

## Reporting bugs and suggesting features

Use the issue templates. A good bug report says which engine, which action, what
you expected and what happened, plus the EngineLab version (title bar) and your
CPU. For sound problems, an exported clip (AUDIO HQ) is worth more than a
description.

## Building and testing

Windows, Visual Studio 2022 (*Desktop development with C++*), CMake 3.24+:

```powershell
cmake --preset windows-vs2022
cmake --build --preset windows-release
ctest --preset windows-release
```

The full suite takes about 15 minutes. If MSBuild fails with `C1060` (compiler
out of heap space), rebuild with fewer parallel nodes, for example
`cmake --build --preset windows-release -- /m:1`.

## Pull requests

- Branch from `main` and keep each PR to one topic.
- The build must have **zero warnings** (warnings are errors) and the **whole**
  `ctest` suite must pass. CI checks both on every push.
- A new test must fail without your fix.
- **Measure, don't guess.** If the change touches physics or audio, include the
  output of the matching harness before and after (see the README's *Measure,
  don't guess* section). A change that alters two factors at once proves
  nothing.
- **Do not change the default sound** unless that is the explicit purpose of
  the PR, shown with the harness before/after.
- Reference values come from the literature or from real recordings, never from
  the simulator's own current output.
- Update the matching page under `docs/` when behaviour changes, and add a line
  to `CHANGELOG.md` for anything users will notice.
- Everything is written in English: code, comments, UI strings, docs and commit
  messages.

## Style

- Standard C++20; follow the style of the surrounding code.
- Classes and structs in `PascalCase`, functions and variables in `camelCase`,
  private members with a trailing underscore (`dynoMutex_`).
- Quantities carry their unit in the name (`indicatedTorqueNm`,
  `exhaustBackPressureKpa`).
- **Nothing blocking on the audio thread:** no heap allocation, no locks, no file
  access and no direct UI calls. Use the existing SPSC queues.
- Comments explain *why*, especially when a value compensates for something
  elsewhere.
