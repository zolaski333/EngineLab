EngineLab
=========

A four-stroke engine simulator whose sound comes from the simulated gas
dynamics, not from samples.

Project page, documentation and issue tracker:
    https://github.com/zolaski333/EngineLab


Getting started
---------------

1. Extract this archive anywhere (keep the folders next to EngineLab.exe).
2. Run EngineLab.exe. Windows SmartScreen may warn that the publisher is
   unknown, because the executable is not code-signed: choose
   "More info" > "Run anyway".
3. Pick an engine in the selector at the top of the window.
4. Hold A to switch the ignition on, then hold S to crank the starter.
5. Throttle: Q (1 %), W (10 %), E (20 %), R (100 %).
   Press D to run the automatic dyno, Tab to change screen, P to pause.

All key bindings can be changed from the KEYS button.


What is in this folder
----------------------

EngineLab.exe      the application
engines/           the engine catalogue (editable YAML)
parts/             camshafts, exhausts, fuels, gearboxes, vehicles
voicing/           the audio mix, per engine (hot-reloaded YAML)
examples/          example engine files and scripts (.els)
assets/            exhaust impulse responses
tools/             command-line offline audio renderers
*.dll              Microsoft Visual C++ runtime


Requirements
------------

Windows 10 or 11, 64-bit. The simulation runs in real time on a 6-core desktop
CPU for most engines; large engines (V8, V12) can fall below real time. The
realtime factor is shown in the diagnostics.


License
-------

EngineLab is released under the MIT license (LICENSE.md). Third-party
components and their licenses are listed in THIRD_PARTY_NOTICES.md.
