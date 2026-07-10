# 🏎️ EngineLab

EngineLab is a real-time, high-fidelity internal combustion engine simulation and audio synthesis application built in modern C++20 and the JUCE framework. It targets a perceptual and acoustically rich simulation driven by coherent, physically motivated simplified equations rather than a 1:1 thermodynamic calculation.

---

## 🌟 Key Features

### 1. Advanced Engine Physics Simulation
- **Dynamic Torque Balance:** Real-time coupling of indicated combustion torque, mechanical friction, pumping losses, dynamic load, starter motor torque, and reciprocating piston inertia.
- **Exact Piston Kinematics:** Exact geometric slider-crank calculations for piston speed, acceleration, and reciprocating mass inertia forces.
- **Compressible Throttle Flow:** Isentropic sonic (choked flow) and subsonic restriction equations for manifold air mass calculations.
- **Sub-stepping Numerical Solver:** Multi-rate integration targeting at least 240 Hz internally to guarantee numeric stability under high torque spikes.

### 2. Physical Engine Configurations
- **Uneven-Firing & Layout Support:** Fully customizable bank angles and per-cylinder crank offset degrees (`crankOffsetDegrees`), allowing modeling of I2, I4, I5, V6, crossplane/flatplane V8s, and custom designs.
- **Exotic Geometry Catalog:** Built-in flat-six and radial-five examples use explicit crank journals, per-cylinder bank offsets, and preserved YAML/JSON round trips so non-standard layouts are first-class configs.
- **Camshaft Timing & Lift:** Dual camshaft (intake/exhaust) modeling parameterized by centerlines, durations, and lift profiles, dynamically animating valves and scaling volumetric efficiency.

### 3. Real-Time Audio Synthesis
- **Stereo Procedural Synthesis:** Lock-free, allocation-free real-time audio threads generating physical combustion pulses, intake noise, mechanical distribution clicks, and starter sound layers.
- **Pressure-Driven Exhaust Pulses:** Firing events carry cylinder pressure, exhaust runner pressure, mass flow, path delay, and resonance into the renderer.
- **Exhaust Graph Resonance:** Multi-node exhaust delay paths modeling piping length delays, primary pipe acoustic resonances, wave reflections, and an internal IR-style muffler network.

### 4. Interactive Dyno Sweep
- **Automated Dyno Sweep:** Automatic brake sweep mapping torque/power curves across the engine speed range.
- **Data Export & Analysis:** CSV curve export, comparison overlay plots, and complete engine configuration serialization (JSON/YAML).

---

## ⌨️ Controls & Navigation

- **`S` (Hold):** Starter motor engagement.
- **`W`:** 25% Throttle.
- **`E`:** 50% Throttle.
- **`R`:** 100% Throttle.
- **`D`:** Start/Stop Automated Dyno Sweep.
- **`F1` to `F12`:** Load available catalog engines.
- **`P`:** Pause/Resume simulation.
- **`Tab`:** Cycle engine, load/transmission, mixer, oscilloscope, and debug views.
- **`1` to `5`:** Set simulation speed scale (0.25x, 0.5x, 1x, 2x, 4x).

---

## 🛠️ Build Requirements

- **Windows 10/11** (64-bit)
- **Visual Studio 2022 or 2026** (with **Desktop development with C++** workload)
- **CMake 3.24 or higher**
- **Git**

All dependencies (JUCE, nlohmann_json, yaml-cpp) are managed directly by CMake via `FetchContent` and will be downloaded and compiled during the initial configuration.

---

## 🚀 Compiling the Project

Run the following commands in a PowerShell terminal:

```powershell
# 1. Configure the build directory
cmake -S . -B build -G "Visual Studio 18 2026" -A x64

# 2. Build the application in Release mode
cmake --build build --config Release --target EngineLabApp

# 3. Build and execute the test suite
cmake --build build --config Release --target EngineLabCoreTests
ctest --test-dir build -C Release --output-on-failure
```

*Note: For Visual Studio 2022, replace the generator option with `-G "Visual Studio 17 2022"`.*

The compiled executable will be located at:
`build/src/app/EngineLabApp_artefacts/Release/EngineLab.exe`

---

## ⚖️ License & Contributions

Please refer to [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) for community guidelines. This repository is licensed under the MIT License.
