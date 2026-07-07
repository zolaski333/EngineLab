# Contributing to EngineLab

First off, thank you for considering contributing to EngineLab! It is people like you who make open-source software a better place.

---

## 📋 Code of Conduct

By participating in this project, you agree to abide by our [Code of Conduct](CODE_OF_CONDUCT.md).

---

## 🛠️ How Can I Contribute?

### 1. Reporting Bugs
- Always check if the bug has already been reported in the Issues section.
- Use a clear and descriptive title.
- Provide a step-by-step description of how to reproduce the bug, including your operating system and CPU details.
- Include logs, screenshots, or error traces if available.

### 2. Suggesting Enhancements
- Open an Issue with a suggestion.
- Explain the behavior you would like to see and why it would be beneficial to the simulation or audio synthesis.

### 3. Submitting Pull Requests
- Fork the repository and create your branch from `main`.
- Write clean, senior-level code following C++20 standards.
- Keep comments up-to-date and maintain existing documentation.
- Add unit tests inside the `tests/` directory for any new logic or physical equations.
- Ensure that the project compiles with no warnings and no errors (`/WX` is enabled on MSVC compilers).
- Run the test suite before submitting:
  ```powershell
  ctest --test-dir build -C Release --output-on-failure
  ```

---

## 🎨 Style Guide

- **Language:** Standard C++20.
- **Naming Conventions:**
  - Classes/Structs: PascalCase (e.g., `EngineSimulator`).
  - Variables/Functions: camelCase (e.g., `indicatedTorqueNm`).
  - Private member variables: camelCase ending with an underscore (e.g., `dynoMutex_`).
  - Constants: camelCase or UPPERCASE (e.g., `airGasConstant`).
- **Safety:** Use `noexcept` on functions where possible. Prefer standard library types and headers.
- **No Blocking in Audio Threads:** Any audio rendering code must not allocate memory on the heap, take locks, or communicate with UI threads directly. Use the SPSC queues.
