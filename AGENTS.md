# AGENTS.md

Agent guidance for `nyush-rm-control` (STM32 RoboMaster firmware).

## Scope and precedence
- This file is for autonomous coding agents operating in this repository.
- This project is a RoboMaster lower-controller firmware written in C and built with the ARM none-eabi toolchain.

## Project snapshot
- Target MCU: STM32F407IGH6 (RoboMaster Type C board).
- Language: C11-style embedded C with `arm-none-eabi-gcc`.
- RTOS: FreeRTOS.
- Primary build entrypoint: `justfile`.
- Backend build system: `Makefile`.
- Layered design:
  - `bsp/`: hardware abstraction + HAL boundary.
  - `modules/`: reusable drivers/algorithms.
  - `application/`: robot behavior and coordination.

## Build / lint / test commands

### Command policy (important)
- Agents must use `just` commands only for build/clean/rebuild/verification workflows.
- Do not run `make` or CMake commands directly as an agent.
- If a needed operation is not exposed by `just`, ask the user before introducing a new recipe or using non-`just` commands.

### Standard build commands
- Build default config: `just build`
- Build specific robot: `just build robot=infantry`
- Build with custom parallelism: `just build robot=hero jobs=16`
- Clean: `just clean`
- Rebuild: `just rebuild robot=infantry`

### Flash commands (frequent during verification)
- Default flash (DFU): `just flash`
- Explicit methods:
  - `just flash dfu`
  - `just flash dap`
  - `just flash dap-openocd`
  - `just flash stlink`
  - `just flash jlink`
- Agent safety policy: do not execute flash/download commands as part of autonomous validation.

### Logging and local tooling
- RTT dashboard CLI: `just logger-cli --mode dashboard`
- RTT websocket bridge/dashboard: `just logger`
- Python env bootstrap for scripts: `just py-bootstrap`

## Lint and testing reality
- There is no dedicated unit-test framework in this repository.
- There is no standalone lint target (`lint`, `format`, `clang-format`) configured at repo level.
- Validation is compile-driven (CI also compiles via Make and CMake).
- Practical lint substitute: build with strict compiler warnings/errors through configured flags.

## Running a “single test”
When asked to run one test, use one targeted check:

1. Single configuration compile smoke test (fast and reliable):
   - `just build robot=infantry jobs=12`
2. Single-module on-board validation (manual):
   - Use the workflow in `CLAUDE.md`:
   - temporarily bypass `RobotInit()` in `Src/main.c`, init BSP + target module only, validate on hardware.

Always finish significant changes with at least one full build (`just build`).

## Architecture rules (must follow)
- Respect 3-layer separation:
  - App layer: no direct HAL/peripheral calls.
  - Module layer: use BSP instances, avoid direct HAL.
  - BSP layer: only place for HAL interactions.
- Use pub-sub (`modules/message_center`) for cross-app communication.
- Do not add direct coupling between app subsystems.
- Respect compile-time board role macros (`ONE_BOARD`, `GIMBAL_BOARD`, `CHASSIS_BOARD`).
- Check `application/robot_def.h` before behavior/config changes.

## Change strategy (important)
- Keep changes minimal and non-invasive; prefer the smallest patch that solves the task.
- This repo migrated from `basic_framework`; avoid modifying reusable framework modules unless necessary.
- If a task requires changing `modules/` behavior or adding a new cross-cutting interface, consult the user first.
- For new features, first search whether `basic_framework`-style interfaces already exist in this repo before introducing new ones.

## Code style conventions

### General formatting
- Keep files UTF-8 and LF line endings.
- Use existing local formatting style in touched files.
- Keep edits minimal and avoid unrelated formatting churn.

### Naming
- Functions: PascalCase verb-object phrases, ideally <= 4 words.
  - Examples: `RobotInit`, `ChassisTask`, `SetMotorControl`.
- Variables: `snake_case`, lowercase, descriptive.
- Macros/constants/enum values: `UPPER_SNAKE_CASE`.
- Type suffixes:
  - `_t`: simpler data types.
  - `_s`: complex structures/configs.
  - `_e`: enum types.
  - `XXXInstance`: module/BSP runtime instance objects.

### Includes/imports
- Use `#include "..."` for project headers.
- Keep include groups stable and minimal:
  1) current module header,
  2) other project headers,
  3) standard/CMSIS/HAL headers.
- Do not add unused includes.

### Types and units
- Prefer fixed-width integers (`uint8_t`, `int16_t`, etc.) for protocol/embedded boundaries.
- Use SI units internally (m, s, rad, etc.).
- Reuse conversion macros from `modules/general_def.h`.
- Preserve existing struct packing/alignment patterns for communication payloads.

### Robot-specific configurability
- Robot behavior is configuration-driven via `application/robot_configs/` and `application/robot_config_select.h`.
- For control logic changes (for example in `chassis.c`), prefer configurable switches/parameters over hardcoded global behavior changes.
- Default to affecting only the target robot configuration unless user explicitly requests all-robot behavior changes.

### Error handling and defensive checks
- Validate pointers, lengths, IDs, ranges, and mode values early.
- Use `LOGINFO` / `LOGWARNING` / `LOGERROR` for diagnosable issues.
- Follow fail-fast local patterns for impossible states where safety is critical.
- Do not silently ignore critical control-path errors.
- Apply the framework principle: “treat your users as idiots”.

### RTOS/concurrency rules
- Do not add blocking delay calls inside critical sections.
- Prefer DWT-based timing utilities (`bsp_dwt`) when precision is required.
- Protect shared state where preemption can break consistency.
- Keep task-loop timing and control frequency constraints in mind.

### Performance constraints
- Treat CPU time, stack/heap, and bus bandwidth as constrained resources on STM32.
- Avoid unnecessary allocations, repeated heavy math, and extra per-cycle logging in high-frequency control loops.
- Evaluate real-time impact when adding branches, copies, or synchronization in task hot paths.

### CubeMX-generated file safety
- In `Src/main.c`, `Src/freertos.c`, and related generated files:
  - place custom logic only inside `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks.
  - avoid edits outside user blocks unless unavoidable.

### Documentation expectations
- Public interfaces in headers should have concise comments.
- Add comments for non-obvious logic only; avoid comment noise.
- New modules should include a usage-oriented `.md` file in their module directory.

## Agent workflow recommendations
- Before edits, inspect board macros and topic wiring (`robot_def.h`, app init/task files).
- Run the narrowest useful check first, then one full build.
- Do not modify unrelated files while implementing focused changes.
- Keep behavior-impacting changes small and easy to review.
- If root cause is unclear, do not guess; add targeted SEGGER RTT logs (`LOGINFO`/`LOGWARNING`/`LOGERROR`) and use evidence to iterate.
- Keep code comments and docs in English; keep user-facing replies in the user's language.

## High-risk files (treat carefully)
- `application/robot_def.h` (global robot configuration).
- `Src/main.c`, `Src/freertos.c` (CubeMX regeneration risk).
- Motor/sensor drivers in `modules/` (real-time/safety sensitive).
- `justfile`, `Makefile`, `CMakeLists.txt` (project-wide developer impact).
