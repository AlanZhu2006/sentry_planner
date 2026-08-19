# NYUSH RoboMaster Control

NYUSH Robotics Club RoboMaster firmware based on Hunan University YueLu team's **basic_framework**.

- Quick start: see the [Setup Guide](docs/setup-guide.md) and [Flashing Guide](docs/flashing-guide.md)
- Toolchain: Justfile, Makefile, ARM GNU Toolchain
- Target: STM32H723VGH6 (OmniCtrl Pro 2)
- RTOS: FreeRTOS
- Original framework: [HNUYueLuRM/basic_framework](https://github.com/HNUYueLuRM/basic_framework)

> **For comprehensive framework documentation, features, and architecture details, see [docs/basic_framework/README.md](docs/basic_framework/README.md)**

## Documentation

- **[Jetson ROS 2 full-stack reproduction](ros2_ws/README.md)** - MID-360, FAST-LIO, ICP, Nav2, mapping, and WASD setup
- **[Swerve small-gyro migration design](docs/sentry-swerve-spin-mode.md)** - migration plan from the legacy sentry controller
- **[Setup Guide](docs/setup-guide.md)** - Development environment setup (Mac/Windows/Linux)
- **[Flashing Guide](docs/flashing-guide.md)** - Firmware flashing methods
- **[CAN Communication](docs/can.md)** - CAN bus protocol for GM6020 and C620 motors
- **[GitHub Commands](docs/github-commands.md)** - Git workflow reference

### Framework Documentation

- **[Basic Framework README](docs/basic_framework/README.md)** - Full framework introduction (Chinese)
- **[Architecture Guide](docs/basic_framework/架构介绍与开发指南.md)** - 3-layer architecture details (Chinese)
- **[VSCode + Ozone Setup](docs/basic_framework/VSCode+Ozone使用方法.md)** - Development workflow (Chinese)
- `docs/user-guides/` - Motor and legacy framework reference manuals

## Repository Structure

- `application/` - Robot control logic (cmd, gimbal, chassis, shoot)
- `modules/` - Hardware-agnostic drivers (motors, sensors, algorithms)
- `bsp/` - Hardware abstraction layer (CAN, UART, peripherals)
- `docs/` - Guides and references
- `ros2_ws/` - Reproducible Jetson MID-360/FAST-LIO/ICP/Nav2 stack
- `justfile` - Unified task entrypoint (build/flash/RTT)
- `Makefile` - Build system

## Build & Flash

### Build

```bash
just build                         # default: robot=infantry jobs=12
just build hero 16                 # robot, jobs
just rebuild infantry
```
Or use VSCode: `Ctrl+Shift+B` / `Cmd+Shift+B`

To pin this workspace to one robot locally, create `.robot_type` with one line:

```text
infantry
```

Supported values are `infantry`, `hero`, `sentry`, `sentry_swerve`, and
`damiao`. When this file exists, `just build` and `just flash` use it by
default, and passing a different robot value fails fast.
Run `just sync-vscode-robot` after changing `.robot_type` if you want VSCode IntelliSense to refresh before the next build or flash. Any `just build` / `just flash` also refreshes it automatically.

### Flash

```bash
just flash                         # auto-detect: pyOCD / J-Link / DFU
just flash pyocd                  # explicit pyOCD
just flash --method pyocd --serial 69655005
just flash jlink
```
Or use **STM32CubeProgrammer** (GUI) - see [Flashing Guide](docs/flashing-guide.md)

### RTT Tools

```bash
just logger-cli --mode split
just logger

# short aliases
just b infantry
just f pyocd
just lc --mode split
just lg
```

You can pass through script flags directly, for example:

```bash
just logger-cli --mode split --serial 69655005
just logger --ws-port 9000 --serial 69655005
```

These recipes auto-check Python environment and dependencies, and create `.venv` if needed.

### Sentry swerve full stack

The tested Jetson-side MID-360, FAST-LIO, ICP, Nav2, browser WASD, and VNC
configuration is versioned under `ros2_ws/`; see the
[full reproduction guide](ros2_ws/README.md). Daily entry points are:

```bash
just map arena
just wasd
just map-save arena
just nav arena
just logger
```

The latest validated `arena` PCD/PGM/YAML set is included. Navigation has no
wheel-odometry localization fallback and requires an explicit map name.

## Architecture

3-layer design with pub-sub messaging:

- **BSP** - Hardware abstraction (CAN, UART, peripherals)
- **Modules** - Hardware-agnostic drivers (motors, sensors, algorithms)
- **Application** - Robot control logic (gimbal, chassis, shooter, command)

Configuration: `application/robot_def.h`

## Credits

- **Original framework:** Hunan University YueLu RoboMaster Team (2022-2023)
- **NYUSH adaptation:** NYUSH Robotics Club
- License: MIT
