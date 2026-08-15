# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

PX4 uses **CMake + Kconfig** with a `Makefile` wrapper. Board/variant targets defined as `boards/**/*.px4board`.

### Common Build Commands

```sh
# SITL (default for development)
make px4_sitl_default

# Firmware for hardware boards
make px4_fmu-v5_default
make px4_fmu-v6x_default

# SITL with specific simulator
make px4_sitl_default gazebo-classic
make px4_sitl_default gz

# Run unit tests (all, or filter by name)
make tests
make tests TESTFILTER=<name>

# Integration tests
make tests_integration

# Generate metadata (parameters, airframes)
make px4_metadata

# Code formatting (astyle)
make format
make check_format

# Clean
make clean
make distclean
```

Build outputs go to `build/<config>/`. Module enable/disable is in `.px4board` files via Kconfig (`CONFIG_MODULES_<NAME>=y`).

## Control Flow Architecture

```
Sensors → EKF2 (estimation) → Commander → Navigator
                                              ↓
        Flight Mode Manager → Controllers → Control Allocator → Actuators
                                (pos/att/rate)
```

| Layer | Module | Role |
|-------|--------|------|
| Estimation | `ekf2` | Attitude/position/velocity/wind estimation (primary) |
| Command | `commander` | Arming, mode transitions, failsafe logic |
| Navigation | `navigator` | Mission execution, geofence, RTL, takeoff/land |
| Flight Modes | `flight_mode_manager` | Translates nav setpoints → controller setpoints per mode |
| Controllers | `mc_pos_control`, `mc_att_control`, `fw_att_control`, `fw_rate_control` | Position/attitude/rate control |
| Mixing | `control_allocator` | Maps control signals to actuators |
| VTOL | `vtol_att_control` | MC↔FW transition management |

## Module Pattern

Every module in `src/modules/<name>/` has:
```cmake
# CMakeLists.txt
px4_add_module(MODULE modules__<name> MAIN <name> SRCS <name>.cpp)
```
- Entry point via `ModuleBase::task_spawn()` or `custom_command()`
- uORB messages in `msg/<Topic>.msg` (field defs with types), generated C++ headers at build time
- Subscribe: `uORB::Subscription`, Publish: `uORB::Publication`

## This Fork: ABC VTOL (v1.17.0)

Custom VTOL airframe types at `src/modules/vtol_att_control/`:
- `abc_vtol.cpp/h` — generic ABC VTOL
- `abc_vtol_4r.cpp/h` — 4-rotor ABC VTOL (this project)
- `standard.cpp`, `tailsitter.cpp`, `tiltrotor.cpp` — upstream types

Key parameters: `VT_TYPE=3` (ABC VTOL, `abc_vtol`), `VT_TYPE=4` (ABC VTOL 4-rotor, `abc_vtol_4r`, currently used for testing), `CA_AIRFRAME=17`, `VT_FW_DIFTHR_EN`.

Airframe configs in `ROMFS/`:
- `init.d/airframes/13889_abc_vtol_4r` (firmware)
- `init.d-posix/airframes/13889_gz_abc_vtol_4r` (SITL)

Quick workflow:
```sh
make px4_sitl_default
PX4_SYS_AUTOSTART=13889 ./build/px4_sitl_default/bin/px4
```

## Testing

- **Unit tests**: `px4_add_unit_gtest()` in module CMakeLists
- **Functional tests**: `px4_add_functional_gtest()` (needs full PX4 runtime)
- Run: `make tests TESTFILTER=<name>` or directly `./build/px4_sitl_test/unit-<name>`
- Integration: `make tests_integration` (MAVSDK-based, needs SITL)

## Formatting

- C++: astyle (Google-style derivative). `make format` to auto-fix
- CMake: lowercase functions, uppercase variables
