# Tested environment snapshot

This is evidence for the 2026-08-19 validation, not a request to downgrade a
newer compatible ROS 2 Humble installation.

| Component | Tested value |
| --- | --- |
| Jetson OS | Ubuntu 22.04.5 LTS, Jetson Linux R36.5.0, aarch64 |
| Kernel | `5.15.185-tegra` |
| ROS | Humble desktop `0.10.0`; Nav2 `1.1.20` |
| PCL | `1.12.1` |
| GCC for STM32 | `arm-none-eabi-gcc 10.3.1 20210621` |
| CMake | `3.22.1` |
| Python | `3.10.12` |
| Just | `1.50.0` |
| RMW | Cyclone DDS `rmw_cyclonedds_cpp 1.3.4` |
| TigerVNC | `1.12.0`, system display `:1`, TCP port 5901 |

The source commits for FAST-LIO, Livox, and pcd2pgm are hard-pinned in
`dependencies.repos`; those pins are authoritative even when the apt package
versions above have advanced.
