# Flashing OmniCtrl Pro 2

This repository targets the OmniCtrl Pro 2 with an STM32H723VGH6. The board's
USB Type-C connection exposes an integrated J-Link-compatible debug probe, so
J-Link is the preferred programming path.

## Safety first

Before flashing a full robot configuration, lift the chassis and disconnect or
disable motor power. The firmware initializes the configured CAN buses and
motor applications after boot.

## Build

```bash
just build
```

The default robot is `infantry`. To build another configuration, pass the robot
and optional job count as positional recipe arguments:

```bash
just rebuild hero 12
just rebuild sentry 12
just rebuild damiao 12
```

The generated files are:

- `build/nyush_rm_control_h723.elf`
- `build/nyush_rm_control_h723.hex`
- `build/nyush_rm_control_h723.bin`

## Recommended: integrated J-Link

1. Connect the board's USB Type-C port and make sure target power is present.
2. Install SEGGER J-Link Software and verify that `JLinkExe` is on `PATH`.
3. Program the currently selected robot:

```bash
just flash jlink
```

Equivalent explicit recipe:

```bash
just flash-jlink
```

For multiple probes or another robot configuration:

```bash
just flash jlink --robot hero --serial <JLINK_SERIAL>
```

The configured target name is `STM32H723VG`, interface `SWD`, with a default
speed of 4000 kHz.

## Other transports

Use these only when the corresponding probe or boot mode is actually available:

```bash
just flash pyocd   # pyOCD-supported CMSIS-DAP/ST-Link probe
just flash dfu     # STM32 ROM DFU mode; verify first with dfu-util -l
just flash         # auto-detect a supported transport
```

## Troubleshooting

- If J-Link cannot connect, close RTT dashboard/logger processes, reconnect USB,
  press reset, and retry at a lower speed with `just flash jlink --freq 1000`.
- If more than one probe is connected, pass `--serial` explicitly.
- If the board starts but peripherals do not respond, confirm that the firmware
  was built for `STM32H723VGH6` and check the CAN/UART connector mapping.
- If motors move unexpectedly, remove motor power immediately and verify the
  selected robot configuration and CAN IDs before another test.

See [rtt-dashboard.md](rtt-dashboard.md) for reading logs and telemetry after
programming.
