# VT03/VT13 Remote Control Notes

Source document checked into this repository:

- `docs/user-guides/RoboMaster裁判系统相机图传模块VT03&VT13使用说明书-CN&EN.pdf`

This note summarizes the VT03/VT13 UART frame fields used by the firmware and
the repo-specific control mapping built on top of that frame.

## UART protocol

- Frame length: `21 bytes`
- Transmission period: `14 ms`
- Baud rate: `921600`
- Data bits: `8`
- Stop bits: `1`
- Parity: `None`
- Flow control: `None`
- Start of frame: `0xA9 0x53`
- CRC in the DJI manual: `CRC-16/CCITT-FALSE`
  - Polynomial: `0x1021`
  - Initial value: `0xFFFF`
  - No input/output reflection
  - No XOR out

Compatibility note:

- The firmware also accepts the team's legacy `crc_16()` variant so older VT
  remotes can coexist with the manual-defined frame during migration.

## Bit fields used by the firmware

- `bit[26:16]`: `channel 0`, right stick horizontal
- `bit[37:27]`: `channel 1`, right stick vertical
- `bit[48:38]`: `channel 2`, left stick vertical
- `bit[59:49]`: `channel 3`, left stick horizontal
- `bit[61:60]`: `mode switch`
  - `0 = C`
  - `1 = N`
  - `2 = S`
- `bit[62]`: `pause button`
- `bit[63]`: `custom button (left)`
- `bit[64]`: `custom button (right)`
- `bit[75:65]`: `dial`
- `bit[76]`: `trigger`
- `bit[95:80]`: `mouse x`
- `bit[111:96]`: `mouse y`
- `bit[127:112]`: `mouse z`
- `bit[129:128]`: `mouse left`
- `bit[131:130]`: `mouse right`
- `bit[133:132]`: `mouse middle`
- `bit[151:136]`: keyboard bitmask
- `bit[167:152]`: CRC16

## Keyboard bitmap

- `bit0`: `W`
- `bit1`: `S`
- `bit2`: `A`
- `bit3`: `D`
- `bit4`: `Shift`
- `bit5`: `Ctrl`
- `bit6`: `Q`
- `bit7`: `E`
- `bit8`: `R`
- `bit9`: `F`
- `bit10`: `G`
- `bit11`: `Z`
- `bit12`: `X`
- `bit13`: `C`
- `bit14`: `V`
- `bit15`: `B`

## Current repo mapping

The VT remote path is mapped as follows in this repository:

- `mode switch`
  - `C`: chassis rotate mode
  - `N`: normal drive mode
  - `S`: clear emergency stop
- `pause button`
  - Toggles vision control
  - Enabling vision control disables keyboard/mouse control
- `trigger`
  - Manual fire
- `custom button (left)`
  - Latches emergency stop
- `custom button (right)`
  - Toggles keyboard/mouse control
  - Enabling keyboard/mouse control disables vision control
- `dial`
  - Commands chassis `wz` in normal drive mode
  - Positive and negative dial motion follow the dial direction

When keyboard/mouse control is enabled:

- `W/S/A/D`: chassis translation
- `mouse x/y`: gimbal yaw/pitch
- `Z`: cycle bullet speed
- `E`: cycle load mode
- `R`: toggle lid
- `F`: toggle friction wheels
- `C`: cycle keyboard movement speed scale

## Robot config switch

Each robot configuration selects the remote link through
`REMOTE_CONTROL_UART_HANDLE`:

- `huart1`: VT03/VT13 UART remote
- `huart3`: DBUS remote

The upper-layer control logic and the dashboard follow that robot config and
switch both parsing and display behavior accordingly.
