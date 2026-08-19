# Sentry USB WASD Steering Test

This standalone mode controls four GM6020 steering motors and four M3508 drive motors. The physical CAN ID mapping is 1/2/3/4 = left-front/right-front/right-rear/left-rear.

The board accepts a heartbeat command over its USB CDC virtual serial port:

- `w`: forward, using each module's recorded mechanical zero.
- `a`: left, 90 degrees from the recorded zero.
- `s`: backward, 180 degrees from the recorded zero.
- `d`: right, -90 degrees from the recorded zero.
- `q`: forward-left (`w+a`), halfway between forward and left.
- `e`: forward-right (`w+d`), halfway between forward and right.
- `z`: backward-left (`s+a`), halfway between backward and left.
- `c`: backward-right (`s+d`), halfway between backward and right.
- `!`: disable all steering output.

After all four feedback streams have remained online for 200 ms, the board automatically selects `w` and returns every steering module to its recorded forward zero, even if the host tool has not started. It keeps holding that direction until USB control begins. Once a valid W/A/S/D command arrives, the M3508 drive motors start immediately while the GM6020 steering motors move toward the requested direction. A missing heartbeat for 500 ms disables all motor outputs. Any offline motor also disables all outputs. Red means a motor is offline, blue means host control has timed out or explicitly disabled output, yellow means steering is moving, and green means all four modules are aligned.

Each wheel module independently limits its steering move to 90 degrees. If the requested direction is more than 90 degrees from the module's current direction, the firmware selects the equivalent steering angle 180 degrees opposite and reverses that module's M3508 target speed. An exact 90-degree request keeps the normal drive direction.

Build and flash `sentry_swerve`, then connect the C board's USB CDC port to the host. Start the terminal controller with:

```sh
just _py scripts/sentry_steer_keyboard.py --port /dev/ttyACM0
```

The host tool reads Linux keyboard press/release events. Hold W/A/S/D to drive in that direction, or hold two adjacent keys for diagonal motion. Releasing one key falls back to the remaining direction, and releasing all direction keys immediately disables output. Press Space to disable output and Q to disable output and exit. It discovers keyboards through `/dev/input/by-id` or `/proc/bus/input/devices`. If more than one keyboard is connected, pass its `/dev/input/event*` path with `--input`. The user running the tool must have read access to that device. Raise the chassis and keep clear of the steering mechanism during initial direction-sign verification.

For control from a browser through an SSH tunnel, start the controller on the Jetson:

```sh
just _py scripts/sentry_steer_keyboard.py --port /dev/ttyACM0 --web
```

Keep that SSH session open. In a second terminal on the client computer, forward the web port:

```sh
ssh -N -L 8765:127.0.0.1:8765 nyu@JETSON_IP
```

Open the tokenized `http://127.0.0.1:8765/...` URL printed by the controller. The server listens on Jetson loopback only. Browser focus loss, missing browser heartbeat for 300 ms, controller exit, and the firmware's 500 ms command timeout all stop motor output.
