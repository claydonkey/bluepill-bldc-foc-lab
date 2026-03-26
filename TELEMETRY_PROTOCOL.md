# USB Telemetry and Command Protocol

This document describes the USB CDC protocol used by the `stm32f103cbt6_foc` firmware and the browser dashboard in [app.html](./app.html) and [app.js](./app.js).

It covers:

- command/response flow
- live FOC telemetry
- diagnostics telemetry
- velocity and position-related fields
- trace/debug data exported by the web UI

## Overview

The firmware exposes a line-based JSON protocol over USB CDC.

Browser to MCU:

- plain-text commands terminated with newline

MCU to browser:

- JSON objects terminated with `\r\n`

The browser dashboard polls the MCU using:

- `GET_TELEMETRY`
- `GET_DIAG`

and also sends control/tuning commands such as:

- `START`
- `STOP`
- `SET_VELOCITY:<value>`
- `SET_POSITION:<value>`
- `SET_PID:<kp>,<ki>,<kd>`
- `SET_POSITION_PID:<kp>,<ki>,<kd>`

## Transport

- Interface: USB CDC
- Framing: newline-delimited text / JSON
- Polling: controlled by the web app
- Default web polling interval: `200 ms` in [app.js](./app.js)

The firmware also sends a startup ready message:

```json
{"status":"ready","fw":"trace_dbg_3"}
```

## Main Commands

### Link / status

- `PING`
- `GET_STATUS`
- `GET_TELEMETRY`
- `GET_DIAG`

### Motor control

- `START`
- `STOP`
- `START_OPENLOOP`
- `TEST_VECTOR:<index>`
- `SET_VELOCITY:<rad_per_s>`
- `SET_POSITION:<rad>`

### Mode / modulation

- `SET_MODULATION:SVPWM`
- `SET_MODULATION:SINE`
- `GET_MODULATION`
- `SET_PHASE_MAP:<0..5>`

### Velocity tuning

- `GET_PID`
- `SET_PID:<kp>,<ki>,<kd>`
- `GET_VOLTAGE_LIMIT`
- `SET_VOLTAGE_LIMIT:<volts>`
- `GET_VELOCITY_RAMP`
- `SET_VELOCITY_RAMP:<rad_per_s2>`

### Position tuning

- `GET_POSITION_PID`
- `SET_POSITION_PID:<kp>,<ki>,<kd>`
- `GET_POSITION_VELOCITY_LIMIT`
- `SET_POSITION_VELOCITY_LIMIT:<rad_per_s>`
- `GET_POSITION_ACCEL_LIMIT`
- `SET_POSITION_ACCEL_LIMIT:<rad_per_s2>`
- `GET_POSITION_DECEL_LIMIT`
- `SET_POSITION_DECEL_LIMIT:<rad_per_s2>`
- `GET_POSITION_TORQUE_ASSIST`
- `SET_POSITION_TORQUE_ASSIST:<volts>`

### Low-speed assist

- `GET_FEEDFORWARD`
- `SET_FEEDFORWARD:<voltage>,<fade_speed>`
- `GET_LOW_SPEED_BIAS`
- `SET_LOW_SPEED_BIAS:<voltage>,<fade_speed>`

### Autotune

- `START_AUTOTUNE`
- `START_POSITION_AUTOTUNE`

## Telemetry Packets

There are two main telemetry packet types:

- `foc`
- `diag`

### `GET_TELEMETRY` response

The firmware returns a compact, scaled packet:

```json
{
  "foc": {
    "fw": "trace_dbg_3",
    "vi": 123,
    "mechi": 4567,
    "mrawi": 4501,
    "pmrawi": 4400,
    "mdelti": 101,
    "pmechi": 4466,
    "ti": 100,
    "tri": 95,
    "tpi": 6283,
    "erri": 5,
    "uqi": 120,
    "vlimi": 1200,
    "vrampi": 10000,
    "pvlimi": 400,
    "pacci": 8000,
    "pdeci": 8000,
    "pboosti": 0,
    "usat": 0,
    "mode": 1,
    "mod": 1,
    "pm": 0,
    "adiri": 1000,
    "aligni": 3037,
    "lc": 123456,
    "r": 2048,
    "pwm": [900, 850, 870],
    "per": 1799,
    "cb": 10000,
    "derr": 0,
    "start": 10000,
    "run": 1
  }
}
```

### Scaling

The firmware transmits many values as scaled integers to keep telemetry formatting lighter.

Scale by `100`:

- `vi` -> velocity, rad/s
- `ti` -> target velocity, rad/s
- `tri` -> ramped velocity target, rad/s
- `erri` -> velocity error
- `uqi` -> q-axis voltage command
- `vlimi` -> voltage limit
- `vrampi` -> velocity ramp rate
- `pvlimi` -> position velocity limit
- `pacci` -> position accel limit
- `pdeci` -> position decel limit
- `pboosti` -> position torque assist

Scale by `1000`:

- `mechi` -> multi-turn mechanical position, rad
- `mrawi` -> current raw mechanical angle, rad
- `pmrawi` -> previous raw mechanical angle, rad
- `mdelti` -> wrapped encoder delta, rad
- `pmechi` -> previous multi-turn mechanical angle, rad
- `tpi` -> target position, rad
- `adiri` -> sensor direction
- `aligni` -> alignment offset, rad

Not scaled:

- `fw` -> firmware tag
- `usat` -> `1` if voltage output is saturated
- `mode` -> control mode enum
- `mod` -> modulation mode enum
- `pm` -> phase map
- `lc` -> loop counter
- `r` -> raw AS5600 angle count
- `pwm` -> phase PWM compare values
- `per` -> PWM period
- `cb` -> encoder DMA callback count
- `derr` -> encoder DMA error count
- `start` -> encoder DMA start count
- `run` -> motor running flag

## Field Meanings

### Core motion fields

- `vi`
  - measured velocity
- `ti`
  - commanded target velocity
- `tri`
  - ramped/internal velocity target
- `mechi`
  - multi-turn mechanical position
- `tpi`
  - target position
- `erri`
  - velocity loop error
- `uqi`
  - commanded q-axis voltage

### Position / encoder debug fields

- `r`
  - raw AS5600 count
- `mrawi`
  - current raw mechanical angle in radians
- `pmrawi`
  - previous raw mechanical angle
- `mdelti`
  - wrapped angle delta between samples
- `pmechi`
  - previous multi-turn mechanical angle

These are especially useful when debugging:

- position jumps
- unwrap errors
- inconsistent position traces

### Runtime configuration fields

- `vlimi`
  - voltage limit
- `vrampi`
  - velocity ramp limit
- `pvlimi`
  - max inner speed in position mode
- `pacci`
  - position accel limit
- `pdeci`
  - position decel limit
- `pboosti`
  - position torque assist
- `mode`
  - control mode
- `mod`
  - sine / SVPWM mode

### Drive / health fields

- `usat`
  - output saturation flag
- `lc`
  - FOC loop count
- `cb`
  - DMA callback count
- `derr`
  - DMA error count
- `start`
  - DMA start count
- `run`
  - motor running state

## `GET_DIAG` Response

Diagnostics are lighter-weight and sent less often by the web app:

```json
{
  "diag": {
    "lc": 123456,
    "sent": 0,
    "failed": 0,
    "enc": 2048,
    "cb": 10000,
    "err": 0,
    "start": 10000,
    "run": 1
  }
}
```

Fields:

- `lc` -> loop count
- `sent` -> telemetry/messages sent counter
- `failed` -> telemetry/messages failed counter
- `enc` -> raw encoder count
- `cb` -> encoder DMA callbacks
- `err` -> encoder DMA errors
- `start` -> encoder DMA starts
- `run` -> motor running

## Other JSON Responses

The firmware also emits direct responses for individual commands, for example:

```json
{"ping":"pong"}
{"motor_driver":"enabled"}
{"motor_status":"running"}
{"modulation":"SINE"}
{"pid":{"kp":0.500,"ki":20.000,"kd":0.000}}
{"position_pid":{"kp":1.200,"ki":0.000,"kd":0.040}}
{"target_pos":12.345}
```

## Web App Trace Export

The browser dashboard exports a richer trace than the live packet stream.

The exported JSON includes:

- measured velocity history
- measured position history
- target velocity history
- target position history
- raw angle history
- raw mechanical angle history
- previous mechanical angle history
- mechanical delta history
- debug console lines
- active tuning values
- polling interval

This is the preferred format for offline analysis of:

- overshoot
- slow approach
- position discontinuities
- encoder unwrap issues

## Web App Decoding

The browser converts the scaled integer telemetry back to engineering values in [app.js](./app.js).

Examples:

- divide `vi` by `100` to get velocity
- divide `mechi` by `1000` to get position in radians

If you change firmware field names or scaling, you must update the decoding logic in the web app as well.

## Performance Notes

- The web app polls at a configurable interval, currently `200 ms` by default.
- `GET_DIAG` is intentionally less frequent than `GET_TELEMETRY`.
- The firmware uses scaled integers in `GET_TELEMETRY` to reduce formatting overhead compared with full float JSON.

## Recommended Use

For normal dashboard use:

- rely on `GET_TELEMETRY` for live motion data
- use `GET_DIAG` for slower health/status updates

For debugging:

- export trace + console from the web app
- compare measured velocity/position against target traces
- inspect `raw_angle`, `previous_mechanical_angle`, and `mechanical_delta` when position behavior looks wrong
