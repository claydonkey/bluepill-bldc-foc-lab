# Motor Velocity Telemetry over USB CDC

This document explains how to send motor velocity data from your STM32F103CBT6 to a web interface via USB CDC (serial connection).

## Overview

The firmware now:
1. **Measures** motor velocity in real-time via the FOC algorithm
2. **Sends** velocity data as JSON over USB CDC every 100ms
3. **Receives** and processes control commands from the USB host

The web interface (`app.html` and `app.js`):
1. **Connects** to the STM32 via WebSerial API
2. **Displays** live velocity data in a dashboard
3. **Sends** commands to control the motor

## Firmware Changes

### Modified Files

#### `Core/Inc/foc.h`
- Added `float FOC_GetVelocity(void);` - function to retrieve current motor velocity

#### `Core/Src/foc.c`
- Added `FOC_GetVelocity()` getter function that returns the static `velocity` variable

#### `Core/Src/main.c`
- Added `send_velocity_telemetry()` function that:
  - Sends velocity data every 100ms (configurable via `TELEMETRY_INTERVAL`)
  - Formats data as JSON: `{"vel":1.23,"target":0.40}`
- Added call to `send_velocity_telemetry()` in main loop
- Added telemetry timing variables

## Data Format

### Telemetry (MCU → Browser)
```json
{"vel":1.23,"target":0.40}
```
- `vel`: Current motor velocity (rad/s)
- `target`: Target velocity setpoint (rad/s)

### Commands (Browser → MCU)
Existing commands still work:
- `START` - Enable motor
- `STOP` - Disable motor
- `SET_VELOCITY:0.5` - Set target velocity to 0.5 rad/s
- `SET_PID:0.5,20,0` - Set PID parameters (Kp,Ki,Kd)

## Using the Web Dashboard

### Requirements
1. **Browser**: Chrome, Edge, or other browser supporting Web Serial API
2. **USB**: STM32 connected via USB (configured for CDC)
3. **Baud Rate**: 115200 (configured in `app.js`)

### Steps
1. Open `app.html` in your web browser
2. Click **"🔌 Connect Serial Port"** button
3. Select your STM32 device from the dialog
4. Watch real-time velocity in the gauges
5. Use buttons to control motor:
   - **Start Motor** - Enables motor
   - **Stop Motor** - Disables motor
   - **Set Velocity** - Enter desired velocity and apply

## Configuration

### Change Telemetry Interval
In `main.c`, modify `TELEMETRY_INTERVAL`:
```c
#define TELEMETRY_INTERVAL 100  // milliseconds
```

### Change Baud Rate
In `app.js`, modify the `open()` call:
```javascript
await state.port.open({ baudRate: 115200 });
```

### Change Data Format
Modify the `snprintf` format string in `send_velocity_telemetry()`:
```c
snprintf(telemetry_msg, sizeof(telemetry_msg),
    "{\"vel\":%.2f,\"target\":%.2f}\r\n", current_velocity, current_target);
```

## Troubleshooting

### No data received
1. Check USB connection is working (verify in Device Manager)
2. Verify baud rate matches (`115200`)
3. Check that `FOC_Loop()` is being called (velocity should be non-zero)

### Connection fails
1. Browser must support Web Serial API (Chrome/Edge/Opera)
2. HTTPS required for WebSerial (or localhost)
3. Try a different USB port

### Data formatting issues
1. Check trailing `\r\n` in telemetry message
2. Verify JSON format is valid
3. Open browser console to see received data

## Performance Notes

- Telemetry sends every 100ms ≈ 10 Hz update rate
- CDC buffer size is compatible with current message size
- JSON format is human-readable and easily parseable
- Consider reducing `TELEMETRY_INTERVAL` for faster updates if needed

## Hardware Requirements

- STM32F103CBT6 with USB CDC configured (already set up)
- USB cable for connection to computer
- 115200 baud rate serial communication

## Future Enhancements

1. Add more telemetry data (current, position, temperature, etc.)
2. Implement data logging to CSV
3. Add graph plotting over time
4. Implement PID tuning interface
5. Add error/warning logging
