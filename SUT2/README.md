# Rocket Flight Control System

A comprehensive flight control algorithm for autonomous rocket flight with automatic parachute deployment based on sensor data and flight events.

## Overview

This system implements a complete rocket flight control algorithm that monitors sensor data in real-time and automatically deploys parachutes at the appropriate moments during flight. The system uses UART1 for communication and GPIO pins for parachute deployment control.

## Features

- **Automatic Launch Detection**: Detects rocket launch when Z-axis acceleration exceeds 10 m/s²
- **Motor Burnout Delay**: Waits 5 seconds after launch to ensure motor burnout phase completion
- **Altitude Monitoring**: Tracks altitude and deploys drogue parachute after reaching 1500m
- **Tilt Detection**: Monitors rocket body angles and detects excessive tilt (>60°)
- **Apogee Detection**: Automatically detects when the rocket begins descent
- **Dual Parachute System**: 
  - Drogue parachute deployment at apogee
  - Main parachute deployment at 550m altitude
- **Real-time Status Transmission**: Sends flight status every second via UART1
- **Command Interface**: Accepts ground station commands for test control

## Hardware Requirements

- STM32F1xx microcontroller
- UART1 for communication (115200 baud)
- GPIO 14 (Port B) for drogue parachute deployment
- GPIO 15 (Port B) for main parachute deployment
- Sensors for altitude, pressure, acceleration, and angle measurements

## Communication Protocols

### Telemetry Data Protocol

The system receives telemetry data in the following format:

| Byte | Content | Description |
|------|---------|-------------|
| 1 | 0xAB | Header |
| 2-5 | FLOAT32 | Altitude (meters) |
| 6-9 | FLOAT32 | Pressure (hPa) |
| 10-13 | FLOAT32 | Acceleration X (m/s²) |
| 14-17 | FLOAT32 | Acceleration Y (m/s²) |
| 18-21 | FLOAT32 | Acceleration Z (m/s²) |
| 22-25 | FLOAT32 | Angle X (degrees) |
| 26-29 | FLOAT32 | Angle Y (degrees) |
| 30-33 | FLOAT32 | Angle Z (degrees) |
| 34 | UINT8 | Checksum |
| 35 | 0x0D | Footer 1 |
| 36 | 0x0A | Footer 2 |

### Command Protocol

Ground station commands for test control:

| Function | Header | Command | Checksum | Footer 1 | Footer 2 |
|----------|--------|---------|----------|----------|----------|
| Start SIT | 0xAA | 0x20 | 0x8C | 0x0D | 0x0A |
| Start SUT | 0xAA | 0x22 | 0x8E | 0x0D | 0x0A |
| Stop Test | 0xAA | 0x24 | 0x90 | 0x0D | 0x0A |

### Status Transmission Protocol

The system transmits status every second:

| Byte | Content |
|------|---------|
| 1 | 0xAA (Header) |
| 2 | Status Word (lower byte) |
| 3 | Status Word (upper byte) |
| 4 | Checksum |
| 5 | 0x0D |
| 6 | 0x0A |

## Flight Event Sequence

1. **Launch Detection**: Z-axis acceleration > 10 m/s²
2. **Motor Burnout Delay**: 5-second wait after launch
3. **Altitude Threshold**: Altitude > 1500m
4. **Tilt Detection**: Body angle > 60° around X or Y axis
5. **Apogee Detection**: Altitude begins decreasing
6. **Drogue Deployment**: All conditions met, activate GPIO 14
7. **Main Parachute**: Altitude < 550m, activate GPIO 15

## Status Flags

| Bit | Description |
|-----|-------------|
| 0 | Launch detected |
| 1 | Motor burnout delay completed |
| 2 | Minimum altitude exceeded |
| 3 | Excessive tilt detected |
| 4 | Descent detected |
| 5 | Drogue parachute deployment command issued |
| 6 | Altitude below main parachute threshold |
| 7 | Main parachute deployment command issued |
| 8-15 | Reserved |

## Implementation Details

### Data Filtering

- **Moving Average Filter**: 5-point moving average applied to altitude and Z-axis acceleration
- **Checksum Validation**: All packets validated using modulo 256 checksum
- **Packet Validation**: Header and footer validation for data integrity

### GPIO Control

- **Drogue Parachute**: GPIO 14 (Port B) - activated for 100ms when deployment conditions met
- **Main Parachute**: GPIO 15 (Port B) - activated for 100ms at 550m altitude
- **Safety**: Both pins reset to low after deployment

### Timing Control

- **Status Transmission**: Every 1000ms (1 second)
- **Motor Burnout Delay**: 5000ms (5 seconds) after launch detection
- **Main Loop Delay**: 10ms to prevent busy waiting

## Usage

### 1. Compile and Flash

Compile the project in STM32CubeIDE and flash to your STM32F1xx microcontroller.

### 2. Connect Hardware

- Connect UART1 to your ground station or computer
- Connect GPIO 14 and 15 to parachute deployment mechanisms
- Ensure sensors are connected and providing data

### 3. Send Commands

To start the Synthetic Flight Test (SUT):
```
0xAA 0x22 0x8E 0x0D 0x0A
```

To stop the test:
```
0xAA 0x24 0x90 0x0D 0x0A
```

### 4. Monitor Status

The system will transmit status packets every second showing the current flight state.

## Testing

The system can be tested by:
- Sending command packets via UART1
- Monitoring status packets for flight state changes
- Observing GPIO pin states during flight events
- Using a serial terminal to send test commands

## Safety Features

- **Automatic Reset**: System resets to idle state on STOP command
- **GPIO Safety**: All deployment pins reset to low after activation
- **Data Validation**: All incoming data validated before processing
- **State Management**: Clear state transitions prevent unintended deployments

## Configuration

Key parameters can be adjusted in the `main.c` file:

```c
#define LAUNCH_ACCELERATION_THRESHOLD 10.0f      // m/s²
#define MOTOR_BURNOUT_DELAY_MS 5000             // 5 seconds
#define MIN_ALTITUDE_THRESHOLD 1500.0f           // meters
#define EXCESSIVE_TILT_THRESHOLD 60.0f           // degrees
#define MAIN_PARACHUTE_ALTITUDE 550.0f           // meters
```

## Troubleshooting

### Common Issues

1. **No Status Transmission**: Check UART1 connection and baud rate
2. **Parachutes Not Deploying**: Verify GPIO connections and sensor data
3. **System Not Responding**: Send STOP command to reset system

### Debug Information

- Monitor UART1 for status packets
- Check GPIO pin states during flight
- Verify sensor data is being received correctly

## License

This project is provided as-is for educational and development purposes. Ensure proper testing and validation before use in actual rocket flights.

## Support

For technical support or questions about the implementation, please refer to the code comments and this documentation.
