# Minerva II Flight Computer Firmware

## Overview

This project provides the firmware for the **Minerva II** flight computer, a system designed for high power rockets (HPR). It manages sensor data acquisition, pyro channel control, state estimation, data logging, flight event detection, and data transmission autonomously during flight.

Written in C++ and built on top of the Arduino framework, the firmware is designed to run on a Teensy 4.1 inserted into a Minerva II sensor and power integration board with a focus on real-time responsiveness and reliability.

---

## Features

- **Sensor Integration**:
  - Accelerometer, gyroscope, magnetometer, barometer, GPS
  - Sensor fusion via the **Madgwick AHRS** filter for orientation and a **Kalman filter** for altitude estimation
- **Energetics Control**:
  - Support for multiple pyro channels with firing logic and continuity sensing
- **Flight State Detection**:
  - Detection of arming conditions
  - Apogee detection and timed parachute deployment
  - Flag-based state machine
- **Data Logging**:
  - SD card logging with actively managed buffer for performance
  - Dual-format output: binary and serial (for debugging)
- **Communications**:
  - Serial communication over UART or USB for live telemetry and command reception
  - Packet integrity verification using CRC32
- **Command Interface**:
  - Serial-triggered events

---

## Key Components

### `main.cpp`

- Main control loop
- Initializes all subsystems: sensors, state logic, logging, pyro channels
- Uses interrupts to handle real-time sensor updates
- Manages serial command handling and packet transmission

### `Pyro` Class (`pyro.cpp/.h`)

Handles:

- Firing pyrotechnic devices via GPIO
- Monitoring continuity with sensors
- Bitwise encoding of pyro status into a telemetry packet

### `Sensors` Class (`Sensors.h`)

Wraps multiple sensors and provides calibrated readings:

- **Accelerometer & Gyroscope**: via BMI088
- **Magnetometer**: MMC5983MA
- **Barometer**: BMP581
- **GPS**: u-blox M10S

Supports:

- Rotation correction via quaternion math
- Soft/hard iron calibration for magnetometer
- Continuous sensor polling using SPI/I2C

Note: The actual implementation of the sensor interface probably should not have been put in the .h file, but it works and so I never touched it -SP

### `State` Class (`state.cpp/.h`)

Implements the state machine for flight logic:

- Uses a bitmask (`uint16_t`) to store states, each bit in the 2-byte int represents a different flag
- Arming logic based on altitude, velocity, acceleration, and time
- Apogee detection via velocity near zero
- Time-delayed deployment of drogue and main parachutes

### 📦 `Kalman Filter` (`kf.h`)

Provides basic 3-state Kalman filtering to estimate:

- Position (altitude)
- Velocity
- Acceleration

Used to smooth noisy barometer and accelerometer readings.

### 💾 `Logging` Class (`logging.cpp/.h`)

Manages:

- SD card initialization and file creation
- Logging packet structures
- Buffering to optimize SD writes
- Also prints packets for debugging purposes

### 🧱 `structs.h`

Defines key data structures:

- `minerva_II_packet`: Telemetry and logging packet
- `state_packet`: Encodes flight state and related timestamps

---

## Build & Run

### Requirements

- Arduino-compatible microcontroller (Teensy, STM32, etc.)
- Arduino IDE or PlatformIO
- Required libraries:
  - `BMI088`, `BMP581`, `MMC5983MA`, `u-blox GNSS`
  - `Quaternion`, `BasicLinearAlgebra`, `MadgwickAHRS`
  - `SdFat`, `FastCRC`
- See: `platform.ini` for relevant dependency URLS

### Upload

Use the Arduino IDE, Teensy Bootloader, or Platform IO to upload `main.cpp` and ensure board configuration matches your hardware.

---

## Serial Commands

During flight or testing, you can trigger events using serial commands:

| Command | Action                     |
|---------|----------------------------|
| `P0H`   | Fire pyro channel 0        |
| `P1H`   | Fire pyro channel 1        |
| `P2H`   | Fire pyro channel 2        |
| `P3H`   | Fire pyro channel 3        |
| `CAM`   | Turn on camera (via P0)    |
| `FUL`   | Increase radio packet rate |
| `SAV`   | Decrease radio packet rate |

All commands use a CRC32 checksum and must be sent in the format:

0x44 0x55 <CMD (3 bytes)> <CRC32 (4 bytes)>

---

## State Flags

The flight logic uses a 16-bit bitmask for space efficient state tracking:

| Bit | Flag                        |
|-----|-----------------------------|
| 0   | ARMED                       |
| 1   | REACHED_ARMING_ALTITUDE    |
| 2   | REACHED_ARMING_VELOCITY    |
| 3   | REACHED_ARMING_ACCELERATION|
| 4   | REACHED_ARMING_DELAY       |
| 5   | REACHED_APOGEE             |
| 6   | FIRED_DROGUE               |
| 7   | FIRED_MAIN                 |
| 8   | FIRED_SUSTAINER (future)   |

---

## Developer Notes

- **Data Synchronization**: Sensor reads and state updates are handled on interrupt triggers to maintain time-aligned data.
- **Failsafe Handling**: SD card retries on failure; GPS reinitializes on timeout.
- **Timekeeping**: Uses `elapsedMicros` and `elapsedMillis` from Teensy framework.

---

## Authors

- Shivam Patel — [shivam.patel94@rutgers.edu](mailto:shivam.patel94@rutgers.edu), RPL 2021-2025
- Carlton Wu — [carlton.wu@rutgers.edu](mailto:carlton.wu@rutgers.edu), RPL 2021-2024

---

## License

&copy; 2021-2025 Shivam Patel and Carlton Wu. All rights reserved.
