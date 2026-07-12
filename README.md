# Gesture Recognition System

**Project:** Gesture Recognition System  
**Maintainer:** Bibek Poudel (<bp2376@nyu.edu>)  
**Framework:** Mbed OS  
**Target Hardware:** STM32F429I Discovery Board  
**License:** MIT

---

## Overview

This project implements a real-time gesture recognition system using the onboard gyroscope of the STM32F429I Discovery board. The system captures motion data, converts it into directional patterns, and compares user gestures against predefined reference gestures.

Developed in C++ with Mbed OS, the project demonstrates embedded systems concepts such as sensor interfacing, SPI communication, calibration, signal processing, and real-time feedback using LEDs and push buttons.

---

## Features

- Real-time gesture acquisition using the gyroscope sensor
- Gesture mapping and directional encoding
- Gesture matching against reference patterns
- Sensor calibration for improved accuracy
- Visual feedback through onboard LEDs
- User-controlled gesture recording via push buttons

---

## Hardware Components

| Component | Description |
|:----------|:------------|
| Gyroscope Sensor | Measures angular motion along the x, y, and z axes |
| STM32F429I Discovery Board | Main embedded platform |
| LEDs | Indicate recognition success or failure |
| Push Button | Starts gesture recording |
| SPI Interface | Enables communication with the sensor |

---

## System Workflow

```text
Initialize System
        │
        ▼
Calibrate Gyroscope
        │
        ▼
Record Reference Gesture
        │
        ▼
Wait for Blue Button Press
        │
        ▼
Capture User Gesture
        │
        ▼
Map Sensor Data
        │
        ▼
Compare with Reference
        │
        ▼
Display Result Using LEDs
```

---

## Project Structure

### Core Functions

| Function | Description |
|:----------|:------------|
| `record_gesture_data()` | Captures gyroscope data during gesture execution |
| `map_gesture_data()` | Converts raw sensor values into directional representations |
| `compare_gesture()` | Compares the recorded gesture with the reference pattern |
| `calibrate_sensor()` | Computes sensor offsets and improves accuracy |

---

## Results

### Performance

- ~90% gesture recognition accuracy
- Robust against small timing variations
- Real-time processing on embedded hardware

### Limitations

- Performance decreases for very fast gestures
- Slow gestures may produce matching errors
- Calibration depends on environmental conditions

---

## Future Improvements

- Improve calibration robustness
- Support more gesture classes
- Integrate additional sensors
- Develop a graphical calibration interface
- Improve tolerance to gesture speed variations

---

## Prerequisites

Before building the project, install:

- Mbed OS
- Mbed CLI or Mbed Studio
- ARM-compatible C++ toolchain
- STM32F429I Discovery Board

---

## Installation

Clone the repository:

```bash
git clone https://github.com/bk-poudel/Real_Time_Embedded_Systems_Project.git
cd RTES_FINAL_Project
```

Compile using Mbed:

```bash
mbed compile
```

Flash the generated binary to the STM32F429I Discovery board.

---

## Usage

### Recording the Reference Gesture

The system records a reference gesture during startup.

To record a new gesture:

1. Press the reset button.
2. Perform the desired reference gesture.

### Performing a Gesture

1. Press the blue button on the STM32F429I Discovery board.
2. Perform the gesture.
3. Wait for the recognition result.

### LED Indicators

| LED State | Meaning |
|:-----------|:---------|
| 🔴 Red LED OFF | Gesture successfully recognized |
| 🔴 Red LED ON | Gesture not recognized |

---

## Contributors

- Bibek Poudel
- Rugved Mhatre
- Akshay Parihalkar

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.
