# Brightness-Coded Aperture Controller - ESP32 Firmware

## Overview

This project implements the hardware control layer for the **High-speed Motion Capture Using Continuously Changing Aperture** system described in the IERG5230 Final Project report.

The ESP32 microcontroller drives a 4-wire 2-phase stepper motor connected to a variable aperture mechanism. By continuously modulating the aperture size during camera exposure, temporal information is encoded into the brightness distribution of motion blur. This enables high-speed motion reconstruction from a single captured frame.

## Hardware Requirements

### Core Components
- **ESP32 Development Board** (ESP32-DevKitC or equivalent)
- **4-wire 2-phase Stepper Motor** (e.g., NEMA 17, 1.8° step angle)
- **Stepper Driver Module** (DRV8825, A4988, or TMC2209 recommended)
- **Variable Aperture Mechanism** (iris diaphragm or liquid crystal attenuator)
- **Home Limit Switch** (optional but recommended)

### Pin Connections (Default Configuration)

| Signal | GPIO Pin | Description |
|--------|----------|-------------|
| STEP   | GPIO 18  | Step pulse to driver |
| DIR    | GPIO 19  | Direction control |
| EN     | GPIO 21  | Driver enable (active low) |
| HOME   | GPIO 22  | Home limit switch (active low) |

### Power Requirements
- **ESP32**: 5V USB or 3.3V regulated
- **Stepper Motor**: 12V DC (depends on motor specifications)
- **Driver Logic**: 3.3V or 5V (check driver specifications)

## Software Architecture

### Module Structure
aperture_controller/
├── aperture_controller.h/c    # Core control logic and state machine
├── stepper_driver.h/c         # Low-level motor driver interface
├── motion_profile.h/c         # Smooth motion profile generation
├── transition_curves.h/c      # Mathematical transition curve implementations
├── brightness_encoder.h/c       # Brightness-to-aperture encoding model
└── main.c                     # Application entry point and tasks

### Key Features

1. **Smooth Motion Profiles**
   - Linear (constant velocity, testing only)
   - Sinusoidal S-curve
   - Cubic spline (continuous acceleration)
   - **Quintic spline** (continuous jerk, recommended for aperture)
   - Trapezoidal (industrial standard)

2. **Brightness-Coded Modulation**
   - 30Hz modulation frequency (synchronized with 60fps camera)
   - Triangular pattern: closing then opening each cycle
   - Real-time brightness encoding for decoder synchronization

3. **Safety Features**
   - Position fault detection
   - Emergency stop with immediate hardware disable
   - Homing timeout protection
   - Step loss detection via position tracking

## Building and Flashing

### Prerequisites
- ESP-IDF v4.4 or later
- Python 3.7+
- CMake and Ninja build tools

### Build Commands

```bash
# Set target device
idf.py set-target esp32

# Build project
idf.py build

# Flash to device (replace /dev/ttyUSB0 with your port)
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor