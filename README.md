# ESP32-Based Sensor PCB

## Overview
This ESP32-based sensor board is designed for various sensing and communication applications. It includes multiple integrated components such as light sensors, an I2C multiplexer, an RGB LED, RS485 communication, UART, and more. Below is a breakdown of the board’s features and how to interface with it.

<p align="center">
  <img src="https://github.com/blazjerman/ESP32-Based-Sensor-PCB/blob/main/pcb.png" width="300">
</p>

## Features
- **Light Sensor**: TEMT6000X01 (Analog Output on IO27, requires 3.3V power, and has a 10kΩ pull-down resistor to GND).
- **I2C Multiplexer**: TCA9548APWR (Address: 0x70, ESP32 SDA: IO21, SCL: IO22; four outputs labeled SD0/SC0 to SD3/SC3).
- **9V Booster**: TLV61046ADBVR (Enable pin: IO19; 9V output available on a labeled terminal block).
- **RS485 Communication**: MAX3485EESA+T
  - DI (Data In): IO17
  - RO (Receive Out): IO16
  - NEG_RE_DE (writing at high reading at low): IO18
  - Outputs: Terminal block labeled A & B
- **External UART**: Connected directly to ESP32 (TX: IO25, RX: IO26, available on a labeled terminal block).
- **Sound Sensor Compatibility**: RT-ZS-BZ-* (Uses UART on RX: IO12 and TX: IO14, operates on 5V).
- **RGB LED**:
  - Red: IO13
  - Green: IO02
  - Blue: IO32
  - Controlled via PWM signals and powered using transistors for higher current (Each LED draws ~20mA; see LTST-G563EGBW datasheet).
- **General-Purpose Terminal Block**: Connected to IO33 (Labeled "GP").
- **Button**: Button for any purpose (on press, it will pull I04 down).
- **Power Outputs**:
  - 3.3V (Labeled "3.3V")
  - 5V (Labeled "5V")
  - 9V (Labeled "9V")
- **USB Port**: Used for power and ESP32 flashing. Includes ESD protection and a 1A fuse.
- **Protection Features**:
  - RS485 protection diodes
  - Overcurrent protection on all power terminals

## Test Code
A `test` folder is included with example code for the Arduino IDE to test the board’s functionality. The folder contains examples for all features, making it easy to verify the hardware setup and operation.

## Pinout Summary
| Component             | ESP32 Pin  | Notes |
|----------------------|-----------|-------|
| **Light Sensor**     | IO27      | 3.3V, 10kΩ pull-down resistor |
| **I2C Multiplexer**  | IO21 (SDA) / IO22 (SCL) | Address 0x70 |
| **9V Booster**       | IO19      | Enable pin |
| **RS485** (MAX3485) | IO17 (DI) / IO16 (RO) / IO18 (RE/DE) | Terminal block labeled A & B |
| **External UART**    | IO25 (TX) / IO26 (RX) | Direct ESP32 connection |
| **Sound Sensor**     | IO12 (RX) / IO14 (TX) | RT-ZS-BZ-* sensor |
| **RGB LED**         | IO13 (Red) / IO02 (Green) / IO32 (Blue) | Controlled via PWM |
| **General Terminal** | IO33      | Labeled "GP" |
| **Button** | I04      | pull down |

## Licensing Information
- This project is licensed under the [MIT License](LICENSE).
- Some symbols, footprints, and 3D models are sourced from [SnapEDA](https://www.snapeda.com) and are licensed under [CC-BY-SA](https://creativecommons.org/licenses/by-sa/4.0/).

For updates or additional library formats, visit SnapEDA.

