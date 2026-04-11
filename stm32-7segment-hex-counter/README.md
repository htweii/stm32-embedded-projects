# STM32 7-Segment Hexadecimal Counter

## Demo

Demo 1

![Demo]([Images]/demo-1.gif)

Demo 2

![Demo]([Images]/demo-2.gif)

### Video Demonstration
- 📺 Video 1: https://youtube.com/shorts/1x1EWusqgnU?feature=share
- 📺 Video 2: https://youtube.com/shorts/Xba3J3fb_Pc?feature=share

---

# Project Overview

This project develops a digital display counting system using the STM32 Nucleo-L476RG to implement an auto-cycling hexadecimal (0-F) counter. By controlling GPIO registers, the system utilizes a Segment Mapping Lookup Table to decode and display values on a Single-Digit 7-Segment Display.

### Documentation
- For more technical details, please refer to the:
  📄 [**Project Report (PDF)**](https://github.com/htweii/stm32-embedded-projects/blob/main/stm32-7segment-hex-counter/%5BDocs%5D/stm32-7segment-hex-counter-v1.0-hsu.pdf)

---

## Hardware Platform

- **MCU:** STM32 Nucleo-L476RG
- **Development Tool:** STM32CubeIDE
- **Programming Language:** C
- **External Component:** Single Digit 7-Segment Display

---

## System Architecture

- Hexadecimal Counter (0-F) → Segment Mapping Table → GPIO Output → Single Digit 7-Segment Display
- The firmware converts each hexadecimal digit into a specific segment pattern and drives the corresponding GPIO pins to update the display.

---

## Features

- Automatic execution of hexadecimal (0-F) continuous cycle counting
- Implementing display control of LED segments (a-g) based on GPIO output levels
- Using Lookup Table for segment encoding conversion to enhance real-time processing performance

---

## Peripherals Used

- GPIO (Configured as Push-Pull Output to drive LED segment displays)

---

## Implementation Details

- Designed a dedicated hexadecimal Segment Mapping Table for the 7-Segment Display LEDs, achieving a one-to-one conversion between logical values and hardware levels.
- Implemented bit operations on GPIO registers, updating GPIO pin states according to the segment patterns encoded in the Lookup Table to display corresponding values, optimizing real-time processing performance.

---

## Embedded Concepts

- GPIO Configuration
- Digital Output Control
- Utilizing Lookup Table to implement encoding definitions, reducing system computational resources
- Implementing the definition and conversion between software logic and hardware levels
