# STM32 Random Number Generator with EEPROM and LCD

## Demo

![Demo]([Images]/demo.gif)

### Video Demonstration
- 📺 Video: https://youtube.com/shorts/FCf5yvo7H1Q?feature=share

---

## Project Overview

This project utilizes the STM32 Nucleo-L476RG to develop a data processing system, utilizing RTC values and noise from an ADC Floating Pin as a Random Seed to generate 4-digit random values. It implements access and read/write operations to external memory EEPROM through I2C protocol, and designs a dynamic scrolling effect to display the random values on a 16x2 LCD.

The system incorporates Human-Machine Interface concepts, allowing users to trigger the generation of new 4-digit random values by pressing the User Button, providing an interactive experience with immediate visible effects.

### Documentation
- For more technical details, please refer to the:
  📄 [**Project Report (PDF)**](https://github.com/htweii/stm32-embedded-projects/blob/main/stm32-eeprom-lcd-random-display/%5BDocs%5D/stm32-eeprom-lcd-random-display-v1.0-hsu.pdf)

---

## Hardware Platform

- **MCU:** STM32 Nucleo-L476RG
- **Development Tool:** STM32CubeIDE
- **Programming Language:** C
- **External Components:** 24LC256 I2C EEPROM, LCD1602A Display

---

## System Architecture

- **[Input]** User Button (External Interrupt) → **[Processing]** RTC + ADC Floating Noise Random Number Generation → **[Storage]** I2C EEPROM Data Storage / Reading → **[Output]** LCD Display
- The system adopts an event-triggered architecture; when the user presses the User Button, it immediately initiates data acquisition and display updates.

---

## Features

- RTC + ADC Floating Noise random number generation
- EEPROM data storage, data is not lost after power-off
- LCD dynamic scrolling display control
- Software Debouncing

---

## Peripherals Used

- I2C (EEPROM Read / Write)
- ADC (Floating Input Noise Random Seed)
- RTC (Random Seed)
- GPIO & EXTI

---

## Implementation Details

- By combining RTC timing values and ADC floating pin noise, highly stochastic random values are generated.
- Random values are interfaced through I2C protocol to be stored in the external EEPROM, and the random values are read back from the EEPROM.
- The LCD displays random values with a scrolling effect.
- Software Debouncing is implemented in the main loop to prevent spurious triggers caused by button bouncing.

---

## Embedded Concepts

- I2C Communication
- Handling External Memory EEPROM
- LCD Display Control
- Random Number Generation in Embedded Systems
