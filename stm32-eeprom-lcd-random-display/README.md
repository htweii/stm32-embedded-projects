# STM32 Random Number Generator with EEPROM and LCD

## Project Overview

This project demonstrates random number generation, external EEPROM storage, and LCD display using an STM32 Nucleo-L476RG.

A random number is generated using RTC and ADC noise as a seed, stored in an external EEPROM, and displayed on a 16x2 LCD.

---

## Hardware Platform

- MCU: STM32 Nucleo-L476RG  
- Development Tool: STM32CubeIDE  
- Programming Language: C  

External Components:
- 24LC256 I2C EEPROM
- LCD1602A display

---

## System Architecture

Random Generator → EEPROM Storage → LCD Display

A button press triggers generation of a new random number.

---

## Features

- Random number generation
- EEPROM data storage
- LCD number display
- Scrolling ticker display

---

## Peripherals Used

- I2C
- GPIO
- ADC
- RTC

---

## Implementation Details

Random numbers are generated using a combination of RTC timing and ADC noise.

The number is stored in external EEPROM using the I2C interface and then read back for display.

The LCD displays the number using a scrolling effect.

---

## Embedded Concepts

- I2C communication
- EEPROM memory interface
- LCD display control
- Random number generation in embedded systems
