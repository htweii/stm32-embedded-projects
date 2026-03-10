# STM32 7-Segment Hexadecimal Counter

## Project Overview
This project implements a hexadecimal counter using an STM32 Nucleo-L476RG and a single-digit 7-segment display.

The system continuously counts from 0 to F and displays the value on the 7-segment display by controlling the segment LEDs through GPIO pins.

The display increments every second and continuously cycles through all hexadecimal digits.

## Hardware Platform
- MCU: STM32 Nucleo-L476RG  
- Development Tool: STM32CubeIDE  
- Programming Language: C  

External Components:
- Single digit 7-segment display

## System Architecture
Counter → Segment Lookup Table → GPIO Output → 7-Segment Display

The firmware converts each hexadecimal digit into a segment pattern and drives the corresponding GPIO pins.

## Features
- Hexadecimal counter (0–F)
- GPIO-based LED segment control
- Lookup table segment encoding

## Peripherals Used
- GPIO

## Implementation Details
Each hexadecimal digit is mapped to a 7-segment LED pattern using a lookup table.

The firmware updates the GPIO output pins according to the encoded segment pattern.

## Embedded Concepts
- GPIO configuration
- Digital output control
- Hardware LED interface
- Lookup table based encoding
