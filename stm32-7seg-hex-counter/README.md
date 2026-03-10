# STM32 7-Segment Hexadecimal Counter
This project implements a hexadecimal counter (0–F) using an STM32 Nucleo-L476RG and a single-digit 7-segment display.
The display increments every second and continuously cycles through all hexadecimal digits.

## Hardware
- STM32 Nucleo-L476RG
- 7-Segment Display (Common Cathode)
- 470Ω resistors

## Pin Mapping
- PB0 -> Segment A
- PB1 -> Segment B
- PB2 -> Segment C
- PB3 -> Segment D
- PB4 -> Segment E
- PB5 -> Segment F
- PB6 -> Segment G

## Features
- GPIO control for LED segments
- Hexadecimal digit encoding
- Array-based segment mapping
- Continuous counter loop

## Embedded Concepts
- Embedded GPIO control
- Bitmask LED pattern encoding
- Digital hardware interfacing
