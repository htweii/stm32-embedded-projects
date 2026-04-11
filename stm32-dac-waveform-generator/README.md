# STM32 DAC Waveform Generator

## Demo

![Demo]([Images]/demo.gif)

### Video Demonstration
- 📺 Video: https://youtu.be/BCqH8ATaU8o

---

## Project Overview

This project utilizes the STM32 Nucleo-L476RG's DAC to implement a multi-waveform signal generator, integrated with an Arduino Uno acting as the ADC acquisition stage to transmit data to the PC software ZaidaScope for real-time waveform verification.

The system uses waveform Lookup Tables to generate multiple analog waveforms, and users can switch between different waveforms in real-time through the User Button.

It leverages DMA Circular Mode for high-speed data transmission, demonstrating a robust cross-platform hardware verification setup.

### Documentation
- For more technical details, please refer to the:
  📄 [**Project Report (PDF)**](https://github.com/htweii/stm32-embedded-projects/blob/main/stm32-dac-waveform-generator/%5BDocs%5D/stm32-dac-waveform-generator-v1.0-hsu.pdf)

---

## Hardware Platform

- **MCU Control Source:** STM32 Nucleo-L476RG
- **MCU Acquisition Receiver:** Arduino Uno (Handles ADC sampling and forwarding)
- **Development Tool:** STM32CubeIDE
- **Programming Language:** C
- **External Component:** ZaidaScope (PC software, acting as an oscilloscope to observe waveforms)

---

## System Architecture

### Signal Generation End (STM32 Nucleo-L476RG)
Waveform Lookup Tables → DMA (Circular Mode) → DAC → Analog Signal Output

### Signal Monitoring End (Arduino Uno & PC)
STM32 DAC Output → Arduino ADC Sampling → Serial Transmission → ZaidaScope (PC)

---

## Features

- Multiple Waveform Generation
- DAC Analog Output
- DMA Circular Transmission
- User Button Control for Waveform Switching
- **Supported Waveforms:**
  - Sawtooth wave
  - Square wave
  - Sine wave
  - Trapezoidal wave
  - Half-sine wave
  - Triangle wave
  - Noise wave

---

## Peripherals Used

- DAC
- DMA Circular
- Timer
- GPIO & EXTI

---

## Implementation Details

- Waveform samples are stored in Lookup Tables in memory.
- The Timer2 triggers DAC conversion at a fixed frequency, and DMA transfers waveform samples to the DAC data register in circular mode without CPU intervention.
- The Arduino Uno samples the analog voltage output from STM32 and transmits values to ZaidaScope on the PC via Serial communication to output real-time waveforms, verifying the accuracy of the DAC output.

---

## Embedded Concepts

- DAC Signal Generation
- DMA Data Transfer from Memory to Peripheral
- Timer Triggered Sampling
- Embedded Waveform Synthesis
