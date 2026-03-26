# STM32 DAC Waveform Generator

## Demo

![Demo]([Images]/demo.gif)

- Video:
https://youtu.be/BCqH8ATaU8o

---

## Project Overview

This project implements a multi-waveform signal generator using the DAC peripheral of the STM32 Nucleo-L476RG.

The system generates multiple analog waveforms using a waveform lookup table and continuously output through the DAC using DMA transfer.

A user button is used to switch between different waveform types in real time.

---

## Hardware Platform

- MCU: STM32 Nucleo-L476RG
- Development Tool: STM32CubeIDE
- Programming Language: C

External Components:
- Oscilloscope or signal analyzer (for waveform observation)

---

## System Architecture

Waveform Table → DMA → DAC → Analog Output

Timer2 triggers DAC updates at a fixed sampling rate while DMA continuously transfers waveform data to the DAC.

---

## Features

- Multiple waveform generation
- DAC analog output
- DMA circular transfer
- Button-controlled waveform switching

Supported waveforms:
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
- DMA
- Timer
- GPIO

---

## Implementation Details

Waveform samples are stored in lookup tables in memory.

DMA transfers waveform samples to the DAC data register in circular mode.

Timer2 is used to trigger DAC conversion at a fixed frequency.

---

## Embedded Concepts

- DAC signal generation
- DMA memory-to-peripheral transfer
- Timer-triggered sampling
- Embedded waveform synthesis
