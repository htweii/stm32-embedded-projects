# 📟 STM32 Embedded Systems Portfolio
A dedicated collection of **STM32 (ARM Cortex-M4)** projects focused on peripheral integration, real-time signal processing, and robust firmware architecture. 

With a solid foundation of **5+ years in Java (Android)** and practical experience in **Python-driven automation at MediaTek**, I am now fully committed to the field of embedded systems. 

I strive to apply my previous software engineering discipline to the firmware domain—focusing on writing modular, reliable code while continuously deepening my expertise in low-level hardware control.

---

## 📜 Certifications & Training
- **Code in Place 2025 (Python Programming & Computational Logic)** | Stanford University
- **High-Performance ARM Microcontroller Implementation** | Tze-Chiang Foundation of Science & Technology
- **STM32 Microcontroller Practical Course** | NYCU Laser System Research Center

---

## 🛠 Technical Skill Set

### Microcontrollers & Peripherals
- **Platform:** STM32F4 Series (ARM Cortex-M4)
- **Core Skills:** **EXTI** (Interrupts), **Timer (TIM)**, **DMA** (Direct Memory Access), **ADC/DAC**
- **Protocols:** **I2C** (EEPROM/LCD), **UART** (Serial Debugging), **GPIO** (7-Segment/Keypad)
- **Development Tools:** STM32CubeIDE, STM32CubeMX, **ZaidaScope (Logic Analyzer & Oscilloscope)**

### Software & Automation
- **Embedded C:** Modular Programming, State Machines, ISR Handling, Debouncing.
- **Languages:** **Python** (Automated Testing & Scripting), **Java** (System Logic & Android Background).

---

## 💻 STM32 Projects

### 1️⃣ [STM32 DAC Waveform Generator](https://github.com/htweii/stm32-embedded-projects/tree/main/stm32-dac-waveform-generator)
*Focus: Hardware acceleration and real-time user interaction.*
- **Core Implementation:** Leveraged **Circular DMA** to stream a versatile library of **7 waveform types** (Sawtooth, Square, Sine, Trapezoidal, Half-sine, Triangle, and Noise) through optimized Lookup Tables (LUT) directly to the 12-bit DAC, allowing the signal generation to run autonomously with **zero CPU overhead**.
- **Key Skill:** Implemented **real-time interrupt-driven switching** using the User Button with **software debouncing** to cycle through waveforms via a **State Machine** logic, while maintaining precise sampling rates through **Timer-triggering**.
- **✅ Verification & Results:** 📺 [**Video Demo**](https://youtu.be/BCqH8ATaU8o) | 📄 [**Project Report (PDF)**](https://github.com/htweii/stm32-embedded-projects/blob/main/stm32-dac-waveform-generator/%5BDocs%5D/stm32-dac-waveform-generator-v1.0-hsu.pdf)

### 2️⃣ [STM32 Random Number Generator with EEPROM and LCD](https://github.com/htweii/stm32-embedded-projects/tree/main/stm32-eeprom-lcd-random-display)
*Focus: Serial protocol mastery and non-volatile data persistence.*
- **Core Implementation:** Developed an I2C driver to interface with **24LC256 EEPROM** and **LCD1602**. Utilized **ADC floating-pin noise** as a random seed to generate non-deterministic values.
- **Key Skill:** Managed **multi-device I2C coexistence**, handled **16-bit memory addressing**, and **verified data persistence** to ensure data integrity across power cycles through optimized **Write-Cycle timing (5ms delay)**.
- **✅ Verification & Results:** 📺 [**Video Demo**](https://youtube.com/shorts/FCf5yvo7H1Q?feature=share) | 📄 [**Project Report (PDF)**](https://github.com/htweii/stm32-embedded-projects/blob/main/stm32-eeprom-lcd-random-display/%5BDocs%5D/stm32-eeprom-lcd-random-display-v1.0-hsu.pdf)

### 3️⃣ [STM32 7-Segment Hexadecimal Counter](https://github.com/htweii/stm32-embedded-projects/tree/main/stm32-7segment-hex-counter)
*Focus: Low-level I/O control and hardware logic implementation.*
- **Core Implementation:** Developed a real-time hexadecimal counter (0-F) driving a **7-segment display** through direct **GPIO manipulation**. Implemented a lookup-based logic to cycle through digits with precise timing control.
- **Key Skill:** Designed an efficient **Hex-to-7Segment encoding table** and managed hardware I/O mapping to drive individual segments with optimal logic levels.
- **✅ Verification & Results:** 📺 [**Video Demo 1**](https://youtube.com/shorts/1x1EWusqgnU?feature=share) / [**Demo 2**](https://youtube.com/shorts/Xba3J3fb_Pc?feature=share) | 📄 [**Project Report (PDF)**](https://github.com/htweii/stm32-embedded-projects/blob/main/stm32-7segment-hex-counter/%5BDocs%5D/stm32-7segment-hex-counter-v1.0-hsu.pdf)

---

## 🚀 Goal
To become a professional Embedded Firmware Engineer specializing in MCU systems.
