# Real-Time Embedded 3D Raycaster Game

An ARM Cortex-M0+ based 3D Wolfenstein-style raycaster featuring real-time rendering on an ST7735 TFT display via SPI, custom fixed-point mathematics, and ISR-driven audio and control systems.

---

## 📌 Project Overview
This project implements a full 3D raycasting engine on a resource-constrained ARM Cortex-M0+ microcontroller without hardware floating-point support. The system uses fixed-point arithmetic to achieve a smooth 30Hz target render loop while handling simultaneous user inputs and hardware-driven audio.

---

## ⚙️ Technical Highlights

* **Fixed-Point Raycasting Engine:** Custom Digital Differential Analysis (DDA) ray marching and sprite rendering optimized for high-frame-rate rendering on a 128x160 ST7735 LCD display.
* **Interrupt-Driven Subsystems:** ISR-synchronized ADC sampling for analog joystick, potentiometer, and push-button controls to minimize input latency.
* **Audio & FSM Logic:** Dual-timer ISR DAC audio playback paired with a 4-state Finite State Machine (FSM) controlling game loops, dynamic difficulty scaling, and a randomized reward FIFO.
* **Graphics & Localization:** Custom font rendering supporting localized English and Japanese glyphs for game HUDs and menus.

---

## 🛠️ Hardware & Pinout Setup

* **Microcontroller:** ARM Cortex-M0+ (TI MSPM0)
* **Display:** ST7735 128x160 TFT Display (SPI Protocol)
* **Inputs:** 2-Axis Analog Joystick (ADC), Potentiometer (ADC), Direct Push Buttons
* **Audio:** R-2R Ladder DAC Output connected to an audio amplifier/speaker
