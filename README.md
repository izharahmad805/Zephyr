# CurveTracer

![License](https://img.shields.io/github/license/izharahmad805/CurveTracer)
![Stars](https://img.shields.io/github/stars/izharahmad805/CurveTracer)
![Release](https://img.shields.io/github/v/release/izharahmad805/CurveTracer)

**Microcontroller-based I–V Curve Tracer using INA219 and MCP4725 DAC**  

## Overview
CurveTracer is a project to measure **current–voltage (I–V) characteristics** of electronic components or solar panels. It uses:

- **INA219** for high-precision current measurement  
- **MCP4725 DAC** for voltage sweep  
- STM32 / ESP32 microcontroller for firmware control and data logging  

The system enables **real-time measurement**, data analysis, and hardware–firmware integration.

---

## Features
- Voltage sweep using MCP4725 DAC  
- Current measurement using INA219 sensor  
- Real-time data logging and I–V curve generation  
- Optimized firmware for **low power and efficiency**  
- Compatible with bare-metal and RTOS platforms  

---

## Hardware
- STM32 / ESP32 microcontroller  
- INA219 current sensor  
- MCP4725 DAC  


---

## Firmware
- Firmware handles DAC sweeping, INA219 measurement, and logging  
- Written in **C/C++**, optimized for **performance and low power**  
- **Zephyr RTOS** deployment  

---

## Usage
1. Connect the device under test (solar panel or component)  
2. Upload firmware to microcontroller  
3. Use logging interface to record I–V curves  
4. Analyze results for Voc, Isc, and maximum power point  

---

## Project Structure
