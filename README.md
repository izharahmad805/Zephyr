# CurveTracer
CurveTracer
Curve Tracer using INA219 and MCP4725 DAC

This project implements a solar cell / electronic component curve tracer using an ESP32/STM32 platform, INA219 current sensor, and MCP4725 digital-to-analog converter (DAC). The system sweeps voltages across the device under test and records the I–V characteristics, enabling analysis of component performance.

Features
Voltage sweep using MCP4725 DAC

Current measurement using INA219 sensor

Data acquisition and logging via STM32 / ESP32

Compatible with various electronic components and solar cells

Firmware optimized for real-time measurement and power efficiency

Hardware
Microcontroller: STM32 / ESP32

Current Sensor: INA219

DAC: MCP4725

Additional Components: MOSFET for load switching, voltage divider, supporting circuitry

PCB Design: Integrated for compact and reliable hardware–firmware operation

Software / Firmware
Firmware handles DAC sweeping, INA219 measurement, and data logging

Modular and optimized for low power consumption

Compatible with bare-metal and RTOS platforms

Written in C/C++ with STM32 HAL or Zephyr RTOS (optional)

Usage
Connect the curve tracer to the component or solar panel under test

Power the board and upload firmware to the microcontroller

Use the provided logging interface to record I–V curves

Analyze curves for electrical characteristics such as Voc, Isc, and maximum power point

Project Structure
graphql
Copy code
CurveTracer/
├── firmware/          # Microcontroller firmware
├── hardware/          # PCB schematics and BOM
├── docs/              # Documentation and datasheets
└── README.md          # Project overview
Future Improvements
Support for higher sampling rates and precision

Automated maximum power point tracking (MPPT) for solar panels

GUI for live plotting of I–V curves

References
INA219 Datasheet: Texas Instruments

MCP4725 DAC Datasheet: Microchip
