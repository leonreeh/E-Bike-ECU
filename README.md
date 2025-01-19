# E-Bike ECU

This repository contains the firmware and hardware design files for an Electronic Control Unit (ECU) tailored for E-Bikes. Developed as part of a technical project, it encompasses various components essential for the efficient operation and control of electric bicycles.

## Project Overview

The E-Bike ECU project aims to provide a comprehensive solution for managing and controlling the functionalities of an electric bicycle. The project includes:

- **BLDC-ESC-Firmware**: Firmware for the Brushless DC (BLDC) motor Electronic Speed Controller (ESC).
- **Gehäuse STL**: 3D printable STL files for the ECU housing.
- **LCD Case**: 3D printable STL files for the LCD display casing.
- **Motor Test Stand**: 3D printable STL files for a motor testing Setup.
- **RESC_V2 Hardware**: Hardware design files for the revised ESC version 2, Inlucding Bill of materials an Gerber files for PCB manufacturing.

## Features

- **Motor Control**: Control interface for Sensored BLDC motors up to 60V/1000W.
- **Intigrated Lighting Control**: Designed to drive 12V bike or motorcycle LEDs.


## Getting Started

To get started with this project:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/leonreeh/E-Bike-ECU.git
2. **Explore the Directories**:
Navigate to the BLDC-ESC-Firmware folder for motor control firmware.
Access the Gehäuse STL and LCD Case directories for 3D printable designs.
Refer to the Motor Test Stand folder for test stand setup 3D printable designs.
Review the RESC_V2 Hardware directory for the latest hardware schematic, layout and Bill of Material.

3. **Prerequisites Hardware**:
Sensored BLDC motor and Compatible Battery (12-60V).
3D printer for housing components.
Necessary electronic components as per the hardware design files.

4. **Software**:
STM32CubeIDE 1.16.1 or GNU Make for building the firmware.
Autodesk Fusion 360/Eagle for viewing and editing hardware schematics.
Installation
Firmware Compilation:
Navigate to the BLDC-ESC-Firmware directory.
Upload the compiled firmware to the ESC using ST-Link V2 or your preferred flashing tool.

## Acknowledgments
**Contributing**
Contributions to enhance this project are welcome. Feel free to fork the repository, make improvements, and submit pull requests.

**License**
This project is licensed under the MIT License. See the [wikipedia.org/wiki/MIT-License](https://de.wikipedia.org/wiki/MIT-Lizenz) for details.

This project was inspired and informed by several resources. Special thanks to:

1. **Microchip** AN4064 Application Notes: Sensored (Hall Effect Sensor-Based) Field Oriented Control of Three-Phase BLDC Motor Using dsPIC33CK.
2. **Texas Instruments**
   Application Report: Trapezoidal Control of BLDC Motors Using Hall Effect Sensors.
   Demystifying BLDC motor commutation: Trap, Sine, & FOC
4. **Simple-circuits.com** Sensored brushless DC motor control.
5. **Stratify Labs** Motor Control using PWM and PID.
6. **VESC-Project**  Hardware reference and inspiration.

Special thanks to all contributors and the open-source community for their invaluable resources.

