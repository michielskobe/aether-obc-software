# RMU REV02 Patch

This branch acts as a patch for the incorrect RMU REV02 RXSM signal routing. The original SODS line is removed since this is not connected on the PCB. The LO and SOE lines are crossed since it is routed this way on the PCB. The new updated SOE is renamed to SODS and acts as the SODS signal. The system doesn't wait for a dedicated SOE signal anymore. RX Pin Active Level Inversion is enabled since there is a _P/_N crossing on Rev 02 of the RMU PCB.

# Aether OBC Firmware

Firmware for Aether's On-Board Computer (OBC) based on the STM32L476 microcontroller.

This repository contains an STM32CubeMX-generated project implementing GNSS data acquisition, SATCOM communication, SD card storage, and CAN communication for embedded applications.

## Features

- UART-based GNSS driver
  - Compatible with the Septentrio mosaic-X5 receiver
  - NMEA message parsing
- UART-based SATCOM driver
  - Compatible with the Iridium 9603N transceiver
- SD card driver
  - SPI interface
  - Data logging support
- CAN communication
  - Sensor and subsystem data acquisition
- STM32CubeMX project configuration
- Target platform: STM32L476

## Hardware

### Microcontroller
- STM32L476

### Peripherals
- UART for GNSS and SATCOM communication
- SPI for SD card access
- CAN interface for data acquisition

### Supported GNSS Receiver
- Septentrio mosaic-X5

### Supported SATCOM Transceiver
- Iridium 9603N

## Project Structure

```text
.
├── Core/                 # Application source code
├── Drivers/              # STM32 HAL and CMSIS drivers
├── Middlewares/          # Optional middleware components
├── Scripts/              # Python scripts for debugging purposes
├── *.ioc                 # STM32CubeMX project file
└── README.md
```

## Building the Project

### Requirements
* STM32CubeMX
* STM32CubeIDE (or another ARM GCC-based toolchain)
* STM32 HAL libraries

### Steps

1. Clone the repository:

```bash
git clone <url-to-git-repository>
```
2. Open the .ioc file in STM32CubeMX or STM32CubeIDE.
3. Generate code if required.
4. Build the project using STM32CubeIDE.
5. Flash the firmware to the STM32L476 target board.

## Functionality Overview

### GNSS

The firmware communicates with a Septentrio mosaic-X5 receiver over UART and parses NMEA messages to extract positioning information. GNSS data is then logged and transmitted via the Iridium satellite network.

### SATCOM

The firmware communicates with an Iridium 9603N transceiver over UART and transmits GNSS positioning data through the Iridium satellite network.

### SD Card Logging

Data can be stored on an SD card through the SPI peripheral for later retrieval and analysis.

### CAN Data Acquisition

The CAN interface is used to receive and process data from connected subsystems and sensors. Furthermore, connected subsystems are also orchestrated via CAN according to the mission's timeline.

## Future Additions and Improvements
* Implementation of Iridium transmission software
* Enhanced fault handling
