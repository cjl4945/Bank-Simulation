# Bank Simulation

## Overview
This project simulates a banking environment, focusing on the queuing process. Developed for the **STM32L476RG** microcontroller, it utilizes **FreeRTOS** to manage real-time operations.

## Features
- **Queue Simulation:** Models customer arrival, service, and departure processes.
- **Real-Time Operation:** Employs FreeRTOS for task scheduling and management.
- **Embedded System Implementation:** Runs on the STM32L476RG microcontroller.

## Technologies Used
- **STM32L476RG Microcontroller:** A 32-bit ARM Cortex-M4 MCU from STMicroelectronics.
- **FreeRTOS:** An open-source real-time operating system for embedded devices.
- **C Programming Language:** Core language used for development.
- **Makefile:** For build automation.
- **Assembly:** Utilized for low-level hardware interactions.

## Setup and Usage
1. **Hardware Requirements:**
   - STM32L476RG microcontroller development board.
   - USB connection for programming and power.

2. **Software Requirements:**
   - STM32CubeIDE or compatible development environment.
   - ARM GCC toolchain.

3. **Building and Flashing:**
   - Clone the repository: `git clone https://github.com/cjl4945/Bank-Simulation.git`
   - Open the project in your IDE.
   - Build the project to generate the binary.
   - Flash the binary to the STM32L476RG microcontroller.

4. **Running the Simulation:**
   - After flashing, the microcontroller will start executing the simulation.
   - Monitor the output via a serial connection to observe the queue simulation.

## Project Structure
- `Core/`: Contains the main application code.
- `Drivers/`: Includes hardware abstraction layer drivers.
- `Middlewares/Third_Party/FreeRTOS/Source/`: FreeRTOS kernel source files.
- `Project3-Bank.ioc`: Configuration file for STM32CubeMX.
- `STM32L476RGTX_FLASH.ld`: Linker script for flash memory.
- `STM32L476RGTX_RAM.ld`: Linker script for RAM.

## Contributing
Contributions are welcome. Please fork the repository and create a pull request for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

