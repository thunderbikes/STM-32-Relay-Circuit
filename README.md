# MOSFET Contol for MCU: STM32F205RGT6

This project is a microcontroller-based system designed to control an electric motorcycle by safely managing relay switches for SM24.

## Table of Contents:

1. [Installation](#installation)
2. [Introduction](#introduction)
3. [Features](#features)
4. [Customization](#customization)
5. [Operation](#oeration)
6. [Schematics](#schematics)
7. [Functions](#functions)
8. [Pending Tasks](#pending-tasks)
9. [License](#license)
10. [Contact](#contact)

## Installation

### Prerequisites
- STM32 MCU
- CubeIDE
- All related components and PCB

### Steps

Install CubeIDE and flash [main.c](main.c) onto STM32. 

## Operation

### Initialization:
- Initialize all GPIO pins and configure relays
*deepak might need to add some stuff here regarding what CubeIDE automatically does

### Main Control Loop:
- Transitions through states with ignition, operation and charge variables
- Begins in standby state physical ignition starting the precharge state or charge switch beginning charging. Pump is remains enabled forever during and after precharge
- Precharge ends and operation is true after high voltage sensor on motor reaches 90% of peak battery voltage. There is a 30 second timeout during this state for safey.
- During operation, bike functions as expected and waits for ignition to be turned off.
- Discharge state opens all relays, cuts power and waits 30 seconds before enabling power to put bike into standby. 	

### Emergency Handling:
- Constant timed interrupts that to verify state of bike with state of relays through auxiliary inputs
- Any errors will activate the error handling which opens all relays and also stops power
- External errors will be logged in MCU before opening all relays

## Customization
- How often the interrupt check’s relay states
- How many repeated “UPDATING” states before error handling kicks in

## Schematics
![Before final redesigns (outdated)](Schematic.pdf)

## Functions

### `allRelaysOpen()`

Opens all relays to avoid conflicts when switching between states.

### `allDigitalRead()`

Reads digital inputs for charge, ignition, and directional signals.

### `allAuxDigitalRead()`

Reads digital inputs for the three relay signals.

### `HAL_TIM_PeriodElapsedCallback()`

Interrupt function verifying AUX signals with current state

### `set_precharge()`

1. Starts tick and sets timeout at 30 seconds
2. Open all relays to ensure the system is in a safe state
3. Sets HVC_NEG_Pin and P_CHARGE_Pin to correct values
4. Begins precharge and waits for motor voltage to reach 90% peak battery voltage

### `while_operation()`

1. Listens for ignition to be turned off to enter discharge state
2. Otherwise continue normal operation

### `set_discharge()`

1. 30-second delay for any necessary pre-discharge preparations
2. Open all relays to ensure system is in a safe state before discharging 
3. Set CTRL_OK_Pin and check if it is set correctly

### `set_charging()`

1. Open all relays to ensure the system is in a safe state
2. Set CHARGE_NEG_PIN and check if it is set correctly
3. Set CHARGE_POS_PIN and check if it is set correctly

### `Error_Handler()`
Opens all relays and shuts down power (ctrl_ok=0)

## Pending Tasks

- CANbus intergration
- Error logging (through CANbus)

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact
Email inquiries to ubcthunderbikes@gmail.com


