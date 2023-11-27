# Boda
The current bugs are the SPI communcation with the peripheral not always being detected properly (probably timing but we intend to replace the communcation protocol later anyways) and the motors not running smoothly consistently (most likely due to the voltage/current being provided to the motors)

**Kyle:**
* Aggregated various datasheets/manuals (1)
* Setup Github (.5)
* Got the configuration code generator and ide working including the built in debugger (2)
* Attempted to use various STM32 vscode extensions but none could do everything I wanted (2)
* Decided to use custom toolchain to get all features from STM32CubeIDE to VSCode without limitations of existing extension (10)
* Prepared toolchain and instructions for the rest of team (.5)
* Helped others get toolchain configured (1)
* Assisted in debugging reading SPI from the controller and driving the motor with ARDUINO/A4988 chip (4)

**Alex:**
* Researched STM microcontrollers for prototyping with chips that can be used for PCB's (1)
* Sourced and purchased materials (0.5)
* Searched for documentation of our microcontrollers, components, and peripherals (3)
* Researched Playstation 2 (PS2) Controller Protocol and how to implement it on hardware (2)
* Recreated the PS2 controller protocol on STM Nucleo-F446RE dev board over SPI to send commands and receive current states of the inputs (14) 
* Created a physical testing environment for hardware with waveforms and Diligent Analog Discovery 2 board (2)

**Sean:**
* Setting up and toubleshooting the work environment for STM32 (3)
* Reading documentation for the STM32, A4988, and PS2 controller (1)
* Arduino PWM initialization for the motor driver proof of concept (3)
* STM32 driver implementation for the stepper motors (8)
* Debugging and verifying waveforms using DAD2 board (2)
* Documenting changes and creating diagrams (2)
* Data compiling and video preparation (1)
  


**Leo:**
* Researched microcontrollers that encompased different power/ performance requirements. (2)
* Explored our selected microcontroller manual and relevant peripherals for our project. (3)
* Researched microstepping to achive higher rpms and smoother roations from stepper motors. (3)
* Setup Solidworks project for 3D modeling of arm (1)
* Researched different arms mechanics (4)
* Researched joints for the removable attachments. This will require some solidworks simulation to make sure that the joint meets our stress and strength requiremnets. (7)


**Lucas:**
* Resesarching methods for modeling the mechanics of the arm (5)
* researching stepper motors for limitations such as temp and stength(2)
* Researched Controller Protocol (2)
* found datasheets/manuals (1)
* setup Arduino PWM for the motor driver (3)
* learned the configuration code generator and ide (4)
* collecting required tools for modeling the arms in CAD (1)
* reviewed CAD techniques and methods in preparation to model the robot arm (2)
