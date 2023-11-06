# boda
Up to this point the work completed is reading inputs from the controller and telling a motor to turn on or off. The PS2 controller input is read over SPI and decoded by the STM32F4 microcontroller. Then, if the input is x, the stepper motor is toggled using custom drivers to communcate with the A4988 chip using the ARDUINO headers. Messages describing the actions of the code are printed over serial for debugging. While we have both parts of this working seperately, a bug relating to ownership of pins for different functions is preventing us from enabling both of these functionalities simultaneously so work will have to be done to correct the pin assignments in STM32CubeMX.

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


**Luke:**
