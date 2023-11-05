# boda
Up to this point the work completed is reading inputs from the controller and telling a motor to turn on or off. The PS2 controller input is read over SPI and decoded by the STM32F4 microcontroller. Then, if the input is x, the stepper motor is toggled using custom drivers to communcate with the A4988 chip using the ARDUINO headers. Messages describing the actions of the code are printed over serial for debugging.

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
* Researched STM microcontrollers for prototyping


**Sean:**


**Leo:**


**Luke:**
