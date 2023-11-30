# Boda
The current bugs are the SPI communcation with the peripheral not always being detected properly (probably timing but we intend to replace the communcation protocol later anyways) and the motors not running smoothly consistently (most likely due to the voltage/current being provided to the motors)

**Kyle:**
* Learned how to properly configure our toolchain such that VSCode properly reports any errors and is smooth to work in for every aspect of our project and so that we can easily used the cubeMX tool to configure our processor as desired (2)
* Helped integrate the motor/controller drivers into the RTOS so that controller inputs drive motors (10)
* Helped in debugging the integration of both drivers into the RTOS (5)
* Ensure consistent formatting between all created files and handle merging of branches (3)
* Experiment with all driver and RTOS code written by teammates and communicate with them to ensure I have a clear understanding of the operations of and interactions between all components of our project so that I can better understand any bugs which arise (15)
* Assembled our prototype breadboard in a layout which guarantees easy debugging and testing of all systems while being relatively easy to understand and follow (1)

**Alex:**
* Learning freeRTOS principles, researching STM32 freeRTOS implementation and libraries (15)
* Converting one infinite loop implementation to task/thread based freeRTOS implementation (25)
* Debugging freeRTOS implementation for acceptable task/thread scheduling. (5)
* Create an attachment testing environment, communicating through SPI, for future use. (10)
* Experimented with servo implementation for better motor control using current controller drivers. (10) 
* Finalize PS2 controller drivers, create example cases, create logic for controlling servos using incoming data (5)

**Sean:**
* STM32 driver implementation for the stepper motors V1 and V2 (16)
* Debuggin drivers and increasing reliability (14)
* Creating a parent driver to keep track and manipulate multiple A4988 drivers (7)
* Research and analysis of multi-motor movement. Created algorithm for linear motion (6)
* Research single-pair ethernet protocol for high bandwidth communication with attachment (6)
* Research and iterative design of RPM smoothing models - Linear, Quadratic, Sinusoidal (8)
* Setting up and toubleshooting the work environment for STM32 (3)
* Reading documentation for the STM32, A4988, and PS2 controller (1)
* Arduino PWM initialization for the motor driver proof of concept (3)
* Debugging and verifying waveforms using DAD2 board (3)
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
* Researched the designs of existing robotic arms to get a feel for how I should design ours (8)
* Designed prototypes for all parts of the arm including the base, first axis and second axis (14)
* Researched topics covering linear motion of robotic arms such as inverse kinematics (6)
* Printing and testing the parts to ensure proper function for all ranges of motion (9)
* Conducted tests for the friction fits we are using to secure parts to one another, this included printing the fittings with many different tolerances (12)
* Assembled the robot in Solidworks and physically by fitting all motors into the proper place and all the parts together (14)
