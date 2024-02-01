Overview
For our alpha build of the Boda we created a main pcb that serves to integrate our different components and provides a platform for extended development, alongside this main control board we have designed several physical components that will serve as our testing platform for the kinematics of the robotic arm. Lastly we have started to design and implement a new control system that gives us more flexibility than our current gamepad. 

Repos
Attachment 1 (Servo and Finger Libraries Included):
https://github.com/AlexsIntroductions/Boda-Attachment-GRIP
Updated Arm, Motor, and PS2 Controller Drivers:
https://github.com/Kyle-912/Boda/tree/working-sean
Controller App:
https://github.com/Kyle-912/Boda/tree/feature/frontend 
Altium Files:
https://smcneil-smcneil.365.altium.com/designs/D4B4EB05-56A4-41AF-8AA4-D2E3C336BF63


Usability	2
Interface	2
Navigation	3
Perception	4
Responsiveness	4
Build Quality	6
Robustness	6
Consistency	9
Aesthetic Rigor	10
Vertical Features	11
External Interface	11
Persistent State	11
Internal Systems	11
Attachment 1 First Functional Test	13
Ramp Function Testing	14

Usability
Interface

Over the last week, we discussed how we interact with the arm and it was proposed to have a wireless controller rather than a hardwired one. This gives the operator the opportunity to use the robotic arm without being directly next to the arm or any hazards in the environment which the arm is in. 
To implement this we decided on bluetooth because of its ease of use and also provides a latency that is minimal in the range of 200 ms. 
Our initial GUI design was to directly copy the PS2 controller into an application and have the controller app serve as a replacement to the physical controller. However, as we started to develop use cases and the human machine interaction of our system, we decided that having a controller app would allow us to provide custom features to the users of the Boda arm and also make the robot more accessible. 
One of these custom features is to have a specific tab for the attachments. This allows us to provide multiple different interfaces for each attachment. Because the attachments are developed independently of the interface, we will create different attachment controllers and the attachment developer is able to decide which interface they would like for their attachment to use. When the attachment is connected the app will load the correct interface for the connected attachment.
Here is our current GUI Draft:




For our current iteration, we have created a library for the STM32 that allows for PS2 controller communication, and many of the features of the library assume it is for this project specifically. For example, the controller was intended to be hard wired into the system, thus the library always assumes that a controller is connected. Some pins are also hard coded and not modular. Since we are phasing out the physical controller for a more accessible Bluetooth controller, we are not planning on updating the library much further other than for testing purposes. Until then we will be using the PS2 controller for testing and as such have a control scheme based around its design. The two joysticks are used to control the limb’s arm and the shoulder and face-shape buttons are used for the attachments. We believe that these controls are easy to figure out by picking up the controller and pressing inputs to figure out what they do, instructions can be created but we are planning to phase out the PS2 controller. Future control schemes will be developed for phones and can have tooltips showing and explaining buttons.

Navigation

	Currently, the movement of the robot arm is completely dependent on the PS2 input. The data is parsed from the PS2 drivers and commands are sent to the robot arm using the arm drivers. The drivers allow for points to be added and traversed, along with free jogging at a variable RPM. Points can be easily added to a list with one function call that takes in the current position of the robot arm. Points can be traversed with another function that requires a desired destination point. Jogging involves continuously moving the respectful motor at an RPM determined by the joystick’s distance from the center - ranging from min_rpm to max_rpm.

	These controls can easily be mapped to a bluetooth adaptation. The function calls are not dependent on the PS2 drivers and can be called freely in the main program. The only changes will be present within the data parsing function - although it will have a similar algorithm. The information being sent from the bluetooth will be in the form of characters over UART as opposed to SPI. The characters will not be constrained by the SP2 protocol, so we can make the commands more recognizable. There will be RPM and motor commands, along with attachment-specific commands.

	Currently, there are no notable bugs. Our biggest limitation was conforming to the PS2 protocol, but persistent testing and debugging has allowed us to quickly and reliably read data. The only downside of using a joystick as opposed to tailored buttons is with accidental input. Moving the direction of just one motor is difficult because any drift along that joystick axis will move the motor on the perpendicular axis.
We completed our first test of our grip attachment using our servo library. Since all our attachments are being made with an rp2040 and the C SDK, for any attachments we make we are making libraries for some of the components we use. By doing this we can increase the amount of resources we can supply to other users but also create a better base repo for developers to start from by seeing what features people would need for themselves.
In terms of functionality, the grip uses four servos structured as two ‘fingers’ (two servos each). The fingers operate symmetrically, where they will open and close at the same time and same rate with one button per action. Each finger has one servo labeled as the top, referring to the furthest joint, and the other labeled bottom, referring to the closest joint. The fingers’ top and bottom can be opened or closed simultaneously or each joint can be opened or closed individually to give the attachment better grip on any object.
There are only minor bugs in the current version of the grip attachment, mostly regarding how I implemented the servo library. Currently I have plans to expand the library and there are a few functions I didn't need that I had not completed. In order to have a more stable library, error checking for misuse of any functions needs to be done and there are several assumptions made when I wrote it. One being in the mapping function, where I assume that the range to map to the rp2040’s max PWM value is going to be 16 bits or smaller.

Perception
Our current control scheme is based on the PS2 controller but we are planning to migrate this to a Bluetooth phone app. The decision was made since it was getting difficult to test using only one controller and since we would have to break apart the port of another one to get at the pins to be able to have a second was too unintuitive. This decision to change to an app controller allows us to make more intuitive and complex controls for the limb and lets users create controllers specifically for the open-sourced attachment they are developing. 

Responsiveness

	The drivers to control the motors rely on dedicated timers to designate each pulse. This means that each motor is dependent on its own timer - 4 timers total. This was initially viewed as a downside, but has not proved any major issues. There appears to be smooth movement at a reasonable RPM, but there are occasional small kicks that could be the fault of slightly inaccurate duty durations. Inside of an RTOS system, everything appears to function as expected with 2 motors, with testing on 4 motors being conducted once the PCB arrives. 

	There is virtually no delay between when a function is called to move a motor and when that motor moves. The process involves a few calculations to determine the number of steps to travel and to calculate the pulse width given an RPM. After this, the timer starts and the motor immediately begins to move.
	
	Since all of the motors use the same timer period callback, there was concern about the addition of ramp up and down periods. The pulse lengths for the periods were initially calculated in real-time, but have been converted to values within a personal ramp array to avoid unnecessary callback times. This should hopefully allow for easy pseudo-simultaneous use of the same function.
The controller operates within its own thread of our freeRTOS in the main limb. Every 25 us, an update function is called to receive the current states of the PS2 controllers inputs, namely the directional pad, four front facing buttons, three center buttons, the four shoulder buttons, and the positions of the four potentiometers for the joysticks. The 25 us delay is by design as the PS2 controller requires at least a 14 us delay between transmissions and through testing we found 25 us to be responsive. These states are stored in a global array locked behind a mutex that each thread needs to acquire before checking/altering any data. This is to ensure that there are no read/write conflicts when we interpret inputs and use the data.
Currently our attachment and limb communicate through SPI. For controls, every 1 ms an SPI transmission is sent to the attachment containing the current states of the buttons being pressed. For the attachments, the four shape buttons and the four shoulder buttons’ states are sent. This triggers an interrupt in the attachment and it will SPI receive the byte containing the 8 button states and store it to global memory to be referred to by the rest of the system. The Attachment itself does not run RTOS and instead uses a superloop with a delay between each loop of 10 ms. This is to mimic how our current limb handles inputs with a 10ms delay. While the attachment receives input data via an interrupt every 1ms, it will only act on an input every 10 ms to allow better user response to inputs.
	To reduce latency between the user input and the arm’s response, we plan on limiting the amount of data transmitted between the controller app and the arm. Furthermore, one of our stretch goals is to have the capability of logging data, at this stage the transmission protocol will prioritize user input over sending logging data. To ensure our logging data is as complete as possible a buffer will hold the data until it is ready to be sent. With these measures, we expect to have a latency that is around the typical human response time and therefore smooth operation.

	One of the main issues we faced with our prototype was that when we wanted the robot to move slowly, the stoppers would have a hard time making this movement smoothly, additionally we did not have enough torque to lift the weight we wanted to. The solution to both of these problems was to add a gearbox to our joint designs. Adding a gearbox improves the resolution and precision of the robotic arm's movements. By reducing the speed and increasing torque, finer adjustments can be made, leading to more accurate positioning of the arm. This is particularly important in applications where precise control is required. 



Build Quality
Robustness

	The stepper motor library needs to output a consistent, specific output to the stepper motor to allow for seamless movement without jittering or jamming. 

After creating the initial stepper motor driver, jittering was observed at lower motor RPM. After testing, It was determined that the cutoff before jittering occurred around 30 RPM. Moving forward, we will keep this value as the minimum RPM because we want to avoid jittering or motor jamming. This is reasonable because the new 80:1 gearbox allows for a faster minimum RPM while retaining precision.
Testing was conducted at 30 V motor supply and found a 113 mA current draw at 400 RPM and 72 mA draw at idle. Additional sustained testing between points uncovered:
·         	Intermittent small “kicking” in the motor throughout travel at higher RPM.
·      	Jittering at lower RPMs (also present during ramp up and ramp down sequence).
·     	 Small chance for the motor to lock during travel. 
		- No root cause determined. 
		- Appears random.  
		- Drop from 113 mA to 83 mA
·          	Thermal issues are resolved by placing the motor on a metal plate.

Another notable change was the addition of RPM ramp zones. At first, the stepper motor instantly jumped to the desired RPM from rest, creating a sharp jerk. Ramping up the RPM to the desired RPM and ramping down the RPM back to rest allows for smoother travel. This ramp function required reverse engineering the desired acceleration function and optimizing jerk.


Jerk (j) is the rate of change of the acceleration. It is an important consideration because it is responsible for the smooth movement of the load. It is modeled by  and reinforces the idea of using a ramp function that agrees with small jerk values. 

There are a few notable features of the ramp function and reasons why it was chosen.

1)      The ramp in RPM from 0-2π perfectly goes into the constant RPM region for 2π-4π.
	- This is reflected in the acceleration having a smooth transition to 0 at 2π
2)      The function has a smoother transition compared to the linear ramp function.
	 – The jerk would be very large at the transition points due to a rapid change in acceleration from constant to zero and vice versa.
Overall, this function contains the benefits of a linear acceleration (low jerk) while eliminating the jump in jerk observed at the transition points of a linear RPM ramp.


Additionally, tweaks made to the robot arm driver responsible for the simultaneous movement of all stepper motors. The timing algorithm present on the arm driver allows for the synchronized start and stop of all the motors contributing to moving the arm, but it required testing to refine. Initially, one motor would reach its destination position before another, requiring an RPM adjustment to make the motor with less travel distance slower. The RPM function was also faulty - referencing a wrong RPM value and skewing results. This involved the use of an oscilloscope and debugger to identify and fix.

The PCB also received several revisions before being sent for fabrication. The first version was large and had a lot of wasted space, making it more than double the price of the final version. After realizing that PCBs under 4x4 inches were significantly cheaper, the board was optimized for space. These changes caused conflicts with clearance and trace integrity, but after choosing to also place components on the backside of the board, the parts were able to fit. The final version includes all of the necessary parts for now and has enough room for possible additions in later versions.


Figure 1: Top View of Boda V1 PCB

Figure 2: Bottom View of Boda V1 PCB 


Features Present on PCB
Top
Bottom
STM32 Connector
Motor Driver Connector (x4)
Motor Power Connector (30V, 0–1 A)
H-F Decoupling Capacitor (100 nF) (x4)
Bulk Decoupling Capacitor (50V 10uF)
Optional bottom Bluetooth Connector
SPI2, SPI3 Connectors
 
M1 – M4 Con
 
Bluetooth connector
 



	Currently our RTOS implementation has 5 threads to run: PS2 Controller update, stepper motor 1, stepper motor 2, stepper motor 3, and Attachment Input. The PS2 Controller Update thread runs every 25 us at the highest priority compared to other threads. This is to make sure any inputs we process are the most recent ones. To store inputs, a 7 byte array stores the values of all the PS2 buttons and joystick positions. Each of the stepper motors runs on a thread that is called every 10 ms which was chosen through testing. It gave nice feedback to make sure actions were not happening before the user could interpret what occurred. These all run at a low. The Attachment communication thread runs every 1 ms and sends a byte containing the states of the 8 buttons assigned to control it. This thread runs at a medium priority.

The array containing all input data is shared between each thread and is locked behind a mutex which each thread is required to acquire before executing. This results in a round-robin-esque queue that forms in reference to the stepper motor threads which are being called repeatedly and being interrupted by the higher priority PS2 and attachment threads. 

	One of the primary functions of the gearbox is to amplify torque. In the context of a robotic arm, this means that the motor's rotational force can be translated into a higher torque at the arm's joints. This is crucial for lifting heavier loads or overcoming resistance in the system. The driving factor behind implementing this feature was to use weaker motors that will draw less power and have a smaller profile keeping the size and weight of the device down. In addition, when we were using the motor to directly drive the limb, making slow, subtle movements was difficult but the gearbox will allow us to avoid that.

	To enable us to seat the STM board into the PCB all the pin assignments needed to be remapped since many of them were on the inner female headers instead of the outer male headers. Pins were grouped according to system/function and were spaced out to allow easier PCB traces. A reference document with all pin assignments and all the enabled systems and their settings was created.

Consistency

	The motor driver controls the start, stop, and RPM of an individual stepper motor. The robot arm driver uses the drivers for each motor to control the position and motion of the load as a whole. Each stepper motor plays a role in the overall movement of the load. This independent component is calculated based on the deltas between two points, and is used to correctly adjust each motor’s performance to move to the correct position at the correct speed. 

Stepper motor RPM is a vital component that ensures that all of the motors start and stop at the same time. In one full rotation of the stepper motor, there are 200 steps. Given two positions, there will be a delta in the number of steps. This delta is the number of steps the stepper motor will need to travel to reach the desired end location. This number can also vary between each motor that is responsible for moving the load. To accommodate for this and ensure that the motors reach the final position uniformly, the RPM of the motors is proportional to the number of steps needed to travel. If one motor has more travel steps, then it will also have a higher RPM to accomplish those steps quicker and stop with the other motors.

There are several physical, electrical components involved in this process. Each degree of freedom has a motor and each motor has an A4988 motor driver. These drivers are powered by a 30V supply, and each component is connected with wires to the STM32. To confirm the proper configuration of everything, a PCB was created. This PCB houses all the A4988 stepper motor drivers, the power supply lines, capacitor, and I/O pinout.

	Each attachment will contain a rp2040 pico board which can be spoken to over SPI and with two GPIO signals. Using a STM board for the attachments in addition to the arm was deemed to be overpowered and programming multiple STM boards was leading to developmental confusion and differing behavior depending on the order in which the boards were programmed. All of us have a pico board so we will be able to develop multiple attachments simultaneously without having to share hardware.

Aesthetic Rigor

	The first prototype of our arm was entirely 3d printed, the process for this design included measuring and modeling the motors in SolidWorks, then the base and the first section of the limb connecting the shoulder and elbow joints were modeled and printed. The initial prints were used to test what the best tolerances for holding the motors in place would be.

	For the prototype design no large forces were applied to any components of the arm. To ensure that our arm can endure greater forces we decided to reduce the number of 3d printed parts as these can be fragile. We will now have a wooden base and the limb will be made up of mostly PVC parts with select sections of the joints being 3d printed to hold the motors in place.

	After deciding to use a gearbox and doing a bit of research, the three main options were to use a harmonic drive, a cycloidal drive or a planetary gearbox. Although the harmonic and cycloidal gearboxes had some features that made them desirable, we ended up going with the planetary gearbox because the design and materials were more accessible for a project of this scale. By buying a few bearings and M3 fasteners, we were able to build a planetary gearbox that provides our motors with an 80:1 gear reduction, greatly increasing the torque and precision of our robotic arm.

	For the GUI, we decided on moving away from joysticks and instead providing buttons that control the direction of the arm’s motors along with a set of sliders that control the speed at which each respective motor moves. In the bottom ribbon of our app, we will have tabs that allow the operator to switch between the main arm, the attachment, and a toggle for teaching the robot movements. Currently, we have a preliminary design for the main controller and are working to design different configurations for the attachments.

Vertical Features
External Interface

	Our grip attachment is our first demonstrable for our interchangeable attachment use case. We have been able to learn a lot about how we want to set up our attachment systems and for our port when we begin to develop our second attachment and the software/hardware around interchanging them.

There are plenty of robotic arms in the market nowadays, the main thing that sets our apart is the ability to swap out the attachment at the wrist of the arm, giving the robot a huge range of use cases. The idea is to have a base robotic arm that will be compatible with any attachment that follows the specified guidelines for weight, communication protocol etc. This will mean that anyone can design the attachment they need for their own specialized use case. Below are some examples of attachments that we plan to design.
An attachment will be designed which uses a photoresistor to detect the brightest direction and point the arm to face that direction, mimicking the behavior of a sunflower. This will demonstrate the arms ability to use data received from an attachment to motivate movements of the arm itself.
Another attachment we have designed is a claw or hand style attachment that can pick up small objects such as a solo cup. This attachment will allow us to test the strength of our arm and determine how much it can lift.
	Having a hardwired controller implies that the use cases are limited to the physical controller’s design and the controller not being damaged. To overcome both of these issues, we decided to implement a controller application that can be used on ios and android devices that have bluetooth. Having a controller app allows to have software driven controls that can change as the robot matures and new attachments are made. Furthermore bluetooth is widely implemented in most mobile devices and allows the controlling device to be changed on demand. 

Persistent State

The main data which is shared and sent throughout our systems is the controller/input data. In the limb, the data is managed through mutexes to make sure no two threads are manipulating the data at the same time. Our threads are set up to make sure the input data is current for our motor control, and that the attachments have a consistent access to their designated inputs for control. Responsiveness is a main goal for this project and we believe our current system provides a responsive experience. 

Internal Systems

	Our internal systems refer to our Limb and Attachment. Our limb operates on an RTOS which has been described in detail above. Our resources are managed in a timely manner where all threads and external systems are able to receive their data before they are called to execute their next action. Our attachment operates in a superloop but receives data through interrupt, allowing the most current input data to be executed and in a way that allows the user to experience the attachment’s action as the input is pressed.
Attachment 1 First Functional Test





Ramp Function Testing

RAMP UP SEQUENCEFigure 1: Sinusoidal Ramp Up from 40 to 400 RPM

Region 1:
Both motors start with the minimum RPM of 40.

Region 2:

Motors begin to increase RPM based on the sinusoidal ramp function.

Motor on Channel 1 appears to be increasing faster to reach a higher desired motor RPM.

Region 3:
 
Both motors reach their desired RPM and transition to the constant RPM region.


RAMP DOWN SEQUENCE
Figure 2: Sinusoidal Ramp Down from 400 to 40 RPM

Region 1:
Motors are still at their peak RPM.

 M1 Period: 0.61 ms
M2 Period: 1.3 ms

Region 2:

 Both motors begin to decrease in RPM based on a sinusoidal ramp.
M1 Period: 0.58 ms
M2 Period: 3.8 ms

Region 3:
Both motors are decreasing their RPM based on a sinusoidal function.

Region 4:
Both motors end their ramp with the minimum RPM of 40.


