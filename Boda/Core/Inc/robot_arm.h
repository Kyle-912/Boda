#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include "A4988.h"
#include <stdint.h>
#include <stdarg.h> // For variable length arguments

#define MAX_COORDINATES 10
#define MAX_MOTORS 4

typedef struct {
    uint16_t steps[MAX_MOTORS]; // Array to hold steps for each motor
} Coordinate;

typedef struct {
    stepper* motors[MAX_MOTORS]; // Array of pointers to stepper structs
    uint8_t num_coords; // Number of coordinates
    uint8_t num_motors; // Number of motors
    float rpm; // Rotations per minute
    bool is_jogging; // Boolean for Arm Jogging
    Coordinate coordinates[MAX_COORDINATES]; // Array of coordinates
} arm;

// Function declarations
void init_arm(arm* arm, float rpm_, int num_motors, ...);
void adjust_motors(float rpm, arm* arm, uint8_t coord);
void adjust_motors_sinusoidal_gen(float rpm, arm* arm, uint8_t coord, double rise_time);
void adjust_motors_sinusoidal(float rpm, arm* arm, uint8_t coord, double rise_time);
float longest_travel_2(float rpm, arm* arm, uint8_t coord);
void move(arm* arm, uint8_t to_coord);

// Sets the arm's position to its home position.
void home(arm* arm);

// Sets rpm for arm
void set_arm_rpm(arm* arm, float rpm_);

// Saves the current coordinate of the arm.
void save_coordinate(arm* arm);

// Sets a coordinate at a specified index for a variable number of motors.
void set_coordinate(arm* arm, uint8_t coord_index, int num_motors, ...);

// Deletes the last saved coordinate of the arm.
void del_coordinate(arm* arm);

// Moves the arm to a specified coordinate with arm->rpm
void move(arm* arm, uint8_t to_coord);
// Moves the arm to a specified coordinate with specified rpm
void move_rpm(arm* arm, uint8_t to_coord, float rpm);

// callback for timer period handled by arm driver
void callback_pulse(arm* arm, TIM_HandleTypeDef *htim);

// Assuming these functions are defined elsewhere in your project
// void init_sinusoidal_vars(int16_t steps, float rpm, double rise_time, stepper* motor);
// void HAL_GPIO_WritePin(void* port, int pin, int state);
// void HAL_TIM_Base_Start_IT(void* timer);
// void HAL_GPIO_WritePin(void* port, int pin, int state);
// void set_rpm(stepper *motor, float rpm);
// void move_stepper_steps(stepper *motor, int16_t steps, float rpm);

#endif // ROBOT_ARM_H
