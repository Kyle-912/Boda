#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include "A4988.h"
#include <stdint.h>

#define MAX_COORDINATES 10
#define MAX_MOTORS 2

typedef struct
{
    uint16_t m1;
    uint16_t m2;
} Coordinate2;

// Define the robot arm struct
typedef struct
{
    stepper *motors[MAX_MOTORS];
    // stepper* motor1;
    // stepper* motor2;
    uint8_t num_coords;
    float rpm;
    Coordinate2 coordinates[MAX_COORDINATES];
} arm;

// Initializes the arm with given parameters.
void init_arm(arm *arm, float rpm_, stepper *motor1, stepper *motor2);

// Adjusts motor positions based on the arm configuration and a specific coordinate.
void adjust_motors(float rpm, arm *arm, uint8_t coord);

// Computes the longest travel time for a given rpm and arm configuration to reach a specified coordinate.
float longest_travel_2(float rpm, arm *arm, uint8_t coord);

// Sets the arm's position to its home position.
void home(arm *arm);

// Saves the current coordinate of the arm.
void save_coordinate(arm *arm);

// Sets a coordinate at a specified index.
void set_coordinate(arm *arm, uint8_t coord_index, uint16_t step1, uint16_t step2);

// Deletes the last saved coordinate of the arm.
void del_coordinate(arm *arm);

// Moves the arm to a specified coordinate.
void move(arm *arm, uint8_t to_coord);

#endif // ROBOT_ARM