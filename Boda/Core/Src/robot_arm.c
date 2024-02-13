#include "robot_arm.h"
#include <math.h>
#include <stdarg.h>  // Include this for variable length arguments

// void init_arm_2(arm* arm, float rpm_, stepper *motor1, stepper *motor2)
// {
//     arm->motors[0] = motor1;
//     arm->motors[1] = motor2;
//     arm->rpm = rpm_;
//     arm->num_motors = 2;


//     arm->num_coords = 0;
//     for (int i = 0; i < MAX_COORDINATES; i++) {
//         for (int j = 0; j < MAX_MOTORS; j++) {
//             arm->coordinates[i].steps[j] = 0;
//     }
// }
// }

// void init_arm_3(arm* arm, float rpm_, stepper *motor1, stepper *motor2, stepper *motor3)
// {
//     // Initialize motor pointers
//     arm->motors[0] = motor1;
//     arm->motors[1] = motor2;
//     arm->motors[3] = motor3;
//     arm->num_motors = 3;

//     // Initalize arm rpm (for now might change later)
//     arm->rpm = rpm_;

//     // Initialize coordinates
//     arm->num_coords = 0;
//     for (int i = 0; i < MAX_COORDINATES; i++) {
//         for (int j = 0; j < MAX_MOTORS; j++) {
//             arm->coordinates[i].steps[j] = 0;
//         }
//     }
// }

// void init_arm_4(arm* arm, float rpm_, stepper *motor1, stepper *motor2, stepper *motor3, stepper *motor4)
// {
//     // Initialize motor pointers
//     arm->motors[0] = motor1;
//     arm->motors[1] = motor2;
//     arm->motors[3] = motor3;
//     arm->motors[4] = motor4;
//     arm->num_motors = 4;

//     // Initalize arm rpm (for now might change later)
//     arm->rpm = rpm_;

//     // Initialize coordinates
//     arm->num_coords = 0;
//     for (int i = 0; i < MAX_COORDINATES; i++) {
//         for (int j = 0; j < MAX_MOTORS; j++) {
//             arm->coordinates[i].steps[j] = 0;
//         }
//     }
// }

void init_arm(arm* arm, float rpm_, int num_motors, ...) {
    va_list args;
    va_start(args, num_motors); // Initialize the argument list

    arm->num_motors = num_motors; // Set the number of motors
    arm->rpm = rpm_; // Initialize RPM
    arm->num_coords = 0; // Initialize the number of coordinates
    
    // Initialize motor pointers
    for (int i = 0; i < num_motors; i++) {
        arm->motors[i] = va_arg(args, stepper*);
    }

    va_end(args); // Clean up the argument list

    // Initialize coordinates to zero
    for (int i = 0; i < MAX_COORDINATES; i++) {
        for (int j = 0; j < MAX_MOTORS; j++) {
            arm->coordinates[i].steps[j] = 0;
        }
    }
}


// void adjust_motors(float rpm, arm* arm, uint8_t coord)
// {
//     // uint16_t deltas[MAX_MOTORS];
//     // deltas[0] = arm->coordinates[coord].m1 - arm->motors[0]->step_count;
//     // deltas[1] = arm->coordinates[coord].m2 - arm->motors[1]->step_count;
//     asm ("nop");
//     int16_t delta_steps_1 = arm->coordinates[coord].m1 - arm->motors[0]->step_count;
//     int16_t delta_steps_2 = arm->coordinates[coord].m2 - arm->motors[1]->step_count;

//     int16_t abs_delta_1;
//     int16_t abs_delta_2;

//     abs_delta_1 = delta_steps_1;
//     abs_delta_2 = delta_steps_2;

//     if (abs_delta_1 < 0)
//         abs_delta_1 = abs_delta_1 * -1;
//     if (abs_delta_2 < 0)
//         abs_delta_2 = abs_delta_2 * -1;

//     float time1_top = (abs_delta_1 * 1.8);
//     float time2_top = (abs_delta_2 * 1.8);

//     float temp_rpm = arm->rpm;

//     float time1_bottom = (rpm * 360 / 60);
//     float time2_bottom = (rpm * 360 / 60);

//     float time1 = time1_top / time1_bottom;
//     float time2 = time2_top / time2_bottom;

//     // float time1 = (abs_delta_1 * 1.8) / (rpm * 360 / 60);
//     // float time2 = (abs_delta_2 * 1.8) / (rpm * 360 / 60);

//     float longest_time;

//     if (time1 > time2)
//         longest_time = time1;
//     else 
//         longest_time = time2;

//     float rpm_bottom = (longest_time * 360 / 60);

//     float m1_rpm = time1_top / rpm_bottom;
//     float m2_rpm = time2_top / rpm_bottom;

//     arm->motors[0]->rpm = m1_rpm;
//     arm->motors[1]->rpm = m2_rpm;

//     move_stepper_steps(arm->motors[0], delta_steps_1, m1_rpm);
//     move_stepper_steps(arm->motors[1], delta_steps_2, m2_rpm);
//     // for (int i = 0; i < MAX_MOTORS; i++)
//     // {
//     //     move_stepper_steps(arm->motors[i], deltas[i], arm->motors[i]->rpm);
//     // }
// }

void adjust_motors_sinusoidal_gen(float rpm, arm* arm, uint8_t coord, double rise_time) {
    // Calculate the delta steps for each motor

    // -- POSSIBLE generalized -----------------------------
    int16_t longest_delta = 0;
    int16_t delta_steps[MAX_MOTORS] = {0};

    // For Each Motor
    for (int i = 0; i < arm->num_motors; i++) {
        // Calculate Delta Steps for motor
        delta_steps[i] = arm->coordinates[coord].steps[i] - arm->motors[i]->step_count;

        // Update Longest Step Delta, considering the absolute value of delta_steps
        int16_t abs_delta = abs(delta_steps[i]); // Get the absolute value of the delta
        longest_delta = (abs_delta > longest_delta) ? abs_delta : longest_delta;
    }

    // Calculate Longest Time Using the Longest Step Delta
    float longest_time = (longest_delta * 1.8) / (rpm * 6);

    // For Each Motor
    for (int i = 0; i < arm->num_motors; i++)
    {
        // Calculate Motor Specific RPM
        double rpm = (fabs(delta_steps[i]) * 1.8) / (longest_time * 6);

        // Initialize Necessary Variables For Movement
        init_sinusoidal_vars(abs(delta_steps[i]), rpm * 1.0f, rise_time, arm->motors[i]);

        // Set Motor Dir_State
        arm->motors[i]->dir_state = (delta_steps[i] < 0) ? 1 : 0;

        // Set Motor Steps_Remaining
        arm->motors[i]->steps_remaining = abs(delta_steps[i]);

        // Set Motor Sleep to HIGH
        HAL_GPIO_WritePin(arm->motors[i]->sleep_port, arm->motors[i]->sleep_pin, SET);
    }   

    // Start timer for each motor
    for (int i = 0; i < arm->num_motors; i++)
    {
        HAL_TIM_Base_Start_IT(arm->motors[i]->timer);
    }
}

// void adjust_motors_sinusoidal(float rpm, arm* arm, uint8_t coord, double rise_time) {

//     int16_t delta_steps_1 = arm->coordinates[coord].m1 - arm->motors[0]->step_count;
//     int16_t delta_steps_2 = arm->coordinates[coord].m2 - arm->motors[1]->step_count;

//     // Calculate movement times and synchronize RPMs
//     float longest_time = fmax(
//         fabs(delta_steps_1 * 1.8) / (rpm * 6),
//         fabs(delta_steps_2 * 1.8) / (rpm * 6)
//     );

//     float m1_rpm = (fabs(delta_steps_1) * 1.8) / (longest_time * 6);
//     float m2_rpm = (fabs(delta_steps_2) * 1.8) / (longest_time * 6);

//     double temp_rpm_1 = m1_rpm;
//     double temp_rpm_2 = m2_rpm;

//     // Initialize sinusoidal movement for each motor

//     // if (arm->motors[0]->sinusoidal_ramp)
//     // {
//     //     init_sinusoidal_vars(abs(delta_steps_1), temp_rpm_1 * 1.0f, rise_time, arm->motors[0]);
//     // }
//     // else
//     // {

//     // }

//     // if (arm->motors[1]->sinusoidal_ramp)
//     // {
//     //     init_sinusoidal_vars(abs(delta_steps_2), temp_rpm_2 * 1.0f, rise_time, arm->motors[1]);
//     // }
//     // else
//     // {

//     // }

//     init_sinusoidal_vars(abs(delta_steps_1), temp_rpm_1 * 1.0f, rise_time, arm->motors[0]);
//     init_sinusoidal_vars(abs(delta_steps_2), temp_rpm_2 * 1.0f, rise_time, arm->motors[1]);

//     // Set the direction and steps remaining for each motor
//     arm->motors[0]->dir_state = (delta_steps_1 < 0) ? 1 : 0;
//     arm->motors[1]->dir_state = (delta_steps_2 < 0) ? 1 : 0;
//     arm->motors[0]->steps_remaining = abs(delta_steps_1);
//     arm->motors[1]->steps_remaining = abs(delta_steps_2);

//     // Set sleep output HIGH
//     HAL_GPIO_WritePin(arm->motors[0]->sleep_port, arm->motors[0]->sleep_pin, SET);
//     HAL_GPIO_WritePin(arm->motors[1]->sleep_port, arm->motors[1]->sleep_pin, SET);

//     // Output DIR state
//     HAL_GPIO_WritePin(arm->motors[0]->dir_port, arm->motors[0]->dir_pin, arm->motors[0]->dir_state);
//     HAL_GPIO_WritePin(arm->motors[1]->dir_port, arm->motors[1]->dir_pin, arm->motors[1]->dir_state);

//     // Start the timer for each motor
//     HAL_TIM_Base_Start_IT(arm->motors[0]->timer);
//     HAL_TIM_Base_Start_IT(arm->motors[1]->timer);
// }


void set_arm_rpm(arm* arm, float rpm_)
{
    arm->rpm = rpm_;
}


void set_coordinate(arm* arm, uint8_t coord_index, int num_motors, ...) {
    if (coord_index < MAX_COORDINATES) {
        va_list args;
        va_start(args, num_motors); // Initialize the argument list
        
        for (int i = 0; i < num_motors; i++) {
            // Read the next argument as an int, then cast it to uint16_t
            arm->coordinates[coord_index].steps[i] = (uint16_t)va_arg(args, int);
        }

        va_end(args); // Clean up the argument list
    }
}


void del_coordinate(arm* arm) {
    if (arm->num_coords > 0) {
        arm->num_coords--;
        for (int i = 0; i < MAX_MOTORS; i++) {
            arm->coordinates[arm->num_coords].steps[i] = 0;
        }
    }
}



void home(arm* arm) {
    for (int i = 0; i < arm->num_motors; i++) {
        arm->motors[i]->step_count = 0;
    }
}

void save_coordinate(arm* arm) {
    if (arm->num_coords < MAX_COORDINATES) {
        for (int i = 0; i < arm->num_motors; i++) {
            arm->coordinates[arm->num_coords].steps[i] = arm->motors[i]->step_count;
        }
        arm->num_coords++;
    }
}



void move(arm* arm, uint8_t to_coord)
{   
    adjust_motors_sinusoidal_gen(arm->rpm, arm, to_coord, 4.0);
    // adjust_motors_sinusoidal(arm->rpm, arm, to_coord, 4.0);

    // adjust_motors(arm->rpm, arm, to_coord);

    // reset rpms
    // void set_rpm(stepper *motor, float rpm)
    for (int i = 0; i < arm->num_motors; i++)
    {
        set_rpm(arm->motors[i], arm->rpm);
    }
}