#include "robot_arm.h"

void init_arm(arm *arm, float rpm_, stepper *motor1, stepper *motor2)
{
    arm->motors[0] = motor1;
    arm->motors[1] = motor2;
    arm->rpm = rpm_;

    // arm->motor1 = motor1;
    // arm->motor2 = motor2;
    arm->num_coords = 0;
    for (int i = 0; i < MAX_COORDINATES; i++)
    {
        arm->coordinates[i].m1 = 0;
        arm->coordinates[i].m2 = 0;
    }
}

// float longest_travel_2(float rpm, double deg1, double deg2)
// {
//     // float longest_time = 0;
//     // float i_time = 0;
//     // for (int i = 0; i < MAX_MOTORS; i++)
//     // {
//     //     i_time =
//     //     if ()
//     // }

//     float time1 = deg1 / (rpm * 360 / 60);
//     float time2 = deg2 / (rpm * 360 / 60);

//     if (time1 > time2)
//     {
//         return time1;
//     }
//     else
//     {
//         return time2;
//     }
// }

void adjust_motors(float rpm, arm *arm, uint8_t coord)
{
    // uint16_t deltas[MAX_MOTORS];
    // deltas[0] = arm->coordinates[coord].m1 - arm->motors[0]->step_count;
    // deltas[1] = arm->coordinates[coord].m2 - arm->motors[1]->step_count;
    asm("nop");
    int16_t delta_steps_1 = arm->coordinates[coord].m1 - arm->motors[0]->step_count;
    int16_t delta_steps_2 = arm->coordinates[coord].m2 - arm->motors[1]->step_count;

    int16_t abs_delta_1;
    int16_t abs_delta_2;

    abs_delta_1 = delta_steps_1;
    abs_delta_2 = delta_steps_2;

    if (abs_delta_1 < 0)
        abs_delta_1 = abs_delta_1 * -1;
    if (abs_delta_2 < 0)
        abs_delta_2 = abs_delta_2 * -1;

    float time1_top = (abs_delta_1 * 1.8);
    float time2_top = (abs_delta_2 * 1.8);

    float temp_rpm = arm->rpm;

    float time1_bottom = (rpm * 360 / 60);
    float time2_bottom = (rpm * 360 / 60);

    float time1 = time1_top / time1_bottom;
    float time2 = time2_top / time2_bottom;

    // float time1 = (abs_delta_1 * 1.8) / (rpm * 360 / 60);
    // float time2 = (abs_delta_2 * 1.8) / (rpm * 360 / 60);

    float longest_time;

    if (time1 > time2)
        longest_time = time1;
    else
        longest_time = time2;

    float rpm_bottom = (longest_time * 360 / 60);

    float m1_rpm = time1_top / rpm_bottom;
    float m2_rpm = time2_top / rpm_bottom;

    arm->motors[0]->rpm = m1_rpm;
    arm->motors[1]->rpm = m2_rpm;

    move_stepper_steps(arm->motors[0], delta_steps_1, m1_rpm);
    move_stepper_steps(arm->motors[1], delta_steps_2, m2_rpm);
    // for (int i = 0; i < MAX_MOTORS; i++)
    // {
    //     move_stepper_steps(arm->motors[i], deltas[i], arm->motors[i]->rpm);
    // }
}

// float longest_travel_2(float rpm, arm* arm, uint8_t coord)
// {
//     // float longest_time = 0;
//     // float i_time = 0;
//     // for (int i = 0; i < MAX_MOTORS; i++)
//     // {
//     //     delta_steps = arm->coordinates[coord].m1 -
//     //     i_time = (arm->coordinates[i])
//     //     if ()
//     // }

//     int16_t delta_steps_1 = arm->coordinates[coord].m1 - arm->motors[0]->step_count;
//     int16_t delta_steps_2 = arm->coordinates[coord].m2 - arm->motors[1]->step_count;

//     // take absolute value
//     if (delta_steps_1 < 0)
//     {
//         delta_steps_1 = delta_steps_1 * -1;
//     }

//     if (delta_steps_2 < 0)
//     {
//         delta_steps_2 = delta_steps_2 * -1;
//     }

//     float time1 = (delta_steps_1 * 1.8) / (rpm * 360 / 60);
//     float time2 = (delta_steps_2 * 1.8) / (rpm * 360 / 60);

//     if (time1 > time2)
//     {
//         return time1;
//     }
//     else
//     {
//         return time2;
//     }
// }

// void adjust_rpm_2(float new_time, arm* arm, double deg1, double deg2)
// {
//     arm->motors[0]->rpm = deg1 / (new_time * 360 / 60);
//     arm->motors[1]->rpm = deg2 / (new_time * 360 / 60);
// }

void home(arm *arm)
{
    arm->motors[0]->step_count = 0;
    arm->motors[1]->step_count = 0;
}

void save_coordinate(arm *arm)
{
    if (arm->num_coords < MAX_COORDINATES)
    {
        arm->coordinates[arm->num_coords].m1 = arm->motors[0]->step_count;
        arm->coordinates[arm->num_coords].m2 = arm->motors[1]->step_count;
        arm->num_coords++;
    }
}

void set_coordinate(arm *arm, uint8_t coord_index, uint16_t step1, uint16_t step2)
{
    if (coord_index < MAX_COORDINATES)
    {
        arm->coordinates[coord_index].m1 = step1;
        arm->coordinates[coord_index].m2 = step2;
    }
}

void del_coordinate(arm *arm)
{
    if (arm->num_coords > 0)
    {
        arm->num_coords--;
        arm->coordinates[arm->num_coords].m1 = 0;
        arm->coordinates[arm->num_coords].m2 = 0;
    }
}

void move(arm *arm, uint8_t to_coord)
{
    adjust_motors(arm->rpm, arm, to_coord);

    // reset rpms
    // void set_rpm(stepper *motor, float rpm)
    for (int i = 0; i < MAX_MOTORS; i++)
    {
        set_rpm(arm->motors[i], arm->rpm);
    }
}