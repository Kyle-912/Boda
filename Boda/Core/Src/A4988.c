#include "A4988.h"

#define STEP_PULSE(steps, microsteps, rpm) (60.0*1000000L/steps/microsteps/rpm)

const uint8_t MS_TABLE[] = {0b000, 0b001, 0b010, 0b011, 0b111};
const uint8_t MS_TABLE_SIZE = sizeof(MS_TABLE);

void init_stepper(stepper *motor) {
    motor->steps = 0;
    motor->dir_state = 0;
    motor->enable_pin = 0;
    motor->step_count = 0;
    motor->steps_remaining = 0;
    motor->microsteps = 0;
    motor->rpm = 0;
    motor->step_pulse = 0;

    motor->dir_port = NULL;  // Setting to NULL as it's a pointer
    motor->dir_pin = 0;

    motor->step_port = NULL; // Setting to NULL as it's a pointer
    motor->step_pin = 0;

    motor->ms1_port = NULL;  // Setting to NULL as it's a pointer
    motor->ms1_pin = 0;

    motor->ms2_port = NULL;  // Setting to NULL as it's a pointer
    motor->ms2_pin = 0;

    motor->ms3_port = NULL;  // Setting to NULL as it's a pointer
    motor->ms3_pin = 0;

    motor->timer = NULL;
}

// Function implementations
void init_dir_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin) {
    motor->dir_port = port;
    motor->dir_pin = pin;
}

void init_step_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin) {
    motor->step_port = port;
    motor->step_pin = pin;
}

void init_ms1_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin) {
    motor->ms1_port = port;
    motor->ms1_pin = pin;
}

void init_ms2_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin) {
    motor->ms2_port = port;
    motor->ms2_pin = pin;
}

void init_ms3_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin) {
    motor->ms3_port = port;
    motor->ms3_pin = pin;
}

// set microsteps
void set_microsteps(stepper *motor, short microsteps) {
    motor->microsteps = microsteps;
}

// set timer
void set_microsteps(stepper *motor, TIM_HandleTypeDef * timer) {
    motor->timer = timer;
    timer.pUserData = (void *)motor;
}

// set rpm
void set_microsteps(stepper *motor, float rpm) {
    motor->rpm = rpm;
}


uint8_t setMicrostep(stepper *motor) {
    int i = 0;
    while (i < MS_TABLE_SIZE){
        if (motor->microsteps & (1<<i)){
            uint8_t mask = MS_TABLE[i];
            // digitalWrite(ms3_pin, mask & 4);
            HAL_GPIO_WritePin(motor->ms3_port, motor->ms3_pin, mask & 4);
            // digitalWrite(ms2_pin, mask & 2);
            HAL_GPIO_WritePin(motor->ms2_port, motor->ms2_pin, mask & 2);
            // digitalWrite(ms1_pin, mask & 1);
            HAL_GPIO_WritePin(motor->ms1_port, motor->ms1_pin, mask & 1);
            return 0;
        }
        i++;
    }
    return -1;
}

long calcStepsForRotation(stepper *motor, long deg){
    return deg * motor->steps * (long)motor->microsteps / 360;
}

long calcStepsForRotation(stepper *motor, double deg){
    return deg * motor->steps * motor->microsteps / 360;
}


void move_stepper_deg(stepper *motor, double deg) {

    motor->steps_remaining = calcStepsForRotation(motor, deg);
    motor->step_pulse = STEP_PULSE(motor->steps, motor->microsteps, motor->rpm);
    motor->step_count = 0;

    // Set the new pulse
    motor->timer->Instance->ARR = motor->step_pulse;
    if (HAL_TIM_Base_Init(motor->timer) != HAL_OK) {
        // Handle potential error
    }

    // Set timer to 0
    __HAL_TIM_SET_COUNTER(motor->timer, 0);

    // Start timer
    HAL_TIM_Base_Start_IT(motor->timer);

    // 
    // while (motor->step_count < motor->steps_remaining)
    // {
    //     __HAL_TIM_SET_COUNTER(motor->timer, 0);
    // }



}
