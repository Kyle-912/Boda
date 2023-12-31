#include "A4988.h"

// #define STEP_PULSE(steps, microsteps, rpm) (60.0 * 1000000L / steps / microsteps / rpm)

const uint8_t MS_TABLE[] = {0b000, 0b001, 0b010, 0b011, 0b111};
const uint8_t MS_TABLE_SIZE = sizeof(MS_TABLE);

void init_stepper(stepper *motor, short spr)
{
    motor->steps = spr;
    motor->dir_state = 0;
    motor->enable_pin = 0;
    motor->step_count = 0;
    motor->steps_remaining = 0;
    motor->microsteps = 1; // Default = 1 (full steps)
    motor->rpm = 0;
    motor->step_pulse = 0;
    motor->enable_microsteps = 1;

    motor->dir_port = NULL; // Default = Null (pointer)
    motor->dir_pin = 0;     // Default = 0 (uint16_t)

    motor->step_port = NULL; // Default = Null (pointer)
    motor->step_pin = 0;     // Default = 0 (uint16_t)

    motor->sleep_port = NULL; // Default = Null (pointer)
    motor->sleep_pin = 0;     // Default = 0 (uint16_t)

    motor->ms1_port = NULL; // Default = Null (pointer)
    motor->ms1_pin = 0;     // Default = 0 (uint16_t)

    motor->ms2_port = NULL; // Default = Null (pointer)
    motor->ms2_pin = 0;     // Default = 0 (uint16_t)

    motor->ms3_port = NULL; // Default = Null (pointer)
    motor->ms3_pin = 0;     // Default = 0 (uint16_t)

    motor->timer = NULL; // Default = Null (pointer)
}

// Function implementations
void init_dir_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin)
{
    motor->dir_port = port;
    motor->dir_pin = pin;
}

void init_step_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin)
{
    motor->step_port = port;
    motor->step_pin = pin;
    HAL_GPIO_WritePin(port, pin, RESET);
}

void init_sleep_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin)
{
    motor->sleep_port = port;
    motor->sleep_pin = pin;
    // HAL_GPIO_WritePin(port, pin, RESET);
}

void init_ms1_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin)
{
    motor->ms1_port = port;
    motor->ms1_pin = pin;
}

void init_ms2_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin)
{
    motor->ms2_port = port;
    motor->ms2_pin = pin;
}

void init_ms3_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin)
{
    motor->ms3_port = port;
    motor->ms3_pin = pin;
}

// set microsteps
void set_microsteps(stepper *motor, short microsteps)
{
    motor->microsteps = microsteps;
}

// set timer
void set_timer(stepper *motor, TIM_HandleTypeDef *timer)
{
    motor->timer = timer;
}

// set rpm
void set_rpm(stepper *motor, float rpm)
{
    motor->rpm = rpm;
}

// set dir state (clockwise/counter-clockwise)
void set_dir_state(stepper *motor, short dir_state)
{
    motor->dir_state = dir_state;
}

// set dir state (clockwise/counter-clockwise)
void set_micro_en(stepper *motor, uint8_t micro_en)
{
    motor->enable_microsteps = micro_en;
}

// set dir state (clockwise/counter-clockwise)
// void home(stepper *motor) {
//     motor->step_count = 0;
// }

uint8_t setMicrostep(stepper *motor)
{
    int i = 0;
    while (i < MS_TABLE_SIZE)
    {
        if (motor->microsteps & (1 << i))
        {
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

// long calcStepsForRotation(stepper *motor, long deg){
//     return deg * motor->steps * (long)motor->microsteps / 360;
// }

long calcStepsForRotation(stepper *motor, double deg)
{
    return deg * motor->steps * motor->microsteps / 360;
}

void move_stepper_steps(stepper *motor, int16_t steps, float rpm)
{
    // if negative steps, set dir
    if (steps < 0)
    {
        motor->dir_state = 1;
        steps = steps * -1;
    }
    // set steps
    motor->steps_remaining = steps;
    motor->step_count = 0;

    // set rpm
    motor->rpm = rpm;
    motor->step_pulse = STEP_PULSE(motor->steps, motor->microsteps, rpm);

    // Set step output LOW
    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);

    // Set sleep output HIGH
    HAL_GPIO_WritePin(motor->sleep_port, motor->sleep_pin, SET);

    // Output DIR state
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->dir_state);

    // Set the new pulse
    motor->timer->Instance->ARR = motor->step_pulse;
    if (HAL_TIM_Base_Init(motor->timer) != HAL_OK)
    {
        // Handle potential error
    }

    // Set timer to 0
    __HAL_TIM_SET_COUNTER(motor->timer, 0);

    // Start timer
    HAL_TIM_Base_Start_IT(motor->timer);
}

void move_stepper_deg(stepper *motor, double deg)
{

    if (motor->enable_microsteps)
    {
        if (setMicrostep(motor) == -1)
        {
            // Error in setting microsteps
            return;
        }
    }

    // First attempt at opposite rotation
    if (deg < 0)
    {
        deg = deg * -1;
        motor->dir_state = 1;
    }

    // Set step output LOW
    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);

    // Set sleep output HIGH
    HAL_GPIO_WritePin(motor->sleep_port, motor->sleep_pin, SET);

    motor->steps_remaining = calcStepsForRotation(motor, deg);
    motor->step_pulse = STEP_PULSE(motor->steps, motor->microsteps, motor->rpm);
    motor->step_count = 0;

    // Output DIR state
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->dir_state);

    // Set the new pulse
    motor->timer->Instance->ARR = motor->step_pulse;
    if (HAL_TIM_Base_Init(motor->timer) != HAL_OK)
    {
        // Handle potential error
    }

    // Set timer to 0
    __HAL_TIM_SET_COUNTER(motor->timer, 0);

    // Start timer
    HAL_TIM_Base_Start_IT(motor->timer);

    // Handler will take care of the rest
}

void pulse_stepper(stepper *motor)
{
    // IF no more steps remaining in move:
    if (motor->steps_remaining <= 0)
    {
        // reset step pin (don't keep step pin high)
        HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);
        // STOP timer
        HAL_TIM_Base_Stop_IT(motor->timer);
    }
    else
    {
        // Read current step pin state
        GPIO_PinState currentPinState = HAL_GPIO_ReadPin(motor->step_port, motor->step_pin);

        // IF pin state is set: reset the pin and wait step_pulse
        if (currentPinState == GPIO_PIN_SET)
        {
            HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);
            __HAL_TIM_SET_AUTORELOAD(motor->timer, motor->step_pulse);
        }
        // ELSE: set the pin for 20 us
        // We should pull HIGH for at least 1-2us (step_high_min)
        else
        {
            // Check if within bounds of motor
            //--------------------------------
            // 1. if 0 and trying to decrease
            if (motor->step_count <= 0 && motor->dir_state)
                return;
            // 2. if 360 and trying to increase
            else if (motor->step_count >= MAX_DEGREES && !motor->dir_state)
                return;
            //--------------------------------

            // Not outside bounds -- GREAT
            HAL_GPIO_WritePin(motor->step_port, motor->step_pin, SET);
            __HAL_TIM_SET_AUTORELOAD(motor->timer, 20);

            // Dec steps remaining in current move
            motor->steps_remaining--;

            // Inc/Dec the stepper position
            if (!motor->dir_state) // 0 = clockwise?
                motor->step_count++;
            else // 1 = counter-clockwise?
                motor->step_count--;
        }
        // RESET timer
        __HAL_TIM_SET_COUNTER(motor->timer, 0);
    }
}