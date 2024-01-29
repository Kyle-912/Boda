#include "A4988.h"
#include <math.h>
#define PI 3.14159265358979323846

// Global array to store pre-calculated RPM values for ramp-up
// #define RAMP_STEPS 80
// float rpm_values[RAMP_STEPS];

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
    motor->max_steps = 200*80;
    motor->min_rpm = 40;

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



    motor->precalculation_done = false;
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
void set_micro_en(stepper *motor, uint8_t micro_en) {
    motor->enable_microsteps = micro_en;
}

// set dir state (clockwise/counter-clockwise)
// void home(stepper *motor) {
//     motor->step_count = 0;
// }

uint8_t setMicrostep(stepper *motor) {
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

void set_max_steps(stepper *motor, short max)
{
    motor->max_steps = max;
}

// long calcStepsForRotation(stepper *motor, long deg){
//     return deg * motor->steps * (long)motor->microsteps / 360;
// }

long calcStepsForRotation(stepper *motor, double deg)
{
    return deg * motor->steps * motor->microsteps / 360;
}

void move_stepper_steps(stepper *motor, int16_t steps_, float rpm_)
{
    if (motor->enable_microsteps){
        if (setMicrostep(motor) == -1)
        {
            // Error in setting microsteps
            return;
        }
    }

    // if negative steps, set dir
    if (steps_ < 0)
    {
        motor->dir_state = 1;
        steps_ = steps_ * -1;
    }
    else
    {
        motor->dir_state = 0;
    }
    // set steps
    motor->steps_remaining = steps_;
    // motor->step_count = 0;

    // set rpm
    motor->rpm = rpm_;
    motor->step_pulse = STEP_PULSE(motor->max_steps, motor->microsteps, rpm_);

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

void move_stepper_deg(stepper *motor, double deg) {

    if (motor->enable_microsteps){
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
    else
    {
        motor->dir_state = 0;
    }

    // Set step output LOW
    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);

    // Set sleep output HIGH
    HAL_GPIO_WritePin(motor->sleep_port, motor->sleep_pin, SET);

    motor->steps_remaining = calcStepsForRotation(motor, deg);
    motor->step_pulse = STEP_PULSE(motor->max_steps, motor->microsteps, motor->rpm);
    // motor->step_count = 0;

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
        {
            // reset step pin (don't keep step pin high)
            HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);
            // STOP timer
            HAL_TIM_Base_Stop_IT(motor->timer);
            // Reset Steps
            motor->steps_remaining = 0;
            return;
        }
        // 2. if 360 and trying to increase
        else if (motor->step_count >= motor->max_steps && !motor->dir_state)
        {
            // reset step pin (don't keep step pin high)
            HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);
            // STOP timer
            HAL_TIM_Base_Stop_IT(motor->timer);
            // Reset Steps
            motor->steps_remaining = 0;
            return;
        }
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


void pulse_stepper_sinusoid(stepper *motor)
{

    // Read current step pin state
    GPIO_PinState currentPinState = HAL_GPIO_ReadPin(motor->step_port, motor->step_pin);

    // IF pin state is set: reset the pin and wait step_pulse
    if (currentPinState == GPIO_PIN_SET)
    {
        HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);
        if (motor->steps_remaining > 0) {
            calculate_next_sinusoidal_pulse(motor);
            __HAL_TIM_SET_AUTORELOAD(motor->timer, motor->step_pulse);
        }
        else
        {
            motor->precalculation_done = false;
        }
    }
    // ELSE: set the pin for 20 us
    // We should pull HIGH for at least 1-2us (step_high_min)
    else if (motor->steps_remaining > 0)
    {
        // Check if within bounds of motor
        //--------------------------------
        // 1. if 0 and trying to decrease
        if (motor->step_count <= 0 && motor->dir_state) 
        {
            // reset step pin (don't keep step pin high)
            HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);
            // STOP timer
            HAL_TIM_Base_Stop_IT(motor->timer);
            // Reset Steps
            motor->steps_remaining = 0;
            return;
        }
        // 2. if 360 and trying to increase
        else if (motor->step_count >= motor->max_steps && !motor->dir_state)
        {
            // reset step pin (don't keep step pin high)
            HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);
            // STOP timer
            HAL_TIM_Base_Stop_IT(motor->timer);
            // Reset Steps
            motor->steps_remaining = 0;
            return;
        }
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
    else
    {
        return;
    }
    // RESET timer
    __HAL_TIM_SET_COUNTER(motor->timer, 0);
}

// Initialize these values before starting the motor movement
void init_sinusoidal_vars(int total, double peak_rpm, double rise_time, stepper *motor) {
    float temp_peak = peak_rpm;
    motor->peak_rpm = peak_rpm;
    motor->current_step = 1;
    motor->total_steps = total;
    double steps_per_second_at_peak = peak_rpm * motor->max_steps / 60.0;
    motor->ramp_steps = (int)(rise_time * steps_per_second_at_peak);
    if (2 * motor->ramp_steps > motor->total_steps) {
        motor->ramp_steps = motor->total_steps / 2;
    }
    motor->max_delay = 60.0 / (peak_rpm * motor->max_steps); // Max delay in seconds
}

void calculate_next_sinusoidal_pulse(stepper *motor) {
    // Increment the current step counter
    motor->current_step++;

    // Ensure we do not exceed total steps
    if (motor->current_step >= motor->total_steps) {
        motor->current_step = motor->total_steps - 1;
    }

    // Calculate the RPM based on the sinusoidal ramp profile
    float rpm = calculate_rpm_basic_ramp(motor);

    // Update the step pulse duration in the motor struct
    motor->step_pulse = SIMPLIFIED_STEP_PULSE(rpm);
}

// Function to pre-calculate ramp-up RPM values
void precalculate_ramp_values(stepper *motor) {
    for (int i = 0; i < RAMP_STEPS; i++) {
        float normalized_step = ((float)i / RAMP_STEPS) * M_PI;
        float ramp_up_value = sin(normalized_step - M_PI) + normalized_step;
        motor->rpm_values[i] = fmax((ramp_up_value / M_PI) * motor->peak_rpm, motor->min_rpm);
    }
    asm("nop");
}

// Function to get RPM based on current step
float get_rpm_for_step(stepper *motor) {
    if (motor->current_step >= motor->total_steps) {
        return 0;
    }
    
    if (motor->current_step < RAMP_STEPS) {
        // Ramp-up phase
        float ret_rpm = motor->rpm_values[motor->current_step];
        asm("nop");
        return ret_rpm;
        // return rpm_values[current_step];
    } else if (motor->current_step >= motor->total_steps - RAMP_STEPS) {
        // Ramp-down phase
        int index = RAMP_STEPS - 1 - (motor->current_step - (motor->total_steps - RAMP_STEPS));
        float ret_rpm = motor->rpm_values[index];
        asm("nop");
        return ret_rpm;
        // return rpm_values[index];
    } else {
        // Constant speed phase
        return motor->peak_rpm;
    }
}

// Main function to calculate RPM based on the current step
float calculate_rpm_basic_ramp(stepper *motor) {

    // IF ramp steps not calculated yet
    if (!motor->precalculation_done) {

        // Calculate the ramp steps for the movement
        precalculate_ramp_values(motor);
        motor->precalculation_done = true;
    }

    // Get the RPM for the current step
    return get_rpm_for_step(motor);
}