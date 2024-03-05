#ifndef A4988_H
#define A4988_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define MAX_DEGREES 360

#define FULL_STEPS 1
#define HALF_STEPS 2
#define QUART_STEPS 3
#define RAMP_STEPS 80

// Define the stepper structure
typedef struct
{
    short steps;
    short dir_state;
    short max_steps;
    short step_limit;
    uint16_t enable_pin;
    uint8_t enable_microsteps;
    long step_count;
    long steps_remaining;
    short microsteps;
    float rpm;
    long step_pulse;

    // -- Vars associated with Sinusoidal RPM  --
    float peak_rpm;
    float min_rpm;
    short current_step;
    short total_steps;
    short ramp_steps;
    double max_delay;
    volatile bool step_completed_flag;
    volatile bool precalculation_done;
    volatile bool sinusoidal_ramp;
    float rpm_values[RAMP_STEPS];
    // ------------------------------------------

    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

    GPIO_TypeDef *step_port;
    uint16_t step_pin;

    GPIO_TypeDef *sleep_port;
    uint16_t sleep_pin;

    GPIO_TypeDef *ms1_port;
    uint16_t ms1_pin;

    GPIO_TypeDef *ms2_port;
    uint16_t ms2_pin;

    GPIO_TypeDef *ms3_port;
    uint16_t ms3_pin;

    TIM_HandleTypeDef *timer;
} stepper;

// Define constants
extern const uint8_t MS_TABLE[];
extern const uint8_t MS_TABLE_SIZE;
#define STEP_PULSE(steps, microsteps, rpm) (60.0 * 1000000L / steps / microsteps / rpm)
// #define SIMPLIFIED_STEP_PULSE(rpm) (60.0 * 5000L / rpm)
#define SIMPLIFIED_STEP_PULSE(rpm) (300000L / rpm)

// Function prototypes
void init_stepper(stepper *motor, short spr);
void init_dir_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_step_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_sleep_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_ms1_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_ms2_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_ms3_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void set_microsteps(stepper *motor, short microsteps);
void set_step_limit(stepper *motor, short max);
void set_timer(stepper *motor, TIM_HandleTypeDef *timer_);
void set_rpm(stepper *motor, float rpm);
void set_dir_state(stepper *motor, short dir_state);
void set_micro_en(stepper *motor, uint8_t micro_en);
uint8_t setMicrostep(stepper *motor);
long calcStepsForRotation(stepper *motor, double deg);
void move_stepper_steps(stepper *motor, int16_t steps_, float rpm_);
void move_stepper_deg(stepper *motor, double deg);
void pulse_stepper(stepper *motor);
void pulse_stepper_sinusoid(stepper *motor);
void calculate_next_sinusoidal_pulse(stepper *motor);

void init_sinusoidal_vars(int total, double peak_rpm, double rise_time, stepper *motor);
// double calculate_next_sinusoidal_pulse(stepper *motor);

float calculate_rpm_basic_ramp(stepper *motor);
#endif // A4988_H