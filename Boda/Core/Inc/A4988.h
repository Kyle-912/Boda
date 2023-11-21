#ifndef A4988_H
#define A4988_H

#include "stm32f4xx_hal.h"

#define FULL_STEPS 1
#define HALF_STEPS 2
#define QUART_STEPS 3

// Define the stepper structure
typedef struct {
    short steps;
    short dir_state;
    uint16_t enable_pin;
    uint8_t enable_microsteps;
    long step_count;
    long steps_remaining;
    short microsteps;
    float rpm;
    long step_pulse;

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
#define STEP_PULSE(steps, microsteps, rpm) (60.0*1000000L/steps/microsteps/rpm)

// Function prototypes
void init_stepper(stepper *motor, short spr);
void init_dir_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_step_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_sleep_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_ms1_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_ms2_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_ms3_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void set_microsteps(stepper *motor, short microsteps);
void set_timer(stepper *motor, TIM_HandleTypeDef *timer);
void set_rpm(stepper *motor, float rpm);
void set_dir_state(stepper *motor, short dir_state);
uint8_t setMicrostep(stepper *motor);
long calcStepsForRotation(stepper *motor, double deg);
void move_stepper_deg(stepper *motor, double deg);

#endif // A4988_H
