#ifndef A4988_H
#define A4988_H

#include "stm32f4xx_hal.h"

typedef struct {
    short steps;
    short dir_state;
    short enable_pin;
    long step_count;
	long steps_remaining;
    short microsteps;
    float rpm;
    long step_pulse;

    GPIO_TypeDef * dir_port;
    uint16_t dir_pin;

    GPIO_TypeDef * step_port;
    uint16_t step_pin;

    GPIO_TypeDef * ms1_port;
    uint16_t ms1_pin;

    GPIO_TypeDef * ms2_port;
    uint16_t ms2_pin;

    GPIO_TypeDef * ms3_port;
    uint16_t ms3_pin;

    TIM_HandleTypeDef * timer;
} stepper;

	// long steps_to_brake;
	// long step_pulse;
    // long cruise_step_pulse;
	// long rest;
	// long step_count;

// Function declaration for initializing the A4988 struct
// struct stepper init(short steps, short dir_pin, short step_pin, short ms1_pin, short ms2_pin, short ms3_pin);

void init_dir_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_step_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_ms1_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_ms2_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);
void init_ms3_pin(stepper *motor, GPIO_TypeDef *port, uint16_t pin);

#endif // A4988_H
