#include "A4988.h"


struct A4988 {
    short steps;
    short dir_pin;
    short step_pin;

    short ms1_pin;
    short ms2_pin;
    short ms3_pin;

    short enable_pin;

    long steps_to_cruise;
	long steps_remaining;

	short dir_state;

	long steps_to_brake;
	long step_pulse;
    long cruise_step_pulse;
	long rest;
	long step_count;
};


struct A4988 initA4988(short steps, short dir_pin, short step_pin, short ms1_pin, short ms2_pin, short ms3_pin) {
    struct A4988 driver;

    driver.steps = steps;
    driver.dir_pin = dir_pin;
    driver.step_pin = step_pin;

    driver.ms1_pin = ms1_pin;
    driver.ms2_pin = ms2_pin;
    driver.ms3_pin = ms3_pin;

    // no enable pin
    driver.enable_pin = -1;

    driver.steps_to_cruise = 0;
	driver.steps_remaining = 0;
	driver.dir_state = 0;
	driver.steps_to_brake = 0;
	driver.step_pulse = 0;
    driver.cruise_step_pulse = 0;
	driver.rest = 0;
	driver.step_count = 0;

    return driver;
};

// void begin(struct A4988* driver, float rpm, short microsteps)


