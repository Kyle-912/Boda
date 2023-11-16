#ifndef A4988_H
#define A4988_H

// Define the A4988 struct
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

// Function declaration for initializing the A4988 struct
struct A4988 initA4988(short steps, short dir_pin, short step_pin, short ms1_pin, short ms2_pin, short ms3_pin);

#endif // A4988_H
