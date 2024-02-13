#include "stm32f4xx_hal.h"

#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#define DPAD_INDEX 1
#define BUTTON_INDEX 2
#define JOYSTICK_R_RL 3
#define JOYSTICK_R_UD 4
#define JOYSTICK_L_RL 5
#define JOYSTICK_L_UD 6

#define X 0xBF
#define CIRCLE 0xDF
#define TRIANGLE 0xEF
#define SQUARE 0x7F

#define R1 0xF7
#define R2 0xFD
#define L1 0xFB
#define L2 0xFE

#define DRIGHT 0xDF
#define DUP 0xEF
#define DDOWN 0xBF
#define DLEFT 0x7F

#define START 0xF7
#define SELECT 0xFE
#define L3 0xFD
#define R3 0xFB

#define NEUTRAL 0x7F
#define DEADZONE_HI 0x9F
#define DEADZONE_LO 0x5F

// typedef int bool; // Define a custom boolean type
// #define true 1    // Define true as 1
// #define false 0   // Define false as 0

typedef struct
{
    GPIO_TypeDef *Ack_GPIO;
    uint16_t Ack_PIN;
    GPIO_TypeDef *CS_GPIO;
    uint16_t CS_PIN;
    SPI_HandleTypeDef *spi;
    TIM_HandleTypeDef *tim;
    uint8_t *PS2Data[7];
} PS2ControllerHandler;

void PS2_Update(PS2ControllerHandler *ps2i);
bool Is_Button_Pressed(PS2ControllerHandler *ps2i, uint8_t button);
bool Is_DPad_Pressed(PS2ControllerHandler *ps2i, uint8_t button);
uint8_t Is_Joystick_Right_Moved(PS2ControllerHandler *ps2i, uint8_t direction);
uint8_t Is_Joystick_Left_Moved(PS2ControllerHandler *ps2i, uint8_t direction);
void delay_2_25us(TIM_HandleTypeDef *tim, uint16_t us);

#endif