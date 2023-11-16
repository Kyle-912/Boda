#include "controller_driver.h"

//Called before button/joystick check to get latest inputs
void PS2_Update(PS2ControllerHandler *ps2i)
{
    // wait time until data will be ready on the controller
    if(ps2i->tim->Instance->CNT < 10){
        return;
    }
    uint8_t temp = 0b00000001;
    // set chip select low
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

    // Send Command: 0x01
    HAL_SPI_Transmit(ps2i->spi, &temp, 1, 10);
    // Trigger the acknowledge signal to signal a byte has completed
    HAL_GPIO_WritePin(ps2i->GPIO, ps2i->PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ps2i->GPIO, ps2i->PIN, GPIO_PIN_SET);

    delay_2_25us(ps2i->tim, 1);

    // Send Command: 0x42
    temp = 0b01000010;
    HAL_SPI_Transmit(ps2i->spi, &temp, 1, 10);
    // Trigger Achnowledge
    HAL_GPIO_WritePin(ps2i->GPIO, ps2i->PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ps2i->GPIO, ps2i->PIN, GPIO_PIN_SET);
    delay_2_25us(ps2i->tim, 1);

    // Send Command: 0x00 7 times to read incoming data
    temp = 0x00;
    for (uint8_t i = 0; i < 7; i++)
    {
        HAL_SPI_TransmitReceive(ps2i->spi, &temp, &ps2i->PS2Data[i], 1, 10);

        HAL_GPIO_WritePin(ps2i->GPIO, ps2i->PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ps2i->GPIO, ps2i->PIN, GPIO_PIN_SET);
        delay_2_25us(ps2i->tim, 1);
    }
    // Set Chip Select High
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

    // start a timer to count until the next set of data will be ready on the controller
    ps2i->tim->Instance->CNT = 0;
    ps2i->tim->Instance->CR1 |= TIM_CR1_CEN;
}

bool Is_Button_Pressed(PS2ControllerHandler *ps2i, uint8_t button){
    uint8_t temp = ps2i->PS2Data[BUTTON_INDEX];
    temp |= button;
    if(temp == button){
        return true;
    }
    else{
        return false;
    }
}

bool Is_DPad_Pressed(PS2ControllerHandler *ps2i, uint8_t button){
    uint8_t temp = ps2i->PS2Data[DPAD_INDEX];
    temp |= button;
    if(temp == button){
        return true;
    }
    else{
        return false;
    }
}

uint8_t Is_Joystick_Right_Moved(PS2ControllerHandler *ps2i, uint8_t direction){
    if(direction == 3 || direction == 4){
        uint8_t temp = ps2i->PS2Data[direction];
        if(temp < 0x50 || temp > 0x90){
            return temp;
        }
        return NEUTRAL;
    }
    return NEUTRAL;
}

uint8_t Is_Joystick_Left_Moved(PS2ControllerHandler *ps2i, uint8_t direction){
    if(direction == 5 || direction == 6){
        uint8_t temp = ps2i->PS2Data[direction];
        if(temp < 0x50 || temp > 0xA0){
            return temp;
        }
        return NEUTRAL;
    }
    return NEUTRAL;
}
/*
POSSIBLE FUNCTION TO CHECK MULTIPLE BUTTONS AT THE SAME TIME
bool* Is_Button_Pressed(uint8_t* button, bool* store){
    for (int i = 0; i < sizeof(button) / sizeof(button[i]); i++){
        if (button is pressed){
            store[i] = true;
        }
        else{
            store[i] = false;
        }
    }
}
*/

void delay_2_25us(TIM_HandleTypeDef *tim, uint16_t us)
{
    tim->Instance->CNT = 0;
    tim->Instance->CR1 |= TIM_CR1_CEN;
    while (tim->Instance->CNT < us)
        ;
    tim->Instance->CR1 |= TIM_CR1_CEN;
}