#include "PS2.h"

// Called before button/joystick check to get latest inputs
void PS2_Update(PS2ControllerHandler *ps2i)
{
    uint8_t temp = 0b00000001;

    // set chip select low
    HAL_GPIO_WritePin(ps2i->CS_GPIO, ps2i->CS_PIN, GPIO_PIN_RESET);

    // Send Command: 0x01
    HAL_SPI_Transmit(ps2i->spi, &temp, 1, 10);
    // Trigger the acknowledge signal to signal a byte has completed
    HAL_GPIO_WritePin(ps2i->Ack_GPIO, ps2i->Ack_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ps2i->Ack_GPIO, ps2i->Ack_PIN, GPIO_PIN_SET);

    delay_2_25us(ps2i->tim, 1);

    // Send Command: 0x42
    temp = 0b01000010;
    HAL_SPI_Transmit(ps2i->spi, &temp, 1, 10);
    // Trigger Acknowledge
    HAL_GPIO_WritePin(ps2i->Ack_GPIO, ps2i->Ack_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ps2i->Ack_GPIO, ps2i->Ack_PIN, GPIO_PIN_SET);
    delay_2_25us(ps2i->tim, 1);

    // Send Command: 0x00 7 times to read incoming data
    temp = 0x00;
    for (uint8_t i = 0; i < 7; i++)
    {
        HAL_SPI_TransmitReceive(ps2i->spi, &temp, &ps2i->PS2Data[i], 1, 10);

        HAL_GPIO_WritePin(ps2i->Ack_GPIO, ps2i->Ack_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ps2i->Ack_GPIO, ps2i->Ack_PIN, GPIO_PIN_SET);
        delay_2_25us(ps2i->tim, 1);
    }
    // Set Chip Select High
    HAL_GPIO_WritePin(ps2i->CS_GPIO, ps2i->CS_PIN, GPIO_PIN_SET);
}

bool Is_Button_Pressed(PS2ControllerHandler *ps2i, uint8_t button)
{
    uint8_t temp = ps2i->PS2Data[BUTTON_INDEX];
    temp |= button;
    if (temp == button)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Is_DPad_Pressed(PS2ControllerHandler *ps2i, uint8_t button)
{
    uint8_t temp = ps2i->PS2Data[DPAD_INDEX];
    temp |= button;
    if (temp == button)
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t Is_Joystick_Right_Moved(PS2ControllerHandler *ps2i, uint8_t direction)
{
    // If the direction isnt for the Right Joystick LR or UD then exit
    if (direction == 3 || direction == 4)
    {
        uint8_t temp = ps2i->PS2Data[direction];
        if (temp < DEADZONE_LO || temp > DEADZONE_HI)
        {
            return temp;
        }
        return NEUTRAL;
    }
    return NEUTRAL;
}

uint8_t Is_Joystick_Left_Moved(PS2ControllerHandler *ps2i, uint8_t direction)
{
    // If the direction isnt for the Left Joystick LR or UD then exit
    if (direction == 5 || direction == 6)
    {
        uint8_t temp = ps2i->PS2Data[direction];
        if (temp < DEADZONE_LO || temp > DEADZONE_HI)
        {
            return temp;
        }
        return NEUTRAL;
    }
    return NEUTRAL;
}

void delay_2_25us(TIM_HandleTypeDef *tim, uint16_t us)
{
    tim->Instance->CNT = 0;
    tim->Instance->CR1 |= TIM_CR1_CEN;
    while (tim->Instance->CNT < us)
        ;
    tim->Instance->CR1 |= TIM_CR1_CEN;
}