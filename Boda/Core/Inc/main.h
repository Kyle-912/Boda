/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Shoulder_1_2_Pin GPIO_PIN_13
#define Shoulder_1_2_GPIO_Port GPIOC
#define Shoulder_1_3_Pin GPIO_PIN_14
#define Shoulder_1_3_GPIO_Port GPIOC
#define Bluetooth_TX_Pin GPIO_PIN_2
#define Bluetooth_TX_GPIO_Port GPIOA
#define Bluetooth_RX_Pin GPIO_PIN_3
#define Bluetooth_RX_GPIO_Port GPIOA
#define Shoulder_2_1_Pin GPIO_PIN_6
#define Shoulder_2_1_GPIO_Port GPIOA
#define Shoulder_2_2_Pin GPIO_PIN_7
#define Shoulder_2_2_GPIO_Port GPIOA
#define Elbow_3_Pin GPIO_PIN_5
#define Elbow_3_GPIO_Port GPIOC
#define PS2_Controller_SCK_Pin GPIO_PIN_10
#define PS2_Controller_SCK_GPIO_Port GPIOB
#define Attachment_GPIO_Pin GPIO_PIN_13
#define Attachment_GPIO_GPIO_Port GPIOB
#define PS2_Controller_MISO_Pin GPIO_PIN_14
#define PS2_Controller_MISO_GPIO_Port GPIOB
#define PS2_Controller_MOSI_Pin GPIO_PIN_15
#define PS2_Controller_MOSI_GPIO_Port GPIOB
#define Elbow_2_Pin GPIO_PIN_6
#define Elbow_2_GPIO_Port GPIOC
#define Elbow_1_Pin GPIO_PIN_8
#define Elbow_1_GPIO_Port GPIOC
#define PS2_Controller_Chip_Select_1_Pin GPIO_PIN_13
#define PS2_Controller_Chip_Select_1_GPIO_Port GPIOA
#define PS2_Controller_Chip_Select_2_Pin GPIO_PIN_14
#define PS2_Controller_Chip_Select_2_GPIO_Port GPIOA
#define Attachment_SCK_Pin GPIO_PIN_10
#define Attachment_SCK_GPIO_Port GPIOC
#define Attachment_MISO_Pin GPIO_PIN_11
#define Attachment_MISO_GPIO_Port GPIOC
#define Attachment_MOSI_Pin GPIO_PIN_12
#define Attachment_MOSI_GPIO_Port GPIOC
#define Shoulder_2_3_Pin GPIO_PIN_6
#define Shoulder_2_3_GPIO_Port GPIOB
#define Shoulder_1_1_Pin GPIO_PIN_7
#define Shoulder_1_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
