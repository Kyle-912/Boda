/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "controller_driver.h"
#include "A4988.h"
#include "robot_arm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Motor 1 
#define motor1_dir_port GPIOC
#define motor1_dir_pin GPIO_PIN_8
#define motor1_step_port GPIOC
#define motor1_step_pin GPIO_PIN_6
#define motor1_sleep_port GPIOC
#define motor1_sleep_pin GPIO_PIN_5

// Motor 2 
#define motor2_dir_port GPIOA
#define motor2_dir_pin GPIO_PIN_6
#define motor2_step_port GPIOA
#define motor2_step_pin GPIO_PIN_7
#define motor2_sleep_port GPIOB
#define motor2_sleep_pin GPIO_PIN_6

// Motor 3
#define motor3_dir_port GPIOB
#define motor3_dir_pin GPIO_PIN_7
#define motor3_step_port GPIOC
#define motor3_step_pin GPIO_PIN_13
#define motor3_sleep_port GPIOC
#define motor3_sleep_pin GPIO_PIN_14

// typedef int bool; // Define a custom boolean type
// #define true 1    // Define true as 1
// #define false 0   // Define false as 0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define map_range(value, in_min, in_max, out_min, out_max) \
    (((value) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

#define low_rpm 50
#define high_rpm 300

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

stepper *motor1 = NULL;
stepper *motor2 = NULL;
stepper *motor3 = NULL;
stepper *motor4 = NULL;

PS2ControllerHandler ps2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */

void PS2_Init(PS2ControllerHandler *ps2);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  // Enable TIM3 global Interrupt & set priority
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  PS2_Init(&ps2);

  char *messageX = "X has been Pressed\r\n";
  char *messageO = "O has been Pressed\r\n";

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  bool toggle1 = true;
  bool toggle2 = true;

  float rpm = 400;
  short microsteps = FULL_STEPS;
  double deg = 20;
  const short spr = 200; // Steps per revolution

  // Motor 1 
  stepper stepper_motor_1;
  motor1 = &stepper_motor_1;
  init_stepper(motor1, spr);
  init_dir_pin(motor1, motor1_dir_port, motor1_dir_pin);
  init_step_pin(motor1, motor1_step_port, motor1_step_pin);
  init_sleep_pin(motor1, motor1_sleep_port, motor1_sleep_pin);
  set_micro_en(motor1, 0);
  set_timer(motor1, &htim3);
  set_rpm(motor1, rpm);

  // Motor 2 
  stepper stepper_motor_2;
  motor2 = &stepper_motor_2;
  init_stepper(motor2, spr);
  init_dir_pin(motor2, motor2_dir_port, motor2_dir_pin);
  init_step_pin(motor2, motor2_step_port, motor2_step_pin);
  init_sleep_pin(motor2, motor2_sleep_port, motor2_sleep_pin);
  set_micro_en(motor2, 0);
  set_timer(motor2, &htim14);
  set_rpm(motor2, rpm);

  // Motor 2 
  stepper stepper_motor_3;
  motor3 = &stepper_motor_3;
  init_stepper(motor3, spr);
  init_dir_pin(motor3, motor3_dir_port, motor3_dir_pin);
  init_step_pin(motor3, motor3_step_port, motor3_step_pin);
  init_sleep_pin(motor3, motor3_sleep_port, motor3_sleep_pin);
  set_micro_en(motor3, 0);
  set_timer(motor3, &htim13);
  set_rpm(motor3, rpm);

  double mapped_left = 0;
  double mapped_up = 0;

  arm robot_arm_var;
  arm* robot_arm = &robot_arm_var;
  // init_arm_2(robot_arm, 200.0f, motor1, motor2);
  init_arm(robot_arm, 200.0, 3, motor1, motor2, motor3);

  home(robot_arm);

  // set_coordinate(robot_arm, 0, 10, 10);
  set_coordinate(robot_arm, 0, 3, 2*80, 2*80, 2*80);
  set_coordinate(robot_arm, 1, 3, 120*80, 200*80, 160*80);
  set_coordinate(robot_arm, 2, 3, 30*80, 60*80, 45*80);
  uint8_t coord = 0;
  uint8_t rpm_step = 12;

  int8_t flip = 1;
  while (1)
  {
    // PS2_Update(&ps2);


    // --------------------------------- MOTOR 2 GRADUAL INC
    // if (Is_Button_Pressed(&ps2, X))
    // {
    //   set_rpm(motor1, rpm_step);
    //   move_stepper_deg(motor1, 500 * flip);
    //   while(motor1->steps_remaining);
    //   // HAL_Delay(5000);
    //   rpm_step += 5;
    //   // set_rpm(motor1, rpm_step);
    //   // move_stepper_deg(motor1, -500);
    //   // HAL_Delay(5000);
    //   // rpm_step += 5;

    //   flip *= -1;
    // }
    // HAL_Delay(500);
    // ---------------------------------

    // ----------------------------------------------
    // for(int i = 0; i < 10; i++) {
    //   // move_stepper_steps(motor1, 30, 100);
    //   move_stepper_deg(motor2, 60);
    //   // move_stepper_steps(motor2, 30, 100);
    //   HAL_Delay(1000);
    // }
    // for(int i = 0; i < 10; i++) {
    //   // move_stepper_steps(motor1, -30, 100);
    //   move_stepper_deg(motor2, -60);
    //   // move_stepper_steps(motor2, -30, 100);
    //   HAL_Delay(1000);
    // }
    // ---------------------------------------------- MOTOR INC / DEC


    // ---------------------------------------------- MOVE BETWEEN POINTS
    // if (coord == 0)
    // {
    //   set_arm_rpm(robot_arm, 200);
    //   move(robot_arm, 0);
    //   coord = 1;
    // }
    // else if (coord == 1)
    // {
    //   set_arm_rpm(robot_arm, 20);
    //   move(robot_arm, 1);
    //   coord = 2;
    // }
    // else if (coord == 2)
    // {
    //   set_arm_rpm(robot_arm, 150);
    //   move(robot_arm, 2);
    //   coord = 3;
    // }
    // else if (coord == 3)
    // {
    //   set_arm_rpm(robot_arm, 100);
    //   move(robot_arm, 3);
    //   coord = 0;
    // }
    // ---------------------------------------------- MOVE BETWEEN POINTS


    // if (Is_Button_Pressed(&ps2, X))
    // {
    //   if (coord == 0)
    //   {
    //     set_arm_rpm(robot_arm, 80);
    //     move(robot_arm, 0);
    //     coord = 1;
    //   }
    //   else if (coord == 1)
    //   {
    //     set_arm_rpm(robot_arm, 20);
    //     move(robot_arm, 1);
    //     coord = 2;
    //   }
    //   else if (coord == 2)
    //   {
    //     set_arm_rpm(robot_arm, 150);
    //     move(robot_arm, 2);
    //     coord = 0;
    //   }
    //   // else if (coord == 3)
    //   // {
    //   //   set_arm_rpm(robot_arm, 100);
    //   //   move(robot_arm, 3);
    //   //   coord = 0;
    //   // }

    //   while(motor1->steps_remaining || motor2->steps_remaining);
    // }
    asm("nop");

    if (coord == 0)
    {
      set_arm_rpm(robot_arm, 450);
      move(robot_arm, 0);
      coord = 1;
    }
    else if (coord == 1)
    {
      set_arm_rpm(robot_arm, 450);
      move(robot_arm, 1);
      coord = 2;
    }
    else if (coord == 2)
    {
      set_arm_rpm(robot_arm, 450);
      move(robot_arm, 2);
      coord = 0;
    }
    // else if (coord == 3)
    // {
    //   set_arm_rpm(robot_arm, 100);
    //   move(robot_arm, 3);
    //   coord = 0;
    // }

    while(motor1->steps_remaining || motor2->steps_remaining || motor3->steps_remaining);

    HAL_Delay(2000);



    // move_stepper_steps(motor1, 30, 200);
    // move_stepper_steps(motor2, 30, 200);

    // move_stepper_deg(motor1, 185);
    // move_stepper_deg(motor1, 60);

    // HAL_Delay(2000);


    // PS2_Update(&ps2);

    // uint8_t left_val = Is_Joystick_Left_Moved(&ps2, JOYSTICK_L_RL);
    // uint8_t up_val = Is_Joystick_Left_Moved(&ps2, JOYSTICK_L_UD);


    // if (left_val != NEUTRAL)
    // {
    //   if (left_val < NEUTRAL)
    //   {
    //     set_dir_state(motor1, 1);
    //     mapped_left = map_range(left_val, 0, 126, low_rpm, high_rpm);
    //   }
    //   else 
    //   {
    //     set_dir_state(motor1, 0);
    //     mapped_left = map_range(left_val, 128, 255, low_rpm, high_rpm);
    //   }
    //   set_rpm(motor1, mapped_left);
    //   HAL_UART_Transmit(&huart2, (uint8_t *)messageO, strlen(messageX), 100);
    //   toggle1 = true;
    // }
    // else
    // {
    //   toggle1 = false;
    // }

    // if (up_val != NEUTRAL)
    // {
    //   if (up_val < NEUTRAL)
    //   {
    //     set_dir_state(motor2, 1);
    //     mapped_up = map_range(up_val, 0, 126, low_rpm, high_rpm);
    //   }
    //   else 
    //   {
    //     set_dir_state(motor2, 0);
    //     mapped_up = map_range(up_val, 128, 255, low_rpm, high_rpm);
    //   }
    //   set_rpm(motor2, mapped_up);
    //   HAL_UART_Transmit(&huart2, (uint8_t *)messageO, strlen(messageO), 100);
    //   toggle2 = true;
    // }
    // else
    // {
    //   toggle2 = false;
    // }

    // if (toggle1 && !motor1->steps_remaining)
    // {
    //   move_stepper_deg(motor1, deg);
    // }

    // // move_stepper_deg(motor1, deg);
    // // move_stepper_deg(motor2, deg);
    // // HAL_Delay(5000);

    // if (toggle2 && !motor2->steps_remaining)
    // {
    //   move_stepper_deg(motor2, deg);
    // }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 44;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 44;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 44;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC5 PC6
                           PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA7 PA8
                           PA9 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB4 PB5 PB6
                           PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Callback function
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   // Check if the timer that elapsed is from motor1-motor4
//   if (htim->Instance == motor1->timer->Instance) pulse_stepper(motor1);
//   else if (htim->Instance == motor2->timer->Instance) pulse_stepper(motor2);
//   else if (htim->Instance == motor3->timer->Instance) pulse_stepper(motor3);
//   else if (htim->Instance == motor4->timer->Instance) pulse_stepper(motor4);
// }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    stepper *current_motor = NULL;

    // Determine which motor's timer has elapsed
    if (htim->Instance == motor1->timer->Instance) current_motor = motor1;
    else if (htim->Instance == motor2->timer->Instance) current_motor = motor2;
    else if (htim->Instance == motor3->timer->Instance) current_motor = motor3;
    // Add conditions for motor3 and motor4 if needed

    if (current_motor) {
        pulse_stepper_sinusoid(current_motor);

        // Adjust the step_pulse based on the sinusoidal profile for the next step
        // This requires a function that calculates the delay for the next step
        // based on the current step number and the sinusoidal profile
        // if (current_motor->steps_remaining > 0) {
        //     calculate_next_sinusoidal_pulse(current_motor);
        //     __HAL_TIM_SET_AUTORELOAD(current_motor->timer, current_motor->step_pulse);
        //     // RESET timer
        //     __HAL_TIM_SET_COUNTER(current_motor->timer, 0);
        // }
    }
}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim->Instance == motor1->timer->Instance) {
//         pulse_stepper(motor1);
//         motor1->step_completed_flag = true;
//     } else if (htim->Instance == motor2->timer->Instance) {
//         pulse_stepper(motor2);
//         motor2->step_completed_flag = true;
//     }
//     // Add similar else-if blocks for other motors (motor3, motor4, etc.) if needed
// }



void PS2_Init(PS2ControllerHandler *ps2)
{
  ps2->GPIO = GPIO_PIN_12;
  ps2->PIN = GPIOB;
  ps2->spi = &hspi2;
  ps2->tim = &htim1;
  ps2->tim->Instance->CNT = 0;
  ps2->tim->Instance->CR1 |= TIM_CR1_CEN;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
