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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "PS2.h"
#include "A4988.c"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef int bool; // Define a custom boolean type
#define true 1    // Define true as 1
#define false 0   // Define false as 0
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
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* Definitions for PS2DataUpdate */
osThreadId_t PS2DataUpdateHandle;
const osThreadAttr_t PS2DataUpdate_attributes = {
  .name = "PS2DataUpdate",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for StepperMotor1 */
osThreadId_t StepperMotor1Handle;
const osThreadAttr_t StepperMotor1_attributes = {
  .name = "StepperMotor1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StepperMotor2 */
osThreadId_t StepperMotor2Handle;
const osThreadAttr_t StepperMotor2_attributes = {
  .name = "StepperMotor2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mPS2Data */
osMutexId_t mPS2DataHandle;
const osMutexAttr_t mPS2Data_attributes = {
  .name = "mPS2Data"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
void StartPS2DataUpdate(void *argument);
void StartStepperMotor1(void *argument);
void StartStepperMotor2(void *argument);

/* USER CODE BEGIN PFP */

void PS2_Init(PS2ControllerHandler *ps2);

PS2ControllerHandler ps2;

float rpm = 300;
short microsteps = FULL_STEPS;
double deg = 20;
const short spr = 200; // Steps per revolution

stepper* motor1 = NULL;
stepper* motor2 = NULL;
stepper* motor3 = NULL;
stepper* motor4 = NULL;
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
  /* USER CODE BEGIN 2 */

  // Enable TIM3 global Interrupt & set priority
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  //----------STEPPER VARIABLES----------//

  //---------STEPPER INIT-----------//

  //----------PS2 INIT----------//
  PS2_Init(&ps2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mPS2Data */
  mPS2DataHandle = osMutexNew(&mPS2Data_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of PS2DataUpdate */
  PS2DataUpdateHandle = osThreadNew(StartPS2DataUpdate, NULL, &PS2DataUpdate_attributes);

  /* creation of StepperMotor1 */
  StepperMotor1Handle = osThreadNew(StartStepperMotor1, NULL, &StepperMotor1_attributes);

  /* creation of StepperMotor2 */
  StepperMotor2Handle = osThreadNew(StartStepperMotor2, NULL, &StepperMotor2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB4 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void pulse_stepper(stepper *motor)
{
  // motor1->steps_remaining--;
  if (motor->steps_remaining <= 0)
  {
    HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);
    HAL_TIM_Base_Stop_IT(motor->timer);
  }
  // We should pull HIGH for at least 1-2us (step_high_min)
  else
  {
    GPIO_PinState currentPinState = HAL_GPIO_ReadPin(motor->step_port, motor->step_pin);
    if (currentPinState == GPIO_PIN_SET)
    {
      HAL_GPIO_WritePin(motor->step_port, motor->step_pin, RESET);
      __HAL_TIM_SET_AUTORELOAD(motor->timer, motor->step_pulse);
    }
    else
    {
      HAL_GPIO_WritePin(motor->step_port, motor->step_pin, SET);
      __HAL_TIM_SET_AUTORELOAD(motor->timer, 20);
      motor->steps_remaining--;
    }
    __HAL_TIM_SET_COUNTER(motor->timer, 0);
  }
}

// Callback function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == motor1->timer->Instance)
  {
    pulse_stepper(motor1);
  }
  if (htim->Instance == motor2->timer->Instance)
  {
    pulse_stepper(motor2);
  }
  if (htim->Instance == motor3->timer->Instance)
  {
    pulse_stepper(motor3);
  }
  if (htim->Instance == motor4->timer->Instance)
  {
    pulse_stepper(motor4);
  }
}

void PS2_Init(PS2ControllerHandler *ps2)
{
  ps2->GPIO = GPIOB;
  ps2->PIN = GPIO_PIN_12;
  ps2->spi = &hspi2;
  ps2->tim = &htim1;
  HAL_GPIO_WritePin(ps2->GPIO, ps2->PIN, GPIO_PIN_SET);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartPS2DataUpdate */
/**
 * @brief  Function implementing the PS2DataUpdate thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPS2DataUpdate */
void StartPS2DataUpdate(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
    // Attempt to get the PS2Data Mutex
  for (;;)
  {
    osMutexWait(mPS2DataHandle, 50);
    PS2_Update(&ps2);
    // delay for 25 microseconds
    osMutexRelease(mPS2DataHandle);
    osDelay(25U);
  }
    // Return the PS2Data Mutex
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartStepperMotor1 */
/**
 * @brief Function implementing the StepperMotor1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStepperMotor1 */
void StartStepperMotor1(void *argument)
{
  /* USER CODE BEGIN StartStepperMotor1 */

  //----------Stepper Init----------//
  stepper stepper_motor;
  motor1 = &stepper_motor;
  init_stepper(&stepper_motor, spr);
  init_dir_pin(&stepper_motor, GPIOA, GPIO_PIN_10);
  init_step_pin(&stepper_motor, GPIOB, GPIO_PIN_8);
  init_sleep_pin(&stepper_motor, GPIOB, GPIO_PIN_5);
  set_micro_en(&stepper_motor, 0);
  set_timer(&stepper_motor, &htim3);
  set_rpm(&stepper_motor, rpm);

  //----------Task Variables----------//
  char *messageR = "Stick Moved Right\r\n";
  char *messageL = "Stick Moved Left\r\n";
  bool toggle = false;
  double mapped_left = 0;
  uint8_t left_val;

  /* Infinite loop */
  for (;;)
  {

    //Get the current value of the joystick
    osMutexWait(mPS2DataHandle, 10);
    left_val = Is_Joystick_Left_Moved(&ps2, JOYSTICK_L_RL);

    //evaluate joystick status
    if (left_val != NEUTRAL)
    {
      if (left_val < NEUTRAL)
      {
        set_dir_state(&stepper_motor, 1);
        mapped_left = map_range(left_val, 0, 126, low_rpm, high_rpm);
        HAL_UART_Transmit(&huart2, (uint8_t *)messageL, strlen(messageL), 100);
      }
      else
      {
        set_dir_state(&stepper_motor, 0);
        mapped_left = map_range(left_val, 128, 255, low_rpm, high_rpm);
        HAL_UART_Transmit(&huart2, (uint8_t *)messageR, strlen(messageR), 100);
      }
      set_rpm(&stepper_motor, mapped_left);
      toggle = true;
    }
    else
    {
      toggle = false;
    }

    //If the joystick is moved move the motor
    if (toggle && !stepper_motor.steps_remaining)
    {
      move_stepper_deg(&stepper_motor, deg);
    }

    // Return the PS2Data Mutex
    osMutexRelease(mPS2DataHandle);
    osDelay(10);
  }
  /* USER CODE END StartStepperMotor1 */
}

/* USER CODE BEGIN Header_StartStepperMotor2 */
/**
 * @brief Function implementing the StepperMotor2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStepperMotor2 */
void StartStepperMotor2(void *argument)
{
  /* USER CODE BEGIN StartStepperMotor2 */

  //----------Stepper Init----------//
  stepper stepper_motor;
  motor2 = &stepper_motor;
  init_stepper(&stepper_motor, spr);
  init_dir_pin(&stepper_motor, GPIOA, GPIO_PIN_9);
  init_step_pin(&stepper_motor, GPIOA, GPIO_PIN_8);
  init_sleep_pin(&stepper_motor, GPIOB, GPIO_PIN_4);
  set_micro_en(&stepper_motor, 0);
  set_timer(&stepper_motor, &htim14);
  set_rpm(&stepper_motor, rpm);

  //----------Task Variables----------//
  char *messageU = "Stick Moved Up\r\n";
  char *messageD = "Stick Moved Down\r\n";
  bool toggle = false;
  uint8_t up_val;
  double mapped_up = 0;

  /* Infinite loop */
  for (;;)
  {
    //Get the PS2Data Mutex
    osMutexWait(mPS2DataHandle, 10);

    //get the current value of the joystick up down
    up_val = Is_Joystick_Left_Moved(&ps2, JOYSTICK_L_UD);

    if (up_val != NEUTRAL)
    {
      if (up_val < NEUTRAL)
      {
        set_dir_state(&stepper_motor, 1);
        mapped_up = map_range(up_val, 0, 126, low_rpm, high_rpm);
      HAL_UART_Transmit(&huart2, (uint8_t *)messageU, strlen(messageU), 100);
      }
      else
      {
        set_dir_state(&stepper_motor, 0);
        mapped_up = map_range(up_val, 128, 255, low_rpm, high_rpm);
      HAL_UART_Transmit(&huart2, (uint8_t *)messageD, strlen(messageD), 100);
      }
      set_rpm(&stepper_motor, mapped_up);
      toggle = true;
    }
    else
    {
      toggle = false;
    }

    //If the joystick is moved move the motor
    if (toggle && !stepper_motor.steps_remaining)
    {
      move_stepper_deg(&stepper_motor, deg);
    }

    osMutexRelease(mPS2DataHandle);
    osDelay(10);
  }
  /* USER CODE END StartStepperMotor2 */
}

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
