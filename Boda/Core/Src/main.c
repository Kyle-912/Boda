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
#include "A4988.h"
#include "robot_arm.h"
#include "stm32f4xx_hal_tim.h"

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
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* Definitions for PS2DataUpdate */
osThreadId_t PS2DataUpdateHandle;
const osThreadAttr_t PS2DataUpdate_attributes = {
    .name = "PS2DataUpdate",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for RobotArm */
osThreadId_t RobotArmHandle;
const osThreadAttr_t RobotArm_attributes = {
    .name = "RobotArm",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Attachment */
osThreadId_t AttachmentHandle;
const osThreadAttr_t Attachment_attributes = {
    .name = "Attachment",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Bluetooth */
osThreadId_t BluetoothHandle;
const osThreadAttr_t Bluetooth_attributes = {
    .name = "Bluetooth",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for mPS2Data */
osMutexId_t mPS2DataHandle;
const osMutexAttr_t mPS2Data_attributes = {
    .name = "mPS2Data"};
/* Definitions for mAttachmentData */
osMutexId_t mAttachmentDataHandle;
const osMutexAttr_t mAttachmentData_attributes = {
    .name = "mAttachmentData"};
/* USER CODE BEGIN PV */

PS2ControllerHandler ps2;
float rpm = 300;
short microsteps = FULL_STEPS;
double deg = 20;
const short spr = 200; // Steps per revolution
stepper *motor1 = NULL;
stepper *motor2 = NULL;
stepper *motor3 = NULL;
stepper *motor4 = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM13_Init(void);
void StartPS2DataUpdate(void *argument);
void StartRobotArm(void *argument);
void StartAttachment(void *argument);
void StartBluetooth(void *argument);

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
  MX_SPI3_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Enable TIM3 global Interrupt & set priority
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  //----------PS2 INIT----------//
  PS2_Init(&ps2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mPS2Data */
  mPS2DataHandle = osMutexNew(&mPS2Data_attributes);

  /* creation of mAttachmentData */
  mAttachmentDataHandle = osMutexNew(&mAttachmentData_attributes);

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

  /* creation of RobotArm */
  RobotArmHandle = osThreadNew(StartRobotArm, NULL, &RobotArm_attributes);

  /* creation of Attachment */
  AttachmentHandle = osThreadNew(StartAttachment, NULL, &Attachment_attributes);

  /* creation of Bluetooth */
  BluetoothHandle = osThreadNew(StartBluetooth, NULL, &Bluetooth_attributes);

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
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
  htim1.Init.Prescaler = 72 - 1;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Base_SLEEP_Pin | Elbow_SLEEP_Pin | Elbow_STEP_Pin | Elbow_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Shoulder_DIR_Pin | Shoulder_STEP_Pin | PS2_Controller_Chip_Select_Pin | PS2_Controller_Acknowledge_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Attachment_GPIO_Pin | Shoulder_SLEEP_Pin | Base_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Base_STEP_Pin */
  GPIO_InitStruct.Pin = Base_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Base_STEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Base_SLEEP_Pin Elbow_SLEEP_Pin Elbow_STEP_Pin Elbow_DIR_Pin */
  GPIO_InitStruct.Pin = Base_SLEEP_Pin | Elbow_SLEEP_Pin | Elbow_STEP_Pin | Elbow_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Shoulder_DIR_Pin Shoulder_STEP_Pin PS2_Controller_Chip_Select_Pin PS2_Controller_Acknowledge_Pin */
  GPIO_InitStruct.Pin = Shoulder_DIR_Pin | Shoulder_STEP_Pin | PS2_Controller_Chip_Select_Pin | PS2_Controller_Acknowledge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Attachment_GPIO_Pin Shoulder_SLEEP_Pin Base_DIR_Pin */
  GPIO_InitStruct.Pin = Attachment_GPIO_Pin | Shoulder_SLEEP_Pin | Base_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Callback function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  stepper *current_motor = NULL;

  // Determine which motor's timer has elapsed
  if (htim->Instance == motor1->timer->Instance)
    current_motor = motor1;
  else if (htim->Instance == motor2->timer->Instance)
    current_motor = motor2;
  else if (htim->Instance == motor3->timer->Instance)
    current_motor = motor3;
  // Add conditions for motor3 and motor4 if needed

  if (current_motor)
  {
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

void PS2_Init(PS2ControllerHandler *ps2)
{
  ps2->Ack_GPIO = GPIOA;
  ps2->Ack_PIN = GPIO_PIN_14;
  ps2->CS_GPIO = GPIOA;
  ps2->CS_PIN = GPIO_PIN_13;
  ps2->spi = &hspi2;
  ps2->tim = &htim1;
  HAL_GPIO_WritePin(ps2->Ack_GPIO, ps2->Ack_PIN, GPIO_PIN_SET);
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

/* USER CODE BEGIN Header_StartRobotArm */
/**
 * @brief Function implementing the RobotArm thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRobotArm */
void StartRobotArm(void *argument)
{
  /* USER CODE BEGIN StartRobotArm */

  // Motor 1
  // stepper stepper_motor_1;
  // motor1 = &stepper_motor_1;
  // init_stepper(motor1, spr);
  // init_dir_pin(motor1, motor1_dir_port, motor1_dir_pin);
  // init_step_pin(motor1, motor1_step_port, motor1_step_pin);
  // init_sleep_pin(motor1, motor1_sleep_port, motor1_sleep_pin);
  // set_micro_en(motor1, 0);
  // set_timer(motor1, &htim3);
  // set_rpm(motor1, rpm);

  // // Motor 2
  // stepper stepper_motor_2;
  // motor2 = &stepper_motor_2;
  // init_stepper(motor2, spr);
  // init_dir_pin(motor2, motor2_dir_port, motor2_dir_pin);
  // init_step_pin(motor2, motor2_step_port, motor2_step_pin);
  // init_sleep_pin(motor2, motor2_sleep_port, motor2_sleep_pin);
  // set_micro_en(motor2, 0);
  // set_timer(motor2, &htim14);
  // set_rpm(motor2, rpm);

  // // Motor 2
  // stepper stepper_motor_3;
  // motor3 = &stepper_motor_3;
  // init_stepper(motor3, spr);
  // init_dir_pin(motor3, motor3_dir_port, motor3_dir_pin);
  // init_step_pin(motor3, motor3_step_port, motor3_step_pin);
  // init_sleep_pin(motor3, motor3_sleep_port, motor3_sleep_pin);
  // set_micro_en(motor3, 0);
  // set_timer(motor3, &htim13);
  // set_rpm(motor3, rpm);

  // double mapped_left = 0;
  // double mapped_up = 0;

  // arm robot_arm_var;
  // arm *robot_arm = &robot_arm_var;
  // // init_arm_2(robot_arm, 200.0f, motor1, motor2);
  // init_arm(robot_arm, 200.0, 3, motor1, motor2, motor3);

  // home(robot_arm);

  // // set_coordinate(robot_arm, 0, 10, 10);
  // set_coordinate(robot_arm, 0, 3, 2 * 80, 2 * 80, 2 * 80);
  // set_coordinate(robot_arm, 1, 3, 120 * 80, 200 * 80, 160 * 80);
  // set_coordinate(robot_arm, 2, 3, 30 * 80, 60 * 80, 45 * 80);
  // uint8_t coord = 0;
  // uint8_t rpm_step = 12;

  // int8_t flip = 1;

  /* Infinite loop */
  for (;;)
  {
    // osMutexWait(mPS2DataHandle, 10);

    // if (coord == 0)
    // {
    //   set_arm_rpm(robot_arm, 450);
    //   move(robot_arm, 0);
    //   coord = 1;
    // }
    // else if (coord == 1)
    // {
    //   set_arm_rpm(robot_arm, 450);
    //   move(robot_arm, 1);
    //   coord = 2;
    // }
    // else if (coord == 2)
    // {
    //   set_arm_rpm(robot_arm, 450);
    //   move(robot_arm, 2);
    //   coord = 0;
    // }
    // // else if (coord == 3)
    // // {
    // //   set_arm_rpm(robot_arm, 100);
    // //   move(robot_arm, 3);
    // //   coord = 0;
    // // }

    // while (motor1->steps_remaining || motor2->steps_remaining || motor3->steps_remaining)
    //   ;

    // osMutexRelease(mPS2DataHandle);
    // osDelay(1);
  }
  /* USER CODE END StartRobotArm */
}

/* USER CODE BEGIN Header_StartAttachment */
/**
 * @brief Function implementing the Attachment thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAttachment */
void StartAttachment(void *argument)
{
  /* USER CODE BEGIN StartAttachment */

  /*****
   * TODO:
   *  - Enum the states of the commection protocol
   *    - Wait :
   *        > While the received SPI transmission is 0 send command 0x64
   *    - Conntecting :
   *        > After a response is received for the first time, ask for the buttons it would like to use as inputs
   *        > Will be 8 transmissions, one for each button? or 4 with two buttons being sent at the same time.
   *    - Input:
   *        > Will now send SPI transactions every milisecond to get the WHO_AM_I response
   *        > IF nothing is received -> go to the disconnected state
   *        > If there is something to transmit, go to command or input
   *    - Disconnected:
   *        > Any memory stuff to make sure it recognizes that it isnt connected anymore
   */

  /* Infinite loop */
  uint16_t transmit = 0;
  uint16_t received = 0;
  uint8_t lowByte;
  uint8_t highByte;
  uint8_t command = 0x64;

  enum States
  {
    Wait,
    Identify,
    Connecting,
    Input,
    Disconnected
  };

  enum States curState = Wait;

  uint8_t buttons[8] = {R1, R2, L1, L2, X, CIRCLE, SQUARE, TRIANGLE};
  uint8_t buttonsBuffer[9] = {0, 0, 0, 0, 0, 0, 0, 0, '\n'};
  uint8_t buffer[3] = {0, 0, '\n'};
  uint8_t buttonsGotten = 0;

  bool awaitResponse = false;
  uint8_t commandDelay = 0;
  uint8_t baseDelay = 3;

  uint8_t temp = 0;
  uint16_t ID = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

  for (;;)
  {
    transmit = 0;

    // Set the proper commmand settings for the current state
    if (awaitResponse == false)
    {
      switch (curState)
      {
      case Wait:
        // always waiting for a response with no delay: Waiting for 0x01 received
        command = 0;
        awaitResponse = true;
        commandDelay = 0;
        buttonsGotten = 0;
        ID = 0;
        break;
      case Identify:
        command = 0x64;
        commandDelay = baseDelay;
        awaitResponse = true;
        buttonsGotten = 0;
        break;
      case Connecting:
        if (buttonsGotten == 0)
        {
          command = 0x01;
        }
        else if (buttonsGotten == 2)
        {
          command = 0x02;
        }
        else if (buttonsGotten == 4)
        {
          command = 0x03;
        }
        else if (buttonsGotten == 6)
        {
          command = 0x04;
        }
        commandDelay = baseDelay;
        awaitResponse = true;
        break;
      case Input:
        command = 0;
        commandDelay = 0;
        awaitResponse = false;
        // Get the PS2Data Mutex
        osMutexWait(mPS2DataHandle, 10);
        for (int i = 0; i < 8; i++)
        {
          transmit |= (Is_Button_Pressed(&ps2, buttons[i]) << i);
        }
        // The mutex can be released here as it is no longer needed to be held. Helps free up time
        osMutexRelease(mPS2DataHandle);
        break;
      case Disconnected:
        // send the emergency response command
        command = 0x0E;
        awaitResponse = true;
        commandDelay = baseDelay;
        break;
      default:
        break;
      }
    }

    transmit |= ((uint16_t)(command) << 8);
    // Transmit the Input, Send Command
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&transmit, (uint8_t *)&received, 1, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

    // check for Disconnection, receiving 0 is always bad:
    if (received == 0 && curState != Wait && curState != Disconnected)
    {
      curState = Disconnected;
      awaitResponse = false;
    }
    // if a command is waiting for a response, and there is still a delay, decrease the delay
    else if (awaitResponse == true && commandDelay > 0)
    {
      commandDelay--;
    }
    // Whether or not a response is waiting to be received, though if one is, the delay is over
    else
    {
      // check/change states
      switch (curState)
      {
      case Wait:
        // if the 0x01 we wanted is received:
        if (received == 0x01)
        {
          curState = Identify;
        }

        break;
      case Identify:
        // if response is valid?
        if (received != 0x01)
        {
          curState = Connecting;

          temp = (uint8_t)(received & 0xFF);
          ID = (received >> 8);
          ID |= ((uint16_t)temp << 8);
          //Test Code ------------------------------
          HAL_UART_Transmit(&huart2, "\nID: ", 6, 1);
          HAL_UART_Transmit(&huart2, (uint8_t *)&ID, 2, 1);
          HAL_UART_Transmit(&huart2, "\n", 1, 1);
          //Test Code ------------------------------
        }
        else
        {
          curState = Wait;
        }
        break;
      case Connecting:

        // TEST CODE---------------------------------------------
        // for (int i = 0; i < 2; i++)
        // {
        //   if (i == 0)
        //   {
        //     temp = (uint8_t)(received & 0xFF);
        //   }
        //   else
        //   {
        //     temp = (uint8_t)((received >> 8) & 0xFF);
        //   }
        //   switch (temp)
        //   {
        //   case X:
        //     buffer[i] = 'X';
        //     break;
        //   case CIRCLE:
        //     buffer[i] = 'O';
        //     break;
        //   case TRIANGLE:
        //     buffer[i] = '^';
        //     break;
        //   case SQUARE:
        //     buffer[i] = 'S';
        //     break;
        //   case R1:
        //     buffer[i] = 'r';
        //     break;
        //   case R2:
        //     buffer[i] = 'R';
        //     break;
        //   case L1:
        //     buffer[i] = 'l';
        //     break;
        //   case L2:
        //     buffer[i] = 'L';
        //     break;
        //   default:
        //     buffer[i] = 'G';
        //     break;
        //   }
        // }
        // HAL_UART_Transmit(&huart2, (uint8_t *)&buffer, 3, 1);
        // TEST CODE---------------------------------------------

        lowByte = (uint8_t)(received & 0xFF);
        highByte = (uint8_t)((received >> 8) & 0xFF);
        // If Not Valid Data Go to Disconnected cause comething might be wrong
        if (lowByte == X || lowByte == CIRCLE || lowByte == TRIANGLE || lowByte == SQUARE || lowByte == R1 || lowByte == R2 || lowByte == L1 || lowByte == L2)
        {
          if (highByte == X || highByte == CIRCLE || highByte == TRIANGLE || highByte == SQUARE || highByte == R1 || highByte == R2 || highByte == L1 || highByte == L2)
          {
            buttons[buttonsGotten++] = lowByte;
            buttons[buttonsGotten++] = highByte;
          }
        }

        // If all buttons have been gotten go to Input State
        if (buttonsGotten == 8)
        {
          curState = Input;

          // TEST CODE---------------------------------------------
          HAL_UART_Transmit(&huart2, "\n", 1, 1);
          for (int i = 0; i < 8; i++)
          {
            switch (buttons[i])
            {
            case X:
              buttonsBuffer[i] = 'X';
              break;
            case CIRCLE:
              buttonsBuffer[i] = 'O';
              break;
            case TRIANGLE:
              buttonsBuffer[i] = '^';
              break;
            case SQUARE:
              buttonsBuffer[i] = 'S';
              break;
            case R1:
              buttonsBuffer[i] = 'r';
              break;
            case R2:
              buttonsBuffer[i] = 'R';
              break;
            case L1:
              buttonsBuffer[i] = 'l';
              break;
            case L2:
              buttonsBuffer[i] = 'L';
              break;
            default:
              buttonsBuffer[i] = 'G';
              break;
            }
          }
          HAL_UART_Transmit(&huart2, (uint8_t *)&buttonsBuffer, 9, 1);
          // TEST CODE---------------------------------------------
        }
        break;
      case Input:
        break;
      case Disconnected:
        if (received == 0xAA)
        {
          curState = Input;
        }
        else
        {
          curState = Wait;
        }
        break;
      default:
        break;
      }
      awaitResponse = false;
      received = 0;
    }

    osDelay(1);
  }
  /* USER CODE END StartAttachment */
}

/* USER CODE BEGIN Header_StartBluetooth */
/**
 * @brief Function implementing the Bluetooth thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBluetooth */
void StartBluetooth(void *argument)
{
  /* USER CODE BEGIN StartBluetooth */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBluetooth */
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

#ifdef USE_FULL_ASSERT
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
