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
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

UART_HandleTypeDef huart1;
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
    .stack_size = 256 * 4,
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
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for mPS2Data */
osMutexId_t mPS2DataHandle;
const osMutexAttr_t mPS2Data_attributes = {
    .name = "mPS2Data"};
/* Definitions for mAttachmentData */
osMutexId_t mAttachmentDataHandle;
const osMutexAttr_t mAttachmentData_attributes = {
    .name = "mAttachmentData"};
/* Definitions for mAttachmentCommand */
osMutexId_t mAttachmentCommandHandle;
const osMutexAttr_t mAttachmentCommand_attributes = {
    .name = "mAttachmentCommand"};
/* USER CODE BEGIN PV */

PS2ControllerHandler ps2;
uint8_t attachmentCommand = 0;
bool attachmentConnected = false;

enum States
{
  Wait,
  Identify,
  Connecting,
  Input,
  Disconnected
};

float rpm = 300;
short microsteps = FULL_STEPS;
double deg = 20;
const short spr = 200; // Steps per revolution

uint8_t vert_val;
uint8_t horiz_val;
// volatile uint8_t receivedFlag = 0; // Flag to indicate data reception
// volatile uint8_t rxByte;           // Byte received from UART

// variables for Motors
stepper *motor1 = NULL;
stepper *motor2 = NULL;
stepper *motor3 = NULL;
stepper *motor4 = NULL;

// Variables for Robot Arm
arm *robot_arm;

uint8_t current_coord = 0;

uint8_t RxBuffer[2] = {0}; // Buffer to store data received

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
static void MX_USART1_UART_Init(void);
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
  MX_USART1_UART_Init();
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

  /* creation of mAttachmentCommand */
  mAttachmentCommandHandle = osMutexNew(&mAttachmentCommand_attributes);

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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Base_SLEEP_Pin | Elbow_SLEEP_Pin | Elbow_STEP_Pin | Elbow_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Shoulder_DIR_Pin | Shoulder_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Attachment_GPIO_GPIO_Port, Attachment_GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PS2_Controller_CS_Pin | PS2_Controller_Ack_Pin | Shoulder_SLEEP_Pin | Base_DIR_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : Shoulder_DIR_Pin Shoulder_STEP_Pin */
  GPIO_InitStruct.Pin = Shoulder_DIR_Pin | Shoulder_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Blue_Button_Pin */
  GPIO_InitStruct.Pin = Blue_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Blue_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Attachment_GPIO_Pin */
  GPIO_InitStruct.Pin = Attachment_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Attachment_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PS2_Controller_CS_Pin PS2_Controller_Ack_Pin Shoulder_SLEEP_Pin Base_DIR_Pin */
  GPIO_InitStruct.Pin = PS2_Controller_CS_Pin | PS2_Controller_Ack_Pin | Shoulder_SLEEP_Pin | Base_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void jog_motor(stepper *motor, float rpm, bool dir, int cont_amount) // MOVE THIS!
{
  if (motor->steps_remaining)
  {
    motor->steps_remaining = cont_amount;
  }
  else
  {
    // move_stepper_steps(motor1, 500, 350.0);
    set_rpm(motor, rpm);
    if (dir)
    {
      move_stepper_deg(motor, 30.0);
    }
    else
    {
      move_stepper_deg(motor, -30.0);
    }
  }
}

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
  ps2->Ack_GPIO = PS2_Controller_Ack_GPIO_Port;
  ps2->Ack_PIN = PS2_Controller_Ack_Pin;
  ps2->CS_GPIO = PS2_Controller_CS_GPIO_Port;
  ps2->CS_PIN = PS2_Controller_CS_Pin;
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

    if (Is_DPad_Pressed(&ps2, DUP))
    {
      osMutexWait(mAttachmentCommandHandle, 50);
      attachmentCommand = 128;
      osMutexRelease(mAttachmentCommandHandle);
    }
    else if (Is_DPad_Pressed(&ps2, DDOWN))
    {
      osMutexWait(mAttachmentCommandHandle, 50);
      attachmentCommand = 129;
      osMutexRelease(mAttachmentCommandHandle);
    }
    else{
      osMutexWait(mAttachmentCommandHandle, 50);
      attachmentCommand = 0;
      osMutexRelease(mAttachmentCommandHandle);
    }

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
  // Function Variables
  float rpm = 300;
  short microsteps = FULL_STEPS;
  double deg = 20;
  // const short spr = 200; // Steps per revolution

  stepper *stepper_motor1 = pvPortMalloc(sizeof(stepper));
  if (stepper_motor1 == NULL)
  {
    // Handle memory allocation error
  }

  stepper *stepper_motor2 = pvPortMalloc(sizeof(stepper));
  if (stepper_motor2 == NULL)
  {
    // Handle memory allocation error
  }

  stepper *stepper_motor3 = pvPortMalloc(sizeof(stepper));
  if (stepper_motor3 == NULL)
  {
    // Handle memory allocation error
  }

  // Motor 3
  motor3 = stepper_motor3;
  init_stepper(motor3, 140);
  init_dir_pin(motor3, Elbow_DIR_GPIO_Port, Elbow_DIR_Pin);
  init_step_pin(motor3, Elbow_STEP_GPIO_Port, Elbow_STEP_Pin);
  init_sleep_pin(motor3, Elbow_SLEEP_GPIO_Port, Elbow_SLEEP_Pin);
  set_micro_en(motor3, 0);
  set_timer(motor3, &htim13);
  set_rpm(motor3, rpm);

  // Motor 2
  motor2 = stepper_motor2;
  init_stepper(motor2, 120);
  init_dir_pin(motor2, Shoulder_DIR_GPIO_Port, Shoulder_DIR_Pin);
  init_step_pin(motor2, Shoulder_STEP_GPIO_Port, Shoulder_STEP_Pin);
  init_sleep_pin(motor2, Shoulder_SLEEP_GPIO_Port, Shoulder_SLEEP_Pin);
  set_micro_en(motor2, 0);
  set_timer(motor2, &htim14);
  set_rpm(motor2, rpm);

  // Motor 1
  motor1 = stepper_motor1;
  init_stepper(motor1, 200);
  init_dir_pin(motor1, Base_DIR_GPIO_Port, Base_DIR_Pin);
  init_step_pin(motor1, Base_STEP_GPIO_Port, Base_STEP_Pin);
  init_sleep_pin(motor1, Base_SLEEP_GPIO_Port, Base_SLEEP_Pin);
  set_micro_en(motor1, 0);
  set_timer(motor1, &htim3);
  set_rpm(motor1, rpm);
  // set_step_limit(motor1, 100);

  // Robot Arm
  arm robot_arm_var;
  robot_arm = &robot_arm_var;
  init_arm(robot_arm, 200.0, 3, motor1, motor2, motor3);
  // init_arm(robot_arm, 200.0, 2, motor1, motor3);
  home(robot_arm);
  set_arm_rpm(robot_arm, 300);

  uint8_t RxBuffer_prev[2] = {0};

  /* Infinite loop */
  for (;;)
  {

    // IF jogging bit is set
    if (RxBuffer[0] & 0x80)
    {
      robot_arm->is_jogging = true;
      // IF bit for M1 movement set
      if (RxBuffer[0] & 0x1)
      {
        // Check DIR bit and move
        if (RxBuffer[0] & 0x2)
        {
          jog_motor(motor1, 40, 1, 50);
        }
        else
        {
          jog_motor(motor1, 40, 0, 50);
        }
      }

      // IF bit for M2 movement set
      if (RxBuffer[0] & 0x4)
      {
        // Check DIR bit and move
        if (RxBuffer[0] & 0x8)
        {
          // Check DIR bit and move
          jog_motor(motor2, 200, 1, 300);
        }
        else
        {
          jog_motor(motor2, 200, 0, 300);
        }
      }

      // IF bit for M3 movement set
      if (RxBuffer[0] & 0x10)
      {
        if (RxBuffer[0] & 0x20)
        {
          jog_motor(motor3, 200, 1, 300);
        }
        else
        {
          jog_motor(motor3, 200, 0, 300);
        }
      }

      // Save Position to Coordinate
      if ((RxBuffer[1] & 0x1))
      {
        if ((RxBuffer_prev[1] & 0x1) == 0)
        {
          save_coordinate(robot_arm);
        }
      }
    }
    // NOT Jogging
    else
    {
      if ((RxBuffer[1] & 0x80))
      {
        robot_arm->is_jogging = false;
        if (RxBuffer_prev[1] != RxBuffer[1])
        {
          uint8_t to_coord = (RxBuffer[1] >> 4) - 8;
          move(robot_arm, to_coord);
        }
      }
    }

    RxBuffer_prev[0] = RxBuffer[0];
    RxBuffer_prev[1] = RxBuffer[1];

    // If User Is Jogging Robot
    // =======================================================================================
    // ----------------------------------------Vertical---------------------------------------
    // -----------------------------------------Motor1----------------------------------------
    // if (robot_arm->is_jogging)
    // {
    //   vert_val = Is_Joystick_Left_Moved(&ps2, JOYSTICK_L_UD);
    //   horiz_val = Is_Joystick_Left_Moved(&ps2, JOYSTICK_L_RL);
    //   // If moving vertically
    //   if (vert_val != NEUTRAL)
    //   {
    //     // Vertical is less than neutral (Down)
    //     if (vert_val < NEUTRAL)
    //     {
    //       set_dir_state(motor1, 1);
    //       mapped_up = map_range(vert_val, 127, 0, low_rpm, high_rpm);
    //       HAL_UART_Transmit(&huart2, (uint8_t *)messageU, strlen(messageU), 100);
    //     }

    //     // Vertical is greater than neutral (up)
    //     else
    //     {
    //       set_dir_state(motor1, 0);
    //       mapped_up = map_range(vert_val, 127, 255, low_rpm, high_rpm);
    //       HAL_UART_Transmit(&huart2, (uint8_t *)messageD, strlen(messageD), 100);
    //     }

    //     // If the joystick is moved move the motor
    //     if (!motor1->steps_remaining)
    //     {
    //       set_rpm(motor1, mapped_up);
    //       move_stepper_deg(motor1, deg);
    //     }
    //   }
    //   // ---------------------------------------Horizontal--------------------------------------
    //   // -----------------------------------------Motor2----------------------------------------
    //   if (horiz_val != NEUTRAL)
    //   {
    //     if (horiz_val < NEUTRAL)
    //     {
    //       set_dir_state(motor2, 1);
    //       mapped_left = map_range(horiz_val, 127, 0, low_rpm, high_rpm);
    //       HAL_UART_Transmit(&huart2, (uint8_t *)messageL, strlen(messageL), 100);
    //     }

    //     else
    //     {
    //       set_dir_state(motor2, 0);
    //       mapped_left = map_range(horiz_val, 127, 255, low_rpm, high_rpm);
    //       HAL_UART_Transmit(&huart2, (uint8_t *)messageR, strlen(messageR), 100);
    //     }

    //     // If the joystick is moved move the motor
    //     if (!motor2->steps_remaining)
    //     {
    //       set_rpm(motor2, mapped_left);
    //       move_stepper_deg(motor2, deg);
    //     }
    //   }
    //   // =======================================================================================
    // }
    // else
    // {
    //   // USER IS NOT JOGGING -> POINT SWITCHING

    // }
    osDelay(1);
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

  /* Infinite loop */
  uint16_t transmit = 0;
  uint16_t received = 0;
  uint8_t lowByte;
  uint8_t highByte;

  enum States curState = Wait;

  uint8_t buttons[8] = {R1, R2, L1, L2, X, CIRCLE, SQUARE, TRIANGLE};
  uint8_t buttonsBuffer[9] = {0, 0, 0, 0, 0, 0, 0, 0, '\n'};
  uint8_t buffer[3] = {0, 0, '\n'};
  uint8_t buttonsGotten = 0;

  bool awaitResponse = false;
  uint8_t commandDelay = 0;
  uint8_t baseDelay = 5;

  uint8_t temp = 0;
  uint16_t ID = 0;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

  for (;;)
  {
    transmit = 0;
    // Acquire Attachment Command mutex
    osMutexWait(mAttachmentCommandHandle, 50);
    // Set the proper commmand settings for the current state
    if (awaitResponse == false)
    {
      switch (curState)
      {
      case Wait:
        // always waiting for a response with no delay: Waiting for 0x01 received
        attachmentCommand = 0;
        awaitResponse = true;
        commandDelay = 0;
        buttonsGotten = 0;
        attachmentConnected = false;
        ID = 0;
        break;
      case Identify:
        attachmentCommand = 0x64;
        commandDelay = baseDelay;
        awaitResponse = true;
        buttonsGotten = 0;
        break;
      case Connecting:
        if (buttonsGotten == 0)
        {
          attachmentCommand = 0x01;
        }
        else if (buttonsGotten == 2)
        {
          attachmentCommand = 0x02;
        }
        else if (buttonsGotten == 4)
        {
          attachmentCommand = 0x03;
        }
        else if (buttonsGotten == 6)
        {
          attachmentCommand = 0x04;
        }
        commandDelay = baseDelay;
        awaitResponse = true;
        attachmentConnected = false;
        break;
      case Input:
        commandDelay = 0;
        awaitResponse = false;
        attachmentConnected = true;

        if (attachmentCommand != 0)
        {
          commandDelay = baseDelay;
          awaitResponse = true;
        }

        break;
      case Disconnected:
        // send the emergency response command
        attachmentCommand = 0x0E;
        awaitResponse = true;
        commandDelay = baseDelay;
        attachmentConnected = false;
        break;
      default:
        break;
      }
    }
    // If an attachment is connected
    if (attachmentConnected)
    {
      // Get the PS2Data Mutex
      osMutexWait(mPS2DataHandle, 10);
      for (int i = 0; i < 8; i++)
      {
        // Get the states of the buttons the controller wants
        transmit |= (Is_Button_Pressed(&ps2, buttons[i]) << i);
      }
      // The mutex can be released here as it is no longer needed to be held. Helps free up time
      osMutexRelease(mPS2DataHandle);
    }
    transmit |= ((uint16_t)(attachmentCommand) << 8);

    // No longer need mutex for attachmentCommand
    osMutexRelease(mAttachmentCommandHandle);

    // Transmit the Input, Send Command
    HAL_GPIO_WritePin(Attachment_GPIO_GPIO_Port, Attachment_GPIO_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&transmit, (uint8_t *)&received, 1, 1);
    HAL_GPIO_WritePin(Attachment_GPIO_GPIO_Port, Attachment_GPIO_Pin, GPIO_PIN_SET);

    // check for Disconnection, receiving 0 is always bad unless its data:
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
        if (received == 0x0001)
        {
          curState = Identify;
          awaitResponse = false;
        }
        break;
      case Identify:
        // if response is valid?
        if (received != 0x01)
        {
          curState = Connecting;
          awaitResponse = false;

          temp = (uint8_t)(received & 0xFF);
          ID = (received >> 8);
          ID |= ((uint16_t)temp << 8);

          // Test Code ------------------------------
          // HAL_UART_Transmit(&huart2, "\nID: ", 6, 1);
          // HAL_UART_Transmit(&huart2, (uint8_t *)&ID, 2, 1);
          // HAL_UART_Transmit(&huart2, "\n", 1, 1);
          // Test Code ------------------------------
        }
        else
        {
          curState = Wait;
          awaitResponse = false;
        }
        break;
      case Connecting:

        lowByte = (uint8_t)(received & 0xFF);
        highByte = (uint8_t)((received >> 8) & 0xFF);
        // If Not Valid Data something might be wrong
        if (lowByte == X || lowByte == CIRCLE || lowByte == TRIANGLE || lowByte == SQUARE || lowByte == R1 || lowByte == R2 || lowByte == L1 || lowByte == L2)
        {
          if (highByte == X || highByte == CIRCLE || highByte == TRIANGLE || highByte == SQUARE || highByte == R1 || highByte == R2 || highByte == L1 || highByte == L2)
          {
            buttons[buttonsGotten] = lowByte;
            buttonsGotten += 1;
            buttons[buttonsGotten] = highByte;
            buttonsGotten += 1;
            awaitResponse = false;
          }
        }

        // If all buttons have been gotten go to Input State
        if (buttonsGotten == 8)
        {
          curState = Input;
          awaitResponse = false;
          attachmentConnected = true;
          // TEST CODE---------------------------------------------
          // HAL_UART_Transmit(&huart2, "\n", 1, 1);
          // for (int i = 0; i < 8; i++)
          // {
          //   switch (buttons[i])
          //   {
          //   case X:
          //     buttonsBuffer[i] = 'X';
          //     break;
          //   case CIRCLE:
          //     buttonsBuffer[i] = 'O';
          //     break;
          //   case TRIANGLE:
          //     buttonsBuffer[i] = '^';
          //     break;
          //   case SQUARE:
          //     buttonsBuffer[i] = 'S';
          //     break;
          //   case R1:
          //     buttonsBuffer[i] = 'r';
          //     break;
          //   case R2:
          //     buttonsBuffer[i] = 'R';
          //     break;
          //   case L1:
          //     buttonsBuffer[i] = 'l';
          //     break;
          //   case L2:
          //     buttonsBuffer[i] = 'L';
          //     break;
          //   default:
          //     buttonsBuffer[i] = 'G';
          //     break;
          //   }
          // }
          // HAL_UART_Transmit(&huart2, (uint8_t *)&buttonsBuffer, 9, 1);
          // TEST CODE---------------------------------------------
        }
        break;
      case Input:
        awaitResponse = false;
        break;
      case Disconnected:
        if (received == 0xAA)
        {
          curState = Input;
          awaitResponse = false;
        }
        else
        {
          curState = Wait;
          awaitResponse = false;
        }
        break;
      default:
        break;
      }
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
  // Set the NVIC priority for USART1 interrupts to 5, with 0 subpriority.
  // Priorities lower than the RTOS max syscall interrupt priority
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);

  // Enable the interrupt request (IRQ) for USART1.
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  // Begin asynchronous reception on USART1 using interrupt mode.
  HAL_UART_Receive_IT(&huart1, RxBuffer, sizeof(RxBuffer));

  // Task loop
  for (;;)
  {
    // Wait indefinitely for a notification from the UART interrupt callback.
    // The notification indicates that data has been received.
    // If ulTaskNotifyTake returns a value greater than 0,
    // it means a notification was received successfully.
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
    {
      // Prepare to receive more data
      HAL_UART_Receive_IT(&huart1, RxBuffer, sizeof(RxBuffer));
    }

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
