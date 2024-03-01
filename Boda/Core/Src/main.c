/* USER CODE BEGIN Header */
/**
 * ***********************
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
#include "A4988.h"
#include "robot_arm.h"
#include "stm32f4xx_hal_tim.h"
#include <stdbool.h>

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


// Controller Input Processing
#define UPDATE_BUTTONS_SIGNAL 0x01

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
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for BLE */
osThreadId_t BLEHandle;
const osThreadAttr_t BLE_attributes = {
  .name = "BLE",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Arm_Control */
osThreadId_t Arm_ControlHandle;
const osThreadAttr_t Arm_Control_attributes = {
  .name = "Arm_Control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

// Variables for PS2
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

// Variables for controller processing:


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM13_Init(void);
static void MX_USART1_UART_Init(void);
void Start_BLE(void *argument);
void Start_Arm_Control(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_SPI3_Init();
  MX_TIM13_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  // HAL_UART_Receive_IT(&huart2, &rxByte, 1);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Enable TIM3 global Interrupt & set priority

  // HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  // HAL_NVIC_EnableIRQ(USART2_IRQn);

  // HAL_NVIC_SetPriority(USART2_IRQn, 10, 0); // Set interrupt priority as needed
  // HAL_NVIC_EnableIRQ(USART2_IRQn);

  // HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  // HAL_NVIC_EnableIRQ(USART1_IRQn);
  //----------PS2 INIT----------//

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of BLE */
  BLEHandle = osThreadNew(Start_BLE, NULL, &BLE_attributes);

  /* creation of Arm_Control */
  Arm_ControlHandle = osThreadNew(Start_Arm_Control, NULL, &Arm_Control_attributes);

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
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  huart1.Init.BaudRate = 9600;
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Attachment_GPIO_Pin|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC5 PC6
                           PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Attachment_GPIO_Pin PB6 PB7 */
  GPIO_InitStruct.Pin = Attachment_GPIO_Pin|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void jog_motor(stepper *motor, float rpm, bool dir, int cont_amount)
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // Notify the BLE task that data has been received
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(BLEHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// Callback function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  callback_pulse(robot_arm, htim);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_BLE */
/**
* @brief Function implementing the BLE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_BLE */
void Start_BLE(void *argument)
{
  /* USER CODE BEGIN 5 */

  // Set the NVIC priority for USART1 interrupts to 5, with 0 subpriority.
  // Priorities lower than the RTOS max syscall interrupt priority
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);

  // Enable the interrupt request (IRQ) for USART1.
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  // Begin asynchronous reception on USART1 using interrupt mode.
  HAL_UART_Receive_IT(&huart1, RxBuffer, sizeof(RxBuffer));

  // Task loop
  for(;;) 
  {
    // Wait indefinitely for a notification from the UART interrupt callback. 
    // The notification indicates that data has been received. 
    // If ulTaskNotifyTake returns a value greater than 0, 
    // it means a notification was received successfully.
    if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) 
    {
      // Prepare to receive more data
      HAL_UART_Receive_IT(&huart1, RxBuffer, sizeof(RxBuffer));
    }

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_Arm_Control */
/**
 * @brief Function implementing the Arm_Control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Arm_Control */
void Start_Arm_Control(void *argument)
{
  /* USER CODE BEGIN Start_Arm_Control */

  // Function Variables
  float rpm = 300;
  short microsteps = FULL_STEPS;
  double deg = 20;
  // const short spr = 200; // Steps per revolution

  stepper *stepper_motor1 = pvPortMalloc(sizeof(stepper));
  if (stepper_motor1 == NULL) {
      // Handle memory allocation error
  }
  
  stepper *stepper_motor2 = pvPortMalloc(sizeof(stepper));
  if (stepper_motor2 == NULL) {
      // Handle memory allocation error
  }

  stepper *stepper_motor3 = pvPortMalloc(sizeof(stepper));
  if (stepper_motor3 == NULL) {
      // Handle memory allocation error
  }

  // // Motor 3
  motor3 = stepper_motor3;
  init_stepper(motor3, 140);
  init_dir_pin(motor3, motor3_dir_port, motor3_dir_pin);
  init_step_pin(motor3, motor3_step_port, motor3_step_pin);
  init_sleep_pin(motor3, motor3_sleep_port, motor3_sleep_pin);
  set_micro_en(motor3, 0);
  set_timer(motor3, &htim13);
  set_rpm(motor3, rpm);

  // // Motor 2
  motor2 = stepper_motor2;
  init_stepper(motor2, 120);
  init_dir_pin(motor2, motor2_dir_port, motor2_dir_pin);
  init_step_pin(motor2, motor2_step_port, motor2_step_pin);
  init_sleep_pin(motor2, motor2_sleep_port, motor2_sleep_pin);
  set_micro_en(motor2, 0);
  set_timer(motor2, &htim14);
  set_rpm(motor2, rpm);

  // // Motor 1
  motor1 = stepper_motor1;
  init_stepper(motor1, 200);
  init_dir_pin(motor1, motor1_dir_port, motor1_dir_pin);
  init_step_pin(motor1, motor1_step_port, motor1_step_pin);
  init_sleep_pin(motor1, motor1_sleep_port, motor1_sleep_pin);
  set_micro_en(motor1, 0);
  set_timer(motor1, &htim3);
  set_rpm(motor1, rpm);
  // set_step_limit(motor1, 100);


  // // Robot Arm
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
  /* USER CODE END Start_Arm_Control */
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
