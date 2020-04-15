/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef LPM_ON_IND
#define LPM_ON_IND()
#define LPM_OFF_IND()
#endif /* LPM_ON_IND */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LPTIM_HandleTypeDef hlptim1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
extern bool hs_timer_initialized;
extern uint32_t hs_timer_counter_extension;
extern bool tim15_initialized;
extern uint16_t counter_extension_tim15;
extern bool tim5_initialized;
extern uint32_t counter_extension_tim5;

char debug_print_buffer[256];
op_mode_t op_mode = OP_MODE_RESET;
const op_mode_t op_mode_sm[NUM_OP_MODES][NUM_OP_MODE_EVENTS] =
{
  /* event:             OP_MODE_EVT_INIT, OP_MODE_EVT_WAKEUP, OP_MODE_EVT_DONE, OP_MODE_EVT_STOPPED, OP_MODE_EVT_RESTORED */
  /* state RESET: */  { OP_MODE_ACTIVE,   OP_MODE_RESET,      OP_MODE_RESET,    OP_MODE_RESET,       OP_MODE_RESET   },
  /* state ACTIVE: */ { OP_MODE_RESET,    OP_MODE_ACTIVE,     OP_MODE_IDLE,     OP_MODE_RESET,       OP_MODE_RESET   },
  /* state IDLE: */   { OP_MODE_RESET,    OP_MODE_ACTIVE,     OP_MODE_IDLE,     OP_MODE_LPM,         OP_MODE_RESET   },
  /* state LPM: */    { OP_MODE_RESET,    OP_MODE_WOKEN,      OP_MODE_RESET,    OP_MODE_RESET,       OP_MODE_RESET   },
  /* state WOKEN: */  { OP_MODE_RESET,    OP_MODE_WOKEN,      OP_MODE_RESET,    OP_MODE_RESET,       OP_MODE_ACTIVE  },
  /* NOTE: for all invalid transitions, go back to RESET state */
};
lp_mode_t lp_mode = LOW_POWER_MODE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM15_Init(void);
static void MX_LPTIM1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool swo_println(const char* str)
{
  while (*str)
  {
    ITM_SendChar(*str);
    str++;
  }
  ITM_SendChar('\r');
  ITM_SendChar('\n');

  return true;
}

bool swo_print(const char* str)
{
  while (*str)
  {
    ITM_SendChar(*str);
    str++;
  }
  return true;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  system_boot();

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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */

  system_init();

  /* initialize state machine for handling low-power modes */
  op_mode = op_mode_sm[op_mode][OP_MODE_EVT_INIT];
  /* disable PC13 (RF_DIO1) interrupt (only needed for wakeup from LPM) */
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* FreeRTOS tasks, queues, etc. */
  RTOS_Init();

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */
  HAL_LPTIM_Counter_Start_IT(&hlptim1, 0xffff);
  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0xFFFFFFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 2 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1024;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BOLT_REQ_Pin|BOLT_MODE_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RADIO_NRESET_Pin|RADIO_ANT_SW_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(COM_GPIO1_GPIO_Port, COM_GPIO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RADIO_DIO1_WAKEUP_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO1_WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_DIO1_WAKEUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOLT_IND_Pin APP_IND_Pin RADIO_BUSY_Pin */
  GPIO_InitStruct.Pin = BOLT_IND_Pin|APP_IND_Pin|RADIO_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOLT_ACK_Pin */
  GPIO_InitStruct.Pin = BOLT_ACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOLT_ACK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOLT_REQ_Pin BOLT_MODE_Pin */
  GPIO_InitStruct.Pin = BOLT_REQ_Pin|BOLT_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB4 PB5 
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_NSS_Pin */
  GPIO_InitStruct.Pin = RADIO_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RADIO_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_NRESET_Pin */
  GPIO_InitStruct.Pin = RADIO_NRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RADIO_NRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_ANT_SW_Pin */
  GPIO_InitStruct.Pin = RADIO_ANT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RADIO_ANT_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COM_GPIO1_Pin */
  GPIO_InitStruct.Pin = COM_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(COM_GPIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void prepare_lpm(void)
{
  /* --- prepare the MCU for low power mode --- */
  if (op_mode == OP_MODE_IDLE)
  {
    if (lp_mode == LP_MODE_SLEEP) {
      // do not update op_mode since we are already in IDLE and we are not entering a real LPM state
      HAL_SuspendTick();
    }
    else if ( (lp_mode == LP_MODE_STOP2)   ||
              (lp_mode == LP_MODE_STANDBY) ||
              (lp_mode == LP_MODE_SHUTDOWN)     )
    {
      /* make sure the radio is in sleep mode */
      radio_sleep(false);

      /* notes on stop mode:
      * - SRAM1, SRAM2 and all registers content are retained
      * - all clocks in the VCORE domain are stopped, the PLL, the MSI, the HSI16 and the HSE are disabled
      * - LSE and LPTIM keep running
      * - all I/O pins keep the state
      */
      __set_BASEPRI( (TICK_INT_PRIORITY + 1) << (8 - __NVIC_PRIO_BITS) );   /* or:  __disable_irq() */
      __DSB();
      __ISB();

      __HAL_RCC_PWR_CLK_ENABLE();

      CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);  /* suspend FreeRTOS SysTick */
      HAL_SuspendTick();                                  /* suspend HAL tick */

      /* disable all unused peripherals */
      __HAL_TIM_DISABLE(&htim2);
      __HAL_TIM_DISABLE(&htim15);
      __HAL_TIM_DISABLE(&htim16);
      __HAL_UART_DISABLE(&huart1);
      __HAL_SPI_DISABLE(&hspi1);
      __HAL_SPI_DISABLE(&hspi2);

      /* configure HSI as clock source (16MHz) */
      RCC_OscInitTypeDef RCC_OscInitStruct = {0};
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
      RCC_OscInitStruct.OscillatorType        = RCC_OSCILLATORTYPE_HSI;
      RCC_OscInitStruct.HSIState              = RCC_HSI_ON;
      RCC_OscInitStruct.HSICalibrationValue   = RCC_HSICALIBRATION_DEFAULT;
      RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_NONE;
      if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
      /* select HSI as system clock */
      RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
      RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_HSI;
      if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
      HAL_RCCEx_WakeUpStopCLKConfig(RCC_STOP_WAKEUPCLOCK_HSI);
      /* disable MSI */
      RCC_OscInitStruct.OscillatorType    = RCC_OSCILLATORTYPE_MSI;
      RCC_OscInitStruct.MSIState          = RCC_MSI_OFF;
      RCC_OscInitStruct.PLL.PLLState      = RCC_PLL_NONE;
      if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

      /* configure unused GPIOs for minimal current drain (make sure there are no floating inputs) */
      GPIO_InitTypeDef GPIO_InitStruct = {0}; // this is used further down!
      // NOTE1: Not necessary at the moment since pins are either not used or by default already configured as input
      // NOTE2: For some constellation the analog mode causes a higher power consumpton than leaving it configured as input
      // NOTE3: restore of RADIO_DIO1_Pin breaks wakeup from LPM, reason unknown (PA15 is connected to PC13 in hardware on the COM board)
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = RADIO_DIO1_Pin;
      // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = RADIO_BUSY_Pin;
      // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = BOLT_SCK_Pin|BOLT_MOSI_Pin|BOLT_MISO_Pin;
      // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = UART_TX_Pin|UART_RX_Pin;
      // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      //// disabling Radio SPI pins causes floating pin in low-power mode (reason unknown)
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = RADIO_SCK_Pin|RADIO_MISO_Pin|RADIO_MOSI_Pin;   // SPI for radio: RADIO_SCK_Pin|RADIO_MISO_Pin|RADIO_MOSI_Pin
      // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      //// RADIO_DIO1 (PC13) should not be disabled in low power mode -> it would no longer be possible to wake up from STOP2 with a radio interrupt
      // // GPIO_InitStruct.Pin = RADIO_DIO1_WAKEUP_Pin;        // HSE pins: GPIO_PIN_14|GPIO_PIN_15
      // // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

      /* turn off LEDs */
      led_off(LED_EVENT);
      led_off(LED_SYSTEM);
      PIN_CLR(COM_GPIO1);     /* has external pulldown */

      /* configure BOLT TREQ in EXTI mode */
      GPIO_InitStruct.Pin = COM_TREQ_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      HAL_GPIO_Init(COM_TREQ_GPIO_Port, &GPIO_InitStruct);
      HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(EXTI3_IRQn);

      /* disable and clear unused interrupts */
      HAL_NVIC_DisableIRQ(USART1_IRQn);
      HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
      HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
      HAL_NVIC_DisableIRQ(SPI1_IRQn);
      HAL_NVIC_DisableIRQ(SPI2_IRQn);
      HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
      HAL_NVIC_DisableIRQ(TIM2_IRQn);

      /* configure RF_DIO1 on PC13 interrupt for wakeup from LPM */
      __HAL_GPIO_EXTI_CLEAR_IT(RADIO_DIO1_WAKEUP_Pin); // important for low-power consumption in STOP2 mode -> see README
      if(lp_mode == LP_MODE_STOP2) {
        HAL_NVIC_SetPriority(RADIO_DIO1_WAKEUP_EXTI_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(RADIO_DIO1_WAKEUP_EXTI_IRQn);
      }

      /* disable LPTIM1 (necessary to enter STANDBY or SHUTDOWN) (still did not work so far) */
      if (lp_mode == LP_MODE_STANDBY || lp_mode == LP_MODE_SHUTDOWN) {
        __HAL_LPTIM_DISABLE(&hlptim1);
      }

      if (lp_mode == LP_MODE_STOP2) {
        // Clear flags of all WAKEUP lines (necessary for STOP2 as well?)
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

        MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP2);    /* set Stop mode 2 */
      }
      else if (lp_mode == LP_MODE_STANDBY) {
        // Clear flags of all WAKEUP lines (necessary to successfully enter STANDBY mode after sending with radio)
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

        MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STANDBY);  /* set Standby mode */
      }
      else if (lp_mode == LP_MODE_SHUTDOWN) {
        // Clear flags of all WAKEUP lines (necessary to successfully enter SHUTDOWN mode after sending with radio)
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

        MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_SHUTDOWN); /* set Shutdown mode */
      }
      SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));     /* set SLEEPDEEP bit */

      LPM_ON_IND();

      /* clock gating (-> shouldn't be necessary if clock source is disabled!) */
      /*__HAL_RCC_GPIOA_CLK_DISABLE();
      __HAL_RCC_GPIOB_CLK_DISABLE();
      __HAL_RCC_GPIOC_CLK_DISABLE();
      __HAL_RCC_GPIOD_CLK_DISABLE();
      __HAL_RCC_GPIOE_CLK_DISABLE();
      __HAL_RCC_GPIOH_CLK_DISABLE();
      __HAL_RCC_DMA1_CLK_DISABLE();
      __HAL_RCC_FLASH_CLK_DISABLE();
      __HAL_RCC_SYSCFG_CLK_DISABLE();
      __HAL_RCC_PWR_CLK_DISABLE();*/

      op_mode = op_mode_sm[op_mode][OP_MODE_EVT_STOPPED];
      __set_BASEPRI(0);     /* re-enable interrupts */
    }
  }
}


void resume_from_lpm(void)
{
  if (op_mode == OP_MODE_IDLE) {
    /* MCU was in sleep mode, only tick needs to be restored */
    HAL_ResumeTick();
    // do not update op_mode since we are already in IDLE
  }
  else if (op_mode == OP_MODE_WOKEN) {
    /* MCU was in STOP2, STANDBY, or SHUTDOWN mode, different components need to be restored */

    /* make sure the following code runs atomically */
    __set_BASEPRI( (TICK_INT_PRIORITY + 1) << (8 - __NVIC_PRIO_BITS) );   /* mask interrupts */
    __DSB();
    __ISB();

    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

    /* make sure all required clocks are enabled */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_FLASH_CLK_ENABLE();

    SystemClock_Config();                               /* restore clock config (and resume HAL tick) */

    /* restore GPIO config */
    // NOTE1: Not necessary at the moment since pins are either not used or by default already configured as input
    // NOTE2: For some constellation the analog mode causes a higher power consumpton than leaving it configured as input
    // NOTE3: restore of RADIO_DIO1_Pin breaks wakeup from LPM, reason unknown (PA15 is connected to PC13 in hardware on the COM board)
    // GPIO_InitTypeDef GPIO_InitStruct = {0};
    // GPIO_InitStruct.Pin  = RADIO_BUSY_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // GPIO_InitStruct.Pin = BOLT_SCK_Pin|BOLT_MOSI_Pin|BOLT_MISO_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // GPIO_InitStruct.Pin = UART_TX_Pin|UART_RX_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //// disabling Radio SPI pins causes floating pin in low-power mode (reason unknown)
    // GPIO_InitStruct.Pin = RADIO_SCK_Pin|RADIO_MISO_Pin|RADIO_MOSI_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // GPIO_InitStruct.Pin = RADIO_DIO1_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    // GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    // HAL_GPIO_Init(RADIO_DIO1A15_GPIO_Port, &GPIO_InitStruct);

    /* restore peripherals */
    __HAL_TIM_ENABLE(&htim2);
    __HAL_TIM_ENABLE(&htim15);
    __HAL_TIM_ENABLE(&htim16);
    __HAL_UART_ENABLE(&huart1);
    __HAL_SPI_ENABLE(&hspi1);
    __HAL_SPI_ENABLE(&hspi2);

    /* disable BOLT TREQ EXTI */
    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(COM_TREQ_Pin);

    /* re-enable interrupts */
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    /* disable RF_DIO1 on PC13 interrupt (only neede for wakeup from LPM) */
    HAL_NVIC_DisableIRQ(RADIO_DIO1_WAKEUP_EXTI_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(RADIO_DIO1_WAKEUP_Pin);

    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);    /* resume FreeRTOS SysTick */

    op_mode = op_mode_sm[op_mode][OP_MODE_EVT_RESTORED];
    LPM_OFF_IND();
    __set_BASEPRI(0);   /* enable interrupts */
    HAL_ResumeTick();

    /* wake the radio */
    radio_wakeup();
  }
}


#if USE_SWO
int _write(int32_t file, uint8_t *ptr, int32_t len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  /* return len; */
  uint32_t i;
  for(i = 0; i < len; i++)
  {
    ITM_SendChar(ptr[i]);
  }
  return len;
}
#endif /* USE_SWO */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  vTaskSuspend(NULL);       // we don't need this task
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2) {
    hs_timer_handle_overflow();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  led_on(LED_EVENT);
  while (1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
