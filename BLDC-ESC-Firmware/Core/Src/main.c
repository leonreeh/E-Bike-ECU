/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*
 * Machine State
 */
int STATE = DEBUGST;
/*
 * Drive Varaibles
 */

int dir = 1;
int duty = 0;

/*
 * Speed measurment Varaibles
 */
int hallCC = 0;
float rpm = 0;
/*
 * ADC Variables
 */
uint16_t rawThrot =0;
//uint16_t pwmThrot =0;

//ADC data [Current,Voltage,Temp,speed]
float ADC_VAL[4];
/*
 * Digital input Variables
 */
//Button data[Light,Blinker L, Blinker R, Aux]
uint16_t but[4];

// Global variable to track time in SWFAULT state (in 100ms units)
uint32_t swfault_time_counter = 0;

/*
 * Display variables
 */
lcd_ar lcd_val;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
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

	//init button array
	but[0]=1;
	but[1]=1;
	but[2]=1;
	but[3]=1;

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /*
   * Start 100ms AUX Timer
   */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  //Close Relay
  HAL_GPIO_WritePin(GPIOB,PB5_DO_DC_ON_Pin,GPIO_PIN_RESET);
  //Init Blinker
  TIM3->CCR1 = 250;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  TIM3->CCR2 = 250;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

   HD44780_Init(4);
   HD44780_Clear();

   HD44780_SetCursor(6,0);
   HD44780_PrintStr("Init...");
   HAL_Delay(250);

   HD44780_SetCursor(5,1);
   HD44780_PrintStr("BLDC DRIVE");
   HAL_Delay(250);

   HD44780_SetCursor(8,2);
   HD44780_PrintStr("By:");
   HAL_Delay(250);

   HD44780_SetCursor(5,3);
   HD44780_PrintStr("LEON REEH");
   HAL_Delay(800);
   HD44780_Clear();

   Init_lcd_ar(&lcd_val);
   resetDO();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Read inputs
	  readADCs();
	  readDI();
	  doADCs();
	  /* STATE MACHINE */
	  switch(STATE){
	  	  case READY:
	  		  ready();
	  		  setDO();
	  		  break;

	  	  case DRIVE:
	  		  drive();
	  		  setDO();
	  		  break;

	  	  case BREAK:
	  		  breaking();
	  		  setDO();
	  		  break;

	  	  case SWFAULT:
	  		  resetDO();
	  		  swfault();
	  		  break;

	  	  case HWFAULT:
	  		  resetDO();
	  		  hwfault();
	  		  break;

	  	  case DEBUGST:
	  		  debug();
	  		  setDO();
	  		  break;

	  	  default:
	  		  resetDO();
	  		  hwfault();
	  		  break;
	  	  }

	  //Update LCD every 500ms
	  if(timcc>=5){
		  update_lcd_val(&lcd_val,ADC_VAL);
		  writeState();
		  timcc =0;
	  }

	  HAL_Delay(25);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOB, PB1_LED_RED_Pin|PB0_LED_GREEN_Pin|PB13_U_Pin|PB14_V_Pin
                          |PB15_W_Pin|PB3_DO_LIGHT_Pin|PB8_DO_FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PB5_DO_DC_ON_GPIO_Port, PB5_DO_DC_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13_DI_AUX_Pin PC10_DI_LIGHT_Pin PC11_DI_BLINKER_L_Pin PC12_DI_BLINKER_R_Pin */
  GPIO_InitStruct.Pin = PC13_DI_AUX_Pin|PC10_DI_LIGHT_Pin|PC11_DI_BLINKER_L_Pin|PC12_DI_BLINKER_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI5_Break_Pin EXTI6_HALL_U_Pin EXTI7_HALL_V_Pin EXTI8_HALL_W_Pin */
  GPIO_InitStruct.Pin = EXTI5_Break_Pin|EXTI6_HALL_U_Pin|EXTI7_HALL_V_Pin|EXTI8_HALL_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1_LED_RED_Pin PB0_LED_GREEN_Pin PB3_DO_LIGHT_Pin PB5_DO_DC_ON_Pin
                           PB8_DO_FAN_Pin */
  GPIO_InitStruct.Pin = PB1_LED_RED_Pin|PB0_LED_GREEN_Pin|PB3_DO_LIGHT_Pin|PB5_DO_DC_ON_Pin
                          |PB8_DO_FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13_U_Pin PB14_V_Pin PB15_W_Pin */
  GPIO_InitStruct.Pin = PB13_U_Pin|PB14_V_Pin|PB15_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI9_FAULT_Pin */
  GPIO_InitStruct.Pin = EXTI9_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI9_FAULT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Interrupt Pin Function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Handle Break Signal Interrupt
    if (GPIO_Pin == EXTI5_Break_Pin) {
    	handleBreakInterrupt();
    }
    // Handle Hardware Fault Interrupt
    else if (GPIO_Pin == EXTI9_FAULT_Pin) {
    	handleHardwareFaultInterrupt();
    }
    // Handle Hall Sensor Interrupt
    else if (STATE == DRIVE || STATE == READY) {
        if (GPIO_Pin == EXTI6_HALL_U_Pin || GPIO_Pin == EXTI7_HALL_V_Pin || GPIO_Pin == EXTI8_HALL_W_Pin) {
        	handleHallSensorInterrupt(GPIO_Pin);
        }
    } else {
        // No operation for other states
        __NOP();
    }
}
/* Interrupt Handler Functions*/

/**
  * @brief Handles the brake interrupt signal, transitioning motor state between BREAK and READY
  * @param None
  * @retval void
  */
void handleBreakInterrupt() {
    uint16_t breakSignal = (GPIOC->IDR & GPIO_IDR_ID5) ? 0x0001 : 0x0000;
    if (breakSignal == 0) {
        // Transition to BREAK state
        STATE = BREAK;
        HAL_GPIO_WritePin(PB0_LED_GREEN_GPIO_Port, PB0_LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PB1_LED_RED_GPIO_Port, PB1_LED_RED_Pin, GPIO_PIN_SET);
    } else {
        // Transition to READY state if motor is fully stopped
    	if(rpm >=5){
    		STATE = DRIVE;
    	}else{
    		STATE = READY;
    	}
        HAL_GPIO_WritePin(PB0_LED_GREEN_GPIO_Port, PB0_LED_GREEN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PB1_LED_RED_GPIO_Port, PB1_LED_RED_Pin, GPIO_PIN_RESET);
    }
}

/**
  * @brief Handles hardware fault interrupts, transitioning motor state to HWFAULT
  * @param None
  * @retval void
  */
void handleHardwareFaultInterrupt() {
	STATE = HWFAULT;
    // Optional: Implement hardware fault handling
    // Example readADCs() and determine fault cause;
}

/**
  * @brief Handles hall sensor interrupts, updates commutator step for motor control
  * @param GPIO_Pin      = Pin number that triggered the interrupt
  * @retval void
  */
void handleHallSensorInterrupt(uint16_t GPIO_Pin) {
    hallCC++; // Increment hall sensor counter

    // Read hall sensor states
    uint16_t hall[3];
    hall[0] = (GPIOC->IDR & GPIO_IDR_ID6) ? 0x0001 : 0x0000; // Sensor U
    hall[1] = (GPIOC->IDR & GPIO_IDR_ID7) ? 0x0001 : 0x0000; // Sensor V
    hall[2] = (GPIOC->IDR & GPIO_IDR_ID8) ? 0x0001 : 0x0000; // Sensor W

    // Trapazoidal control
    uint16_t commutatorStep = hallState(hall);
    commutator(commutatorStep, duty, dir);

    // Optional: Implement sine control as needed
    /*
     * uint16_t commutatorStep = hallState(hall);
     * uint16_t phaseAngle = electricalAngle(commutatorStep);
     * FOCcommutator(phaseAngle, duty);
     */
}

/* Input Functions*/
/**
  * @brief Reads ADC values for voltage, current, and temperature, and updates corresponding variables
  * @param None
  * @retval void
  */
void readADCs(){
	uint16_t a =0;  //Int dump for ADC
	float x = 0.0;
	//READ Voltage
	ADC3_Select_CH(0);
	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, ADC_TIMEOUT);
	a =HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);
	x = adc_volt(a);
	ADC_VAL[0] = 0.15 * x + (1.0- 0.15)*ADC_VAL[0];

	//READ Current
	ADC3_Select_CH(1);
	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, ADC_TIMEOUT);
	a =HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);
	x = adc_volt(a);
	ADC_VAL[1] = 0.15 * x + (1.0- 0.15)*ADC_VAL[1];

	//READ Temperature
	ADC3_Select_CH(2);
	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, ADC_TIMEOUT);
	a =HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);
	x = adc_volt(a);
	ADC_VAL[2] = 0.15 * x + (1.0- 0.15)*ADC_VAL[2];

}
/**
  * @brief Processes ADC values to monitor and control system states, including fault detection and fan control
  * @param None
  * @retval void
  */
void doADCs() {
    // Ensure ADC_VAL array has enough elements
    if (sizeof(ADC_VAL) / sizeof(ADC_VAL[0]) < 3) {
        // Handle error (e.g., log it, set state to fault)
        STATE = SWFAULT;
        HD44780_SetCursor(0, 1);
        HD44780_PrintStr("ERR:ADC VAL SIZE");
        return;
    }

    // Release SW_FAULT if all conditions are normal
    if (STATE == SWFAULT) {
        if (ADC_VAL[0] < SW_OV && ADC_VAL[0] > SW_UV + 2 &&  // Voltage OK
            ADC_VAL[1] <= 1 &&                              // Current OK
            ADC_VAL[2] <= Temp_FAN_ON) {                    // Temperature OK
            STATE = BREAK;
        }
    }
    // Device is in normal condition
    if (STATE !=SWFAULT){
        // Control the fan based on temperature
        if (ADC_VAL[2] >= Temp_FAN_ON) {
            HAL_GPIO_WritePin(GPIOB, PB8_DO_FAN_Pin, GPIO_PIN_RESET); // FAN ON
        } else if (ADC_VAL[2] <= Temp_FAN_OFF) {
            HAL_GPIO_WritePin(GPIOB, PB8_DO_FAN_Pin, GPIO_PIN_SET);   // FAN OFF
        }

        // Check for software faults and set error messages
        if (ADC_VAL[0] >= SW_OV) {
            setFaultState("ERR:SW OV");
        } else if (ADC_VAL[0] <= SW_UV) {
            setFaultState("ERR:SW UV");
        } else if (ADC_VAL[1] >= SW_OC) {
            setFaultState("ERR:SW OC");
        } else if (ADC_VAL[2] >= SW_OT) {
            setFaultState("ERR:SW OT");
        }
    }
}

/**
  * @brief Sets the system state to SWFAULT and displays the provided error message
  * @param errorMessage    = Pointer to a string containing the fault error message
  * @retval void
  */
void setFaultState(const char* errorMessage) {
    STATE = SWFAULT;
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(errorMessage);
}

/**
  * @brief Write the Hardware state to the corresponding position onto the display
  * @retval void
  */
void writeState() {
    const char *state; // String to hold the text corresponding to the state

    switch (STATE) {
        case READY:
            state = "READY";
            break;
        case DRIVE:
            state = "DRIVE";
            break;
        case BREAK:
            state = "BREAK";
            break;
        case SWFAULT:
            state = "SWFAULT";
            break;
        case HWFAULT:
            state = "HWFAULT";
            break;
        case DEBUGST:
        	state = "DEBUG";
            break;
        default:
            state = "UNKNOWN"; // Fallback for undefined states
            break;
    }

    // Set the cursor to position (0, 2)
    HD44780_SetCursor(0, 2);

    // Write the corresponding state string to the display
    HD44780_PrintStr(state);
}


/**
  * @brief Reads digital input values from GPIO pins and updates button states
  * @param None
  * @retval void
  */
void readDI(){
    // Button data[Light, Blinker L, Blinker R, Aux]
    uint16_t but_new[4];
    but_new[0] = (GPIOC->IDR & GPIO_IDR_ID10) ? 0x0001 : 0x0000;
    but_new[1] = (GPIOC->IDR & GPIO_IDR_ID11) ? 0x0001 : 0x0000;
    but_new[2] = (GPIOC->IDR & GPIO_IDR_ID12) ? 0x0001 : 0x0000;
    but_new[3] = (GPIOC->IDR & GPIO_IDR_ID13) ? 0x0001 : 0x0000;

    for (int i = 0; i < 4; i++) {
    	but[i] = but_new[i];
        }
}
/* Output Functions*/
/**
  * @brief Sets digital output states based on button inputs and updates system behavior accordingly
  * @param None
  * @retval void
  */
void setDO() {
	//Data error
	if (sizeof(but) / sizeof(but[0]) < 4) {
		setFaultState("ERR: DO SIZE");
	}

    // Button data: [Light, Blinker L, Blinker R, Aux]
    // Toggle lights based on button state
    HAL_GPIO_WritePin(GPIOB, PB3_DO_LIGHT_Pin, but[0] == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Handle Blinker Left (PWM Control)
    if (but[1] == 1) {
        TIM3->CCR2 = BLINKER_START;  // Start PWM
    } else {
        TIM3->CCR2 = BLINKER_STOP;  // Stop PWM
    }

    // Handle Blinker Right (PWM Control)
    if (but[2] == 1) {
        TIM3->CCR1 = BLINKER_START;  // Start PWM
    } else {
        TIM3->CCR1 = BLINKER_STOP;  // Stop PWM
    }

    // Handle Aux button action
    if (but[3] == 0) {
        HAL_GPIO_TogglePin(PB1_LED_RED_GPIO_Port, PB1_LED_RED_Pin); // Toggle LED
        STATE = DRIVE;                                             // Set state to DRIVE
    }
}

/**
  * @brief Resets digital output states into off state
  * @param None
  * @retval void
  */
void resetDO(){
	//main light off
	HAL_GPIO_WritePin(GPIOB, PB3_DO_LIGHT_Pin,GPIO_PIN_RESET);
	TIM3->CCR2 = BLINKER_STOP;  // Stop Blinker Left PWM
	TIM3->CCR1 = BLINKER_STOP;  // Stop Blinker Right PWM
}

/* STATE Machine Functions */
/**
  * @brief Prepares the system for motor operation by initializing throttle input and PWM control
  * @param None
  * @retval void
  */
void ready() {
    // Start ADC conversion
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 20);

    // Get throttle raw value
    rawThrot = HAL_ADC_GetValue(&hadc1);

    // Map throttle value to duty cycle range
    int THrotduty = map(rawThrot, MINADC, MAXADC, MINDUTY, MAXDUTY);

    // Check if throttle duty cycle exceeds the threshold
    if (THrotduty >= THROTTLE_THRESHOLD) {
        // Initialize PWM outputs to zero
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;

        // Start PWM for all three channels
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

        // Initialize BLDC motor
        initBLDC();

        // Set initial duty cycle
        duty = 10; // Set initial duty cycle; modify as needed

        // Change state to DRIVE
        STATE = DRIVE;
    }
}

/**
  * @brief Controls the motor drive by reading throttle input and updating the PWM duty cycle
  * @param None
  * @retval void
  */
void drive() {
    // Start ADC conversion
    HAL_ADC_Start(&hadc1);

    // Wait for ADC conversion to complete with a timeout
    if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT) == HAL_OK) {
        // Get raw throttle value
        rawThrot = HAL_ADC_GetValue(&hadc1);

        // Map the raw throttle value to a duty cycle range
        int THrotduty = map(rawThrot, MINADC, MAXADC, MINDUTY, MAXDUTY);

        // Update the duty cycle
        duty = THrotduty;
    } else {
        // Handle ADC conversion error (optional)
        duty = 0; // Set duty to a safe value in case of failure
        setFaultState("ERR: ADC");
        // Optionally log or display an error message
    }
}

/**
  * @brief Engages the braking mechanism by stopping PWM outputs and activating braking GPIO pins
  * @param None
  * @retval void
  */
void breaking() {
    // Stop all PWM channels
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    duty = 0; // Reset duty cycle to zero

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    // Set GPIO pins for braking mode
    HAL_GPIO_WritePin(GPIOB, PB13_U_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, PB14_V_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, PB15_W_Pin, GPIO_PIN_SET);
}

/**
  * @brief Handles the software fault state by engaging braking and transitioning to HWFAULT after a timeout
  * @param None
  * @retval void
  */
void swfault() {
    // Perform breaking to ensure the system is in a safe state
    breaking();

    // Check if the timeout for SWFAULT has elapsed (30s = 300 units of 100ms)
    if (swfault_time_counter >= SWFAULT_TIMOUT) {
        // Transition to HWFAULT state
        STATE = HWFAULT;
    }
}

/**
  * @brief Handles hardware faults by engaging braking, displaying an error message, and halting execution
  * @param None
  * @retval void
  */
void hwfault(){
	STATE = HWFAULT;
	breaking();
	HD44780_SetCursor(0,1);
	HD44780_PrintStr("ERROR:HW FAULT");
	//Stuck untill Power on reset
	while(1){
		HAL_Delay(100);

	}
}

void debug(){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	duty = 0;
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_RESET);
}

/**
  * @brief Configures the ADC3 channel for the selected input
  * @param ch    = Channel number to configure (0, 1, or 2)
  * @retval void
  */
void ADC3_Select_CH(int ch){
	ADC_ChannelConfTypeDef sConfig = {0};
	if(ch ==0){
		  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		  */
		  sConfig.Channel = ADC_CHANNEL_0;
		  sConfig.Rank = 1;
		  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }
	}

	if(ch==1){
		  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		  */
		  sConfig.Channel = ADC_CHANNEL_1;
		  sConfig.Rank = 1;
		  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }
	}
	if(ch==2){
		  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		  */
		  sConfig.Channel = ADC_CHANNEL_13;
		  sConfig.Rank = 1;
		  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }
	}
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
