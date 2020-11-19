/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
unsigned ADC_val;
int enableLEDs;
int LEDsRunning;

enum LightIndication {
	Off = 0x0,
	Front = 0x1 << 0,
	Back = 0x1 << 1,
	Left = 0x1 << 2,
	Right = 0x1 << 3
};

uint8_t rxbuffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void SetCalibrationLights(uint8_t, uint8_t);
void DecodeData(uint8_t);
void speed_conversion(uint8_t, uint8_t);
void angle_conversion(uint8_t, uint8_t);
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
  ADC_val = 0;
  LEDsRunning = 0;
  enableLEDs = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* TIM3 controls LEDs during calibration */
  MX_TIM3_Init();

  /* TIM2 controls DC motors */
  MX_TIM2_Init();

  /* TIM15 controls servo */
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //START TIMERS
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Enabling the Receiver Interrupt for UART transmission
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  DecodeData(rxbuffer);
	  //Testing Calibration LEDs
	  /*
	  uint8_t message = 0;
	  uint8_t temp = 0;
	  for (int i = 0; i <= 3; i++) {
	  	  temp = 0x1 << 7; //Calibration Mode
		  temp |= (i << 5); //Direction
		  for (int j = 1; j <= 2; j++) {
			  message = temp | j; //Pressure
			  DecodeData(message);
			  HAL_Delay(3000);
		  }
	  }*/

//	  //Test Motors
//	  uint8_t message = 0;
//	  //uint8_t temp = 0;
//
//	  message = 0;
//	  message = 0 << 5;
//	  message |= 31;
//	  DecodeData(message);
//	  HAL_Delay(4000);
//
//	  message = 0;
//	  message = 1 << 5;
//	  message |= 1;
//	  DecodeData(message);
//	  HAL_Delay(4000);
//
//	  message = 0;
//	  message = 2 << 5;
//	  message |= 1;
//	  DecodeData(message);
//	  HAL_Delay(4000);
//
//	  message = 0;
//	  message = 3 << 5;
//	  message |= 31;
//	  DecodeData(message);
//	  HAL_Delay(4000);
//
//	  message = 0;
//	  message = 1 << 5;
//	  message |= 1;
//	  DecodeData(message);
//	  HAL_Delay(4000);

	  /*
	  for (int i = 0; i <= 3; i++) {
		  temp = 0x0 << 7; //Calibration Mode
		  temp |= (i << 5); //Direction
		  for (int j = 0; j <= 31; j++) {
			  message = temp | j; //Speed
			  DecodeData(message);
			  HAL_Delay(3000);
		  }
	  }*/

	  /*
	  for (int i = 0 ; i < 32; i++) {
		  speed_conversion(i, 0); //Forward
		  angle_conversion(i, 1); //Right
		  HAL_Delay(1000);
	  }

	  for (int i = 0 ; i < 32; i++) {
		  speed_conversion(i, 3); //Backward
		  angle_conversion(i, 2); //Left
		  HAL_Delay(1000);
	  }
	  */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1199;
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
  sConfigOC.Pulse = 599;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim15.Init.Prescaler = 95;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 9999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  HAL_TIM_MspPostInit(&htim15);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, INA_Pin|INB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INA_Pin INB_Pin */
  GPIO_InitStruct.Pin = INA_Pin|INB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* void DecodeData(uint8_t message)
 * Function takes in the received rxbuffer value and decodes it into the appropriate
 * mode, speed, and direction bits. If the remote is in calibration mode, then motors are disabled
 * and LEDs are enabled. Else, motors are enabled and appropriate speed translations into
 * PWM are performed.*/
void DecodeData(uint8_t message) {
	uint8_t mode = message >> 7; //Get bit-7
	uint8_t direction = (message >> 5) & 0x3; // Get bit-6 and bit-5
	uint8_t speed = message & 0x1F; //Get the bottom 5 bits (0 to 4)

	//Check for Calibration Mode (mode is 0 if calibration)
	if (mode == 0) {
		enableLEDs = 1;
		speed_conversion(0, direction); //Don't run the DC Motors
		angle_conversion(0, 0); //Don't turn the Servo Motor
	} else {
		enableLEDs = 0;
		speed_conversion(speed, direction);
		angle_conversion(speed, direction);
	}

	SetCalibrationLights(direction, speed);
}

// This is to take care of the case where connection is disrupted DURING gameplay.
/* TODO: Create a while loop or timer which is constantly checking for connection on the Bluetooth. If it's not
		connected, then the Fail Safe mechanism takes over.
			- Timer that fires every so often (3 seconds?)
			- If no new message (not necessarily different data) has been received, then set 'connected' global
				varibale to FALSE, else set to true
			- Make sure the Decode function handles the 'connected' variable first (top of the function),
				I.e. If not connected, set everything to zero, blink LEDs, return from Decode function
*/

// TODO: While car is not connected, blink all car lights every second to indicate disconnection

/* void SetCalibrationLights(uint8_t direction, uint8_t pressure)
 * Depending on the mode and speed bits, change the intensity of the headlights corresponding to the
 * states in calibration state machine. */
void SetCalibrationLights(uint8_t direction, uint8_t pressure) {
	if (enableLEDs) {
		//If LEDs are already running, return

		int indication = 0;
		int pulseWidth = 0;

		//Enable Timer Channels
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		switch (direction) {
			case 0: indication = Back; break;
			case 1: indication = Right; break;
			case 2: indication = Left; break;
			case 3: indication = Front; break;
			default: indication = Off; break;
		}

		//Note: Prescaler value = 9999
		// here, pressure is referring to the speed bits that are sent during calibration mode.
		if (pressure == 1) {
			//Rest Pressure
			__HAL_TIM_SET_AUTORELOAD(&htim3, 2399); //2Hz
		} else if (pressure == 2) {
			//Full Pressure
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1199); //4Hz
		}

		//Get half of the ARR to set the duty cycle to 50%
		pulseWidth = ((__HAL_TIM_GET_AUTORELOAD(&htim3) + 1) / 2) - 1;

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ((indication & Front) || (indication & Left)) * pulseWidth);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ((indication & Front) || (indication & Right)) * pulseWidth);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ((indication & Back) || (indication & Left)) * pulseWidth);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ((indication & Back) || (indication & Right)) * pulseWidth);

		LEDsRunning = 1;
	} else {
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

		LEDsRunning = 0;
	}
}

/* void angle_conversion(uint8_t input, uint8_t right_left)
 * Function to convert the speed bits into corresponding angle
 * by taking the inverse. Hardcoded values are based on servo datasheet and
 * prior calculations. */
void angle_conversion(uint8_t input, uint8_t right_left) {
	int pulse_width = 0;

	if (right_left == 1) {
		//Right
		pulse_width = (int)(400 + (250 * input / 31));
	} else if(right_left == 2) {
		//Left
		pulse_width = (int)(1100 - (250 * input / 31));
	} else {
		//No turning
		pulse_width = 750;
	}

	htim15.Instance->CCR1 = pulse_width;
}

/* void speed_conversion(uint8_t input, uint8_t front_back)
 * Function to convert the speed bits (uint8_t input) into correct PWM output for
 * DC motors. */
void speed_conversion(uint8_t input, uint8_t front_back)
{
	int duty_cycle_percentage = 0;

	/* The following min_percentage is based on tested values to see when the DC
	 * motors actually run and when they do not. max_percentage is based on when
	 * we thought the DC motors speed was too high, so we limited the max PWM output. */
	// was at 50 and 80
	int min_percentage = 40;
	int max_percentage = 50;

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);

	// This selection construct takes care of the polarity input to the motor driver.
	if(front_back == 3 || front_back == 1 || front_back == 2)//clockwise, forward/right/left
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); //INA
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //INB
	}
	else if(front_back == 0)//anticlockwise, back
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); //INA
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1); //INB
	}

	// if speed is 0, then no duty cycle
	if(input == 0) {
		duty_cycle_percentage = 0;
	}
	// if speed is 1, then min duty cycle
	else if (input == 1) {
		duty_cycle_percentage = min_percentage;
	}
	// if speed is in acceleration range, then compute correct duty cycle percentage
	else {
		duty_cycle_percentage = ((max_percentage - min_percentage)*(input - 1)/30) + min_percentage;
	}

	htim2.Instance->CCR4 = duty_cycle_percentage * (htim2.Instance->ARR+1) / 100;
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
