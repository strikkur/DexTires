/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "calibration.h"
#include "transmission.h"
#include "direction.h"
#include "speed.h"

// include speedConstants for testing
#include "speedConstantsSettings.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//counter to indicate 3 secs for calibration mode

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc;
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  // timer interrupt should be happening irrespective of this external interrupt; the ADC value should be updating still by the TIM3

  // calibration mode button was pressed, so change the global mode, set_state , and cal_complete values
  mode = 0;
  modeflag = 0;
  set_state = BEGIN;
  cal_complete = 0;
  last_state_completed = 0;

  //indication of first state
  //transmission_handler(huart1, 3, 1, mode);
  //HAL_Delay(3000);

  //Start timer 15
  //HAL_TIM_Base_Stop_IT(&htim15);
  //HAL_TIM_Base_Start_IT(&htim15);
  //statemachine(huart1, set_state, htim15);


  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles ADC and COMP interrupts (COMP interrupts through EXTI lines 21 and 22).
  */
void ADC1_COMP_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_COMP_IRQn 0 */

  /* USER CODE END ADC1_COMP_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_COMP_IRQn 1 */

  /* USER CODE END ADC1_COMP_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /* Start ADC-DMA conversions to place the raw FSR ADC values into the RawFSRInput buffer */
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)RawFSRInput, 5);
  RawFSRAvg = (RawFSRInput[0] + RawFSRInput[1] + RawFSRInput[2] + RawFSRInput[3] + RawFSRInput[4]) / 5;

/* Calibration code will be added back in later; need to add external interrupt for button */
  if (mode == 0){
	  // perform calibration polling every 10ms when in calibration mode and skip other speed/direction computations
	  // calibration computations taken care of in calibration.c
	  calADCavg = calADCavg + RawFSRAvg;
	  tempcounter = tempcounter + 1;

//	  if (last_state_completed) {
//		  fourTavg();
//		  fourk();
//		  pressureArrayInit();
//	  }
  }
  else if (mode && cal_complete) {

	  direction = controlScheme(RawFSRInput);
	  if (direction == 3){
		  // if direction is forward, compute speed based on global forward motion measurements
		  speed = conditional(frontTr, frontTmax, RawFSRAvg, pressurefront);
	  }
	  if (direction == 0){
		  // if direction is reverse, compute speed based on global reverse motion measurements
		  speed = conditional(reverseTr, reverseTmax, RawFSRAvg, pressurereverse);
	  }
	  if (direction == 2){
		  // if direction is left, compute speed based on global left motion measurements
		  speed = conditional(leftTr, leftTmax, RawFSRAvg, pressureleft);
	  }
	  if (direction == 1){
		  // if direction is right, compute speed based on global right motion measurements
		  speed = conditional(rightTr, rightTmax, RawFSRAvg, pressureright);
	  }
  }
  else {
	// fail-safe/error-handling
	// gameplay mode but calibration hasn't happened yet; i.e. when user turns on device for first time
	transmission_handler(huart1, 3, 0, mode);

  }

  // Enable IT does nothing when in calibration mode
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM15 global interrupt.
  */
void TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM15_IRQn 0 */
  // this timer should only do stuff if calibration mode is entered
  if (mode == 0) {
	  //(CalibrationState)set_state++;
	  statemachine(huart1, set_state, htim15);
	  // set next state variable
	  (CalibrationState)set_state++;
	  if(last_state_completed) {
		  HAL_TIM_Base_Stop(&htim15);
		  cal_complete = 1;
		  mode = 1;
		  modeflag = 0;
		  fourTavg();
		  fourk();
		  pressureArrayInit();
	  }
  }
  /* USER CODE END TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM15_IRQn 1 */

  /* USER CODE END TIM15_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

	// only call interrupt-driven transmission handler if in gameplay mode
	if(mode && cal_complete) {
		transmission_handler(huart1, direction, speed, mode);
	}
//	if(mode) {
//		transmission_handler(huart1, direction, speed, mode);
//	}0

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
