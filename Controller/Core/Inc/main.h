/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "calibration.h"
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
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
extern uint32_t RawFSRInput[5];
extern uint32_t RawFSRAvg;

/* Speed constants */
extern uint32_t frontTr, frontTmax, frontTavg, reverseTr, reverseTmax, reverseTavg, leftTr, leftTmax, leftTavg, rightTr, rightTmax, rightTavg;
extern int leftk, rightk, frontk, reversek;

/* Pressure arrays to split the acceleration ranges into 29-buckets appropriately for each hand motion. */
extern int pressurefront[29], pressurereverse[29], pressureleft[29], pressureright[29];

/* Default mode for gameplay --> mode = 1 is gameplay; mode = 0 is calibration. */
extern int mode;

/* Buffer holding user's current direction corresponding to hand motion. */
extern uint8_t direction;

/* Buffer holding 5-bit speed translated from user's current pressure input. */
extern uint8_t speed;

/* Calibration Variables */
extern uint32_t calADCavg;
extern enum CalibrationState set_state;
extern uint8_t last_state_completed;
extern uint8_t cal_complete;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
