/* This file contains all the functions needed for calibration state meachine
 *
 * Functions in here will be called by the USART_IRQ_Handler in stm32f0xx_it.c
 *
 * Author: Sumati Gupta
 * Date: 11/17/2020
 *
 * */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

/* INCLUDES */
#include "main.h"
#include "transmission.h"

/* VARIABLES */
typedef enum CalibrationState {
	FORWARD_REST,
	FORWARD_MAX,
	REVERSE_REST,
	REVERSE_MAX,
	LEFT_REST,
	LEFT_MAX,
	RIGHT_REST,
	RIGHT_MAX,
} CalibrationState;

/* FUNCTIONS */
void statemachine(UART_HandleTypeDef, CalibrationState);



#endif /* INC_CALIBRATION_H_ */
