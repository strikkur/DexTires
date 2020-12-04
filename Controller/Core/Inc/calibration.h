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
	BEGIN,
	FORWARD_REST,
	WAIT1,
	FORWARD_MAX,
	WAIT2,
	REVERSE_REST,
	WAIT3,
	REVERSE_MAX,
	WAIT4,
	LEFT_REST,
	WAIT5,
	LEFT_MAX,
	WAIT6,
	RIGHT_REST,
	WAIT7,
	RIGHT_MAX,
	END,
} CalibrationState;

/* FUNCTIONS */
void statemachine(UART_HandleTypeDef, CalibrationState, TIM_HandleTypeDef htim15);



#endif /* INC_CALIBRATION_H_ */
