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

/* FUNCTIONS */
void statemachine(UART_HandleTypeDef);

#endif /* INC_CALIBRATION_H_ */
