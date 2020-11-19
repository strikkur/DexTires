/* This file contains all the functions needed for the higher-level UART Transmission algorithms.
 *
 * Functions in here will be called by the USART_IRQ_Handler in stm32f0xx_it.c
 *
 * Author: Samyukta Trikkur
 * Date: 11/17/2020
 *
 * */

/* INCLUDES */
#include "transmission.h"
#include "main.h"

/* VARIABLES */
uint8_t txbuffer;

/* FUNCTIONS */
void transmission_handler(UART_HandleTypeDef huart, uint8_t dir, uint8_t speed, int mode) {

	dir = dir << 5;
	mode = mode << 7;
	txbuffer = (mode | dir | speed);
	//	txbuffer = (uint8_t) (RawFSRInput[1] >> 4);
	HAL_UART_Transmit(&huart, (uint8_t *)&txbuffer, 1, 100);
}

