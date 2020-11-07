/*
 * mylibrary.c
 *
 *  Created on: Oct 1, 2020
 *      Author: sumat
 */
#include "mylibrary.h"

uint8_t txbuffer;

void Message_handler()
{
	  if(txbuffer == 1) {
		  txbuffer = 2;
		  HAL_UART_Transmit(&huart1, (uint8_t *)&txbuffer, 1, 100);
	  }
	  else {
		  txbuffer = 1;
		  HAL_UART_Transmit(&huart1, (uint8_t *)&txbuffer, 1, 100);
	  }
}


