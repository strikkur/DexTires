/*
 * mylibrary.c
 *
 *  Created on: Oct 1, 2020
 *      Author: sumat
 */
#include "mylibrary.h"

uint8_t buffer;
uint8_t timer_count = 0;
uint8_t buffer_index = 0;

void Message_handler()
{

	if(buffer == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//		HAL_UART_Transmit(&huart1, (uint8_t*)"RED LED IS ON\r\n", strlen("RED LED IS ON\r\n"), 500);
	}
	else if (buffer == 2)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
//		HAL_UART_Transmit(&huart1, (uint8_t*)"GREEN LED IS ON\r\n", strlen("GREEN LED IS ON\r\n"), 500);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
//		HAL_UART_Transmit(&huart1, (uint8_t*)"both LED's are off\r\n", strlen("both LED's are off\r\n"), 500);
	}
	buffer = 0;
	buffer_index = 0;
	timer_count = 0;
}


