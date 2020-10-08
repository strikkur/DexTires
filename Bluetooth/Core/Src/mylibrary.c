/*
 * mylibrary.c
 *
 *  Created on: Oct 1, 2020
 *      Author: sumat
 */
#include "mylibrary.h"

char buffer[50];
//uint8_t buffer[50];
uint8_t timer_count = 0;
uint8_t buffer_index = 0;

uint8_t string_compare(char *array1, char *array2 , uint16_t length)
{
	 uint8_t comVAR=0, i;
	 for(i=0;i<length;i++)
	   	{
	   		  if(array1[i]==array2[i])
	   	  		  comVAR++;
	   	  	  else comVAR=0;
	   	}
	 if (comVAR==length)
		 	return 1;
	 else 	return 0;
}
void Message_handler()
{
	//char * red =  // turn to char array and send account last \0. look into more.
	//char * green =
	if(string_compare(buffer, "RED LED", strlen("RED LED"))) //\0 might be needed
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_UART_Transmit(&huart1, (uint8_t*)"RED LED IS ON\n", strlen("RED LED IS ON\n") + 1, 500);
	}else
	if (string_compare(buffer, "GREEN LED", strlen("GREEN LED")))
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)"GREEN LED IS ON\n", strlen("GREEN LED IS ON\n") + 1, 500);
		}
	else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_UART_Transmit(&huart1, (uint8_t*)"both LED's are off\n", strlen("both LED's are off\n") + 1, 500);
		}
	memset(buffer, 0, sizeof(buffer));
	buffer_index = 0;
	timer_count = 0;
}


