/* This file contains all the functions needed to perform the calibration computations
 * and state machine.
 *
 * Functions in here will be called by the USART_IRQ_Handler in stm32f0xx_it.c
 *
 * Author: Sumati Gupta
 * Date: 11/18/2020
 *
 * */

/* INCLUDES */
#include "calibration.h"

/* VARIABLES */

/* FUNCTIONS */
/* void statemachine(UART_HandleTypeDef huart)
 * implements the calibration state machine that indicates to car which threshold value we are
 * currently calculating, takes the average value of user input adc value for 3 secs and updates the
 * appropriate global threshold variable. speed = 1 for resting pressure, speed = 2 for max pressure.
 * */
void statemachine(UART_HandleTypeDef huart){
	mode = 0;

	//forward rest
	transmission_handler(huart, mode, 3, 1);
	HAL_Delay(3000);
	frontTr = calADCavg / 300; //sum of ADC_avg values every 10ms for 3 secs / 300;
	calADCavg = 0;

	//forward max
	transmission_handler(huart, mode, 3, 2);
	HAL_Delay(3000);
	frontTmax = calADCavg / 300;
	calADCavg = 0;

	//reverse rest
	transmission_handler(huart, mode, 0, 1);
	HAL_Delay(3000);
	reverseTr = calADCavg / 300;
	calADCavg = 0;

	//reverse max
	transmission_handler(huart, mode, 0, 2);
	HAL_Delay(3000);
	reverseTmax = calADCavg / 300;
	calADCavg = 0;

	//left rest
	transmission_handler(huart, mode, 2, 1);
	HAL_Delay(3000);
	leftTr = calADCavg / 300;
	calADCavg = 0;

	//left max
	transmission_handler(huart, mode, 2, 2);
	HAL_Delay(3000);
	leftTmax = calADCavg / 300;

	//right rest
	transmission_handler(huart, mode, 1, 1);
	HAL_Delay(3000);
	rightTr = calADCavg / 300;
	calADCavg = 0;

	//right max
	transmission_handler(huart,mode, 1, 2);
	HAL_Delay(3000);
	rightTmax = calADCavg / 300;
	calADCavg = 0;

	mode = 1;
	return;
}
