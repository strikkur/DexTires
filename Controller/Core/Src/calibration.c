/* This file contains all the functions needed to perform the calibration computations
 * and state machine.
 *
 * Functions in here will be called by the USART_IRQ_Handler in stm32f0xx_it.c
 *
 * Author: Sumati Gupt6a
 * Date: 11/18/2020
 *
 * */

/* INCLUDES */
#include "calibration.h"

/* VARIABLES */

/* FUNCTIONS */
/* void statemachine(UART_HandleTypeDef huart, uint8_t set_state)
 * implements the calibration state machine that indicates to car which threshold value we are
 * currently calculating, takes the average value of user input adc value for 3 secs and updates the
 * appropriate global threshold variable. speed = 1 for resting pressure, speed = 2 for max pressure.
 * transmission handler indicating next state is called in previous state
 * */
void statemachine(UART_HandleTypeDef huart, CalibrationState set_state, TIM_HandleTypeDef htim15) {

	switch(set_state) {
	    case BEGIN:
	    	transmission_handler(huart, 3, 1, mode);
	    	break;
		case FORWARD_REST:
			//transmission_handler(huart, 3, 1, mode);
			frontTr = calADCavg / tempcounter; //sum of ADC_avg values every 10ms for 3 secs / 300;
			calADCavg = 0;
			tempcounter = 0;
			last_state_completed = 0;
			transmission_handler(huart, 3, 0, mode);
			break;
		case WAIT1:
			transmission_handler(huart, 3, 2, mode);
			last_state_completed = 0;
			calADCavg = 0;
			tempcounter = 0;
			break;
		case FORWARD_MAX:
			//transmission_handler(huart, 3, 2, mode);
			frontTmax = calADCavg / tempcounter;
			calADCavg = 0;
			tempcounter = 0;
			last_state_completed = 0;
			transmission_handler(huart, 3, 0, mode);
			break;
		case WAIT2:
			transmission_handler(huart, 0, 1, mode);
			last_state_completed = 0;
			calADCavg = 0;
			tempcounter = 0;
			break;
		case REVERSE_REST:
			//transmission_handler(huart, 0, 1, mode);
			reverseTr = calADCavg / tempcounter;
			calADCavg = 0;
			tempcounter = 0;
			last_state_completed = 0;
			transmission_handler(huart, 3, 0, mode);
			break;
		case WAIT3:
			transmission_handler(huart, 0, 2, mode);
			last_state_completed = 0;
			calADCavg = 0;
			tempcounter = 0;
			break;
		case REVERSE_MAX:
			//transmission_handler(huart, 0, 2, mode);
			reverseTmax = calADCavg / tempcounter;
			calADCavg = 0;
			tempcounter = 0;
			last_state_completed = 0;
			transmission_handler(huart, 3, 0, mode);
			break;
		case WAIT4:
			transmission_handler(huart, 2, 1, mode);
			last_state_completed = 0;
			calADCavg = 0;
			tempcounter = 0;
			break;
		case LEFT_REST:
			//transmission_handler(huart, 2, 1, mode);
			leftTr = calADCavg / tempcounter;
			calADCavg = 0;
			tempcounter = 0;
			last_state_completed = 0;
			transmission_handler(huart, 3, 0, mode);
			break;
		case WAIT5:
			transmission_handler(huart, 2, 2, mode);
			last_state_completed = 0;
			calADCavg = 0;
			tempcounter = 0;
			break;
		case LEFT_MAX:
			//transmission_handler(huart, 2, 2, mode);
			leftTmax = calADCavg / tempcounter;
			calADCavg = 0;
			tempcounter = 0;
			last_state_completed = 0;
			transmission_handler(huart, 3, 0, mode);
			break;
		case WAIT6:
			transmission_handler(huart, 1, 1, mode);
			last_state_completed = 0;
			calADCavg = 0;
			tempcounter = 0;
			break;
		case RIGHT_REST:
			//transmission_handler(huart, 1, 1, mode);
			rightTr = calADCavg / tempcounter;
			calADCavg = 0;
			tempcounter = 0;
			last_state_completed = 0;
			transmission_handler(huart, 3, 0, mode);
			break;
		case WAIT7:
			transmission_handler(huart, 1, 2, mode);
			last_state_completed = 0;
			tempcounter = 0;
			calADCavg = 0;
			break;
		case RIGHT_MAX:
			transmission_handler(huart, 3, 0, mode);
			rightTmax = calADCavg / tempcounter;
			calADCavg = 0;
			tempcounter = 0;
			last_state_completed = 0;
			calADCavg = 0;
			break;
		case END:
			last_state_completed = 1;
			calADCavg = 0;
			break;
		default:
			// we should never reach here...statemachine should always work properly!!
			// the following transmission should never happen; we need to have a fail-safe/error handling for that
			transmission_handler(huart, 0, 0, 0);
	}
	return;
}
