/* This file contains all the functions needed for each hand motion to be translated
 * into a direction.
 *
 * Functions in here will be called by the USART_IRQ_Handler in stm32f0xx_it.c
 *
 * Author: Sumati Gupta
 * Date: 11/17/2020
 *
 * */

/* INCLUDES */
#include "direction.h"

/* FUNCTIONS */

/*
 * uint8_t controlScheme(uint32_t *ADCValueArr)
 *  Takes in the RawFSRInput array and determines what the user input
 *  hand motion is and returns corresponding direction value. See function
 *  for control scheme values.
 *  */
uint8_t controlScheme(uint32_t * ADCValueArr){

	uint32_t indexfinger = ADCValueArr[0];
	uint32_t middle = ADCValueArr[1];
	uint32_t ring = ADCValueArr[2];
	uint32_t pinky = ADCValueArr[3];
	uint32_t thumb = ADCValueArr[4];

	if ((indexfinger > ADCintialvalue) && (middle > ADCintialvalue) && (ring > ADCintialvalue) && (pinky > ADCintialvalue) && (thumb > ADCintialvalue)){
		// hand motion is forward
		return 3;
	}
	else if((indexfinger == ADCintialvalue) && (middle > ADCintialvalue) && (ring > ADCintialvalue) && (pinky > ADCintialvalue) && (thumb > ADCintialvalue)){
		// hand motion is right
		return 1;
	}
	else if((indexfinger > ADCintialvalue) && (middle > ADCintialvalue) && (ring > ADCintialvalue) && (pinky == ADCintialvalue) && (thumb > ADCintialvalue)){
		// hand motion is left
		return 2;
	}
	else if((indexfinger == ADCintialvalue) && (middle > ADCintialvalue) && (ring > ADCintialvalue) && (pinky == ADCintialvalue) && (thumb > ADCintialvalue)){
		// hand motion is reverse
		return 0;
	}
	else{
		// hand motion is invalid; so send forward by default
		return 3;
	}

}
