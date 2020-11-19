/* This file contains all the functions needed for each hand motion to be translated
 * into a direction.
 *
 * Functions in here will be called by the USART_IRQ_Handler in stm32f0xx_it.c
 *
 * Author: Sumati Gupta
 * Date: 11/17/2020
 *
 * */

#ifndef INC_DIRECTION_H_
#define INC_DIRECTION_H_

/* INCLUDES */
#include "main.h"

/* DEFINES */
// This is the initial ADC value of the FSRs without any pressure. Setting to 0 for now, will have to mess around with this while testing.
#define ADCintialvalue 0

/* FUNCTIONS */
uint8_t controlScheme(uint32_t *);

#endif /* INC_DIRECTION_H_ */
