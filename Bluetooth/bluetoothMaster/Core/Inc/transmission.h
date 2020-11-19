#ifndef INC_TRANSMISSION_H_
#define INC_TRANSMISSION_H_
/* This file contains all the main variables and declarations needed for the UART Transmission algorithms.
 *
 *
 * Author: Samyukta Trikkur
 * Date: 11/17/2020
 *
 * */

/* INCLUDES */
#include "main.h"

/* VARIABLES */
extern uint8_t txbuffer;

/* FUNCTIONS */
void transmission_handler(UART_HandleTypeDef, uint8_t, uint8_t, int);

#endif /* INC_TRANSMISSION_H_ */
