/* This file contains all the declarations for the functions needed to
 * encode the 5-bit speed value which will get transmitted to the car.
 *
 * Author: Sumati Gupta
 * Date: 11/17/2020
 *
 * */

#ifndef INC_SPEED_H_
#define INC_SPEED_H_

#include <stdint.h>

/* DEFINES */
// max speed possible in 5-bit value; our max speed "bucket"
#define MAX_SPEED 31
// min speed possible in 5-bit value; our min speed "bucket"
#define MIN_SPEED 1

/* VARIABLES */
extern uint8_t speed;

/* FUNCTIONS */
int conditional(uint32_t, uint32_t, uint32_t, int *);
int speedcalc(uint32_t, uint32_t, int *);

#endif /* INC_SPEED_H_ */
