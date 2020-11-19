#ifndef INC_SPEEDCONSTANTSSETTINGS_H_
#define INC_SPEEDCONSTANTSSETTINGS_H_

/* This file contains all the main variables and declarations needed for setting the speed constants.
 *
 * Author: Sumati Gupta & Samyukta Trikkur
 * Date: 11/17/2020
 *
 * */

/* INCLUDES */
#include <math.h>
#include "main.h"
/* VARIABLES */

/* FUNCTIONS */
uint32_t Tavgcalc(uint32_t, uint32_t);
int kcalc(uint32_t, uint32_t);
void fourTavg();
void fourk();
void pressureArrayInit();


#endif /* INC_SPEEDCONSTANTSSETTINGS_H_ */
