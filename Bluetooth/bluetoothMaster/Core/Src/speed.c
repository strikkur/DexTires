/* This file contains all the functions needed to encode the 5-bit speed value which
 * will get transmitted to the car.
 *
 * Author: Sumati Gupta
 * Date: 11/17/2020
 *
 * */

/* INCLUDES */
#include "speed.h"
#include "main.h"

/* VARIABLES */
// 8-bit variable to hold the 5-bit speed value for intermediate processing
uint8_t speed; //set only bits 3 - 7


/* FUNCTIONS */

/* int conditional(uint32_t Tr,uint32_t Tmax, uint32_t ADC_avg, int* pressurearray)
 * Checks which range the user input ADC value is in and appropriately sets speed.
 * If ADCavg is less than resting threshold of corresponding hand motion, then speed is 0.
 * If ADCavg is in between resting threshold and average threshold of corresponding hand motion, then speed is MIN_SPEED.
 * If ADCAvg is more than maximum threshold, then speed is MAX_SPEED.
 * Else, need to compute speed as it lies in our "bucket range" for acceleration computation. */
int conditional(uint32_t Tr, uint32_t Tmax, uint32_t ADC_avg, int* pressurearray){

	// calculation Tavg
	int speed = 0;
	uint32_t Tavg = (Tr + Tmax) / 2;
	// k = kcalc(Tavg, Tmax);

	if (ADC_avg < Tr){
		speed = 0;
	}
	else if (ADC_avg >= Tmax){
		speed = MAX_SPEED;
	}
	else if ((ADC_avg > Tr) && (ADC_avg < Tavg)){
		speed = MIN_SPEED;
	}
	else if ((ADC_avg > Tavg) && (ADC_avg < Tmax)){
			speed = speedcalc(Tavg, Tmax, pressurearray);
	}
	return speed;

}

/* int speedcalc(uint32_t Tavg, uint32_t Tmax, int* pressurearray)
 * Speed is deemed as lying in "bucket range" for acceleration computation.
 * Refers to corresponding pressure array for correct hand motion and assigns the
 * correct 5-bits. This is returned. If function returns -1, something is WRONG!! */
int speedcalc(uint32_t Tavg, uint32_t Tmax, int * pressurearray){

	// The (j + 2) is offsetting for the lower two already assigned values; (0 speed and MIN_SPEED of 1).

	for (int j = 0; j < 29; j++){
		// if current pressure exceeds last bucket but not max threshold, then return 30.
		if (j == 28){
			if ((RawFSRAvg >= pressurearray[j]) && (RawFSRAvg < Tmax)) {
						return j + 2;
			}
		}
		// if current pressure lies between two buckets, then return corresponding current speed.
		if ((RawFSRAvg >= pressurearray[j]) && (RawFSRAvg < pressurearray[j + 1])) {
			return j + 2;
		}
	}

	return -1;

}
