/* This file contains all the functions needed to encode the 5-bit speed value which
 * will get transmitted to the car.
 *
 * Author: Sumati Gupta & Samyukta Trikkur
 * Date: 11/17/2020
 *
 * */

/* NOTES */
/* Concept of k-value elucidated through parallel "pressure ruler" and "speed ruler":
 * 8-bit value transmitted to Car Slave contains one bit for mode (i.e. calibration or not),
 * two bits for direction, and 5 bits reserved for speed. Thus, we must discretize the range from rest pressure to max pressure
 * (separately for each hand motion) into 32 buckets to be able to translate pressure into 5 bit speed (i.e. 2^5 - 1). Each bucket will
 * be of length k: (k = (max_pressure - Tavg) / 29). Buckets are zero-indexed. We divide by 29 rather than 31, as two buckets are reserved
 * for 0 speed and min_speed. Computation of k is done in kcalc() in speedConstantsSettings.c.
 *
 * This discretization of 32 buckets will form a "ruler" of intervals of pressure which will be stored in global
 * pressurearray for each hand motion (done by pressureArrayInit() in speedConstantsSettings.c). The pressure buckets correspond
 * to the following speed values:
 *
 * "pressure ruler"
 * | _________________________    _________________________  | _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ |
 * 							Tr								Tavg														Tmax
 * 					rest pressure						= (Tmax - Tr) / 2											max pressure
 *
 * corresponding "speed ruler"
 * | _________________________    _________________________  | _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ |
 * 				0								1			   2 3 4 5 6 7 8 9...........................................31
 * 											min_speed																	max_speed
 * 															 |____________________acceleration range_____________________|
 *
 * These speed values will be transmitted to the Car Slave for decoding and outputting appropriate PWM to motors.
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

	if (ADC_avg < Tr){
		speed = 0;
	}
	else if (ADC_avg >= Tmax){
		speed = MAX_SPEED;
	}
	else if ((ADC_avg > Tr) && (ADC_avg < Tavg)){
		speed = MIN_SPEED;
	}
	else if ((ADC_avg >= Tavg) && (ADC_avg < Tmax)){
			speed = speedcalc(Tavg, Tmax, pressurearray);
	}
	return speed;

}

/* int speedcalc(uint32_t Tavg, uint32_t Tmax, int* pressurearray)
 * Speed is deemed as lying in "bucket range" for acceleration computation.
 * Refers to corresponding pressure array for correct hand motion and assigns the
 * correct 5-bits. This is returned. If function returns -1, it entails that a speed bucket
 * could not be found, i.e. something is WRONG!! */
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
