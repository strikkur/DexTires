/* This file contains all the functions needed to calculate resting threshold, average threshold,
 * and k-values for all hand motions.
 *
 * Author: Sumati Gupta & Samyukta Trikkur
 * Date: 11/17/2020
 *
 * */

/* INCLUDES */
#include "speedConstantsSettings.h"
/* VARIABLES */

/* FUNCTIONS */

/* uint32_t Tavgcalc(uint32_t Tmax, uint32_t Tr)
 * Function that computes the average threshold pressure for each hand motion. */
uint32_t Tavgcalc(uint32_t Tmax, uint32_t Tr){
	return floor((Tmax+Tr)/2);
}

/* int kcalc(uint32_t Tavg, uint32_t Tmax)
 * Function that computes length of bucket (marked as k-value) for each hand motion. */
int kcalc(uint32_t Tavg, uint32_t Tmax){
	return floor((Tmax - Tavg)/29);
}

/* void fourTavg()
 * Function that sets global Tavg values for each hand motion (Tavgcalc gets called four different times). */
void fourTavg(){
	frontTavg = Tavgcalc(frontTmax, frontTr);
	reverseTavg = Tavgcalc(reverseTmax, reverseTr);
	leftTavg = Tavgcalc(leftTmax, leftTr);
	rightTavg = Tavgcalc(rightTmax, rightTr);
}

/* void fourk()
 * Function that sets global k-values for each hand motion (kcalc gets called four different times). */
void fourk(){
	frontk = kcalc(frontTavg, frontTmax);
	reversek = kcalc(reverseTavg, reverseTmax);
	leftk = kcalc(leftTavg, leftTmax);
	rightk = kcalc(rightTavg, rightTavg);
}

/* void pressureArrayInit()
 * Function that sets global pressure array values based on k-length bucket for each hand motion.
 * Should only get called once all four k-values and Tavg values are calculated.
 * These arrays will be used as lookup tables during speed value transmission. */
void pressureArrayInit() {
	int i;
	for (i = 0; i < 29; i++){
		pressurefront[i] = frontTavg + ((i + 1) * frontk);
	}
	for (i = 0; i < 29; i++){
			pressurereverse[i] = reverseTavg + ((i + 1) * reversek);
		}
	for (i = 0; i < 29; i++){
			pressureleft[i] = leftTavg + ((i + 1) * leftk);
		}
	for (i = 0; i < 29; i++){
			pressureright[i] = rightTavg + ((i + 1) * rightk);
		}
}
