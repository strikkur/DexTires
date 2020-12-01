/*
 * headlights.h
 *
 *  Created on: Sep 15, 2020
 *      Author: Lakshya Goyal
 */

/****************************************************
* PREREQUISITE:
* All Headlights should be connected to the same
* GPIO Port e.g. PA2, PA3, PA4, PA4.
*
* USAGE:
* Call the SetHeadlightIO() function to register
* the GPIO Port and each GPIO Pin number for each
* Headlight.
****************************************************/

#ifndef INC_HEADLIGHTS_H_
#define INC_HEADLIGHTS_H_

enum LightIndication {
	Off = 0x0,
	Front = 0x1 << 0,
	Back = 0x1 << 1,
	Left = 0x1 << 2,
	Right = 0x1 << 3
};

void SetHeadlightIO(GPIO_TypeDef* GPIOx, uint16_t FrontLeft, uint16_t FrontRight,
		uint16_t BackLeft, uint16_t BackRight);

void SetLights(int indication);

#endif /* INC_HEADLIGHTS_H_ */
