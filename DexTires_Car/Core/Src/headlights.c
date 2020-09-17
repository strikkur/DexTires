/*
 * headlights.c
 *
 *  Created on: Sep 15, 2020
 *      Author: Lakshya Goyal
 */

#include "main.h"
#include "stm32f0xx_it.h"
#include "headlights.h"

GPIO_TypeDef* GPIOxHeadlights;
uint16_t PinFrontLeft;
uint16_t PinFrontRight;
uint16_t PinBackLeft;
uint16_t PinBackRight;

/*
 * Registers the Headlight LEDs to the specified
 * GPIO Port and Pins.
 *
 * Param: GPIOx
 * Pointer to GPIOx Port Base address
 *
 * Params: The rest of the parameters are the GPIO_Pin
 * numbers for the respective headlight.
 */
void SetHeadlightIO(GPIO_TypeDef* GPIOx, uint16_t FrontLeft, uint16_t FrontRight,
		uint16_t BackLeft, uint16_t BackRight) {
	GPIOxHeadlights = GPIOx;
	PinFrontLeft = FrontLeft;
	PinFrontRight = FrontRight;
	PinBackLeft = BackLeft;
	PinBackRight = BackRight;
}

/*
void SetIOFrontLeftHL(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIOxFrontLeft = GPIOx;
}

void SetIOFrontRightHL(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIOxFrontRight = GPIOx;
}

void SetIOBackLeftHL(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIOxBackLeft = GPIOx;
}

void SetIOBackRightHL(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIOxBackRight = GPIOx;
} */

/*
 * Sets the lights based on the specified indications.
 *
 * Param: indication
 * Enum to select which lights will be turned on.
 *
 * Note: 'indication' can be OR-red together for different
 * combination of lights.
 */
void SetLights(int indication) {
	HAL_GPIO_WritePin(GPIOxHeadlights, PinFrontLeft, (indication & Front) | (indication & Left));
	HAL_GPIO_WritePin(GPIOxHeadlights, PinFrontRight, (indication & Front) | (indication & Right));
	HAL_GPIO_WritePin(GPIOxHeadlights, PinBackLeft, (indication & Back) | (indication & Left));
	HAL_GPIO_WritePin(GPIOxHeadlights, PinBackRight, (indication & Back) | (indication & Right));
}
