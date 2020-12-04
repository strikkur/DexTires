/* This file contains all the functions needed to store threshold values to flash
 *
 *
 * Author: Sumati Gupta
 * Date: 11/25/2020
 *
 * */

#ifndef INC_FLASHSTORAGE_H_
#define INC_FLASHSTORAGE_H_

/* INCLUDES */
#include "main.h"


/* FUNCTIONS */
void flash_initialize();
void flash_deinitialize();
void flash_erase(uint8_t);
void Write_Flash(uint32_t , uint8_t);


#endif /* INC_FLASHSTORAGE_H_ */

