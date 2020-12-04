/* This file contains all the functions needed to store threshold values to flash
 *
 *
 * Author: Sumati Gupta
 * Date: 11/25/2020
 *
 * */

/* INCLUDES */
#include "flashStorage.h"

void flash_initialize()
{
	HAL_FLASH_Unlock();
}

void flash_deinitialize()
{
	HAL_FLASH_Lock();
}

void flash_erase(uint8_t sector)
{
//    uint32_t error = 0;
//    FLASH_EraseInitTypeDef FLASH_EraseInitStruct =
//    {
//    	.TypeErase = FLASH_TYPEERASE_SECTORS,
//	.Sector = (uint32_t)sector,
//	.NbSectors = 1,
//	.VoltageRange = FLASH_VOLTAGE_RANGE_3
//    };

 //   HAL_FLASHEx_Erase(&FLASH_EraseInitStruct,&error);
}



void Write_Flash(uint32_t FlashAddress, uint8_t data)
{
     HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FlashAddress, data);
}
