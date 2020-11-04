#ifndef INC_MYLIBRARY_H_
#define INC_MYLIBRARY_H_

#include "main.h"
#include <string.h>
//#include "stm32_hal_legacy.h"

extern uint8_t buffer;
extern uint8_t timer_count, buffer_index;
extern UART_HandleTypeDef huart1;

void Message_handler();

#endif /* INC_MYLIBRARY_H_ */
