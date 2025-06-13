#ifndef INC_MYPRINTF_H_
#define INC_MYPRINTF_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void myprintf(UART_HandleTypeDef *huart, const char *fmt, ...);

#endif /* INC_MYPRINTF_H_ */
