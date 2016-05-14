/**
 ******************************************************************************
 * @file    debug.h
 * @author  piotr@nicecircuits.com
 * @date    2016-05-12
 * @brief
 ******************************************************************************
 */
#ifndef DEBUG_H_
#define DEBUG_H_

#include <inttypes.h>
#include "config.h"
#include <stdbool.h>
#include "stm32f0xx_hal.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;


/**
 * Prints to debug UART. Uses printf format and variable list of arguments.
 * @param format printf-like format string.
 * @return 0 if OK
 */
uint_fast8_t debugPrint(const char* format, ...);

#endif /* DEBUG_H_ */
