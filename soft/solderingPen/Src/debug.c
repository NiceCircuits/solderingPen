/**
 ******************************************************************************
 * @file    debug.c
 * @author  piotr@nicecircuits.com
 * @date    2016-05-12
 * @brief
 ******************************************************************************
 */

#include "config.h"

#include "debug.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if DEBUG_ENABLE

static char debugUsartBuffer[DEBUG_BUFFER_SIZE] = "UART buffer empty";
/// Debug start flag. Set to one after starting first debug print.
static bool debugStartFlag = 0;

uint_fast8_t debugPrint(const char* format, ...) {
	uint16_t len = 0;
	va_list arglist;

	while ((debugStartFlag == 1)
			&& (HAL_DMA_GetState(&hdma_usart2_tx) != HAL_DMA_STATE_READY)) {
		// wait for previous transfer.
	}
	debugStartFlag = true; // Indicate, that transfer has started.
	// pass variable argument list to vsnprintf function to format
	// formatted string will be available in debugBuffer
	va_start(arglist, format);
	len = len
			+ (uint16_t) vsnprintf(debugUsartBuffer + len,
					(size_t) (DEBUG_BUFFER_SIZE - len - 2), format, arglist);
	va_end(arglist);
	// setup DMA transfer
	HAL_UART_Transmit(&huart2, (uint8_t*) debugUsartBuffer, len, DEBUG_TIMEOUT);
	return HAL_OK;
}

#else // DEBUG_ENABLE
uint_fast8_t debugPrint(const char* format, ...) {
	return HAL_OK;
}
#endif // DEBUG_ENABLE
