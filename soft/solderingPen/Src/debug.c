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

/**
 * Prints to debug interface. Uses printf format and variable list of arguments.
 * @param format printf-like format string.
 * @return Status.
 */
HAL_StatusTypeDef debugPrint(const char* format, ...) {
	uint16_t len = 0;
	va_list arglist;

	// pass variable argument list to vsnprintf function to format
	// formatted string will be available in debugBuffer
	va_start(arglist, format);
	len = len + (uint16_t) vsnprintf(debugUsartBuffer + len, (size_t) (DEBUG_BUFFER_SIZE - len - 2), format, arglist);
	va_end(arglist);
	// setup I2C transfer
	HAL_I2C_Master_Transmit(&hi2c1, DEBUG_I2C_ADDR, debugUsartBuffer, len, DEBUG_TIMEOUT);
	return HAL_OK;
}

/**
 * Prints raw data to debug interface.
 * @param data Data to be sent.
 * @param len Length of data to be sent.
 * @return Status.
 */
HAL_StatusTypeDef debug_print_raw(const char* data, uint16_t len) {
	HAL_I2C_Master_Transmit(&hi2c1, DEBUG_I2C_ADDR, data, len, DEBUG_TIMEOUT);
	return HAL_OK;
}

#else // DEBUG_ENABLE
uint_fast8_t debugPrint(const char* format, ...) {
	return HAL_OK;
}
#endif // DEBUG_ENABLE
