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

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef debug_print(const char* format, ...);

HAL_StatusTypeDef debug_print_raw(const char* data, uint16_t len) ;

#endif /* DEBUG_H_ */
