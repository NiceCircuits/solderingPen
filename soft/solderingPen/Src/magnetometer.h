/**
 ******************************************************************************
 * @file    magnetometer.h
 * @author  piotr@nicecircuits.com
 * @date    2016-06-02
 * @brief
 ******************************************************************************
 */
#ifndef SRC_MAGNETOMETER_H_
#define SRC_MAGNETOMETER_H_

#include "stm32f0xx_hal.h"
#include "stm32f070x6.h"
#include "config.h"
#include <inttypes.h>

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef magnetometerInit();

HAL_StatusTypeDef magnetometerRead(uint16_t* buffer);

#endif /* SRC_MAGNETOMETER_H_ */
