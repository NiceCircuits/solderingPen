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

#include "config.h"
#include <inttypes.h>

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef magnetometer_init();

HAL_StatusTypeDef magnetometer_read(uint16_t* buffer);

#endif /* SRC_MAGNETOMETER_H_ */
