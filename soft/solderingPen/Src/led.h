/**
 ******************************************************************************
 * @file    led.h
 * @author  piotr@nicecircuits.com
 * @date    2016-05-19
 * @brief
 ******************************************************************************
 */
#ifndef SRC_LED_H_
#define SRC_LED_H_

#include "stm32f0xx_hal.h"
#include "stm32f070x6.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim3;

enum {
	/// Maximum LED PWM duty.
	LED_PWM_MAX = 65535,
};

HAL_StatusTypeDef ledStartPwm();

HAL_StatusTypeDef ledCmd(uint16_t r, uint16_t g, uint16_t b);

#endif /* SRC_LED_H_ */
