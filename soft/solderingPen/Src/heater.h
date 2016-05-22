/**
 ******************************************************************************
 * @file    heater.h
 * @author  piotr@nicecircuits.com
 * @date    2016-05-19
 * @brief
 ******************************************************************************
 */
#ifndef SRC_HEATER_H_
#define SRC_HEATER_H_

#include "stm32f0xx_hal.h"
#include "stm32f070x6.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim14, htim1;

extern volatile bool heaterPwmRisingEdgeFlag;
extern volatile bool heaterPwmFallingEdgeFlag;

HAL_StatusTypeDef heaterDelayStart(uint16_t delay);

bool heaterDelayElapsed();

HAL_StatusTypeDef heaterStartPwm();

HAL_StatusTypeDef heaterCmd(uint16_t duty);

enum {
	/// Value for 100% duty
	HEATER_PWM_FULL = 9999,
	/// Maximum allowed value of heater PWM duty
	HEATER_PWM_MAX = HEATER_PWM_FULL * 93 / 100,
};

#endif /* SRC_HEATER_H_ */
