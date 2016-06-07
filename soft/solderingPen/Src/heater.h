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
#include <stdbool.h>

/// State of heater driver load.
typedef enum {
	/// Invalid state
	TIP_ERROR,
	/// Load is OK, diagnostics inside limits.
	TIP_OK,
	/// Overload on heater detected.
	TIP_HEATER_OVERLOAD,
	/// Open load on heater detected.
	TIP_HEATER_OPEN_LOAD,
	/// Short on sensor detected.
	TIP_SENSOR_SHORT,
	/// Sensor open detected.
	TIP_SENSOR_OPEN,
	/// Tip is disconnected.
	TIP_DISCONNECTED
} tip_state_t;

extern TIM_HandleTypeDef htim14, htim1;

extern volatile bool heaterPwmRisingEdgeFlag;
extern volatile bool heaterPwmFallingEdgeFlag;
extern bool is_pullup_on;

HAL_StatusTypeDef heaterDelayStart(uint16_t delay);

bool heaterDelayElapsed();

HAL_StatusTypeDef heaterStartPwm();

HAL_StatusTypeDef heaterCmd(uint16_t duty);

tip_state_t heater_diagnostics(int32_t *pwmSet);

#endif /* SRC_HEATER_H_ */
