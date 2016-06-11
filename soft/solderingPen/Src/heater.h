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

extern volatile bool heater_pwm_rising_edge_flag;
extern volatile bool heater_pwm_falling_edge_flag;
extern bool heater_is_pullup_on;

HAL_StatusTypeDef heater_delay_start(uint16_t delay);

bool heater_delay_elapsed();

HAL_StatusTypeDef heater_start_pwm();

HAL_StatusTypeDef heater_cmd(uint16_t duty);

tip_state_t heater_diagnostics(bool *tip_diagnostics_invalid_flag, int32_t last_pwm_set);

#endif /* SRC_HEATER_H_ */
