/**
 ******************************************************************************
 * @file    heater.c
 * @author  piotr@nicecircuits.com
 * @date    2016-05-19
 * @brief
 ******************************************************************************
 */
#include "heater.h"

/// Flag set when rising edge on heater PWM occurs.
volatile bool heaterPwmRisingEdgeFlag = 0;
/// Flag set when falling edge on heater PWM occurs.
volatile bool heaterPwmFallingEdgeFlag = 0;

/**
 * Start heater PWM.
 * @return Status
 */
HAL_StatusTypeDef heaterStartPwm() {
	// Start heater PWM timer.
	__HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);
	return HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
}

/**
 * Set Heater PWM duty.
 * @param duty New duty.
 * @return Status.
 */
HAL_StatusTypeDef heaterCmd(uint32_t duty) {
	if (duty > HEATER_PWM_MAX) {
		htim14.Instance->CCR1 = HEATER_PWM_MAX;
		return HAL_ERROR;
	}
	htim14.Instance->CCR1 = duty;
	return HAL_OK;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim14) {
		heaterPwmFallingEdgeFlag = true;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim14) {
		heaterPwmRisingEdgeFlag = true;
	}
}
