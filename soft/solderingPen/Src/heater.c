/**
 ******************************************************************************
 * @file    heater.c
 * @author  piotr@nicecircuits.com
 * @date    2016-05-19
 * @brief
 ******************************************************************************
 */
#include "heater.h"
#include "led.h"
#include "adc.h"
#include "debug.h"

/// Flag set when rising edge on heater PWM occurs.
volatile bool heater_pwm_rising_edge_flag = 0;
/// Flag set when falling edge on heater PWM occurs.
volatile bool heater_pwm_falling_edge_flag = 0;
/// Is pullup on?
bool heater_is_pullup_on = false;

/**
 * Function used to generate precise delay needed for starting ADC and magnetometer.
 * @param delay Delay in unit of 100us.
 * @return Status.
 */
HAL_StatusTypeDef heater_delay_start(uint16_t delay) {
	HAL_TIM_Base_Stop(&htim1);
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	if (delay == 0) {
		HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
		return HAL_ERROR;
	} else {
		htim1.Instance->ARR = (uint16_t) delay;
		HAL_TIM_Base_Start(&htim1);
		return HAL_OK;
	}
}

/**
 * Check if heater delay is elapsed.
 * @return
 */
bool heater_delay_elapsed() {
	bool result = __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE);
	if (result) {
		__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	}
	return result;
}

/**
 * Start heater PWM.
 * @return Status
 */
HAL_StatusTypeDef heater_start_pwm() {
	HAL_StatusTypeDef result;
	// Start heater PWM timer.
	__HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);
	result = HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
	// Generate event manually at the first cycle.
	HAL_TIM_GenerateEvent(&htim14, TIM_EVENTSOURCE_CC1);
	return result;
}

/**
 * Set Heater PWM duty.
 * @param duty New duty.
 * @return Status.
 */
HAL_StatusTypeDef heater_cmd(uint16_t duty) {
	if (duty > HEATER_PWM_MAX) {
		htim14.Instance->CCR1 = HEATER_PWM_MAX;
		return HAL_ERROR;
	}
	htim14.Instance->CCR1 = duty;
	return HAL_OK;
}

/**
 * Calculate heater driver and sensor diagnostics.
 * @param tip_diagnostics_invalid_flag Set if PWM duty is too low for some time. Indicates that it must be increased for
 * one cycle.
 * @param last_pwm_set Last PWM value.
 * @return Diagnostic state.
 */
tip_state_t heater_diagnostics(bool *tip_diagnostics_invalid_flag, int32_t last_pwm_set) {
	/// Limits for open load and overload of heater driver.
	int32_t heater_open_load_limit, heater_oveload_limit;
	/// Counter for time without proper current feedback diagnosis.
	static uint_fast16_t heater_no_fb_cnt = 0;
	/// Value of sensor input when pullup was off.
	static uint16_t sensor_pullup_off_value = 0;
	/// Value of sensor input when pullup was on.
	uint16_t sensor_pullup_on_value;
	/// State of heater driver load.
	tip_state_t tip_state = TIP_OK;
	/// Previous value of tip state.
	static tip_state_t tip_state_last = TIP_OK;
	/// Is sensor open?
	static bool sensor_open = false;

	if (heater_is_pullup_on) {
		sensor_pullup_on_value = adc_get(ADC_SENSOR);
		HAL_GPIO_WritePin(sensor_pullup_cmd_GPIO_Port, sensor_pullup_cmd_Pin, GPIO_PIN_RESET);
		if (sensor_pullup_on_value > (sensor_pullup_off_value + SENSOR_DIAGNOSTIC_THRESHOLD)) {
			sensor_open = true;
		} else {
			sensor_open = false;
		}
	} else {
		sensor_pullup_off_value = adc_get(ADC_SENSOR);
		HAL_GPIO_WritePin(sensor_pullup_cmd_GPIO_Port, sensor_pullup_cmd_Pin, GPIO_PIN_SET);
	}
	heater_is_pullup_on = !heater_is_pullup_on;

	// Check if last PWM duty was long enough for correct feedback reading
	if (last_pwm_set >= HEATER_FB_PWM_MIN) {
		heater_open_load_limit = (((int32_t) adc_get(ADC_VIN_SENSE_HEATER_OFF) * HEATER_OPEN_LOAD_COEF_A)
				>> HEATER_COEF_BIT_SHIFT) + HEATER_OPEN_LOAD_COEF_B;
		heater_oveload_limit = (((int32_t) adc_get(ADC_VIN_SENSE_HEATER_OFF) * HEATER_OVERLOAD_COEF_A)
				>> HEATER_COEF_BIT_SHIFT) + HEATER_OVERLOAD_COEF_B;
		if (adc_get(ADC_DRIVER_FB) < heater_open_load_limit) {
			if (sensor_open) {
				tip_state = TIP_DISCONNECTED;
			} else {
				tip_state = TIP_HEATER_OPEN_LOAD;
			}
		} else if (adc_get(ADC_DRIVER_FB) > heater_oveload_limit) {
			tip_state = TIP_HEATER_OVERLOAD;
		} else {
			if (sensor_open) {
				tip_state = TIP_SENSOR_OPEN;
			} else {
				tip_state = TIP_OK;
			}
		}
		heater_no_fb_cnt = 0; // Reset no feedback counter.
	} else {
		heater_no_fb_cnt++;
		if (heater_no_fb_cnt >= HEATER_MAX_TIME_NO_FB) {
			heater_no_fb_cnt = 0;
			*tip_diagnostics_invalid_flag = true;
		}
		tip_state = tip_state_last;
	}
	tip_state_last = tip_state;
	return tip_state;
}

/**
 * Callback from timer driver.
 * @param htim Pointer to HAL timer structure.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim14) {
		heater_pwm_falling_edge_flag = true;
	}
}

/**
 * Callback from timer driver.
 * @param htim Pointer to HAL timer structure.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim14) {
		heater_pwm_rising_edge_flag = true;
	}

}
