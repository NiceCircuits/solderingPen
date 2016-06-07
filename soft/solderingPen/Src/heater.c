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
volatile bool heaterPwmRisingEdgeFlag = 0;
/// Flag set when falling edge on heater PWM occurs.
volatile bool heaterPwmFallingEdgeFlag = 0;
/// Is pullup on?
bool is_pullup_on = false;

/**
 * Function used to generate precise delay needed for starting ADC and magnetometer.
 * @param delay Delay in unit of 100us.
 * @return Status.
 */
HAL_StatusTypeDef heaterDelayStart(uint16_t delay) {
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
bool heaterDelayElapsed() {
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
HAL_StatusTypeDef heaterStartPwm() {
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
HAL_StatusTypeDef heaterCmd(uint16_t duty) {
	if (duty > HEATER_PWM_MAX) {
		htim14.Instance->CCR1 = HEATER_PWM_MAX;
		return HAL_ERROR;
	}
	htim14.Instance->CCR1 = duty;
	return HAL_OK;
}

/**
 * Calculate heater driver and sensor diagnostics.
 * @param pwmSet Pointer to new PWM value. It will be changed in this function if no proper diagnostics occurred for
 * a while!
 * @return Diagnostic state.
 */
tip_state_t heater_diagnostics(int32_t *pwmSet) {
	/// Previous value of set PWM
	static int32_t lastPwmSet = 0;
	/// Limits for open load and overload of heater driver.
	int32_t heaterOpenLoadLimit, heaterOveloadLimit;
	/// Counter for time without proper current feedback diagnosis.
	static uint_fast16_t heaterNoFbCounter = 0;
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

	if (is_pullup_on) {
		sensor_pullup_on_value = adcGet(adcSensor);
		HAL_GPIO_WritePin(sensor_pullup_cmd_GPIO_Port, sensor_pullup_cmd_Pin, GPIO_PIN_RESET);
		debugPrint("%d %d\r\n", sensor_pullup_on_value, sensor_pullup_off_value);
		if (sensor_pullup_on_value > (sensor_pullup_off_value + SENSOR_DIAGNOSTIC_THRESHOLD)) {
			sensor_open = true;
		} else {
			sensor_open = false;
		}
	} else {
		sensor_pullup_off_value = adcGet(adcSensor);
		HAL_GPIO_WritePin(sensor_pullup_cmd_GPIO_Port, sensor_pullup_cmd_Pin, GPIO_PIN_SET);
	}
	is_pullup_on = !is_pullup_on;

	// Check if last PWM duty was long enough for correct feedback reading
	if (lastPwmSet >= HEATER_FB_PWM_MIN) {
		heaterOpenLoadLimit = (((int32_t) adcGet(adcVinSenseHeaterOff) * HEATER_OPEN_LOAD_COEF_A)
				>> HEATER_COEF_BIT_SHIFT) + HEATER_OPEN_LOAD_COEF_B;
		heaterOveloadLimit =
				(((int32_t) adcGet(adcVinSenseHeaterOff) * HEATER_OVERLOAD_COEF_A) >> HEATER_COEF_BIT_SHIFT)
						+ HEATER_OVERLOAD_COEF_B;
		if (adcGet(adcDriverFb) < heaterOpenLoadLimit) {
			if (sensor_open) {
				tip_state = TIP_DISCONNECTED;
			} else {
				tip_state = TIP_HEATER_OPEN_LOAD;
			}
		} else if (adcGet(adcDriverFb) > heaterOveloadLimit) {
			tip_state = TIP_HEATER_OVERLOAD;
		} else {
			if (sensor_open) {
				tip_state = TIP_SENSOR_OPEN;
			} else {
				tip_state = TIP_OK;
			}
		}
		heaterNoFbCounter = 0; // Reset no feedback counter.
	} else {
		heaterNoFbCounter++;
		if (heaterNoFbCounter >= HEATER_MAX_TIME_NO_FB) {
			heaterNoFbCounter = 0;
			// Increase PWM duty for one next cycle to gain correct current sense reading.
			if (*pwmSet < HEATER_FB_PWM_MIN) {
				*pwmSet = HEATER_FB_PWM_MIN;
			}
		}
		tip_state = tip_state_last;
	}
	// Store pwmSet value for next run.
	lastPwmSet = *pwmSet;
	tip_state_last = tip_state;
	return tip_state;
}

/**
 * Callback from timer driver.
 * @param htim Pointer to HAL timer structure.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim14) {
		heaterPwmFallingEdgeFlag = true;
	}
}

/**
 * Callback from timer driver.
 * @param htim Pointer to HAL timer structure.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim14) {
		heaterPwmRisingEdgeFlag = true;
	}

}
