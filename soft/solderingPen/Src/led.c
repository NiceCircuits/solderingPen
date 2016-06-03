/**
 ******************************************************************************
 * @file    led.c
 * @author  piotr@nicecircuits.com
 * @date    2016-05-19
 * @brief
 ******************************************************************************
 */

#include "led.h"

/**
 * Start RGB LED PWM timer.
 * @return Status.
 */
HAL_StatusTypeDef ledStartPwm() {
	HAL_StatusTypeDef status = HAL_OK;
	status |= HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	status |= HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	status |= HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	return status;
}

/**
 * Set PWM duty for RGB LED.
 * @param r Duty for red LED.
 * @param g Duty for green LED.
 * @param b Duty for blue LED.
 * @return Status.
 */
HAL_StatusTypeDef ledCmd(uint16_t r, uint16_t g, uint16_t b) {
	htim3.Instance->CCR1 = r;
	htim3.Instance->CCR2 = g;
	htim3.Instance->CCR4 = b;
	return HAL_OK;
}

/**
 * LED state machine loop
 * @param currentState Current state of the device.
 * @return Status.
 */
// TODO: error status (megnetometer etc.)
HAL_StatusTypeDef ledLoop(state_t currentState) {
	/// State during previous loop.
	static state_t lastState = STATE_INVALID;
	static int32_t cnt = 0;
	static int32_t dir = 1;
	if (currentState != lastState) {
		cnt = 0;
		dir = 0;
	}
	switch (currentState) {
	case STATE_OK_HIGH_TEMP:
		ledCmd(LED_FULL_BRIGHTNESS, 0, 0);
		break;
	case STATE_OK:
		ledCmd(0, LED_FULL_BRIGHTNESS, 0);
		break;
	case STATE_OK_LOW_TEMP:
		ledCmd(LED_FULL_BRIGHTNESS * LED_LOW_TEMP_RED_PERCENT / 100,
				LED_FULL_BRIGHTNESS * (100 - LED_LOW_TEMP_RED_PERCENT) / 100,
				0);
		break;
	case STATE_STANDBY:
		cnt += dir;
		if (cnt >= LED_STANDBY_MAX) {
			dir = -LED_STANDBY_STEP;
		}
		if (cnt <= LED_STANDBY_MIN) {
			dir = LED_STANDBY_STEP;
		}
		ledCmd(0, 0, (uint16_t) cnt);
		break;
	case STATE_DISCONNECTED:
		cnt += dir;
		if (cnt >= LED_DISCONNECTED_MAX) {
			dir = -LED_DISCONNECTED_STEP;
		}
		if (cnt <= LED_DISCONNECTED_MIN) {
			dir = LED_DISCONNECTED_STEP;
		}
		ledCmd(0, (uint16_t) cnt, 0);
		break;
	case STATE_ERROR_OVERLOAD:
	case STATE_ERROR_OPEN_LOAD:
	case STATE_INVALID:
	case STATE_LOW_SUPPLY:
	case STATE_HIGH_SUPPLY:
	case STATE_ERROR_HIGH_TEMP:
	case STATE_ERROR_LOW_TEMP:
	default:
		cnt++;
		if (cnt == LED_ERROR_BLINK_TIME) {
			ledCmd(LED_FULL_BRIGHTNESS, 0, 0);
		}
		if (cnt >= LED_ERROR_CYCLE_TIME) {
			cnt = 0;
			ledCmd(0, 0, 0);
		}
		break;
	}
	lastState = currentState;
	return HAL_OK;
}
