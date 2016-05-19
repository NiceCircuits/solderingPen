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
