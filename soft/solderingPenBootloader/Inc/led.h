/**
 ******************************************************************************
 * @file    led.h
 * @author  piotr@nicecircuits.com
 * @date    2017-03-04
 * @brief
 ******************************************************************************
 */
#ifndef SRC_LED_H_
#define SRC_LED_H_

#include "stm32f0xx_hal.h"
#include "config.h"

enum {
  /// Maximum LED PWM duty.
  LED_PWM_MAX = 65535,
};

/**
 * Start RGB LED PWM timer.
 * @return Status.
 */
HAL_StatusTypeDef led_init();

/**
 * Set PWM duty for RGB LED.
 * @param r Duty for red LED, 0 to LED_PWM_MAX.
 * @param g Duty for green LED, 0 to LED_PWM_MAX.
 * @param b Duty for blue LED, 0 to LED_PWM_MAX.
 * @return Status.
 */
HAL_StatusTypeDef led_cmd(uint16_t r, uint16_t g, uint16_t b);

#endif /* SRC_LED_H_ */
