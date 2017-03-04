/**
 ******************************************************************************
 * @file    config.h
 * @author  piotr@nicecircuits.com
 * @date    2017-03-02
 * @brief   Configuration file for soldering pen bootloader
 ******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Public defines ------------------------------------------------------------*/

#define vin_sense_sig_Pin GPIO_PIN_0
#define vin_sense_sig_GPIO_Port GPIOA
#define driver_fb_sig_Pin GPIO_PIN_1
#define driver_fb_sig_GPIO_Port GPIOA
#define sensor_pullup_cmd_Pin GPIO_PIN_2
#define sensor_pullup_cmd_GPIO_Port GPIOA
#define sensor_sig_Pin GPIO_PIN_3
#define sensor_sig_GPIO_Port GPIOA
#define driver_cmd_Pin GPIO_PIN_4
#define driver_cmd_GPIO_Port GPIOA
#define led_R_cmd_Pin GPIO_PIN_6
#define led_R_cmd_GPIO_Port GPIOA
#define led_G_cmd_Pin GPIO_PIN_7
#define led_G_cmd_GPIO_Port GPIOA
#define led_B_cmd_Pin GPIO_PIN_1
#define led_B_cmd_GPIO_Port GPIOB
#define DELAY_TIMER TIM14

/* Public declarations -------------------------------------------------------*/

void Error_Handler(void);


#endif
