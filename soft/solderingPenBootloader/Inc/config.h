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

/// Default core frequency
#define CORE_FREQ  48000000
/// Baud rate of bootloader
#define UART_BAUDRATE  1200
/// Timeout for UART reception
#define UART_TIMEOUT_MS 2000
/// Address of first byte of application
#define APP_START_ADDR 0x08003000
/// Size of application info block
#define APP_INFO_SIZE 1024
/// Address of application info block in Flash
#define APP_INFO_ADDR (FLASH_BANK1_END + 1 - APP_INFO_SIZE)
/// Block size of data to write
#define WRITE_BLOCK_SIZE 128
/// Length of CRC in bytes
#define CRC_SIZE 4
/// Threshold for voltage on sensor_sig to detect bootloader
#define SENSOR_BOOT_THRESHOLD ((uint16_t)(0.9 * 4096))

/// Bootloader commands
enum {
  COMMAND_ERASE = 0x81, COMMAND_WRITE = 0x82, COMMAND_READ_INFO = 0x84, COMMAND_RUN_APP = 0x88
};
/// Bootloader responses
enum {
  RESPONSE_OK = 'o', RESPONSE_ERROR = 'e'
};

/* Public declarations for main.c --------------------------------------------*/

void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#endif
