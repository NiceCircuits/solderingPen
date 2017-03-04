/**
  ******************************************************************************
  * @file    software_uart.h
  * @author  piotr@nicecircuits.com
  * @date    2017-03-02
  * @brief   Slow, half duplex, software UART for soldering pen bootloader
  ******************************************************************************
*/
#ifndef SOFTWARE_UART_H_
#define SOFTWARE_UART_H_

/* Includes ***************************************************************** */
#include <inttypes.h>
#include "stm32f0xx_hal.h"

/* Public function declarations ********************************************* */
/**
 * Initialize software UART.
 * @return Status
 */
HAL_StatusTypeDef software_uart_init();

/**
 * Send data via software UART.
 * @param data Pointer to data buffer to be sent
 * @param length Number of bytes to be sent
 * @return Status
 */
HAL_StatusTypeDef software_uart_send(uint8_t *data, size_t length);

/**
 * Wait for data to be received by software UART.
 * If timeout occurs, error status is returned
 * @param data Pointer to data buffer to write incoming data
 * @param length Expected number of bytes
 * @return Status
 */
HAL_StatusTypeDef software_uart_receive(uint8_t *data, size_t length);

#endif /* SOFTWARE_UART_H_ */
