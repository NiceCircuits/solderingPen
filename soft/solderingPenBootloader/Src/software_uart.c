/**
 ******************************************************************************
 * @file    software_uart.c
 * @author  piotr@nicecircuits.com
 * @date    2017-03-02
 * @brief
 ******************************************************************************
 */

#include "software_uart.h"
#include "config.h"

/* Private variable definitions ********************************************* */
TIM_HandleTypeDef htim;

/* Private function definitions ********************************************* */

/* Public function definitions ********************************************** */

HAL_StatusTypeDef software_uart_init() {
  /* TIM14 init function */

  htim.Instance = DELAY_TIMER;
  htim.Init.Prescaler = 47;
  htim.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim.Init.Period = 0;
  htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim) != HAL_OK) {
    Error_Handler();
  }
}

HAL_StatusTypeDef software_uart_send(uint8_t *data, size_t length) {
}

HAL_StatusTypeDef software_uart_receive(uint8_t *data, size_t length) {
}

