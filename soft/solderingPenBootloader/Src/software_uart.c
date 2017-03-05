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

/* Private variable declarations ******************************************** */
TIM_HandleTypeDef htim14;

/* Private function declarations ******************************************** */

/**
 * Start delay timer. Default delay time is half UART bit length.
 */
HAL_StatusTypeDef software_uart_delay_start();

/**
 * Stop delay timer.
 */
HAL_StatusTypeDef software_uart_delay_stop();

/**
 * Wait for delay timer to count for half UART bit length. After that timer runs again.
 */
void software_uart_half_bit_delay_wait();

/**
 * Wait for delay timer to count for full UART bit length. After that timer runs again.
 */
void software_uart_bit_delay_wait();

/* Private function definitions ********************************************* */

HAL_StatusTypeDef software_uart_delay_start() {
  __HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
  return HAL_TIM_Base_Start(&htim14);
}

HAL_StatusTypeDef software_uart_delay_stop() {
  return HAL_TIM_Base_Stop(&htim14);
}

void software_uart_half_bit_delay_wait() {
  while (__HAL_TIM_GET_FLAG(&htim14, TIM_FLAG_UPDATE) == RESET) {
  }
  __HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
}

void software_uart_bit_delay_wait() {
  software_uart_half_bit_delay_wait();
  software_uart_half_bit_delay_wait();
}

/* Public function definitions ********************************************** */

HAL_StatusTypeDef software_uart_init() {
  /* TIM14 init function */

  htim14.Instance = DELAY_TIMER;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
#define TIMER_PERIOD ((CORE_FREQ) / (BAUDRATE)/2 - 1)
#if TIMER_PERIOD>65535
#error "Bit time too long, use longer prescaler."
#endif
  htim14.Init.Period = TIMER_PERIOD;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
    Error_Handler();
  }
  return HAL_OK;
}

HAL_StatusTypeDef software_uart_send(uint8_t *data, size_t length) {
  size_t byte_cnt;
  int_fast8_t bit_cnt;
  uint8_t byte_to_send;

  software_uart_delay_start();
  for (byte_cnt = 0; byte_cnt < length; byte_cnt++) {
    byte_to_send = data[byte_cnt];
    // Generate start bit
    HAL_GPIO_WritePin(driver_cmd_GPIO_Port, driver_cmd_Pin, GPIO_PIN_SET);
    software_uart_bit_delay_wait();
    // Send bits
    for (bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      HAL_GPIO_WritePin(driver_cmd_GPIO_Port, driver_cmd_Pin, !(byte_to_send & 0x01));
      byte_to_send >>= 1;
      software_uart_bit_delay_wait();
    }
    // Generate stop bit
    HAL_GPIO_WritePin(driver_cmd_GPIO_Port, driver_cmd_Pin, GPIO_PIN_RESET);
    software_uart_bit_delay_wait();
  }

  software_uart_delay_stop();
  return HAL_OK;
}

HAL_StatusTypeDef software_uart_receive(uint8_t *data, size_t length) {
}

;
