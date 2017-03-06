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
#include <stdbool.h>

/* Private variable declarations ******************************************** */
TIM_HandleTypeDef htim14;
volatile bool timeout_start_flag = false, timeout_stop_flag = false, timeout_elapsed_flag = false;

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

/**
 * Start timeout timer. Timeout is defined in config.h
 */
void software_uart_timeout_start();

/**
 * Stop timeout timer.
 */
void software_uart_timeout_stop();

/**
 * Wait for delay timer to count for half UART bit length. After that timer runs again.
 */
bool software_uart_is_timeout_elapsed();

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

void software_uart_timeout_start() {
  timeout_start_flag = true;
  timeout_elapsed_flag = false;
}

void software_uart_timeout_stop() {
  timeout_stop_flag = true;
}

bool software_uart_is_timeout_elapsed() {
  return timeout_elapsed_flag;
}

/**
 * Callback for systick timer interrupt
 */
void HAL_SYSTICK_Callback() {
  static uint_fast16_t counter = 0;
  static bool timeout_running_flag = false;

  if (timeout_running_flag) {
    counter++;
    if (counter >= UART_TIMEOUT_MS) {
      timeout_running_flag = false;
      timeout_elapsed_flag = true;
    }
  }
  if (timeout_start_flag) {
    counter = 0;
    timeout_running_flag = true;
    timeout_start_flag = false;
  }
  if (timeout_stop_flag) {
    timeout_running_flag = false;
    timeout_stop_flag = false;
  }
}

/* Public function definitions ********************************************** */

HAL_StatusTypeDef software_uart_init() {
  /* TIM14 init function */

  htim14.Instance = DELAY_TIMER;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
#define TIMER_PERIOD ((CORE_FREQ) / (UART_BAUDRATE)/2 - 1)
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
  size_t byte_cnt;
  int_fast8_t bit_cnt;
  uint8_t received_byte=0;

  for (byte_cnt = 0; byte_cnt < length; byte_cnt++) {
    software_uart_timeout_start();
    // wait for start bit
    while (HAL_GPIO_ReadPin(sensor_pullup_cmd_GPIO_Port, sensor_pullup_cmd_Pin) == GPIO_PIN_SET) {
      if (software_uart_is_timeout_elapsed()) {
        return HAL_TIMEOUT;
      }
    }
    software_uart_delay_start();
    software_uart_half_bit_delay_wait();
    // check if start bit is valid
    if (HAL_GPIO_ReadPin(sensor_pullup_cmd_GPIO_Port, sensor_pullup_cmd_Pin) == GPIO_PIN_SET) {
      return HAL_ERROR;
    }
    software_uart_bit_delay_wait();
    // Read bits
    for (bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      received_byte >>= 1;
      received_byte |= HAL_GPIO_ReadPin(sensor_pullup_cmd_GPIO_Port, sensor_pullup_cmd_Pin) << 7;
      software_uart_bit_delay_wait();
    }
    // Check if stop bit is valid
    if (HAL_GPIO_ReadPin(sensor_pullup_cmd_GPIO_Port, sensor_pullup_cmd_Pin) == GPIO_PIN_RESET) {
      return HAL_ERROR;
    }
    software_uart_delay_stop();
    data[byte_cnt] = received_byte;
  }

  return HAL_OK;
}

