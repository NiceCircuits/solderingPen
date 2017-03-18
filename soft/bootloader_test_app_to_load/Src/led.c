/**
 ******************************************************************************
 * @file    led.c
 * @author  piotr@nicecircuits.com
 * @date    2017-03-04
 * @brief
 ******************************************************************************
 */

#include "led.h"

TIM_HandleTypeDef htim3;

HAL_StatusTypeDef led_init() {
  HAL_StatusTypeDef status = HAL_OK;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

  status |= HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  status |= HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  status |= HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  return status;
}

HAL_StatusTypeDef led_cmd(uint16_t r, uint16_t g, uint16_t b) {
  htim3.Instance->CCR1 = r;
  htim3.Instance->CCR2 = g;
  htim3.Instance->CCR4 = b;
  return HAL_OK;
}

