/**
 ******************************************************************************
 * @file    led.c
 * @author  piotr@nicecircuits.com
 * @date    2017-03-04
 * @brief
 ******************************************************************************
 */

#include "led.h"

void led_init() {
  /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
  __HAL_RCC_TIM3_CLK_ENABLE()
        ;

  /* Init the base time for the PWM */
  TIM3->CR1 = TIM_COUNTERMODE_UP | TIM_CLOCKDIVISION_DIV1;
  /* Set the Autoreload value */
  TIM3->ARR = (uint32_t) 65535;
  /* Set the Prescaler value */
  TIM3->PSC = (uint32_t) 0;
  /* Generate an update event to reload the Prescaler
   and the repetition counter(only for TIM1 and TIM8) value immediatly */
  TIM3->EGR = TIM_EGR_UG;

  /* Reset the MMS Bits */
  TIM3->CR2 = TIM_TRGO_RESET;
  /* Set or Reset the MSM Bit */
  TIM3->SMCR = TIM_MASTERSLAVEMODE_DISABLE;

  // Configure PWM channels
  TIM3->CCMR1 =
      (TIM_OCMODE_PWM2) // PWM mode 2 for channel 1
      | TIM_CCMR1_OC1PE // Enable preload for channel 1
          | (TIM_OCMODE_PWM2 << 8) // PWM mode 2 for channel 2
          | TIM_CCMR1_OC2PE; // Enable preload for channel 2

  TIM3->CCMR2 =
      (TIM_OCMODE_PWM2 << 8) // PWM mode 2 for channel 4
      | TIM_CCMR2_OC4PE; // Enable preload for channel 4

  /* Enable the Capture compare channels */
  TIM3->CCER = (TIM_CCx_ENABLE << TIM_CHANNEL_1) | (TIM_CCx_ENABLE << TIM_CHANNEL_2)
      | (TIM_CCx_ENABLE << TIM_CHANNEL_4);

  /* Enable the Peripheral */
  TIM3->CR1 |= (TIM_CR1_CEN);
}

void led_deinit() {
  /* Disable the TIM Peripheral Clock */
  if ((TIM3->CCER & TIM_CCER_CCxE_MASK) == 0) {
    if ((TIM3->CCER & TIM_CCER_CCxNE_MASK) == 0) {
      TIM3->CR1 &= ~(TIM_CR1_CEN);
    }
  }
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
  __HAL_RCC_TIM3_CLK_DISABLE();

}

HAL_StatusTypeDef led_cmd(uint16_t r, uint16_t g, uint16_t b) {
  TIM3->CCR1 = r;
  TIM3->CCR2 = g;
  TIM3->CCR4 = b;
  return HAL_OK;
}

