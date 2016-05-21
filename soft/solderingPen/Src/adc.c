/**
 ******************************************************************************
 * @file    adc.c
 * @author  piotr@nicecircuits.com
 * @date    2016-05-19
 * @brief
 ******************************************************************************
 */

#include "adc.h"
#include "debug.h"

uint16_t adcBuffer[adcNumberOfChannels];

/**
 * Additional initialization of ADC.
 * @return Status.
 */
HAL_StatusTypeDef adcInit() {
	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_ChannelConfTypeDef sConfig;

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Alternate = GPIO_AF0_EVENTOUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	return HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

/**
 * Convert some ADC channels while heater is on.
 * @return Status.
 */
HAL_StatusTypeDef adcConvertWhileHeaterOn() {
	hadc.Instance->CHSELR = ADC_CHSELR_CHANNEL(ADC_CHANNEL_0) /* vin_sense_sig */
	| ADC_CHSELR_CHANNEL(ADC_CHANNEL_1) /* driver_fb_sig */
	| ADC_CHSELR_CHANNEL(ADC_CHANNEL_5); /* potentiometer_sig */
	return HAL_ADC_Start_DMA(&hadc, (uint32_t*)adcBuffer, 3);
}

/**
 * Convert some ADC channels while heater is off.
 * @return Status.
 */
HAL_StatusTypeDef adcConvertWhileHeaterOff() {
	hadc.Instance->CHSELR = ADC_CHSELR_CHANNEL(ADC_CHANNEL_0) /* vin_sense_sig */
	| ADC_CHSELR_CHANNEL(ADC_CHANNEL_3); /* sensor_sig */
	return HAL_ADC_Start_DMA(&hadc, (uint32_t*)(adcBuffer + 3), 2);
}

/**
 * Get previously read ADC conversion result.
 * @param channel Channel to get.
 * @return Conversion value.
 */
uint16_t adcGet(adcChannel_t channel) {
	if (channel < adcNumberOfChannels) {
		return (uint16_t) adcBuffer[channel];
	} else {
		return 0;
	}
}

/**
 * Check if last conversion is ready.
 * @return true if last conversion is ready.
 */
bool adcReady() {
	return HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_READY;
}
