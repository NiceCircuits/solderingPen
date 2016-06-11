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

uint16_t adc_buffer[ADC_NUMBER_OF_CHANNELS] = { (VIN_MIN_LSB + VIN_MAX_LSB) / 2, 0, 0, (VIN_MIN_LSB + VIN_MAX_LSB) / 2,
		0 };

/**
 * Additional initialization of ADC.
 * @return Status.
 */
HAL_StatusTypeDef adc_init() {
	GPIO_InitTypeDef gpio_init_struct;
	ADC_ChannelConfTypeDef adc_config;

	gpio_init_struct.Pin = GPIO_PIN_3;
	gpio_init_struct.Mode = GPIO_MODE_ANALOG;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Alternate = GPIO_AF0_EVENTOUT;
	gpio_init_struct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &gpio_init_struct);
	adc_config.Channel = ADC_CHANNEL_3;
	adc_config.Rank = ADC_RANK_CHANNEL_NUMBER;
	adc_config.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	return HAL_ADC_ConfigChannel(&hadc, &adc_config);
}

/**
 * Convert some ADC channels while heater is on.
 * @return Status.
 */
HAL_StatusTypeDef adc_convert_while_heater_on() {
	hadc.Instance->CHSELR = ADC_CHSELR_CHANNEL(ADC_CHANNEL_0) /* vin_sense_sig */
	| ADC_CHSELR_CHANNEL(ADC_CHANNEL_1) /* driver_fb_sig */
	| ADC_CHSELR_CHANNEL(ADC_CHANNEL_5); /* potentiometer_sig */
	return HAL_ADC_Start_DMA(&hadc, (uint32_t*) adc_buffer, 3);
}

/**
 * Convert some ADC channels while heater is off.
 * @return Status.
 */
HAL_StatusTypeDef adc_convert_while_heater_off() {
	hadc.Instance->CHSELR = ADC_CHSELR_CHANNEL(ADC_CHANNEL_0) /* vin_sense_sig */
	| ADC_CHSELR_CHANNEL(ADC_CHANNEL_3); /* sensor_sig */
	return HAL_ADC_Start_DMA(&hadc, (uint32_t*) (adc_buffer + 3), 2);
}

/**
 * Get previously read ADC conversion result.
 * @param channel Channel to get.
 * @return Conversion value.
 */
uint16_t adc_get(adc_channel_t channel) {
	if (channel < ADC_NUMBER_OF_CHANNELS) {
		return (uint16_t) adc_buffer[channel];
	} else {
		return 0;
	}
}

/**
 * Check if last conversion is ready.
 * @return true if last conversion is ready.
 */
bool adc_is_ready() {
	return HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_READY;
}
