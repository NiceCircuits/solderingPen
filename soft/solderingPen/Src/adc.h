/**
 ******************************************************************************
 * @file    adc.h
 * @author  piotr@nicecircuits.com
 * @date    2016-05-19
 * @brief
 ******************************************************************************
 */
#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#include "stm32f0xx_hal.h"
#include <stdbool.h>

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

/// ADC channel select.
typedef enum {
	/// Vin sense ADC channel while heater is on.
	ADC_VIN_SENSE_HEATER_ON,
	/// Heater driver sense feedback ADC channel.
	ADC_DRIVER_FB,
	/// Temperature setting potentiometer ADC channel.
	ADC_POTENTIOMETER,
	/// Vin sense ADC channel while heater is off.
	ADC_VIN_SENSE_HEATER_OFF,
	/// Tip temperature sensor ADC channel.
	ADC_SENSOR,
	/// Number of available ADC channels
	ADC_NUMBER_OF_CHANNELS
} adc_channel_t;

HAL_StatusTypeDef adc_init();

HAL_StatusTypeDef adc_convert_while_heater_on();

HAL_StatusTypeDef adc_convert_while_heater_off();

uint16_t adc_get(adc_channel_t channel);

bool adc_is_ready();

#endif /* SRC_ADC_H_ */
