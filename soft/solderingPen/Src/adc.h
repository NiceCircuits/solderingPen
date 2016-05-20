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
#include "stm32f070x6.h"
#include <stdbool.h>

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

/// ADC channel select.
typedef enum {
	/// Vin sense ADC channel while heater is on.
	adcVinSenseHeaterOn,
	/// Heater driver sense feedback ADC channel.
	adcDriverFb,
	/// Temperature setting potentiometer ADC channel.
	adcPotentiometer,
	/// Vin sense ADC channel while heater is off.
	adcVinSenseHeaterOff,
	/// Tip temperature sensor ADC channel.
	adcSensor,
	/// Number of available ADC channels
	adcNumberOfChannels
} adcChannel_t;

HAL_StatusTypeDef adcInit();

HAL_StatusTypeDef adcConvertWhileHeaterOn();

HAL_StatusTypeDef adcConvertWhileHeaterOff();

uint16_t adcGet(adcChannel_t channel);

bool adcReady();

#endif /* SRC_ADC_H_ */
