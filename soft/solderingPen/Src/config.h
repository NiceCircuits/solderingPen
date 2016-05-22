/**
 ******************************************************************************
 * @file    config.h
 * @author  piotr@nicecircuits.com
 * @date    2016-05-12
 * @brief   Mainconfiguration file
 ******************************************************************************
 */
#ifndef CONFIG_H_
#define CONFIG_H_

//======================= Debug settings =======================================

/// Enable debug UART.
#define DEBUG_ENABLE 1

enum {
	/// Size of debug UART DMA buffer.
	DEBUG_BUFFER_SIZE = 256,
	/// Debug print timeout in ms.
	DEBUG_TIMEOUT = 100
};

//======================= Timing settings ======================================
enum {
	/// Delay between rising edge of heater PWM (heater on) and ADC conversion.
	ADC_HEATER_ON_DELAY = 20,
	/// Delay between falling edge of heater PWM (heater off) and ADC conversion.
	ADC_HEATER_OFF_DELAY = 5
};

//======================= Heater regulator settings ============================
enum {
	/// (TODO: 1800) Sensor voltage for minimum temperature in uV.
	SENSOR_VOLTAGE_T_MIN_UV = 1000,
	/// (TODO: 3200)Sensor voltage for maximum temperature in uV.
	SENSOR_VOLTAGE_T_MAX_UV = 3200,
	/// Gain of input amplifier.
	SENSOR_GAIN_FS = 181,
	/// Supply voltage in uV.
	SUPPLY_VOLTAGE_UV = 3300000,
	/// ADC reading for full scale.
	ADC_FS = 4095,
	/// Proportional gain of regulator - from sensor to heater PWM.
//	REGULATOR_P = 200,
//	/// Integral gain of regulator - from sensor to heater PWM.
//	REGULATOR_I = 1,
//	/// Derivative gain of regulator - from sensor to heater PWM.
//	REGULATOR_D = 0,
	/// Regulator offset - heater PWM value for regulator zero input.
	REGULATOR_OFFSET = 1000,
	/// Sensor reading for minimum temperature in LSB.
	SENSOR_ADC_T_MIN = SENSOR_VOLTAGE_T_MIN_UV * SENSOR_GAIN_FS / 4 * ADC_FS
			/ SUPPLY_VOLTAGE_UV * 4,
	/// Sensor reading for maximum temperature in LSB.
	SENSOR_ADC_T_MAX = SENSOR_VOLTAGE_T_MAX_UV * SENSOR_GAIN_FS / 8 * ADC_FS
			/ SUPPLY_VOLTAGE_UV * 8,
	/*
	 * Potentiometer reading to expected sensor value calculation:
	 * sensor [LSB] = numerator/denominator * potentiometer [LSB] + addend
	 * Denominator is selected to be round number (2^n) to simplify division
	 * into bit shift.
	 */
	/// Bit shift denominator of potentiometer -> expected sensor calculation.
	POT_SENS_DENOMINATOR_BIT_SHIFT = 16,
	/// Numerator of potentiometer -> expected sensor calculation.
	POT_SENS_NUMERATOR = ((SENSOR_ADC_T_MAX - SENSOR_ADC_T_MIN)
			<< POT_SENS_DENOMINATOR_BIT_SHIFT) / ADC_FS,
	/// Addend of potentiometer -> expected sensor calculation.
	POT_SENS_ADDEND = SENSOR_ADC_T_MIN
};

#endif /* CONFIG_H_ */
