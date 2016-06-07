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

#include "stm32f0xx_hal.h"

//=============================================================================
//======================= Settings ============================================
//=============================================================================

//======================= Debug settings =======================================
/// Enable debug UART.
#define DEBUG_ENABLE 1

enum {
	/// Size of debug UART DMA buffer.
	DEBUG_BUFFER_SIZE = 256,
	/// Debug print timeout in ms.
	DEBUG_TIMEOUT = 100,
	/// I2C address to which debug messages are sent (shifted left).
	DEBUG_I2C_ADDR = 0x20,
};

//======================= Timing settings ======================================
enum {
	/// Heater PWM frequency [Hz].
	HEATER_PWM_FREQ = 100,
	/// Heater delay timer frequency (1 / step time) [Hz]
	HEATER_DELAY_FREQ = 10000,
	/// Delay between rising edge of heater PWM (heater on) and ADC conversion in unit of 100us. 1.2 ms - from datasheet.
	ADC_HEATER_ON_DELAY = HEATER_DELAY_FREQ * 12 / 10000,
	/// Delay between falling edge of heater PWM (heater off) and ADC conversion in unit of 100us. 0.7 ms - set experimentally.
	ADC_HEATER_OFF_DELAY = HEATER_DELAY_FREQ * 7 / 10000,
};

//======================= Heater regulator settings ============================
enum {
	/// PWM value for 100% heater PWM duty
	HEATER_PWM_FULL = 9999,
	/// Maximum allowed value of heater PWM duty (for correct reading of sensor voltage)
	HEATER_PWM_MAX = HEATER_PWM_FULL * 90 / 100,
	/// (TODO: 1800) Sensor voltage for minimum temperature in uV.
	SENSOR_VOLTAGE_T_MIN_UV = 1000,
	/// (TODO: 3200)Sensor voltage for maximum temperature in uV.
	SENSOR_VOLTAGE_T_MAX_UV = 3200,
	/// Sensor voltage for standby temperature in uV.
	SENSOR_VOLTAGE_T_STANDBY_UV = 1000,
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
	SENSOR_ADC_T_MIN = SENSOR_VOLTAGE_T_MIN_UV * SENSOR_GAIN_FS / 4 * ADC_FS / SUPPLY_VOLTAGE_UV * 4,
	/// Sensor reading for maximum temperature in LSB.
	SENSOR_ADC_T_MAX = SENSOR_VOLTAGE_T_MAX_UV * SENSOR_GAIN_FS / 8 * ADC_FS / SUPPLY_VOLTAGE_UV * 8,
	/// Sensor reading for maximum temperature in LSB.
	SENSOR_ADC_T_STANDBY = SENSOR_VOLTAGE_T_STANDBY_UV * SENSOR_GAIN_FS / 8 * ADC_FS / SUPPLY_VOLTAGE_UV * 8,
	/*
	 * Potentiometer reading to expected sensor value calculation:
	 * sensor [LSB] = numerator/denominator * potentiometer [LSB] + addend
	 * Denominator is selected to be round number (2^n) to simplify division
	 * into bit shift.
	 */
	/// Bit shift denominator of potentiometer -> expected sensor calculation.
	POT_SENS_DENOMINATOR_BIT_SHIFT = 16,
	/// Numerator of potentiometer -> expected sensor calculation.
	POT_SENS_NUMERATOR = ((SENSOR_ADC_T_MAX - SENSOR_ADC_T_MIN) << POT_SENS_DENOMINATOR_BIT_SHIFT) / ADC_FS,
	/// Addend of potentiometer -> expected sensor calculation.
	POT_SENS_ADDEND = SENSOR_ADC_T_MIN,
};

//======================= diagnostics settings ========================================
enum {
	/*
	 * Open load and overload limit calculation coefficients
	 */
	HEATER_OPEN_LOAD_COEF_A = 5682,
	HEATER_OPEN_LOAD_COEF_B = -24,
	HEATER_OVERLOAD_COEF_A = 15969,
	HEATER_OVERLOAD_COEF_B = 77,
	HEATER_COEF_BIT_SHIFT = 16,
	/// Minimum value of PWM for correct reading of current sense feedback
	HEATER_FB_PWM_MIN = (HEATER_PWM_FULL + 1) * HEATER_PWM_FREQ
			* (ADC_HEATER_ON_DELAY + 3 /* add some time for calculations to perform correctly */) / HEATER_DELAY_FREQ,
	/// Maximum allowed time without correct feedback reading - set to 0.5s
	HEATER_MAX_TIME_NO_FB = HEATER_PWM_FREQ / 2,
	SENSOR_DIAGNOSTIC_THRESHOLD = 250,
};

//======================= LED settings ========================================
enum {
	LED_FULL_BRIGHTNESS = 10000,
	LED_LOW_TEMP_RED_PERCENT = 80,
	LED_STANDBY_MIN = LED_FULL_BRIGHTNESS / 5,
	LED_STANDBY_MAX = LED_FULL_BRIGHTNESS,
	LED_STANDBY_STEP = LED_FULL_BRIGHTNESS / 200,
	LED_DISCONNECTED_MIN = LED_FULL_BRIGHTNESS / 10,
	LED_DISCONNECTED_MAX = LED_FULL_BRIGHTNESS,
	LED_DISCONNECTED_STEP = LED_FULL_BRIGHTNESS / 100,
	LED_ERROR_BLINK_TIME = 5,
	LED_ERROR_CYCLE_TIME = 15
};

//=============================================================================
//======================= Global declarations =================================
//=============================================================================
/// Enumerator for device state
typedef enum {
	/// Invalid state.
	STATE_INVALID = 0,
	/// Temperature under set limit, device is heating up.
	STATE_OK_LOW_TEMP,
	/// Temperature OK, device is working properly.
	STATE_OK,
	/// Temperature over set limit, device is cooling down.
	STATE_OK_HIGH_TEMP,
	/// Standby mode, iron is placed on holder
	STATE_STANDBY,
	/// Tip is disconnected
	STATE_DISCONNECTED,
	/// Supply voltage is too low.
	STATE_LOW_SUPPLY,
	/// Supply voltage is too high.
	STATE_HIGH_SUPPLY,
	/// There is error - heater driver shows overload.
	STATE_ERROR_OVERLOAD,
	/// There is error - heater driver or temperature sensor shows open load.
	STATE_ERROR_OPEN_LOAD,
	/// There is error - tip temperature is over limit.
	STATE_ERROR_HIGH_TEMP,
	/// There is error - tip temperature cannot reach set value.
	STATE_ERROR_LOW_TEMP,
} state_t;

/// Type holding flags indicating minor errors.
typedef enum {
	NO_ERROR_FLAGS = 0, MAGNETOMETER_ERROR_FLAG = 1,
} error_flags_t;

/// Configuration that is saved in EEPROM.
typedef struct {
	int16_t magnetometer_offsets[3];
	char test[100];
} eeprom_config_t;

extern const eeprom_config_t eeprom_config;

HAL_StatusTypeDef eeprom_config_save(eeprom_config_t *new_config);

#endif /* CONFIG_H_ */
