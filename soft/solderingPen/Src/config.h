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
enum{
	/// Delay between rising edge of heater PWM (heater on) and ADC conversion.
	ADC_HEATER_ON_DELAY=20,
	/// Delay between falling edge of heater PWM (heater off) and ADC conversion.
	ADC_HEATER_OFF_DELAY=5
};

#endif /* CONFIG_H_ */
