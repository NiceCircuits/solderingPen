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

#endif /* CONFIG_H_ */
