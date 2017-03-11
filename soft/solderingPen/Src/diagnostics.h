/**
 ******************************************************************************
 * @file    diagnostics.h
 * @author  piotr@nicecircuits.com
 * @date    2016-06-11
 * @brief
 ******************************************************************************
 */
#ifndef SRC_DIAGNOSTICS_H_
#define SRC_DIAGNOSTICS_H_

#include "stm32f0xx_hal.h"
#include <stdbool.h>

/// Structure holding data for diagnostics_n_from_m function
typedef struct {
	/// Number of samples needed to be true to give positive result.
	uint8_t n;
	/// Total number of samples taken into account.
	uint8_t m;
	/// Last sample number.
	uint8_t head;
	/// Moving sum of last samples
	uint8_t sum;
	/// Cyclic buffer holding successive values.
	bool *buffer;
} diagnostics_n_from_m_data_t;

bool diagnostics_n_from_m(bool value, diagnostics_n_from_m_data_t *config);

#endif /* SRC_DIAGNOSTICS_H_ */
