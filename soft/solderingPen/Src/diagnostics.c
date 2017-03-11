/**
 ******************************************************************************
 * @file    diagnostics.c
 * @author  piotr@nicecircuits.com
 * @date    2016-06-11
 * @brief   Universal diagnostics functions
 ******************************************************************************
 */

#include "diagnostics.h"

bool diagnostics_n_from_m(bool value, diagnostics_n_from_m_data_t *config) {
	// Value at config->buffer[config->head] is obsolete. Decrease sum if its true.
	if (config->buffer[config->head] != false) {
		if (config->sum > 0) {
			config->sum--;
		}
	}
	// Store new value
	value = value != false;
	config->buffer[config->head] = value;
	config->sum += value;
	// Go to the next sample in buffer
	config->head++;
	if (config->head >= config->m) {
		config->head = 0;
	}
	if (config->sum >= config->n) {
		return true;
	} else {
		return false;
	}
}
