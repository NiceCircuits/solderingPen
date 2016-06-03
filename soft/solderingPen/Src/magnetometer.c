/**
 ******************************************************************************
 * @file    magnetometer.c
 * @author  piotr@nicecircuits.com
 * @date    2016-06-02
 * @brief
 ******************************************************************************
 */

#include "magnetometer.h"
#include "debug.h"
#include <math.h>

static HAL_StatusTypeDef magnetometer_error = HAL_OK;

/// MAG3110 register addresses
typedef enum {
	MAG_REGISTER_DR_STATUS = 0,
	MAG_REGISTER_OUT_X_MSB = 1,
	MAG_REGISTER_OUT_X_LSB = 2,
	MAG_REGISTER_OUT_Y_MSB = 3,
	MAG_REGISTER_OUT_Y_LSB = 4,
	MAG_REGISTER_OUT_Z_MSB = 5,
	MAG_REGISTER_OUT_Z_LSB = 6,
	MAG_REGISTER_WHO_AM_I = 7,
	MAG_REGISTER_SYSMOD = 8,
	MAG_REGISTER_OFF_X_MSB = 9,
	MAG_REGISTER_OFF_X_LSB = 10,
	MAG_REGISTER_OFF_Y_MSB = 11,
	MAG_REGISTER_OFF_Y_LSB = 12,
	MAG_REGISTER_OFF_Z_MSB = 13,
	MAG_REGISTER_OFF_Z_LSB = 14,
	MAG_REGISTER_DIE_TEMP = 15,
	MAG_REGISTER_CTRL_REG1 = 16,
	MAG_REGISTER_CTRL_REG2 = 17
} magRegister_t;

/// Aliases for magnetometer control and status registers bits.
enum {
	MAG_CTRL_REG1_DR_BITS = 5,
	MAG_CTRL_REG1_OS_BITS = 3,
	MAG_CTRL_REG1_FR_BIT = 2,
	MAG_CTRL_REG1_TM_BIT = 1,
	MAG_CTRL_REG1_AC_BIT = 0,
	MAG_CTRL_REG2_AUTO_MRST_EN_BIT = 7,
	MAG_CTRL_REG2_RAW_BIT = 5,
	MAG_CTRL_REG2_MAG_RST_BIT = 4,
	MAG_DR_STATUS_ZYXDR_BIT = 3
};

/// Values of register
enum {
	/// Value of MAG_REGISTER_SYSMOD register in standby mode.
	MAG_SYSMOD_STANDBY = 0,
	MAG_DR_OS_80Hz = (0 << MAG_CTRL_REG1_DR_BITS) | (0 << MAG_CTRL_REG1_OS_BITS),
};

enum {
/// MAG3110 I2C address (shifted left)
	MAG_I2C_ADDR = 0x1E,
/// Magnetometer i2c transmission timeout [ms]
	MAG_TIMEOUT = 200,
/// Magnetometer initialization timeout, needed for standby mode check [ms]
	MAG_INIT_TIMEOUT = 500,
/// Maximum values of XYZ channels possible to read from magnetometer.
	MAX_XYZ_MAX = 20000,
};

/**
 * Read mangetometer register(s)
 * @param registerAddr Address of register to read.
 * @param buffer Pointer to buffer to store data.
 * @param size Number of bytes to read.
 * @return Status.
 */
HAL_StatusTypeDef magnetometerReadRegister(magRegister_t registerAddr, uint8_t* buffer, uint16_t size) {
	return HAL_I2C_Mem_Read(&hi2c1, MAG_I2C_ADDR, registerAddr, I2C_MEMADD_SIZE_8BIT, buffer, size, MAG_TIMEOUT);
}

/**
 * Write mangetometer register(s)
 * @param registerAddr Address of register to read.
 * @param buffer Pointer to buffer to store data.
 * @param size Number of bytes to read.
 * @return Status.
 */
HAL_StatusTypeDef magnetometerWriteRegister(magRegister_t registerAddr, uint8_t* buffer, uint16_t size) {
	return HAL_I2C_Mem_Write(&hi2c1, MAG_I2C_ADDR, registerAddr, I2C_MEMADD_SIZE_8BIT, buffer, size, MAG_TIMEOUT);
}

/**
 * Initialize MAG3110 magnetometer.
 * @return Status.
 */
HAL_StatusTypeDef magnetometerInit() {
	uint8_t dataBuffer[2];
	uint32_t end_time;
	HAL_StatusTypeDef result;
// First read control register - only change of STANDBY mode is allowed if magnetometer is in active state.
	result = magnetometerReadRegister(MAG_REGISTER_CTRL_REG1, dataBuffer, 1);
	if (result != HAL_OK) {
		magnetometer_error = result;
		return result;
	}
	dataBuffer[0] &= (uint8_t) ~(1 << MAG_CTRL_REG1_AC_BIT); // Set standby mode.
	result = magnetometerWriteRegister(MAG_REGISTER_CTRL_REG1, dataBuffer, 1); // Write back.
	if (result != HAL_OK) {
		magnetometer_error = result;
		return result;
	}
	end_time = HAL_GetTick() + MAG_INIT_TIMEOUT; // Calculate timeout end.
	do {
		result = magnetometerReadRegister(MAG_REGISTER_SYSMOD, dataBuffer, 1);
		if (result != HAL_OK) {
			magnetometer_error = result;
			return result;
		}
		if (HAL_GetTick() >= end_time) {
			magnetometer_error = HAL_TIMEOUT;
			return HAL_TIMEOUT;
		}
	} while (dataBuffer[0] != MAG_SYSMOD_STANDBY); // Wait for standby mode to start.
	dataBuffer[0] = MAG_DR_OS_80Hz | (1 << MAG_CTRL_REG1_AC_BIT); // Enable magnetometer with 80Hz data rate.
	// Enable auto reset and RAW readings.
	dataBuffer[1] = (1 << MAG_CTRL_REG2_AUTO_MRST_EN_BIT) | (1 << MAG_CTRL_REG2_RAW_BIT);
	result = magnetometerWriteRegister(MAG_REGISTER_CTRL_REG1, dataBuffer, 2); // Write configuration.
	magnetometer_error = result;
	return result;
}

/**
 * Read value from magnetometer. Magnitude of magnetic field including calibration offset is calculated.
 * @param buffer Buffer to store output value. Valid if HAL_OK is returned.
 * @return Status.
 */
HAL_StatusTypeDef magnetometerRead(uint16_t* buffer) {
	uint8_t dataBuffer[7];
	HAL_StatusTypeDef result;
	uint16_t data[3], temp;
	uint_fast8_t i, n;

	if (magnetometer_error != HAL_OK) {
		// Errors during initialization - magnetometer is not working properly
		return magnetometer_error;
	} else {

		result = magnetometerReadRegister(MAG_REGISTER_DR_STATUS, dataBuffer, 7);
		if (result != HAL_OK) {
			return result;
		}
		if (((dataBuffer[0] >> MAG_DR_STATUS_ZYXDR_BIT) & 1) == 0) {
			// No new data is available.
			debugPrint("-\r\n");
			return HAL_BUSY;
		} else {
			// New data is available.
			// Read data to second buffer.
			for (i = 0; i < 3; i++) {
				data[i] = (uint16_t) imaxabs((int16_t) ((dataBuffer[i * 2 + 1] << 8) + dataBuffer[i * 2 + 2]));
				if (data[i] > MAX_XYZ_MAX) {
					return HAL_ERROR; // Invalid data
				}
			}
			// Bubble sort from smallest to largest
			for (n = 2; n > 0; n--) {
				for (i = 0; i < n; i++) {
					if (data[i] > data[i + 1]) {
						temp = data[i];
						data[i] = data[i + 1];
						data[i + 1] = temp;
					}
				}
			}
			// Calculate approximation of magnitude
			*buffer = (uint16_t) (data[2] + data[1] / 2 + data[0] / 4);
			return HAL_OK;
		}
	}
}
