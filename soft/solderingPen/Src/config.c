/** ******************************************************************************
 * @file    config.c
 * @author  piotr@nicecircuits.com
 * @date    2016-06-04
 * @brief
 ******************************************************************************
 */

#include "config.h"

__attribute__((__section__(".eeprom"))) const eeprom_config_t eeprom_config;

HAL_StatusTypeDef eeprom_config_save(eeprom_config_t *new_config) {
	HAL_StatusTypeDef result;
	FLASH_EraseInitTypeDef erase_init;
	uint32_t err, i;
	uint32_t *source, *destination;
	const uint32_t size = (sizeof(eeprom_config_t) + sizeof(uint32_t) - 1) / sizeof(uint32_t);

	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.NbPages = 1;
	erase_init.PageAddress = (uint32_t) &eeprom_config;

	result = HAL_FLASH_Unlock();
	if (result != HAL_OK) {
		return result;
	}
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_BSY | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
	result = HAL_FLASHEx_Erase(&erase_init, &err);
	if (result != HAL_OK) {
		return result;
	}
	if (err != 0xFFFFFFFF) {
		return HAL_ERROR;
	}
	source = (uint32_t*) new_config;
	destination = (uint32_t*) &eeprom_config;
	for (i = 0; i < size; i++) {
		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) (destination + i), source[i]);
		if (result != HAL_OK) {
			return result;
		}
	}
	result = HAL_FLASH_Lock();
	return result;
}

