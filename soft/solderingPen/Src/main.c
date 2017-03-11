/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "debug.h"
#include "heater.h"
#include "led.h"
#include "adc.h"
#include "magnetometer.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/// State of heater PWM state machine
typedef enum {
	/// Invalid state.
	HEATER_PWM_STATE_INVALID,
	/// Heater is on, waiting for delay to update measurements.
	HEATER_PWM_STATE_ON_DELAY,
	/// Heater is on, updating measurements.
	HEATER_PWM_STATE_ON_BUSY,
	/// Heater is on, waiting for turn off.
	HEATER_PWM_STATE_ON_IDLE,
	/// Heater is off, waiting for delay to update measurements.
	HEATER_PWM_STATE_OFF_DELAY,
	/// Heater is off, updating measurements.
	HEATER_PWM_STATE_OFF_BUSY,
	/// Heater is off, waiting for turn on.
	HEATER_PWM_STATE_OFF_IDLE
} heater_pwm_state_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	/// State of heater PWM state machine
	heater_pwm_state_t heater_pwm_state = HEATER_PWM_STATE_INVALID;
	/// Temperature set
	uint16_t temperature_set_lsb = 0;
	/// Error of temperature set
	int32_t error_temp_lsb = 0;
	/// Previous value of error of temperature set
	int32_t error_temp_last_lsb = 0;
	/// Integral of error of temperature set over time
	int32_t error_temp_integral_lsb = 0;
	/// Value of PWM to be set
	int32_t pwm_set = 0;
	/// Previous value of PWM.
	static int32_t pwm_set_last = 0;
	/// Proportional gain of regulator - from sensor to heater PWM.
	volatile int32_t REGULATOR_P = 512;
	/// Integral gain of regulator - from sensor to heater PWM.
	volatile int32_t REGULATOR_I = 0;
	/// Derivative gain of regulator - from sensor to heater PWM.
	volatile int32_t REGULATOR_D = 0;
	/// Current state of device.
	state_t current_state = STATE_OK;
	/// Minor error flags
	error_flags_t error_flags = NO_ERROR_FLAGS;
	/// Variable for holding status of functions.
	HAL_StatusTypeDef status;
	/// Previous magnetometer status
	HAL_StatusTypeDef mag_status_last = HAL_OK;
	/// Value read from magnetometer.
	uint16_t magnetometer;
	/// State of heater driver load.
	tip_state_t tip_state = TIP_OK;
	/// Flag for indicating invalid diagnostics (too low PWM)
	bool tip_diagnostics_invalid_flag = false;

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	MX_TIM14_Init();
	MX_TIM1_Init();

	/* USER CODE BEGIN 2 */
	adc_init();
	heater_init();
	led_start_pwm();
	heater_pwm_state = HEATER_PWM_STATE_OFF_IDLE;
	status = magnetometer_init();
	if (status != HAL_OK) {
		error_flags |= MAGNETOMETER_ERROR_FLAG;
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// ==================== PWM phase state machine =====================
		switch (heater_pwm_state) {
		case HEATER_PWM_STATE_ON_DELAY:
			// Heater is on, waiting for delay to update measurements.
			if (heater_delay_elapsed()) {
				heater_pwm_state = HEATER_PWM_STATE_ON_BUSY;
			}
			break;
		case HEATER_PWM_STATE_ON_BUSY:
			// Heater is on, updating measurements.
			adc_convert_while_heater_on();
			// ========== Read and process magnetometer ===========
			status = magnetometer_read(&magnetometer);
			if (status == HAL_OK) {
				// Clear error flag.
				error_flags &= (~MAGNETOMETER_ERROR_FLAG);
			} else {
				/* Cannot read new value from magnetometer. It can happen once per cycle, as magnetometer data rate is
				 * 80 Hz and loop cycle is 100 Hz. */
				if (mag_status_last != HAL_OK) {
					// Cannot read new value twice. Set error flag.
					error_flags |= MAGNETOMETER_ERROR_FLAG;
				}
			}
			mag_status_last = status;
			// --------- heater diagnostics ---------
			tip_state = heater_diagnostics(&tip_diagnostics_invalid_flag, pwm_set_last);
			// ==================== Device state machine =====================
			switch (current_state) {
			case STATE_OK_LOW_TEMP:
			case STATE_OK:
			case STATE_OK_HIGH_TEMP:
			case STATE_STANDBY:
				// ========== Heater regulator control. ===========
				if (!heater_is_pullup_on) {
					// Only if pullup is off.
					// Calculate desired temperature.
					if (current_state == STATE_STANDBY) {
						// Set standby temperature.
						temperature_set_lsb = SENSOR_ADC_T_STANDBY;
					} else {
						// Set temperature set by potentiometer.
						temperature_set_lsb = (uint16_t) ((((uint32_t) adc_get(ADC_POTENTIOMETER) * POT_SENS_NUMERATOR)
								>> POT_SENS_DENOMINATOR_BIT_SHIFT) + POT_SENS_ADDEND);
					}
					// Save previous value of error.
					error_temp_last_lsb = error_temp_lsb;
					// Calculate actual value of temperature error.
					error_temp_lsb = (int32_t) temperature_set_lsb - (int32_t) adc_get(ADC_SENSOR);
					// Calculate integral of temperature error.
					if ((error_temp_integral_lsb + error_temp_lsb) > 100) {
						// Overflow.
						error_temp_integral_lsb = 100;
					} else if ((error_temp_integral_lsb + error_temp_lsb) < -100) {
						// Overflow.
						error_temp_integral_lsb = -100;
					} else {
						error_temp_integral_lsb += error_temp_lsb;
					}
					pwm_set = error_temp_lsb * REGULATOR_P + REGULATOR_OFFSET;
					pwm_set += ((error_temp_integral_lsb * REGULATOR_I) / 16);
					pwm_set += ((REGULATOR_D * (error_temp_lsb - error_temp_last_lsb)) / 16);

					if (pwm_set < 0) {
						pwm_set = 0;
					}
					if (pwm_set > HEATER_PWM_MAX) {
						pwm_set = HEATER_PWM_MAX;
					}
				} //if (!is_pullup_on)
				else {
					pwm_set = pwm_set_last;
				}
				if (tip_diagnostics_invalid_flag) {
					if (pwm_set < HEATER_FB_PWM_MIN) {
						pwm_set = HEATER_FB_PWM_MIN;
					}
					tip_diagnostics_invalid_flag = false;
				}
				// --------- Change state if needed. ---------
				if (adc_get(ADC_VIN_SENSE_HEATER_OFF) > VIN_MAX_LSB) {
					current_state = STATE_HIGH_SUPPLY;
				} else if (adc_get(ADC_VIN_SENSE_HEATER_OFF) < VIN_MIN_LSB) {
					current_state = STATE_LOW_SUPPLY;
				} else if ((tip_state == TIP_HEATER_OPEN) || (tip_state == TIP_SENSOR_OPEN)) {
					current_state = STATE_ERROR_OPEN_LOAD;
				} else if ((tip_state == TIP_HEATER_OVERLOAD) || (tip_state == TIP_SENSOR_SHORT)) {
					current_state = STATE_ERROR_OVERLOAD;
				} else if (tip_state == TIP_DISCONNECTED) {
					current_state = STATE_DISCONNECTED;
				} else if (magnetometer > 10000) {
					current_state = STATE_STANDBY;
				} else if (error_temp_lsb > 30) {
					current_state = STATE_OK_LOW_TEMP;
				} else if (error_temp_lsb < -50) {
					current_state = STATE_OK_HIGH_TEMP;
				} else {
					current_state = STATE_OK;

				}
				break;
			case STATE_ERROR_LOW_TEMP:
				pwm_set = 0;
				break;
			case STATE_LOW_SUPPLY:
			case STATE_HIGH_SUPPLY:
				pwm_set = 0;
				// --------- Change state if needed. ---------
				if (adc_get(ADC_VIN_SENSE_HEATER_OFF) > VIN_MAX_LSB) {
					current_state = STATE_HIGH_SUPPLY;
				} else if (adc_get(ADC_VIN_SENSE_HEATER_OFF) < VIN_MIN_LSB) {
					current_state = STATE_LOW_SUPPLY;
				} else {
					current_state = STATE_OK;
				}
				break;
			case STATE_DISCONNECTED:
				pwm_set = 0;
				if (tip_diagnostics_invalid_flag) {
					pwm_set = HEATER_FB_PWM_MIN;
					tip_diagnostics_invalid_flag = false;
				}
				// --------- Change state if needed. ---------
				if (adc_get(ADC_VIN_SENSE_HEATER_OFF) > VIN_MAX_LSB) {
					current_state = STATE_HIGH_SUPPLY;
				} else if (adc_get(ADC_VIN_SENSE_HEATER_OFF) < VIN_MIN_LSB) {
					current_state = STATE_LOW_SUPPLY;
				} else if ((tip_state == TIP_HEATER_OPEN) || (tip_state == TIP_SENSOR_OPEN)) {
					current_state = STATE_ERROR_OPEN_LOAD;
				} else if ((tip_state == TIP_HEATER_OVERLOAD) || (tip_state == TIP_SENSOR_SHORT)) {
					current_state = STATE_ERROR_OVERLOAD;
				} else if (tip_state == TIP_DISCONNECTED) {
					current_state = STATE_DISCONNECTED;
				} else {
					current_state = STATE_OK;
				}
				break;
			case STATE_ERROR_OVERLOAD:
			case STATE_ERROR_OPEN_LOAD:
				pwm_set = 0;
				if (tip_diagnostics_invalid_flag) {
					pwm_set = HEATER_FB_PWM_MIN;
					tip_diagnostics_invalid_flag = false;
				}
				// --------- Change state if needed. ---------
				if (adc_get(ADC_VIN_SENSE_HEATER_OFF) > VIN_MAX_LSB) {
					current_state = STATE_HIGH_SUPPLY;
				} else if (adc_get(ADC_VIN_SENSE_HEATER_OFF) < VIN_MIN_LSB) {
					current_state = STATE_LOW_SUPPLY;
				} else if ((tip_state == TIP_HEATER_OPEN) || (tip_state == TIP_SENSOR_OPEN)) {
					current_state = STATE_ERROR_OPEN_LOAD;
				} else if ((tip_state == TIP_HEATER_OVERLOAD) || (tip_state == TIP_SENSOR_SHORT)) {
					current_state = STATE_ERROR_OVERLOAD;
				} else if (tip_state == TIP_DISCONNECTED) {
					current_state = STATE_DISCONNECTED;
				} else {
					current_state = STATE_OK;
				}
				break;
			case STATE_ERROR_HIGH_TEMP:
			case STATE_INVALID:
			default:
				pwm_set = 0;
				break;
			}
			debug_print_raw(&current_state, 1);
			pwm_set_last = pwm_set;
			heater_cmd((uint16_t) pwm_set);
			led_loop(current_state);
			heater_pwm_state = HEATER_PWM_STATE_ON_IDLE;
			break;
		case HEATER_PWM_STATE_ON_IDLE:
			// Heater is on, waiting for turn off.
			if (heater_pwm_falling_edge_flag) {
				heater_pwm_falling_edge_flag = 0;
				heater_delay_start(ADC_HEATER_OFF_DELAY);
				heater_pwm_state = HEATER_PWM_STATE_OFF_DELAY;
			}
			break;
		case HEATER_PWM_STATE_OFF_DELAY:
			// Heater is off, waiting for delay to update measurements.
			if (heater_delay_elapsed()) {
				heater_pwm_state = HEATER_PWM_STATE_OFF_BUSY;
			}
			break;
		case HEATER_PWM_STATE_OFF_BUSY:
			// Heater is off, updating measurements.
			adc_convert_while_heater_off();
			heater_pwm_state = HEATER_PWM_STATE_OFF_IDLE;
			break;
		case HEATER_PWM_STATE_OFF_IDLE:
			// Heater is off, waiting for turn on.
			if (heater_pwm_rising_edge_flag) {
				heater_pwm_rising_edge_flag = 0;
				heater_delay_start(ADC_HEATER_ON_DELAY);
				heater_pwm_state = HEATER_PWM_STATE_ON_DELAY;
			}
			break;
		default:
			// Invalid state.
			// TODO!
			break;
		}
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
void MX_ADC_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	HAL_ADC_Init(&hadc);

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&hi2c1);

	/**Configure Analogue filter
	 */
	HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

}

/* TIM1 init function */
void MX_TIM1_Init(void) {

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 4799;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 9;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE);

}

/* TIM3 init function */
void MX_TIM3_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim3);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

	HAL_TIM_MspPostInit(&htim3);

}

/* TIM14 init function */
void MX_TIM14_Init(void) {

	TIM_OC_InitTypeDef sConfigOC;

	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 47;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 9999;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim14);

	HAL_TIM_PWM_Init(&htim14);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_MspPostInit(&htim14);

}

/** 
 * Enable DMA controller clock
 */
void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 */
void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pins : PF0 PF1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : sensor_pullup_cmd_Pin */
	GPIO_InitStruct.Pin = sensor_pullup_cmd_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(sensor_pullup_cmd_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(sensor_pullup_cmd_GPIO_Port, sensor_pullup_cmd_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
