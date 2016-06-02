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
// Includes for Eclipse indexer
#include "stm32f070x6.h"
#include "stm32f0xx_hal_rcc.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/// State of heater PWM state machine
typedef enum {
	/// Invalid state.
	heaterPwmStateInvalid,
	/// Heater is on, waiting for delay to update measurements.
	heaterPwmStateOnDelay,
	/// Heater is on, updating measurements.
	heaterPwmStateOnBusy,
	/// Heater is on, waiting for turn off.
	heaterPwmStateOnIdle,
	/// Heater is off, waiting for delay to update measurements.
	heaterPwmStateOffDelay,
	/// Heater is off, updating measurements.
	heaterPwmStateOffBusy,
	/// Heater is off, waiting for turn on.
	heaterPwmStateOffIdle
} heaterPwmState_t;

heaterPwmState_t heaterPwmState = heaterPwmStateInvalid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	/// Temperature set
	uint16_t temperatureSetLsb = 0;
	/// Error of temperature set
	int32_t errorTemp = 0;
	/// Previous value of error of temperature set
	int32_t errorTempPrev = 0;
	/// Integral of error of temperature set over time
	int32_t errorTempIntegral = 0;
	/// Value of PWM to be set
	int32_t pwmSet = 0;
	/// Previous value of set PWM
	int32_t lastPwmSet = 0;
	/// Limits for open load and overload of heater driver
	int32_t heaterOpenLoadLimit, heaterOveloadLimit;
	/// Counter for time without proper current feedback diagnosis
	uint_fast16_t heaterNoFbCounter = 0;
	/// Proportional gain of regulator - from sensor to heater PWM.
	volatile int32_t REGULATOR_P = 1024;
	/// Integral gain of regulator - from sensor to heater PWM.
	volatile int32_t REGULATOR_I = 0;
	/// Derivative gain of regulator - from sensor to heater PWM.
	volatile int32_t REGULATOR_D = 0;
	/// Current state of device.
	static state_t currentState;

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
	MX_USART2_UART_Init();
	MX_TIM1_Init();

	/* USER CODE BEGIN 2 */
	adcInit();
	heaterStartPwm();
	ledStartPwm();
	heaterPwmState = heaterPwmStateOffIdle;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// ==================== PWM phase state machine =====================
//		debugPrint("{s,T,%u}", heaterPwmState * 100);
		switch (heaterPwmState) {
		case heaterPwmStateOnDelay:
			// Heater is on, waiting for delay to update measurements.
			if (heaterDelayElapsed()) {
				heaterPwmState = heaterPwmStateOnBusy;
			}
			break;
		case heaterPwmStateOnBusy:
			// Heater is on, updating measurements.
//			debugPrint("{x,T,%u}", x / 10);
//			debugPrint("{s,T,%u}", adcGet(adcSensor));
			//debugPrint("\r\n");
			adcConvertWhileHeaterOn();
			// ==================== Device state machine =====================
			switch (currentState) {
			case STATE_OK_LOW_TEMP:
			case STATE_OK:
			case STATE_OK_HIGH_TEMP:
			case STATE_STANDBY:
				// ========== Heater regulator control. ===========
				// Calculate desired temperature.
				temperatureSetLsb = (uint16_t) ((((uint32_t) adcGet(adcPotentiometer) * POT_SENS_NUMERATOR)
						>> POT_SENS_DENOMINATOR_BIT_SHIFT) + POT_SENS_ADDEND);
//			debugPrint("{set,T,%u}", temperatureSetLsb);
				// Save previous value of error.
				errorTempPrev = errorTemp;
				// Calculate actual value of temperature error.
				errorTemp = (int32_t) temperatureSetLsb - (int32_t) adcGet(adcSensor);
				// Calculate integral of temperature error.
				if ((errorTempIntegral + errorTemp) > 100) {
					// Overflow.
					errorTempIntegral = 100;
				} else if ((errorTempIntegral + errorTemp) < -100) {
					// Overflow.
					errorTempIntegral = -100;
				} else {
					errorTempIntegral += errorTemp;
				}
//			debugPrint("{e,T,%d}", errorTemp - 100);
//			debugPrint("{int,T,%d}", errorTempIntegral - 100);
				pwmSet = errorTemp * REGULATOR_P + REGULATOR_OFFSET;
				pwmSet += ((errorTempIntegral * REGULATOR_I) / 16);
				pwmSet += ((REGULATOR_D * (errorTemp - errorTempPrev)) / 16);

				if (pwmSet < 0) {
					pwmSet = 0;
				}
				if (pwmSet > HEATER_PWM_MAX) {
					pwmSet = HEATER_PWM_MAX;
				}
				// --------- heater diagnostics ---------
				// Check if last PWM duty was long enough for correct feedback reading
				if (lastPwmSet >= HEATER_FB_PWM_MIN) {
					heaterOpenLoadLimit = (((int32_t) adcGet(adcVinSenseHeaterOff) * HEATER_OPEN_LOAD_COEF_A)
							>> HEATER_COEF_BIT_SHIFT) + HEATER_OPEN_LOAD_COEF_B;
					heaterOveloadLimit = (((int32_t) adcGet(adcVinSenseHeaterOff) * HEATER_OVERLOAD_COEF_A)
							>> HEATER_COEF_BIT_SHIFT) + HEATER_OVERLOAD_COEF_B;
					heaterNoFbCounter = 0; // Reset no feedback counter.
					debugPrint("{l,T,%u}", heaterOpenLoadLimit);
					debugPrint("{fb,T,%u}", adcGet(adcDriverFb));
					debugPrint("{h,T,%u}", heaterOveloadLimit);
					debugPrint("{p,T,%u}", lastPwmSet / 64);
				} else {
					heaterNoFbCounter++;
					if (heaterNoFbCounter >= HEATER_MAX_TIME_NO_FB) {
						heaterNoFbCounter = 0;
						// Increase PWM duty for one next cycle to gain correct current sense reading.
						if (pwmSet < HEATER_FB_PWM_MIN) {
							pwmSet = HEATER_FB_PWM_MIN;
						}
					}
				}
				if (errorTemp > 30) {
					currentState = STATE_OK_LOW_TEMP;
				} else if (errorTemp < -50) {
					currentState = STATE_OK_HIGH_TEMP;
				} else {
					currentState = STATE_OK;
				}
				break;
			case STATE_ERROR_OVERLOAD:
			case STATE_ERROR_OPEN_LOAD:
			case STATE_ERROR_HIGH_TEMP:
				break;
			case STATE_ERROR_LOW_TEMP:
				break;
			case STATE_LOW_SUPPLY:
			case STATE_HIGH_SUPPLY:
				break;
			}
			heaterCmd((uint16_t) pwmSet);
			ledLoop(currentState);
			// Store pwmSet value for next loop run
			lastPwmSet = pwmSet;
			heaterPwmState = heaterPwmStateOnIdle;
			break;
		case heaterPwmStateOnIdle:
			// Heater is on, waiting for turn off.
			if (heaterPwmFallingEdgeFlag) {
				heaterPwmFallingEdgeFlag = 0;
				heaterDelayStart(ADC_HEATER_OFF_DELAY);
				heaterPwmState = heaterPwmStateOffDelay;
			}
			break;
		case heaterPwmStateOffDelay:
			// Heater is off, waiting for delay to update measurements.
			if (heaterDelayElapsed()) {
				heaterPwmState = heaterPwmStateOffBusy;
			}
			break;
		case heaterPwmStateOffBusy:
			// Heater is off, updating measurements.
			adcConvertWhileHeaterOff();
			heaterPwmState = heaterPwmStateOffIdle;
			break;
		case heaterPwmStateOffIdle:
			// Heater is off, waiting for turn on.
			if (heaterPwmRisingEdgeFlag) {
				heaterPwmRisingEdgeFlag = 0;
				heaterDelayStart(ADC_HEATER_ON_DELAY);
				heaterPwmState = heaterPwmStateOnDelay;
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
	htim14.Init.Prescaler = 47; //479
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

/* USART2 init function */
void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 500000;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&huart2);

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
	/* DMA1_Channel4_5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
