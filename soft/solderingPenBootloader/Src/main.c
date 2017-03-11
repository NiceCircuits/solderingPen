/**
 ******************************************************************************
 * @file    main.c
 * @author  piotr@nicecircuits.com
 * @date    2017-03-02
 * @mainpage Soldering pen bootloader
 *           This program supports loading new firmware for Nicecircuits Soldering Pen.
 *           More info available at:
 *           http://nicecircuits.com/solderingpen/
 *           http://nicecircuits.com/solderingpen-bootloader/
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32f0xx_hal.h"
#include "software_uart.h"
#include "led.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
CRC_HandleTypeDef hcrc;
typedef void (*pFunction)(void);
extern pFunction jump_to_application;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_CRC_Init(void);
/// Start application
HAL_StatusTypeDef run_application();

/* Private function bodies ---------------------------------------------------*/
HAL_StatusTypeDef run_application() {
  // Check if first byte (top of stack pointer) of user flash is OK
  if (((*(__IO uint32_t*) APP_START_ADDR) & 0x2FFE0000) == 0x20000000) {
    // Load reset vector from application Flash
    jump_to_application = (pFunction) (*(__IO uint32_t*) (APP_START_ADDR + 4));
    // Load stack pointer from application Flash
    __set_MSP(*(__IO uint32_t*) APP_START_ADDR);
    jump_to_application();
  } else {
    // Valid application not loaded
  }
  // Function shall never return
  return HAL_ERROR;
}

int main(void) {

  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_CRC_Init();
  led_init();
  software_uart_init();

  /// Bootloader data buffer
  uint8_t data[WRITE_BLOCK_SIZE + CRC_SIZE];
  /// Address to save data in Flash
  uint32_t address = APP_START_ADDR;
  /// Result of operations
  HAL_StatusTypeDef result;
  /// Error information for erase operation
  uint32_t page_err;
  /// Default counter
  uint32_t cnt;
  /// Calculated CRC value
  uint32_t crc;
  /// Status to return to UART
  uint8_t response;
  /// Pointer to data to be read
  uint8_t* read_p;
  /// Initialization data for GPIO
  GPIO_InitTypeDef GPIO_InitStruct;
  /// Value from ADC
  uint32_t adc;

  /// Initialization structure for Flash erase
  FLASH_EraseInitTypeDef erase_init = { .TypeErase = FLASH_TYPEERASE_PAGES, .PageAddress = APP_START_ADDR, .NbPages =
      (FLASH_BANK1_END + 1 - APP_START_ADDR) / FLASH_PAGE_SIZE };

  // Save init values to structure
  GPIO_InitStruct.Pin = sensor_pullup_cmd_Pin;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  /* Main loop-----------------------------------------------------------------*/
  while (1) {
    /* Check if bootloader cable is connected -----------------------------------*/
    // Config sensor_pullup_cmd_Pin as output and set it low
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, sensor_pullup_cmd_Pin, GPIO_PIN_RESET);
    // delay a bit and read ADC from sensor_sig_Pin
    HAL_Delay(10);
    HAL_ADC_Start(&hadc);
    adc = HAL_ADC_GetValue(&hadc);
    // If voltage on sensor_sig pin is above threshold, start bootloader
    if (adc >= SENSOR_BOOT_THRESHOLD) {
      // Configure sensor_pullup_cmd_Pin as input for UART RxD
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      result = software_uart_receive(data, 1);
      response = RESPONSE_ERROR; // for now
      if (result == HAL_OK) {
        /* Write flash --------------------------------------------------------------*/
        // check write command and if write possible
        if ((data[0] == COMMAND_WRITE) && (address <= (APP_INFO_ADDR - WRITE_BLOCK_SIZE))) {
          response = RESPONSE_OK;
          software_uart_send(&response, 1);
          // Get data block
          response = RESPONSE_ERROR;
          result = software_uart_receive(data, WRITE_BLOCK_SIZE + CRC_SIZE);
          // Check if transmission is OK and check CRC
          if (result == HAL_OK) {
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data, WRITE_BLOCK_SIZE);
            if (crc == *(uint32_t*) (data + WRITE_BLOCK_SIZE)) {
              HAL_FLASH_Unlock();
              for (cnt = 0; cnt < WRITE_BLOCK_SIZE; cnt += 4) {
                result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + cnt, *((uint32_t*) (data + cnt)));
              } // for (cnt = 0; cnt < WRITE_BLOCK_SIZE; cnt += 4)
              HAL_FLASH_Lock();
              if (result == HAL_OK) {
                address += WRITE_BLOCK_SIZE;
                response = RESPONSE_OK;
              } // if (status == HAL_OK)
            } // if (crc == *(uint32_t*) (data + WRITE_BLOCK_SIZE))
          } // if (status == HAL_OK)
        } // if ((data[0] == COMMAND_WRITE) && (address <= (APP_INFO_ADDR - WRITE_BLOCK_SIZE)))
        /* Erase flash --------------------------------------------------------------*/
        else if (data[0] == COMMAND_ERASE) {
          HAL_FLASH_Unlock();
          result = HAL_FLASHEx_Erase(&erase_init, &page_err);
          HAL_FLASH_Lock();
          if (result == HAL_OK) {
            response = RESPONSE_OK;
          } else {
            //response = RESPONSE_ERROR;
          }
        } // else if (data[0] == COMMAND_ERASE)
        /* Read application info block ----------------------------------------------*/
        else if (data[0] == COMMAND_READ_INFO) {
          // TODO read chip ID and type
          read_p = (uint8_t*) APP_INFO_ADDR;
          crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) read_p, APP_INFO_SIZE);
          software_uart_send(read_p, APP_INFO_SIZE);
          software_uart_send((uint8_t*) &crc, 4);
        } // else if (data[0] == COMMAND_READ_INFO)
        /* Read application info block ----------------------------------------------*/
        else if (data[0] == COMMAND_RUN_APP) {
          run_application();
          // if function returns, return error
        } // else if (data[0] == COMMAND_RUN_APP)
        /* Invalid command ----------------------------------------------------------*/
        else {
          // response = RESPONSE_ERROR;
        }
        software_uart_send(&response, 1);
      } // if (result == HAL_OK)
    } // if (adc>=SENSOR_BOOT_THRESHOLD)
    else { // not if (adc>=SENSOR_BOOT_THRESHOLD)
      /* Bootloader not detected, go to application -------------------------------*/
      run_application();
    } // else { // not if (adc>=SENSOR_BOOT_THRESHOLD)
  } // while (1)
}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  /**Configure the Systick interrupt time
   */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
   */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void) {

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
  if (HAL_ADC_Init(&hadc) != HAL_OK) {
    Error_Handler();
  }

  /**Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/* CRC init function */
static void MX_CRC_Init(void) {

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK) {
    Error_Handler();
  }

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
static void MX_GPIO_Init(void) {

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

  /*Configure GPIO pins : sensor_pullup_cmd_Pin driver_cmd_Pin */
  GPIO_InitStruct.Pin = sensor_pullup_cmd_Pin | driver_cmd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, sensor_pullup_cmd_Pin | driver_cmd_Pin, GPIO_PIN_RESET);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
  }
  /* USER CODE END Error_Handler */
}
