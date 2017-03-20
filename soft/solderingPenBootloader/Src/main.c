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
CRC_HandleTypeDef hcrc;
typedef void (*pFunction)(void);
pFunction jump_to_application;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_CRC_Init(void);
/**
 * Start uploaded application
 * @return HAL_ERROR if error detected, otherwise no return
 */
HAL_StatusTypeDef run_application();

/* Private function bodies ---------------------------------------------------*/
HAL_StatusTypeDef run_application() {
  // Check if first byte (top of stack pointer) of user flash is OK
  if (((*(__IO uint32_t*) APP_START_ADDR) & 0x2FFE0000) == 0x20000000) {
    // Deinitialize used peripherals
    led_deinit();
    software_uart_deinit();
    // HAL_ADC_DeInit(&hadc);
    // TODO: extracto to separate file
    /* Reset register ISR */
    ADC1->ISR = (ADC_FLAG_AWD | ADC_FLAG_OVR | ADC_FLAG_EOS | ADC_FLAG_EOC | ADC_FLAG_EOSMP | ADC_FLAG_RDY);
    /* Reset register CFGR1 */
    ADC1->CFGR1 = 0;
    /* Reset register CFGR2 */
    ADC1->CFGR2 = 0;
    /* Reset register SMPR */
    ADC1->SMPR = 0;
    /* Reset register CHSELR */
    ADC1->CHSELR = 0;
    /* Reset register CCR */
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_CRC_DeInit(&hcrc);
    HAL_DeInit();
    // Load reset vector from application Flash
    jump_to_application = (pFunction) (*(__IO uint32_t*) (APP_START_ADDR + 4));
    // Load stack pointer from application Flash
    __set_MSP(*(__IO uint32_t*) APP_START_ADDR);
    jump_to_application();
  } // if (((*(__IO uint32_t*) APP_START_ADDR) & 0x2FFE0000) == 0x20000000)
// Function shall never return
  return HAL_ERROR;
}

int main(void) {
// TODO: LED
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
/// Value from ADC
  uint32_t adc;

/// Initialization structure for Flash erase
  FLASH_EraseInitTypeDef erase_init = { .TypeErase = FLASH_TYPEERASE_PAGES, .PageAddress = APP_START_ADDR, .NbPages =
      (FLASH_BANK1_END + 1 - APP_START_ADDR) / FLASH_PAGE_SIZE };

  /* Main loop-----------------------------------------------------------------*/
  while (1) {
    led_cmd(2000, 0, 0);
    /* Check if bootloader cable is connected -----------------------------------*/
    sensor_pullup_cmd_GPIO_Port->MODER |= GPIO_MODER_MODER2_0; // Set sensor_pullup_cmd_Pin as output
    // delay a bit and read ADC from sensor_sig_Pin
    HAL_Delay(100);
    {
      /* Perform ADC enable and conversion start if no conversion is on going */
      if (((ADC1->CR) & ADC_CR_ADSTART) == 0) {

        /* Clear regular group conversion flag and overrun flag */
        ADC1->ISR = (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR); // Clear bits by writing one

        ADC1->CR |= ADC_CR_ADSTART;

      }
    }
    /* Wait until End of Conversion flag is raised */
    while (HAL_IS_BIT_CLR(ADC1->ISR, (ADC_FLAG_EOC | ADC_FLAG_EOS))) {
    }
    adc = ADC1->DR;
    // If voltage on sensor_sig pin is above threshold, start bootloader
    if (adc >= SENSOR_BOOT_THRESHOLD) {
      led_cmd(0, 0, 2000);
      // Configure sensor_pullup_cmd_Pin as input for UART RxD
      sensor_pullup_cmd_GPIO_Port->MODER &= (~GPIO_MODER_MODER2_0);
      while (1) {
        result = software_uart_receive(data, 1);
        if (result == HAL_OK) {
          response = RESPONSE_ERROR; // for now
          led_cmd(0, 0, 0);
          /* Write flash --------------------------------------------------------------*/
          // check write command and is write possible
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
              // Reset pointer for writing
              address = APP_START_ADDR;
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
            continue; // skip sending response
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
        else if (result == HAL_TIMEOUT) {
          break; // return to bootloader detection
        } // else if (result == HAL_TIMEOUT)
      } // while(1)
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

  __HAL_RCC_ADC1_CLK_ENABLE()
        ; /* Peripheral clock enable */
// TODO: merge all clock enables

  ADC1->CFGR1 = ADC_RESOLUTION_12B
      | ADC_DATAALIGN_RIGHT;

  ADC1->CFGR2 = ADC_CLOCK_ASYNC_DIV1; // (Asynchronous clock mode), generated at product level, no divider
  ADC1->SMPR = ADC_SAMPLETIME_71CYCLES_5;  // Channel sampling time configuration
  ADC1->CHSELR = ADC_CHSELR_CHSEL3; // Select active channel

  /* Enable the ADC peripheral */
  ADC1->CR |= ADC_CR_ADEN;

  /* Wait for ADC effectively enabled */
  while (((ADC1->ISR) & ADC_FLAG_RDY) == 0)
  {
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
// GPIO Ports Clock Enable: GPIOA,B,F
  __IO uint32_t tmpreg;
  RCC->AHBENR |= RCC_AHBENR_GPIOFEN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
// Delay after an RCC peripheral clock enabling
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIOFEN);
  UNUSED(tmpreg);

// Configure GPIOA port
  GPIOA->MODER = 0xFFFFFFFF
      // All pins in Analog input mode, except:
      & (~GPIO_MODER_MODER2_1) // sensor_pullup_cmd_Pin - output
      & (~GPIO_MODER_MODER4_1) // driver_cmd_Pin - output
      & (~GPIO_MODER_MODER6_0) // led_R_cmd_Pin - alternate function
      & (~GPIO_MODER_MODER7_0) // led_G_cmd_Pin - alternate function
      & (~GPIO_MODER_MODER13_0) // SWDIO - alternate function
      & (~GPIO_MODER_MODER14_0); // SWDCLK - alternate function
#if 0
      GPIOA->OTYPER=0; // All outputs open drain
      GPIOA->OSPEEDR=0;// All outputs low speed
      GPIOA->PUPDR=0;
      GPIOA->ODR=0;
#endif
  GPIOA->AFR[0] =
      (GPIO_AF1_TIM3 << (4 * 6)) // led_R_cmd_Pin (6) - alternate function: TIM3
      | (GPIO_AF1_TIM3 << (4 * 7));  // led_G_cmd_Pin (7) - alternate function: TIM3

// Configure GPIOB port
  GPIOB->MODER = 0xFFFFFFFF
      // All pins in Analog input mode, except:
      & (~GPIO_MODER_MODER1_0); // led_B_cmd_Pin - alternate function
  GPIOB->AFR[0] = (GPIO_AF1_TIM3 << (4 * 1)); // led_B_cmd_Pin (1) - alternate function: TIM3

// Configure GPIOF port
  GPIOF->MODER = 0xFFFFFFFF; // All pins in Analog input mode
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
