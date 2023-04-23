/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <mcp2515.h>
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_rtc.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
// #include "rm3100_i2c.h"
#include "rm3100_spi.h"
#include "tmp100_i2c.h"
#include "i2c_detect.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/*
 *  Variáveis do tipo Handle utilizadas nas bibliotecas dos sensores utilizados
 *  Devem obrigatoriamente ser declaradas em algum lugar para que as bibliotecas
 *  possam ser utilizadas
 */
SPI_HandleTypeDef *spi_handle;
I2C_HandleTypeDef *i2c_handle;
RTC_HandleTypeDef hrtc;
// UART_HandleTypeDef *uart_handle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN_Init(void);
static void MX_AT512C_Init(void);
void SystemClock_ConfigLP(void);
void sendPackets(void);
void MX_LPTIM1_Init(void);
void SysTick_Init(uint32_t ticks);
void enter_LPSleep( void );
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_GPIO_Init();
  //  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN 2 */
  //MX_AT512C_Init();
  MX_CAN_Init();

  //MX_LPTIM1_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    TMP100_DATA temp = TMP100_I2C_DATA(MUL_12_bit);
    HAL_Delay(1000);
    RM3100_DATA mag_data = RM3100_SPI_DATA();
    HAL_Delay(1000);
    write_data_to_eeprom(&temp, &mag_data);
    HAL_Delay(1000);

    sendPackets();


//    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
//    enter_LPSleep();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void sendPackets() {
    TMP100_DATA temp_data;
    RM3100_DATA mag_data;
    uint16_t pkg_count = obtainPkgCount();
    for (uint16_t cnt = 0; cnt < pkg_count; cnt++) {
        enum ERROR status = read_data_from_eeprom(&temp_data, &mag_data);

        // Serialize temperature data
        uint8_t temp_data_serialized[8];
        memcpy(temp_data_serialized, &temp_data.temp, sizeof(temp_data.temp));
        memcpy(temp_data_serialized + sizeof(temp_data.temp), &temp_data.status, sizeof(temp_data.status));

        // Serialize magnetometer data
        uint8_t mag_data_serialized[24];
        memcpy(mag_data_serialized, &mag_data.x, sizeof(mag_data.x));
        memcpy(mag_data_serialized + sizeof(mag_data.x), &mag_data.y, sizeof(mag_data.y));
        memcpy(mag_data_serialized + 2 * sizeof(mag_data.x), &mag_data.z, sizeof(mag_data.z));
        memcpy(mag_data_serialized + 3 * sizeof(mag_data.x), &mag_data.gain, sizeof(mag_data.gain));
        memcpy(mag_data_serialized + 3 * sizeof(mag_data.x) + sizeof(mag_data.gain), &mag_data.uT, sizeof(mag_data.uT));

        // Create CAN frames and send them
        struct can_frame frames[3];
        int frame_index = 0;

        frames[frame_index].can_id = 0x123;
        frames[frame_index].can_dlc = 8;
        memcpy(frames[frame_index].data, temp_data_serialized, sizeof(temp_data_serialized));
        frame_index++;

        frames[frame_index].can_id = 0x456;
        frames[frame_index].can_dlc = 8;
        memcpy(frames[frame_index].data, mag_data_serialized, 8);
        frame_index++;

        frames[frame_index].can_id = 0x789 + 1;
        frames[frame_index].can_dlc = 8;
        memcpy(frames[frame_index].data, mag_data_serialized + 8, sizeof(mag_data_serialized) - 8);

        for (int i = 0; i < 3; i++) {
            enum ERROR retorno = sendMessage1(&frames[i]);
            HAL_Delay(10);
        }
    }
}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* Atribuição de hi2c1 como handle spi */
  i2c_handle = &hi2c1;
  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  /* Atribui a variável para o spi do RM3100 */
  spi_handle = &hspi2;
  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_Delay(100);
  RM3100_SPI_SETUP(&GPIO_InitStruct);
  HAL_Delay(100);
  TMP100_I2C_SETUP(RESOLUTION_12_BIT);
  HAL_Delay(100);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Config_SysClk_MSI_131( void )
{
    // Enable the MSI
    RCC->CR |= RCC_CR_MSION;
    while( (RCC->CR & RCC_CR_MSIRDY) == 0 ); // Wait until MSI is ready

    // Select MSI as the system clock source
    RCC->CFGR &= ~( RCC_CFGR_SW );
    RCC->CFGR |= RCC_CFGR_SW_MSI;

    // Wait until the system clock source is MSI
    while( (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI );

    // Set MSI to 131.072 kHz
    RCC->ICSCR &= ~( RCC_ICSCR_MSIRANGE );
    RCC->ICSCR |= RCC_ICSCR_MSIRANGE_3; // Set MSI range to 7 (131.072 kHz)
}

void MX_LPTIM1_Init(void)
{
  /* Enable the LSI clock */
  RCC->CSR |= RCC_CSR_LSION;
  while((RCC->CSR & RCC_CSR_LSIRDY) == 0);

  /* Enable the LPTIM1 clock */
  RCC->APB1ENR |= RCC_APB1ENR_LPTIM1EN;

  /* Reset LPTIM1 peripheral */
  RCC->APB1RSTR |= RCC_APB1RSTR_LPTIM1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_LPTIM1RST;

  /* Configure LPTIM1 in interrupt mode */
  LPTIM1->CR = 0;
  LPTIM1->CFGR |= LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_1;  // Set prescaler to divide by 16
  LPTIM1->CFGR |= LPTIM_CFGR_WAVE;                          // Set waveform to toggle on compare match
  LPTIM1->CMP = 0xFFFF;                                     // Set compare value to maximum
  LPTIM1->ARR = 1953;                                       // Set auto-reload value to generate interrupt every 30 seconds
  LPTIM1->CR |= LPTIM_CR_ENABLE;                            // Enable LPTIM1
  LPTIM1->IER |= LPTIM_IER_ARRMIE;                          // Enable interrupt on auto-reload match

  /* Enable the LPTIM1 interrupt */
  NVIC_EnableIRQ(LPTIM1_IRQn);
}

void LPTIM1_IRQHandler(void)
{
  /* Clear the LPTIM1 auto-reload match flag */
  LPTIM1->ISR &= ~LPTIM_ISR_ARRM;

  /* Disable LPTIM1 */
  LPTIM1->CR &= ~LPTIM_CR_ENABLE;

  /* Disable the LPTIM1 interrupt */
  NVIC_DisableIRQ(LPTIM1_IRQn);

  /* Exit sleep mode */
  HAL_PWR_DisableSleepOnExit();
}

//void SystemClock_ConfigLP(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

void enter_LPSleep( void )
{
    /* 1. The Flash memory can be switched off by using the control bits
              (SLEEP_PD in the FLASH_ACR register). This reduces power consumption
              but increases the wake-up time. */
    FLASH->ACR |= FLASH_ACR_SLEEP_PD;
    /* 2. Each digital IP clock must be enabled or disabled by using the
                RCC_APBxENR and RCC_AHBENR registers */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    /* 3. The frequency of the system clock must be decreased to not exceed the
                frequency of f_MSI range1. */
    // Set MSI 131.072 kHz as system clock
    Config_SysClk_MSI_131();
    /* 4. The regulator is forced in low-power mode by software
                (LPSDSR bits set ) */
    PWR->CR |= PWR_CR_LPSDSR; // voltage regulator in low-power mode during sleep
    /* 5. Follow the steps described in Section 6.3.5: Entering low-power mode */
    SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk ); // low-power mode = sleep mode
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; // reenter low-power mode after ISR
    __WFI(); // enter low-power mode
}


static void MX_CAN_Init(void)
{
	reset();
	setBitrate1(CAN_5KBPS);
	setNormalMode();
}

static void MX_AT512C_Init(void)
{
	bool writeStatus = false;
	bool readStatus = false;
	bool eraseStatus = false;
	#define MEM_ADDR    0x00u
	uint8_t  wData[] = "Hello World 123";
	uint8_t  rData[25];

	if(at24_isConnected()){
		//eraseStatus = at24_eraseChip();
		HAL_Delay(10);
		writeStatus = at24_write(MEM_ADDR,wData, 15, 100);
		HAL_Delay(10);
		readStatus = at24_read(MEM_ADDR,rData, 15, 100);
		HAL_Delay(10);
	}


}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
