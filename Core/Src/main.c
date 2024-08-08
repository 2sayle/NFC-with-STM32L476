/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief         : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "macros.h"
#include "st25r3911b_hal_driver.h"
#include "st25r3911b_errors.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ST25R Global variables */
byte ST25R_IRQ_FLAG = 0;
irq_reason_t ST25R_IRQ_REASON = ST25R_NO_IRQ;
byte st25r_irq_buffer[3] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
static void ST25R_ExtractIRQReason(void);
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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Check the presence of the IC */
  if (ST25R_CheckIC() != ST25R_OK) {
    Error_Handler();
  }

  /* Initialize ST25R3911B */
  if (ST25R_PowerUpSequence() != ST25R_OK) {
    Error_Handler();
  }
  if (ST25R_SelectTypeA() != ST25R_OK) {
    Error_Handler();
  }

  /* Perform Anticoll A */
  if (ST25R_PerformAnticollA() != ST25R_OK) {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED4_Pin|LED5_Pin|SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_MCU_Pin */
  GPIO_InitStruct.Pin = IRQ_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_MCU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED6_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin LED5_Pin SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LED5_Pin|SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


static void ST25R_ExtractIRQReason(void) {

  /* Store content of interrupt register for better readability */
  byte mainIrqReg = st25r_irq_buffer[0];
  byte timerNfcIrqReg = st25r_irq_buffer[1];
  byte errorWupIrqReg = st25r_irq_buffer[2];

  if (BIT7_MASK(mainIrqReg)) {
    ST25R_IRQ_REASON = ST25R_OSC_FREQ_STABLE_IRQ;
  }
  else if (BIT6_MASK(mainIrqReg)) {
    ST25R_IRQ_REASON = ST25R_FIFO_WATER_LVL_IRQ;
  }
  else if (BIT5_MASK(mainIrqReg)) {
    ST25R_IRQ_REASON = ST25R_RX_START_IRQ;
  }
  else if (BIT4_MASK(mainIrqReg)) {
    ST25R_IRQ_REASON = ST25R_RX_STOP_IRQ;
  }
  else if (BIT3_MASK(mainIrqReg)) {
    ST25R_IRQ_REASON = ST25R_TX_STOP_IRQ;
  }
  else if (BIT2_MASK(mainIrqReg)) {
    ST25R_IRQ_REASON = ST25R_BIT_COLL_IRQ;
  }
  else if (BIT1_MASK(mainIrqReg)) {

    if (BIT7_MASK(timerNfcIrqReg)) {
      ST25R_IRQ_REASON = ST25R_DIRECT_CMD_STOP_IRQ;
    }
    else if (BIT6_MASK(timerNfcIrqReg)) {
      ST25R_IRQ_REASON = ST25R_NO_RESP_TIM_IRQ;
    }
    else if (BIT5_MASK(timerNfcIrqReg)) {
      ST25R_IRQ_REASON = ST25R_GP_TIM_IRQ;
    }
    else if (BIT4_MASK(timerNfcIrqReg)) {
      ST25R_IRQ_REASON = ST25R_EXT_FIELD_HIGH_IRQ;
    }
    else if (BIT3_MASK(timerNfcIrqReg)) {
      ST25R_IRQ_REASON = ST25R_EXT_FIELD_LOW_IRQ;
    }
    else if (BIT2_MASK(timerNfcIrqReg)) {
      ST25R_IRQ_REASON = ST25R_COLL_DURING_RF_COLL_AVOIDANCE_IRQ;
    }
    else if (BIT1_MASK(timerNfcIrqReg)) {
      ST25R_IRQ_REASON = ST25R_MIN_GUARD_TIME_VIOL_IRQ;
    }
    else if (BIT0_MASK(timerNfcIrqReg)) {
      ST25R_IRQ_REASON = ST25R_INITIATOR_RECV_IN_TARGET_MODE_IRQ;
    }

  }
  else if (BIT0_MASK(mainIrqReg)) {
    if (BIT7_MASK(errorWupIrqReg)) {
      ST25R_IRQ_REASON = ST25R_CRC_ERROR_IRQ;
    }
    else if (BIT6_MASK(errorWupIrqReg)) {
      ST25R_IRQ_REASON = ST25R_PARITY_ERROR_IRQ;
    }
    else if (BIT5_MASK(errorWupIrqReg)) {
      ST25R_IRQ_REASON = ST25R_SOFT_FRAMING_ERROR_IRQ;
    }
    else if (BIT4_MASK(errorWupIrqReg)) {
      ST25R_IRQ_REASON = ST25R_HARD_FRAMING_ERROR_IRQ;
    }
    else if (BIT3_MASK(errorWupIrqReg)) {
      ST25R_IRQ_REASON = ST25R_WUP_TIM_IRQ;
    }
    else if (BIT2_MASK(errorWupIrqReg)) {
      ST25R_IRQ_REASON = ST25R_WUP_TIM_AMP_MEAS_IRQ;
    }
    else if (BIT1_MASK(errorWupIrqReg)) {
      ST25R_IRQ_REASON = ST25R_WUP_TIM_PHASE_MEAS_IRQ;
    }
    else if (BIT0_MASK(errorWupIrqReg)) {
      ST25R_IRQ_REASON = ST25R_WUP_TIM_CAP_MEAS_IRQ;
    }
    
  }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == IRQ_MCU_Pin) {
    ST25R_IRQ_FLAG = 1;

    /* Read IRQ status registers (addresses: 0x17, 0x18, 0x19) */
    if (ST25R_ReadMultipleRegisters(ST25R_MAIN_INTERRUPT_REG, st25r_irq_buffer, 3) != ST25R_OK) {
      Error_Handler();
    }

    /* Extract IRQ reason */
    ST25R_ExtractIRQReason();
    
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

#ifdef  USE_FULL_ASSERT
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
