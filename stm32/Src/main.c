/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim14);
  __HAL_I2C_ENABLE_IT(&hi2c1, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI);
  SET_BIT(USART1->CR1, USART_CR1_RXNEIE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Wait for interrupts; nothing else to do.
    __asm__("wfi");
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 234;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 6000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 7;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

static const int COMMAND_TIMEOUT = 10;  // number of 10ms intervals to expect a command
static volatile uint8_t command_timeout_ = 0;

volatile union {
  struct {
    uint8_t leds;
    int8_t esc_pwm;
    int8_t servo_pwm;
    uint16_t motor_ticks;
    uint16_t motor_period;
    uint32_t motor_last_clock;
  } __attribute__((packed));
  uint8_t bytes[16];  // must be power of 2
} i2cdata_ = {.bytes = {0}}, i2cshadow_;

// TIM14: 65536us timer interrupt
static volatile uint16_t tim14_hi = 0;
void TIM14_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&htim14, TIM_FLAG_UPDATE) != RESET) {
    __HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
    if ((i2cdata_.motor_last_clock >> 16) == tim14_hi-1) {
      // no motor activity between 65..131ms ago; consider motor stopped
      i2cdata_.motor_period = 0;
    }
    tim14_hi++;
  }
}

// TIM14 is currently configured to tick every 1us but we may want more resolution
static uint32_t get_tim14_ticks() {
  return (((uint32_t)tim14_hi) << 16) + TIM14->CNT;
}

static uint16_t pwmtiming(int8_t pwm8) {
  return 6000 + (2000*pwm8 >> 7);
}

static void update_from_i2cdata() {
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmtiming(i2cdata_.servo_pwm));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmtiming(i2cdata_.esc_pwm));
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, i2cdata_.leds & 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (i2cdata_.leds & 2) >> 1);
  command_timeout_ = COMMAND_TIMEOUT;
}

// If we don't receive a command from the host, reset our outputs to zero so we
// don't run away
static void reset_commands() {
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmtiming(0));
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmtiming(0));
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
}

static void update_phase() {
  // We only really care about the speed of the motor itself, so just track
  // that instead of individual phase windings.

  uint32_t t0 = i2cdata_.motor_last_clock;
  uint32_t t1 = get_tim14_ticks();
  int32_t p = t1 - t0;

  // smooth out period readings with an adaptive filter
  // low pass, ~15Hz bandwidth
  // the filter coefficient depends on period, as we aren't updating on a fixed
  // interval but rather on period itself.

  if (p < 32768) {
    // this can overflow if |lastp - p| >= 32768 but i'm not going
    // to worry about it
    int32_t lastp = i2cdata_.motor_period;
    p += ((p*(lastp - p)) >> 16);
  }
  if (p < 65535 && p > 0) {
    i2cdata_.motor_period = p;
  } else {
    i2cdata_.motor_period = 0;
  }

  i2cdata_.motor_last_clock = t1;
  i2cdata_.motor_ticks++;
}

void EXTI0_1_IRQHandler(void) {
  int update = 0;
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    update = 1;
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
    update = 1;
  }
  if (update) {
    update_phase();
  }
}

void EXTI4_15_IRQHandler(void) {
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
    update_phase();
  }
}

// I2C slave

void I2C1_IRQHandler(void) {
  // The HAL I2C slave interrupt handler stuff is all garbage; implement our own here.
  static uint8_t reg_ptr = 0;
  static uint8_t firstbyte = 0;

  uint32_t itflags = READ_REG(I2C1->ISR);
  if (itflags & I2C_FLAG_ADDR) {  // address matched; start a transfer
    __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_ADDR);
    if (I2C_GET_DIR(&hi2c1) == I2C_DIRECTION_TRANSMIT) {
      // prepare to receive bytes from master; first byte is register number
      firstbyte = 1;
    }
  }
  if (itflags & I2C_FLAG_TXE) {  // transmit register empty; send next byte
    reg_ptr = (reg_ptr+1) & (sizeof(i2cdata_) - 1);
    I2C1->TXDR = i2cshadow_.bytes[reg_ptr];
  }
  if (itflags & I2C_FLAG_RXNE) {  // receive register not empty
    uint8_t databyte = I2C1->RXDR & 0xff;
    if (firstbyte) {
      reg_ptr = databyte;
      firstbyte = 0;
      reg_ptr &= sizeof(i2cdata_) - 1;
      // flush TX buffer and reload with new data byte

      // just to ensure we don't spuriously re-trigger the interrupt we're
      // already in; I am not sure whether this is necessary:
      __disable_irq();
      i2cshadow_ = i2cdata_;  // make a shadow copy of the data for atomic reading
      I2C1->ISR |= I2C_FLAG_TXE;  // setting TXE flushes TXDR
      I2C1->TXDR = i2cshadow_.bytes[reg_ptr];
      __enable_irq();
    } else {
      reg_ptr &= sizeof(i2cdata_) - 1;
      i2cdata_.bytes[reg_ptr++] = databyte;
      update_from_i2cdata();
    }
  }
}

static uint8_t usart_txbuf[5];
static uint8_t usart_txptr = 0;
static uint8_t usart_rxbuf[4];
static int8_t usart_rxptr = -1;

void TIM3_IRQHandler(void) {
  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) {
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    usart_txbuf[4] = 0xaa;
    for (uint8_t i = 0; i < 4; i++) {
      // send motor ticks and period (4 bytes, starting at 3rd i2c register)
      usart_txbuf[i] = i2cdata_.bytes[3+i];
      usart_txbuf[4] += usart_txbuf[i];
    }
    usart_txbuf[4] = ~usart_txbuf[4];
    USART1->TDR = 0xaa;
    SET_BIT(USART1->CR1, USART_CR1_TXEIE);
    usart_txptr = 0;

    if (command_timeout_ > 0) {
      command_timeout_--;
      if (command_timeout_ == 0) {
        reset_commands();
      }
    }
  }
}

void process_serial_msg(const uint8_t *buf) {
  uint8_t cksum = 0x55;
  for (uint8_t i = 0; i < 4; i++) {
    cksum += buf[i];
  }
  if (cksum != 0xff) {
    // invalid message!
    if (usart_txptr == 5) {  // don't respond unless we're not transmitting anything
      USART1->TDR = 0xfe;  // respond with a made-up NAK scheme
    }
  } else {
    for (uint8_t i = 0; i < 3; i++) {
      i2cdata_.bytes[i] = buf[i];
    }
    update_from_i2cdata();
  }
}

void USART1_IRQHandler(void) {
  uint32_t itflags = READ_REG(USART1->ISR);
  if (itflags & USART_ISR_TXE) {
    if (usart_txptr < 5) {
      USART1->TDR = usart_txbuf[usart_txptr++];
    } else {
      CLEAR_BIT(USART1->CR1, USART_CR1_TXEIE);
    }
  }
  if (itflags & USART_ISR_RXNE) {
    uint8_t chr = READ_REG(USART1->RDR);
    if (usart_rxptr >= sizeof(usart_rxbuf)) {
      usart_rxptr = -1;
    }
    if (usart_rxptr < 0 && chr == 0x55) {
      usart_rxptr = 0;
    } else if (usart_rxptr >= 0) {
      usart_rxbuf[usart_rxptr++] = chr;
      if (usart_rxptr == 4) {
        process_serial_msg(usart_rxbuf);
        usart_rxptr = -1;
      }
    }
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
