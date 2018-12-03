
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

RTC_HandleTypeDef hrtc;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t data_buf[3] = {0};
uint32_t adc_result = 0;
uint8_t DRDY_flag;
uint8_t check_tx = 0;
uint8_t check_rx = 0;
uint8_t	Config_Reg0 = 0x31;
uint8_t Config_Reg1 = 0x04; //06 - temperature
uint8_t Config_Reg2 = 0x98; //10
uint8_t Config_Reg3 = 0x00;

uint8_t	check_Config_Reg0 = 0;
uint8_t check_Config_Reg1 = 0;
uint8_t check_Config_Reg2 = 0;
uint8_t check_Config_Reg3 = 0;
uint8_t count=0;
uint8_t Flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void SPIx_Error (void);
static uint8_t SPIx_WriteRead(uint8_t Byte);
void ADS1220_Send_command (uint8_t command);
void ADS1220_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr);
void ADS1220_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
//	MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
	//HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET);
	BUZZER_OFF;
  /* USER CODE END 2 */
	
	ADS1220_Send_command(ADS1220_CMD_RESET);
	
	HAL_Delay(100);
	
	ADS1220_IO_Write(&Config_Reg0,ADS1220_0_REGISTER);
	ADS1220_IO_Write(&Config_Reg1,ADS1220_1_REGISTER);
	ADS1220_IO_Write(&Config_Reg2,ADS1220_2_REGISTER);
	ADS1220_IO_Write(&Config_Reg3,ADS1220_3_REGISTER);
	
	ADS1220_IO_Read(&check_Config_Reg0,ADS1220_0_REGISTER);
	ADS1220_IO_Read(&check_Config_Reg1,ADS1220_1_REGISTER);
	ADS1220_IO_Read(&check_Config_Reg2,ADS1220_2_REGISTER);
	ADS1220_IO_Read(&check_Config_Reg3,ADS1220_3_REGISTER);
	
	ADS1220_Send_command(ADS1220_CMD_SYNC);

	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*	count+=1;
if(count==0x8F)
{
	count=0;
	BUZZER_ON;
	HAL_Delay(500);
	BUZZER_OFF;
	
}	
	*/	
	if(!DRDY_flag)
	{
		
		
		
		for(int i=0; i<3; i++)
		{
			ADS1220_CS_LOW();
			
			data_buf[i] = SPIx_WriteRead(DUMMY_BYTE);
			
			ADS1220_CS_HIGH();
		}
		
		
		
		adc_result = data_buf[0];
		adc_result = (adc_result<<8)| data_buf[1];
		adc_result = (adc_result<<8)| data_buf[2];
		HAL_Delay(50);
		//printf("%X",adc_result);

		//DRDY_flag = 1;

	}
	
	if(adc_result>=SHORT)
	{
			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_SET);
	}
	
	if(adc_result!=0x00FFFFFF)
	{
		Flag=1;
	}
	if(mode==0)
	{
		if((adc_result>=(DRY-Delta))&&(adc_result<=(DRY+Delta)))
		{
			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_RESET);
			BUZZER_OFF;
			
		}
		if((adc_result>=(SPONGE-Delta))&&(adc_result<=(SPONGE+Delta)))
		{
			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_RESET);
			BUZZER_ON;
			HAL_Delay(500);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_SET);
			BUZZER_OFF;
			HAL_Delay(500);
			
		}
		if((adc_result>=(KOORTIK-Delta))&&(adc_result<=(KOORTIK+Delta)))
		{
			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_RESET);
			BUZZER_ON;
			HAL_Delay(300);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_SET);
			BUZZER_OFF;
			HAL_Delay(300);
			
		}
		if((adc_result>=(BLOOD-Delta))&&(adc_result<=(BLOOD+Delta)))
		{
			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_SET);
			BUZZER_ON;
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			BUZZER_OFF;
			HAL_Delay(100);
			
		}
		if((adc_result==0x00FFFFFF)&&(Flag==1))
		{
			//NVIC_SystemReset();
			
		}
	}
	else
	{
		if(adc_result>=(DRY-Delta))
		{
			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_RESET);
			BUZZER_OFF;
		}
		if((adc_result>=(SPONGE-Delta))&&(adc_result<=(SPONGE+Delta)))
		{
			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_RESET);
			BUZZER_OFF;
			HAL_Delay(500);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_SET);
			HAL_Delay(500);
			
		}
		if((adc_result>=(KOORTIK-Delta))&&(adc_result<=(KOORTIK+Delta)))
		{
			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_SET);
			BUZZER_ON;
			HAL_Delay(300);
			BUZZER_OFF;
			HAL_Delay(300);
			
		}
		if((adc_result>=(BLOOD-Delta))&&(adc_result<=(BLOOD+Delta)))
		{
			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, GPIO_PIN_SET);
			BUZZER_OFF;
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			
		}
		
		
	}

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{
	__HAL_RCC_SPI2_CLK_ENABLE();

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 10;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = TIMER_PERIOD;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = EXT_TIM_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim10);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
	HAL_GPIO_TogglePin(GPIOB, LED_R_Pin|LED_GR_Pin|LED_Y_Pin);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_GR_Pin|LED_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	/*SPI_NNS_PIN*/
	
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
			//DRDY_flag = 0;
	
}
void ADS1220_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr)
{
  /* Configure the MS bit: 
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */

  WriteAddr = (uint8_t)ADS1220_CMD_WREG|(WriteAddr<<2);
 
  /* Set chip select Low at the start of the transmission */
  ADS1220_CS_LOW();
  /* Send the Address of the indexed register */
  SPIx_WriteRead(WriteAddr);
  /* Send the data that will be written into the device (MSB First) */
  SPIx_WriteRead(*pBuffer);
  /* Set chip select High at the end of the transmission */ 
  ADS1220_CS_HIGH();
}

void ADS1220_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr)
{  
  
  ReadAddr = (uint8_t)ADS1220_CMD_RREG|(ReadAddr<<2);
 
  /* Set chip select Low at the start of the transmission */
	ADS1220_CS_LOW();
  
  /* Send the Address of the indexed register */
  SPIx_WriteRead(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  
    /* Send dummy byte (0x00) to generate the SPI clock to GYROSCOPE (Slave device) */
    *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
  
  /* Set chip select High at the end of the transmission */ 
  ADS1220_CS_LOW();
}

static uint8_t SPIx_WriteRead(uint8_t Byte)
{

  uint8_t receivedbyte = 0;
  
  /* Send a Byte through the SPI peripheral */
  /* Read byte from the SPI bus */
  if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, 0x1000) != HAL_OK)
  {
    SPIx_Error();
  }
  
  return receivedbyte;
}

static void SPIx_Error (void)
{
  /* De-initialize the SPI comunication BUS */
  HAL_SPI_DeInit(&hspi1);
  
  /* Re- Initiaize the SPI comunication BUS */
  HAL_SPI_Init(&hspi1);
}

void ADS1220_Send_command (uint8_t command)
{
	ADS1220_CS_LOW();
	
	SPIx_WriteRead(command);
	
	ADS1220_CS_HIGH();
	
	HAL_Delay(170);
	
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
