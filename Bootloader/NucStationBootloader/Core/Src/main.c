/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EEPROMHelper.h"
//#include "GameFlasher.h"


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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void LoadFirmware (void);
bool CheckArrayIsEnd(uint8_t *_buffer, uint8_t *compareTo, uint8_t *lastThree);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t dataWrite[16];

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  const unsigned char nucStationLogo [] = {
		  	0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xff, 0x82, 0x03, 0xff, 0xff, 0xff, 0xfc, 0x07, 0x87,
			0xff, 0xff, 0xff, 0xf8, 0x0f, 0xcf, 0xff, 0xff, 0xff, 0xe0, 0x0f, 0xcf, 0xef, 0xff, 0xff, 0xc0,
			0x0f, 0xcf, 0xe7, 0xff, 0xff, 0x80, 0x0f, 0x8f, 0xe3, 0xff, 0xff, 0x80, 0x0c, 0x0c, 0x63, 0xff,
			0xff, 0x00, 0x00, 0x00, 0x11, 0xff, 0xfe, 0x00, 0x41, 0xe0, 0x0f, 0xff, 0xfe, 0x07, 0xc7, 0xc7,
			0x8f, 0xff, 0xfc, 0x0f, 0xcf, 0xc7, 0xcf, 0xff, 0xfc, 0x0f, 0xcf, 0xcf, 0xcf, 0xff, 0xf8, 0x0f,
			0xcf, 0xcf, 0xcf, 0xff, 0xf8, 0x0f, 0xc7, 0x8f, 0xcf, 0xff, 0xf8, 0x03, 0xc3, 0x0f, 0x8f, 0xbf,
			0xf8, 0x00, 0x00, 0x10, 0x00, 0x3f, 0xf8, 0x00, 0x10, 0x20, 0x20, 0x3f, 0xf8, 0x00, 0x0f, 0xc3,
			0xe0, 0x3f, 0xf8, 0x00, 0x0f, 0xc7, 0xe0, 0x3f, 0xf8, 0x00, 0x0f, 0xcf, 0xe0, 0x3f, 0xf8, 0x00,
			0x0f, 0xcf, 0xe0, 0x3f, 0xf8, 0x00, 0x0f, 0xc7, 0xc0, 0x3f, 0xf8, 0x00, 0x07, 0x83, 0x80, 0x3f,
			0xf8, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x2e, 0x20, 0x00, 0x00, 0x3f, 0xfc, 0x44, 0xa0, 0x00,
			0x00, 0x7f, 0xfc, 0x24, 0xa4, 0x00, 0x00, 0x7f, 0xfe, 0x54, 0x60, 0x00, 0x00, 0xff, 0xfe, 0x00,
			0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x80, 0xa0, 0x00, 0x03, 0xff,
			0xff, 0xc0, 0xe1, 0x9a, 0x03, 0xff, 0xff, 0xe0, 0xa4, 0x49, 0x0f, 0xff, 0xff, 0xf0, 0xa4, 0xc1,
			0x1f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
			0xc0, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			0xdd, 0xff, 0xf1, 0xbf, 0xb7, 0xff, 0xcd, 0xff, 0xee, 0xbf, 0xbf, 0xff, 0xcd, 0xb4, 0x6e, 0x11,
			0x14, 0x43, 0xd5, 0xb3, 0x63, 0xad, 0xb3, 0x9d, 0xd5, 0xb3, 0xfc, 0xbd, 0xb3, 0x9d, 0xd9, 0xb3,
			0xee, 0xa1, 0xb3, 0x9d, 0xd9, 0xb3, 0x6e, 0xad, 0xb3, 0x9d, 0xdd, 0x84, 0xf1, 0x80, 0x94, 0x5d
  };


  printf("Starting NucStation\r\n");
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // to show we are in bootloader mode

  LCD_setRST(GPIOB, GPIO_PIN_13);
  LCD_setCE(GPIOB, GPIO_PIN_14);
  LCD_setDC(GPIOB, GPIO_PIN_15);
  LCD_setDIN(GPIOB, GPIO_PIN_1);
  LCD_setCLK(GPIOB, GPIO_PIN_2);
  LCD_init();

  LCD_draw(nucStationLogo, 18, 0, 48, 48);
  LCD_refreshScr();


  HAL_Delay(100);

  bool foundI2CFlag = false;
  uint8_t ret;

  printf("Scanning \r\n");
  for(uint8_t i = 1; i < 128; i++)
  {
      ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
      if (ret != HAL_OK) /* No ACK Received At That Address */
      {

    	 // printf("Device not found in 0x%X", i);
      }
      else if(ret == HAL_OK)
      {
          printf("Device found in 0x%X  \r\n", i);
          foundI2CFlag = true;
      }
  }
  printf("Scanning fin \r\n");

  if(foundI2CFlag)
  {
	  LCD_print("L", 0, 0);

	  char percentLoad[5] = {0};

	  uint8_t buffer[64] = {0};
	  uint8_t lastThree[3]  = {0, 0, 0};
	  uint8_t endCheckArray[3] = {'E', 'G', 'E'};

	  uint8_t wordCounter = 0;
	  uint8_t lastFour[4] = {0};
	  uint16_t sector = 0;
	  uint32_t lastFourWord[1] = {0};

	  uint8_t j = 0;

	  for(int i = 0; i < PAGE_NUM; i++)
	  {
		  EEPROM_Read(i, 0, buffer, 64);

		  for(j = 0; j < 64; j++)
		  {
			  lastFour[wordCounter] = buffer[j];
			  wordCounter++;
			  if(wordCounter == 4)
			  {
				  //FLASH
				  lastFourWord[0] = lastFour[0] | (lastFour[1] << 8) | (lastFour[2] << 16) | (lastFour[3] << 24);
				  wordCounter = 0;

				  if(sector == 0) FlashTheGame(lastFourWord, sector, true);
				  else FlashTheGame(lastFourWord, sector, false);

				  sector++;
				  itoa((int) i, percentLoad, 10);
				  LCD_print(percentLoad, 0, 0);
			  }
		  }


		  if(CheckArrayIsEnd(buffer, endCheckArray, lastThree))
		  {
			  printf("End on page %d \r\n", i);
			  break;
		  }
	  }
  }
  else
  {
	  printf("Reloading \r\n");
	  LCD_print("R", 0, 0);
	  HAL_Delay(500);
  }

  LoadFirmware();
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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

  /* USER CODE END I2C1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


}

/* USER CODE BEGIN 4 */
/**
  * @brief Print the characters to UART (printf).
  * @retval int
  */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

static void LoadFirmware ( void )
{
	printf("Loading firmware... \r\n");

	void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*) (0x08008000 + 4U)));
	//__set_MSP(*(volatile uint32_t*) 0x08007FFF);
	// Turn OFF the Green Led to tell the user that Bootloader is not running
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET );    //Green LED OFF
	printf("Booting game... \r\n");
	app_reset_handler();    //call the app reset handler
}

bool CheckArrayIsEnd(uint8_t *_buffer, uint8_t *compareTo, uint8_t *lastThree)
{
	int x = 0;
	int y = 0;

	bool equalArrays = false;

	for(x = 0; x < PAGE_SIZE; x++)
	{
	  	lastThree[0] = lastThree[1];
	  	lastThree[1] = lastThree[2];
	  	lastThree[2] = _buffer[x];

		equalArrays = true;
		for (y = 0; y < 3; y++)
		{
		  if( lastThree[y] != compareTo[y] )
		  {
			  equalArrays = false;
				break;
		  }
		}

		if(equalArrays) break;
	}

	return equalArrays;
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
