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
#include <main.hpp>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define START_LENGTH 10;
#define START_SPEED 2;

static void ResetGame ( void );

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim6;

 UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void UPrint(char _out[]);
int length_of_string(char* p);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool gameOver = false;


char scoreText[8] = {0};

uint8_t currentLength = START_LENGTH;
uint8_t direction = 1; //0 up 1 right 2 down 3 left

uint8_t segmentPosesX[64] = {0};
uint8_t segmentPosesY[64] = {0};

bool segments[64] = {0};
uint8_t tailSegment = START_LENGTH;


uint8_t baitX = 0;
uint8_t baitY = 0;

uint8_t maxX = 0;
uint8_t maxY = 0;
uint8_t minX = 84;
uint8_t minY = 48;


int score = 1;


void SetRandomBait()
{
	LCD_setPixel(baitX, baitY, false);
	baitX = LCD_WIDTH;
	baitX = ( rand() % (baitX - 16) ) + 16;

	baitY = LCD_HEIGHT;
	baitY = ( rand() % (baitY - 16) ) + 16;
	LCD_setPixel(baitX, baitY, true);


	LCD_refreshArea(baitX - 2, baitY - 2, baitX + 2, baitY + 2);
}

void SetGameOver()
{
	gameOver = true;
	LCD_clrBuffer();
	LCD_clrScr();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	tailSegment--;
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
  MX_TIM6_Init();


  /* USER CODE BEGIN 2 */
#ifdef __cplusplus
  UPrint("Setting up game... \r\n");

  HAL_TIM_Base_Start(&htim6);

  LCD_setRST(GPIOB, GPIO_PIN_13);
  LCD_setCE(GPIOB, GPIO_PIN_14);
  LCD_setDC(GPIOB, GPIO_PIN_15);
  LCD_setDIN(GPIOB, GPIO_PIN_1);
  LCD_setCLK(GPIOB, GPIO_PIN_2);
  LCD_init();

  for(int i = 0; i < currentLength; i++)
  {
	  segments[i] = true;

	  segmentPosesY[i] = 20;
	  segmentPosesX[i] = currentLength + 16 - i;

  }

  direction = 1;

  GamepadButton BUTTON_Left(GPIOA, GPIO_PIN_7);
  GamepadButton BUTTON_Right(GPIOA, GPIO_PIN_6);

  GamepadButton BUTTON_Up(GPIOA, GPIO_PIN_12);
  GamepadButton BUTTON_Down(GPIOA, GPIO_PIN_11);

  GamepadButton BUTTON_A(GPIOC, GPIO_PIN_7);
  GamepadButton BUTTON_B(GPIOB, GPIO_PIN_6);

  SetRandomBait();

  LCD_drawRectangle(0, 10, 83, 47);

  LCD_refreshScr();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		BUTTON_Left.UpdateButton();
		BUTTON_Right.UpdateButton();
		BUTTON_Up.UpdateButton();
		BUTTON_Down.UpdateButton();
		BUTTON_A.UpdateButton();
		BUTTON_B.UpdateButton();

		if(!gameOver)
		{

			if(BUTTON_Up.IsPressing && direction != 2)
			{
				direction = 0;
			}
			else if(BUTTON_Right.IsPressing && direction != 3)
			{
				direction = 1;
			}
			else if(BUTTON_Down.IsPressing && direction != 0)
			{
				direction = 2;
			}
			else if(BUTTON_Left.IsPressing && direction != 1)
			{
				direction = 3;
			}

			LCD_setPixel(segmentPosesX[tailSegment], segmentPosesY[tailSegment], false);
			for(int i = 63; i > 0; i--)
			{
				if(!segments[i]) continue;

				segmentPosesX[i] = segmentPosesX[i - 1];
				segmentPosesY[i] = segmentPosesY[i - 1];

				if(segmentPosesX[i] > maxX) maxX = segmentPosesX[i];
				if(segmentPosesY[i] > maxY) maxY = segmentPosesY[i];

				if(segmentPosesX[i] < minX) minX = segmentPosesX[i];
				if(segmentPosesY[i] < minY) minY = segmentPosesY[i];

				LCD_setPixel(segmentPosesX[i], segmentPosesY[i], true);
			}

			switch (direction)
			{
				case 0:
					segmentPosesY[0]--;
					break;
				case 1:
					segmentPosesX[0]++;
					break;
				case 2:
					segmentPosesY[0]++;
					break;
				case 3:
					segmentPosesX[0]--;
					break;
			}

			LCD_setPixel(segmentPosesX[0], segmentPosesY[0], true);

			for(int i = 1; i < tailSegment + 1; i++)
			{
				if(segmentPosesX[0] == segmentPosesX[i] && segmentPosesY[0] == segmentPosesY[i])
				{
					SetGameOver();
					break;
				}
			}

			if(segmentPosesX[0] == baitX && segmentPosesY[0] == baitY)
			{
				SetRandomBait();
				currentLength++;
				tailSegment++;

				score++;
				segments[tailSegment] = true;

				if(tailSegment == 63)
				{
					ResetGame();
				}
			}

			if(segmentPosesX[0] == 0 || segmentPosesX[0] == 83 || segmentPosesY[0] == 10 || segmentPosesY[0] == 47) SetGameOver();

			LCD_setPixel(baitX, baitY, true);

			LCD_refreshArea(minX - 2, minY - 2, maxX + 2, maxY + 2);

			itoa((int) score, scoreText, 10);
			LCD_print(scoreText, 0, 0);
		}
		else
		{
			LCD_print("GameOver...", 0, 0);

			if(BUTTON_A.IsPressing)
			{
				ResetGame();
			}
		}
	while(4000 > __HAL_TIM_GET_COUNTER(&htim6)); //wait for 25fps

	__HAL_TIM_SET_COUNTER(&htim6,0);

    /* USER CODE END WHILE */



    /* USER CODE BEGIN 3 */

	}
#endif
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 80 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65536 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

	  /*Configure GPIO pin : PC1 */
	  GPIO_InitStruct.Pin = GPIO_PIN_1;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA8 PA9 PA15 */
	  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UPrint(char _out[])
{
	HAL_UART_Transmit(&huart2, (uint8_t *) _out, length_of_string(_out),10);
}

int length_of_string(char* p) {
    int count = 0;

    while (*p != '\0') {
        count++;
        p++;
    }

    return count;
}

static void ResetGame ( void )
{

    uint32_t reset_handler_add = *((volatile uint32_t *)(0x08008000 + 4));
    void (*app_reset_handler)(void) = (void (*)(void))reset_handler_add;
    app_reset_handler();
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
