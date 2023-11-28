/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile int UART1_Transmitted;
volatile int UART1_Received;;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1)
	{
		UART1_Transmitted = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		UART1_Received = 1;
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

enum LEDSTATE{
	INIT = 0,
	IDLE,
	WAVE,
	BLINK,
	SLIDE,
	COMET,
	COMET1,
	GLOW,
	GLOW1,
	SOC,
	CLEARALL
};

enum LEDSTATE stmState = INIT;

int datasentflag = 0;
int changeFlag = 0;

#define personalLED
int LEDhigh;
int LEDarr;
int LEDlow;
int MAX_LED;
int RESET_LED_DELAY;

#ifdef personalLED
	#define LEDhigh 70
	#define LEDlow 35
	#define LEDarr 105
	#define MAX_LED 60
	#define RESET_LED_DELAY 50
#else
	#define LEDhigh 50
	#define LEDlow 25
	#define LEDarr 75
	#define MAX_LED 144
	#define RESET_LED_DELAY 320
#endif

uint8_t LED_Data[MAX_LED][4];
uint16_t pwmData[(24*MAX_LED)+RESET_LED_DELAY];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		changeFlag = 1;
	}
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	if(LEDnum < 0)
		LEDnum = 0;
	if(LEDnum > MAX_LED)
		LEDnum = MAX_LED;
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = LEDhigh;  //70	ARR: 105
			}

			else pwmData[indx] = LEDlow;  //35

			indx++;
		}

	}

	for (int i=0; i<RESET_LED_DELAY; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);

	while (!datasentflag){};
	datasentflag = 0;
}

void clearLEDData()
{
	for(int x = 0; x < MAX_LED; x++)
	{
		Set_LED(x, 0, 0, 0);
	}
}

void setAllLED(int red, int green, int blue)
{
	for(int x = 0; x < MAX_LED; x++)
	{
		Set_LED(x, red, green, blue);
	}
}

void blinkLED(int onTime, int offTime, int red, int green, int blue)
{
	setAllLED(red, green, blue);
	WS2812_Send();
	HAL_Delay(onTime);
	clearLEDData();
	WS2812_Send();
	HAL_Delay(offTime);
}

int head = 0;
int tail = 0;

void cometLED(int delay, int red, int green, int blue, int cometLength)
{
	head++;
	tail++;
	if(head == MAX_LED)
		head--;
	if(tail == MAX_LED)
	{
		head = 0;
		tail = 0;
	}
	int x = 0;
	if(head < cometLength)
	{
		tail--;
		while(x < head)
		{
			Set_LED(x, red, green, blue);
			x++;
		}
	}
	else
	{
		while(x < tail)
		{
			Set_LED(x, 0, 0, 0);
			x++;
		}
	}
	while(x < head)
	{
		Set_LED(x, red, green, blue);
		x++;
	}
	while(x < MAX_LED)
	{
		Set_LED(x, 0, 0, 0);
		x++;
	}
	WS2812_Send();
	HAL_Delay(delay);
}


int slideHead = 0;
void slideLED(int delay, int red, int green, int blue)
{
	slideHead++;
	if(slideHead == MAX_LED)
	{
		slideHead = 0;
		clearLEDData();
		WS2812_Send();
	}
	Set_LED(slideHead, red, green, blue);
	WS2812_Send();
	HAL_Delay(delay);
}

int dir = 0;
float redIncrement;
int currRed;
float greenIncrement;
int currGreen;
float blueIncrement;
int currBlue;

void glowInitLED(int red, int green, int blue, int red1, int green1, int blue1)
{
	redIncrement = (red1-red)/100;
	greenIncrement = (green1-green)/100;
	blueIncrement = (blue1-blue)/100;
	currRed = red;
	currBlue = blue;
	currGreen = green;
	dir = 1;
}

void glowLED(int delay, int red, int green, int blue, int red1, int green1, int blue1)
{
	if(dir == 0)
	{
		if((currRed - red) < (2*redIncrement))
			dir = 1;
		currRed = currRed - redIncrement;
		currGreen = currGreen - greenIncrement;
		currBlue = currBlue - blueIncrement;
	}
	else if(dir == 1)
	{
		if((red1-currRed) < (2*redIncrement))
			dir = 0;
		currRed = currRed + redIncrement;
		currGreen = currGreen + greenIncrement;
		currBlue = currBlue + blueIncrement;
	}
	setAllLED(currRed, currGreen, currBlue);
	WS2812_Send();
	HAL_Delay(delay);
}

void soc(int soc, int red, int green, int blue)
{
	float tempNUM =(soc*MAX_LED)/10000;
	int numLEDs = (int)(tempNUM);
	Set_LED(numLEDs, 0, 0, 0);
	for(int x = 0; x < numLEDs; x++)
	{
		Set_LED(x, red, green, blue);
	}
	WS2812_Send();
	HAL_Delay(500);
	Set_LED(numLEDs, red, green, blue);
	WS2812_Send();
	HAL_Delay(500);
}

int currTick = 0;
void wave(int waveRed, int waveGreen, int waveBlue, int bgRed, int bgGreen, int bgBlue, int waveLength, int intensity)
{
	int x = 0;
	int y = MAX_LED;
	setAllLED(bgRed, bgGreen, bgBlue);
	int cycleTicks = tail*intensity;
	if(currTick > cycleTicks)
	{
		currTick = 0;
		if(dir == 1)
		{
			if(head <= 0)
			{
				if(tail < 3)
				{
					dir = 0;
					head = tail;
				}
				else
				{
					tail--;
					while(x < tail)
					{
						Set_LED(x, waveRed, waveGreen, waveBlue);
						Set_LED(y, waveRed, waveGreen, waveBlue);
						x++;
						y--;
					}
				}
			}
			else
			{
				head--;
				tail = head + waveLength;
				x = head;
				y = MAX_LED - head;
				while(x < tail)
				{
					Set_LED(x, waveRed, waveGreen, waveBlue);
					Set_LED(y, waveRed, waveGreen, waveBlue);
					x++;
					y--;
				}
			}
		}
		else
		{
			head++;
			tail = head - waveLength;
			x = tail;
			y = MAX_LED - tail;
			while(x < head)
			{
				Set_LED(x, waveRed, waveGreen, waveBlue);
				Set_LED(y, waveRed, waveGreen, waveBlue);
				x++;
				y--;
			}
			if(head > (MAX_LED/2 + 5))
			{
				dir = 1;
				head = tail;
			}
		}
		WS2812_Send();
	}
	else
		currTick++;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag = 1;
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  int tempSOC = 0;
  htim1.Instance->ARR = LEDarr;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 switch(stmState)
	 {
	 case INIT:
		 clearLEDData();
		 WS2812_Send();
		 stmState = IDLE;
		 break;
	 case IDLE:
		 if(changeFlag == 1)
		 {
			 changeFlag = 0;
			 stmState = WAVE;
		 }
		 break;
	 case WAVE:
		 wave(75, 50, 50, 30, 20, 20, 15, 50);
		 if(changeFlag == 1)
		 {
			 changeFlag = 0;
			 stmState = SOC;
		 }
		 break;
	 case SOC:
		 soc(tempSOC, 10, 100, 20);
		 if(changeFlag == 1)
		 {
			 changeFlag = 0;
			 if(tempSOC == 10000)
				 stmState = BLINK;
			 tempSOC = tempSOC + 1000;
		 }
		 break;
	 case BLINK:
		 blinkLED(600, 500, 100, 100, 0);
		 if(changeFlag == 1)
		 {
			 changeFlag = 0;
			 stmState = SLIDE;
		 }
		 break;
	 case SLIDE:
		 slideLED(5,100,20,20);
		 if(changeFlag == 1)
		 {
			 changeFlag = 0;
			 stmState = COMET;
		 }
		 break;
	 case COMET:
		 cometLED(60,100,20,0,80);
		 if(changeFlag == 1)
		 {
			 changeFlag = 0;
			 stmState = COMET1;
		 }
		 break;
	 case COMET1:
		 cometLED(5,120,0,120,20);
		 if(changeFlag == 1)
		 {
			 changeFlag = 0;
			 glowInitLED(0, 0, 100, 100, 0, 0);
			 stmState = GLOW;
		 }
		 break;
	 case GLOW:
		 glowLED(40, 0, 0, 100, 100, 0, 0);
		 if(changeFlag == 1)
		 {
			 changeFlag = 0;
			 glowInitLED(0, 0, 0, 255, 255, 255);
			 stmState = GLOW1;
		 }
		 break;
	 case GLOW1:
		 glowLED(4, 0, 0, 0, 255, 255, 255);
		 if(changeFlag == 1)
		 {
			 changeFlag = 0;
			 stmState = CLEARALL;
		 }
		 break;
	 case CLEARALL:
		 clearLEDData();
		 WS2812_Send();
		 stmState = IDLE;
		 break;

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 75;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blue_Pin */
  GPIO_InitStruct.Pin = blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
