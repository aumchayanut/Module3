/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define End_Eff 0x23
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

//UART
typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;

UARTStucrture UART2 =
{ 0 };

uint8_t MainMemory[255] =
{ 0 };
static uint8_t parameter[256] =
{ 0 };

typedef enum
{
	Idle,
	Start,
	Frame1,
	Frame2,
	Frame3,
//	Mode1,
//	Mode2,
//	Mode3,
//	Mode4,
//	Mode5,
//	Mode6,
//	Mode7,
//	Mode8,
//	Mode9,
//	Mode10,
//	Mode11,
//	Mode12,
//	Mode13,
//	Mode14,
	CS

} UARTState;
uint8_t Mode = 0;
uint8_t Frame = 0;
uint8_t n_Station = 0;
uint16_t Station[10] = {0};


//PID
int InitialPWM = 1300 ; //offset start pwm
uint64_t TimestampPID = 0 ; //PID period
float SampleTime = 0.00001 ; //PID period
float PreviousPWM = 0 ;
float preErr1,preErr2,Propotional,Integrator,Differentiator,preVel,Tau ;
float DeltaU;
float P = 20;
float I = 5;
float D = 0;
float p,i,d;

//Traj
float StartTime ; //Start moving time
uint8_t StartMoving = 0 ; //Start moving signal
uint64_t t = 0 ; //traj gen period
uint8_t ST = 0 ; //1 when gen traj
float velocity; //velocity gen
float position;
float acceleration;
float StartPos;
float FinalPos = 30 ; //Degree
float Qi,Qf,TA,TV,T,tau ;
float TA1,TA2,TA3,TV1 ;
uint64_t Trajtimestamp;
uint8_t via_point = 0;


//Set Home
uint8_t HomeSignal[8] = {0} ; //read from encoder
uint8_t SetHomeFlag = 0 ; //Status set home - 1 working
uint8_t StartSetHome = 0 ; //Set home trigger
uint8_t sum;
uint64_t SetHomeTimeStamp = 0;
uint8_t home = 0;


//Read Encoder
uint64_t TimestampEncoder = 0 ;//period of low pass read velocity
float EncoderVel = 0 ; //Velovity after low pass


//General
float request = 0 ; //Velocity want to be
uint16_t PWMPercent = 0 ; //Motor PWM 0 - 10000
uint64_t _micros = 0;
float VelocityRPM ; //velocity RPM unit
float Degree = 0 ; //Position Degree unit
uint8_t Direction = 0 ; //0 CW - 1 CCW - 2 Stop
uint8_t ButtonBuffer[2] = {0} ; //Blue button
uint64_t pidtuner = 0;
uint8_t StartTune = 0;

float x = 0 ; // request changed

uint16_t EndEffStatus = 0 ;
uint8_t test = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros() ;
float EncoderVelocity_Update();
void PWMgeneration() ;
float Velocity() ;
void PID() ;
void PIDinit() ;
void Trajec();
void SetHome() ;
void EndEffWrite() ;
void GoToGoal(float goal);

//UART Function
void UARTInit(UARTStucrture *uart);
void UARTResetStart(UARTStucrture *uart);
uint32_t UARTGetRxHead(UARTStucrture *uart);
int16_t UARTReadChar(UARTStucrture *uart);
void UARTTxDumpBuffer(UARTStucrture *uart);
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);
void Protocal(int16_t dataIn, UARTStucrture *uart);
uint16_t CheckSum(uint8_t Mode, uint8_t Frame, uint16_t Data);



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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
	UART2.huart = &huart2;
	UART2.RxLen = 255;
	UART2.TxLen = 255;
	UARTInit(&UART2);
	UARTResetStart(&UART2);
  /* USER CODE BEGIN 2 */

//	UART2.huart = &huart2;
//	UART2.RxLen = 255;
//	UART2.TxLen = 255;
//	UARTInit(&UART2);
//	UARTResetStart(&UART2);


  PIDinit() ;

  // start PWM
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // start micros
  HAL_TIM_Base_Start_IT(&htim2);

  // start Encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//***********General********************************************************************

	  VelocityRPM = Velocity() ; //rpm unit
	  Degree = htim3.Instance->CNT * 360.0 / 2048.0 ; //Degree unit
	  PWMgeneration() ; //Gen PWM
	  home = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
	  ButtonBuffer[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) ; // Read Blue button
//**************************************************************************************
//**********Blue Button Push*********************
	  if (!ButtonBuffer[0] && ButtonBuffer[1])
	  {
//		  EndEffWrite() ;
//		  z = 1 ;
		  StartSetHome = 1 ; //Set home trigger
		  SetHomeFlag = 0;
//		  SetHomeFlag = 0 ;
	  }
	  ButtonBuffer[1] = ButtonBuffer[0] ;
//************************************************
//**********Set Home******************************
	  if (StartSetHome == 1)
	  {
		  SetHome() ;
	  }
//***********************************************
//**************PID******************************
	  if (micros() - TimestampPID > 1000)
	  {
		  if (StartMoving || StartSetHome)
		  {
			  P = p;
			  I = i;
			  D = d;
			  PID() ;
		  }
		  TimestampPID = micros() ;
	  }
//************************************************
//**************UART******************************
	  int16_t inputChar = UARTReadChar(&UART2);
	  if (inputChar != -1)
	  {
		  Protocal(inputChar, &UART2);
	  }

//	  if (micros()-pidtuner > 5000000)
//	  {
//		  StartTune = !StartTune;
//	  }
//
//	  if (StartTune)
//	  {
//		  request = 5;
//	  }
//	  else
//	  {
//		  request = 0;
//	  }

//********other**************
	  if (request == 0)
	  {
		  Direction = 2 ;
	  }
//****************************
	  if (StartMoving == 0)
	  {
		  T = 0;
		  TV = 0;
		  TA = 0;
		  ST = 0;
	  }
	  if (StartMoving == 1)
	  {
		  Trajec();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 1000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2047;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Home_Pin */
  GPIO_InitStruct.Pin = Home_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Home_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void EndEffWrite()
{
	if (hi2c1.State == HAL_I2C_STATE_READY)
	{
//		HAL_Delay(500);
//		HAL_I2C_Master_Transmit(&hi2c1, 0x23 << 1, 0x45, 1	, 1000); //Write eff
		HAL_I2C_Master_Transmit_IT(&hi2c1, 0x23<<1, 0x45, 1);
	}

}
//void GoToGoal(float goal)
//{
//
//}

void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;
}
void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}
uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}
int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;

	}
	return Result;

}
void UARTTxDumpBuffer(UARTStucrture *uart)
{
	static uint8_t MultiProcessBlocker = 0;

	if (uart->huart->gState == HAL_UART_STATE_READY && !MultiProcessBlocker)
	{
		MultiProcessBlocker = 1;

		if (uart->TxHead != uart->TxTail)
		{
			//find len of data in buffer (Circular buffer but do in one way)
			uint16_t sentingLen =
					uart->TxHead > uart->TxTail ?
							uart->TxHead - uart->TxTail :
							uart->TxLen - uart->TxTail;

			//sent data via DMA
			HAL_UART_Transmit_DMA(uart->huart, &(uart->TxBuffer[uart->TxTail]),
					sentingLen);
			//move tail to new position
			uart->TxTail = (uart->TxTail + sentingLen) % uart->TxLen;

		}
		MultiProcessBlocker = 0;
	}

}
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len)
{
	//check data len is more than buffer?
	uint16_t lenAddBuffer = (len <= uart->TxLen) ? len : uart->TxLen;

	// find number of data before end of ring buffer
	uint16_t numberOfdataCanCopy =
			lenAddBuffer <= uart->TxLen - uart->TxHead ?
					lenAddBuffer : uart->TxLen - uart->TxHead;
	//copy data to the buffer
	memcpy(&(uart->TxBuffer[uart->TxHead]), pData, numberOfdataCanCopy);

	//Move Head to new position

	uart->TxHead = (uart->TxHead + lenAddBuffer) % uart->TxLen;
	//Check that we copy all data That We can?
	if (lenAddBuffer != numberOfdataCanCopy)
	{
		memcpy(uart->TxBuffer, &(pData[numberOfdataCanCopy]),
				lenAddBuffer - numberOfdataCanCopy);
	}
	UARTTxDumpBuffer(uart);
}
void Protocal(int16_t dataIn,UARTStucrture *uart)
{
	//all Static Variable
	static UARTState State = Idle;
	uint8_t StartBit = 0b1001;
//	static uint16_t datalen = 0;
	static uint16_t CollectedData = 0;
//	static uint8_t inst = 0;
//
//	static uint16_t CRCCheck = 0;
//	static uint16_t packetSize = 0;
//	static uint16_t CRC_accum;

//	//State Machine
////	switch (State)
////	{
////	case Idle:
////		if (dataIn&0xf0 == 1001)
////			Mode = dataIn&0xf;
////			if (Mode == 0b0001)
////			{
////				Mode = 1;
////				Frame = 2;
////			}
////			if (Mode == 0b0010)
////			{
////				Mode = 2;
////				Frame = 1;
////			}
////			if (Mode == 0b0011)
////			{
////				Mode = 3;
////				Frame = 1;
////			}
////			if (Mode == 0b0100)
////			{
////				Mode = 4;
////				Frame = 2;
////			}
////			if (Mode == 0b0101)
////			{
////				Mode = 5;
////				Frame = 2;
////			}
////			if (Mode == 0b0110)
////			{
////				Mode = 6;
////				Frame = 2;
////			}
////			if (Mode == 0b0111)
////			{
////				Mode = 7;
////				Frame = 3;
////			}
////			if (Mode == 0b1000)
////			{
////				Mode = 8;
////				Frame = 1;
////			}
////			if (Mode == 0b1001)
////			{
////				Mode = 9;
////				Frame = 1;
////			}
////			if (Mode == 0b1010)
////			{
////				Mode = 10;
////				Frame = 1;
////			}
////			if (Mode == 0b1011)
////			{
////				Mode = 11;
////				Frame = 1;
////			}
////			if (Mode == 0b1100)
////			{
////				Mode = 12;
////				Frame = 1;
////			}
////			if (Mode == 0b1101)
////			{
////				Mode = 13;
////				Frame = 1;
////			}
////			if (Mode == 0b1110)
////			{
////				Mode = 14;
////				Frame = 1;
////			}
////			if (Frame == 1)
////			{
////				State = Frame1;
////			}
////			if (Frame == 2)
////			{
////				State = Frame2;
////			}
////			if (Frame == 3)
////			{
////				State = Frame3;
////			}
////		break;
////	case Frame1:
////			if (dataIn == CheckSum(Mode, Frame, CollectedData))
////			{
////
////			}
////		break;
//	case DNMXP_2ndHeader:
//		if (dataIn == 0xFD)
//			State = DNMXP_3rdHeader;
//		else if (dataIn == 0xFF)
//			; //do nothing
//		else
//			State = DNMXP_idle;
//		break;
//	case DNMXP_3rdHeader:
//		if (dataIn == 0x00)
//			State = DNMXP_Reserved;
//		else
//			State = DNMXP_idle;
//		break;
//	case DNMXP_Reserved:
//		if ((dataIn == MotorID) | (dataIn == 0xFE))
//			State = DNMXP_ID;
//		else
//			State = DNMXP_idle;
//		break;
//	case DNMXP_ID:
//		datalen = dataIn & 0xFF;
//		State = DNMXP_LEN1;
//		break;
//	case DNMXP_LEN1:
//		datalen |= (dataIn & 0xFF) << 8;
//		State = DNMXP_LEN2;
//		break;
//	case DNMXP_LEN2:
//		inst = dataIn;
//		State = DNMXP_Inst;
//		break;
//	case DNMXP_Inst:
//		if (datalen > 3)
//		{
//			parameter[0] = dataIn;
//			CollectedData = 1; //inst 1 + para[0] 1
//			State = DNMXP_ParameterCollect;
//		}
//		else
//		{
//			CRCCheck = dataIn & 0xff;
//			State = DNMXP_CRCAndExecute;
//		}
//
//		break;
//	case DNMXP_ParameterCollect:
//
//		if (datalen-3 > CollectedData)
//		{
//			parameter[CollectedData] = dataIn;
//			CollectedData++;
//		}
//		else
//		{
//			CRCCheck = dataIn & 0xff;
//			State = DNMXP_CRCAndExecute;
//		}
//		break;
//	case DNMXP_CRCAndExecute:
//		CRCCheck |= (dataIn & 0xff) << 8;
//		//Check CRC
//		CRC_accum = 0;
//		packetSize = datalen + 7;
//		//check overlapse buffer
//		if (uart->RxTail - packetSize >= 0) //not overlapse
//		{
//			CRC_accum = update_crc(CRC_accum,
//					&(uart->RxBuffer[uart->RxTail - packetSize]),
//					packetSize - 2);
//		}
//		else//overlapse
//		{
//			uint16_t firstPartStart = uart->RxTail - packetSize + uart->RxLen;
//			CRC_accum = update_crc(CRC_accum, &(uart->RxBuffer[firstPartStart]),
//					uart->RxLen - firstPartStart);
//			CRC_accum = update_crc(CRC_accum, uart->RxBuffer, uart->RxTail - 2);
//
//		}
//
//		if (CRC_accum == CRCCheck)
//		{
//			switch (inst)
//			{
//			case 0x01:// ping
//			{
//				//create packet template
//				uint8_t temp[] =
//				{ 0xff, 0xff, 0xfd, 0x00, 0x00, 0x04, 0x00, 0x55,0x00, 0x00, 0x00};
//				//config MotorID
//				temp[4] = MotorID;
//				//calcuate CRC
//				uint16_t crc_calc = update_crc(0, temp, 9);
//				temp[9] = crc_calc & 0xff;
//				temp[10] = (crc_calc >> 8) & 0xFF;
//				//Sent Response Packet
//				UARTTxWrite(uart, temp, 11);
//				break;
//			}
//
//			case 0x02://READ
//			{
//				uint16_t startAddr = (parameter[0]&0xFF)|(parameter[1]<<8 &0xFF);
//				uint16_t numberOfDataToRead = (parameter[2]&0xFF)|(parameter[3]<<8 &0xFF);
//				uint8_t temp[] = {0xff,0xff,0xfd,0x00,0x00,0x00,0x00,0x55,0x00};
//				temp[4] = MotorID;
//				temp[5] = (numberOfDataToRead + 4) & 0xff ; // +inst+err+crc1+crc2
//				temp[6] = ((numberOfDataToRead + 4)>>8) & 0xff ;
//				uint16_t crc_calc = update_crc(0, temp, 9);
//				crc_calc = update_crc(crc_calc ,&(Memory[startAddr]),numberOfDataToRead);
//				uint8_t crctemp[2];
//				crctemp[0] = crc_calc&0xff;
//				crctemp[1] = (crc_calc>>8)&0xff;
//				UARTTxWrite(uart, temp,9);
//				UARTTxWrite(uart, &(Memory[startAddr]),numberOfDataToRead);
//				UARTTxWrite(uart, crctemp,2);
//				break;
//			}
//			case 0x03://WRITE
//			{
//				//LAB
//				uint16_t startAddr = (parameter[0]&0xFF)|(parameter[1]<<8 &0xFF);
//				uint16_t ParameterNumber = datalen - 5 ;
//				for (int i = 0; i < ParameterNumber-1; i++)
//				{
//					Memory[startAddr + i] = parameter[i+1] ;
//				}
//				uint8_t temp[] = {0xff,0xff,0xfd,0x00,0x00,0x04,0x00,0x55,0x00};
//				temp[4] = MotorID ;
//				uint16_t crc_calc = update_crc(0, temp, 9);
//				uint8_t crctemp[2];
//				crctemp[0] = crc_calc&0xff;
//				crctemp[1] = (crc_calc>>8)&0xff;
//				UARTTxWrite(uart, temp,9);
//				UARTTxWrite(uart, crctemp,2);
//
//			}
////			default: //Unknown Inst
////			{
////				uint8_t temp[] =
////				{ 0xff, 0xff, 0xfd, 0x00, 0x00, 0x05, 0x00, 0x55, 0x02, 0x00,
////						0x00 };
////				temp[4] = MotorID;
////				uint16_t crc_calc = update_crc(0, temp, 9);
////				temp[9] = crc_calc & 0xff;
////				temp[10] = (crc_calc >> 8) & 0xFF;
////				UARTTxWrite(uart, temp, 11);
////
////				break;
////			}
//			}
//		}
//		else //crc error
//		{
//			uint8_t temp[] =
//			{ 0xff, 0xff, 0xfd, 0x00, 0x00, 0x05, 0x00, 0x55, 0x03, 0x00, 0x00 };
//			temp[4] = MotorID;
//			uint16_t crc_calc = update_crc(0, temp, 9);
//			temp[9] = crc_calc & 0xff;
//			temp[10] = (crc_calc >> 8) & 0xFF;
//			UARTTxWrite(uart, temp, 11);
//		}
//		State = DNMXP_idle;
//		break;
	}
}

uint16_t CheckSum(uint8_t Mode, uint8_t Frame, uint16_t Data)
{
	uint8_t Start = 0b1001;
	uint16_t result = 0;
	if (Frame == 1)
	{
		result = ~((Start<<4)|Mode);
	}
	if (Frame == 2)
	{
		result = ~(((Start<<4)|Mode)+Data);
	}
	if (Frame == 3)
	{
		result = ~(((Start<<4)|Mode)+Data);
	}
	return result;
}
void SetHome()
{
	HomeSignal[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) ; //Read set home
	sum = HomeSignal[0] + HomeSignal[1]+ HomeSignal[2]+ HomeSignal[3]+ HomeSignal[4]+ HomeSignal[5]+ HomeSignal[6]+ HomeSignal[7];

	if (SetHomeFlag == 0)
	{
		request = 5;
		if (sum > 0)
		{
			SetHomeFlag = 1;
			SetHomeTimeStamp = micros();
		}
	}
	if (SetHomeFlag == 1)
	{
		request = 0;
		if (micros()-SetHomeTimeStamp > 1000000)
		{
			SetHomeFlag = 2;
		}
	}
	if (SetHomeFlag == 2)
	{
		request = -0.5;
		if (sum > 0)
		{
			request = 0;
			htim3.Instance->CNT = 0;
			SetHomeFlag = 3;
		}
	}



}


void Trajec()
{
	float Vmax = 10 ; //rpm
	Vmax = Vmax * 0.10472 ; //rad per sec
	float Amax = 0.5 ;  //rad per sec square
	if (ST == 0)
	{
		StartTime = micros() ;
		ST = 1 ;
		StartPos = Degree * 3.14159 / 180.0 ; //rad
	}
	tau = (micros() - StartTime) / 1000000 ; //sec

	Qi = StartPos ;
	Qf = FinalPos * 3.14159 / 180.0 ;

	if (Qf - Qi > 3.14159265)
	{
		Qi += 2*3.14159265;
	}
	if (Qf - Qi < -3.14159265)
	{
		Qf += 2*3.14159265;
	}
	TA1= (40.0 * sqrt(3.0))*(Qf - Qi);
	TA2 = TA1/(3.0*Amax) ;
	if (TA2 >= 0)
	{
		TA3 = TA2;
	}
	else
	{
		TA3 = TA2 - (2*TA2);
	}
	TA = sqrt(TA3) / 2.0 ;

	TV1 = (15*Qf - 15*Qi)/(8*Vmax);
	if (TV1 >= 0)
	{
		TV = TV1;
	}
	else
	{
		TV = TV1 - (2*TV1);
	}

	if (TV > TA)
	{
		T = TV ;
	}
	if (TV <= TA)
	{
		T = TA ;
	}
	float a0 = Qi ;
	float a1 = 0 ;
	float a2 = 0 ;
	float a3 = (1 / (2 * (T*T*T))) * (20 * (Qf-Qi)) ;
	float a4 = (1 / (2 * (T*T*T*T))) * (30 * (Qi-Qf)) ;
	float a5 = (1 / (2 * (T*T*T*T*T))) * (12 * (Qf-Qi)) ;

	if (micros() - StartTime < T*1000000)
	{
		if (micros() - Trajtimestamp > 1500)
		{
			position = a0 + a1*tau + a2*tau*tau + a3*tau*tau*tau + a4*tau*tau*tau*tau + a5*tau*tau*tau*tau*tau; //rad
			velocity = a1 + 2*a2*tau + 3*a3*tau*tau + 4*a4*tau*tau*tau + 5*a5*tau*tau*tau*tau; //rad/s
			acceleration = 2*a2 + 6*a3*tau + 12*a4*tau*tau + 20*a5*tau*tau*tau; //rad per secsquare

			position = position * 180 / 3.1415 ; //degree
			velocity = velocity * 9.549297; //rpm
			Trajtimestamp = micros() ;
			request = velocity ;
		}
	}
	if (micros() - StartTime > (T*1000000)+500000)
	{
		StartMoving = 0;
	}

}

void PIDinit()
{
	preErr1 = 0 ;
	preErr2 = 0 ;
	PreviousPWM = 0 ;
	PWMPercent = 0 ;
	Propotional = 0 ;
	Integrator = 0 ;
	Differentiator = 0 ;
	P = 0 ;
	I = 0 ;
	D = 0 ;
	Tau = 0 ;
}

void PID()
{
	float req,Vel ;
	if (request < 0)
	{
		req = - request ;
		Vel = - VelocityRPM ;
		Direction = 1 ;
	}
	if (request > 0)
	{
		req = request ;
		Vel = VelocityRPM ;
		Direction = 0 ;
	}
	float error = req - Vel;
	DeltaU = ((P+I+D)*error) - ((P+(2*D))*preErr1) + (D*preErr2) ;
	PWMPercent = PreviousPWM + DeltaU + InitialPWM ;
	if (PWMPercent > 10000)
	{
		PWMPercent = 10000;
	}
	if (PWMPercent < 0)
	{
		PWMPercent = 0 ;
	}
	PreviousPWM = PWMPercent - InitialPWM ;
	preErr2 = preErr1 ;
	preErr1 = error ;

}

//void PID()
//{
////	P = 60 ;
////	I = 1500 ;
////	D = 30 ;
//	Tau = 0.2 ;
//	float req,Vel ;
//	if (request < 0)
//	{
//		req = - request ;
//		Vel = - VelocityRPM ;
//		Direction = 1 ;
//	}
//	if (request > 0)
//	{
//		req = request ;
//		Vel = VelocityRPM ;
//		Direction = 0 ;
//	}
//	float error = req - Vel ;
//	Propotional = P * error ;
//	Integrator = Integrator + (0.5 * I * SampleTime * (error + preErr1)) ;
//
//	//********Anti Windup*************
//	float maxInt,minInt ;
//	if (Propotional < 10000)
//	{
//		maxInt = 10000 - Propotional ;
//	}
//	else
//	{
//		maxInt = 0 ;
//	}
//	if (Propotional > 0)
//	{
//		minInt = 0 - Propotional ;
//	}
//	else
//	{
//		minInt = 0 ;
//	}
//	//********************************
//	if (Integrator > maxInt)
//	{
//		Integrator = maxInt ;
//	}
//	else if (Integrator < minInt)
//	{
//		Integrator = minInt ;
//	}
//
//	Differentiator = (2*D*(VelocityRPM - preVel)) + (Differentiator * (2*Tau - SampleTime)) / (2 * Tau + SampleTime) ;
//
//	PWMPercent = Propotional + Integrator + Differentiator + InitialPWM;
//	if (PWMPercent > 10000)
//	{
//		PWMPercent = 10000 ;
//	}
//	if (PWMPercent < 0)
//	{
//		PWMPercent = 0 ;
//	}
//	preErr1 = error ;
//	preVel = Vel ;
//}


float Velocity()
{
	  if (micros() - TimestampEncoder >= 100)
	  {
		  TimestampEncoder = micros();
		  EncoderVel = (EncoderVel * 999 + EncoderVelocity_Update()) / 1000.0;
	  }
	  return EncoderVel * 60.0 / 2048.0; //pulse per sec to rpm
}
void PWMgeneration()
{
	  if (Direction == 0)
	  {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10000);
	  }
	  if (Direction == 1)
	  {
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	  }
	  if (Direction == 2)
	  {
		  PIDinit() ;
		  PWMPercent = 0 ;
	  }
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWMPercent);
}

#define  HTIM_ENCODER htim3
#define  MAX_SUBPOSITION_OVERFLOW 1024
#define  MAX_ENCODER_PERIOD 2048

float EncoderVelocity_Update()
{
	//Save Last state
	static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;

	//read data
	uint32_t EncoderNowPosition = HTIM_ENCODER.Instance->CNT;
	uint64_t EncoderNowTimestamp = micros();

	int32_t EncoderPositionDiff;
	uint64_t EncoderTimeDiff;

	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;

	//compensate overflow and underflow
	if (EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{
		EncoderPositionDiff -= MAX_ENCODER_PERIOD;
	}
	else if (-EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{
		EncoderPositionDiff += MAX_ENCODER_PERIOD;
	}

	//Update Position and time
	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;

	//Calculate velocity
	//EncoderTimeDiff is in uS
	return (EncoderPositionDiff * 1000000) / (float) EncoderTimeDiff;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micros += 4294967295;
	}
}
uint64_t micros()
{
	return _micros + htim2.Instance->CNT;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
