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
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define End_Eff 0x23
#define Ack1 0b101100001110101
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
uint16_t n=0;
typedef enum
{
	Idle,
	Frame1,
	Frame2_1,
	Frame2_2,
	Frame3_n,
	Frame3_station,
	CheckSum2,
	CheckSum3,

} State;
uint8_t Mode = 0;
uint8_t Frame = 0;
uint8_t n_Station = 0;
uint16_t Station[10] = {0};
uint8_t input = 0;
uint8_t UARTsuccess = 0;
uint8_t UARTerror = 0;
uint8_t enable_eff = 0;

//PID
int InitialPWM = 6500 ; //offset start pwm
uint64_t TimestampPID = 0 ; //PID period
float SampleTime = 0.00001 ; //PID period
float PreviousPWM = 0 ;
float preErr1,preErr2,Propotional,Integrator,Differentiator,preVel,Tau ;
float DeltaU;
float P = 0;
float I = 0;
float D = 0;
float p = 47.8125;
float i = 12.8525;
float d = 10;
float PP,II,DD,SUM,PPreerror;

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
float VmaxRPM = 10 ; //rpm
float VmaxReal = 0;
uint8_t YangMaiTrong = 0;
uint8_t FinishedTraj = 0;
uint64_t Trong1Vi = 0;
uint8_t Vi = 0;


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
uint8_t FinishedStation = 0;
uint8_t FinishedTask = 0;

uint16_t EndEffStatus = 0 ;
uint8_t test = 0;
uint8_t frame1 = 0;
uint8_t frame2 = 0;
uint8_t frame3 = 0;
uint16_t DataInTest = 0 ;
uint8_t checksumtest = 0;
uint8_t ReachedStation = 0;
uint8_t StartNstation = 10;
uint8_t MovingState = 0;
uint64_t movingTimestamp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
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
void SecondPID() ;

void WriteACK1();
void WriteACK2();

//UART Function
void UARTInit(UARTStucrture *uart);
void UARTResetStart(UARTStucrture *uart);
uint32_t UARTGetRxHead(UARTStucrture *uart);
int16_t UARTReadChar(UARTStucrture *uart);
void UARTTxDumpBuffer(UARTStucrture *uart);
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);
void Protocal(int16_t dataIn, UARTStucrture *uart);
int16_t CheckSumFunction(uint8_t CheckSum, uint8_t Frame, uint8_t Data);



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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

UART2.huart = &huart2;
UART2.RxLen = 255;
UART2.TxLen = 255;
UARTInit(&UART2);
UARTResetStart(&UART2);


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
//	  Station[0] = 45;
//	  Station[1] = 360;
//	  Station[2] = 25;
//	  Station[3] = 30;
//	  Station[4] = 45;
//	  Station[5] = 270;
//	  Station[6] = 45;
//	  Station[7] = 90;
//	  Station[8] = 270;
//	  Station[9] = 0;
//	  if (StartNstation!=0)
//	  {
//		  if (micros() - movingTimestamp >= 5000000)
//		  {
//			  StartMoving = 1 ;
//		  }
//	  }
//
//	  if (StartNstation != 0)
//	  {
//		  FinalPos = Station[FinishedStation];
//	  }

//***********General********************************
	  ButtonBuffer[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if (ButtonBuffer[1] == 0 && ButtonBuffer[0]== 1)
	  {
		  EndEffWrite();
	  }
	  ButtonBuffer[1] = ButtonBuffer[0];
	  Degree = htim3.Instance->CNT * 360.0 / 2048.0 ; //Degree unit
	  PWMgeneration() ; //Gen PWM
//***************************************************
//**********Get Real Vmax****************************
	  float V;
	  VelocityRPM = Velocity() ; //rpm unit
	  if (VelocityRPM < 0)
	  {
		  V = (-1) * VelocityRPM;
		  if (V > VmaxReal)
		  {
			  VmaxReal = VelocityRPM;
		  }
	  }
	  else
	  {
		  V = VelocityRPM;
		  if (V > VmaxReal)
		  {
			  VmaxReal = VelocityRPM;
		  }
	  }
//*****************************************************
//**********Set Home******************************
	  if (StartSetHome == 1)
	  {
		  SetHome() ;
	  }
//***********************************************
//**************PID******************************
	  if (micros() - TimestampPID > 1000)
	  {
		  P = p;
		  I = i;
		  D = d;
		  PID() ;
		  TimestampPID = micros() ;
	  }
//************************************************
//**************UART******************************
	  int16_t inputChar = UARTReadChar(&UART2);
	  if (inputChar != -1)
	  {
		  Protocal(inputChar, &UART2);
	  }

	  if (Mode == 8)
	  {
		  if (FinishedTask)
		  {
			  WriteACK2();
		  }
	  }
	  if (Mode == 10) //Read ACK
	  {
		  int16_t inputChar = UARTReadChar(&UART2);
		  if (inputChar != -1)
		  {
			  MainMemory[n] = inputChar ;
			  n++ ;
		  }
	  }
	  if (Mode == 11) //Read ACK
	  {
		  int16_t inputChar = UARTReadChar(&UART2);
		  if (inputChar != -1)
		  {
			  MainMemory[n] = inputChar ;
			  n++ ;
		  }
	  }
	  if (Mode == 12)
	  {
		  enable_eff = 1;
	  }
	  if (Mode == 13)
	  {
		  enable_eff = 0;
	  }

//Mode 14 do in function protocal
//	  if (Mode == 14)
//	  {
//
//	  }


//********Stop Motor***************************************
	  if (request == 0)
	  {
		  Direction = 2 ;
	  }
//****************************************************
//*******init Traj***********************************
	  if (StartMoving == 0)
	  {
		  T = 0;
		  TV = 0;
		  TA = 0;
		  ST = 0;
//		  PIDinit();
	  }
//**************************************************
//*******Start Generate Trajectory*******************
	  if (StartMoving == 1)
	  {
		  Trajec();
	  }
	  if (FinishedTraj)
	  {
		  if (FinalPos - Degree < -0.5 || FinalPos - Degree > 0.5)
		  {
			  YangMaiTrong = 1;
			  SUM = 0; //init second PID
		  }
		  else
		  {
			  EndEffWrite();
		  }

	  }


//***************************************************
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  UARTTxDumpBuffer(&UART2);

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
  hi2c1.Init.ClockSpeed = 10000;
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
  htim1.Init.Period = 50000;
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
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
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
	if (hi2c1.State == HAL_I2C_STATE_READY && enable_eff)
	{
		HAL_Delay(500);
		uint8_t temp = 0x45;
		uint8_t add = 0x23;
		HAL_I2C_Master_Transmit(&hi2c1, add << 1, &temp, 1, 1000); //Write eff
//		HAL_I2C_Master_Transmit_IT(&hi2c1, 0x23<<1, 0x45, 1);
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
	uint16_t Result = -1; // -1 Mean no new data

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
	static State State = Idle;
	static uint8_t StartBit = 0b1001;
	static uint8_t ModeIN = 0;
	static uint8_t CheckSum = 0;
	static uint16_t CollectedData = 0;
	static uint16_t CollectedData2 = 0;
	static uint16_t stationSUM;
	static uint8_t CurrentAngle1 = 0;
	static uint8_t CurrentAngle2 = 0;

	DataInTest = dataIn&0xf0;


//	//State Machine
	switch (State)
	{
	case Idle:
		UARTsuccess = 0;
		UARTerror = 0;
		if (DataInTest == 0b10010000)
		{
			ModeIN = dataIn&0xf;
			if (ModeIN == 0b0001)
			{
				Mode = 1;
				CheckSum = (StartBit << 4) | 0b1;
				Frame = 2;
				State = Frame2_1;
			}
			if (ModeIN == 0b0010)
			{
				Mode = 2;
				State = Frame1;
				Frame = 1;
				CheckSum = (StartBit << 4) | 0b10;
			}
			if (ModeIN == 0b0011)
			{
				Mode = 3;
				State = Frame1;
				Frame = 1;
				CheckSum = (StartBit << 4) | 0b11;
			}
			if (ModeIN == 0b0100)
			{
				Mode = 4;
				State = Frame2_1;
				Frame = 2;
				CheckSum = (StartBit << 4) | 0b100;
			}
			if (ModeIN == 0b0101)
			{
				Mode = 5;
				State = Frame2_1;
				Frame = 2;
				CheckSum = (StartBit << 4) | 0b101;
			}
			if (ModeIN == 0b0110)
			{
				Mode = 6;
				State = Frame2_1;
				Frame = 2;
				CheckSum = (StartBit << 4) | 0b110;
			}
			if (ModeIN == 0b0111)
			{
				Mode = 7;
				State = Frame3_n;
				Frame = 3;
				CheckSum = (StartBit << 4) | 0b111;
			}
			if (ModeIN == 0b1000)
			{
				Mode = 8;
				State = Frame1;
				Frame = 1;
				CheckSum = (StartBit << 4) | 0b1000;
			}
			if (ModeIN == 0b1001)
			{
				Mode = 9;
				State = Frame1;
				Frame = 1;
				CheckSum = (StartBit << 4) | 0b1001;
			}
			if (ModeIN == 0b1010)
			{
				Mode = 10;
				State = Frame1;
				Frame = 1;
				CheckSum = (StartBit << 4) | 0b1010;
			}
			if (ModeIN == 0b1011)
			{
				Mode = 11;
				State = Frame1;
				Frame = 1;
				CheckSum = (StartBit << 4) | 0b1011;
			}
			if (ModeIN == 0b1100)
			{
				Mode = 12;
				State = Frame1;
				Frame = 1;
				CheckSum = (StartBit << 4) | 0b1100;
			}
			if (ModeIN == 0b1101)
			{
				Mode = 13;
				State = Frame1;
				Frame = 1;
				CheckSum = (StartBit << 4) | 0b1101;
			}
			if (ModeIN == 0b1110)
			{
				Mode = 14;
				State = Frame1;
				Frame = 1;
				CheckSum = (StartBit << 4) | 0b1110;
			}
		}
		else
		{
			State = Idle;
		}
		break;
	case Frame1:
		frame1 = dataIn;
		checksumtest = CheckSumFunction(CheckSum, Frame, CollectedData);
			if (frame1 == checksumtest)
			{
				UARTsuccess += 1;
				if (Mode == 2 || Mode == 3 || Mode == 12 || Mode == 13 || Mode == 14)
				{
					WriteACK1();
				}
				if (Mode == 8) // Go to Station N
				{
					WriteACK1();
				}
				if (Mode == 10) //Decimal 4 degree
				{
					WriteACK1();
					CurrentAngle1 = (int8_t)(Degree * 10000 * 3.14159265 / 256 /180) ;
					CurrentAngle2 = (int8_t)((int)(Degree* 10000 * 3.14159265 / 180) % 256) ;
					uint8_t temp[] = {CurrentAngle1, CurrentAngle2};
					UARTTxWrite(&UART2, temp, 2);
					n = 0;
					//read Ack in while loop
				}
				if (Mode == 11)
				{
					WriteACK1();
					uint8_t temp[] = {(int8_t)VmaxReal * 255 / 10};
					UARTTxWrite(&UART2, temp, 1) ;
					//send Vmax
					n = 0;
					//read Ack in while loop

				}
				if (Mode == 14)
				{
					StartSetHome = 1;
					SetHomeFlag = 0;
				}
				State = Idle;
			}
			else
			{
				Mode = 0;
				UARTerror += 1;
				State = Idle;
			}
		break;

	case Frame2_1:
		CollectedData = dataIn;
		State = Frame2_2;
		break;
	case Frame2_2:
		CollectedData2 = dataIn;
		State = CheckSum2;
		break;
	case CheckSum2:
		frame2 = dataIn;
		test = CheckSum + CollectedData + CollectedData2;//test
		checksumtest = CheckSumFunction(CheckSum, Frame, CollectedData + CollectedData2);
		if (frame2 == checksumtest)
		{
			UARTsuccess += 1;
			if (Mode == 4)
			{
				VmaxRPM = (double)CollectedData2/255*10 ;
				WriteACK1();
			}
			if (Mode == 5)
			{
				FinalPos = (((double)CollectedData * 256) + ((double)CollectedData2))/10000/3.14159265*180;
				WriteACK1();
			}

			State = Idle;
		}
		else
		{
			UARTerror += 1;
			State = Idle;
		}

		break;
	case Frame3_n:
		n_Station = dataIn;
		State = Frame3_station;
		break;
	case Frame3_station:
		if (n_Station > 0)
		{
			Station[via_point] = dataIn&0xf;
			via_point++;
			n_Station--;
		}
		if (n_Station==0)
		{
			State = Idle;
			break;
		}
		if (n_Station>0)
		{
			Station[via_point] = (dataIn&0xf0)>>4;
			via_point++;
			n_Station--;
			State = Frame3_station;
		}
		if (n_Station==0)
		{
			State = Idle;
			break;
		}
		break;
	}
}

int16_t CheckSumFunction(uint8_t CheckSum, uint8_t Frame, uint8_t Data)
{
	uint16_t result = 0;
	if (Frame == 1)
	{
		result = ~(CheckSum);
	}
	if (Frame == 2)
	{
		result = ~((CheckSum)+Data);
	}
	if (Frame == 3)
	{
		result = ~((CheckSum)+Data);
	}
	return result;
}

void WriteACK1()
{
	//*********write ACK1*****************
	uint8_t temp[] = {0x58, 0b01110101};
	UARTTxWrite(&UART2, temp, 2);
	//************************************
}
void WriteACK2()
{
	//*********write ACK2*****************
	uint8_t temp[] = {70, 0x6e};
	UARTTxWrite(&UART2, temp, 2);
	//************************************
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
			SetHomeFlag = 0;
			StartSetHome = 0;
		}
	}
}


void Trajec()
{
	float Vmax;
	Vmax = VmaxRPM * 0.10472 ; //rad per sec
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
		FinishedTraj = 1;
//		if (Mode == 5)
//		{
//			WriteACK1();
//		}
//		StartNstation -= 1 ;
//		movingTimestamp = micros();
//		FinishedStation++;



//		MovingState = 1;
//
//		if (StartNstation == -1)
//		{
//			StartMoving = 0;
//			StartNstation = 10;
//		}

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
	if (PWMPercent > 50000)
	{
		PWMPercent = 50000;
	}
	if (PWMPercent < 0)
	{
		PWMPercent = 0 ;
	}
	PreviousPWM = PWMPercent - InitialPWM ;
	preErr2 = preErr1 ;
	preErr1 = error ;

}

void SecondPID()
{
	PP = 3;
	II = 0.01;
	DD = 1;
	if (FinalPos - Degree < -350 && FinalPos > -360)
	{
		FinalPos = FinalPos + 360;
	}
	if (FinalPos - Degree > 350 && FinalPos <= 360)
	{
		FinalPos = FinalPos + 360;
	}
	if (FinalPos > Degree)
	{
		Direction = 0;
	}
	if (FinalPos < Degree)
	{
		Direction = 1;
	}
	float error = FinalPos - Degree;
	SUM = SUM + error ;
	PWMPercent = (PP*error) + (II*SUM) + (DD*(error - PPreerror)) ;
	PPreerror = error;
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
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50000);
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
