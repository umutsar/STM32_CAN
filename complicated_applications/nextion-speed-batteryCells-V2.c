/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t adc1, adc2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ************************* CAN CODES ***************************

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef receiver_structure;

uint16_t interrupt_counter;
uint8_t RxNextionEvent[4];

uint8_t Transmitted_Data[8];
uint32_t TxMailbox;


uint8_t RxData[8];
uint32_t sender_info[5];

uint8_t cell_1;
uint8_t cell_2;
uint8_t cell_3;
uint8_t cell_4;

uint8_t cell_5;
uint8_t cell_6;
uint8_t cell_7;
uint8_t cell_8;

uint8_t cell_9;
uint8_t cell_10;
uint8_t cell_11;
uint8_t cell_12;

uint8_t cell_13;
uint8_t cell_14;
// ************************* CAN CODES END***************************

// ************************* NEXTION FUNCTIONS ***************************
uint8_t Cmd_End[3] = {0xFF, 0xFF, 0xFF};
uint32_t speed_fixed;
uint32_t ledYandiMi;
//char bkcmd[7] = "bkcmd=1";

void Nextion_SendString(char * Id, char * String)
{
	char buf[50];
    int len = sprintf(buf, "%s.txt=\"%s\"",Id, String);

//	HAL_UART_Transmit(&huart1, (uint8_t *)bkcmd, 7, 1000);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 1000);
}

void NextionSendNumber(char *Id, int number)
{
    char buf[50];
    int len = sprintf(buf, "%s.val=%d", Id, number);

//    HAL_UART_Transmit(&huart1, (uint8_t *)bkcmd, 7, 1000);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
    HAL_UART_Transmit(&huart1, Cmd_End, 3, 1000);
}

void NextionSetVisibility(char *Id, int visibility)
{
    char buf[50];
    int len = sprintf(buf, "vis %s,%d", Id, visibility);

//    HAL_UART_Transmit(&huart1, (uint8_t *)bkcmd, 7, 1000);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
    HAL_UART_Transmit(&huart1, Cmd_End, 3, 1000);
}
// ************************* NEXTION FUNCTIONS END***************************

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(
		RxNextionEvent[0] == 36 &&
		RxNextionEvent[1] == 49 &&
		RxNextionEvent[2] == 26 &&
		RxNextionEvent[3] == 38
	  )
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
		ledYandiMi += 1;
	}

	if(
		RxNextionEvent[0] == 36 &&
		RxNextionEvent[1] == 49 &&
		RxNextionEvent[2] == 25 &&
		RxNextionEvent[3] == 38
	  )
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
	}

	HAL_UART_Receive_IT(&huart1, RxNextionEvent, 4);
	interrupt_counter += 1;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	speed_fixed = speed_fixed;
	ledYandiMi = ledYandiMi;
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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0X401;


	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan);
	HAL_Delay(100);


	HAL_UART_Receive_IT(&huart1, RxNextionEvent, 4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  ledYandiMi = ledYandiMi;
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  adc1 = HAL_ADC_GetValue(&hadc1);


  	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &receiver_structure, RxData);
  	  sender_info[0] = receiver_structure.StdId;
  	  sender_info[1] = receiver_structure.ExtId;
  	  sender_info[2] = receiver_structure.IDE;
  	  sender_info[3] = receiver_structure.RTR;
  	  sender_info[4] = receiver_structure.DLC;
  	  // receiver_structure.StdId == 880 &&
  	  if(RxData[0] == 1) {
  		  cell_1 = RxData[1];
  		  cell_2 = RxData[2];
  		  cell_3 = RxData[3];
  		  cell_4 = RxData[4];

  		  cell_5 = RxData[5];
  		  cell_6 = RxData[6];
  		  cell_7 = RxData[7];

  		for(int i = 0; i < 7; i++) {
  		    char buffer[10];  // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
  		    sprintf(buffer, "j%d", i + 1);  // "j1", "j2", ..., "j8" şeklinde metin oluştur
  		    NextionSendNumber(buffer, RxData[i + 1]);  // Oluşturulan metni ve RxData dizisindeki değeri gönder
  		}

  	  }

  	  else if(RxData[0] == 2){
  		  cell_8 = RxData[1];
  		  cell_9 = RxData[2];
  		  cell_10 = RxData[3];
  		  cell_11 = RxData[4];

  		  cell_12 = RxData[5];
  		  cell_13 = RxData[6];
  		  cell_14 = RxData[7];

  		for(int i = 0; i < 7; i++) {
			char buffer[10];  // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
			sprintf(buffer, "j%d", i + 8);  // "j1", "j2", ..., "j8" şeklinde metin oluştur
			NextionSendNumber(buffer, RxData[i + 1]);  // Oluşturulan metni ve RxData dizisindeki değeri gönder
		}
  	  }
		NextionSetVisibility("beniaksyebagla", 0);
		NextionSetVisibility("bataryauyari", 0);


		speed_fixed = ((adc1 * 80) / 4096);

		ledYandiMi = ledYandiMi;

		if(speed_fixed <= 60) {
			NextionSendNumber("n11", speed_fixed);
			NextionSendNumber("z0", (3 * (speed_fixed / 2)) + 270);
		}

		if(speed_fixed > 60) {
			NextionSendNumber("n11", speed_fixed);
			NextionSendNumber("z0", (3 * (speed_fixed / 2)) - 90);
		}

		NextionSendNumber("deneme", speed_fixed);


		HAL_UART_Transmit(&huart2, RxData, 8, 200);

		HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */


	CAN_FilterTypeDef can_filter_structure;

	can_filter_structure.FilterActivation = CAN_FILTER_ENABLE;
	can_filter_structure.FilterBank = 13;
	can_filter_structure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter_structure.FilterIdHigh = 0;
	can_filter_structure.FilterIdLow = 0;
	can_filter_structure.FilterMaskIdHigh = 0x0000;
	can_filter_structure.FilterMaskIdLow = 0x0000;
	can_filter_structure.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_structure.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_structure.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan, &can_filter_structure);



  /* USER CODE END CAN_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
