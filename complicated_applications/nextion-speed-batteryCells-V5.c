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
#include <stdbool.h>
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
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
#define RxNextionBufferSize 20
uint8_t RxNextionBuffer[RxNextionBufferSize];

uint32_t adc1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void synchronization();

// ************************************** RING BUFFER CODES ********************************************

#define BufferSize 750

typedef struct
{
    int buffer[BufferSize];
    int readNumber;
    int writeIndex;
    int readIndex;
} RingBuffer;

void initRingBuffer(RingBuffer *newBuffer)
{
    newBuffer->writeIndex = 0;
    newBuffer->readIndex = 0;
}

int counter = 10;
void write(RingBuffer *newBuffer, int readNumberParam)
{
    if ((newBuffer->writeIndex + 1) % BufferSize != newBuffer->readIndex)
    {
        newBuffer->buffer[newBuffer->writeIndex] = readNumberParam;
        newBuffer->writeIndex = (newBuffer->writeIndex + 1) % BufferSize;
    }
    else
    {
        printf("Buffer is FULL!\n");
    }
}

int read(RingBuffer *newBuffer)
{
    newBuffer->readNumber = -1;

    if (newBuffer->readIndex != newBuffer->writeIndex)
    {
        newBuffer->readNumber = newBuffer->buffer[newBuffer->readIndex];
        newBuffer->readIndex = (newBuffer->readIndex + 1) % BufferSize;
    }
    else
    {
        printf("Buffer is EMPTY!\n");
    }

    return newBuffer->readNumber;
}
int deger;
uint8_t gecici[19] = {0, 3, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Sürekli değişecek buffer.
uint8_t loraSendData[19];
uint32_t getClock;

bool flag = 0;
bool synchronizationFlag = 0;
uint32_t sonGonderilmeSaati;

// ************************************** RING BUFFER CODES ********************************************

// ************************* CAN CODES ***************************

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef receiver_structure;

uint16_t interrupt_counter;

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
uint8_t cell_15;
uint8_t cell_16;

uint8_t cell_17;
uint8_t cell_18;
uint8_t cell_19;
uint8_t cell_20;
// ************************* CAN CODES END***************************

// ************************* NEXTION FUNCTIONS ***************************
uint8_t Cmd_End[3] = {0xFF, 0xFF, 0xFF};
uint32_t speed_fixed;

uint8_t bigBufferLora[4000] = {0};
uint16_t currentIndex = 0;

// Ayarlar Sayfasındaki Sürgülerin konumu. (0 veya 1)
bool sw0 = 1;
bool sw1 = 1;
bool sw2 = 1;
bool sw3 = 1;

bool farAcikmi = 0;

bool VeriUlastiFlag = 0;
bool bms1_PacketNumber = 0;
bool bms2_PacketNumber = 0;

bool veriBiriktimi = 0;

uint8_t requestBmsPacket[8] = {1, 1, 1, 1, 1, 1, 1, 1};
// uint8_t loraKontrolVerisiGonder[8] = {0,0,18, 255, 0, 255, 0, 255};
uint8_t loraGelenVeri[12];
uint8_t ornekVeri[28] = {0, 1, 18,
                         30, 20, 60, 100, 45,
                         12, 43, 23, 23, 23,
                         43, 76, 53, 75, 45,
                         54, 65, 100, 90, 45,
                         76, 23, 98, 43, 25};

void NextionSendString(char *Id, char *String)
{
    char buf[50];
    int len = sprintf(buf, "%s.txt=\"%s\"", Id, String);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
    HAL_UART_Transmit(&huart1, Cmd_End, 3, 1000);
}

void NextionSendNumber(char *Id, int number)
{
    char buf[50];
    int len = sprintf(buf, "%s.val=%d", Id, number);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
    HAL_UART_Transmit(&huart1, Cmd_End, 3, 1000);
}

void NextionSetVisibility(char *Id, int visibility)
{
    char buf[50];
    int len = sprintf(buf, "vis %s,%d", Id, visibility);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
    HAL_UART_Transmit(&huart1, Cmd_End, 3, 1000);
}
// ************************* NEXTION FUNCTIONS END***************************
uint16_t veriAkisHiziDelay = 650;
uint32_t anlikSure = 0;
uint16_t ekranKoordinatlariX_Y[2];

void EkranKonumunuAl()
{
    ekranKoordinatlariX_Y[0] = (RxNextionBuffer[1] * 256) + RxNextionBuffer[2];
    ekranKoordinatlariX_Y[1] = (RxNextionBuffer[3] * 256) + RxNextionBuffer[4];
}

// ***************************** UART DMA CALLBACK *********************************
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {

        if (RxNextionBuffer[0] == 103)
        {
            EkranKonumunuAl();
        }
        // Farı aç
        if (strncmp((const char *)RxNextionBuffer, "LediYak", strlen("LediYak")) == 0)
        {
            if (farAcikmi == 0)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
                farAcikmi = 1;
            }
        }

        // Farı kapat
        if (strncmp((const char *)RxNextionBuffer, "LediSondur", strlen("LediSondur")) == 0)
        {
            if (farAcikmi == 1)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
                farAcikmi = 0;
            }
        }

        // Veri Akış hızı Hızlı/Yavaş
        if (strncmp((const char *)RxNextionBuffer, "VeriAkisHizi", strlen("VeriAkisHizi")) == 0)
        {
            if (sw2 == 1)
            {
                sw2 = 0;
                veriAkisHiziDelay = 1000;
            }
            else
            {
                sw2 = 1;
                veriAkisHiziDelay = 20;
            }
        }

        // Batarya Verileri Açık/Kapalı
        if (strncmp((const char *)RxNextionBuffer, "BataryaVerileri", strlen("BataryaVerileri")) == 0)
        {
            if (sw1 == 1)
            {
                sw1 = 0;
            }
            else
            {
                sw1 = 1;
            }
        }

        // Loraya Giden Veri Açık/Kapalı
        if (strncmp((const char *)RxNextionBuffer, "YerIstasyonuVerisi", strlen("YerIstasyonuVerisi")) == 0)
        {
            if (sw3 == 1)
            {
                sw3 = 0;
                NextionSendNumber("sw3", 0);
            }
            else
            {
                sw3 = 1;
                NextionSendNumber("sw3", 1);
            }
        }

        // Hız Göstergesi Açık/Kapalı
        if (strncmp((const char *)RxNextionBuffer, "HizGostergesi", strlen("HizGostergesi")) == 0)
        {
            if (sw0 == 1)
            {
                sw0 = 0;
                NextionSendNumber("sw0", 0);
                NextionSendNumber("n11", 0);
                NextionSendNumber("z0", 270);
            }
            else
            {
                sw0 = 1;
                NextionSendNumber("sw0", 1);
            }
        }
        interrupt_counter += 1;

        for (int i = Size; i < RxNextionBufferSize; i++)
        {
            RxNextionBuffer[i] = 0;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxNextionBuffer, sizeof(RxNextionBuffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxNextionBuffer, sizeof(RxNextionBuffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }

    if (huart == &huart2)
    {
        if (RxNextionBuffer[0] == 12 && RxNextionBuffer[1] == 12 &&
            RxNextionBuffer[2] == 12 && RxNextionBuffer[3] == 12)
        {
            flag = 1;
            getClock = HAL_GetTick();
            synchronizationFlag = 1;
        }

        for (int i = Size; i < RxNextionBufferSize; i++)
        {
            RxNextionBuffer[i] = 0;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxNextionBuffer, sizeof(RxNextionBuffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxNextionBuffer, sizeof(RxNextionBuffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    RingBuffer newBuffer1;
    initRingBuffer(&newBuffer1);

    void synchronization()
    {
        __disable_irq(); // Kesmeleri devre dışı bırak
        for (int i = 0; i < 19; i++)
        {
            deger = read(&newBuffer1);
            if (deger != -1)
            {
                gecici[i] = deger;
            }
        }
        __enable_irq();
    }

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    HAL_CAN_Start(&hcan);
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_CAN_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */
    //	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxNextionBuffer, RxNextionBufferSize);
    //	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxNextionBuffer, sizeof(RxNextionBuffer));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxNextionBuffer, sizeof(RxNextionBuffer));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0X401;

    loraSendData[0] = 0;
    loraSendData[1] = 3;
    loraSendData[2] = 18;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // uint32_t currentTime = HAL_GetTick(); // Anlık süreyi alma kodu
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &receiver_structure, RxData);
        if (sw0)
        {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 100);
            adc1 = HAL_ADC_GetValue(&hadc1);
            speed_fixed = ((adc1 * 80) / 4096);
        }

        sender_info[0] = receiver_structure.StdId;
        sender_info[1] = receiver_structure.ExtId;
        sender_info[2] = receiver_structure.IDE;
        sender_info[3] = receiver_structure.RTR;
        sender_info[4] = receiver_structure.DLC;

        if (sw1)
        {
            if (RxData[0] == 1)
            {
                for (int i = 0; i < 5; i++)
                {
                    char buffer[10]; // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
                    sprintf(buffer, "j%d", i + 1);
                    NextionSendNumber(buffer, RxData[i + 1]);
                    NextionSendNumber("status", 401);
                }
            }

            else if (RxData[0] == 2)
            {
                for (int i = 0; i < 5; i++)
                {
                    char buffer[10];                          // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
                    sprintf(buffer, "j%d", i + 6);            // "j1", "j2", ..., "j8" şeklinde metin oluştur
                    NextionSendNumber(buffer, RxData[i + 1]); // Oluşturulan metni ve RxData dizisindeki değeri gönder
                    NextionSendNumber("status", 402);
                }
            }

            else if (RxData[0] == 3)
            {
                for (int i = 0; i < 5; i++)
                {
                    char buffer[10];                          // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
                    sprintf(buffer, "j%d", i + 11);           // "j1", "j2", ..., "j8" şeklinde metin oluştur
                    NextionSendNumber(buffer, RxData[i + 1]); // Oluşturulan metni ve RxData dizisindeki değeri gönder
                    NextionSendNumber("status", 403);
                }
            }

            else if (RxData[0] == 4)
            {
                for (int i = 0; i < 5; i++)
                {
                    char buffer[10];                          // Bir sayıyı metne dönüştürmek için kullanılacak bir tampon
                    sprintf(buffer, "j%d", i + 16);           // "j1", "j2", ..., "j8" şeklinde metin oluştur
                    NextionSendNumber(buffer, RxData[i + 1]); // Oluşturulan metni ve RxData dizisindeki değeri gönder
                    NextionSendNumber("status", 404);
                }
            }
        }

        NextionSetVisibility("beniaksyebagla", 0);
        NextionSetVisibility("bataryauyari", 0);
        if (sw0)
        {
            if (speed_fixed <= 60)
            {
                NextionSendNumber("n11", speed_fixed);
                NextionSendNumber("z0", (3 * (speed_fixed / 2)) + 270);
            }

            if (speed_fixed > 60)
            {
                NextionSendNumber("n11", speed_fixed);
                NextionSendNumber("z0", (3 * (speed_fixed / 2)) - 90);
            }
        }

        if (sw3)
        {
            loraSendData[3] = RxData[0];
            loraSendData[4] = 0;
            loraSendData[5] = RxData[1];
            loraSendData[6] = 0;
            loraSendData[7] = RxData[2];
            loraSendData[8] = 0;
            loraSendData[9] = RxData[3];
            loraSendData[10] = 0;

            loraSendData[11] = RxData[4];
            loraSendData[12] = 0;
            loraSendData[13] = RxData[5];
            loraSendData[14] = 0;
            loraSendData[15] = RxData[6];
            loraSendData[16] = 0;
            loraSendData[17] = RxData[7];
            loraSendData[18] = 0;
        }

        HAL_Delay(veriAkisHiziDelay);

        /* ********************
          VeriUlastiFlag = 0;

          HAL_UART_Transmit(&huart2, requestBmsPacket, 1, 250);
          HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &receiver_structure, RxData);
          requestBmsPacket[0] = 1;
          HAL_Delay(100);

          HAL_UART_Transmit(&huart2, requestBmsPacket, 1, 250);
          HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &receiver_structure, RxData);
          requestBmsPacket[0] = 2;
          HAL_Delay(100);
        ********************* */

        // Yeni Kodlar

        for (int i = 0; i < sizeof(loraSendData); i++)
        {
            __disable_irq(); // Kesmeleri devre dışı bırak
            write(&newBuffer1, loraSendData[i]);
            __enable_irq();
        }

        if (HAL_GetTick() - sonGonderilmeSaati > 5000 && flag == 0)
        {
            // Yeniden bağlantı sağlandı mı diye kontrol et.
            HAL_UART_Transmit(&huart2, gecici, sizeof(gecici), 1000);
            sonGonderilmeSaati = HAL_GetTick();
            flag = 0;
        }

        // Birinci Mesaj Gönderme
        if (synchronizationFlag == 1)
        {
            synchronization();
            HAL_UART_Transmit(&huart2, gecici, sizeof(gecici), 1000);
            sonGonderilmeSaati = HAL_GetTick();
            HAL_Delay(350);

            synchronization();
            HAL_UART_Transmit(&huart2, gecici, sizeof(gecici), 1000);
            sonGonderilmeSaati = HAL_GetTick();
            HAL_Delay(350);

            synchronization();
            HAL_UART_Transmit(&huart2, gecici, sizeof(gecici), 1000);
            sonGonderilmeSaati = HAL_GetTick();
            HAL_Delay(350);

            flag = 0;
            synchronizationFlag = 0;
        }
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    /* DMA1_Channel7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
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
