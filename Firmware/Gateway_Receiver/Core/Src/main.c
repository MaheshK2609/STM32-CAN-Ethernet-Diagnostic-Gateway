/* USER CODE BEGIN Header */
/**
  * Receiver  Nucleo-F446RE   (CAN only, no Ethernet)
  * CAN in -> UART (Tera Term). Prints every frame it hears plus the CAN error counters.
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACCEPT_ALL_IDS  1      /* 1 = accept every CAN ID (for debugging). 0 = only 0x100 and 0x101 */
#define TX_SELF_TEST    1      /* 1 = this board also sends frame 0x7AA every second, to test its own CAN transceiver. Set to 0 when done */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

volatile int16_t temp_x100 = 0;
volatile uint32_t press_pa = 0;
volatile int32_t ds_x100 = 0;

volatile uint8_t dsValid = 0;
volatile uint32_t rxCount = 0;
volatile uint32_t lastRxTick = 0;
volatile uint32_t lastDsTick = 0;

/* CAN debug variables for MCUViewer (it can only show global variables) */
volatile uint32_t dbgEsr = 0, dbgLec = 0, dbgLecLatched = 0, dbgTec = 0, dbgRec = 0, dbgBusOff = 0, dbgFifo0 = 0, dbgState = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Led_Blink(uint8_t times, uint32_t ms)
{
  for (uint8_t i = 0; i < times; i++)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(ms);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(ms);
  }
}

/* RING BEGIN: the interrupt stores raw frames here, the main loop prints them */
typedef struct { uint32_t id; uint8_t dlc; uint8_t d[8]; } Frame_t;
static Frame_t ring[8];
static volatile uint8_t rHead = 0, rTail = 0;

static void Ring_Push(uint32_t id, uint8_t dlc, const uint8_t *d)
{
  Frame_t *f = &ring[rHead];
  f->id = id;
  f->dlc = dlc;
  for (int i = 0; i < 8; i++) f->d[i] = d[i];
  rHead = (uint8_t)((rHead + 1U) & 7U);
}

/* RING END */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) break;

    rxCount++;
    lastRxTick = HAL_GetTick();
    Ring_Push((RxHeader.IDE == CAN_ID_STD) ? RxHeader.StdId : RxHeader.ExtId, (uint8_t)RxHeader.DLC, RxData);

    if (RxHeader.StdId == 0x100 && RxHeader.DLC == 6)
    {
      temp_x100 = (int16_t)(RxData[0] | (RxData[1] << 8));
      press_pa = (uint32_t)RxData[2]
               | ((uint32_t)RxData[3] << 8)
               | ((uint32_t)RxData[4] << 16)
               | ((uint32_t)RxData[5] << 24);
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);     // LED follows the sender's BMP frames
    }
    else if (RxHeader.StdId == 0x101 && RxHeader.DLC == 2)
    {
      ds_x100 = (int16_t)(RxData[0] | (RxData[1] << 8));
      dsValid = 1;
      lastDsTick = HAL_GetTick();
    }
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  Led_Blink(3, 150);                                    // 3 blinks = firmware is running

  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler(); // fast LD2 blinking = CAN failed to start
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();

  static const char hello[] = "\r\nCAN receiver ready (500 kbps). Waiting for frames...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)hello, sizeof(hello) - 1, 100);

  uint32_t lastPrint = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if TX_SELF_TEST
    static uint32_t lastTx = 0;
    if (HAL_GetTick() - lastTx >= 1000)
    {
      lastTx = HAL_GetTick();
      CAN_TxHeaderTypeDef h = {0};
      uint8_t d[2] = {0xAB, 0xCD};
      uint32_t mb;
      h.StdId = 0x7AA; h.IDE = CAN_ID_STD; h.RTR = CAN_RTR_DATA; h.DLC = 2;
      HAL_CAN_AddTxMessage(&hcan1, &h, d, &mb);         // watch tec / lec in the CAN line below
    }
#endif

    if (HAL_GetTick() - lastPrint >= 2000)
    {
      lastPrint = HAL_GetTick();
      uint32_t now = HAL_GetTick();

      if ((now - lastDsTick) > 5000U) dsValid = 0;       // DS18B20 data went stale

      char msg[128];
      int len;

      if (rxCount == 0 || (now - lastRxTick) > 5000U)
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // LED off = no CAN data
        len = snprintf(msg, sizeof(msg), "No CAN data\r\n");
      }
      else
      {
        char dsStr[20];

        int32_t t = temp_x100;
        const char *s1 = "";
        if (t < 0) { s1 = "-"; t = -t; }

        int32_t d = ds_x100;
        const char *s2 = "";
        if (d < 0) { s2 = "-"; d = -d; }

        if (dsValid)
          snprintf(dsStr, sizeof(dsStr), "%s%ld.%02ld C", s2, (long)(d / 100), (long)(d % 100));
        else
          snprintf(dsStr, sizeof(dsStr), "--");

        len = snprintf(msg, sizeof(msg),
            "BMP280: %s%ld.%02ld C, %lu.%02lu hPa | DS18B20: %s\r\n",
            s1, (long)(t / 100), (long)(t % 100),
            (unsigned long)(press_pa / 100), (unsigned long)(press_pa % 100),
            dsStr);
      }
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, 100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig = {0};

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
#if ACCEPT_ALL_IDS
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;            // mask 0 = accept every ID
#else
  canfilterconfig.FilterIdHigh = 0x100<<5;
  canfilterconfig.FilterMaskIdHigh = 0x7FE<<5;          // accepts 0x100 and 0x101
#endif
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 20;

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, W5500_CS_Pin|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA5 W5500_RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|W5500_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_CS_Pin */
  GPIO_InitStruct.Pin = W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(W5500_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
  /* Fast LD2 blinking forever = the program hit an error (clock, CAN, SPI or UART init failed). */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    for (volatile uint32_t i = 0; i < 400000U; i++) { }
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
