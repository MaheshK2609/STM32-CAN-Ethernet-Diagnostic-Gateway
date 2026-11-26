/* USER CODE BEGIN Header */
/**
  * Board A sender  Nucleo-F446RE   (CAN debug build)
  * BMP280 (I2C1) + DS18B20 (PB0) -> CAN1 (0x100 env, 0x101 DS18B20)
  * Watch the dbg... variables in MCUViewer (import this project's .elf).
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "ds18b20.h"
#include "can_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef LD2_Pin
#define LD2_Pin        GPIO_PIN_5
#define LD2_GPIO_Port  GPIOA
#endif

#define SENDER_TEST_FRAME  1   /* 1 = send a fixed frame 0x100 every second, no sensors involved. LD2 toggles only while frames are acknowledged */
#define CAN_LOOPBACK_TEST  0   /* 1 = internal loopback (bus not needed): frames loop back inside the chip, watch dbgLoopRx rise */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

BMP280_t  bmp;     /* BMP280 driver instance (watch bmp.ok, bmp.temp_x100, bmp.press_pa) */
DS18B20_t ds;      /* DS18B20 driver instance (watch ds.ok, ds.temp_x100, ds.cntCrc ...) */

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;

/* CAN debug variables for MCUViewer (it can only show global variables) */
volatile uint32_t dbgEsr = 0, dbgLec = 0, dbgLecLatched = 0, dbgTec = 0, dbgRec = 0, dbgBusOff = 0;
volatile uint32_t dbgFree = 3, dbgSent = 0, dbgSkipped = 0, dbgLoopRx = 0, dbgState = 0;

static uint8_t CAN_Send(uint16_t id, const uint8_t *data, uint8_t dlc)
{
  TxHeader.StdId = id;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = dlc;
  TxHeader.TransmitGlobalTime = DISABLE;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) { dbgSkipped++; return 0; }   /* all 3 mailboxes stuck: nobody acknowledges */
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t *)data, &TxMailbox) != HAL_OK) { dbgSkipped++; return 0; }
  dbgSent++;
  return 1;
}

static void Dbg_Update(void)
{
  uint32_t e = CAN1->ESR;
  dbgEsr    = e;
  dbgLec    = (e >> 4) & 7U;                 /* last error code: 0 none, 3 no acknowledgement, 4/5 bit error, 6 CRC */
  if (dbgLec != 0U) dbgLecLatched = dbgLec;
  dbgTec    = (e >> 16) & 0xFFU;             /* transmit error counter */
  dbgRec    = (e >> 24) & 0xFFU;             /* receive error counter */
  dbgBusOff = (e >> 2) & 1U;                 /* 1 = bus-off: this board stopped sending until reset */
  dbgFree   = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);   /* 3 = nothing waiting, 0 = all stuck */
  dbgState  = (uint32_t)HAL_CAN_GetState(&hcan1);
#if CAN_LOOPBACK_TEST
  CAN_RxHeaderTypeDef rh;
  uint8_t rd[8];
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rh, rd) == HAL_OK) dbgLoopRx++; else break;
  }
#endif
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
  MX_I2C1_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();   /* fast LD2 blinking = CAN failed to start */
  BMP280_Init(&bmp, &hi2c1);
  DS18B20_Init(&ds, GPIOB, GPIO_PIN_0, &htim3);

  if (!bmp.ok)
  {
    for (int i = 0; i < 10; i++)          /* 10 fast blinks = BMP280 not found */
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(100);
    }
  }

  uint32_t last = 0, lastTest = 0;
  uint8_t tick = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Dbg_Update();

#if SENDER_TEST_FRAME
	  if (HAL_GetTick() - lastTest >= 1000)
	  {
	    lastTest += 1000;
	    uint8_t acked = (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3U);   /* 3 = the earlier frames were acknowledged */
	    uint8_t d[VEH_CAN_DLC_ENV];
	    veh_pack_env(d, 2500, 94000);                                      /* 25.00 C, 940.00 hPa */
	    CAN_Send(VEH_CAN_ID_ENV, d, VEH_CAN_DLC_ENV);
	    if (acked) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);                  /* LD2 only toggles while frames get acknowledged */
	  }
#endif

	  if (HAL_GetTick() - last >= 2000)
	  {
	    last += 2000;                       /* fixed rate, no drift */
	    tick ^= 1;

	    if (bmp.ok && BMP280_Read(&bmp))
	    {
	      uint8_t acked = (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3U);
	      uint8_t d[VEH_CAN_DLC_ENV];
	      veh_pack_env(d, bmp.temp_x100, bmp.press_pa);
	      CAN_Send(VEH_CAN_ID_ENV, d, VEH_CAN_DLC_ENV);   /* 0x100 every 2 s */
#if !SENDER_TEST_FRAME
	      if (acked) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
#else
	      (void)acked;
#endif
	    }
	    if (tick == 0 && ds.ok)
	    {
	      uint8_t d[VEH_CAN_DLC_DS];
	      veh_pack_ds(d, ds.temp_x100);
	      CAN_Send(VEH_CAN_ID_DS, d, VEH_CAN_DLC_DS);     /* 0x101 every 4 s */
	    }
	  }

	  DS18B20_Service(&ds);
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
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

#if CAN_LOOPBACK_TEST
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;                   // CAN_RX pin is ignored, frames loop back inside the chip
  if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();   // re-initialise with the new mode (CAN is not started yet)
#endif

  CAN_FilterTypeDef canfilterconfig = {0};

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
#if CAN_LOOPBACK_TEST
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;             // accept every ID, so the looped-back frames are counted
#else
  canfilterconfig.FilterIdHigh = 0x103<<5;
  canfilterconfig.FilterMaskIdHigh = 0x103<<5;
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
  hi2c1.Init.ClockSpeed = 100000;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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
  /* Fast LD2 blinking forever = the program hit an error (clock, CAN or I2C init failed). */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    for (volatile uint32_t i = 0; i < 400000U; i++) { }
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
