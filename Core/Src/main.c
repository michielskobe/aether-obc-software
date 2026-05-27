/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "data_packet.h"
#include "mosaic_x5.h"
#include "sd_spi.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
} can_rx_msg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RMU_STARTUP_FLAG      (1U << 0)  // 0x0001
#define GNSS_STARTUP_FLAG     (1U << 1)  // 0x0002
#define IRIDIUM_STARTUP_FLAG  (1U << 2)  // 0x0004
#define SD_CARD_STARTUP_FLAG  (1U << 3)  // 0x0008
#define DATA_ACQ_STARTUP_FLAG (1U << 4)  // 0x0010
#define LO_FLAG               (1U << 5)  // 0x0020
#define SODS_FLAG             (1U << 6)  // 0x0040
#define SOE_FLAG              (1U << 7)  // 0x0080
#define RMU_SHUTDOWN_FLAG     (1U << 8)  // 0x0100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* Definitions for SystemManager */
osThreadId_t SystemManagerHandle;
const osThreadAttr_t SystemManager_attributes = {
  .name = "SystemManager",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for RMUManager */
osThreadId_t RMUManagerHandle;
const osThreadAttr_t RMUManager_attributes = {
  .name = "RMUManager",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for GNSSManager */
osThreadId_t GNSSManagerHandle;
const osThreadAttr_t GNSSManager_attributes = {
  .name = "GNSSManager",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataAcquisition */
osThreadId_t DataAcquisitionHandle;
const osThreadAttr_t DataAcquisition_attributes = {
  .name = "DataAcquisition",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SDCardManager */
osThreadId_t SDCardManagerHandle;
const osThreadAttr_t SDCardManager_attributes = {
  .name = "SDCardManager",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IridiumManager */
osThreadId_t IridiumManagerHandle;
const osThreadAttr_t IridiumManager_attributes = {
  .name = "IridiumManager",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SD_CardQueue */
osMessageQueueId_t SD_CardQueueHandle;
const osMessageQueueAttr_t SD_CardQueue_attributes = {
  .name = "SD_CardQueue"
};
/* Definitions for IridiumQueue */
osMessageQueueId_t IridiumQueueHandle;
const osMessageQueueAttr_t IridiumQueue_attributes = {
  .name = "IridiumQueue"
};
/* Definitions for CAN_RxQueue */
osMessageQueueId_t CAN_RxQueueHandle;
const osMessageQueueAttr_t CAN_RxQueue_attributes = {
  .name = "CAN_RxQueue"
};
/* USER CODE BEGIN PV */
static uint8_t sd_block[SD_BLOCK_SIZE];
static uint16_t block_index = 0;
static uint32_t current_block_addr = 3; // Start writing after the reserved blocks (0-2) on the SD card
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_CRC_Init(void);
void StartSystemManager(void *argument);
void StartRMUManager(void *argument);
void StartGNSSManager(void *argument);
void StartDataAcquisition(void *argument);
void StartSDCardManager(void *argument);
void StartIridiumManager(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void SendCANCommand(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef header = {
        .IDE   = CAN_ID_STD,
        .StdId = id,
        .RTR   = CAN_RTR_DATA,
        .DLC   = dlc
    };

    uint32_t mailbox;

    if (HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)data, &mailbox) != HAL_OK)
    {
        Error_Handler();
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  __HAL_DMA_ENABLE_IT(&hdma_uart5_rx, DMA_IT_TE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SD_CardQueue */
  SD_CardQueueHandle = osMessageQueueNew (128, sizeof(data_packet_t), &SD_CardQueue_attributes);

  /* creation of IridiumQueue */
  IridiumQueueHandle = osMessageQueueNew (128, sizeof(data_packet_t), &IridiumQueue_attributes);

  /* creation of CAN_RxQueue */
  CAN_RxQueueHandle = osMessageQueueNew (30, sizeof(can_rx_msg_t), &CAN_RxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SystemManager */
  SystemManagerHandle = osThreadNew(StartSystemManager, NULL, &SystemManager_attributes);

  /* creation of RMUManager */
  RMUManagerHandle = osThreadNew(StartRMUManager, NULL, &RMUManager_attributes);

  /* creation of GNSSManager */
  GNSSManagerHandle = osThreadNew(StartGNSSManager, NULL, &GNSSManager_attributes);

  /* creation of DataAcquisition */
  DataAcquisitionHandle = osThreadNew(StartDataAcquisition, NULL, &DataAcquisition_attributes);

  /* creation of SDCardManager */
  SDCardManagerHandle = osThreadNew(StartSDCardManager, NULL, &SDCardManager_attributes);

  /* creation of IridiumManager */
  IridiumManagerHandle = osThreadNew(StartIridiumManager, NULL, &IridiumManager_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hcan1.Init.Prescaler = 40;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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

  // Configure CAN filter to receive messages
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank           = 0;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;

  // Target ID: 0x000, shifted left by 5 into the register
  filter.FilterIdHigh         = (0x000 << 5);   // 0x0000
  filter.FilterIdLow          = 0x0000;

  // Mask: only check the top 3 bits of the 11-bit ID (bits 10–8)
  // 0x700 << 5 = 0xE000 -> forces bits 10/9/8 to match (must be 0)
  filter.FilterMaskIdHigh     = (0x700 << 5);   // 0xE000
  filter.FilterMaskIdLow      = 0x0000;

  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation     = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
      Error_Handler();
  }

  // Start the CAN peripheral to begin receiving messages
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  Error_Handler();
  }

  // Activate CAN RX interrupt
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 4129;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 0;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 38400;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 19200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHDN_Pin|PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IR_ON_OFF_GPIO_Port, IR_ON_OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RXSM_SODS_Pin RXSM_SOE_Pin RXSM_LO_Pin */
  GPIO_InitStruct.Pin = RXSM_SODS_Pin|RXSM_SOE_Pin|RXSM_LO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PGOOD_Pin */
  GPIO_InitStruct.Pin = PGOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PGOOD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHDN_Pin PWR_EN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin|PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NA_Pin */
  GPIO_InitStruct.Pin = NA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_ON_OFF_Pin */
  GPIO_InitStruct.Pin = IR_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IR_ON_OFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RI_Pin */
  GPIO_InitStruct.Pin = RI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RI_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  This function handles CAN RX FIFO 0 message pending callback.
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *                the configuration information for the specified CAN.
  * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  // Read the CAN message from the hardware FIFO and put it into the CAN_RxQueue for processing by the DataAcquisition thread
  can_rx_msg_t msg;

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg.RxHeader, msg.RxData) != HAL_OK)
    {
      break;
    }

    osMessageQueuePut(CAN_RxQueueHandle, &msg, 0U, 0U);
  }
}

/**
  * @brief  This function handles UART RX event callback for UART5 (Mosaic-X5).
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART.
  * @param  size: number of bytes received in the current UART RX event.
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size){
  if (huart->Instance == UART5) {
    mosaic_uart_rx_cb(size);
  }
}

/**
  * @brief  This function handles UART error callback for UART5 (Mosaic-X5).
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        // Clear error flags by resetting UART reception
        __HAL_UART_CLEAR_OREFLAG(huart);

        HAL_UART_DMAStop(huart);

        mosaic_x5_init(); // Re-initialize the mosaic-X5 UART reception
    }
}

/** 
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin: Specifies the pins connected to corresponding EXTI line
 * @retval None
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case RXSM_LO_Pin:
      osThreadFlagsSet(SystemManagerHandle, LO_FLAG);
      break;
    case RXSM_SODS_Pin:
      osThreadFlagsSet(SystemManagerHandle, SODS_FLAG);
      break;
    case RXSM_SOE_Pin:
      osThreadFlagsSet(SystemManagerHandle, SOE_FLAG);
      break;
    default:
      break;
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSystemManager */
/**
  * @brief  Function implementing the SystemManager thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSystemManager */
void StartSystemManager(void *argument)
{
  /* USER CODE BEGIN 5 */

  // Signal the GNSSManager to start up by setting the GNSS_STARTUP_FLAG
  osThreadFlagsSet(GNSSManagerHandle, GNSS_STARTUP_FLAG);

  // Signal the SDCardManager to start up by setting the SD_CARD_STARTUP_FLAG
  osThreadFlagsSet(SDCardManagerHandle, SD_CARD_STARTUP_FLAG);

  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRMUManager */
/**
* @brief Function implementing the RMUManager thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRMUManager */
void StartRMUManager(void *argument)
{
  /* USER CODE BEGIN StartRMUManager */
  // Wait for SystemManager to start up and set the RMU_STARTUP_FLAG before proceeding
  osThreadFlagsWait(RMU_STARTUP_FLAG, osFlagsWaitAny, osWaitForever);
  /* Infinite loop */
  for(;;)
  {
    // Check for the RMU_SHUTDOWN_FLAG to know when to terminate the RMU Manager task (non-blocking)
    uint32_t flags = osThreadFlagsWait(RMU_SHUTDOWN_FLAG, osFlagsWaitAny, 0);
    
    // If the RMU_SHUTDOWN_FLAG is set, break out of the loop and terminate the RMU Manager task
    if ((int32_t)flags >= 0 && (flags & RMU_SHUTDOWN_FLAG))
    {
      break;
    }

    osDelay(1);
  }

  // Terminate RMU Manager Task
  osThreadExit();

  /* USER CODE END StartRMUManager */
}

/* USER CODE BEGIN Header_StartGNSSManager */
/**
* @brief Function implementing the GNSSManager thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGNSSManager */
void StartGNSSManager(void *argument)
{
  /* USER CODE BEGIN StartGNSSManager */

  // Wait for SystemManager to start up and set the GNSS_STARTUP_FLAG before proceeding
  osThreadFlagsWait(GNSS_STARTUP_FLAG, osFlagsWaitAny, osWaitForever);
  
  // Initialize the mosaic-X5 GNSS receiver
  while (mosaic_x5_init() != 0){
    osDelay(1);
  }
  
  /* Infinite loop */
  for(;;)
  {
    // Wait for and handle GNSS data
    gnss_data_handler();
    osDelay(1);
  }
  /* USER CODE END StartGNSSManager */
}

/* USER CODE BEGIN Header_StartDataAcquisition */
/**
* @brief Function implementing the DataAcquisition thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataAcquisition */
void StartDataAcquisition(void *argument)
{
  /* USER CODE BEGIN StartDataAcquisition */
  // Wait for SystemManager to start up and set the DATA_ACQ_STARTUP_FLAG before proceeding
  osThreadFlagsWait(DATA_ACQ_STARTUP_FLAG, osFlagsWaitAny, osWaitForever);

  can_rx_msg_t rx_msg;
  data_packet_t entry;
  
  /* Infinite loop */
  for(;;)
  {
    // Wait for a CAN message to be received and put into the CAN_RxQueue by the RX ISR
    if (osMessageQueueGet(CAN_RxQueueHandle, &rx_msg, NULL, osWaitForever) == osOK)
        {
          // Get the current tick count to use as a timestamp for the data packet
          uint32_t tick = HAL_GetTick();

          entry.timestamp[0] = (tick >> 16) & 0xFF;
          entry.timestamp[1] = (tick >> 8)  & 0xFF;
          entry.timestamp[2] = tick & 0xFF;

          // Use the lower 8 bits of the CAN message's standard ID as the sensor ID for this data packet
          entry.id = (uint8_t)rx_msg.RxHeader.StdId;

          // Copy the first 4 bytes of the CAN message data into the data field of the sensor entry
          // TODO: implement a more robust way of handling different types of CAN messages with varying data lengths and formats, based on the sensor ID
          memcpy(entry.data, rx_msg.RxData, 4);

          // Put the sensor entry into both the SD card queue and the Iridium queue for processing by the respective manager threads
          osMessageQueuePut(SD_CardQueueHandle, &entry, 0, 0);
          osMessageQueuePut(IridiumQueueHandle, &entry, 0, 0);
        }
  }
  /* USER CODE END StartDataAcquisition */
}

/* USER CODE BEGIN Header_StartSDCardManager */
/**
* @brief Function implementing the SDCardManager thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSDCardManager */
void StartSDCardManager(void *argument)
{
  /* USER CODE BEGIN StartSDCardManager */ 

  // Initialize SD card
  while (sd_init() != 0) {
    // Initialization failed, retry after a delay
    osDelay(100);
  }

  // Wait for SystemManager to start up and set the SD_CARD_STARTUP_FLAG before proceeding
  osThreadFlagsWait(SD_CARD_STARTUP_FLAG, osFlagsWaitAny, osWaitForever);
  
  /* Infinite loop */
  data_packet_t entry;
  for(;;)
  {
    // Wait for next sensor entry
    osMessageQueueGet(SD_CardQueueHandle, &entry, NULL, osWaitForever);

    // Copy into block buffer
    memcpy(&sd_block[block_index], &entry, sizeof(data_packet_t));
    block_index += sizeof(data_packet_t);

    // If block full, write to SD
    if (block_index >= SD_BLOCK_SIZE) {
        if (sd_write_block(current_block_addr, sd_block) == 0) {
          current_block_addr++;
        }
        block_index = 0;
    }
  }
  /* USER CODE END StartSDCardManager */
}

/* USER CODE BEGIN Header_StartIridiumManager */
/**
* @brief Function implementing the IridiumManager thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIridiumManager */
void StartIridiumManager(void *argument)
{
  /* USER CODE BEGIN StartIridiumManager */
  // Wait for SystemManager to start up and set the IRIDIUM_STARTUP_FLAG before proceeding
  osThreadFlagsWait(IRIDIUM_STARTUP_FLAG, osFlagsWaitAny, osWaitForever);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartIridiumManager */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
