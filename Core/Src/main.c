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
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "can.h"
#include "cmsis_os2.h"
#include "command_interface.h"
#include "data_acquisition.h"
#include "data_packet.h"
#include "iridium.h"
#include "mosaic_x5.h"
#include "rxsm_interface.h"
#include "sd_spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Struct containing all information about a single System Orchestrator task signal (LO/SODS/SOE/FFU Ejection)
typedef struct {
  uint32_t       valid_flag;
  uint32_t       invalid_flag;
  GPIO_TypeDef  *port;
  uint16_t       pin;
  GPIO_PinState  active_state;
  uint8_t       *metadata_field;
} orchestrator_signal_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SD_INIT_RETRIES_PER_POWER_CYCLE 3       // Number of allowed failed sd_init()-executions before power cycling
#define SD_STARTUP_RETRY_LIMIT          5       // Number of allowed failed retry sequences before continuing without SD Card
#define SD_METADATA_UPDATE_INTERVAL     16      // Update metadata every 16 sector writes (every 8 kB)
#define FTPS_DEPLOY_TIMEOUT_MS          10000   // 10 second time-out to allow antenna deployment before issuing fTPS deployment
#define MODE_SELECTION_TIMEOUT_MS       540000  // 9 minute time-out interval to allow the system to be put into test mode

#define MANIFOLD_PRESSURE_THRESHOLD     0.4     // Repressurise torus below 0.4 bar gauge
#define PARACHUTE_ALTITUDE_THRESHOLD    5000    // Deploy parachute below 5 kilometers MSL
#define ALTITUDE_DELTA_WINDOW_MS        2100    // Check with newest altitude value (~2 seconds ago)
#define LANDED_ALTITUDE_DELTA_THRESHOLD 1       // Altitude must not vary by 1 meter for landing detection
#define LANDED_ROTATION_THRESHOLD       0       // No rotation when landed
#define GYRO_SENSITIVITY_MDPS_PER_DIGIT 70      // FS +-2000 dps
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* Definitions for SysOrchestrator */
osThreadId_t SysOrchestratorHandle;
const osThreadAttr_t SysOrchestrator_attributes = {
  .name = "SysOrchestrator",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for RMUManager */
osThreadId_t RMUManagerHandle;
const osThreadAttr_t RMUManager_attributes = {
  .name = "RMUManager",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IridiumManager */
osThreadId_t IridiumManagerHandle;
const osThreadAttr_t IridiumManager_attributes = {
  .name = "IridiumManager",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CmdInterface */
osThreadId_t CmdInterfaceHandle;
const osThreadAttr_t CmdInterface_attributes = {
  .name = "CmdInterface",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
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
/* Definitions for SensorDataQueue */
osMessageQueueId_t SensorDataQueueHandle;
const osMessageQueueAttr_t SensorDataQueue_attributes = {
  .name = "SensorDataQueue"
};
/* Definitions for TelecommandQueue */
osMessageQueueId_t TelecommandQueueHandle;
const osMessageQueueAttr_t TelecommandQueue_attributes = {
  .name = "TelecommandQueue"
};
/* Definitions for MissionPhaseDataQueue */
osMessageQueueId_t MissionPhaseDataQueueHandle;
const osMessageQueueAttr_t MissionPhaseDataQueue_attributes = {
  .name = "MissionPhaseDataQueue"
};
/* USER CODE BEGIN PV */
/* General PV */
volatile mission_mode_t mission_mode = MISSION_MODE_FLIGHT; // Put the system automatically in flight mode, this can be changed to test mode while waiting for the RXSM signals
volatile flight_state_t flight_state = FLIGHT_STATE_PRE_EJECTION;
metadata_t mission_metadata = {0}; // Struct to store mission metadata
static const orchestrator_signal_t LO_SIGNAL   = {LO_VALID_EDGE_FLAG, LO_INVALID_EDGE_FLAG, RXSM_LO_GPIO_Port, RXSM_LO_Pin, GPIO_PIN_RESET, &mission_metadata.rxsm_lo};
static const orchestrator_signal_t SODS_SIGNAL = {SODS_VALID_EDGE_FLAG, SODS_INVALID_EDGE_FLAG, RXSM_SODS_GPIO_Port, RXSM_SODS_Pin, GPIO_PIN_RESET, &mission_metadata.rxsm_sods };
// static const orchestrator_signal_t SOE_SIGNAL  = {SOE_VALID_EDGE_FLAG, SOE_INVALID_EDGE_FLAG, RXSM_SOE_GPIO_Port, RXSM_SOE_Pin, GPIO_PIN_RESET, &mission_metadata.rxsm_soe };
static const orchestrator_signal_t EJ_SIGNAL   = {EJECTION_VALID_EDGE_FLAG, EJECTION_INVALID_EDGE_FLAG, FFU_EJECT_DETECT_GPIO_Port, FFU_EJECT_DETECT_Pin, GPIO_PIN_SET, &mission_metadata.ffu_ejection };
static osTimerId_t ejection_safety_timer; // Timer intended to verify valid ejection detection
static volatile uint32_t ejection_safety_timer_duration = 75000;
static osTimerId_t parachute_deployment_safety_timer; // Timer intended to ensure parachute deployment
static volatile uint32_t parachute_deployment_safety_timer_duration = 835000; // 5 kilometer timestamp (ms) according to REXUS trajectory simulations
static osTimerId_t landing_safety_timer; // Timer intended to ensure landing sequence is executed
static volatile uint32_t landing_safety_timer_duration = 1500000; // Landing timestamp (ms), exaggeration from REXUS trajectory simulations

/* Iridium PV */
static osTimerId_t iridium_off_timer; // Timer intended to shut off Iridium after landing
static osTimerId_t iridium_tx_timer;  // Timer intended to schedule Iridium transmissions
static volatile uint32_t iridium_tx_timer_interval = 28000; // Interval between Iridium transmissions in ms
static IridiumCtx_t s_iridium;        // Iridium FSM driver context

/* SD Card PV */
static uint8_t sd_block[SD_BLOCK_SIZE];   // Buffer to store SD write content
static uint16_t block_index = 0;          // Value of current sd_block index
static uint32_t sd_write_block_addr = 3;  // Start writing after the reserved blocks (0-2) on the SD card

osMutexId_t sd_mutex_id;                  // Mutex to protect SD write and read operations
 
const osMutexAttr_t SD_Card_Mutex_attr = {
  "SD_CardMutex",                            // human readable mutex name
   osMutexPrioInherit | osMutexRobust,  // attr_bits
  NULL,                                    // memory for control block   
  0U                                      // size for control block
};

/* RMU PV */
static uint8_t rxsm_rx_byte;                      // Variable to store RXSM UART IT RX byte

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
void StartSystemOrchestrator(void *argument);
void StartRMUManager(void *argument);
void StartGNSSManager(void *argument);
void StartDataAcquisition(void *argument);
void StartSDCardManager(void *argument);
void StartIridiumManager(void *argument);
void StartCommandInterface(void *argument);

/* USER CODE BEGIN PFP */
static void Iridium_OffTimer_Callback(void *argument);
static void Iridium_TxTimer_Callback(void *argument);
static void EjectionTimer_Callback(void *argument);
static void ParachuteTimer_Callback(void *argument);
static void LandingTimer_Callback(void *argument);

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  // Enable UART5 RX DMA interrupt for receiving data from the GNSS module
  __HAL_DMA_ENABLE_IT(&hdma_uart5_rx, DMA_IT_TE);
  
  // Initialize the CAN pending command tracking system and mutex
  can_pending_init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  sd_mutex_id = osMutexNew(&SD_Card_Mutex_attr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  iridium_tx_timer = osTimerNew(Iridium_TxTimer_Callback, osTimerPeriodic, NULL, NULL);
  iridium_off_timer = osTimerNew(Iridium_OffTimer_Callback, osTimerOnce, NULL, NULL);
  ejection_safety_timer = osTimerNew(EjectionTimer_Callback, osTimerOnce, NULL, NULL);
  parachute_deployment_safety_timer = osTimerNew(ParachuteTimer_Callback, osTimerOnce, NULL, NULL);
  landing_safety_timer = osTimerNew(LandingTimer_Callback, osTimerOnce, NULL, NULL);

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SD_CardQueue */
  SD_CardQueueHandle = osMessageQueueNew (128, sizeof(data_packet_t), &SD_CardQueue_attributes);

  /* creation of IridiumQueue */
  IridiumQueueHandle = osMessageQueueNew (16, sizeof(data_packet_t), &IridiumQueue_attributes);

  /* creation of SensorDataQueue */
  SensorDataQueueHandle = osMessageQueueNew (128, sizeof(can_rx_msg_t), &SensorDataQueue_attributes);

  /* creation of TelecommandQueue */
  TelecommandQueueHandle = osMessageQueueNew (16, sizeof(can_rx_msg_t), &TelecommandQueue_attributes);

  /* creation of MissionPhaseDataQueue */
  MissionPhaseDataQueueHandle = osMessageQueueNew (32, sizeof(can_rx_msg_t), &MissionPhaseDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SysOrchestrator */
  SysOrchestratorHandle = osThreadNew(StartSystemOrchestrator, NULL, &SysOrchestrator_attributes);

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

  /* creation of CmdInterface */
  CmdInterfaceHandle = osThreadNew(StartCommandInterface, NULL, &CmdInterface_attributes);

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

  // Configure CAN filter to receive OBC messages
  CAN_FilterTypeDef obc_filter = {0};

  obc_filter.FilterBank           = 0;
  obc_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  obc_filter.FilterScale          = CAN_FILTERSCALE_32BIT;

  // Target: IDs whose top 3 bits are 0b000 → matches 0x000–0x0FF
  // 0x000 << 5 places the 11-bit ID into bits [15:5] of the 16-bit register
  obc_filter.FilterIdHigh         = (0x000 << 5);
  obc_filter.FilterIdLow          = 0x0000;

  // Mask: 0x700 << 5 = 0xE000 — only bits [15:13] of the register are checked,
  // corresponding to CAN ID bits [10:8]. Bits [7:0] are don't-care.
  obc_filter.FilterMaskIdHigh     = (0x700 << 5);
  obc_filter.FilterMaskIdLow      = 0x0000;

  obc_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  obc_filter.FilterActivation     = ENABLE;

  // Configure CAN filter to receive data messages
  CAN_FilterTypeDef data_filter = {0};

  data_filter.FilterBank           = 1;
  data_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  data_filter.FilterScale          = CAN_FILTERSCALE_32BIT;

  // Target: IDs whose top 3 bits are 0b101 → matches 0x500–0x5FF
  // 0x500 << 5 places the 11-bit ID into bits [15:5] of the 16-bit register
  data_filter.FilterIdHigh         = (0x500 << 5);
  data_filter.FilterIdLow          = 0x0000;

  // Same mask as obc_filter — only top 3 bits of the incoming ID are compared
  data_filter.FilterMaskIdHigh     = (0x700 << 5);
  data_filter.FilterMaskIdLow      = 0x0000;

  data_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  data_filter.FilterActivation     = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &obc_filter) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_CAN_ConfigFilter(&hcan1, &data_filter) != HAL_OK)
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
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  hlpuart1.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
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
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GNSS_CB_SHDN_Pin|SD_SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RMU_CAM_TRIG_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHDN_Pin|PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IR_ON_OFF_GPIO_Port, IR_ON_OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GNSS_CB_SHDN_Pin SD_SHDN_Pin */
  GPIO_InitStruct.Pin = GNSS_CB_SHDN_Pin|SD_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMU_CAM_TRIG_Pin SD_CS_Pin */
  GPIO_InitStruct.Pin = RMU_CAM_TRIG_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RXSM_LO_Pin RXSM_SODS_Pin FFU_EJECT_DETECT_Pin */
  GPIO_InitStruct.Pin = RXSM_LO_Pin|RXSM_SODS_Pin|FFU_EJECT_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
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
  /* Read the CAN message from the hardware FIFO and put it into the SensorsQueue for processing by the Data Acquisition Task, 
   * and potentially also into the MissionPhaseDataQueue for processing by the System Orchestrator Task,
   * or into the TelecommandQueue for processing by the Command Interface Task, depending on the message ID. */
  can_rx_msg_t msg;

  // Empty FIFO 0 and handle its contents based on ID
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg.RxHeader, msg.RxData) != HAL_OK)
    {
      break; // Break if no more messages are pressent in FIFO 0.
    }

    uint16_t id = msg.RxHeader.StdId;

    // 0x0** (Telecommand)
    if ((id & 0x700) == 0x000)
    {
      osMessageQueuePut(TelecommandQueueHandle, &msg, 0, 0);
    }
    // 0x5** (Sensor data)
    else if ((id & 0x700) == 0x500)
    {
      osMessageQueuePut(SensorDataQueueHandle, &msg, 0, 0);
      if (id == 0x507 || id == 0x517 || id == 0x518) // If the message is a mission phase data message, also put it into the MissionPhaseDataQueue for processing by the System Orchestrator Task
      {
        osMessageQueuePut(MissionPhaseDataQueueHandle, &msg, 0, 0);
      }
    }
  }
}

/**
 * @brief  Called by HAL when a DMA TX transfer completes on any UART.
 *         Forward to the Iridium driver when it is UART4.
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4) {
    Iridium_TxCpltCallback(&s_iridium);
  }
}

/**
  * @brief  Called by HAL when the IDLE line fires or DMA RX completes on any UART.
  *         Forward to the Iridium driver when it is UART4.
  *         Forward to the GNSS driver when it is UART5.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART.
  * @param  size: number of bytes received in the current UART RX event.
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size){
  if (huart->Instance == UART4) {
    Iridium_RxEventCallback(&s_iridium, size);
  } 
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
  if (huart->Instance == UART4){
    // Clear error flags by resetting UART reception
    __HAL_UART_CLEAR_OREFLAG(huart);

    HAL_UART_DMAStop(huart);

    // Initialise Iridium driver
    Iridium_Init(&s_iridium, &huart4);
  }
  else if (huart->Instance == UART5)
  {
    // Clear error flags by resetting UART reception
    __HAL_UART_CLEAR_OREFLAG(huart);

    HAL_UART_DMAStop(huart);

    mosaic_x5_init(); // Re-initialize the mosaic-X5 UART reception
  }
}

/**
 * @brief  This function handles UART receive complete callback for LPUART1.
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &hlpuart1)
    {
      RXSM_PushByte(rxsm_rx_byte);
      
      HAL_UART_Receive_IT(&hlpuart1, &rxsm_rx_byte, 1);
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
    case RXSM_LO_Pin: // LO signal from RXSM
      if (mission_metadata.ffu_ejection == 0xFF)
        HAL_NVIC_DisableIRQ(RXSM_LO_EXTI_IRQn); // Pogo link is physically gone, pin is floating, disable interrupt
      // Check the state of the LO pin to determine if it is a valid or invalid edge
      if (HAL_GPIO_ReadPin(RXSM_LO_GPIO_Port, RXSM_LO_Pin) == GPIO_PIN_RESET)
        osThreadFlagsSet(SysOrchestratorHandle, LO_VALID_EDGE_FLAG);
      else
        osThreadFlagsSet(SysOrchestratorHandle, LO_INVALID_EDGE_FLAG);
      break;
    case RXSM_SODS_Pin: // SODS signal from RXSM
      if (mission_metadata.ffu_ejection == 0xFF)
        HAL_NVIC_DisableIRQ(RXSM_SODS_EXTI_IRQn); // Pogo link is physically gone, pin is floating, disable interrupt
      // Check the state of the SODS pin to determine if it is a valid or invalid edge
      if (HAL_GPIO_ReadPin(RXSM_SODS_GPIO_Port, RXSM_SODS_Pin) == GPIO_PIN_RESET)
        osThreadFlagsSet(SysOrchestratorHandle, SODS_VALID_EDGE_FLAG);
      else
        osThreadFlagsSet(SysOrchestratorHandle, SODS_INVALID_EDGE_FLAG);
      break;
    /*case RXSM_SOE_Pin: // SOE signal from RXSM
      if (mission_metadata.ffu_ejection == 0xFF)
        HAL_NVIC_DisableIRQ(RXSM_SOE_EXTI_IRQn); // Pogo link is physically gone, pin is floating, disable interrupt
      // Check the state of the SOE pin to determine if it is a valid or invalid edge
      if (HAL_GPIO_ReadPin(RXSM_SOE_GPIO_Port, RXSM_SOE_Pin) == GPIO_PIN_RESET)
        osThreadFlagsSet(SysOrchestratorHandle, SOE_VALID_EDGE_FLAG);
      else
        osThreadFlagsSet(SysOrchestratorHandle, SOE_INVALID_EDGE_FLAG);
      break;*/
    case FFU_EJECT_DETECT_Pin: // Ejection detection signal from FFU
      // Check the state of the FFU_EJECT_DETECT pin to determine if it is a valid or invalid edge
      if (HAL_GPIO_ReadPin(FFU_EJECT_DETECT_GPIO_Port, FFU_EJECT_DETECT_Pin) == GPIO_PIN_SET)
        osThreadFlagsSet(SysOrchestratorHandle, EJECTION_VALID_EDGE_FLAG);
      else
        osThreadFlagsSet(SysOrchestratorHandle, EJECTION_INVALID_EDGE_FLAG);
      break;
    case PGOOD_Pin: // PGOOD signal from Iridium modem
      // Unused in current configuration, disable interrupt since it otherwise triggers very often due to supercap charging
      HAL_NVIC_DisableIRQ(PGOOD_EXTI_IRQn);
      break;
    case NA_Pin: // NA signal from Iridium modem
      // Unused in current configuration, disable interrupt
      HAL_NVIC_DisableIRQ(NA_EXTI_IRQn);
      break; // Unused in current configuration 
    case RI_Pin: // RI signal from Iridium modem
      // Unused in current configuration, disable interrupt
      HAL_NVIC_DisableIRQ(RI_EXTI_IRQn);
      break; // Unused in current configuration 
    default:
      break;
  }
}

/**
 * @brief  Timer that disables Iridium and GNSS after expiring.
 * @param  argument: Not used
 * @retval None
 */
static void Iridium_OffTimer_Callback (void *argument) {
  // Disable Iridium by cutting Iridium 5V power rail
  send_can_command_tracked(EPS_RAIL_DISABLE_CAN_ID, EPS_RAIL_DISABLE_CAN_REPLY_ID, (uint8_t[]){IRIDIUM_5V_RAIL_ID}, 1);  

  // Disable mosaic-x5 by pulling P-MOSFET gate high
  HAL_GPIO_WritePin(GNSS_CB_SHDN_GPIO_Port, GNSS_CB_SHDN_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Periodic 15 s timer callback to trigger Iridium transmission.
 *         Only posts the thread flag and never touches UART or the session SM.
 * @param  argument: Not used
 * @retval None
 */
static void Iridium_TxTimer_Callback(void *argument)
{
    (void)argument;
    osThreadFlagsSet(IridiumManagerHandle, IRIDIUM_TX_FLAG);
}

/**
 * @brief  Timer that ensured landing can't be detected before nose cone separation.
 *         No-op. Timer's expiry is now detected via osTimerIsRunning() in
 *         wait_for_validated_signal() rather than via a state flag set here.
 * @param  argument: Not used
 * @retval None
 */
static void EjectionTimer_Callback(void *argument){
  (void)argument;
  // Intentionally empty.
}

/**
 * @brief  Timer that ensures parachute deployment.
 * @param  argument: Not used
 * @retval None
 */
static void ParachuteTimer_Callback(void *argument){
  (void)argument;
  
  if (mission_mode == MISSION_MODE_FLIGHT){
    send_can_command_tracked(IFS_ARM_BW2_CAN_ID, IFS_ARM_BW2_CAN_REPLY_ID, (uint8_t[]){0x00}, 1);
  }
}

/**
 * @brief  Timer that ensures enabling of the landing sequence.
 * @param  argument: Not used
 * @retval None
 */
static void LandingTimer_Callback(void *argument){
  (void)argument;
  flight_state = FLIGHT_STATE_LANDED;
}

/**
 * @brief Sets the signal's valid flag if its GPIO is already in the active state.
 *
 * This handles events that occurred before the orchestrator started waiting,
 * such as after a reboot. It converts the current GPIO level into the same
 * thread flag that would have been generated by the EXTI ISR.
 *
 * @param s: orchestrator_signal_t typedef struct containing all information about the signal
 *
 * @retval None
 */
static void inline set_valid_flag_if_active(const orchestrator_signal_t *s)
{
  if (HAL_GPIO_ReadPin(s->port, s->pin) == s->active_state)
    osThreadFlagsSet(SysOrchestratorHandle, s->valid_flag);
}

/**
 * @brief Wait until a signal is confirmed, allowing higher-priority signals
 *        to preempt.
 *
 * Before blocking, the current GPIO levels are sampled and converted into
 * thread flags so events that occurred before waiting (e.g. after a reboot)
 * are not missed.
 *
 * The function then waits for either the target signal or any preemptor.
 * Once a candidate signal asserts, it must remain continuously valid for
 * one second without an invalid edge; otherwise it is treated as a glitch
 * and waiting resumes.
 *
 * If multiple signals are pending simultaneously, ejection takes priority
 * because it represents the highest-priority permanent event.
 *
 * @param target       Signal whose confirmation advances the current stage.
 * @param preemptors   Signals that may interrupt waiting.
 * @param n_preemptors Number of entries in @p preemptors.
 *
 * @return Pointer to the signal that was successfully confirmed.
 */
static const orchestrator_signal_t *wait_for_validated_signal(const orchestrator_signal_t *target, const orchestrator_signal_t **preemptors, size_t n_preemptors)
{
  set_valid_flag_if_active(target);
  for (size_t i = 0; i < n_preemptors; i++)
    set_valid_flag_if_active(preemptors[i]);

  uint32_t wait_mask = target->valid_flag;
  for (size_t i = 0; i < n_preemptors; i++)
    wait_mask |= preemptors[i]->valid_flag;

  for (;;)
  {
    uint32_t result = osThreadFlagsWait(wait_mask, osFlagsWaitAny, osWaitForever);
    if (result & osFlagsError)
      continue;

    // Ejection always takes priority if present, since it's the highest-priority and physically permanent event.
    const orchestrator_signal_t *candidate = NULL;
    if (result & EJECTION_VALID_EDGE_FLAG)
      candidate = &EJ_SIGNAL;
    else if (result & target->valid_flag)
      candidate = target;
    else
      for (size_t i = 0; i < n_preemptors; i++)
        if (result & preemptors[i]->valid_flag) { candidate = preemptors[i]; break; }

    // Discard candidate if it is ejection while ejection safety timer has not expired.
    // Pre-LO: LO hasn't happened yet this boot, so ejection can't be legitimate — forbidden.
    // Post-LO: forbidden only while the safety timer is still counting down.
    // (If a reboot skipped restarting the timer, isRunning() reads false, same as "expired" —
    // this is the accepted worst-case fallback for a reboot mid-window.)
    if (candidate == &EJ_SIGNAL && (mission_metadata.rxsm_lo != 0xFF || osTimerIsRunning(ejection_safety_timer)))
        continue;

    // 1s debounce: must stay valid with no invalid edge, or it's a glitch.
    // First discard any stale invalid-edge flag from before this debounce window started
    osThreadFlagsClear(candidate->invalid_flag);
    if (osThreadFlagsWait(candidate->invalid_flag, osFlagsWaitAny, pdMS_TO_TICKS(1000)) == osFlagsErrorTimeout)
      return candidate;
    // glitch — candidate->valid_flag was already consumed by the match above and
    // will only be re-armed by a fresh EXTI edge. Ejection is a permanent level
    // change (connector separation pulls the line high and it stays high), so if
    // contact chatter during separation trips a glitch inside the debounce window,
    // no further edge will ever come. Re-sample current levels so a still-active
    // line is re-flagged without needing a new edge.
    set_valid_flag_if_active(target);
    for (size_t i = 0; i < n_preemptors; i++)
      set_valid_flag_if_active(preemptors[i]);
    // loop back, still waiting on the full mask
  }
}

/**
 * @brief Wait for the next confirmed signal, then persist it.
 *
 * Blocks until either the target signal or a preempting signal is confirmed.
 * Marks the confirmed signal in the mission metadata and immediately writes
 * the updated metadata to persistent storage.
 *
 * @param target       Signal whose confirmation is awaited.
 * @param preemptors   Signals that may interrupt target signal
 * @param n            Number of entries in @p preemptors.
 *
 * @retval             None
 */
static void wait_and_record_signal(const orchestrator_signal_t *target, const orchestrator_signal_t **preemptors, size_t n)
{
    const orchestrator_signal_t *result = wait_for_validated_signal(target, preemptors, n);
    *result->metadata_field = 0xFF;

    osMutexAcquire(sd_mutex_id, osWaitForever);
    metadata_write(&mission_metadata);
    osMutexRelease(sd_mutex_id);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSystemOrchestrator */
/**
  * @brief  Function implementing the SysOrchestrator thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSystemOrchestrator */
void StartSystemOrchestrator(void *argument)
{
  /* USER CODE BEGIN 5 */
  // Signal the RMUManager to start up by setting the RMU_STARTUP_FLAG
  osThreadFlagsSet(RMUManagerHandle, RMU_STARTUP_FLAG);

  // Wait until the SD card is initialised and ready
  // If the SD card fails to initialise, the System Orchestrator task continues while the SD Card Manager Task keeps trying to initialise.
  osThreadFlagsWait(SD_CARD_INIT_FLAG, osFlagsWaitAny, osWaitForever);
 
  // Read the mission metadata from the SD card and store it in the global mission_metadata struct
  osMutexAcquire(sd_mutex_id, osWaitForever); 
  metadata_read(&mission_metadata);
  if (mission_metadata.last_written_sector < 3){
    mission_metadata.last_written_sector = 4; // Ensure that the last written sector is not in the reserved area of the SD card
  } else if (mission_metadata.last_written_sector > 3){
    sd_write_block_addr = mission_metadata.last_written_sector ++; // Set the current block address to the next block after the last written sector
  }
  osMutexRelease(sd_mutex_id); 

  // Wait for the Lift-Off (LO) signal before proceeding with the rest of the system startup sequence.
  // Only wait if LO, SODS, SOE and ejection have NOT already been recorded.
  // SODS, SOE and ejection can each preempt this wait if they arrive first since they imply LO already happened, 
  // and any of the four already being set (from this run or a prior reboot) means LO is implicitly confirmed, 
  // so skip waiting and proceed immediately.
  if (mission_metadata.rxsm_lo != 0xFF && mission_metadata.rxsm_sods != 0xFF &&
    mission_metadata.rxsm_soe != 0xFF && mission_metadata.ffu_ejection != 0xFF) {
    const orchestrator_signal_t *pre[] = {&SODS_SIGNAL, &EJ_SIGNAL};
    wait_and_record_signal(&LO_SIGNAL, pre, 2);
    // Start a timer until nose cone separation to protect against premature ejection
    // osTimerStart(ejection_safety_timer, pdMS_TO_TICKS(ejection_safety_timer_duration)); 
  } 

  // Instruct EPS to switch to internal power 30 seconds after LO in case this has not happened yet through RXSM uplink
  osDelay(pdMS_TO_TICKS(3000)); // TODO: Increase to 30 seconds
  send_can_command_tracked(EPS_BATTERIES_ENABLE_CAN_ID, EPS_BATTERIES_ENABLE_CAN_REPLY_ID, (uint8_t[]){0x00}, 1);
  
  // Wait for the Start-Of-Data-Storage (SODS) signal before proceeding with the rest of the system startup sequence.
  // SOE and ejection can each preempt this wait if they arrive first since they imply SODS already happened, 
  // and any of the three already being set means SODS is implicitly confirmed, so skip waiting and proceed immediately.
  if (mission_metadata.rxsm_sods != 0xFF && mission_metadata.rxsm_soe != 0xFF && mission_metadata.ffu_ejection != 0xFF) {
    const orchestrator_signal_t *pre[] = {&EJ_SIGNAL};
    wait_and_record_signal(&SODS_SIGNAL, pre, 1);
  }

  // Signal the SDCardManager to start up by setting the SD_CARD_STARTUP_FLAG
  osThreadFlagsSet(SDCardManagerHandle, SD_CARD_STARTUP_FLAG);

  // Signal the DataAcquisition task to start up by setting the DATA_ACQ_STARTUP_FLAG
  osThreadFlagsSet(DataAcquisitionHandle, DATA_ACQ_STARTUP_FLAG);

  // Signal the RMUManager to turn on the RMU camera by setting the RMU_CAMERA_TRIGGER_FLAG
  osThreadFlagsSet(RMUManagerHandle, RMU_CAMERA_TRIGGER_FLAG);

  // Instruct EPS to turn on camera system power rail
  send_can_command_tracked(EPS_RAIL_ENABLE_CAN_ID, EPS_RAIL_ENABLE_CAN_REPLY_ID, (uint8_t[]){CS_5V_RAIL_ID}, 1);

  // Wait for the Start-Of-Experiment (SOE) signal before proceeding with the rest of the system startup sequence.
  // Ejection can preempt this wait if it arrives first since it implies SOE already happened, 
  // and either already being set means SODS is implicitly confirmed, so skip waiting and proceed immediately.
  /*if (mission_metadata.rxsm_soe != 0xFF && mission_metadata.ffu_ejection != 0xFF) {
    const orchestrator_signal_t *pre[] = {&EJ_SIGNAL};
    wait_and_record_signal(&SOE_SIGNAL, pre, 1);
  }*/

  // Instruct EPS to turn on UHFCOM, GNSS and IFS 3V3 power rails
  send_can_command_tracked(EPS_RAIL_ENABLE_CAN_ID, EPS_RAIL_ENABLE_CAN_REPLY_ID, (uint8_t[]){GNSS_3V3_RAIL_ID}, 1);
  send_can_command_tracked(EPS_RAIL_ENABLE_CAN_ID, EPS_RAIL_ENABLE_CAN_REPLY_ID, (uint8_t[]){IFS_3V3_RAIL_ID}, 1);

  // Signal the GNSSManager to start up by setting the GNSS_STARTUP_FLAG
  osThreadFlagsSet(GNSSManagerHandle, GNSS_STARTUP_FLAG);

  // Wait for the ejection signal before proceeding with the rest of the system startup sequence.
  // Only wait if ejection has NOT already been recorded (from this run or a prior reboot).
  // If it has already been triggered, skip waiting and proceed immediately.
  if (mission_metadata.ffu_ejection != 0xFF) {
    wait_and_record_signal(&EJ_SIGNAL, NULL, 0);
  }

  // Start a timer to ensure parachute deployment and landing detection
  osTimerStart(parachute_deployment_safety_timer, pdMS_TO_TICKS(parachute_deployment_safety_timer_duration));  
  osTimerStart(landing_safety_timer, pdMS_TO_TICKS(landing_safety_timer_duration));

  // Signal the RMUManager to terminate itself by setting the RMU_SHUTDOWN_FLAG
  // Only do it in flight mode, so RXSM uplink can be used for testing purposes
  if (mission_mode == MISSION_MODE_FLIGHT){
    osThreadFlagsSet(RMUManagerHandle, RMU_SHUTDOWN_FLAG);
  } 

  // Instruct EPS to turn on Iridium 5V power rail, 5 seconds after ejection
  osDelay(pdMS_TO_TICKS(5000));
  send_can_command_tracked(EPS_RAIL_ENABLE_CAN_ID, EPS_RAIL_ENABLE_CAN_REPLY_ID, (uint8_t[]){IRIDIUM_5V_RAIL_ID}, 1);
  
  // Instruct EPS to turn on IFS 5V power rail and start arming actuators (ONLY IN FLIGHT MODE)
  if (mission_mode == MISSION_MODE_FLIGHT){
    send_can_command_tracked(EPS_RAIL_ENABLE_CAN_ID, EPS_RAIL_ENABLE_CAN_REPLY_ID, (uint8_t[]){IFS_5V_RAIL_ID}, 1);

    // Issue antenna burn-wire ARM signal
    send_can_command_tracked(IFS_ARM_BW1_CAN_ID, IFS_ARM_BW1_CAN_REPLY_ID, (uint8_t[]){0x00}, 1);

    osThreadFlagsWait(ANTENNA_DEPLOYED_FLAG, osFlagsWaitAny, FTPS_DEPLOY_TIMEOUT_MS);

    // Issue CGG1 ARM signal
    send_can_command_tracked(IFS_ARM_CGG1_CAN_ID, IFS_ARM_CGG1_CAN_REPLY_ID, (uint8_t[]){0x00}, 1);
  }

  // Signal the IridiumManager task to start up by setting the IRIDIUM_STARTUP_FLAG
  osThreadFlagsSet(IridiumManagerHandle, IRIDIUM_STARTUP_FLAG);

  can_rx_msg_t rx_msg;
  float altitude = MAXFLOAT;
  float altitude_ref = MAXFLOAT;
  uint32_t altitude_ref_tick = 0;
  float manifold_pressure = MAXFLOAT; 
  float rotation_rate = MAXFLOAT;
  uint16_t landed_counter = 0;
  bool new_rotation_sample = false;
  uint32_t deploy_tick = 0;
  bool deploy_tick_set = false;

  for (;;)
  {
    // Get data message out of the MissionPhaseDataQueue, without blocking on it. 
    if (osMessageQueueGet(MissionPhaseDataQueueHandle, &rx_msg, NULL, 100U) == osOK) {
      switch (rx_msg.RxHeader.StdId)
      {
        case 0x502: // Altitude data
        {
          uint16_t altitude_raw = (rx_msg.RxData[0] << 8) | rx_msg.RxData[1];
          altitude = altitude_raw * 1.5f; // Convert raw value to altitude in meters
          break;
        }
        case 0x517: // Manifold pressure data - already converted, 4-byte big-endian float in kPa
        {
          uint32_t manifold_raw = ((uint32_t)rx_msg.RxData[0] << 24) | ((uint32_t)rx_msg.RxData[1] << 16) | ((uint32_t)rx_msg.RxData[2] << 8) | (uint32_t)rx_msg.RxData[3];
          memcpy(&manifold_pressure, &manifold_raw, sizeof(float));
          break;
        }
        case 0x520: // Rotation data: [Yaw, Roll, Pitch], 2 bytes each, big-endian signed int
        {
          int16_t yaw_raw = (int16_t)((rx_msg.RxData[0] << 8) | rx_msg.RxData[1]);
          int16_t roll_raw = (int16_t)((rx_msg.RxData[2] << 8) | rx_msg.RxData[3]);
          int16_t pitch_raw = (int16_t)((rx_msg.RxData[4] << 8) | rx_msg.RxData[5]);

          float yaw = yaw_raw * GYRO_SENSITIVITY_MDPS_PER_DIGIT / 1000.0f;
          float roll = roll_raw * GYRO_SENSITIVITY_MDPS_PER_DIGIT / 1000.0f;
          float pitch = pitch_raw * GYRO_SENSITIVITY_MDPS_PER_DIGIT / 1000.0f;

          // Magnitude of the angular velocity vector
          rotation_rate = sqrtf(yaw * yaw + roll * roll + pitch * pitch);

          new_rotation_sample = true;
          break;
        }
        default:
          break;
      }
    }

    if (mission_mode == MISSION_MODE_FLIGHT){
      switch (flight_state) {
        case FLIGHT_STATE_PRE_EJECTION:
          // Wait for ejection to put system in FLIGHT_STATE_POST_EJECTION
          break;
        case FLIGHT_STATE_POST_EJECTION:
          // Manifold pressure drop → fire CGG2
          if (mission_metadata.cgg1_fired && !mission_metadata.cgg2_fired && manifold_pressure <= MANIFOLD_PRESSURE_THRESHOLD)
          {
            // Issue CGG2 ARM signal
            send_can_command_tracked(IFS_ARM_CGG2_CAN_ID, IFS_ARM_CGG2_CAN_REPLY_ID, (uint8_t[]){0x00}, 1);
          }
          // Altitude thresholds crossed or timer expired → deploy parachute
          if (altitude <= PARACHUTE_ALTITUDE_THRESHOLD)
          {
            send_can_command_tracked(IFS_ARM_BW2_CAN_ID, IFS_ARM_BW2_CAN_REPLY_ID, (uint8_t[]){0x00}, 1);
            osTimerStop(parachute_deployment_safety_timer);
          }
          break;
        case FLIGHT_STATE_PARACHUTE_DEPLOYED: // Set after IFS confirmation that BW2 is fired
          // Parachute open → fire CGG2 if not already fired, but with delay to ensure parachute has stabilized after deployment
          if (!deploy_tick_set) {
            deploy_tick = osKernelGetTickCount();
            deploy_tick_set = true;
          }        
          if (!mission_metadata.cgg2_fired && (osKernelGetTickCount() - deploy_tick >= pdMS_TO_TICKS(30000))){
              // Issue CGG2 ARM signal
              send_can_command_tracked(IFS_ARM_CGG2_CAN_ID, IFS_ARM_CGG2_CAN_REPLY_ID, (uint8_t[]){0x00}, 1);
          }
          if (altitude_ref == MAXFLOAT){
            altitude_ref = altitude;
            altitude_ref_tick = osKernelGetTickCount();
          }
          if (new_rotation_sample){
            new_rotation_sample = false;

            uint32_t now = osKernelGetTickCount();
            bool window_elapsed = (now - altitude_ref_tick) >= pdMS_TO_TICKS(ALTITUDE_DELTA_WINDOW_MS);
            bool altitude_stable = window_elapsed &&
                                (fabsf(altitude - altitude_ref) <= LANDED_ALTITUDE_DELTA_THRESHOLD);

            

            if (altitude_stable)
            {
              if (rotation_rate <= LANDED_ROTATION_THRESHOLD)
              {
                landed_counter++;
              }
              else if (landed_counter > 0){
                  landed_counter--;
              }
            }
            else
            {
              landed_counter = 0;
            }

            // Slide the comparison window forward once it's elapsed, regardless
            // of the outcome above, so we keep comparing to "~2s ago" rather
            // than freezing on the very first sample taken in this state.
            if (window_elapsed)
            {
              altitude_ref = altitude;
              altitude_ref_tick = now;
            }

            if (landed_counter >= 80) // 5 seconds at 16 Hz
            {
              flight_state = FLIGHT_STATE_LANDED;
              osTimerStop(landing_safety_timer);

            }
          }
          break;
        case FLIGHT_STATE_LANDED:
          // Disable unused power rails
          send_can_command_tracked(EPS_RAIL_DISABLE_CAN_ID, EPS_RAIL_DISABLE_CAN_REPLY_ID, (uint8_t[]){CS_5V_RAIL_ID}, 1);
          send_can_command_tracked(EPS_RAIL_DISABLE_CAN_ID, EPS_RAIL_DISABLE_CAN_REPLY_ID, (uint8_t[]){IFS_3V3_RAIL_ID}, 1);

          // Enable beacon mode
          send_can_command_tracked(UHFCOM_BEACON_ENABLE_CAN_ID, UHFCOM_BEACON_ENABLE_CAN_REPLY_ID, (uint8_t[]){0x00}, 1);

          // Start Iridium timer (5 minutes)
          osTimerStart(iridium_off_timer, pdMS_TO_TICKS(300000));

          flight_state = FLIGHT_STATE_COMPLETE;
          break;
        case FLIGHT_STATE_COMPLETE:
          osDelay(1000);
          break;
        default:
          break;
      }
    }
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
  // Initialize RXSM parser variables
  RXSM_Init();

  // Wait for SystemOrchestrator to start up and set the RMU_STARTUP_FLAG before proceeding
  osThreadFlagsWait(RMU_STARTUP_FLAG, osFlagsWaitAny, osWaitForever);

  // Start UART reception from RXSM
  HAL_UART_Receive_IT(&hlpuart1, &rxsm_rx_byte, 1);

  RXSM_Telecommand_t tc = {0};

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

    // Check for the RMU_CAMERA_TRIGGER_FLAG to know when to enable the RMU camera (non-blocking)
    flags = osThreadFlagsWait(RMU_CAMERA_TRIGGER_FLAG, osFlagsWaitAny, 0);
    
    // If the RMU_CAMERA_TRIGGER_FLAG is set, assert the camera trigger GPIO signal
    if ((int32_t)flags >= 0 && (flags & RMU_CAMERA_TRIGGER_FLAG))
    {
      HAL_GPIO_WritePin(RMU_CAM_TRIG_GPIO_Port, RMU_CAM_TRIG_Pin, GPIO_PIN_SET);;
    }

    // Otherwise, resume RXSM communication functionality
    if (RXSM_GetMessage(&tc))
    {
      RXSMInterface_ProcessMessage(&tc);
    }

    osDelay(1);
  }

  // Broken out of main loop, initialise RMU Manager Task termination

  // De-assert the camera trigger GPIO signal (Connection with RMU is lost so camera is powered independently from FFU)
  HAL_GPIO_WritePin(RMU_CAM_TRIG_GPIO_Port, RMU_CAM_TRIG_Pin, GPIO_PIN_RESET);

  // Stop LPUART1 RX
  HAL_UART_AbortReceive_IT(&hlpuart1);

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

  // Wait for SystemOrchestrator to start up and set the GNSS_STARTUP_FLAG before proceeding
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
  // Wait for SystemOrchestrator to start up and set the DATA_ACQ_STARTUP_FLAG before proceeding
  osThreadFlagsWait(DATA_ACQ_STARTUP_FLAG, osFlagsWaitAny, osWaitForever);

  can_rx_msg_t rx_msg;
  
  /* Infinite loop */
  for(;;)
  {
    // Wait for a CAN message to be received and put into the SensorDataQueueHandle by the CAN RX ISR
    if (osMessageQueueGet(SensorDataQueueHandle, &rx_msg, NULL, osWaitForever) == osOK) {
      DataAcquisition_ProcessMessage(&rx_msg);
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
  // Enable SD card power (P-MOSFET gate low)
  HAL_GPIO_WritePin(SD_SHDN_GPIO_Port, SD_SHDN_Pin, GPIO_PIN_RESET);
  osDelay(50);   // Allow power rail to stabilize

  uint32_t powerCycleCount = 0;
  bool sd_available = false;
  bool startup_released = false;

  // Initialize SD card
  while (!sd_available)
  {
    // Try  few times before power cycling
    for (int retry = 0; retry < SD_INIT_RETRIES_PER_POWER_CYCLE; retry++)
    {
        if (sd_init() == 0)
        {
            sd_available = true;
            break;
        }

        osDelay(100);
    }

    if (sd_available)
    {
        break;
    }

    powerCycleCount++;

    // After N power cycles, allow the rest of the system to continue
    if (!startup_released && powerCycleCount >= SD_STARTUP_RETRY_LIMIT)
    {
        startup_released = true;

        osThreadFlagsSet(SysOrchestratorHandle, SD_CARD_INIT_FLAG);
    }

    // Power cycle the SD card
    HAL_GPIO_WritePin(SD_SHDN_GPIO_Port, SD_SHDN_Pin, GPIO_PIN_SET);   // OFF
    osDelay(200);

    HAL_GPIO_WritePin(SD_SHDN_GPIO_Port, SD_SHDN_Pin, GPIO_PIN_RESET); // ON
    osDelay(200);
  }

  // Write a blank block to the first data block address to ensure the SD card is ready for subsequent writes
  osMutexAcquire(sd_mutex_id, osWaitForever);
  memset(sd_block, 0x00, SD_BLOCK_SIZE);
  sd_write_block(sd_write_block_addr, sd_block);
  sd_write_block_addr++;    
  osMutexRelease(sd_mutex_id);

  // Signal to SystemOrchestrator that SD card is ready
  osThreadFlagsSet(SysOrchestratorHandle, SD_CARD_INIT_FLAG);

  // Wait for SystemOrchestrator to start up and set the SD_CARD_STARTUP_FLAG before proceeding
  osThreadFlagsWait(SD_CARD_STARTUP_FLAG, osFlagsWaitAny, osWaitForever);
  
  /* Infinite loop */
  data_packet_t entry;
  for(;;)
  {
    // Wait for next data entry in the SD Card Queue
    osMessageQueueGet(SD_CardQueueHandle, &entry, NULL, osWaitForever);

    // Copy into block buffer
    memcpy(&sd_block[block_index], &entry, sizeof(data_packet_t));
    block_index += sizeof(data_packet_t);

    // If block full, write to SD
    if (block_index == SD_BLOCK_SIZE) {
        osMutexAcquire(sd_mutex_id, osWaitForever);
        for (int i = 0; i < 5; i++) { // Retry up to 5 times if write fails
            if (sd_write_block(sd_write_block_addr, sd_block) == 0) {
                if (sd_write_block_addr - mission_metadata.last_written_sector >= SD_METADATA_UPDATE_INTERVAL) {
                    // Update mission metadata with the new last written sector address (and updated GNSS data)
                    mission_metadata.last_written_sector = sd_write_block_addr;
                    metadata_write(&mission_metadata);
                }
                sd_write_block_addr++;
                break;
            }
            osDelay(100);
        }
        block_index = 0; 
        osMutexRelease(sd_mutex_id);
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
  // Wait for SystemOrchestrator to start up and set the IRIDIUM_STARTUP_FLAG before proceeding
  osThreadFlagsWait(IRIDIUM_STARTUP_FLAG, osFlagsWaitAny, osWaitForever);

  // Enable the LTC3225 Supercap charger
  HAL_GPIO_WritePin(GPIOB, SHDN_Pin, GPIO_PIN_SET);

  // Enable output power with the LTC4210
  HAL_GPIO_WritePin(GPIOB, PWR_EN_Pin, GPIO_PIN_SET);

  // Set ON/OFF pin high to enable the modem
  HAL_GPIO_WritePin(GPIOC, IR_ON_OFF_Pin, GPIO_PIN_SET);
  
  // Initialise Iridium driver
  Iridium_Init(&s_iridium, &huart4);

  // Start periodic TX timer
  osTimerStart(iridium_tx_timer, pdMS_TO_TICKS(iridium_tx_timer_interval));

  
  /* Infinite loop */
  for(;;)
  {
    // 1. Drain queue
    data_packet_t pkt;
    while (osMessageQueueGet(IridiumQueueHandle, &pkt, NULL, 0u) == osOK)
    {
        Iridium_PushPacket(&s_iridium, &pkt);
    }

    // 2. Drive Iridium session State Machine
    Iridium_SessionTick(&s_iridium);

    //3. Check for TX flag (1-tick timeout = natural yield)
    uint32_t flags = osThreadFlagsWait(IRIDIUM_TX_FLAG, osFlagsWaitAny, 1u);
    if (flags == IRIDIUM_TX_FLAG)
    {
        /*
          * Iridium_SessionStart() is a no-op if:
          *   - session_state != IDLE  (previous session still running)
          */
        Iridium_SessionStart(&s_iridium);
    }

    osDelay(1);
  }
  /* USER CODE END StartIridiumManager */
}

/* USER CODE BEGIN Header_StartCommandInterface */
/**
* @brief Function implementing the CmdInterface thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommandInterface */
void StartCommandInterface(void *argument)
{
  /* USER CODE BEGIN StartCommandInterface */
  can_rx_msg_t rx_msg;

  /* Infinite loop */
  for(;;)
  {
    // Wait for a CAN message to be received and put into the TelecommandQueue by the CAN RX ISR
    if (osMessageQueueGet(TelecommandQueueHandle, &rx_msg, NULL, 100U) == osOK) {
      CommandInterface_ProcessMessage(&rx_msg);
    }

    /* Retransmit any commands whose reply has not arrived in time */
    can_pending_retry();
    osDelay(1);

  }
  /* USER CODE END StartCommandInterface */
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
