/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t soc;
  uint16_t soh;
  uint16_t temperature;
  uint16_t voltage;
  uint16_t current;
} BatteryData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* INA219 Definitions */
#define INA219_ADDRESS  (0x40 << 1)  // 7-bit address shifted left
#define INA219_CONFIG   0x00
#define INA219_SHUNT    0x01
#define INA219_BUS      0x02
#define INA219_POWER    0x03
#define INA219_CURRENT  0x04
#define INA219_CAL      0x05

/* Battery Parameters */
#define BATTERY_CAPACITY_mAh  3000.0f
#define FULL_CHARGE_VOLTAGE   12.6f
#define EMPTY_VOLTAGE        9.0f
#define LM35_MV_PER_C        10.0f    // LM35 outputs 10mV per °C
#define VREF                 3.3f     // Reference voltage

/* CAN Parameters */
#define Battery_QUEUE_SIZE  8
#define CAN_TX_MSG_ID        0x124   // CAN message ID for battery data
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

osThreadId defaultTaskHandle;
osThreadId batteryMonitorTHandle;
osThreadId canTaskHandle;
osMessageQId BatterQueueHandle;
osMutexId batteryDataMutexHandle;
/* USER CODE BEGIN PV */
/* Global Variables for Debugging (view in Live Expressions) */
volatile float g_voltage = 0;
volatile float g_current = 0;
volatile float g_temperature = 0;
volatile float g_soc = 100.0f;
volatile float g_soh = 100.0f;
volatile float g_remainingCapacity_mAh = BATTERY_CAPACITY_mAh;
volatile float g_totalDischarged_mAh = 0;

/* CAN Variables */
BatteryData_t batteryData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
void StartDefaultTask(void const * argument);
void StartBatteryMonitorTask(void const * argument);
void StartCANTask(void const * argument);

/* USER CODE BEGIN PFP */

/* Private function prototypes */
void INA219_Init(void);
void INA219_WriteReg(uint8_t reg, uint16_t value);
uint16_t INA219_ReadReg(uint8_t reg);
float INA219_ReadBusVoltage(void);
float INA219_ReadCurrent(void);
float Read_LM35_Temperature(void);
float Calculate_SOC(float current_mA, float voltage_V);
float VoltageBased_SOC(float voltage_V); //helper function for SOC (on Charging and Discharging
float Calculate_SOH(void);
void Update_Battery_Parameters(void);

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize INA219
  INA219_Init();

  // Calibrate ADC with dummy reads
  for(uint8_t i=0; i<5; i++) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    HAL_ADC_GetValue(&hadc1);
    HAL_Delay(10);
  }
  // Initialize CAN filter (accept all messages)
  CAN_FilterTypeDef can_filter = {
      .FilterBank = 0,
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterScale = CAN_FILTERSCALE_32BIT,
      .FilterIdHigh = 0x0000,
      .FilterIdLow = 0x0000,
      .FilterMaskIdHigh = 0x0000,
      .FilterMaskIdLow = 0x0000,
      .FilterFIFOAssignment = CAN_RX_FIFO0,
      .FilterActivation = ENABLE,
      .SlaveStartFilterBank = 14
  };
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);

  // Start CAN
  HAL_CAN_Start(&hcan1);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of batteryDataMutex */
  osMutexDef(batteryDataMutex);
  batteryDataMutexHandle = osMutexCreate(osMutex(batteryDataMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of BatterQueue */
  osMessageQDef(BatterQueue, 8, uint16_t);
  BatterQueueHandle = osMessageCreate(osMessageQ(BatterQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of batteryMonitorT */
  osThreadDef(batteryMonitorT, StartBatteryMonitorTask, osPriorityAboveNormal, 0, 256);
  batteryMonitorTHandle = osThreadCreate(osThread(batteryMonitorT), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, StartCANTask, osPriorityNormal, 0, 256);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

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
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

	// Add in MX_GPIO_Init():
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Update_Battery_Parameters(void) {
	 // Read voltage with validation
	    float measured_voltage = INA219_ReadBusVoltage();
	    if (measured_voltage < 5.0f || measured_voltage > 15.0f) { // Sanity check
	        g_voltage = 0;
	    } else {
	        g_voltage = measured_voltage;
	    }
  g_current = INA219_ReadCurrent();
  // Read LM35 for battery temperature
  g_temperature = Read_LM35_Temperature();
  // Calculate SOC and SOH
  g_soc = Calculate_SOC(g_current, g_voltage);
  g_soh = Calculate_SOH();

  // Update the battery data structure
  batteryData.soc = g_soc;
  batteryData.soh = g_soh;
  batteryData.temperature = g_temperature;
  batteryData.voltage = g_voltage;
  batteryData.current = g_current;
}
/* INA219 Functions */
void INA219_Init(void) {
  // Reset the INA219
  INA219_WriteReg(INA219_CONFIG, 0x8000);
  HAL_Delay(10);

  // Configure for 32V range, 12-bit ADC
  uint16_t config = 0x399F;
  INA219_WriteReg(INA219_CONFIG, config);

  // Calibration for 3.2A range, 0.1Ω shunt
  float current_LSB = 3.2f / 32768.0f;
  uint16_t cal = (uint16_t)(0.04096f / (current_LSB * 0.1f));
  INA219_WriteReg(INA219_CAL, cal);
}

void INA219_WriteReg(uint8_t reg, uint16_t value) {
  uint8_t data[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
  HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDRESS, data, 3, HAL_MAX_DELAY);
}

uint16_t INA219_ReadReg(uint8_t reg) {
  uint8_t data[2];
  HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDRESS, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, INA219_ADDRESS, data, 2, HAL_MAX_DELAY);
  return (data[0] << 8) | data[1];
}

float INA219_ReadBusVoltage(void) {
  uint16_t reg = INA219_ReadReg(INA219_BUS);
  return (float)((reg >> 3) * 4) / 1000.0f;  // Convert to volts
}

float INA219_ReadCurrent(void) {
  int16_t reg = (int16_t)INA219_ReadReg(INA219_CURRENT);
  float current = (float)reg * 0.1f;  // 0.1mA per LSB

  // Add deadzone for noise rejection
  if (fabs(current) < 5.0f) return 0.0f;  // Treat <5mA as 0
  return current;
}

/* LM35 Temperature Sensor Function */
float Read_LM35_Temperature(void) {
    #define INVALID_READING_THRESHOLD  5
    #define MIN_VALID_VOLTAGE          0.1f     // ~10°C
    #define MAX_VALID_VOLTAGE          1.5f     // ~150°C
    #define DISCONNECTED_VOLTAGE       0.03f    // <30mV = probably floating input

    static uint8_t invalid_count = 0;
    static float last_valid_temp = 0.0f;

    // Average over 5 samples
    float voltage_sum = 0.0f;
    const uint8_t samples = 5;
    for (uint8_t i = 0; i < samples; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
            float voltage = (adcValue * VREF) / 4095.0f;
            voltage_sum += voltage;
        }
        HAL_Delay(2);
    }

    float avg_voltage = voltage_sum / samples;

    // Disconnected check
    if (avg_voltage < DISCONNECTED_VOLTAGE) {
        invalid_count++;
        if (invalid_count >= INVALID_READING_THRESHOLD) {
            return 0.0f;  // Confirmed disconnected
        }
        return last_valid_temp;
    }

    // Valid voltage range check
    if (avg_voltage >= MIN_VALID_VOLTAGE && avg_voltage <= MAX_VALID_VOLTAGE) {
        invalid_count = 0;
        last_valid_temp = avg_voltage * 100.0f;  // LM35: 10mV/°C
        return last_valid_temp;
    }

    // If out-of-range, keep last value temporarily
    invalid_count++;
    if (invalid_count >= INVALID_READING_THRESHOLD) {
        return 0.0f;
    }
    return last_valid_temp;
}

/* Battery Calculations */
float Calculate_SOC(float current_mA, float voltage_V) {
    static bool initialized = false;
    static uint32_t lastTime = 0;

    if (!initialized || (HAL_GetTick() - lastTime > 3600000)) { // 1-hour timeout
        g_remainingCapacity_mAh = VoltageBased_SOC(voltage_V)/100.0f * BATTERY_CAPACITY_mAh;
        initialized = true;
        lastTime = HAL_GetTick();
        return VoltageBased_SOC(voltage_V);
    }

    // Calculate time difference in hours
    float delta_h = (HAL_GetTick() - lastTime) / 3600000.0f;
    lastTime = HAL_GetTick();

    // Handle charging/discharging
    if (current_mA > 0.1f) { // Discharging (ignore noise < 0.1mA)
        g_remainingCapacity_mAh -= current_mA * delta_h;
        g_totalDischarged_mAh += current_mA * delta_h;
    }
    else if (current_mA < -0.1f) { // Charging (ignore noise > -0.1mA)
        g_remainingCapacity_mAh += fabsf(current_mA) * delta_h;
    }

    // Clamp capacity
    g_remainingCapacity_mAh = fmaxf(0.0f, fminf(g_remainingCapacity_mAh, BATTERY_CAPACITY_mAh));

    // Voltage-based corrections
    if (voltage_V >= FULL_CHARGE_VOLTAGE) {
        g_remainingCapacity_mAh = BATTERY_CAPACITY_mAh;
    }
    else if (voltage_V <= EMPTY_VOLTAGE) {
        g_remainingCapacity_mAh = 0.0f;
    }

    return (g_remainingCapacity_mAh / BATTERY_CAPACITY_mAh) * 100.0f;
}
// Helper: Estimate SOC from voltage (when Coulomb counting is unreliable)
float VoltageBased_SOC(float voltage_V) {
    // More accurate 3S Li-ion voltage curve (adjust per your battery specs)
    if (voltage_V >= 12.6f) return 100.0f;
    if (voltage_V <= 9.0f)  return 0.0f;

    // Piecewise linear approximation
    if (voltage_V >= 12.3f) return 90.0f + (voltage_V - 12.3f) * 33.3f;  // 12.3-12.6V: 90-100%
    if (voltage_V >= 11.8f) return 70.0f + (voltage_V - 11.8f) * 40.0f;  // 11.8-12.3V: 70-90%
    if (voltage_V >= 11.1f) return 30.0f + (voltage_V - 11.1f) * 57.1f;  // 11.1-11.8V: 30-70%
    return (voltage_V - 9.0f) * 14.3f;                                   // 9.0-11.1V: 0-30%
}

float Calculate_SOH(void) {
  return ((BATTERY_CAPACITY_mAh - g_totalDischarged_mAh) / BATTERY_CAPACITY_mAh) * 100.0f;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	  for(;;)
	  {
	    osDelay(1000);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBatteryMonitorTask */
/**
* @brief Function implementing the batteryMonitorT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBatteryMonitorTask */
void StartBatteryMonitorTask(void const * argument)
{
  /* USER CODE BEGIN StartBatteryMonitorTask */
	  /* Infinite loop */
	  for(;;)
	  {
	    osMutexWait(batteryDataMutexHandle, osWaitForever);
	    Update_Battery_Parameters();
	    osMutexRelease(batteryDataMutexHandle);
	    osDelay(1000);  // Update every second
	  }
  /* USER CODE END StartBatteryMonitorTask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void const * argument)
{
  /* USER CODE BEGIN StartCANTask */
	  BatteryData_t batteryDataToSend;
	    CAN_TxHeaderTypeDef txHeader;
	    uint8_t canData[8];
	    uint32_t txMailbox;

	    // Configure CAN TX header
	    txHeader.StdId = CAN_TX_MSG_ID;       // Use defined message ID
	    txHeader.ExtId = 0;
	    txHeader.RTR = CAN_RTR_DATA;
	    txHeader.IDE = CAN_ID_STD;
	    txHeader.DLC = 8;                     // Using 6 bytes (SOC, SOH, Temp)
	    txHeader.TransmitGlobalTime = DISABLE;

	    for(;;) {
	        // Get the latest battery data (protected by mutex)
	        osMutexWait(batteryDataMutexHandle, osWaitForever);
	        memcpy(&batteryDataToSend, &batteryData, sizeof(BatteryData_t));
	        osMutexRelease(batteryDataMutexHandle);

	        // Prepare CAN message data
	        // Bytes 0-1: SOC (uint16_t, scaled by 100 for 0.01% resolution)
	        uint16_t soc_scaled = (uint16_t)(batteryDataToSend.soc * 100);
	        canData[0] = (soc_scaled >> 8) & 0xFF;
	        canData[1] = soc_scaled & 0xFF;

	        // Bytes 2-3: SOH (uint16_t, scaled by 100 for 0.01% resolution)
	        uint16_t soh_scaled = (uint16_t)(batteryDataToSend.soh * 100);
	        canData[2] = (soh_scaled >> 8) & 0xFF;
	        canData[3] = soh_scaled & 0xFF;

	        // Bytes 4-5: Temperature (uint16_t, scaled by 100 for 0.01°C resolution)
	        uint16_t temp_scaled = (uint16_t)(batteryDataToSend.temperature * 100);
	        canData[4] = (temp_scaled >> 8) & 0xFF;
	        canData[5] = temp_scaled & 0xFF;

	        // Bytes 6-7: FRAME Identifier bytes
	         canData[6] = 0xAA;  // Frame identifier
	         canData[7] = 0x01;  // Frame 1 marker

	        // Send CAN message
	        HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, canData, &txMailbox);

	        /* Frame 2: Voltage and Current (4 bytes) + padding + Frame identifier bytes*/
	        int16_t voltage_scaled = (int16_t)(batteryDataToSend.voltage * 100);  // 0.01V resolution
	        int16_t current_scaled = (int16_t)(batteryDataToSend.current * 1000);   // 1mA resolution

	        canData[0] = (voltage_scaled >> 8) & 0xFF;
	        canData[1] = voltage_scaled & 0xFF;
	        canData[2] = (current_scaled >> 8) & 0xFF;
	        canData[3] = current_scaled & 0xFF;
	        canData[4] = 0x00;  // Padding
	        canData[5] = 0x00;  // Padding
	        canData[6] = 0xAA;  // Frame identifier
	        canData[7] = 0x02;  // Frame 2 marker

	        HAL_CAN_AddTxMessage(&hcan1, &txHeader, canData, &txMailbox);

	        osDelay(100);  // Adjust delay as needed
	        if (status != HAL_OK) {
	            // Optional: Add error handling or retry logic here
	        }
	        else{
	        	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	        }

	        // Wait before sending next message (e.g., 100ms)
	        osDelay(100);
	    }
  /* USER CODE END StartCANTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
