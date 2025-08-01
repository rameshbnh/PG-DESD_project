/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
//#include "stm32f4xx_hal_adc_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float temperature;
    uint32_t timestamp;
} TempData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHEEL_CIRCUMFERENCE 0.314f   // Wheel circumference in meters
#define PULSES_PER_REVOLUTION 1      // Number of pulses per wheel revolution

#define CAN_TX_ID 0x123       // CAN message ID
#define TEMP_QUEUE_SIZE 5     // Size of temperature data queue
#define ADC_SAMPLE_RATE_MS 1000 // Sampling rate in milliseconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

osThreadId DefaultTaskHandle;
osThreadId adcTaskHandle;
osThreadId canTaskHandle;
osThreadId Engine_RPM_SHandle;
osMutexId rpmMutexHandle;
/* USER CODE BEGIN PV */
osMessageQId tempQueueHandle; // Queue handle for temperature data
uint32_t temp;
uint32_t temptime;

volatile uint32_t pulseCount = 0;
volatile uint32_t lastTime = 0;
volatile uint16_t rpm = 0;
volatile uint16_t speed = 0;       // Speed in m/s
volatile uint16_t newSpeed = 0;    // Speed in Km/h
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void StartTask(void const * argument);
void StartADCTask(void const * argument);
void StartCANTask(void const * argument);
void StartEngineTask(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void calculateRPM(void);
void sendCANData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) { // Hall sensor connected to PA0
        pulseCount++;
    }
}

void calculateRPM() {
    uint32_t currentTime = HAL_GetTick();
    uint32_t elapsedTime = currentTime - lastTime;

    if (elapsedTime >= 1000) { // Calculate RPM every second
        rpm = (pulseCount * 60) / PULSES_PER_REVOLUTION;
        speed = ((float)rpm / 60.0f) * WHEEL_CIRCUMFERENCE;
        newSpeed = speed * 3.6f; // Convert m/s to km/h
        pulseCount = 0;
        lastTime = currentTime;

        // Send data via CAN
        //sendCANData();
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
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

  /* USER CODE BEGIN RTOS_MUTEX */
  osMutexDef(rpmMutex);
  rpmMutexHandle = osMutexCreate(osMutex(rpmMutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  osMessageQDef(tempQueue, TEMP_QUEUE_SIZE, TempData_t);
  tempQueueHandle = osMessageCreate(osMessageQ(tempQueue), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DefaultTask */
  osThreadDef(DefaultTask, StartTask, osPriorityIdle, 0, 128);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* definition and creation of adcTask */
  osThreadDef(adcTask, StartADCTask, osPriorityAboveNormal, 0, 256);
  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, StartCANTask, osPriorityNormal, 0, 256);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* definition and creation of Engine_RPM_S */
  osThreadDef(Engine_RPM_S, StartEngineTask, osPriorityHigh, 0, 256);
  Engine_RPM_SHandle = osThreadCreate(osThread(Engine_RPM_S), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){
	 // temp=StartADCTask();
  }
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  //GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
	// Add in MX_GPIO_Init():
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {1};
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask */
/**
  * @brief  Function implementing the DefaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask */
void StartTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	  /* Initialize lastTime */
		    lastTime = HAL_GetTick();

		    /* Start timer interrupt */
		    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

		    /* Start CAN */
		    HAL_CAN_Start(&hcan1);

		    /* Infinite loop */
		    for(;;) {
		        osDelay(1000);
		    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartADCTask */
/* USER CODE END Header_StartADCTask */
void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */
	// Constants (compile-time optimized)
	    const float vRef = 3.3f;               // Must match your actual VREF
	    const float scaleFactor = vRef * 100.0f / 4095.0f;

	    // Static variables
	    static TempData_t tempData;
	    static uint32_t adcValue;
	    static uint32_t validReadings = 0;

	    // Initialize with dummy reads
	    for(uint8_t i=0; i<5; i++) {
	        HAL_ADC_Start(&hadc1);
	        HAL_ADC_PollForConversion(&hadc1, 1);
	        HAL_ADC_GetValue(&hadc1);
	        osDelay(10);
	    }

	    for(;;)
	    {
	        HAL_ADC_Start(&hadc1);

	        if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	        {
	            adcValue = HAL_ADC_GetValue(&hadc1);

	            /* Robust validation checks */
	            if(adcValue < 10) {  // Minimum expected raw value (~0.08V = 8Â°C)
	                tempData.temperature = 0.0f;  // Treat as disconnected
	            }
	            else {
	                // Only consider stable readings
	                float newTemp = adcValue * scaleFactor;

	                // Validate plausible temperature range
	                if(newTemp >= 0.0f && newTemp <= 150.0f) {
	                    tempData.temperature = newTemp;
	                    validReadings++;
	                }
	            }
	            temp=tempData.temperature;

	            // Only send data after 5 valid readings
	            if(validReadings > 5) {
	                tempData.timestamp = osKernelSysTick();
	                osMessagePut(tempQueueHandle, (uint32_t)&tempData, 0);
	            }

	            HAL_ADC_Start(&hadc1);  // Restart immediately
	        }
	        osDelay(200);  // 5Hz update rate
	    }
  /* USER CODE END StartADCTask */
}

/* USER CODE BEGIN Header_StartCANTask */
/* USER CODE END Header_StartCANTask */
void StartCANTask(void const * argument)
{
  /* USER CODE BEGIN StartCANTask */
	osEvent event;
		    TempData_t *tempData;
		    CAN_TxHeaderTypeDef txHeader = {
		        .StdId = CAN_TX_ID,
		        .ExtId = 0,
		        .RTR = CAN_RTR_DATA,
		        .IDE = CAN_ID_STD,
		        .DLC = 8,        // 8 bytes total
		        .TransmitGlobalTime = DISABLE
		    };
		    uint8_t canData[8];
		    uint32_t txMailbox;

		    for(;;) {
		        // Wait for temperature data
		        event = osMessageGet(tempQueueHandle, osWaitForever);

		        if (event.status == osEventMessage) {
		            tempData = (TempData_t *)event.value.p;

		            // Prepare all data in the CAN message
		            // Bytes 0-1: RPM (uint16_t)
		            canData[0] = (rpm >> 8) & 0xFF;
		            canData[1] = rpm & 0xFF;

		            // Bytes 2-3: Speed (uint16_t km/h)
		            canData[2] = (newSpeed >> 8) & 0xFF;
		            canData[3] = newSpeed & 0xFF;

		            // Bytes 4-5: Temperature (uint16_t °C)
		            uint16_t temp_int = (uint16_t)(tempData->temperature);
		            canData[4] = (temp_int >> 8) & 0xFF;
		            canData[5] = temp_int & 0xFF;

		            // Bytes 6-7: Timestamp (uint16_t - truncated from uint32_t)
		            canData[6] = (tempData->timestamp >> 8) & 0xFF;
		            canData[7] = tempData->timestamp & 0xFF;

		            // Send CAN message
		            if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, canData, &txMailbox) != HAL_OK) {
		                // Error handling could be added here
		            }

		            // Optional debug output
		            // printf("Sent - RPM: %d, Speed: %d km/h, Temp: %d°C\n", rpm, newSpeed, temp_int);
		        }
		    }
  /* USER CODE END StartCANTask */
}

/* USER CODE BEGIN Header_StartEngineTask */
/**
  * @brief  Function implementing the Engine_RPM_spee thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEngineTask */
void StartEngineTask(void const * argument)
{
  /* USER CODE BEGIN StartEngineTask */
	/* Infinite loop */
		    for(;;) {
		        osMutexWait(rpmMutexHandle, osWaitForever);
		        calculateRPM();
		        osMutexRelease(rpmMutexHandle);

		        osDelay(100); // 100ms delay for RPM calculation
		    }
  /* USER CODE END StartEngineTask */
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
