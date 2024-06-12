/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MFS.h"
#include "customer.h"
#include "teller.h"

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
RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 180 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TellerTask01 */
osThreadId_t TellerTask01Handle;
const osThreadAttr_t TellerTask01_attributes = {
  .name = "TellerTask01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TellerTask02 */
osThreadId_t TellerTask02Handle;
const osThreadAttr_t TellerTask02_attributes = {
  .name = "TellerTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TellerTask03 */
osThreadId_t TellerTask03Handle;
const osThreadAttr_t TellerTask03_attributes = {
  .name = "TellerTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for customerQueueMutex */
osMutexId_t customerQueueMutexHandle;
const osMutexAttr_t customerQueueMutex_attributes = {
  .name = "customerQueueMutex"
};
/* USER CODE BEGIN PV */
/*Segment byte maps for numbers 0 to 9 */
const char SEGMENT_MAP[] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0X80,0X90};

/* Byte maps to select digit 1 to 4 */
const char SEGMENT_SELECT[] = {0xF1,0xF2,0xF4,0xF8};

SemaphoreHandle_t teller_break01 = NULL;
SemaphoreHandle_t teller_break02 = NULL;
SemaphoreHandle_t teller_break03 = NULL;

int bank_closed = 0;






/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void *argument);
void StartTellerTask01(void *argument);
void StartTellerTask02(void *argument);
void StartTellerTask03(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  teller_break01 = xSemaphoreCreateMutex();
  teller_break02 = xSemaphoreCreateMutex();
  teller_break03 = xSemaphoreCreateMutex();


  HAL_TIM_Base_Start(&htim2);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of customerQueueMutex */
  customerQueueMutexHandle = osMutexNew(&customerQueueMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TellerTask01 */
  TellerTask01Handle = osThreadNew(StartTellerTask01, NULL, &TellerTask01_attributes);

  /* creation of TellerTask02 */
  TellerTask02Handle = osThreadNew(StartTellerTask02, NULL, &TellerTask02_attributes);

  /* creation of TellerTask03 */
  TellerTask03Handle = osThreadNew(StartTellerTask03, NULL, &TellerTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  //initialize queue thread
  customer_queue_init();
  teller_init(NUM_TELLERS);
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
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  htim2.Init.Prescaler = 79;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|SHLD_D7_SEG7_Clock_Pin
                          |SHLD_D8_SEG7_Data_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHLD_D4_SEG7_Latch_Pin|LED4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_A5_Pin SHLD_A4_Pin */
  GPIO_InitStruct.Pin = SHLD_A5_Pin|SHLD_A4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_A0_Pin SHLD_D2_Pin */
  GPIO_InitStruct.Pin = SHLD_A0_Pin|SHLD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_A1_Pin SHLD_A2_Pin */
  GPIO_InitStruct.Pin = SHLD_A1_Pin|SHLD_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin SHLD_D7_SEG7_Clock_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|SHLD_D7_SEG7_Clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_A3_Pin */
  GPIO_InitStruct.Pin = SHLD_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SHLD_A3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_D6_Pin SHLD_D5_Pin */
  GPIO_InitStruct.Pin = SHLD_D6_Pin|SHLD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_D9_Pin */
  GPIO_InitStruct.Pin = SHLD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SHLD_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_D8_SEG7_Data_Pin */
  GPIO_InitStruct.Pin = SHLD_D8_SEG7_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SHLD_D8_SEG7_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_D4_SEG7_Latch_Pin */
  GPIO_InitStruct.Pin = SHLD_D4_SEG7_Latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SHLD_D4_SEG7_Latch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_D15_Pin SHLD_D14_Pin */
  GPIO_InitStruct.Pin = SHLD_D15_Pin|SHLD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * Gererates a distance number between the max and min intergers
 */
int random_distance(int min, int max){
	uint32_t rnum = 0;
	HAL_RNG_GenerateRandomNumber(&hrng, &rnum);
	return min + (max-min)*(double)(rnum/4294967295.0);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	osThreadId id;                                           // id for the currently running thread
	id = osThreadGetId ();
	char buffer[128];
	int len;
	//int i = 1000;
	//TickType_t xDelay;
	len = sprintf(buffer, "\r\nBANK IS OPEN.\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);

  /* Infinite loop */
  for(;;)
  {
	  int sim_time_sec = US_TO_SIM_SEC(htim2.Instance->CNT);
	  if (!bank_closed && (sim_time_sec > BANK_HOURS*60*60)){
		  //close the bank
		  bank_closed = 1;
		  //Delete Customer task so no more customers are added
		  vTaskDelete(customer_queue.handle);

		  len = sprintf(buffer, "\r\nBank is Closed!\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t *) buffer, len, ~0);
	  }
	  if (bank_closed && customer_queue.len == 0 && teller_end(NUM_TELLERS)){
		  len = sprintf(buffer, "\r\nFINAL METRICS:");
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tTotal Customers Served: %d", customer_queue.num_served);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tCustomers Served by Teller: T1: %d, T2: %d, T3: %d", tellers[0].customers_served,tellers[1].customers_served, tellers[2].customers_served);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tAverage Time Customers Spent in Queue: %.2lfsec", (double)customer_queue.total_wait_time/(double)customer_queue.num_served);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tAverage Time Customers Spent with Tellers: T1:%.2lfsec, T2:%.2lfsec, T3:%.2lfsec", (double)tellers[0].total_transaction_time/(double)tellers[0].customers_served,
				  (double)tellers[1].total_transaction_time/(double)tellers[1].customers_served,
				  (double)tellers[2].total_transaction_time/(double)tellers[2].customers_served);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tAverage Time Tellers Waited for Customers: T1:%.2lfsec, T2:%.2lfsec, T3:%.2lfsec", (double)tellers[0].total_wait_time/(double)tellers[0].customers_served,
				  (double)tellers[1].total_wait_time/(double)tellers[1].customers_served,
				  (double)tellers[2].total_wait_time/(double)tellers[2].customers_served);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tNumber of breaks: T1:%d, T2:%d, T3:%d ", tellers[0].num_breaks, tellers[1].num_breaks, tellers[2].num_breaks);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  len = sprintf(buffer, "\r\n\tAverage time for breaks: T1:%.2lfsec, T2:%.2lfsec, T3:%.2lfsec", (double)tellers[0].total_break_time/(double)tellers[0].num_breaks,
				  (double)tellers[1].total_break_time/(double)tellers[1].num_breaks,
				  (double)tellers[2].total_break_time/(double)tellers[2].num_breaks);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  len = sprintf(buffer, "\r\n\tMax break time: T1:%dsec, T2:%dsec, T3:%dsec", tellers[0].max_break_time, tellers[1].max_break_time, tellers[2].max_break_time);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  len = sprintf(buffer, "\r\n\tMin break time: T1:%dsec, T2:%dsec, T3:%dsec", tellers[0].min_break_time, tellers[1].min_break_time, tellers[2].min_break_time);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);


		  len = sprintf(buffer, "\r\n\tMax Customer Wait Time in Queue: %dsec", customer_queue.max_wait_time);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tMax Teller Wait Time for Customer: T1:%dsec, T2:%dsec, T3:%dsec", tellers[0].max_wait_time, tellers[1].max_wait_time, tellers[2].max_wait_time);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tMax Teller Transaction Time: T1:%dsec, T2:%dsec, T3:%dsec", tellers[0].max_transaction_time, tellers[1].max_transaction_time, tellers[2].max_transaction_time);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tMax Customer Queue Depth: %d", customer_queue.max_len);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  HAL_Delay(10);
		  len = sprintf(buffer, "\r\n\tFinal idle count: %ld", app_idle_cnt);
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		  osThreadTerminate(id);
	  }
	  else {
		  displayNum(customer_queue.len);
		  int hour = sim_time_sec/60/60;
		  int min = sim_time_sec/60 - hour*60;
		  int sec = sim_time_sec - hour*60*60 - min*60;
		  hour += BANK_OPEN;
		  len = sprintf(buffer, "\r\n%02d:%02d:%02d\tWaiting:%d\r\n", hour,min,sec,customer_queue.len);
		  HAL_UART_Transmit(&huart2, (uint8_t *) buffer, len, ~0);
		  //xDelay = (SIM_SEC_TO_US(240)* .001)/portTICK_PERIOD_MS;
		  //osDelay(1);
		  for (int i = 0; i < NUM_TELLERS; i++) {
			  len += sprintf(buffer+len, "\t|T%d,Served:%d,%s|", i + 1, tellers[i].customers_served, get_status_name(tellers[i].status));
		  }
		  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
	  }

	  //osDelay(((uint32_t)SIM_SEC_TO_US(240)/ 1000) * 1000 / configTICK_RATE_HZ);
	  osDelay(100);
	  //vTaskDelay(2000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTellerTask01 */
/**
* @brief Function implementing the TellerTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTellerTask01 */
void StartTellerTask01(void *argument)
{
  /* USER CODE BEGIN StartTellerTask01 */
	osThreadId id;                                           // id for the currently running thread
	id = osThreadGetId ();
	TELLER *t = &tellers[0];

  /* Infinite loop */
	t->wait_start = htim2.Instance->CNT;
  for(;;)
  {
	if (check_btn1()){
		t->status = BREAK;
	}
	else{
		t->status=AVAIL;
		if (t->break_set && htim2.Instance->CNT >= t->next_break){
			uint32_t break_len = RAND_BREAK_LENGTH;
			t->status = BREAK;
			osDelay(SIM_SEC_TO_US(break_len) / 1000);
			t->status = AVAIL;
			t->next_break = htim2.Instance->CNT + SIM_SEC_TO_US(RAND_NEXT_BREAK);
			t->total_break_time += break_len;
			t->num_breaks++;
			if (break_len > t->max_break_time){
				t->max_break_time = break_len;
			}
			if (break_len < t->min_break_time){
				t->min_break_time = break_len;
			}
		}
		else{
			int err = customer_queue_pop();
			if(!err){
				uint32_t wait_time = htim2.Instance->CNT - t->wait_start;
				wait_time = US_TO_SIM_SEC(wait_time);
				t->total_wait_time += wait_time;
				if (wait_time > t->max_wait_time){
					t->max_wait_time = wait_time;
				}
				t->wait_start = 0;
				t->status = BUSY;
				int transaction_time = RAND_TELLER_TIME;
				t->total_transaction_time += transaction_time;
				if (transaction_time > t->max_transaction_time){
					t->max_transaction_time = transaction_time;
				}
				osDelay(SIM_SEC_TO_US(transaction_time)/1000);
				t->wait_start = htim2.Instance->CNT;
				t->customers_served++;
				t->status = AVAIL;
				if (!t->break_set){
					t->next_break = htim2.Instance->CNT + SIM_SEC_TO_US(RAND_NEXT_BREAK);
					t->break_set = 1;
				}
			}
		}

	}
	int sim_time_sec = US_TO_SIM_SEC(htim2.Instance->CNT);
	if ((sim_time_sec > BANK_HOURS*60*60) && customer_queue.len == 0){
		osThreadTerminate(id);
	}
    osDelay(1);
  }
  /* USER CODE END StartTellerTask01 */
}

/* USER CODE BEGIN Header_StartTellerTask02 */
/**
* @brief Function implementing the TellerTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTellerTask02 */
void StartTellerTask02(void *argument)
{
  /* USER CODE BEGIN StartTellerTask02 */
	osThreadId id;                                           // id for the currently running thread
	id = osThreadGetId ();
	TELLER *t = &tellers[1];
  /* Infinite loop */
	t->wait_start = htim2.Instance->CNT;
  for(;;)
  {
	  if (check_btn2())
	  {
		t->status = BREAK;
	  }
	  else{
		t->status=AVAIL;
		if (t->break_set && htim2.Instance->CNT >= t->next_break){
			uint32_t break_len = RAND_BREAK_LENGTH;
			t->status = BREAK;
			osDelay(SIM_SEC_TO_US(break_len) / 1000);
			t->status = AVAIL;
			t->next_break = htim2.Instance->CNT + SIM_SEC_TO_US(RAND_NEXT_BREAK);
			t->total_break_time += break_len;
			t->num_breaks++;
			if (break_len > t->max_break_time){
				t->max_break_time = break_len;
			}
			if (break_len < t->min_break_time){
				t->min_break_time = break_len;
			}
		}
		else{
			int err = customer_queue_pop();
			if(!err){
				uint32_t wait_time = htim2.Instance->CNT - t->wait_start;
				wait_time = US_TO_SIM_SEC(wait_time);
				t->total_wait_time += wait_time;
				if (wait_time > t->max_wait_time){
					t->max_wait_time = wait_time;
				}
				t->wait_start = 0;
				t->status = BUSY;
				int transaction_time = RAND_TELLER_TIME;
				t->total_transaction_time += transaction_time;
				if (transaction_time > t->max_transaction_time){
					t->max_transaction_time = transaction_time;
				}
				osDelay(SIM_SEC_TO_US(transaction_time)/1000);
				t->wait_start = htim2.Instance->CNT;
				t->customers_served++;
				t->status = AVAIL;
				if (!t->break_set){
					t->next_break = htim2.Instance->CNT + SIM_SEC_TO_US(RAND_NEXT_BREAK);
					t->break_set = 1;
				}
			}
		}

	}
	int sim_time_sec = US_TO_SIM_SEC(htim2.Instance->CNT);
	if ((sim_time_sec > BANK_HOURS*60*60) && customer_queue.len == 0){
		osThreadTerminate(id);
    osDelay(1);
  }
  }
  /* USER CODE END StartTellerTask02 */
}

/* USER CODE BEGIN Header_StartTellerTask03 */
/**
* @brief Function implementing the TellerTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTellerTask03 */
void StartTellerTask03(void *argument)
{
  /* USER CODE BEGIN StartTellerTask03 */
	osThreadId id;                                           // id for the currently running thread
	id = osThreadGetId ();
	TELLER *t = &tellers[2];
  /* Infinite loop */
	t->wait_start = htim2.Instance->CNT;
  for(;;)
  {
	  if (check_btn3()){
	  		t->status = BREAK;
	  	}
	  	else{
	  		t->status=AVAIL;
	  		if (t->break_set && htim2.Instance->CNT >= t->next_break){
	  			uint32_t break_len = RAND_BREAK_LENGTH;
	  			t->status = BREAK;
	  			osDelay(SIM_SEC_TO_US(break_len) / 1000);
	  			t->status = AVAIL;
	  			t->next_break = htim2.Instance->CNT + SIM_SEC_TO_US(RAND_NEXT_BREAK);
	  			t->total_break_time += break_len;
	  			t->num_breaks++;
	  			if (break_len > t->max_break_time){
	  				t->max_break_time = break_len;
	  			}
	  			if (break_len < t->min_break_time){
	  				t->min_break_time = break_len;
	  			}
	  		}
	  		else{
	  			int err = customer_queue_pop();
	  			if(!err){
	  				uint32_t wait_time = htim2.Instance->CNT - t->wait_start;
	  				wait_time = US_TO_SIM_SEC(wait_time);
	  				t->total_wait_time += wait_time;
	  				if (wait_time > t->max_wait_time){
	  					t->max_wait_time = wait_time;
	  				}
	  				t->wait_start = 0;
	  				t->status = BUSY;
	  				int transaction_time = RAND_TELLER_TIME;
	  				t->total_transaction_time += transaction_time;
	  				if (transaction_time > t->max_transaction_time){
	  					t->max_transaction_time = transaction_time;
	  				}
	  				osDelay(SIM_SEC_TO_US(transaction_time)/1000);
	  				t->wait_start = htim2.Instance->CNT;
	  				t->customers_served++;
	  				t->status = AVAIL;
	  				if (!t->break_set){
	  					t->next_break = htim2.Instance->CNT + SIM_SEC_TO_US(RAND_NEXT_BREAK);
	  					t->break_set = 1;
	  				}
	  			}
	  		}

	  	}
	  	int sim_time_sec = US_TO_SIM_SEC(htim2.Instance->CNT);
	  	if ((sim_time_sec > BANK_HOURS*60*60) && customer_queue.len == 0){
	  		osThreadTerminate(id);
    osDelay(1);
  }
  }
  /* USER CODE END StartTellerTask03 */
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
