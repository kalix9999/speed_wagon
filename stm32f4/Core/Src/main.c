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
#include <stdio.h>
#include "arm_math.h"  // DSP 라이브러리 (FFT용)
#include "fnd.h"
#include "lcd.h"
#include "menu.h"
#include <stdbool.h>
#include <math.h> // 해밍윈도우 M_PI 사용을 위해 추가


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_LEN 512       // 128, 256, 512, 1024, 2048 중 선택 (1024 권장)
#define SAMPLE_RATE 10000  // 10kHz (타이머 설정에 맞춤)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

osThreadId FFTTaskHandle;
osThreadId menuTaskHandle;
osMessageQId buttonQueueHandle;
osSemaphoreId adcBinarySemHandle;
/* USER CODE BEGIN PV */

SPI_HandleTypeDef hspi2;
Packet tx_data; // 보낼 ?  ?  ?  
Packet rx_data; // 받을 ?  ?  ?   (Master ? 보내?   Dummy  ? ???  ?  )
void Update_Sensor_Data_SPI(bool is_overspeed, int speed);

// 1. ADC 버퍼 (DMA가 데이터를 채워넣는 곳)
// uint16_t 타입이어야 합니다 (ADC가 12비트 정수이므로)
uint16_t adc_buffer[FFT_LEN * 2];

// 2. Q15 FFT용 버퍼들
// 입력/출력 버퍼는 FFT 길이의 2배가 필요합니다 (실수부 + 허수부)
q15_t fft_input_q15[FFT_LEN * 2];
q15_t fft_output_q15[FFT_LEN * 2];
q15_t fft_mag_q15[FFT_LEN];      // 최종 크기(Magnitude) 결과
volatile uint32_t process_offset = 0; // 태스크가 읽어야 할 위치 (0 또는 1024)

// [SWV 관찰용 전역 변수]
volatile uint32_t debug_speed = 0;   // 계산된 속도
volatile uint32_t debug_speed_x10 = 0;
volatile uint32_t debug_freq = 0;    // 계산된 주파수
volatile int32_t debug_maxVal = 0;   // 신호 세기 (Magnitude)
volatile uint32_t debug_isr_cnt = 0; // 인터럽트 횟수 카운터
volatile q15_t debug_mag = 0; // 푸리에 편환 주파수별 세기 그래프 보기용도
volatile uint32_t debug_adc_raw = 0;
volatile uint32_t debug_accumulated = 0; // SNR 누적 확인용
volatile uint32_t debug_fft_mag = 0;
char debug_buffer[100]; // 디버그 출력

volatile uint32_t TH_OVERSPEED_km_h = 30;
volatile uint32_t TH_NOISE = 500;

#define ACC_FRAMES 3 // 누적 (너무 많이하면 반응 느려짐)
int32_t fft_accumulated[FFT_LEN] = {0}; // 누적용 버퍼
int acc_count = 0;

q15_t hanning_window[FFT_LEN]; // 해밍윈도우 0~32,000
// FFT 구조체 인스턴스
arm_rfft_instance_q15 S;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C3_Init(void);
void StartFFTTask(void const * argument);
void StartMenuTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
 // Write character to ITM ch.0
 ITM_SendChar(ch);
 return(ch);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  FND_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  for (int i = 0; i < FFT_LEN; i++) {
      // 0.5 * (1 - cos(2*pi*n / (N-1))) 공식을 Q15 포맷으로 변환
      // 32767은 Q15의 1.0에 해당
      float32_t val = 0.5f * (1.0f - cosf(2.0f * 3.141592f * (float)i / (float)(FFT_LEN - 1)));
      hanning_window[i] = (q15_t)(val * 32767.0f);
  }
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of adcBinarySem */
  osSemaphoreDef(adcBinarySem);
  adcBinarySemHandle = osSemaphoreCreate(osSemaphore(adcBinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of buttonQueue */
  osMessageQDef(buttonQueue, 16, uint16_t);
  buttonQueueHandle = osMessageCreate(osMessageQ(buttonQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of FFTTask */
  osThreadDef(FFTTask, StartFFTTask, osPriorityHigh, 0, 2048);
  FFTTaskHandle = osThreadCreate(osThread(FFTTask), NULL);

  /* definition and creation of menuTask */
  osThreadDef(menuTask, StartMenuTask, osPriorityNormal, 0, 128);
  menuTaskHandle = osThreadCreate(osThread(menuTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 998;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|seg_12_Pin|seg_9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, buz_Pin|seg_8_Pin|seg_10_Pin|seg_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, seg_6_Pin|seg_5_Pin|seg_11_Pin|seg_1_Pin
                          |seg_3_Pin|seg_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(seg_4_GPIO_Port, seg_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin seg_12_Pin seg_9_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|seg_12_Pin|seg_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : sw_down_Pin */
  GPIO_InitStruct.Pin = sw_down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sw_down_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : sw_up_Pin sw_back_Pin */
  GPIO_InitStruct.Pin = sw_up_Pin|sw_back_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : buz_Pin */
  GPIO_InitStruct.Pin = buz_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(buz_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : seg_6_Pin seg_5_Pin seg_11_Pin seg_1_Pin
                           seg_3_Pin seg_2_Pin */
  GPIO_InitStruct.Pin = seg_6_Pin|seg_5_Pin|seg_11_Pin|seg_1_Pin
                          |seg_3_Pin|seg_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : sw_ok_Pin */
  GPIO_InitStruct.Pin = sw_ok_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sw_ok_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : seg_4_Pin */
  GPIO_InitStruct.Pin = seg_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(seg_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : seg_8_Pin seg_10_Pin seg_7_Pin */
  GPIO_InitStruct.Pin = seg_8_Pin|seg_10_Pin|seg_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 1. 절반 완료 (Half Complete) -> 앞부분 데이터 처리 요청 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        process_offset = 0; // "앞부분(0번지)부터 읽어라"
        osSemaphoreRelease(adcBinarySemHandle); // 태스크 깨우기 🚩
    }
}

/* 2. 전체 완료 (Full Complete) -> 뒷부분 데이터 처리 요청 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        debug_isr_cnt++;

        process_offset = FFT_LEN; // "뒷부분(1024번지)부터 읽어라"
        osSemaphoreRelease(adcBinarySemHandle); // 태스크 깨우기 🚩

    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPI2)
  {
    HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t*)&tx_data, (uint8_t*)&rx_data, sizeof(Packet));
  }
}

void Update_Sensor_Data_SPI(bool _is_overspeed, int _speed){
	tx_data.is_overspeed = _is_overspeed;
	tx_data.speed = _speed;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t last_tick = 0;
	if (HAL_GetTick() - last_tick < 200) return;
	last_tick = HAL_GetTick();
	osMessagePut(buttonQueueHandle, (uint32_t)GPIO_Pin, 0);

    if (GPIO_Pin == sw_ok_Pin)
    {

    }
    else if (GPIO_Pin == sw_up_Pin)
    {

    }
    else if (GPIO_Pin == sw_down_Pin)
    {

    }
    else if (GPIO_Pin == sw_back_Pin)
    {

    }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartFFTTask */
/**
  * @brief  Function implementing the FFTTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartFFTTask */
void StartFFTTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  arm_rfft_init_q15(&S, FFT_LEN, 0, 1);

  // 버퍼 크기 2배로 설정 (Ping-Pong)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FFT_LEN * 2);
  HAL_TIM_Base_Start(&htim3);
  HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t*)&tx_data, (uint8_t*)&rx_data, sizeof(Packet));

  /* Infinite loop */
  for(;;)
  {
	  // [B] 신호 대기 (인터럽트가 깨울 때까지 잠듦) 💤
	  if (osSemaphoreWait(adcBinarySemHandle, osWaitForever) == osOK)
	  {
		  // [C] 데이터 복사 (Ping-Pong 로직) 🏓
		  // process_offset 변수가 가리키는 곳(0 또는 1024)에서 데이터를 가져옵니다.

		  // DC 제거 및 Q15 변환
		  uint32_t sum = 0;
		  // 평균 구하기 (DC 오프셋)
		  for (int i = 0; i < FFT_LEN; i++) {
			  sum += adc_buffer[process_offset + i];
//			  debug_adc_raw = adc_buffer[process_offset + i]; // 디버깅용
		  }
		  uint32_t dc_offset = sum / FFT_LEN; // 약 2000
		  printf("dc_offset: %lu \r\n", dc_offset);

		  // FFT 입력 버퍼로 복사
//		  for (int i = 0; i < FFT_LEN; i++) {
//			  int16_t val = (int16_t)adc_buffer[process_offset + i] - dc_offset;
//			  fft_input_q15[i] = (q15_t)(val << 3); // 값 증폭 (필요시 조정)
//		  }
		  // 해밍윈도우 추가
		  for (int i = 0; i < FFT_LEN; i++) {
				int16_t val = (int16_t)adc_buffer[process_offset + i] - dc_offset;

				// [중요] 값을 증폭(<<3)하기 전에 Window 함수를 곱해줍니다.
				// Q15 곱셈: (Signal * Window) >> 15
				int32_t windowed_val = ((int32_t)val * hanning_window[i]) >> 15;  //hanning_window: 0~32000==2^15

				// 그 후 증폭 (입력이 작다면)
				fft_input_q15[i] = (q15_t)(windowed_val << 3);
			}

		  // [A] FFT 계산 및 속도 출력
		  arm_rfft_q15(&S, fft_input_q15, fft_output_q15);
		  arm_cmplx_mag_q15(fft_output_q15, fft_mag_q15, FFT_LEN);


		  // [누적 로직 추가]
		  for(int i = 0; i < FFT_LEN; i++) {
			  if(acc_count == 0) fft_accumulated[i] = (int32_t)fft_mag_q15[i];
			  else fft_accumulated[i] += (int32_t)fft_mag_q15[i]; // 값 더하기

			  debug_fft_mag = fft_mag_q15[i];
			  if(acc_count == ACC_FRAMES-1) {
				  debug_accumulated = fft_accumulated[i] / ACC_FRAMES;
			  }
		  }
		  acc_count++;
		  if (acc_count < ACC_FRAMES) continue;
		  acc_count = 0;


		  // Peak 찾기 및 출력 로직
		  uint32_t maxVal = 0;
		  uint32_t maxIndex = 0;
		  int start_index = 6; // 저주파 노이즈 제거
//		  arm_max_q15(&fft_mag_q15[start_index], (FFT_LEN / 2) - start_index, &maxVal, &maxIndex);

//		  arm_max_q15(&fft_accumulated[start_index], (FFT_LEN / 2) - start_index, &maxVal, &maxIndex);
//		  maxIndex += start_index;

		  for (int i = start_index; i < FFT_LEN / 2; i++)
		  {
			  uint32_t avg_val = fft_accumulated[i] / ACC_FRAMES;
			  if(850 < avg_val && avg_val <950) continue; // 왜인지 모르겠지만 이구간에서 노이즈가 항상있음
		      if (avg_val > maxVal)
		      {
		          maxVal = avg_val;
		          maxIndex = i; // 여기서 'i'는 이미 start_index가 포함된 진짜 위치입니다.
		      }
		  }

		  debug_maxVal = maxVal;
//		  debug_mag = 1<<12;
		  if (maxVal > TH_NOISE) // 노이즈 임계값
		  {
			  uint32_t freq_hz = (maxIndex * SAMPLE_RATE) / FFT_LEN;
			  // 속도 = 주파수 / 44 (24.125GHz 기준)
			  uint32_t speed_x10 = (freq_hz * 10) / 44;

			  //세그먼트 출력
			  FND_SetNumber(speed_x10);

			  printf("Freq: %lu Hz, Speed: %lu.%lu km/h\r\n", freq_hz, speed_x10/10, speed_x10%10);
			  sprintf(debug_buffer, "Freq: %lu Hz, Speed: %lu.%lu km/h", freq_hz, speed_x10/10, speed_x10%10);
			  debug_speed = speed_x10/10;
			  debug_speed_x10 = speed_x10;

			  Update_Sensor_Data_SPI((speed_x10/10)>TH_OVERSPEED_km_h, speed_x10/10);
		  }
		  // fft결과 디버깅용
//		  for(volatile int i = 0 ; i<FFT_LEN; i++){
//			  debug_mag = fft_mag_q15[i];
//			  for(volatile int k=0; k<1000; k++); // 데이터 유실방지용
//		  }
//		  tx_data.is_overspeed
	  }
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMenuTask */
/**
* @brief Function implementing the menuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMenuTask */
void StartMenuTask(void const * argument)
{
  /* USER CODE BEGIN StartMenuTask */
	lcd_init();
	menu_init();

	lcd_put_cur(0, 0);  // 2. 첫 번째 줄 첫 칸으로 이동
	lcd_send_string("Hello STM32!"); // 3. 문자열 출력

	lcd_put_cur(1, 0);  // 4. 두 번째 줄로 이동
	lcd_send_string("I2C LCD Test");

	UI_State currentState = STATE_DASHBOARD;
	int currentMenuIdx = 0;
	osEvent event;
  /* Infinite loop */
  for(;;)
  {
	  event = osMessageGet(buttonQueueHandle, 100);

	  if (event.status == osEventMessage)
	  {
		  HAL_GPIO_WritePin(buz_GPIO_Port, buz_Pin, GPIO_PIN_SET);
		  osDelay(100);
		  HAL_GPIO_WritePin(buz_GPIO_Port, buz_Pin, GPIO_PIN_RESET);

		  uint16_t pin = (uint16_t)event.value.v;
//		  printf("pushed btn pin: %u \r\n", pin);
		  switch (currentState) {
			  case STATE_DASHBOARD:
				  if (pin == sw_ok_Pin) currentState = STATE_MENU_LIST; // OK 누르면 메뉴 진입
				  break;

			  case STATE_MENU_LIST:
				  if (pin == sw_up_Pin) currentMenuIdx = (currentMenuIdx + 1) % MENU_COUNT;
				  else if (pin == sw_down_Pin) currentMenuIdx = (currentMenuIdx + MENU_COUNT - 1) % MENU_COUNT;
				  else if (pin == sw_ok_Pin) currentState = STATE_SET_VALUE; // 값 설정 진입
				  else if (pin == sw_back_Pin) currentState = STATE_DASHBOARD; // 뒤로가기
				  break;

			  case STATE_SET_VALUE:
				  if (pin == sw_up_Pin){
					  *(menuItems[currentMenuIdx].target_value) += menuItems[currentMenuIdx].step;
					  if(*(menuItems[currentMenuIdx].target_value) < menuItems[currentMenuIdx].min)
						  *(menuItems[currentMenuIdx].target_value) = menuItems[currentMenuIdx].min;
					  else if(*(menuItems[currentMenuIdx].target_value) > menuItems[currentMenuIdx].max)
						  *(menuItems[currentMenuIdx].target_value) = menuItems[currentMenuIdx].max;
				  }
				  else if (pin == sw_down_Pin){
					  *(menuItems[currentMenuIdx].target_value) -= menuItems[currentMenuIdx].step;
					  if(*(menuItems[currentMenuIdx].target_value) < menuItems[currentMenuIdx].min)
						  *(menuItems[currentMenuIdx].target_value) = menuItems[currentMenuIdx].min;
					  else if(*(menuItems[currentMenuIdx].target_value) > menuItems[currentMenuIdx].max)
						  *(menuItems[currentMenuIdx].target_value) = menuItems[currentMenuIdx].max;
				  }
				  else if (pin == sw_ok_Pin || pin == sw_back_Pin) currentState = STATE_MENU_LIST; // 저장/취소 후 복귀
				  break;
		  }
	  }
	  UpdateLCD(currentState, currentMenuIdx);
  }
  /* USER CODE END StartMenuTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2) {
//	  printf("tim2 alive\r\n");
	  FND_Update();
  }
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
