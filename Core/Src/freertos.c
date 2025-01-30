/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "i2c.h"
#include "printf.h"
#include "usart.h"
#include "SSD1306_OLED.h"
#include "GFX_BW.h"
#include "font_8x5.h"
#include "fonts.h"
#include "arm_math.h"
#include "tim.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SAMPLES 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef struct
{
float Pressure;
float Temperature;
}BmpData_t;

typedef struct
{
	uint8_t OutFreqArray[10];
} FftData_t;

/* USER CODE END Variables */
/* Definitions for HeartBeatTask */
osThreadId_t HeartBeatTaskHandle;
const osThreadAttr_t HeartBeatTask_attributes = {
  .name = "HeartBeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Bmp_280Task */
osThreadId_t Bmp_280TaskHandle;
const osThreadAttr_t Bmp_280Task_attributes = {
  .name = "Bmp_280Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FFTTask */
osThreadId_t FFTTaskHandle;
const osThreadAttr_t FFTTask_attributes = {
  .name = "FFTTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for QueueBmpData */
osMessageQueueId_t QueueBmpDataHandle;
const osMessageQueueAttr_t QueueBmpData_attributes = {
  .name = "QueueBmpData"
};
/* Definitions for QueueFFTData */
osMessageQueueId_t QueueFFTDataHandle;
const osMessageQueueAttr_t QueueFFTData_attributes = {
  .name = "QueueFFTData"
};
/* Definitions for TimerBmpData */
osTimerId_t TimerBmpDataHandle;
const osTimerAttr_t TimerBmpData_attributes = {
  .name = "TimerBmpData"
};
/* Definitions for MutexPrintf */
osMutexId_t MutexPrintfHandle;
const osMutexAttr_t MutexPrintf_attributes = {
  .name = "MutexPrintf"
};
/* Definitions for MutexI2C1Hendle */
osMutexId_t MutexI2C1HendleHandle;
const osMutexAttr_t MutexI2C1Hendle_attributes = {
  .name = "MutexI2C1Hendle"
};
/* Definitions for MutexBmpDataHandle */
osMutexId_t MutexBmpDataHandleHandle;
const osMutexAttr_t MutexBmpDataHandle_attributes = {
  .name = "MutexBmpDataHandle"
};
/* Definitions for MutexBmpData */
osMutexId_t MutexBmpDataHandle;
const osMutexAttr_t MutexBmpData_attributes = {
  .name = "MutexBmpData"
};
/* Definitions for SemaphoreBmpQueue */
osSemaphoreId_t SemaphoreBmpQueueHandle;
const osSemaphoreAttr_t SemaphoreBmpQueue_attributes = {
  .name = "SemaphoreBmpQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void _putchar(char character)
{
  // send char to console etc.
	osMutexAcquire(MutexPrintfHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, (uint8_t*)&character, 1, 1000);
	osMutexRelease(MutexPrintfHandle);
}

float complexABS(float real, float compl) {
	return sqrtf(real*real+compl*compl);
}

/* USER CODE END FunctionPrototypes */

void StartHeartBeatTask(void *argument);
void StartBmp_280Task(void *argument);
void StartOledTask(void *argument);
void StartFFTTask(void *argument);
void TimerBmpDataCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */


}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexPrintf */
  MutexPrintfHandle = osMutexNew(&MutexPrintf_attributes);

  /* creation of MutexI2C1Hendle */
  MutexI2C1HendleHandle = osMutexNew(&MutexI2C1Hendle_attributes);

  /* creation of MutexBmpDataHandle */
  MutexBmpDataHandleHandle = osMutexNew(&MutexBmpDataHandle_attributes);

  /* creation of MutexBmpData */
  MutexBmpDataHandle = osMutexNew(&MutexBmpData_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreBmpQueue */
  SemaphoreBmpQueueHandle = osSemaphoreNew(1, 1, &SemaphoreBmpQueue_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerBmpData */
  TimerBmpDataHandle = osTimerNew(TimerBmpDataCallback, osTimerPeriodic, NULL, &TimerBmpData_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueBmpData */
  QueueBmpDataHandle = osMessageQueueNew (16, sizeof(BmpData_t), &QueueBmpData_attributes);

  /* creation of QueueFFTData */
  QueueFFTDataHandle = osMessageQueueNew (8, sizeof(FftData_t), &QueueFFTData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of HeartBeatTask */
  HeartBeatTaskHandle = osThreadNew(StartHeartBeatTask, NULL, &HeartBeatTask_attributes);

  /* creation of Bmp_280Task */
  Bmp_280TaskHandle = osThreadNew(StartBmp_280Task, NULL, &Bmp_280Task_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(StartOledTask, NULL, &OledTask_attributes);

  /* creation of FFTTask */
  FFTTaskHandle = osThreadNew(StartFFTTask, NULL, &FFTTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartBeatTask */
/**
  * @brief  Function implementing the HeartBeatTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartBeatTask */
void StartHeartBeatTask(void *argument)
{
  /* USER CODE BEGIN StartHeartBeatTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  osDelay(500);
  }
  /* USER CODE END StartHeartBeatTask */
}

/* USER CODE BEGIN Header_StartBmp_280Task */
/**
* @brief Function implementing the Bmp_280Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBmp_280Task */
void StartBmp_280Task(void *argument)
{
  /* USER CODE BEGIN StartBmp_280Task */
	BMP280_t Bmp280;
	BmpData_t _BmpData;
	uint32_t DelayTick = osKernelGetTickCount();


	osMutexAcquire(MutexI2C1HendleHandle, osWaitForever);
	BMP280_Init(&Bmp280, &hi2c1, 0x76);
	osMutexRelease(MutexI2C1HendleHandle);

	osTimerStart(TimerBmpDataHandle, 100);
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(MutexI2C1HendleHandle, osWaitForever);
	  BMP280_ReadPressureAndTemperature(&Bmp280, &_BmpData.Pressure, &_BmpData.Temperature);
	  osMutexRelease(MutexI2C1HendleHandle);

	  if(osOK == osSemaphoreAcquire(SemaphoreBmpQueueHandle, 0));
	  osMessageQueuePut(QueueBmpDataHandle, &_BmpData, 0, osWaitForever);

//	  osMutexAcquire(MutexBmpDataHandleHandle, osWaitForever);
//	  Pressure = _BmpData.Pressure;
//	  Temperature = _BmpData.Temperature;
//	  osMutexRelease(MutexBmpDataHandleHandle);

	  printf("Temperature: %.2f, Pressure: %.2f\n\r", _BmpData.Temperature, _BmpData.Pressure);

	  DelayTick += 10;
	  osDelayUntil(DelayTick);
  }

  /* USER CODE END StartBmp_280Task */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
	char Message[32];
	uint8_t i = 0;
	BmpData_t _BmpData;

	FftData_t FftData;

	osMutexAcquire(MutexI2C1HendleHandle, osWaitForever);
	SSD1306_Init(&hi2c1);
	osMutexRelease(MutexI2C1HendleHandle);

	GFX_SetFont(font_8x5);

	SSD1306_Clear(BLACK);

//	osMutexAcquire(MutexI2C1HendleHandle, osWaitForever);
	SSD1306_Display();
//	osMutexRelease(MutexI2C1HendleHandle);

  /* Infinite loop */
  for(;;)
  {
	SSD1306_Clear(BLACK);

	sprintf(Message, "Hello %d", i++);

	GFX_DrawString(0, 0, Message, WHITE, 0);

//	osMutexAcquire(MutexBmpDataHandleHandle, osWaitForever);
//	_BmpData.Pressure = Pressure;
//	_BmpData.Temperature = Temperature;
//	osMutexRelease(MutexBmpDataHandleHandle);

	osMessageQueueGet(QueueBmpDataHandle, &_BmpData, NULL, 0);

	osMessageQueueGet(QueueFFTDataHandle, &FftData, NULL, 0);


	sprintf(Message, "Press: %.2f", _BmpData.Pressure);
	GFX_DrawString(0, 10, Message, WHITE, 0);

	sprintf(Message, "Temp: %.2f", _BmpData.Temperature);
	GFX_DrawString(0, 20, Message, WHITE, 0);


	//
	// FFT
	//
	for(uint8_t i = 0; i < 10; i++) // Each frequency
	{
	  GFX_DrawFillRectangle(10+(i*11), 64-FftData.OutFreqArray[i], 10, FftData.OutFreqArray[i], WHITE);
	}

//	osMutexAcquire(MutexI2C1HendleHandle, osWaitForever);
	SSD1306_Display();
//	osMutexRelease(MutexI2C1HendleHandle);
//    osDelay(100);

  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartFFTTask */
/**
* @brief Function implementing the FFTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFFTTask */
void StartFFTTask(void *argument)
{
  /* USER CODE BEGIN StartFFTTask */
	arm_rfft_fast_instance_f32 FFTHandler;
	FftData_t FftData;
	int FreqPoint = 0;
	int Offset = 40; // variable noise floor offset

	//
	// FFT
	//
	uint16_t *AdcMicrophone;
	float *FFTInBuffer;
	float *FFTOutBuffer;
	int *Freqs;

	AdcMicrophone = pvPortMalloc(FFT_SAMPLES * sizeof(uint16_t));
	FFTInBuffer = pvPortMalloc(FFT_SAMPLES * sizeof(float));
	FFTOutBuffer = pvPortMalloc(FFT_SAMPLES * sizeof(float));
	Freqs = pvPortMalloc(FFT_SAMPLES * sizeof(int));

	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcMicrophone, FFT_SAMPLES);

	arm_rfft_fast_init_f32(&FFTHandler, FFT_SAMPLES);

  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);

	  for(uint32_t i = 0; i < FFT_SAMPLES; i++)
	  {
		  FFTInBuffer[i] =  (float)AdcMicrophone[i];
	  }

	  arm_rfft_fast_f32(&FFTHandler, FFTInBuffer, FFTOutBuffer, 0);

		FreqPoint = 0;
		// calculate abs values and linear-to-dB
		for (int i = 0; i < FFT_SAMPLES; i = i+2)
		{
			Freqs[FreqPoint] = (int)(20*log10f(complexABS(FFTOutBuffer[i], FFTOutBuffer[i+1]))) - Offset;

			if(Freqs[FreqPoint] < 0)
			{
				Freqs[FreqPoint] = 0;
			}
			FreqPoint++;
		}

		FftData.OutFreqArray[0] = (uint8_t)Freqs[1]; // 22 Hz
		FftData.OutFreqArray[1] = (uint8_t)Freqs[2]; // 63 Hz
		FftData.OutFreqArray[2] = (uint8_t)Freqs[3]; // 125 Hz
		FftData.OutFreqArray[3] = (uint8_t)Freqs[6]; // 250 Hz
		FftData.OutFreqArray[4] = (uint8_t)Freqs[12]; // 500 Hz
		FftData.OutFreqArray[5] = (uint8_t)Freqs[23]; // 1000 Hz
		FftData.OutFreqArray[6] = (uint8_t)Freqs[51]; // 2200 Hz
		FftData.OutFreqArray[7] = (uint8_t)Freqs[104]; // 4500 Hz
		FftData.OutFreqArray[8] = (uint8_t)Freqs[207]; // 9000 Hz
		FftData.OutFreqArray[9] = (uint8_t)Freqs[344]; // 15000 Hz

		osMessageQueuePut(QueueFFTDataHandle, &FftData, 0, osWaitForever);
  }
  /* USER CODE END StartFFTTask */
}

/* TimerBmpDataCallback function */
void TimerBmpDataCallback(void *argument)
{
  /* USER CODE BEGIN TimerBmpDataCallback */
	osSemaphoreRelease(SemaphoreBmpQueueHandle);
  /* USER CODE END TimerBmpDataCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
//	SamplesReady = 1;
		osThreadFlagsSet(FFTTaskHandle, 0x01);
	}
}


/* USER CODE END Application */

