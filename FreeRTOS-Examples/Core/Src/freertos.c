/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "fatfs.h"
#include "app_x-cube-mems1.h"

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
/* USER CODE BEGIN Variables */
uint8_t fileOnce = 0;
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS __attribute__ ((aligned(4)));    /* File system object for SD logical drive */
FIL SDFile __attribute__ ((aligned(4)));
FRESULT res;
uint8_t wtext[] = "This text has been written using FreeRTOS, CMSIS v1.0, and FatFS"; /* File write buffer */
uint32_t byteswritten;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId testTaskHandle;
osThreadId microSDTaskHandle;
osThreadId hts221TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void blinkingLED(void const * argument);
void microSD(void const * argument);
void HTS221(void const * argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of testTask */
  osThreadDef(testTask, blinkingLED, osPriorityLow, 0, 128);
  testTaskHandle = osThreadCreate(osThread(testTask), NULL);

  /* definition and creation of microSDTask */
  osThreadDef(microSDTask, microSD, osPriorityIdle, 0, 128);
  microSDTaskHandle = osThreadCreate(osThread(microSDTask), NULL);

  /* definition and creation of hts221Task */
  osThreadDef(hts221Task, HTS221, osPriorityIdle, 0, 128);
  hts221TaskHandle = osThreadCreate(osThread(hts221Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for MEMS */
  MX_MEMS_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_blinkingLED */
/**
* @brief Function implementing the testTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blinkingLED */
void blinkingLED(void const * argument)
{
  /* USER CODE BEGIN blinkingLED */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    osDelay(100);
  }
  /* USER CODE END blinkingLED */
}

/* USER CODE BEGIN Header_microSD */
/**
* @brief Function implementing the hmicroSD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_microSD */
void microSD(void const * argument)
{
  /* USER CODE BEGIN microSD */
	//MX_FATFS_Init();
  /* Infinite loop */
  for(;;)
  {
	  if(fileOnce == 0){
		  res = f_mount(&SDFatFS, (TCHAR const*) SDPath, 1); // 1. Register a work area
		  if (res == FR_OK){
			  res = f_open(&SDFile, "freertos.txt", FA_CREATE_ALWAYS | FA_WRITE); // 2. Creating a new file for writing/reading later
			  if(res == FR_OK){
				  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				  res = f_write(&SDFile, wtext, sizeof(wtext), (void *) &byteswritten);
				  if((res != FR_OK) || (byteswritten == 0)){
					  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				  }
			  }
			  f_close(&SDFile);
		  }
		  fileOnce = 1;
	  }
    osDelay(500);
  }
  /* USER CODE END microSD */
}

/* USER CODE BEGIN Header_HTS221 */
/**
* @brief Function implementing the hts221Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HTS221 */
void HTS221(void const * argument)
{
  /* USER CODE BEGIN HTS221 */
  /* Infinite loop */
  for(;;)
  {
	  MX_MEMS_Process();
	  osDelay(10);
  }
  /* USER CODE END HTS221 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
