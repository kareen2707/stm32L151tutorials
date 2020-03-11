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
#include "stdio.h"
#include "stdlib.h"
#include "se868k3-reg.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_MSG_LEN 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS __attribute__ ((aligned(4)));    /* File system object for SD logical drive */
FIL SDFile __attribute__ ((aligned(4)));
uint8_t fileCreated = 0;
uint8_t nmea_test[100] = "$GPGSA,A,1,,*1E\r\n$GNRMC,164004.000,A,4027.1783,N,00343.5470,W,0.19,67.20,030320,,,*58\r\n";

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId consumerTaskHandle;
osThreadId producer1Handle;
osMessageQId myQueue01Handle;
osTimerId myTimer01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int32_t parse_gprmc (GPRMC_Infos *gprmc_data, uint8_t *NMEA);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void microSD(void const * argument);
void gnss(void const * argument);
void Callback01(void const * argument);

//extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,SET);
	while(1){

	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
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
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,SET);
		while(1){

		}
}
/* USER CODE END 5 */

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	MX_FATFS_Init();

	UTC_Info *test_utc = (UTC_Info *) pvPortMalloc(sizeof(UTC_Info));
	Coords *test_cords = (Coords *) pvPortMalloc(sizeof(Coords));
	GPRMC_Infos *test_gprmc = (GPRMC_Infos *) pvPortMalloc(sizeof(GPRMC_Infos));
	//uint8_t utc_date = 0;
	//char *gnrmc_status = (char *) pvPortMalloc(2);

	test_utc->utc = 0;
	test_utc->hh = 0;
	test_utc->mm = 0;
	test_utc->ss = 0;

	test_cords->lat = 0;
	test_cords->lon = 0;
	test_cords->ew = 0;
	test_cords->ns = 0;

	test_gprmc->utc = test_utc;
	test_gprmc->status = 'V';
	test_gprmc->xyz = test_cords;
	//test_gprmc->date = 0;

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 3, uint32_t);
  //osMessageQDef(myQueue01, sizeof(struct GPRMC_Infos), GPRMC_Infos);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of consumerTask */
  osThreadDef(consumerTask, microSD, osPriorityNormal, 0, 200);
  consumerTaskHandle = osThreadCreate(osThread(consumerTask), NULL);

  /* definition and creation of producer1 */
  osThreadDef(producer1, gnss, osPriorityNormal, 0, 300);
  producer1Handle = osThreadCreate(osThread(producer1), (void *) test_gprmc);

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
  //MX_FATFS_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_microSD */
/**
* @brief Function implementing the consumerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_microSD */
void microSD(void const * argument)
{
  /* USER CODE BEGIN microSD */
	osEvent rx;
	GPRMC_Infos *rx_gprmc;
	uint8_t byteswritten;
	char utc_data[10];

	if(f_mount(&SDFatFS, (TCHAR const*) SDPath, 1) == FR_OK){ // 1. Register a work area
			if(f_open(&SDFile, "lines.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){ // 2. Creating a new file to write it later
				fileCreated = 1;
			}
		}

  /* Infinite loop */
  for(;;)
  {
	  byteswritten = 0;
	  if(fileCreated){
		  rx = osMessageGet(myQueue01Handle, 10);
		  rx_gprmc = (GPRMC_Infos *)rx.value.p;
		  //utc_data[0] = rx_gprmc->utc->utc;
		  itoa(rx_gprmc->utc->utc, utc_data, 10);
		  //utc_data[0] = (uint8_t) rx_gprmc->utc->hh;
		  //utc_data[1] = (uint8_t) rx_gprmc->utc->mm;
		  //utc_data[2] = (uint8_t) rx_gprmc->utc->ss;
		  //if(f_write(&SDFile, rx_gprmc->utc->utc, sizeof(rx_gprmc->utc->utc), (void *)&byteswritten)){
		  //if(f_write(&SDFile, (const void *)str1, strlen(str1), (void *)&byteswritten)){
			  //f_close(&SDFile);
		  //}
		  f_write(&SDFile, utc_data, sizeof(utc_data), (void *) &byteswritten);
		  f_write(&SDFile, "\r\n", strlen("\r\n"), (void *)&byteswritten);
		  byteswritten = 0;
		  f_sync(&SDFile);
	  }

    osDelay(200);
  }
  /* USER CODE END microSD */
}

/* USER CODE BEGIN Header_gnss */
/**
* @brief Function implementing the producer1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gnss */
void gnss(void const * argument)
{
  /* USER CODE BEGIN gnss */
	GPRMC_Infos * gnrmc_data = (GPRMC_Infos *) argument;
  /* Infinite loop */
  for(;;)
  {

    if(!parse_gprmc(gnrmc_data, nmea_test)){
    	osMessagePut(myQueue01Handle, (uint32_t)gnrmc_data, 10);
    }
    osDelay(100);
  }
  /* USER CODE END gnss */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
  
  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
int32_t parse_gprmc (GPRMC_Infos *gprmc_data, uint8_t *NMEA){
//int32_t parse_gprmc (uint8_t *buffer, uint8_t *NMEA){
//int32_t parse_gprmc (uint8_t *NMEA){

	  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
	  uint8_t valid_msg = 0;

	  //ParseStatus_Typedef status = PARSE_FAIL;
	  int32_t status = 1; // fail

	  if(NMEA == NULL)
	    return status;

	  /* clear the app[][] buffer */
	  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
	    memset(app[i], 0, MAX_MSG_LEN);
	  }

	  for (unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
	  {
	    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
	      app[j][k] = '\0';

	      if (strcmp((char *)app[0], "$GNRMC") == 0)
	      {
	        j++;
	        k = 0;
	        valid_msg = 1;
	        continue;
	      }
	      else {
	        while (NMEA[i++] != '\n');
	        j = k = 0;
	      }
	    }
	    app[j][k++] = NMEA[i];
	  }

	  if (valid_msg == 1) {

		  gprmc_data->utc->utc = atoi((const char *) app[1]);
		  gprmc_data->utc->hh =  atoi((const char *) app[1])/10000;
		  gprmc_data->utc->mm =  (gprmc_data->utc->utc - (gprmc_data->utc->hh * 10000))/100;
		  gprmc_data->utc->ss = gprmc_data->utc->utc - ((gprmc_data->utc->hh * 10000) + (gprmc_data->utc->mm*100));
		  gprmc_data->status = app[2][0];
		  gprmc_data->xyz->lat = atoff((const char *) app[3]);
		  gprmc_data->xyz->ns = app[4][0];
		  gprmc_data->xyz->lon = atoff((const char *)app[5]);
		  gprmc_data->xyz->ns = app[6][0];
		  gprmc_data->date = atoi((const char *) app[9]);
		  valid_msg = 0;
		  status = 0;
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }

	  return status;
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
