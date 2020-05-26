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
//#include "app_x-cube-mems1.h"
#include "hts221.h"
#include "se868k3.h"
#include "custom_bus.h"
#include <stdio.h>
#include <string.h>

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
uint8_t fileCreated = 0;
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS __attribute__ ((aligned(4)));    /* File system object for SD logical drive */
FIL SDFile __attribute__ ((aligned(4)));
static uint8_t new_cmd = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId microSDTaskHandle;
osThreadId hts221TaskHandle;
osThreadId gnssTaskHandle;
osThreadId ds600TaskHandle;
osThreadId sph064TaskHandle;
osThreadId bmx160TaskHandle;
osMutexId gnssMutexHandle;
osMessageQId myQueue01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void microSD(void const * argument);
void HTS221(void const * argument);
void SE868K3(void const * argument);
void dogtemp(void const * argument);
void mphones(void const * argument);
void imu(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
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
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	SE868K3_Object_t* SE868K3_pObj;
	uint8_t* gnss_buffer;
	uint16_t * flags;

	gnss_buffer = (uint8_t*)pvPortMalloc(BUFFER_PCKT_SIZE);
	SE868K3_pObj = (SE868K3_Object_t*) pvPortMalloc(sizeof(SE868K3_Object_t));
	flags = (uint16_t*) pvPortMalloc(sizeof(uint16_t));

	*flags = 0;
	SE868K3_pObj->is_initialized = 0;
	SE868K3_pObj->pileUART = gnss_buffer;


	MX_FATFS_Init();

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of gnssMutex */
	osMutexDef(gnssMutex);
	gnssMutexHandle = osMutexCreate(osMutex(gnssMutex));
	//SE868K3_pObj->pileLock = gnssMutexHandle;

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
	osMessageQDef(myQueue01, 4, uint32_t);
	myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of microSDTask */
  osThreadDef(microSDTask, microSD, osPriorityNormal, 0, 200);
  microSDTaskHandle = osThreadCreate(osThread(microSDTask), NULL);

  /* definition and creation of hts221Task */
//  osThreadDef(hts221Task, HTS221, osPriorityNormal, 0, 128);
//  hts221TaskHandle = osThreadCreate(osThread(hts221Task), NULL);

  /* definition and creation of gnssTask */
  osThreadDef(gnssTask, SE868K3, osPriorityNormal, 0, 300);
  gnssTaskHandle = osThreadCreate(osThread(gnssTask), (void*) SE868K3_pObj);

  /* definition and creation of dogTempTask */
//  osThreadDef(dogtempTask, dogtemp, osPriorityBelowNormal, 0, 96);
//  ds600TaskHandle = osThreadCreate(osThread(dogtempTask), NULL);
//
//  /* definition and creation of sph064Task */
//  osThreadDef(sph064Task, mphones, osPriorityIdle, 0, 128);
//  sph064TaskHandle = osThreadCreate(osThread(sph064Task), NULL);
//
//  /* definition and creation of bmx160Task */
//  osThreadDef(bmx160Task, imu, osPriorityIdle, 0, 96);
//  bmx160TaskHandle = osThreadCreate(osThread(bmx160Task), NULL);

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

  /* init code for MEMS */
  //MX_MEMS_Init();
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
* @brief Function implementing the hmicroSD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_microSD */
void microSD(void const * argument)
{
  /* USER CODE BEGIN microSD */
	//uint8_t *cmd;
	//pilePointers_t *serial = (pilePointers_t *) argument;
	osEvent rx;
	uint8_t byteswritten;

	if(f_mount(&SDFatFS, (TCHAR const*) SDPath, 1) == FR_OK){ // 1. Register a work area
		if(f_open(&SDFile, "real.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){ // 2. Creating a new file to write it later
			fileCreated = 1;
		}
	}

  /* Infinite loop */
  for(;;)
  {
	  byteswritten = 0;
	  	  if(fileCreated && new_cmd){
	  		  if((osMutexWait(gnssMutexHandle, 6)) == osOK){
	  			  rx = osMessageGet(myQueue01Handle, 0);
	  			  new_cmd = 0; // Cleaning the flag
	  			  osMutexRelease(gnssMutexHandle);
	  			  f_write(&SDFile, (const void *) rx.value.p, BUFFER_PCKT_SIZE, (void *)&byteswritten);
	  			  f_write(&SDFile, "\r\n", strlen("\r\n"), (void *)&byteswritten);
	  			  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  			  f_sync(&SDFile);
	  		  }
	  	  }
    osDelay(110);
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
	HTS221_Object_t HTS221_pObj;
	HTS221_IO_t HTS221_pIO;
	HTS221_pIO.Address = (uint8_t) (HTS221_I2C_ADDRESS<<1);
	HTS221_pIO.BusType = HTS221_I2C_BUS;
	HTS221_pIO.Init = BSP_I2C1_Init;
	HTS221_pIO.DeInit = BSP_I2C1_DeInit;
	HTS221_pIO.ReadReg = BSP_I2C1_Recv;
	HTS221_pIO.WriteReg = BSP_I2C1_Send;
	//HTS221_pIO.GetTick = BSP_GetTick;
	uint8_t id, status;
	float temperature, humidity;

	HTS221_RegisterBusIO(&HTS221_pObj, &HTS221_pIO);
	HTS221_Init(&HTS221_pObj);
	HTS221_ReadID(&HTS221_pObj, &id);
	if(id == HTS221_ID){
		HTS221_Get_Init_Status(&HTS221_pObj, &status);
		HTS221_HUM_Enable(&HTS221_pObj);
		HTS221_TEMP_Enable(&HTS221_pObj);
	}
  /* Infinite loop */
  for(;;)
  {
	  HTS221_HUM_GetHumidity(&HTS221_pObj, &humidity);
	  HTS221_TEMP_GetTemperature(&HTS221_pObj, &temperature);
	  osDelay(500);
  }
  /* USER CODE END HTS221 */
}

/* USER CODE BEGIN Header_SE868K3 */
/**
* @brief Function implementing the gnssTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SE868K3 */
void SE868K3(void const * argument)
{
  /* USER CODE BEGIN SE868K3 */

	SE868K3_IO_t SE868K3_pIO;
	SE868K3_Object_t *SE868K3_pObj = (SE868K3_Object_t*) argument;

	SE868K3_pIO.Init = BSP_UART1_Init;
	SE868K3_pIO.DeInit = BSP_UART1_DeInit;
	SE868K3_pIO.Read = BSP_UART1_Recv;
	SE868K3_pIO.Write = BSP_UART1_Send;

	if(SE868K3_RegisterBusIO(SE868K3_pObj, &SE868K3_pIO) != SE868K3_ERROR){
		if(SE868K3_Init(SE868K3_pObj) != SE868K3_OK )
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		}
	}

	//SE868K3_Test(SE868K3_pObj); // re-send the output message rate


  /* Infinite loop */
  for(;;)
  {
	  //if(SE868K3_Read_Packet(SE868K3_pObj, BUFFER_PCKT_SIZE) == 1){
	  if(SE868K3_Read_GNRMC_Pck(SE868K3_pObj) == 1){
		  if((osMutexWait(gnssMutexHandle, 6)) == osOK){
			  osMessagePut(myQueue01Handle, (uint32_t)SE868K3_pObj->pileUART, 0);
			  new_cmd = 1;
			  osMutexRelease(gnssMutexHandle);
		  }
	  }
	  osDelay(100);
  }
  /* USER CODE END SE868K3 */
}


/* USER CODE BEGIN Header_dogtemp */
/**
* @brief Function implementing the ds600Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dogTemp */
void dogTemp(void const * argument)
{
  /* USER CODE BEGIN dogtemp */

  /* Infinite loop */
  for(;;)
  {
    osDelay(20);
  }
  /* USER CODE END dogtemp */
}

/* USER CODE BEGIN Header_mphones */
/**
* @brief Function implementing the sph064Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mphones */
void mphones(void const * argument)
{
  /* USER CODE BEGIN mphones */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END mphones */
}

/* USER CODE BEGIN Header_imu */
/**
* @brief Function implementing the bmx160Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu */
void imu(void const * argument)
{
  /* USER CODE BEGIN imu */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
