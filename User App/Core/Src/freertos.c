/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications/* USER CODE BEGIN Header */
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
#include "ds600.h"
#include "sph0644.h"
#include "bmx160.h"
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

SE868K3_Object_t* SE868K3_pObj;
SE868K3_IO_t* SE868K3_pIO;
uint8_t* gnss_buffer;

HTS221_Object_t* HTS221_pObj;
HTS221_IO_t* HTS221_pIO;
hts221_data_t* HTS221_Data_Read;

DS600_Object_t* DS600_pObj;
DS600_IO_t* DS600_pIO;
float* corporal_temp;

SPH06440_Object_t* SPH06440_pObj;
SPH06440_IO_t* SPH06440_pIO;
uint8_t* sound;

BMX160_Object_t* BMX160_pObj;
BMX160_IO_t* BMX160_pIO;
bmx160_sensor_data* accel;
bmx160_sensor_data* gyro;
bmx160_aux_data* mag;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId microSDTaskHandle;
osThreadId hts221TaskHandle;
osThreadId gnssTaskHandle;
osThreadId ds600TaskHandle;
osThreadId sph064TaskHandle;
osThreadId bmx160TaskHandle;
osMutexId gnssMutexHandle;
osMessageQId gnssQueueHandle;
osMessageQId hts221_QueueHandle;
osMessageQId ds600QueueHandle;
osMessageQId mphonesQueueHandle;
osMessageQId bmx160QueueHandle;
osTimerId bmx160TimerHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void BMX160_delay_ms(uint32_t period);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void microSD(void const * argument);
void HTS221(void const * argument);
void SE868K3(void const * argument);
void DS600(void const * argument);
void mphones(void const * argument);
void imu(void const * argument);
void bmx160_Timer_Callback(void const * argument);

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


	gnss_buffer = (uint8_t*)pvPortMalloc(GLL_MAX_SIZE);
	SE868K3_pObj = (SE868K3_Object_t*)pvPortMalloc(sizeof(SE868K3_Object_t));
	SE868K3_pIO	= (SE868K3_IO_t *) pvPortMalloc(sizeof(SE868K3_IO_t));
	HTS221_pObj = (HTS221_Object_t*) pvPortMalloc(sizeof(HTS221_Object_t));
	HTS221_pIO	= (HTS221_IO_t *) pvPortMalloc(sizeof(HTS221_IO_t));

	SE868K3_pObj->is_initialized = 0;
	SE868K3_pObj->pileUART = gnss_buffer;
	SE868K3_pIO->Init = BSP_UART1_Init;
	SE868K3_pIO->DeInit = BSP_UART1_DeInit;
	SE868K3_pIO->Read = BSP_UART1_Recv;
	SE868K3_pIO->Write = BSP_UART1_Send;

	if(SE868K3_RegisterBusIO(SE868K3_pObj, SE868K3_pIO) != SE868K3_ERROR){
		osThreadDef(gnssTask, SE868K3, osPriorityNormal, 0, 300);
		gnssTaskHandle = osThreadCreate(osThread(gnssTask), (void*) SE868K3_pObj);
		osMessageQDef(myQueue01, 4, uint32_t);
		gnssQueueHandle = osMessageCreate(osMessageQ(myQueue01), NULL);
	}
	else{
		vPortFree(SE868K3_pObj);
		vPortFree(SE868K3_pIO);
	}

	HTS221_pObj->is_initialized = 0;
	HTS221_pIO->Address = (uint8_t)(HTS221_I2C_ADDRESS << 1);
	HTS221_pIO->BusType = HTS221_I2C_BUS;
	HTS221_pIO->Init = BSP_I2C1_Init;
	HTS221_pIO->DeInit = BSP_I2C1_DeInit;
	HTS221_pIO->ReadReg = BSP_I2C1_Recv;
	HTS221_pIO->WriteReg = BSP_I2C1_Send;

	if(HTS221_RegisterBusIO(HTS221_pObj, HTS221_pIO) != HTS221_ERROR){
		osThreadDef(hts221Task, HTS221, osPriorityNormal, 0, 128);
		hts221TaskHandle = osThreadCreate(osThread(hts221Task), (void*) HTS221_pObj);
		osMessageQDef(myQueue02, 4, sizeof(hts221_data_t));
		hts221_QueueHandle = osMessageCreate(osMessageQ(myQueue02), NULL);
	}
	else{
		vPortFree(HTS221_pObj);
		vPortFree(HTS221_pIO);
	}

	DS600_pObj = (DS600_Object_t*) pvPortMalloc(sizeof(DS600_Object_t));
	DS600_pIO = (DS600_IO_t*) pvPortMalloc(sizeof(DS600_IO_t));
	DS600_pObj->is_initialized = 0;
	DS600_pIO->Init = BSP_ADC1_Init;
	DS600_pIO->DeInit = BSP_ADC1_DeInit;
	DS600_pIO->Read = BSP_ADC1_Read;

	if(DS600_RegisterBusIO(DS600_pObj, DS600_pIO) != DS600_ERROR){
		osThreadDef(dogtempTask, DS600, osPriorityBelowNormal, 0, 96);
		ds600TaskHandle = osThreadCreate(osThread(dogtempTask), (void*) DS600_pObj);
		osMessageQDef(myQueue04, 4, uint32_t);
		ds600QueueHandle = osMessageCreate(osMessageQ(myQueue04), NULL);
	}
	else{
		vPortFree(DS600_pObj);
		vPortFree(DS600_pIO);
	}

	BMX160_pObj = (BMX160_Object_t*) pvPortMalloc(sizeof(BMX160_Object_t));
	BMX160_pIO = (BMX160_IO_t*) pvPortMalloc(sizeof(BMX160_IO_t));
	osTimerDef(bmx160Timer, bmx160_Timer_Callback);
	bmx160TimerHandle = osTimerCreate(osTimer(bmx160Timer), osTimerOnce, NULL);

	BMX160_pObj->is_initialized = 0;
	BMX160_pIO->BusType = BMX160_SPI_INTF;
	BMX160_pIO->Init = BSP_SPI1_Init;
	BMX160_pIO->DeInit = BSP_SPI1_DeInit;
	BMX160_pIO->ReadReg = BSP_SPI1_Recv;
	BMX160_pIO->WriteReg = BSP_SPI1_Send;
	BMX160_pIO->Delayms = BMX160_delay_ms;

	if(BMX160_RegisterBusIO(BMX160_pObj, BMX160_pIO) == BMX160_OK){
		osThreadDef(bmx160Task, imu, osPriorityIdle, 0, 128);
		bmx160TaskHandle = osThreadCreate(osThread(bmx160Task), (void*) BMX160_pObj);
		osMessageQDef(myQueue05, 4, sizeof(bmx160_sensor_data));
		bmx160QueueHandle = osMessageCreate(osMessageQ(myQueue05), NULL);
	}
	else{
		vPortFree(BMX160_pObj);
		vPortFree(BMX160_pIO);
		osTimerDelete(bmx160TimerHandle);
	}

	SPH06440_pObj = (SPH06440_Object_t*) pvPortMalloc(sizeof(SPH06440_Object_t));
	SPH06440_pIO = (SPH06440_IO_t*) pvPortMalloc(sizeof(SPH06440_IO_t));

	SPH06440_pObj->is_initialized = 0;
	SPH06440_pIO->Init = BSP_SPI3_Init;
	SPH06440_pIO->Read = BSP_SPI3_Recv;

	if(SPH06440_RegisterBusIO(SPH06440_pObj, SPH06440_pIO) == SPH0644_OK){
		osThreadDef(sph064Task, mphones, osPriorityIdle, 0, 128);
		sph064TaskHandle = osThreadCreate(osThread(sph064Task), (void*) SPH06440_pObj);
		osMessageQDef(myQueue06, 4, uint32_t);
		mphonesQueueHandle = osMessageCreate(osMessageQ(myQueue06), NULL);
	}
	else{
		vPortFree(SPH06440_pObj);
		vPortFree(SPH06440_pIO);
	}

	MX_FATFS_Init();
	if(f_mount(&SDFatFS, (TCHAR const*) SDPath, 1) == FR_OK){ // 1. Register a work area
			if(f_open(&SDFile, "tfm1.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){ // 2. Creating a new file to write it later
				fileCreated = 1;
				osThreadDef(microSDTask, microSD, osPriorityNormal, 0, 200);
				microSDTaskHandle = osThreadCreate(osThread(microSDTask), NULL);
			}
		}

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of gnssMutex */
	osMutexDef(gnssMutex);
	gnssMutexHandle = osMutexCreate(osMutex(gnssMutex));

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
//	osMessageQDef(myQueue01, 4, uint32_t);
//	gnssQueueHandle = osMessageCreate(osMessageQ(myQueue01), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of microSDTask */
//  osThreadDef(microSDTask, microSD, osPriorityNormal, 0, 200);
//  microSDTaskHandle = osThreadCreate(osThread(microSDTask), NULL);

  /* definition and creation of hts221Task */
//  osThreadDef(hts221Task, HTS221, osPriorityNormal, 0, 128);
//  hts221TaskHandle = osThreadCreate(osThread(hts221Task), NULL);

  /* definition and creation of gnssTask */
//  osThreadDef(gnssTask, SE868K3, osPriorityNormal, 0, 300);
//  gnssTaskHandle = osThreadCreate(osThread(gnssTask), (void*) SE868K3_pObj);

  /* definition and creation of dogTempTask */
//  osThreadDef(dogtempTask, dogTemp, osPriorityBelowNormal, 0, 96);
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
	//osEvent rx;
	osEvent temp_rx;
	osEvent imu_rx;
	osEvent gnss_rx;
	osEvent mphones_rx;
	osEvent corporal_rx;
	uint8_t byteswritten;

  /* Infinite loop */
  for(;;)
  {
	  byteswritten = 0;
//	  	  if(fileCreated && new_cmd){
//	  		  if((osMutexWait(gnssMutexHandle, 6)) == osOK){
//	  			  rx = osMessageGet(gnssQueueHandle, 0);
//	  			  new_cmd = 0; // Cleaning the flag
//	  			  osMutexRelease(gnssMutexHandle);
//	  			  f_write(&SDFile, (const void *) rx.value.p, BUFFER_PCKT_SIZE, (void *)&byteswritten);
//	  			  f_write(&SDFile, "\r\n", strlen("\r\n"), (void *)&byteswritten);
//	  			  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//	  			  //f_sync(&SDFile);
//	  		  }
//	  	  }
	  	  temp_rx = osMessageGet(hts221_QueueHandle, 1);
	  	  if(temp_rx.status == osEventMessage){
	  		f_write(&SDFile, (const void *) temp_rx.value.p, 1, (void *)&byteswritten);
	  		vPortFree(HTS221_Data_Read);
	  		f_write(&SDFile, "degrees\r\n", strlen("degrees\r\n"), (void *)&byteswritten);
	  	  }

	  	  mphones_rx = osMessageGet(mphonesQueueHandle, 2);
	  	  if(mphones_rx.status == osEventMessage){
	  		f_write(&SDFile, (const void *) mphones_rx.value.p, 20, (void *)&byteswritten);
	  		vPortFree(sound);
	  		f_write(&SDFile, "pdm\r\n", strlen("pdm\r\n"), (void *)&byteswritten);
	  	  }

	  	  imu_rx = osMessageGet(bmx160QueueHandle, 2);
	  	  if(imu_rx.status == osEventMessage){
	  		f_write(&SDFile, (const void *) imu_rx.value.p, 1, (void *)&byteswritten);
	  		vPortFree(accel);
	  		vPortFree(gyro);
	  		f_write(&SDFile, "G\r\n", strlen("G\r\n"), (void *)&byteswritten);
	  	  }

	  	  corporal_rx = osMessageGet(ds600QueueHandle, 0);
	  	  if(corporal_rx.status == osEventMessage){
	  	   f_write(&SDFile, (const void *) corporal_rx.value.p, 1, (void *)&byteswritten);
	  	   vPortFree(corporal_temp);
	  	   f_write(&SDFile, "C d\r\n", strlen("C d\r\n"), (void *)&byteswritten);
	  	  }


	  	  gnss_rx = osMessageGet(gnssQueueHandle, 0);
	  	  if(gnss_rx.status == osEventMessage){
	  		f_write(&SDFile, (const void *) gnss_rx.value.p, BUFFER_PCKT_SIZE, (void *)&byteswritten);
	  		f_write(&SDFile, "\r\n", strlen("\r\n"), (void *)&byteswritten);
	  	  }
	  	  f_sync(&SDFile);
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

//	HTS221_pIO.Address = (uint8_t) (HTS221_I2C_ADDRESS<<1);
//	HTS221_pIO.BusType = HTS221_I2C_BUS;
//	HTS221_pIO.Init = BSP_I2C1_Init;
//	HTS221_pIO.DeInit = BSP_I2C1_DeInit;
//	HTS221_pIO.ReadReg = BSP_I2C1_Recv;
//	HTS221_pIO.WriteReg = BSP_I2C1_Send;
	//HTS221_pIO.GetTick = BSP_GetTick;
	uint8_t id, status;
	//HTS221_RegisterBusIO(&HTS221_pObj, &HTS221_pIO);
	HTS221_Object_t *HTS221_pObj = (HTS221_Object_t*) argument;
	HTS221_Init(HTS221_pObj);
	HTS221_ReadID(HTS221_pObj, &id);
	if(id == HTS221_ID){
		HTS221_Get_Init_Status(HTS221_pObj, &status);
		HTS221_HUM_Enable(HTS221_pObj);
		HTS221_TEMP_Enable(HTS221_pObj);
	}
  /* Infinite loop */
  for(;;)
  {
	  HTS221_Data_Read = (hts221_data_t*)pvPortMalloc(sizeof(hts221_data_t));
	  HTS221_TEMP_GetTemperature(HTS221_pObj, &HTS221_Data_Read->temperature);
	  HTS221_HUM_GetHumidity(HTS221_pObj, &HTS221_Data_Read->humidity);
	  osMessagePut(hts221_QueueHandle, (uint32_t)HTS221_Data_Read, 2);
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

	//SE868K3_IO_t SE868K3_pIO;
	SE868K3_Object_t *SE868K3_pObj = (SE868K3_Object_t*) argument;
	SE868K3_Init(SE868K3_pObj);
	//SE868K3_pIO.Init = BSP_UART1_Init;
	//SE868K3_pIO.DeInit = BSP_UART1_DeInit;
	//SE868K3_pIO.Read = BSP_UART1_Recv;
	//SE868K3_pIO.Write = BSP_UART1_Send;
	//SE868K3_pIO.GetTick = BSP_GetTick;

//	if(SE868K3_RegisterBusIO(SE868K3_pObj, &SE868K3_pIO) != SE868K3_ERROR){
//		if(SE868K3_Init(SE868K3_pObj) != SE868K3_OK )
//		{
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
//		}
//	}

	//SE868K3_Test(SE868K3_pObj); // re-send the output message rate


  /* Infinite loop */
  for(;;)
  {
	  //if(SE868K3_Read_Packet(SE868K3_pObj, BUFFER_PCKT_SIZE) == 1){
	  if(SE868K3_Read_RMC(SE868K3_pObj) == 1){
		  //if(SE868K3_pObj->pileUART[18] == 'A'){
			  if((osMutexWait(gnssMutexHandle, 6)) == osOK){
				  osMessagePut(gnssQueueHandle, (uint32_t)SE868K3_pObj->pileUART, 0);
				  new_cmd = 1;
				  osMutexRelease(gnssMutexHandle);
			  }
		  //}
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
void DS600(void const * argument)
{
  /* USER CODE BEGIN dogtemp */
	DS600_Object_t *DS600_pObj = (DS600_Object_t*) argument;

  /* Infinite loop */
  for(;;)
  {
	  corporal_temp = (float*) pvPortMalloc(sizeof(float));
	  DS600_GET_Temperature(DS600_pObj, corporal_temp);
	  osMessagePut(ds600QueueHandle, (uint32_t)corporal_temp, 0);
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
	SPH06440_Object_t *SPH06440_pObj = (SPH06440_Object_t*) argument;

  /* Infinite loop */
  for(;;)
  {
	  sound = (uint8_t*)pvPortMalloc(20);
	  SPH06440_GET_samples(SPH06440_pObj, sound, 20);
	  osMessagePut(mphonesQueueHandle, (uint32_t) sound, 5);
    osDelay(30);
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
	BMX160_Object_t *BMX160_pObj = (BMX160_Object_t*) argument;

	/* Select the Output data rate, range and power mode of accelerometer sensor */
	BMX160_pObj->accel_cfg.odr = BMX160_ACCEL_ODR_1600HZ;
	BMX160_pObj->accel_cfg.range = BMX160_ACCEL_RANGE_2G;
	BMX160_pObj->accel_cfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
	BMX160_pObj->accel_cfg.power = BMX160_ACCEL_NORMAL_MODE;

	/* Select the Output data rate, range and power mode of Gyroscope sensor */
	BMX160_pObj->gyro_cfg.odr = BMX160_GYRO_ODR_3200HZ;
	BMX160_pObj->gyro_cfg.range = BMX160_GYRO_RANGE_2000_DPS;
	BMX160_pObj->gyro_cfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
	BMX160_pObj->gyro_cfg.power = BMX160_GYRO_NORMAL_MODE;

	BMX160_set_sens_conf(BMX160_pObj);
  /* Infinite loop */
  for(;;)
  {
	 accel = (bmx160_sensor_data*) pvPortMalloc(sizeof(bmx160_sensor_data));
	 gyro = (bmx160_sensor_data*) pvPortMalloc(sizeof(bmx160_sensor_data));
	 //mag = (bmx160_aux_data*) pvPortMalloc(sizeof(bmx160_aux_data));

	 BMX160_get_sensor_data(BMX160_pObj, BMX160_ACCEL_SEL | BMX160_GYRO_SEL, accel, gyro);
	 osMessagePut(bmx160QueueHandle, (uint32_t) accel, 1);

    osDelay(1);
  }
  /* USER CODE END imu */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void bmx160_Timer_Callback(void const * argument){
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}

static void BMX160_delay_ms(uint32_t period){
	if(osTimerStart(bmx160TimerHandle, period) != osOK){
		while(1){
			//TO DEBUG
		}
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
#include "ds600.h"
#include "sph0644.h"
#include "bmx160.h"
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

SE868K3_Object_t* SE868K3_pObj;
SE868K3_IO_t* SE868K3_pIO;
uint8_t* gnss_buffer;

HTS221_Object_t* HTS221_pObj;
HTS221_IO_t* HTS221_pIO;
hts221_data_t* HTS221_Data_Read;

DS600_Object_t* DS600_pObj;
DS600_IO_t* DS600_pIO;
float* corporal_temp;

SPH06440_Object_t* SPH06440_pObj;
SPH06440_IO_t* SPH06440_pIO;
uint8_t* sound;

BMX160_Object_t* BMX160_pObj;
BMX160_IO_t* BMX160_pIO;
bmx160_sensor_data* accel;
bmx160_sensor_data* gyro;
bmx160_aux_data* mag;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId microSDTaskHandle;
osThreadId hts221TaskHandle;
osThreadId gnssTaskHandle;
osThreadId ds600TaskHandle;
osThreadId sph064TaskHandle;
osThreadId bmx160TaskHandle;
osMutexId gnssMutexHandle;
osMessageQId gnssQueueHandle;
osMessageQId hts221_QueueHandle;
osMessageQId ds600QueueHandle;
osMessageQId mphonesQueueHandle;
osMessageQId bmx160QueueHandle;
osTimerId bmx160TimerHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void BMX160_delay_ms(uint32_t period);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void microSD(void const * argument);
void HTS221(void const * argument);
void SE868K3(void const * argument);
void DS600(void const * argument);
void mphones(void const * argument);
void imu(void const * argument);
void bmx160_Timer_Callback(void const * argument);

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


	gnss_buffer = (uint8_t*)pvPortMalloc(GLL_MAX_SIZE);
	SE868K3_pObj = (SE868K3_Object_t*)pvPortMalloc(sizeof(SE868K3_Object_t));
	SE868K3_pIO	= (SE868K3_IO_t *) pvPortMalloc(sizeof(SE868K3_IO_t));
	HTS221_pObj = (HTS221_Object_t*) pvPortMalloc(sizeof(HTS221_Object_t));
	HTS221_pIO	= (HTS221_IO_t *) pvPortMalloc(sizeof(HTS221_IO_t));

	SE868K3_pObj->is_initialized = 0;
	SE868K3_pObj->pileUART = gnss_buffer;
	SE868K3_pIO->Init = BSP_UART1_Init;
	SE868K3_pIO->DeInit = BSP_UART1_DeInit;
	SE868K3_pIO->Read = BSP_UART1_Recv;
	SE868K3_pIO->Write = BSP_UART1_Send;

	if(SE868K3_RegisterBusIO(SE868K3_pObj, SE868K3_pIO) != SE868K3_ERROR){
		osThreadDef(gnssTask, SE868K3, osPriorityNormal, 0, 300);
		gnssTaskHandle = osThreadCreate(osThread(gnssTask), (void*) SE868K3_pObj);
		osMessageQDef(myQueue01, 4, uint32_t);
		gnssQueueHandle = osMessageCreate(osMessageQ(myQueue01), NULL);
	}
	else{
		vPortFree(SE868K3_pObj);
		vPortFree(SE868K3_pIO);
	}

	HTS221_pObj->is_initialized = 0;
	HTS221_pIO->Address = (uint8_t)(HTS221_I2C_ADDRESS << 1);
	HTS221_pIO->BusType = HTS221_I2C_BUS;
	HTS221_pIO->Init = BSP_I2C1_Init;
	HTS221_pIO->DeInit = BSP_I2C1_DeInit;
	HTS221_pIO->ReadReg = BSP_I2C1_Recv;
	HTS221_pIO->WriteReg = BSP_I2C1_Send;

	if(HTS221_RegisterBusIO(HTS221_pObj, HTS221_pIO) != HTS221_ERROR){
		osThreadDef(hts221Task, HTS221, osPriorityNormal, 0, 128);
		hts221TaskHandle = osThreadCreate(osThread(hts221Task), (void*) HTS221_pObj);
		osMessageQDef(myQueue02, 4, sizeof(hts221_data_t));
		hts221_QueueHandle = osMessageCreate(osMessageQ(myQueue02), NULL);
	}
	else{
		vPortFree(HTS221_pObj);
		vPortFree(HTS221_pIO);
	}

	DS600_pObj = (DS600_Object_t*) pvPortMalloc(sizeof(DS600_Object_t));
	DS600_pIO = (DS600_IO_t*) pvPortMalloc(sizeof(DS600_IO_t));
	DS600_pObj->is_initialized = 0;
	DS600_pIO->Init = BSP_ADC1_Init;
	DS600_pIO->DeInit = BSP_ADC1_DeInit;
	DS600_pIO->Read = BSP_ADC1_Read;

	if(DS600_RegisterBusIO(DS600_pObj, DS600_pIO) != DS600_ERROR){
		osThreadDef(dogtempTask, DS600, osPriorityBelowNormal, 0, 96);
		ds600TaskHandle = osThreadCreate(osThread(dogtempTask), (void*) DS600_pObj);
		osMessageQDef(myQueue04, 4, uint32_t);
		ds600QueueHandle = osMessageCreate(osMessageQ(myQueue04), NULL);
	}
	else{
		vPortFree(DS600_pObj);
		vPortFree(DS600_pIO);
	}

	BMX160_pObj = (BMX160_Object_t*) pvPortMalloc(sizeof(BMX160_Object_t));
	BMX160_pIO = (BMX160_IO_t*) pvPortMalloc(sizeof(BMX160_IO_t));
	osTimerDef(bmx160Timer, bmx160_Timer_Callback);
	bmx160TimerHandle = osTimerCreate(osTimer(bmx160Timer), osTimerOnce, NULL);

	BMX160_pObj->is_initialized = 0;
	BMX160_pIO->BusType = BMX160_SPI_INTF;
	BMX160_pIO->Init = BSP_SPI1_Init;
	BMX160_pIO->DeInit = BSP_SPI1_DeInit;
	BMX160_pIO->ReadReg = BSP_SPI1_Recv;
	BMX160_pIO->WriteReg = BSP_SPI1_Send;
	BMX160_pIO->Delayms = BMX160_delay_ms;

	if(BMX160_RegisterBusIO(BMX160_pObj, BMX160_pIO) == BMX160_OK){
		osThreadDef(bmx160Task, imu, osPriorityIdle, 0, 128);
		bmx160TaskHandle = osThreadCreate(osThread(bmx160Task), (void*) BMX160_pObj);
		osMessageQDef(myQueue05, 4, uint32_t);
		bmx160QueueHandle = osMessageCreate(osMessageQ(myQueue05), NULL);
	}
	else{
		vPortFree(BMX160_pObj);
		vPortFree(BMX160_pIO);
		osTimerDelete(bmx160TimerHandle);
	}

	SPH06440_pObj = (SPH06440_Object_t*) pvPortMalloc(sizeof(SPH06440_Object_t));
	SPH06440_pIO = (SPH06440_IO_t*) pvPortMalloc(sizeof(SPH06440_IO_t));

	SPH06440_pObj->is_initialized = 0;
	SPH06440_pIO->Init = BSP_SPI3_Init;
	SPH06440_pIO->Read = BSP_SPI3_Recv;

	if(SPH06440_RegisterBusIO(SPH06440_pObj, SPH06440_pIO) == SPH0644_OK){
		osThreadDef(sph064Task, mphones, osPriorityIdle, 0, 128);
		sph064TaskHandle = osThreadCreate(osThread(sph064Task), (void*) SPH06440_pObj);
		osMessageQDef(myQueue06, 4, uint32_t);
		mphonesQueueHandle = osMessageCreate(osMessageQ(myQueue06), NULL);
	}
	else{
		vPortFree(SPH06440_pObj);
		vPortFree(SPH06440_pIO);
	}

	MX_FATFS_Init();
	if(f_mount(&SDFatFS, (TCHAR const*) SDPath, 1) == FR_OK){ // 1. Register a work area
			if(f_open(&SDFile, "tfm1.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){ // 2. Creating a new file to write it later
				fileCreated = 1;
				osThreadDef(microSDTask, microSD, osPriorityNormal, 0, 200);
				microSDTaskHandle = osThreadCreate(osThread(microSDTask), NULL);
			}
		}

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of gnssMutex */
	osMutexDef(gnssMutex);
	gnssMutexHandle = osMutexCreate(osMutex(gnssMutex));

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
//	osMessageQDef(myQueue01, 4, uint32_t);
//	gnssQueueHandle = osMessageCreate(osMessageQ(myQueue01), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of microSDTask */
//  osThreadDef(microSDTask, microSD, osPriorityNormal, 0, 200);
//  microSDTaskHandle = osThreadCreate(osThread(microSDTask), NULL);

  /* definition and creation of hts221Task */
//  osThreadDef(hts221Task, HTS221, osPriorityNormal, 0, 128);
//  hts221TaskHandle = osThreadCreate(osThread(hts221Task), NULL);

  /* definition and creation of gnssTask */
//  osThreadDef(gnssTask, SE868K3, osPriorityNormal, 0, 300);
//  gnssTaskHandle = osThreadCreate(osThread(gnssTask), (void*) SE868K3_pObj);

  /* definition and creation of dogTempTask */
//  osThreadDef(dogtempTask, dogTemp, osPriorityBelowNormal, 0, 96);
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
	osEvent rx;
	osEvent temp_rx;
	uint8_t byteswritten;

  /* Infinite loop */
  for(;;)
  {
	  byteswritten = 0;
	  	  if(fileCreated && new_cmd){
	  		  if((osMutexWait(gnssMutexHandle, 6)) == osOK){
	  			  rx = osMessageGet(gnssQueueHandle, 0);
	  			  new_cmd = 0; // Cleaning the flag
	  			  osMutexRelease(gnssMutexHandle);
	  			  f_write(&SDFile, (const void *) rx.value.p, BUFFER_PCKT_SIZE, (void *)&byteswritten);
	  			  f_write(&SDFile, "\r\n", strlen("\r\n"), (void *)&byteswritten);
	  			  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  			  //f_sync(&SDFile);
	  		  }
	  	  }
	  	  temp_rx = osMessageGet(hts221_QueueHandle, 1);
	  	  if(temp_rx.status == osEventMessage){
	  		f_write(&SDFile, (const void *) temp_rx.value.p, 1, (void *)&byteswritten);
	  		vPortFree(HTS221_Data_Read);
	  		f_write(&SDFile, "degrees \r\n", strlen("degrees \r\n"), (void *)&byteswritten);
	  	  }
	  	  f_sync(&SDFile);
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

//	HTS221_pIO.Address = (uint8_t) (HTS221_I2C_ADDRESS<<1);
//	HTS221_pIO.BusType = HTS221_I2C_BUS;
//	HTS221_pIO.Init = BSP_I2C1_Init;
//	HTS221_pIO.DeInit = BSP_I2C1_DeInit;
//	HTS221_pIO.ReadReg = BSP_I2C1_Recv;
//	HTS221_pIO.WriteReg = BSP_I2C1_Send;
	//HTS221_pIO.GetTick = BSP_GetTick;
	uint8_t id, status;
	//HTS221_RegisterBusIO(&HTS221_pObj, &HTS221_pIO);
	HTS221_Object_t *HTS221_pObj = (HTS221_Object_t*) argument;
	HTS221_Init(HTS221_pObj);
	HTS221_ReadID(HTS221_pObj, &id);
	if(id == HTS221_ID){
		HTS221_Get_Init_Status(HTS221_pObj, &status);
		HTS221_HUM_Enable(HTS221_pObj);
		HTS221_TEMP_Enable(HTS221_pObj);
	}
  /* Infinite loop */
  for(;;)
  {
	  HTS221_Data_Read = (hts221_data_t*)pvPortMalloc(sizeof(hts221_data_t));
	  HTS221_TEMP_GetTemperature(HTS221_pObj, &HTS221_Data_Read->temperature);
	  HTS221_HUM_GetHumidity(HTS221_pObj, &HTS221_Data_Read->humidity);
	  osMessagePut(hts221_QueueHandle, (uint32_t)HTS221_Data_Read, 2);
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

	//SE868K3_IO_t SE868K3_pIO;
	SE868K3_Object_t *SE868K3_pObj = (SE868K3_Object_t*) argument;
	SE868K3_Init(SE868K3_pObj);
	//SE868K3_pIO.Init = BSP_UART1_Init;
	//SE868K3_pIO.DeInit = BSP_UART1_DeInit;
	//SE868K3_pIO.Read = BSP_UART1_Recv;
	//SE868K3_pIO.Write = BSP_UART1_Send;
	//SE868K3_pIO.GetTick = BSP_GetTick;

//	if(SE868K3_RegisterBusIO(SE868K3_pObj, &SE868K3_pIO) != SE868K3_ERROR){
//		if(SE868K3_Init(SE868K3_pObj) != SE868K3_OK )
//		{
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
//		}
//	}

	//SE868K3_Test(SE868K3_pObj); // re-send the output message rate


  /* Infinite loop */
  for(;;)
  {
	  //if(SE868K3_Read_Packet(SE868K3_pObj, BUFFER_PCKT_SIZE) == 1){
	  if(SE868K3_Read_RMC(SE868K3_pObj) == 1){
		  //if(SE868K3_pObj->pileUART[18] == 'A'){
			  if((osMutexWait(gnssMutexHandle, 6)) == osOK){
				  osMessagePut(gnssQueueHandle, (uint32_t)SE868K3_pObj->pileUART, 0);
				  new_cmd = 1;
				  osMutexRelease(gnssMutexHandle);
			  }
		  //}
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
void DS600(void const * argument)
{
  /* USER CODE BEGIN dogtemp */
	DS600_Object_t *DS600_pObj = (DS600_Object_t*) argument;

  /* Infinite loop */
  for(;;)
  {
	  corporal_temp = (float*) pvPortMalloc(sizeof(float));
	  DS600_GET_Temperature(DS600_pObj, corporal_temp);
	  osMessagePut(ds600QueueHandle, (uint32_t)corporal_temp, 0);
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
	SPH06440_Object_t *SPH06440_pObj = (SPH06440_Object_t*) argument;

  /* Infinite loop */
  for(;;)
  {
	  sound = (uint8_t*)pvPortMalloc(20);
	  SPH06440_GET_samples(SPH06440_pObj, sound, 20);
	  osMessagePut(mphonesQueueHandle, (uint32_t) sound, 5);
    osDelay(30);
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
	BMX160_Object_t *BMX160_pObj = (BMX160_Object_t*) argument;

	/* Select the Output data rate, range and power mode of accelerometer sensor */
	BMX160_pObj->accel_cfg.odr = BMX160_ACCEL_ODR_1600HZ;
	BMX160_pObj->accel_cfg.range = BMX160_ACCEL_RANGE_2G;
	BMX160_pObj->accel_cfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
	BMX160_pObj->accel_cfg.power = BMX160_ACCEL_NORMAL_MODE;

	/* Select the Output data rate, range and power mode of Gyroscope sensor */
	BMX160_pObj->gyro_cfg.odr = BMX160_GYRO_ODR_3200HZ;
	BMX160_pObj->gyro_cfg.range = BMX160_GYRO_RANGE_2000_DPS;
	BMX160_pObj->gyro_cfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
	BMX160_pObj->gyro_cfg.power = BMX160_GYRO_NORMAL_MODE;

	BMX160_set_sens_conf(BMX160_pObj);
  /* Infinite loop */
  for(;;)
  {
	 accel = (bmx160_sensor_data*) pvPortMalloc(sizeof(bmx160_sensor_data));
	 gyro = (bmx160_sensor_data*) pvPortMalloc(sizeof(bmx160_sensor_data));
	 //mag = (bmx160_aux_data*) pvPortMalloc(sizeof(bmx160_aux_data));

	 BMX160_get_sensor_data(BMX160_pObj, BMX160_ACCEL_SEL | BMX160_GYRO_SEL, accel, gyro);
	 osMessagePut(bmx160QueueHandle, (uint32_t) accel, 1);

    osDelay(1);
  }
  /* USER CODE END imu */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void bmx160_Timer_Callback(void const * argument){
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}

static void BMX160_delay_ms(uint32_t period){
	if(osTimerStart(bmx160TimerHandle, period) != osOK){
		while(1){
			//TO DEBUG
		}
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
