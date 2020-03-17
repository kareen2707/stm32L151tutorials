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
#define BUFFER_SIZE 88
#define MAX_CMD_SIZE 68
#define UART_RX_BUFFER_SIZE 256
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
//Static buffers to test the read_command() function. With the first one the function result is 1 (found) with the second one 0 (not found)
uint8_t nmea_test[BUFFER_SIZE] = "$GPGSA,A,1,,*1E\r\n$GNRMC,164004.000,A,4027.1783,N,00343.5470,W,0.19,67.20,030320,,,*58\r\n";
//uint8_t nmea_test[BUFFER_SIZE] = "N,00343.5470,W,0.19,67.20,030320,,,*58\r\n$GPGSA,A,1,,*1E\r\n$GNRMC,164004.000,A,4027.1783,";

static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t* uart_rx_ptr_head;
static uint8_t* uart_rx_ptr_tail;
static uint16_t num_stored_bytes;

//static uint8_t* uart_rx_ptr_tail = &nmea_test[0];
static uint8_t cmd[MAX_CMD_SIZE];
static uint8_t new_cmd = 0; //Flag to synchronize the reading and writing tasks
UART_HandleTypeDef huart1;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId consumerTaskHandle;
osThreadId producer1Handle;
osMessageQId myQueue01Handle;
osTimerId myTimer01Handle;
osMutexId myMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int32_t read_command(uint8_t * rxData, uint16_t size);
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
	HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, UART_RX_BUFFER_SIZE);

	uart_rx_ptr_head = uart_rx_buffer;
	uart_rx_ptr_tail = uart_rx_ptr_head;

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	osMutexDef(myMut1);
	myMutexHandle = osMutexCreate(osMutex(myMut1));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
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
  osMessageQDef(myQueue01, 4, uint32_t);
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
  producer1Handle = osThreadCreate(osThread(producer1), NULL);

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
	uint8_t byteswritten;
	//uint8_t * temp_cmd;

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
		  if((osMutexWait(myMutexHandle, 6)) == osOK){
			  rx = osMessageGet(myQueue01Handle, 0);
			  new_cmd = 0; // Cleaning the flag
			  osMutexRelease(myMutexHandle);
			  f_write(&SDFile, (const void *) rx.value.p, MAX_CMD_SIZE, (void *)&byteswritten);
			  f_write(&SDFile, "\r\n", strlen("\r\n"), (void *)&byteswritten);
			  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			  f_sync(&SDFile);
		  }
	  }

    osDelay(110);
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

  /* Infinite loop */
  for(;;)
  {
	
	memset((char *) cmd, '\0', MAX_CMD_SIZE);
	if(read_command(cmd, MAX_CMD_SIZE) == 1){
		if((osMutexWait(myMutexHandle, 6)) == osOK){
			osMessagePut(myQueue01Handle, (uint32_t)cmd, 0);
			new_cmd = 1; //Flag enabled
			osMutexRelease(myMutexHandle);
			
		}
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

int32_t read_command(uint8_t * rxData, uint16_t size){

	uint8_t *new_uart_rx_ptr_head;
	uint32_t dma_counter = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	uint16_t new_bytes = 0;

	if(!dma_counter){
		dma_counter = UART_RX_BUFFER_SIZE;
	}
	new_uart_rx_ptr_head = uart_rx_buffer + UART_RX_BUFFER_SIZE - dma_counter;
	if(new_uart_rx_ptr_head >= uart_rx_ptr_head){
		new_bytes = (uint16_t)(new_uart_rx_ptr_head - uart_rx_ptr_head);
		num_stored_bytes += new_bytes;
	}
	else{
		new_bytes = UART_RX_BUFFER_SIZE - ((uint16_t)(uart_rx_ptr_head - new_uart_rx_ptr_head));
		num_stored_bytes += new_bytes;
	}

	uart_rx_ptr_head = new_uart_rx_ptr_head;
	if(num_stored_bytes >= UART_RX_BUFFER_SIZE){
		num_stored_bytes = UART_RX_BUFFER_SIZE;
		uart_rx_ptr_tail = uart_rx_ptr_head;
	}

	uint16_t read_bytes = 0;
	//uint16_t num_stored_bytes = BUFFER_SIZE;
	//uint8_t* uart_rx_ptr_tail = &nmea_test[0];


	while(num_stored_bytes){
		if(((*uart_rx_ptr_tail) != '\r' ) && ((*uart_rx_ptr_tail) != '\n' ) ){
			if(((*uart_rx_ptr_tail) == ',' ) || ((*uart_rx_ptr_tail) == '*' )){
				rxData[read_bytes] = '\0'; //Filling null values with spaces
			}
			else{
				rxData[read_bytes] = *uart_rx_ptr_tail;
			}
			read_bytes++;
			num_stored_bytes--;
			uart_rx_ptr_tail++;

			if(uart_rx_ptr_tail >= &uart_rx_buffer[UART_RX_BUFFER_SIZE]){
				uart_rx_ptr_tail = uart_rx_buffer;
			}
			if(read_bytes >= size){
				return 2; //error
			}
		}

		else{ //end of line detected

			read_bytes++;
			num_stored_bytes--;
			uart_rx_ptr_tail++;
			if(uart_rx_ptr_tail >= &uart_rx_buffer[UART_RX_BUFFER_SIZE]){
				uart_rx_ptr_tail = uart_rx_buffer;
			}
			if(read_bytes >= size){
				return 2;
			}

			if(strcmp((char *) rxData, "$GNRMC")!= 0){
				read_bytes = 0;
				num_stored_bytes--;
				uart_rx_ptr_tail++;
				if(uart_rx_ptr_tail >= &uart_rx_buffer[UART_RX_BUFFER_SIZE]){
					uart_rx_ptr_tail = uart_rx_buffer;
				}

			}
			else{
					return 1; //found command
				}
		}

	}
	return 0; //command not found
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
