/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmx160.h"
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

/* USER CODE BEGIN PV */
struct bmi160_dev sensor;
struct bmi160_sensor_data accel;
struct bmi160_sensor_data gyro;
struct bmi160_int_settg int_config;
SPI_HandleTypeDef hspi1;
static uint8_t flag = 0;
static uint8_t rx_done = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
static int8_t user_spi_read(uint8_t id, uint8_t reg_addr, uint8_t * data, uint16_t len);
static int8_t user_spi_write(uint8_t id, uint8_t reg_addr, uint8_t * data, uint16_t len);
static void user_delay_ms(uint32_t period);


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
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  sensor.id = 0;
  sensor.interface = BMI160_SPI_INTF;
  sensor.read = user_spi_read;
  sensor.write = user_spi_write;
  sensor.delay_ms = user_delay_ms;

  int8_t rslt = BMI160_OK;

  rslt = bmi160_init(&sensor);
  if( rslt == BMI160_OK){ // This means the sensor has been found

	  /* Select the Output data rate, range of accelerometer sensor */
	  sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	  sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
	  sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

	  /* Select the power mode of accelerometer sensor */
	  sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	  /* Select the Output data rate, range of Gyroscope sensor */
	  sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	  sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	  sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

	  /* Select the power mode of Gyroscope sensor */
	  sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	  /* Set the sensor configuration */
	  rslt = bmi160_set_sens_conf(&sensor);
	  if(rslt == BMI160_OK){

		  /* Select the Interrupt channel/pin */
		  int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1

		  /* Select the Interrupt type */
		  int_config.int_type = BMI160_ACC_ANY_MOTION_INT;// Choosing Any motion interrupt

		  /* Select the interrupt channel/pin settings */
		  int_config.int_pin_settg.output_en = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
		  int_config.int_pin_settg.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
		  int_config.int_pin_settg.output_type = BMI160_DISABLE;// Choosing active low output
		  int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;// Choosing edge triggered output
		  int_config.int_pin_settg.input_en = BMI160_DISABLE;// Disabling interrupt pin to act as input
		  int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;// non-latched output

		  /* Select the Any-motion interrupt parameters */
		  int_config.int_type_cfg.acc_any_motion_int.anymotion_en = BMI160_ENABLE;// 1- Enable the any-motion, 0- disable any-motion
		  int_config.int_type_cfg.acc_any_motion_int.anymotion_x = BMI160_ENABLE;// Enabling x-axis for any motion interrupt
		  int_config.int_type_cfg.acc_any_motion_int.anymotion_y = BMI160_ENABLE;// Enabling y-axis for any motion interrupt
		  int_config.int_type_cfg.acc_any_motion_int.anymotion_z = BMI160_ENABLE;// Enabling z-axis for any motion interrupt
		  int_config.int_type_cfg.acc_any_motion_int.anymotion_dur = 0;// any-motion duration
		  int_config.int_type_cfg.acc_any_motion_int.anymotion_thr = 20;// (2-g range) -> (slope_thr) * 3.91 mg, (4-g range) -> (slope_thr) * 7.81 mg, (8-g range) ->(slope_thr) * 15.63 mg, (16-g range) -> (slope_thr) * 31.25 mg

		  /* Set the Any-motion interrupt */
		  rslt = bmi160_set_int_config(&int_config, &sensor); /* sensor is an instance of the structure bmi160_dev  */
  	    		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  	    	}
  }
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);
	  if(flag){ //This will mean any motion has been detected
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  flag = 0;
	  }
	  HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static int8_t user_spi_read(uint8_t id, uint8_t reg_addr, uint8_t * data, uint16_t len){

	int32_t ret = BMI160_OK;
	uint8_t tx_buffer[len];
	tx_buffer[0] = reg_addr;
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, RESET);
	ret = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, data, len, 100);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX_RX);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, SET);
	return (int8_t) ret;

}

static int8_t user_spi_write(uint8_t id, uint8_t reg_addr, uint8_t * data, uint16_t len){

	int32_t ret = BMI160_OK;
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, RESET);
	if(HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 10) != HAL_OK){
		return BMI160_E_COM_FAIL;
	}
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
	if(HAL_SPI_Transmit(&hspi1, data, len, 100) != HAL_OK){
		ret = BMI160_E_COM_FAIL;
	}
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, SET);
	return ret;

}

static void user_delay_ms(uint32_t period){

//	if(osTimerStart(myTimer01Handle, period) != osOK){
//		while(1){
//			//TO DEBUG
//		}
//	}
	HAL_Delay(period);
}

//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
//	tx_done++;
//}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	rx_done++;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
	while(1){
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}
  /* USER CODE END Error_Handler_Debug */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	flag = 1;

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
