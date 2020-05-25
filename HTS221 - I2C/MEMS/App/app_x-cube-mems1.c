/**
  ******************************************************************************
  * File Name          :  stmicroelectronics_x-cube-mems1_7_0_0.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.7.0.0 instances.
  ******************************************************************************
  *
  * COPYRIGHT 2020 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_x-cube-mems1.h"
#include "main.h"
#include <stdio.h>

//Karen
#include "hts221.h"
#include "custom_bus.h"

static float humidity;
static float temperature;
uint8_t ID;
uint8_t status;

HTS221_IO_t HTS221_pIO;
HTS221_Object_t HTS221_pObj;

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */ 

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */
	HTS221_pIO.Address = (uint8_t) (HTS221_I2C_ADDRESS<<1);
	HTS221_pIO.BusType = HTS221_I2C_BUS;
	HTS221_pIO.Init = BSP_I2C1_Init;
	HTS221_pIO.DeInit = BSP_I2C1_DeInit;
	HTS221_pIO.ReadReg = BSP_I2C1_ReadReg;
	HTS221_pIO.WriteReg = BSP_I2C1_WriteReg;
	HTS221_pIO.GetTick = BSP_GetTick;

	HTS221_RegisterBusIO(&HTS221_pObj, &HTS221_pIO);
	HTS221_Init(&HTS221_pObj);

	HTS221_ReadID(&HTS221_pObj, &ID);
	if(ID == HTS221_ID){
			HTS221_Get_Init_Status(&HTS221_pObj, &status);
			HTS221_HUM_Enable(&HTS221_pObj);
			HTS221_TEMP_Enable(&HTS221_pObj);
		}
	else{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		}

  
  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  /* USER CODE BEGIN MEMS_Init_PostTreatment */
  
  /* USER CODE END MEMS_Init_PostTreatment */
}
/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */
  /* USER CODE END MEMS_Process_PreTreatment */
	HTS221_HUM_GetHumidity(&HTS221_pObj, &humidity);
	HTS221_TEMP_GetTemperature(&HTS221_pObj, &temperature);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);


  /* USER CODE BEGIN MEMS_Process_PostTreatment */
  
  /* USER CODE END MEMS_Process_PostTreatment */
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
