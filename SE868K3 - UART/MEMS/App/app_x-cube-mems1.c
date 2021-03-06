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
#include "se868k3.h"
#include "custom_bus.h"

HTS221_IO_t HTS221_pIO;
HTS221_Object_t HTS221_pObj;
SE868K3_IO_t SE868K3_pIO;
SE868K3_Object_t SE868K3_pObj;
static float humidity;
static float temperature;
uint8_t status;
uint8_t ID;

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */ 

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */
	HTS221_pIO.Address = (0x5F<<1);
	HTS221_pIO.BusType = 0;
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
	HTS221_HUM_GetHumidity(&HTS221_pObj, &humidity);
	HTS221_TEMP_GetTemperature(&HTS221_pObj, &temperature);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  /* USER CODE END MEMS_Process_PreTreatment */

  /* USER CODE BEGIN MEMS_Process_PostTreatment */
  
  /* USER CODE END MEMS_Process_PostTreatment */
}

void MX_GNSS_Init(void){

	SE868K3_pIO.Init = BSP_UART1_Init;
	SE868K3_pIO.DeInit = BSP_UART1_DeInit;
	SE868K3_pIO.Read = BSP_UART1_Recv;
	SE868K3_pIO.Write = BSP_UART1_Send;
	//SE868K3_pIO.GetTick = BSP_GetTick;

	SE868K3_RegisterBusIO(&SE868K3_pObj, &SE868K3_pIO);
	SE868K3_Init(&SE868K3_pObj);
}

void MX_GNSS_Process(void)
{
	SE868K3_Test(&SE868K3_pObj);
}
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
