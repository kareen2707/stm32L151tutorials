/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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

#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */
RTC_HandleTypeDef hrtc;
/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */     
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	/* Get local time */
	if(HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN)) return 0;
	if(HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN)) return 0;
	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)sDate.Year << 25)
				| ((DWORD)sDate.Month << 21)
				| ((DWORD)sDate.Date << 16)
				| ((DWORD)sTime.Hours << 11)
				| ((DWORD)sTime.Minutes << 5)
				| ((DWORD)sTime.Seconds >> 1);
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
