/**
 ******************************************************************************
  * @file    bsp_driver_sd.c for L1 (based on stm32l152d_eval_sd.c)
  * @brief   This file includes a generic uSD card driver.
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

#ifdef OLD_API
/* kept to avoid issue when migrating old projects. */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
#else
/* USER CODE BEGIN FirstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END FirstSection */
/* Includes ------------------------------------------------------------------*/
#include "bsp_driver_sd.h"

/* Extern variables ---------------------------------------------------------*/ 
  
extern SD_HandleTypeDef hsd; 

/* Private functions */
HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd); /* dummy function, to be filled in by the user */
HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd); /* dummy function, to be filled in by the user */

/* USER CODE BEGIN BeforeInitSection */
/* can be used to modify / undefine following code or add code */
/* USER CODE END BeforeInitSection */
/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
uint8_t BSP_SD_Init(void)
{
  uint8_t sd_state = MSD_OK;
  /* Check if the SD card is plugged in the slot */
  if (BSP_SD_IsDetected() != SD_PRESENT)
  {
    return MSD_ERROR;
  }
  /* HAL SD initialization */
  sd_state = HAL_SD_Init(&hsd);
  /* Configure SD Bus width (4 bits mode selected) */
  if (sd_state == MSD_OK)
  {
    /* Enable wide operation */
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_1B) != HAL_OK)
    {
      sd_state = MSD_ERROR;
    }
  }

  return sd_state;
}
/* USER CODE BEGIN AfterInitSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END AfterInitSection */

/**
  * @brief  Configures Interrupt mode for SD detection pin.
  * @retval Returns 0 in success otherwise 1. 
  */
uint8_t BSP_SD_ITConfig(void)
{  
  /* TBI: add user code here depending on the hardware configuration used */
  
  return (uint8_t)0;
}

/** @brief  SD detect IT treatment
  * @retval None
  */
void BSP_SD_DetectIT(void)
{
  /* SD detect IT callback */
  BSP_SD_DetectCallback();
  
}

/** @brief  SD detect IT detection callback
  * @retval None
  */
__weak void BSP_SD_DetectCallback(void)
{
  /* NOTE: This function Should not be modified, when the callback is needed,
  the BSP_SD_DetectCallback could be implemented in the user file
  */ 
  
}

/* USER CODE BEGIN BeforeReadBlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeReadBlocksSection */
/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read  
  * @param  NumOfBlocks: Number of SD blocks to read 
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  if(HAL_SD_ReadBlocks(&hsd, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout) != HAL_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode. 
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written  
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  if(HAL_SD_WriteBlocks(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) != HAL_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read  
  * @param  NumOfBlocks: Number of SD blocks to read 
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  uint8_t state = MSD_OK;
  
  /* Invalidate the dma tx handle*/
  hsd.hdmatx = NULL;
    
  /* Prepare the dma channel for a read operation */
  state = SD_DMAConfigRx(&hsd);
  
  if(state == HAL_OK)
  {
    /* Read block(s) in DMA transfer mode */
    state = HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)pData, ReadAddr, NumOfBlocks);
  }
    
  if(state == HAL_OK)
  {
    return MSD_OK;
  }    
  else
  {
    return MSD_ERROR;
  }
}

/* USER CODE BEGIN BeforeWriteDMABlocksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeWriteDMABlocksSection */
/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written  
  * @param  NumOfBlocks: Number of SD blocks to write 
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  uint8_t state = MSD_OK;
  
  /* Invalidate the dma rx handle*/
  hsd.hdmarx = NULL;
    
  /* Prepare the dma channel for a read operation */
  state = SD_DMAConfigTx(&hsd);
  
  if(state == HAL_OK)
  { 
    /* Write block(s) in DMA transfer mode */
    state = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks);
  }
    
  if(state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  } 
}

/* USER CODE BEGIN BeforeEraseSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeEraseSection */
/**
  * @brief  Erases the specified memory area of the given SD card. 
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
  uint8_t sd_state = MSD_OK;

  if (HAL_SD_Erase(&hsd, StartAddr, EndAddr) != HAL_OK)  
  {
    sd_state = MSD_ERROR;
  }

  return sd_state; 
}

/* USER CODE BEGIN BeforeGetCardStateSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeGetCardStateSection */

/**
  * @brief  Gets the current SD card data status.
  * @param  None
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t BSP_SD_GetCardState(void)
{
  HAL_SD_CardStateTypeDef card_state;
  card_state = HAL_SD_GetCardState(&hsd);

  if (card_state == HAL_SD_CARD_TRANSFER)
  {
    return (SD_TRANSFER_OK);
  }
  else if ((card_state == HAL_SD_CARD_SENDING) || 
           (card_state == HAL_SD_CARD_RECEIVING) || 
           (card_state == HAL_SD_CARD_PROGRAMMING))
  {
    return (SD_TRANSFER_BUSY);
  }
  else
  { 
    return(SD_TRANSFER_ERROR);
  }
}

/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None 
  */
void BSP_SD_GetCardInfo(BSP_SD_CardInfo *CardInfo)
{
  /* Get SD card Information */
  HAL_SD_GetCardInfo(&hsd, CardInfo);
}

/* USER CODE BEGIN BeforeCallBacksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeCallBacksSection */
/**
  * @brief SD Abort callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_AbortCallback();
}

/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_WriteCpltCallback();
}

/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_ReadCpltCallback();
}

/* USER CODE BEGIN CallBacksSection_C */
/**
  * @brief BSP SD Abort callback
  * @retval None
  */
__weak void BSP_SD_AbortCallback(void)
{

}

/**
  * @brief BSP Tx Transfer completed callback
  * @retval None
  */
__weak void BSP_SD_WriteCpltCallback(void)
{

}

/**
  * @brief BSP Rx Transfer completed callback
  * @retval None
  */
__weak void BSP_SD_ReadCpltCallback(void)
{

}
/* USER CODE END CallBacksSection_C */
#endif

/**
 * @brief  Detects if SD card is correctly plugged in the memory slot or not.
 * @param  None
 * @retval Returns if SD is detected or not
 */
uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t status = SD_PRESENT;

  /* USER CODE BEGIN 1 */
  /* user code can be inserted here */
  /* USER CODE END 1 */    	

  return status;
}

/* USER CODE BEGIN DMAConfigCode */

HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd)
{
	static DMA_HandleTypeDef hdma_rx;
	HAL_StatusTypeDef status = HAL_OK;

	HAL_DMA_DeInit(hsd->hdmatx);

	/* Configure the DMA Rx parameters */
	hdma_rx.Instance = DMA2_Channel4;
	hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_rx.Init.Mode = DMA_NORMAL;
	hdma_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;

	/* Associate the DMA handle */
	__HAL_LINKDMA(hsd,hdmarx,hdma_rx);

	/* Stop any ongoing transfer and reset the state*/
	  HAL_DMA_Abort(&hdma_rx);

	/* Deinitialize the Channel for new transfer */
	  HAL_DMA_DeInit(&hdma_rx);

	/* Configure the DMA Channel */
	  status = HAL_DMA_Init(&hdma_rx);

	/* NVIC configuration for DMA transfer complete interrupt */
	  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

	  return status;
}


HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd)
{

  static DMA_HandleTypeDef hdma_tx;
  HAL_StatusTypeDef status = HAL_OK;
  
  /* Configure the DMA Tx parameters */
  	hdma_tx.Instance = DMA2_Channel4;
  	hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  	hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  	hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
  	hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  	hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  	hdma_tx.Init.Mode = DMA_NORMAL;
  	hdma_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;

  	/* Associate the DMA handle */
  	__HAL_LINKDMA(hsd,hdmatx,hdma_tx);

  	/* Stop any ongoing transfer and reset the state*/
  	  HAL_DMA_Abort(&hdma_tx);

  	/* Deinitialize the Channel for new transfer */
  	  HAL_DMA_DeInit(&hdma_tx);

  	/* Configure the DMA Channel */
  	  status = HAL_DMA_Init(&hdma_tx);

  	 /* NVIC configuration for DMA transfer complete interrupt */
  	  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 5, 0);
  	  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

  return status;
}
/* USER CODE END DMAConfigCode */

/* USER CODE BEGIN AdditionalCode */
/* user code can be inserted here */
/* USER CODE END AdditionalCode */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
