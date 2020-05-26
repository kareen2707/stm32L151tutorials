/**
  ******************************************************************************
  * @file           : custom_bus.c
  * @brief          : source file for the BSP BUS IO driver
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

/* Includes ------------------------------------------------------------------*/
#include "custom_bus.h"
#include "cmsis_os.h"
#include <string.h>
//#include "usart.h"

__weak HAL_StatusTypeDef MX_I2C1_Init(I2C_HandleTypeDef* hi2c);
__weak HAL_StatusTypeDef MX_USART_Init(UART_HandleTypeDef* huart);
__weak HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef* hspi);
__weak HAL_StatusTypeDef MX_ADC_Init(ADC_HandleTypeDef* hadc);
//__weak HAL_StatusTypeDef MX_SPI3_Init(SPI_HandleTypeDef* hspi);

/** @addtogroup BSP
  * @{
  */

/** @addtogroup CUSTOM
  * @{
  */

/** @defgroup CUSTOM_BUS CUSTOM BUS
  * @{
  */
  

/** @defgroup CUSTOM_BUS_Exported_Variables BUS Exported Variables
  * @{
  */

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
ADC_HandleTypeDef hadc1;

/* DMA Handlers */
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_usart1_rx;

/**
  * @}
  */

/** @defgroup CUSTOM_BUS_Private_Variables BUS Private Variables
  * @{
  */

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
static uint32_t IsI2C1MspCbValid = 0;										
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
static uint32_t IsSPI1MspCbValid = 0;
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

#if(USE_HAL_UART_REGISTER_CALLBACKS == 1)
static uint32_t IsUART1MspCbValid = 0;
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

static uint32_t I2C1InitCounter = 0;	
static uint32_t SPI1InitCounter = 0;
static uint32_t UART1InitCounter = 0;
static uint32_t ADC1InitCounter = 0;
static uint32_t SPI3InitCounter = 0;

static uint8_t SPI3_rx_done = 0;
/**
  * @}
  */

/** @defgroup CUSTOM_BUS_Private_FunctionPrototypes  BUS Private Function
  * @{
  */  

static void I2C1_MspInit(I2C_HandleTypeDef* hI2c); 
static void I2C1_MspDeInit(I2C_HandleTypeDef* hI2c);
#if (USE_CUBEMX_BSP_V2 == 1)
static uint32_t I2C_GetTiming(uint32_t clock_src_hz, uint32_t i2cfreq_hz);
static void Compute_PRESC_SCLDEL_SDADEL(uint32_t clock_src_freq, uint32_t I2C_Speed);
static uint32_t Compute_SCLL_SCLH (uint32_t clock_src_freq, uint32_t I2C_speed);
#endif
static int32_t UART_MspInit(UART_HandleTypeDef* uartHandle);
static void UART_MspDeInit(UART_HandleTypeDef* uartHandle);
static int32_t SPI_MspInit(SPI_HandleTypeDef* hSPI);
static void SPI_MspDeInit(SPI_HandleTypeDef* hSPI);
#if (USE_CUBEMX_BSP_V2 == 1)
static uint32_t SPI_GetPrescaler( uint32_t clk_src_hz, uint32_t baudrate_mbps );
#endif
static void ADC_MspInit(ADC_HandleTypeDef* adcHandle);
static void ADC_MspDeInit(ADC_HandleTypeDef* adcHandle);

/**
  * @}
  */

/** @defgroup CUSTOM_LOW_LEVEL_Private_Functions CUSTOM LOW LEVEL Private Functions
  * @{
  */ 
  
/** @defgroup CUSTOM_BUS_Exported_Functions CUSTOM_BUS Exported Functions
  * @{
  */   

/* BUS IO driver over I2C Peripheral */
/*******************************************************************************
                            BUS OPERATIONS OVER I2C
*******************************************************************************/
/**
  * @brief  Initialize I2C HAL
  * @retval BSP status
  */
int32_t BSP_I2C1_Init(void) 
{

  int32_t ret = BSP_ERROR_NONE;
  
  hi2c1.Instance  = I2C1;

  if(I2C1InitCounter++ == 0)
  {     
    if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET)
    {  
    #if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
      /* Init the I2C Msp */
      I2C1_MspInit(&hi2c1);
    #else
      if(IsI2C1MspCbValid == 0U)
      {
        if(BSP_I2C1_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
        {
          return BSP_ERROR_MSP_FAILURE;
        }
      }
    #endif
      if(ret == BSP_ERROR_NONE)
	  {
    	/* Init the I2C */
    	if(MX_I2C1_Init(&hi2c1) != HAL_OK)
    	{
      		ret = BSP_ERROR_BUS_FAILURE;
    	}
    	else
    	{
      		ret = BSP_ERROR_NONE;
    	}
	  }	
    }
  }
  return ret;
}

/**
  * @brief  DeInitialize I2C HAL.
  * @retval BSP status
  */
int32_t BSP_I2C1_DeInit(void) 
{
  int32_t ret = BSP_ERROR_NONE;
  
  if (I2C1InitCounter > 0)
  {       
    if (--I2C1InitCounter == 0)
    {    
  #if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
    	/* DeInit the I2C */ 
    	I2C1_MspDeInit(&hi2c1);
  #endif  
  		/* DeInit the I2C */ 
  		if (HAL_I2C_DeInit(&hi2c1) != HAL_OK) 
  		{
    		ret = BSP_ERROR_BUS_FAILURE;
  		}
    }
  }
  return ret;
}

/**
  * @brief  Check whether the I2C bus is ready.
  * @param DevAddr : I2C device address
  * @param Trials : Check trials number
  *	@retval BSP status
  */
int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials) 
{
  int32_t ret = BSP_ERROR_NONE;
  
  if (HAL_I2C_IsDeviceReady(&hi2c1, DevAddr, Trials, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    ret = BSP_ERROR_BUSY;
  } 
  
  return ret;
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write
  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BSP status
  */

int32_t BSP_I2C1_Send(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;  
  
  if (HAL_I2C_Mem_Write(&hi2c1, DevAddr,Reg, I2C_MEMADD_SIZE_8BIT,pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {    
    if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Read a register of the device through BUS
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to read
  * @param  pData  Pointer to data buffer to read
  * @param  Length Data Length
  * @retval BSP status
  */
int32_t  BSP_I2C1_Recv(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length) 
{
  int32_t ret = BSP_ERROR_NONE;
  
  if (HAL_I2C_Mem_Read(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  { 
    if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}



/**
  * @brief  Send an amount width data through bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C1_Send_Simplex(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {
  int32_t ret = BSP_ERROR_NONE;	  
  
  if (HAL_I2C_Master_Transmit(&hi2c1, DevAddr, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  Receive an amount of data through a bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C1_Recv_Simplex(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {	
  int32_t ret = BSP_ERROR_NONE;
  
  if (HAL_I2C_Master_Receive(&hi2c1, DevAddr, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)  
/**
  * @brief Register Default BSP I2C1 Bus Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_I2C1_RegisterDefaultMspCallbacks (void)
{

  __HAL_I2C_RESET_HANDLE_STATE(&hi2c1);
  
  /* Register MspInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MSPINIT_CB_ID, I2C1_MspInit)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  /* Register MspDeInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MSPDEINIT_CB_ID, I2C1_MspDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  IsI2C1MspCbValid = 1;
  
  return BSP_ERROR_NONE;  
}

/**
  * @brief BSP I2C1 Bus Msp Callback registering
  * @param Callbacks     pointer to I2C1 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_I2C1_RegisterMspCallbacks (BSP_I2C_Cb_t *Callbacks)
{
  /* Prevent unused argument(s) compilation warning */
  __HAL_I2C_RESET_HANDLE_STATE(&hi2c1);  
 
   /* Register MspInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MSPINIT_CB_ID, Callbacks->pMspInitCb)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  /* Register MspDeInit Callback */
  if (HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MSPDEINIT_CB_ID, Callbacks->pMspDeInitCb) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  
  IsI2C1MspCbValid = 1;
  
  return BSP_ERROR_NONE;  
}
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @brief  Return system tick in ms
  * @retval Current HAL time base time stamp
  */
int32_t BSP_GetTick(void) {
  //return HAL_GetTick();
	return osKernelSysTick();
}

/* I2C1 init function */ 

__weak HAL_StatusTypeDef MX_I2C1_Init(I2C_HandleTypeDef* hi2c)
{
  HAL_StatusTypeDef ret = HAL_OK;
  hi2c->Instance = I2C1;
  hi2c->Init.ClockSpeed = 100000;
  hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c->Init.OwnAddress1 = 0;
  hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c->Init.OwnAddress2 = 0;
  hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(hi2c) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}


static void I2C1_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
}


static void I2C1_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
}

/*******************************************************************************
                            BUS OPERATIONS OVER SPI
*******************************************************************************/
/**
  * @brief  Initializes SPI HAL.
  * @retval BSP status
  */
int32_t BSP_SPI1_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  hspi1.Instance  = SPI1;

  if(SPI1InitCounter++ == 0)
  {
	if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET)
	{
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0)
		/* Init the SPI Msp */
		if(SPI_MspInit(&hspi1)){
			ret = BSP_ERROR_BUS_DMA_FAILURE;
		}
#else
		if(IsSPI1MspCbValid == 0U)
		{
			if(BSP_SPI1_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
			{
				return BSP_ERROR_MSP_FAILURE;
			}
		}
#endif
		if(ret == BSP_ERROR_NONE)
		{
			/* Init the SPI */
			if (MX_SPI_Init(&hspi1) != HAL_OK)
			{
				ret = BSP_ERROR_BUS_FAILURE;
			}
		}
	}
  }

  return ret;
}

int32_t BSP_SPI3_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  hspi3.Instance  = SPI3;

  if(SPI3InitCounter++ == 0)
  {
	if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_RESET)
	{
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0)
		/* Init the SPI Msp */
		if(SPI_MspInit(&hspi3)){
			ret = BSP_ERROR_BUS_DMA_FAILURE;
		}
#else
		if(IsSPI1MspCbValid == 0U)
		{
			if(BSP_SPI1_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
			{
				return BSP_ERROR_MSP_FAILURE;
			}
		}
#endif
		if(ret == BSP_ERROR_NONE)
		{
			/* Init the SPI */
			if (MX_SPI_Init(&hspi3) != HAL_OK)
			{
				ret = BSP_ERROR_BUS_FAILURE;
			}
		}
	}
  }

  return ret;
}

/**
  * @brief  DeInitializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BSP_SPI1_DeInit(void)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  if (SPI1InitCounter > 0)
  {
    if (--SPI1InitCounter == 0)
    {
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0)
	  SPI_MspDeInit(&hspi1);
#endif
	  /* DeInit the SPI*/
	  if (HAL_SPI_DeInit(&hspi1) == HAL_OK)
	  {
		ret = BSP_ERROR_NONE;
	  }
	}
  }
  return ret;
}

int32_t BSP_SPI3_DeInit(void)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;
  if (SPI3InitCounter > 0)
  {
    if (--SPI3InitCounter == 0)
    {
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 0)
	  SPI_MspDeInit(&hspi1);
#endif
	  /* DeInit the SPI*/
	  if (HAL_SPI_DeInit(&hspi1) == HAL_OK)
	  {
		ret = BSP_ERROR_NONE;
	  }
	}
  }
  return ret;
}
/**
  * @brief  Receive Data through SPI BUS.
  * @param DevId: not used in SPI
  * @param REg: register address from where the data will be read
  * @param  pData: Pointer to data buffer to receive
  * @param  Length: Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI1_Recv(uint8_t DevId, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t tx_buffer[Length];
  //tx_buffer[0] = Reg | BMX160_SPI_RD_MASK;
  tx_buffer[0] = Reg;
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, RESET); // Setting low level SS to generate the start condition
  if(HAL_SPI_TransmitReceive(&hspi1, tx_buffer, pData, Length, BUS_SPI1_POLL_TIMEOUT) != HAL_OK)
  {
      ret = BSP_ERROR_UNKNOWN_FAILURE;
  }
  while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX_RX);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, SET); // Setting high level SS to generate the stop condition
  return ret;
}


/**
  * @brief  Receive Data through SPI BUS.
  * @param  pData: Pointer to data buffer to receive
  * @param  Length: Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI3_Recv(uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;
  SPI3_rx_done = 0;
  if(HAL_SPI_Receive_DMA(&hspi3, pData, Length) != HAL_OK){
	  ret = BSP_ERROR_UNKNOWN_FAILURE;
  }
  while(!SPI3_rx_done);
  return ret;
}

/**
  * @brief  Receive Data from SPI BUS
  * @param DevId: not used in SPI
  * @param REg: register address from where the data will be read
  * @param  pData: Pointer to data buffer to send
  * @param  Length: Length of data in byte
  * @retval BSP status
  */
int32_t  BSP_SPI1_Send(uint8_t DevId, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;
  //uint8_t temp_Reg;
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, RESET); // Setting low level SS to generate the start condition
  //temp_Reg = Reg & BMX160_SPI_WR_MASK;
  if(HAL_SPI_Transmit(&hspi1, &Reg, 1, BUS_SPI1_POLL_TIMEOUT) != HAL_OK)
  {
      ret = BSP_ERROR_UNKNOWN_FAILURE;
  }
  while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
  if(HAL_SPI_Transmit(&hspi1, pData, Length, BUS_SPI1_POLL_TIMEOUT) != HAL_OK){
	  ret = BSP_ERROR_UNKNOWN_FAILURE;
  }
  while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, SET); // Setting high level SS to generate the stop condition
  return ret;
}


#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
/**
  * @brief Register Default BSP SPI1 Bus Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_SPI1_RegisterDefaultMspCallbacks (void)
{

  __HAL_SPI_RESET_HANDLE_STATE(&hspi1);

  /* Register MspInit Callback */
  if (HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_MSPINIT_CB_ID, SPI_MspInit)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Register MspDeInit Callback */
  if (HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_MSPDEINIT_CB_ID, SPI_MspDeInit) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }
  IsSPI1MspCbValid = 1;

  return BSP_ERROR_NONE;
}

/**
  * @brief BSP SPI1 Bus Msp Callback registering
  * @param Callbacks     pointer to SPI1 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_SPI1_RegisterMspCallbacks (BSP_SPI_Cb_t *Callbacks)
{
  /* Prevent unused argument(s) compilation warning */
  __HAL_SPI_RESET_HANDLE_STATE(&hspi1);

   /* Register MspInit Callback */
  if (HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_MSPINIT_CB_ID, Callbacks->pMspInitCb)  != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  /* Register MspDeInit Callback */
  if (HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_MSPDEINIT_CB_ID, Callbacks->pMspDeInitCb) != HAL_OK)
  {
    return BSP_ERROR_PERIPH_FAILURE;
  }

  IsSPI1MspCbValid = 1;

  return BSP_ERROR_NONE;
}
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */


/* SPI1 init function */

__weak HAL_StatusTypeDef MX_SPI_Init(SPI_HandleTypeDef* hspi)
{
	HAL_StatusTypeDef ret = HAL_OK;
	if(hspi->Instance == SPI1){
		//hspi->Instance = SPI1;
		hspi->Init.Mode = SPI_MODE_MASTER;
		hspi->Init.Direction = SPI_DIRECTION_2LINES;
		hspi->Init.DataSize = SPI_DATASIZE_8BIT;
		hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi->Init.NSS = SPI_NSS_SOFT;
		hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
		hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi->Init.TIMode = SPI_TIMODE_DISABLE;
		hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi->Init.CRCPolynomial = 10;
		if (HAL_SPI_Init(hspi) != HAL_OK)
		{
		    ret = HAL_ERROR;
		}
	}
	else if(hspi->Instance == SPI3){
		hspi->Init.Mode = SPI_MODE_MASTER;
		hspi->Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
		hspi->Init.DataSize = SPI_DATASIZE_8BIT;
		hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi->Init.NSS = SPI_NSS_SOFT;
		hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
		hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi->Init.TIMode = SPI_TIMODE_DISABLE;
		hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi->Init.CRCPolynomial = 10;
		if (HAL_SPI_Init(hspi) != HAL_OK)
		{
			ret = HAL_ERROR;
		}
	}

  return ret;
}

//__weak HAL_StatusTypeDef MX_SPI3_Init(SPI_HandleTypeDef* hspi)
//{
//	HAL_StatusTypeDef ret = HAL_OK;
//	hspi->Instance = SPI3;
//	hspi->Init.Mode = SPI_MODE_MASTER;
//	hspi->Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
//	hspi->Init.DataSize = SPI_DATASIZE_8BIT;
//	hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
//	hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
//	hspi->Init.NSS = SPI_NSS_SOFT;
//	hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//	hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
//	hspi->Init.TIMode = SPI_TIMODE_DISABLE;
//	hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//	hspi->Init.CRCPolynomial = 10;
//	if (HAL_SPI_Init(hspi) != HAL_OK)
//	{
//		ret = HAL_ERROR;
//	}
//	return ret;
//}


static int32_t SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

	int32_t ret = HAL_OK;
	GPIO_InitTypeDef GPIO_InitStruct;
	/* USER CODE BEGIN SPI1_MspInit 0 */
	if(spiHandle->Instance==SPI1){
		/* Enable Peripheral clock */
	      __HAL_RCC_SPI1_CLK_ENABLE();

	      __HAL_RCC_GPIOA_CLK_ENABLE();
	      /**SPI1 GPIO Configuration
	      PA4		------> SPI1_NSS
	      PA5     ------> SPI1_SCK
	      PA6     ------> SPI1_MISO
	      PA7     ------> SPI1_MOSI
	      */

	      HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

	      /*Configure GPIO pin : PtPin */
	      GPIO_InitStruct.Pin = SPI1_NSS_Pin;
	      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	      GPIO_InitStruct.Pull = GPIO_NOPULL;
	      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	      HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

	      GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	      GPIO_InitStruct.Pull = GPIO_NOPULL;
	      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	      GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	      /* Peripheral DMA init*/

	      hdma_spi1_rx.Instance = DMA1_Channel2;
	      hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	      hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	      hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
	      hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	      hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	      hdma_spi1_rx.Init.Mode = DMA_NORMAL;
	      hdma_spi1_rx.Init.Priority = DMA_PRIORITY_HIGH;
	      if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
	      {
	        return HAL_ERROR;
	      }

	    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

	      hdma_spi1_tx.Instance = DMA1_Channel3;
	      hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	      hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	      hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
	      hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	      hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	      hdma_spi1_tx.Init.Mode = DMA_NORMAL;
	      hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;
	      if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
	      {
	      	return HAL_ERROR;
	      }

	    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);
  }
	else if(spiHandle->Instance==SPI3)
	  {
	  /* USER CODE BEGIN SPI3_MspInit 0 */

	  /* USER CODE END SPI3_MspInit 0 */
	    /* SPI3 clock enable */
	    __HAL_RCC_SPI3_CLK_ENABLE();

	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    /**SPI3 GPIO Configuration
	    PB3     ------> SPI3_SCK
	    PB4     ------> SPI3_MISO
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    /* SPI3 DMA Init */
	    /* SPI3_RX Init */
	    hdma_spi3_rx.Instance = DMA2_Channel1;
	    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
	    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    hdma_spi3_rx.Init.Mode = DMA_NORMAL;
	    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_HIGH;
	    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK)
	    {
	    	return HAL_ERROR;
	    }

	    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi3_rx);

	  /* USER CODE BEGIN SPI3_MspInit 1 */

	  /* USER CODE END SPI3_MspInit 1 */
	  }

  /* USER CODE END SPI1_MspInit 0 */

  /* USER CODE BEGIN SPI1_MspInit 1 */
  return ret;
  /* USER CODE END SPI1_MspInit 1 */
}

static void SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
  /* USER CODE BEGIN SPI1_MspDeInit 0 */
	 if(spiHandle->Instance==SPI1)
	  {
		 /* Peripheral clock disable */
		 __HAL_RCC_SPI1_CLK_DISABLE();

		 /**SPI1 GPIO Configuration
		   PA5     ------> SPI1_SCK
		   PA6     ------> SPI1_MISO
		   PA7     ------> SPI1_MOSI
		 */
		 HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

		 /* Peripheral DMA DeInit*/
		 HAL_DMA_DeInit(spiHandle->hdmarx);
		 HAL_DMA_DeInit(spiHandle->hdmatx);
	  }
  /* USER CODE END SPI1_MspDeInit 0 */

	 else if(spiHandle->Instance==SPI3)
	   {
	   /* USER CODE BEGIN SPI3_MspDeInit 0 */

	   /* USER CODE END SPI3_MspDeInit 0 */
	     /* Peripheral clock disable */
	     __HAL_RCC_SPI3_CLK_DISABLE();

	     /**SPI3 GPIO Configuration
	     PB3     ------> SPI3_SCK
	     PB4     ------> SPI3_MISO
	     */
	     HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4);

	     /* SPI3 DMA DeInit */
	     HAL_DMA_DeInit(spiHandle->hdmarx);
	   /* USER CODE BEGIN SPI3_MspDeInit 1 */

	   /* USER CODE END SPI3_MspDeInit 1 */
	   }
}

#if (USE_CUBEMX_BSP_V2 == 1)
/**
  * @brief  Convert the SPI baudrate into prescaler.
  * @param  clock_src_hz : SPI source clock in HZ.
  * @param  baudrate_mbps : SPI baud rate in mbps.
  * @retval Prescaler dividor
  */
static uint32_t SPI_GetPrescaler( uint32_t clock_src_hz, uint32_t baudrate_mbps )
{
  uint32_t divisor = 0;
  uint32_t spi_clk = clock_src_hz;
  uint32_t presc = 0;

  static const uint32_t baudrate[]=
  {
    SPI_BAUDRATEPRESCALER_2,
    SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,
    SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,
    SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128,
    SPI_BAUDRATEPRESCALER_256,
  };

  while( spi_clk > baudrate_mbps)
  {
    presc = baudrate[divisor];
    if (++divisor > 7)
      break;

    spi_clk= ( spi_clk >> 1);
  }

  return presc;
}
#endif


/*******************************************************************************
                            BUS OPERATIONS OVER USART
*******************************************************************************/

static uint8_t 	uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t* uart_rx_ptr_head;			//Last byte stored in the buffer
static uint8_t* uart_rx_ptr_tail;			//First byte stored in the buffer
static uint16_t num_stored_bytes;


__weak HAL_StatusTypeDef MX_USART_Init(UART_HandleTypeDef* huart)
{
	HAL_StatusTypeDef ret = HAL_OK;
	if(huart->Instance == USART1){
		//huart->Instance = USART1; //Karen: before huart1.Instances, same with the rest of parameters
		huart->Init.BaudRate = 9600;
		huart->Init.WordLength = UART_WORDLENGTH_8B;
		huart->Init.StopBits = UART_STOPBITS_1;
		huart->Init.Parity = UART_PARITY_NONE;
		huart->Init.Mode = UART_MODE_TX_RX;
		huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart->Init.OverSampling = UART_OVERSAMPLING_16;
		if (HAL_UART_Init(huart) != HAL_OK) //Karen: before &huart1
		{
			ret = HAL_ERROR;
		}
		uart_rx_ptr_head = uart_rx_buffer;
		uart_rx_ptr_tail = uart_rx_ptr_head;
	}
	/*Karen: if more USARTs will be used here below add the configuration for each of them. See MX_SPI_Init example*/

	return ret;
}
/**
  * @brief  Initialize UART HAL
  * @retval BSP status
  */
int32_t BSP_UART1_Init(void)
{

  int32_t ret = BSP_ERROR_NONE;

  huart1.Instance  = USART1;

  if(UART1InitCounter++ == 0)
  {
    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_RESET)
    {
    #if (USE_HAL_UART_REGISTER_CALLBACKS == 0)
      /* Init the UART Msp */
      if(UART_MspInit(&huart1)){
    	  ret = BSP_ERROR_BUS_FAILURE;
      }
    #else
      if(IsI2C1MspCbValid == 0U)
      {
        if(BSP_I2C1_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
        {
          return BSP_ERROR_MSP_FAILURE;
        }
      }
    #endif
      if(ret == BSP_ERROR_NONE)
	  {
    	/* Init the I2C */
    	if(MX_USART_Init(&huart1) != HAL_OK)
    	{
      		ret = BSP_ERROR_BUS_FAILURE;
    	}
    	else if(HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, UART_RX_BUFFER_SIZE) != HAL_OK)
    	{
    		ret = BSP_ERROR_PERIPH_FAILURE;
    	}
    	else
    	{
      		ret = BSP_ERROR_NONE;
    	}
	  }
    }
  }
  return ret;
}


/**
  * @brief  DeInitialize UART HAL.
  * @retval BSP status
  */
int32_t BSP_UART1_DeInit(void)
{
  int32_t ret = BSP_ERROR_NONE;

  if (UART1InitCounter > 0)
  {
    if (--UART1InitCounter == 0)
    {
  #if (USE_HAL_UART_REGISTER_CALLBACKS == 0)
    	/* DeInit the I2C */
    	UART_MspDeInit(&huart1);
  #endif
  		/* DeInit the I2C */
  		if (HAL_UART_DeInit(&huart1) != HAL_OK)
  		{
    		ret = BSP_ERROR_BUS_FAILURE;
  		}
    }
  }
  return ret;
}

/**
  * @brief  Send an amount width data through bus
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @param 	timeout: Timeout to transmit
  * @retval BSP status
  */
int32_t BSP_UART1_Send(uint8_t *pData, uint16_t Length, uint32_t timeout){

	int32_t ret = BSP_ERROR_NONE;
	if(HAL_UART_Transmit(&huart1, pData, Length, timeout) != HAL_OK){
		ret = BSP_ERROR_PERIPH_FAILURE;
	}
	return ret;
}

/**
  * @brief  Receive an amount of data using DMA
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */

//int32_t BSP_UART1_Recv(uint8_t *rxData, uint16_t size){ //Karen implementation
int32_t BSP_UART1_Recv(uint8_t *pData, uint16_t Length, char *message){
	//int32_t ret = BSP_ERROR_NONE;
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

	while(num_stored_bytes){
		if(((*uart_rx_ptr_tail) != '\r' ) && ((*uart_rx_ptr_tail) != '\n' ) ){
			if(((*uart_rx_ptr_tail) == ',' ) || ((*uart_rx_ptr_tail) == '*' )){
				pData[read_bytes] = '\0'; //Filling null values with spaces
			}
			else{
				pData[read_bytes] = *uart_rx_ptr_tail;
			}
			read_bytes++;
			num_stored_bytes--;
			uart_rx_ptr_tail++;

			if(uart_rx_ptr_tail >= &uart_rx_buffer[UART_RX_BUFFER_SIZE]){
				uart_rx_ptr_tail = uart_rx_buffer;
			}
			if(read_bytes >= Length){
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
			if(read_bytes >= Length){
				return 2;
			}

			//if(strcmp((char *) rxData, "$GNRMC")!= 0){
			if(strcmp((char *) pData, message)!= 0){ //Karen: new generic reception
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


static int32_t UART_MspInit(UART_HandleTypeDef* uartHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	int32_t ret = HAL_OK;
	  if(uartHandle->Instance==USART1)
	  {
	  /* USER CODE BEGIN USART1_MspInit 0 */

	  /* USER CODE END USART1_MspInit 0 */
	    /* USART1 clock enable */
	    __HAL_RCC_USART1_CLK_ENABLE();

	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    /**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    /* USART1 DMA Init */
	    /* USART1_RX Init */
	    hdma_usart1_rx.Instance = DMA1_Channel5;
	    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR; //KAREN: be careful with the code regeneration
	    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
	    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
	    {
	      //ret = HAL_ERROR;
	    	return HAL_ERROR;
	    }

	    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

	    /* USART1 interrupt Init */
	    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	    HAL_NVIC_EnableIRQ(USART1_IRQn);

	  }
	  return ret;
}

static void UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
	if(uartHandle->Instance==USART1)
	  {
	  /* USER CODE BEGIN USART1_MspDeInit 0 */

	  /* USER CODE END USART1_MspDeInit 0 */
	    /* Peripheral clock disable */
	    __HAL_RCC_USART1_CLK_DISABLE();

	    /**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
	    */
	    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

	    /* USART1 DMA DeInit */
	    HAL_DMA_DeInit(uartHandle->hdmarx);

	    /* USART1 interrupt Deinit */
	    HAL_NVIC_DisableIRQ(USART1_IRQn);
	  /* USER CODE BEGIN USART1_MspDeInit 1 */

	  /* USER CODE END USART1_MspDeInit 1 */
	  }
}

/*******************************************************************************
                            BUS OPERATIONS OVER ADC
*******************************************************************************/

/**
  * @brief  Initialize adc HAL
  * @retval BSP status
  */
int32_t BSP_ADC1_Init(void)
{

  int32_t ret = BSP_ERROR_NONE;

  hadc1.Instance  = ADC1;

  if(ADC1InitCounter++ == 0)
  {

    if (HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_RESET)
    {
      /* Init the I2C Msp */
      ADC_MspInit(&hadc1);
      if(ret == BSP_ERROR_NONE)
	  {
    	/* Init the I2C */
    	if(MX_ADC_Init(&hadc1) != HAL_OK)
    	{
      		ret = BSP_ERROR_BUS_FAILURE;
    	}
    	else
    	{
      		ret = BSP_ERROR_NONE;
    	}
	  }
    }
  }
  return ret;
}


/**
  * @brief  DeInitialize ADC HAL.
  * @retval BSP status
  */
int32_t BSP_ADC1_DeInit(void)
{
  int32_t ret = BSP_ERROR_NONE;

  if (ADC1InitCounter > 0)
  {
    if (--ADC1InitCounter == 0)
    {
    	/* DeInit the ADC */
    	ADC_MspDeInit(&hadc1);
    }
  }
  return ret;
}

__weak HAL_StatusTypeDef MX_ADC_Init(ADC_HandleTypeDef* hadc)
{
	HAL_StatusTypeDef ret = HAL_OK;
	ADC_ChannelConfTypeDef sConfig = {0};

	  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	  */
	if(hadc->Instance == ADC1){
		 //hadc->Instance = ADC1;
		hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
		hadc->Init.Resolution = ADC_RESOLUTION_12B;
		hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc->Init.ScanConvMode = ADC_SCAN_DISABLE;
		hadc->Init.EOCSelection = ADC_EOC_SEQ_CONV;
		hadc->Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
		hadc->Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
		hadc->Init.ChannelsBank = ADC_CHANNELS_BANK_A;
		hadc->Init.ContinuousConvMode = DISABLE;
		hadc->Init.NbrOfConversion = 1;
		hadc->Init.DiscontinuousConvMode = DISABLE;
		hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		hadc->Init.DMAContinuousRequests = DISABLE;
		if (HAL_ADC_Init(hadc) != HAL_OK)
		{
			 //Error_Handler();
			return HAL_ERROR;
		}
		/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.*/
		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
		if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
		{
			//Error_Handler();
			return HAL_ERROR;
		}
	}
	/*Karen: If ADC2 will be used, here below add the code needed to configure it as MX_SPI_Init*/

	  return ret;
}

/**
  * @brief  Read tha analog value
  * @param  pData  Pointer to data variable to store the data read
  * @retval BSP status
  */
int32_t  BSP_ADC1_Read(uint16_t *pData)
{
  int32_t ret = BSP_ERROR_NONE;
  HAL_ADC_Start(&hadc1);

  if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC))
  {
    *pData = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
  }
  return ret;
}

static void ADC_MspInit(ADC_HandleTypeDef * adcHandle){

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	  if(adcHandle->Instance==ADC1)
	  {
	  /* USER CODE BEGIN ADC1_MspInit 0 */

	  /* USER CODE END ADC1_MspInit 0 */
	    /* ADC1 clock enable */
	    __HAL_RCC_ADC1_CLK_ENABLE();

	    __HAL_RCC_GPIOC_CLK_ENABLE();
	    /**ADC GPIO Configuration
	    PC2     ------> ADC_IN12
	    */
	    GPIO_InitStruct.Pin = DOGTEMP_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(DOGTEMP_GPIO_Port, &GPIO_InitStruct);

	    /* ADC1 interrupt Init */
	    HAL_NVIC_SetPriority(ADC1_IRQn, 5, 0);
	    HAL_NVIC_EnableIRQ(ADC1_IRQn);
	  /* USER CODE BEGIN ADC1_MspInit 1 */

	  /* USER CODE END ADC1_MspInit 1 */
	  }

	  /*Karen: If ADC2 will be used, here below add the code needed to configure the HW pines needed*/
}

static void ADC_MspDeInit(ADC_HandleTypeDef* adcHandle){

	if(adcHandle->Instance==ADC1)
	  {
	  /* USER CODE BEGIN ADC1_MspDeInit 0 */

	  /* USER CODE END ADC1_MspDeInit 0 */
	    /* Peripheral clock disable */
	    __HAL_RCC_ADC1_CLK_DISABLE();

	    /**ADC GPIO Configuration
	    PC2     ------> ADC_IN12
	    */
	    HAL_GPIO_DeInit(DOGTEMP_GPIO_Port, DOGTEMP_Pin);

	    /* ADC1 interrupt Deinit */
	    HAL_NVIC_DisableIRQ(ADC1_IRQn);
	  /* USER CODE BEGIN ADC1_MspDeInit 1 */

	  /* USER CODE END ADC1_MspDeInit 1 */
	  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi->Instance == SPI3){
		SPI3_rx_done++;
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
