#include "bmx160.h"

/**
 * @}
 */

/** @defgroup LSM6DSR_Private_Function_Prototypes LSM6DSR Private Function Prototypes
 * @{
 */

static int32_t ReadRegWrap (void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static void	   DelayWrap   (void *Handle, uint32_t Period);


/** @defgroup LSM6DSR_Exported_Functions LSM6DSR Exported Functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t BMX160_RegisterBusIO(BMX160_Object_t *pObj, BMX160_IO_t *pIO)
{
	int32_t ret = BMX160_OK;
	uint8_t data;
	uint8_t try = 3;

	if (pObj == NULL)
	{
		ret = BMX160_E_NULL_PTR;
	}
	else
	{
		pObj->IO.Init      = pIO->Init;
		pObj->IO.DeInit    = pIO->DeInit;
		pObj->IO.BusType   = pIO->BusType;
		pObj->IO.Address   = pIO->Address;
		pObj->IO.WriteReg  = pIO->WriteReg;
		pObj->IO.ReadReg   = pIO->ReadReg;
		pObj->IO.GetTick   = pIO->GetTick;
		pObj->IO.Delay	   = pIO->Delay; //Karen: this must be implemented in custom_bus

		pObj->Ctx.read_reg  = ReadRegWrap;
		pObj->Ctx.write_reg = WriteRegWrap;
		pObj->Ctx.delay		= DelayWrap;
		pObj->Ctx.handle   	= pObj;
		
		pObj->chip_id		= 0;

		if (pObj->IO.Init == NULL)
		{
			ret = BMX160_E_DEV_NOT_FOUND;
		}
		else(pObj->IO.Init() != BMX160_OK)
		{
			ret = BMX160_E_COM_FAIL;
		}
	}

	return ret;
}

int32_t BMX160_Soft_Reset(BMX160_Object_t *pObj){
	
	int32_t ret;
	uint8_t data = BMX160_SOFT_RESET_CMD;
	/* Reset the device */
    ret = bmi160_write_regs(&(pObj->Ctx), BMX160_COMMAND_REG_ADDR, &data, 1);
	bmx160_delay(&(pObj->Ctx), BMX160_SOFT_RESET_DELAY_MS);
	if((ret == BMX160_OK) && (pObj->IO.BusType == BMX160_SPI_4WIRES_BUS)){
		ret = bmx160_read_regs(&(pObj->Ctx), BMX160_SPI_COMM_TEST_ADDR, &data, 1);
	}
	if(ret == BMX160_OK){
		//default_param_settg(pObj);
		/* Initializing accel and gyro params with default values */
		pObj->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
		pObj->accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
		pObj->accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
		pObj->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
		pObj->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
		pObj->gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
		pObj->gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
		pObj->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;

		/* To maintain the previous state of accel configuration */
		pObj->prev_accel_cfg = pObj->accel_cfg;

		/* To maintain the previous state of gyro configuration */
		pObj->prev_gyro_cfg = pObj->gyro_cfg;
	}
	
}

//bmx160_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len
int32_t BMX160_Init(LSM6DSR_Object_t *pObj)
{
	int32_t ret = BMX160_OK;
	uint8_t data;
	uint8_t try = 3;
	
	if(pObj->IO.BusType == BMX160_SPI_4WIRES_BUS){ // Dummy read of BMX160_SPI_COMM_TEST_ADDR register to enable SPI Interface 
		pObj->dummy_byte = 1;
		ret = bmx160_read_regs(&(pObj->Ctx), BMX160_SPI_COMM_TEST_ADDR, &data, 1);
	}
	else{
		pObj->dummy_byte = 1;
	}
		
	if(ret == BMX160_OK){
		while((try--) && (pObj->chip_id != BMX160_CHIP_ID)){
			ret = bmx160_read_regs(&(pObj->Ctx), BMX160_CHIP_ID_ADDR, &data, 1); // Read chip_id 
		}
		if((ret == BMX160_OK) && (pObj->chip_id == BMX160_CHIP_ID)){
			pObj->any_sig_sel = BMX160_BOTH_ANY_SIG_MOTION_DISABLED;
			ret = BMX160_Soft_Reset(pObj);
		}
		else{
			ret = BMX160_E_DEV_NOT_FOUND;
		}
	}
	return ret;
}

/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  BMX160_Object_t *pObj = (BMX160_Object_t *)Handle;

  return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
}

/**
 * @brief  Wrap Write register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  BMX160_Object_t *pObj = (BMX160_Object_t *)Handle;

  return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
}

static void DelayWrap(void *Handle, uint32_t Period)
{
	BMX160_Object_t *pObj = (BMX160_Object_t *)Handle;
	pObj->IO.Delay(Period);
}