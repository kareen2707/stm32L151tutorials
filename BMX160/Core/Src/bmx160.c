/*
 * bmx160.c
 *
 *  Created on: 19 mar. 2020
 *      Author: kareen
 */

#include "bmx160.h"

/**
 * @}
 */

/** @defgroup LSM6DSR_Private_Function_Prototypes LSM6DSR Private Function Prototypes
 * @{
 */
static int32_t set_ACCEL_Conf(BMX160_Object_t *pObj);
static int32_t set_GYRO_Conf(BMX160_Object_t *pObj);
static int32_t check_ACCEL_Conf(BMX160_Object_t *pObj, uint8_t *data);
static int32_t check_GYRO_Conf(BMX160_Object_t *pObj, uint8_t *data);
static int32_t process_accel_odr(BMX160_Object_t *pObj, uint8_t *data);
static int32_t process_accel_bw(BMX160_Object_t *pObj, uint8_t *data);
static int32_t process_accel_range(BMX160_Object_t *pObj, uint8_t *data);
static int32_t set_accel_pwr(BMX160_Object_t *pObj);
static int32_t process_gyro_odr(BMX160_Object_t *pObj, uint8_t *data);
static int32_t process_gyro_bw(BMX160_Object_t *pObj, uint8_t *data);
static int32_t process_gyro_range(BMX160_Object_t *pObj, uint8_t *data);
static int32_t set_gyro_pwr(BMX160_Object_t *pObj);
static int32_t process_under_sampling(BMX160_Object_t *pObj, uint8_t *data);
static int32_t check_invalid_settings(BMX160_Object_t *pObj);
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

		ret = pObj->IO.Init();
	}

	return ret;
}

int32_t BMX160_Soft_Reset(BMX160_Object_t *pObj){

	int32_t ret;
	uint8_t data = BMX160_SOFT_RESET_CMD;
	/* Reset the device */
    ret = bmx160_write_reg(&(pObj->Ctx), BMX160_COMMAND_REG_ADDR, &data, 1);
	bmx160_delay(&(pObj->Ctx), BMX160_SOFT_RESET_DELAY_MS);
	if((ret == BMX160_OK) && (pObj->IO.BusType == BMX160_SPI_4WIRES_BUS)){
		ret = bmx160_read_reg(&(pObj->Ctx), BMX160_SPI_COMM_TEST_ADDR, &data, 1);
	}
	if(ret == BMX160_OK){
		//default_param_settg(pObj);
		/* Initializing accel and gyro params with default values */
		pObj->accel_cfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
		pObj->accel_cfg.odr = BMX160_ACCEL_ODR_100HZ;
		pObj->accel_cfg.power = BMX160_ACCEL_SUSPEND_MODE;
		pObj->accel_cfg.range = BMX160_ACCEL_RANGE_2G;
		pObj->gyro_cfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
		pObj->gyro_cfg.odr = BMX160_GYRO_ODR_100HZ;
		pObj->gyro_cfg.power = BMX160_GYRO_SUSPEND_MODE;
		pObj->gyro_cfg.range = BMX160_GYRO_RANGE_2000_DPS;

		/* To maintain the previous state of accel configuration */
		pObj->prev_accel_cfg = pObj->accel_cfg;

		/* To maintain the previous state of gyro configuration */
		pObj->prev_gyro_cfg = pObj->gyro_cfg;
	}
	return ret;
}

//bmx160_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len
int32_t BMX160_Init(BMX160_Object_t *pObj)
{
	int32_t ret = BMX160_OK;
	uint8_t data;
	uint8_t try = 3;

	if(pObj->IO.BusType == BMX160_SPI_4WIRES_BUS){ // Dummy read of BMX160_SPI_COMM_TEST_ADDR register to enable SPI Interface
		pObj->dummy_byte = 1;
		ret = bmx160_read_reg(&(pObj->Ctx), BMX160_SPI_COMM_TEST_ADDR, &data, 1);
	}
	else{
		pObj->dummy_byte = 1;
	}

	if(ret == BMX160_OK){
		while((try--) && (pObj->chip_id != BMX160_CHIP_ID)){
			ret = bmx160_read_reg(&(pObj->Ctx), BMX160_CHIP_ID_ADDR, &pObj->chip_id, 1); // Read chip_id
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

/*!
 * @brief This API configures the power mode, range and bandwidth  of sensor.
 */

int32_t BMX160_Set_Sens_Conf(BMX160_Object_t *pObj){

	int32_t res = BMX160_OK;

	res = set_ACCEL_Conf(pObj);
	if(res == BMX160_OK){
		res = set_GYRO_Conf(pObj);
		if(res == BMX160_OK){
			/* write power mode for accel and gyro */
			res = BMX160_Set_PWR_Mode(pObj);
			if(res == BMX160_OK){
				res = check_invalid_settings(pObj);
			}
		}
	}
	return res;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */

int32_t BMX160_Set_PWR_Mode(BMX160_Object_t *pObj){

	int32_t res;

	res = set_accel_pwr(pObj);
	if(res == BMX160_OK){
		res = set_gyro_pwr(pObj);
	}
	return res;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
int32_t BMX160_Get_PWR_Mode(BMX160_Object_t *pObj, bmx160_pmu_status *pmu_status){

	int32_t res= BMX160_OK;
	uint8_t power_mode = 0;

	res = bmx160_read_reg(&(pObj->Ctx), BMX160_PMU_STATUS_ADDR, &power_mode, 1);
	if(res == BMX160_OK){
		/* Power mode of the accel,gyro,aux sensor is obtained */
		 pmu_status->aux_pmu_status = BMX160_GET_BITS_POS_0(power_mode, BMX160_MAG_POWER_MODE);
		 pmu_status->gyro_pmu_status = BMX160_GET_BITS(power_mode, BMX160_GYRO_POWER_MODE);
		 pmu_status->accel_pmu_status = BMX160_GET_BITS(power_mode, BMX160_ACCEL_POWER_MODE);
	}
	return res;
}
/*!
 * @brief This API check the accel configuration.
 */
static int32_t set_ACCEL_Conf(BMX160_Object_t *pObj){

	int32_t res;
	uint8_t data[2] = {0};

	res = check_ACCEL_Conf(pObj, data);
	if(res == BMX160_OK){
		/* Write output data rate and bandwidth */
		res = bmx160_write_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, &data[0], 1);
		if(res == BMX160_OK){
			pObj->prev_accel_cfg.odr = pObj->accel_cfg.odr;
			pObj->prev_accel_cfg.bw = pObj->accel_cfg.bw;

			/* write accel range */
			res = bmx160_write_reg(&(pObj->Ctx), BMX160_ACCEL_RANGE_ADDR, &data[1], 1);
			if(res == BMX160_OK){
				pObj->prev_accel_cfg.range = pObj->accel_cfg.range;
			}
		}

	}

	return res;
}

static int32_t check_ACCEL_Conf(BMX160_Object_t *pObj, uint8_t *data){

	int32_t res;
	res = bmx160_read_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, data, 2);
	if(res == BMX160_OK){
		res = process_accel_odr(pObj, &data[0]);
		if(res == BMX160_OK){
			res = process_accel_bw(pObj, &data[0]);
			if(res == BMX160_OK){
				res = process_accel_range(pObj, &data[1]);
			}
		}
	}
	return res;
}

static int32_t process_accel_odr(BMX160_Object_t *pObj, uint8_t *data){

	int32_t res = BMX160_OK;
	uint8_t temp = 0;
	uint8_t odr = 0;

	if(pObj->accel_cfg.odr <= BMX160_ACCEL_ODR_MAX){
		if(pObj->accel_cfg.odr != pObj->prev_accel_cfg.odr){
			odr = (uint8_t)pObj->accel_cfg.odr;
			temp = *data & ~BMX160_ACCEL_ODR_MASK;
			/* Adding output data rate */
			*data = temp | (odr & BMX160_ACCEL_ODR_MASK);
		}
	}
	else{
		res = BMX160_E_OUT_OF_RANGE;
	}
	return res;
}

static int32_t process_accel_bw(BMX160_Object_t *pObj, uint8_t *data){

	int32_t res = BMX160_OK;
	uint8_t temp = 0;
	uint8_t bw = 0;

	if(pObj->accel_cfg.bw <= BMX160_ACCEL_BW_MAX){
		if(pObj->accel_cfg.bw != pObj->prev_accel_cfg.bw){
			bw = (uint8_t)pObj->accel_cfg.bw;
			temp = *data & ~BMX160_ACCEL_BW_MASK;
			/* Adding bandwidth */
			*data = temp | ((bw << 4) & BMX160_ACCEL_ODR_MASK);
		}
	}
	else{
		res = BMX160_E_OUT_OF_RANGE;
	}
		return res;

}

static int32_t process_accel_range(BMX160_Object_t *pObj, uint8_t *data){
	int32_t res = BMX160_OK;
	uint8_t temp = 0;
	uint8_t range = 0;

	if(pObj->accel_cfg.range <= BMX160_ACCEL_RANGE_MAX){
		if(pObj->accel_cfg.range != pObj->prev_accel_cfg.range){
			range = (uint8_t)pObj->accel_cfg.range;
			temp = *data & ~BMX160_ACCEL_RANGE_MASK;
			/* Adding output data rate */
			*data = temp | (range & BMX160_ACCEL_RANGE_MASK);
		}
	}
	else{
		res = BMX160_E_OUT_OF_RANGE;
	}
	return res;
}

static int32_t set_accel_pwr(BMX160_Object_t *pObj){

	int32_t res = BMX160_OK;
	uint8_t data = 0;
	if((pObj->accel_cfg.power >= BMX160_ACCEL_SUSPEND_MODE) && (pObj->accel_cfg.power <= BMX160_ACCEL_LOWPOWER_MODE)){
		if(pObj->accel_cfg.power != pObj->prev_accel_cfg.power){
			res = process_under_sampling(pObj, &data);
			if(res == BMX160_OK){
				/* Write accel power */
				res = bmx160_write_reg(&(pObj->Ctx), BMX160_COMMAND_REG_ADDR, &pObj->accel_cfg.power, 1);
				/* Add delay of 3.8 ms - refer data sheet table 24*/
				if(pObj->prev_accel_cfg.power == BMX160_ACCEL_SUSPEND_MODE){
					bmx160_delay(&(pObj->Ctx), BMX160_ACCEL_DELAY_MS);
				}
				pObj->prev_accel_cfg.power = pObj->accel_cfg.power;
			}
		}
	}
	else{
		res = BMX160_E_OUT_OF_RANGE;
	}
	return res;
}

static int32_t process_under_sampling(BMX160_Object_t *pObj, uint8_t *data){

	int32_t res;
	uint8_t temp = 0;
	uint8_t pre_filter = 0;

	res = bmx160_read_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, data, 1);
	if(res == BMX160_OK){
		if(pObj->accel_cfg.power == BMX160_ACCEL_LOWPOWER_MODE){
			temp = *data & ~BMX160_ACCEL_UNDERSAMPLING_MASK;
			/* Set under-sampling parameter */
			*data = temp | ((1 << 7) & BMX160_ACCEL_UNDERSAMPLING_MASK);
			/* Write data */
			res = bmx160_write_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, data, 1);
			/* disable the pre-filter data in low power mode */
			if(res == BMX160_OK){
				/* Disable the Pre-filter data*/
				res = bmx160_write_reg(&(pObj->Ctx), BMX160_INT_DATA_0_ADDR, &pre_filter, 2);
			}
		}
		else if(*data & BMX160_ACCEL_UNDERSAMPLING_MASK){
			temp = *data & ~BMX160_ACCEL_UNDERSAMPLING_MASK;
			/* disable under-sampling parameter if already enabled */
			 *data = temp;
			/* Write data */
			 res = bmx160_write_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, data, 1);
		}
	}
	return res;
}

static int32_t set_GYRO_Conf(BMX160_Object_t *pObj){

	int32_t res;
	uint8_t data[2] = {0};

	res = check_GYRO_Conf(pObj, data);
	if(res == BMX160_OK){
		/* Write output data rate and bandwidth */
		res = bmx160_write_reg(&(pObj->Ctx), BMX160_GYRO_CONFIG_ADDR, &data[0], 1);
		if(res == BMX160_OK){
			pObj->prev_gyro_cfg.odr = pObj->gyro_cfg.odr;
			pObj->prev_gyro_cfg.bw = pObj->gyro_cfg.bw;

			/* write accel range */
			res = bmx160_write_reg(&(pObj->Ctx), BMX160_GYRO_RANGE_ADDR, &data[1], 1);
			if(res == BMX160_OK){
				pObj->prev_gyro_cfg.range = pObj->gyro_cfg.range;
			}
		}

	}

	return res;
}

static int32_t check_GYRO_Conf(BMX160_Object_t *pObj, uint8_t *data){

	int32_t res;
	res = bmx160_read_reg(&(pObj->Ctx), BMX160_GYRO_CONFIG_ADDR, data, 2);
		if(res == BMX160_OK){
			res = process_gyro_odr(pObj, &data[0]);
			if(res == BMX160_OK){
				res = process_gyro_bw(pObj, &data[0]);
				if(res == BMX160_OK){
					res = process_gyro_range(pObj, &data[1]);
				}
			}
		}
		return res;
}

static int32_t process_gyro_odr(BMX160_Object_t *pObj, uint8_t *data){

	int32_t res = BMX160_OK;
	uint8_t temp = 0;
	uint8_t odr = 0;

	if(pObj->gyro_cfg.odr <= BMX160_ACCEL_ODR_MAX){
		if(pObj->gyro_cfg.odr != pObj->prev_gyro_cfg.odr){
			odr = (uint8_t)pObj->gyro_cfg.odr;
			temp = *data & ~BMX160_GYRO_ODR_MASK;
			/* Adding output data rate */
			*data = temp | (odr & BMX160_GYRO_ODR_MASK);
		}
	}
	else{
		res = BMX160_E_OUT_OF_RANGE;
	}
	return res;
}

static int32_t process_gyro_bw(BMX160_Object_t *pObj, uint8_t *data){

	int32_t res = BMX160_OK;
	uint8_t temp = 0;
	uint8_t bw = 0;

	if(pObj->gyro_cfg.bw <= BMX160_GYRO_BW_MAX){
		if(pObj->gyro_cfg.bw != pObj->prev_gyro_cfg.bw){
			bw = (uint8_t)pObj->gyro_cfg.bw;
			temp = *data & ~BMX160_GYRO_BW_MASK;
			/* Adding bandwidth */
			*data = temp | ((bw << 4) & BMX160_GYRO_ODR_MASK);
		}
	}
	else{
		res = BMX160_E_OUT_OF_RANGE;
	}
		return res;

}

static int32_t process_gyro_range(BMX160_Object_t *pObj, uint8_t *data){

	int32_t res = BMX160_OK;
	uint8_t temp = 0;
	uint8_t range = 0;

	if(pObj->gyro_cfg.range <= BMX160_GYRO_RANGE_MAX){
		if(pObj->gyro_cfg.range != pObj->prev_gyro_cfg.range){
			range = (uint8_t)pObj->gyro_cfg.range;
			temp = *data & ~BMX160_GYRO_RANGE_MASK;
			/* Adding output data rate */
			*data = temp | (range & BMX160_GYRO_RANGE_MASK);
		}
	}
	else{
		res = BMX160_E_OUT_OF_RANGE;
	}
	return res;
}

static int32_t set_gyro_pwr(BMX160_Object_t *pObj){

	int32_t res = BMX160_OK;

	if((pObj->gyro_cfg.power == BMX160_GYRO_SUSPEND_MODE) || (pObj->gyro_cfg.power == BMX160_GYRO_NORMAL_MODE) || (pObj->gyro_cfg.power == BMX160_GYRO_FASTSTARTUP_MODE)){
		if(pObj->gyro_cfg.power != pObj->prev_gyro_cfg.power){
			/* Write gyro power */
			res = bmx160_write_reg(&(pObj->Ctx), BMX160_COMMAND_REG_ADDR, &pObj->gyro_cfg.power, 1);
			if(pObj->prev_gyro_cfg.power == BMX160_GYRO_SUSPEND_MODE){
				/* Delay of 80 ms - datasheet Table 24 */
				bmx160_delay(&(pObj->Ctx), BMX160_GYRO_DELAY_MS);
			}
			else if((pObj->prev_gyro_cfg.power == BMX160_GYRO_FASTSTARTUP_MODE) && (pObj->gyro_cfg.power == BMX160_GYRO_NORMAL_MODE)){
				/* This delay is required for transition from  fast-startup mode to normal mode - datasheet Table 3 */
				bmx160_delay(&(pObj->Ctx), 10); //Create macro
			}
			else{
				/* do nothing */
			}
			pObj->prev_gyro_cfg.power = pObj->gyro_cfg.power;
		}
	}
	else{
		res = BMX160_E_OUT_OF_RANGE;
	}
	return res;
}

static int32_t check_invalid_settings(BMX160_Object_t *pObj){

	int32_t res;
	uint8_t data = 0;

	/* read the error reg */
	res = bmx160_read_reg(&(pObj->Ctx), BMX160_ERROR_REG_ADDR, &data, 1);
	data = data >> 1;
	data = data & BMX160_ERR_REG_MASK;
	if(data == 1){
		res = BMX160_E_ACCEL_ODR_BW_INVALID;
	}
	else if(data == 2){
		res = BMX160_E_GYRO_ODR_BW_INVALID;
	}
	else if(data == 3){
		res = BMX160_E_LWP_PRE_FLTR_INT_INVALID;
	}
	else if(data == 7){
		res = BMX160_E_LWP_PRE_FLTR_INVALID;
	}
	return res;
}
/*!
 * @brief This API set the gyro configuration.
 *
 * @param[in] dev: pObj
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */



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
