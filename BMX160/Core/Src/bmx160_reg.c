#include "bmx160_reg.h"

/**
  * @defgroup    BMX160
  * @brief       This file provides a set of functions needed to drive the
  *              lsm6dsr enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    BMX160_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
  
 int32_t bmx160_read_reg(bmx160_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t bmx160_write_reg(bmx160_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}

void bmx160_delay (bmx160_ctx_t *ctx, uint32_t period)
{
	ctx->delay(ctx->handle, period);
}
