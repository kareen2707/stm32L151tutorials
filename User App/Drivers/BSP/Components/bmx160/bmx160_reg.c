/*
 * bmx160_reg.c
 *
 *  Created on: 29 mar. 2020
 *      Author: karen@b105.upm.es
 */

#include "bmx160_reg.h"

const uint8_t int_mask_lookup_table[13] = {
    BMX160_INT1_SLOPE_MASK, BMX160_INT1_SLOPE_MASK, BMX160_INT2_LOW_STEP_DETECT_MASK, BMX160_INT1_DOUBLE_TAP_MASK,
    BMX160_INT1_SINGLE_TAP_MASK, BMX160_INT1_ORIENT_MASK, BMX160_INT1_FLAT_MASK, BMX160_INT1_HIGH_G_MASK,
    BMX160_INT1_LOW_G_MASK, BMX160_INT1_NO_MOTION_MASK, BMX160_INT2_DATA_READY_MASK, BMX160_INT2_FIFO_FULL_MASK,
    BMX160_INT2_FIFO_WM_MASK
};

/**
  * @defgroup  HTS221_interfaces_functions
  * @brief     This section provide a set of functions used to read and write
  *            a generic register of the device.
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
  int32_t ret; //int8_t rslt = BMI160_OK;

  /* Variable to define temporary length */
  uint16_t temp_len = len + 1;
  /* Variable to define temporary buffer */
  uint8_t temp_buf[temp_len];
  /* Variable to define loop */
  uint16_t indx = 0;
  if(len == 0){
	  ret = BMX160_READ_WRITE_LENGHT_INVALID;
  }
  else{
	  ret = ctx->read_reg(ctx->handle, reg, temp_buf, temp_len);
	  if(ret == BMX160_OK){
		  /* Read the data from the position next to dummy byte */
		  while (indx < len){
			  data[indx] = temp_buf[indx+1];
		      indx++;
		  }
	  }
  }
  return ret;

}


/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param 	mdoe  it determines the writing mode depending on the sensor power mode.
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t bmx160_write_reg(bmx160_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len, uint8_t mode)
{
  int32_t ret;
  uint8_t count = 0;
  if(len == 0){
	  ret = BMX160_READ_WRITE_LENGHT_INVALID;
  }
  else{
	  if (mode){ /* Burst mode only allowed in NORMAL_POWER_MODE*/
		  ret = ctx->write_reg(ctx->handle, reg, data, len);
		  /*  BMX160 data sheet section 3.4 */
		  ctx->delay_ms(ctx->handle, BMX160_ONE_MS_DELAY);
	  }
	  else{
		  for(; count<len; count++){ // Karen: I think here there is an error in the original driver.
			  ret = ctx->write_reg(ctx->handle, reg, &data[count], 1);
			  reg++;
			  ctx->delay_ms(ctx->handle, BMX160_ONE_MS_DELAY);
		  }
	  }
  }

  return ret;
}

void bmx160_delay_ms(bmx160_ctx_t* ctx, uint32_t period){
	ctx->delay_ms(ctx->handle, period);
}

/*!
 * @brief This API configure the behavioural setting of interrupt pin.
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] dev       : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_int_out_ctrl(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
{
    int32_t rslt;
    uint8_t temp = 0;
    uint8_t data = 0;

    /* Configuration of output interrupt signals on pins INT1 and INT2 are
     * done in BMI160_INT_OUT_CTRL_ADDR register*/
    rslt = bmx160_read_reg(ctx, BMX160_INT_OUT_CTRL_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        /* updating the interrupt pin structure to local structure */
        const bmx160_int_pin_settg *intr_pin_sett = &(int_config->int_pin_settg);

        /* Configuring channel 1 */
        if (int_config->int_channel == BMX160_INT_CHANNEL_1)
        {
            /* Output enable */
            temp = data & ~BMX160_INT1_OUTPUT_EN_MASK;
            data = temp | ((intr_pin_sett->output_en << 3) & BMX160_INT1_OUTPUT_EN_MASK);

            /* Output mode */
            temp = data & ~BMX160_INT1_OUTPUT_MODE_MASK;
            data = temp | ((intr_pin_sett->output_mode << 2) & BMX160_INT1_OUTPUT_MODE_MASK);

            /* Output type */
            temp = data & ~BMX160_INT1_OUTPUT_TYPE_MASK;
            data = temp | ((intr_pin_sett->output_type << 1) & BMX160_INT1_OUTPUT_TYPE_MASK);

            /* edge control */
            temp = data & ~BMX160_INT1_EDGE_CTRL_MASK;
            data = temp | ((intr_pin_sett->edge_ctrl) & BMX160_INT1_EDGE_CTRL_MASK);
        }
        else
        {
            /* Configuring channel 2 */
            /* Output enable */
            temp = data & ~BMX160_INT2_OUTPUT_EN_MASK;
            data = temp | ((intr_pin_sett->output_en << 7) & BMX160_INT2_OUTPUT_EN_MASK);

            /* Output mode */
            temp = data & ~BMX160_INT2_OUTPUT_MODE_MASK;
            data = temp | ((intr_pin_sett->output_mode << 6) & BMX160_INT2_OUTPUT_MODE_MASK);

            /* Output type */
            temp = data & ~BMX160_INT2_OUTPUT_TYPE_MASK;
            data = temp | ((intr_pin_sett->output_type << 5) & BMX160_INT2_OUTPUT_TYPE_MASK);

            /* edge control */
            temp = data & ~BMX160_INT2_EDGE_CTRL_MASK;
            data = temp | ((intr_pin_sett->edge_ctrl << 4) & BMX160_INT2_EDGE_CTRL_MASK);
        }
        rslt = bmx160_write_reg(ctx, BMX160_INT_OUT_CTRL_ADDR, &data, 1, ctx->mode); //Normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the mode(input enable, latch or non-latch) of interrupt pin.
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] dev       : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_int_latch(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
{
    int32_t rslt;
    uint8_t temp = 0;
    uint8_t data = 0;

    /* Configuration of latch on pins INT1 and INT2 are done in
     * BMI160_INT_LATCH_ADDR register*/
    rslt = bmx160_read_reg(ctx, BMX160_INT_LATCH_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        /* updating the interrupt pin structure to local structure */
        const bmx160_int_pin_settg *intr_pin_sett = &(int_config->int_pin_settg);
        if (int_config->int_channel == BMX160_INT_CHANNEL_1)
        {
            /* Configuring channel 1 */
            /* Input enable */
            temp = data & ~BMX160_INT1_INPUT_EN_MASK;
            data = temp | ((intr_pin_sett->input_en << 4) & BMX160_INT1_INPUT_EN_MASK);
        }
        else
        {
            /* Configuring channel 2 */
            /* Input enable */
            temp = data & ~BMX160_INT2_INPUT_EN_MASK;
            data = temp | ((intr_pin_sett->input_en << 5) & BMX160_INT2_INPUT_EN_MASK);
        }

        /* In case of latch interrupt,update the latch duration */

        /* Latching holds the interrupt for the amount of latch
         * duration time */
        temp = data & ~BMX160_INT_LATCH_MASK;
        data = temp | (intr_pin_sett->latch_dur & BMX160_INT_LATCH_MASK);

        /* OUT_CTRL_INT and LATCH_INT address lie consecutively,
         * hence writing data to respective registers at one go */
        rslt = bmx160_write_reg(ctx, BMX160_INT_LATCH_ADDR, &data, 1, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This API configures the pins to fire the
 * interrupt signal when it occurs
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] ctx        : Structure instance of bmx160 ctx.
 * @retval zero -> Success / -ve value -> Error.
 */
int32_t set_intr_pin_config(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
{
    int8_t rslt;

    /* configure the behavioral settings of interrupt pin */
    rslt = config_int_out_ctrl(ctx, int_config);
    if (rslt == BMX160_OK)
    {
        rslt = config_int_latch(ctx, int_config);
    }

    return rslt;
}

/*!
 * @brief This API configure the source of data(filter & pre-filter) for any-motion interrupt.
 *
 * @param[in] any_motion_int_cfg  : Structure instance of bmi160_acc_any_mot_int_cfg.
 * @param[in] ctx         		  : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_any_motion_src(bmx160_ctx_t* ctx, const bmx160_acc_any_mot_int_cfg *any_motion_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Configure Int data 1 register to add source of interrupt */
    rslt = bmx160_read_reg(ctx, BMX160_INT_DATA_1_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_MOTION_SRC_INT_MASK;
        data = temp | ((any_motion_int_cfg->anymotion_data_src << 7) & BMX160_MOTION_SRC_INT_MASK);

        /* Write data to DATA 1 address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_DATA_1_ADDR, &data, 1, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for both accel and gyro. This interrupt occurs when new accel and gyro data come.
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] ctx         : Structure instance of bmi160_dev.
 * @retval zero -> Success / -ve value -> Error.
 */
int32_t set_accel_gyro_data_ready_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
 {
     int32_t rslt;

     if (int_config == NULL)
     {
         rslt = BMX160_E_NULL_PTR;
     }
     else
     {
         rslt = enable_data_ready_int(ctx);
         if (rslt == BMX160_OK)
         {
             /* Configure Interrupt pins */
             rslt = set_intr_pin_config(ctx, int_config);
             if (rslt == BMX160_OK)
             {
                 rslt = map_hardware_interrupt(ctx, int_config);
             }
         }
     }

     return rslt;
 }
/*!
 * @brief This API configure necessary setting of any-motion interrupt.
 * @param[in] int_config       : Structure instance of bmi160_int_settg.
 * @param[in] any_motion_int_cfg   : Structure instance of bmi160_acc_any_mot_int_cfg.
 * @param[in] dev          : Structure instance of bmi160_dev.
 *
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_any_motion_int_settg(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_any_mot_int_cfg *any_motion_int_cfg){

	int32_t rslt;
	/* Configure Interrupt pins */
	rslt = set_intr_pin_config(ctx, int_config);
	if (rslt == BMX160_OK)
	{
		rslt = disable_sig_motion_int(ctx);
	    if (rslt == BMX160_OK)
	    {
	        rslt = map_feature_interrupt(ctx, int_config);
	        if (rslt == BMX160_OK)
	        {
	            rslt = config_any_motion_src(ctx, any_motion_int_cfg);
	            if (rslt == BMX160_OK)
	            {
	                rslt = config_any_dur_threshold(ctx, any_motion_int_cfg);
	            }
	        }
	    }
	 }

	    return rslt;
}

/*!
 * @brief This API disable the sig-motion interrupt.
 * @param[in] ctx   : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t disable_sig_motion_int(bmx160_ctx_t* ctx)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Disabling Significant motion interrupt if enabled */
    rslt = bmx160_read_reg(ctx, BMX160_INT_MOTION_3_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = (data & BMX160_SIG_MOTION_SEL_MASK);
        if (temp)
        {
            temp = data & ~BMX160_SIG_MOTION_SEL_MASK;
            data = temp;

            /* Write data to register */
            rslt = bmx160_write_reg(ctx, BMX160_INT_MOTION_3_ADDR, &data, 1, ctx->mode); //normal mode
        }
    }

    return rslt;
}

/*!
 *  @brief This API is used to map/unmap the Dataready(Accel & Gyro), FIFO full and FIFO watermark interrupt
 *
 *  @param[in] int_config     : Structure instance of bmi160_int_settg which stores the interrupt type and interrupt channel
 *              				configurations to map/unmap the interrupt pins
 *  @param[in] dev            : Structure instance of bmi160_dev.
 *  @retval zero -> Success  / -ve value -> Error
 */

int32_t map_feature_interrupt(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
{
    int32_t rslt;
    uint8_t data[3] = { 0, 0, 0 };
    uint8_t temp[3] = { 0, 0, 0 };

    rslt = bmx160_read_reg(ctx, BMX160_INT_MAP_0_ADDR, data, 3);
    if (rslt == BMX160_OK)
    {
        temp[0] = data[0] & ~int_mask_lookup_table[int_config->int_type];
        temp[2] = data[2] & ~int_mask_lookup_table[int_config->int_type];
        switch (int_config->int_channel)
        {
            case BMX160_INT_CHANNEL_NONE:
                data[0] = temp[0];
                data[2] = temp[2];
                break;
            case BMX160_INT_CHANNEL_1:
                data[0] = temp[0] | int_mask_lookup_table[int_config->int_type];
                data[2] = temp[2];
                break;
            case BMX160_INT_CHANNEL_2:
                data[2] = temp[2] | int_mask_lookup_table[int_config->int_type];
                data[0] = temp[0];
                break;
            case BMX160_INT_CHANNEL_BOTH:
                data[0] = temp[0] | int_mask_lookup_table[int_config->int_type];
                data[2] = temp[2] | int_mask_lookup_table[int_config->int_type];
                break;
            default:
                rslt = BMX160_E_OUT_OF_RANGE;
        }
        if (rslt == BMX160_OK)
        {
            rslt = bmx160_write_reg(ctx, BMX160_INT_MAP_0_ADDR, data, 3, ctx->mode);
        }
    }

    return rslt;
}

/*!
 *  @brief This API is used to map/unmap the Any/Sig motion, Step det/Low-g, Double tap, Single tap, Orientation, Flat, High-G, Nomotion interrupt pins.
 *
 *  @param[in] int_config     : Structure instance of bmi160_int_settg which stores the interrupt type and interrupt channel
 *              				configurations to map/unmap the interrupt pins
 *  @param[in] dev            : Structure instance of bmi160_dev.
 *  @retval zero -> Success  / -ve value -> Error
 */
int32_t map_hardware_interrupt(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    rslt = bmx160_read_reg(ctx, BMX160_INT_MAP_1_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~int_mask_lookup_table[int_config->int_type];
        temp = temp & ~((uint8_t)(int_mask_lookup_table[int_config->int_type] << 4));
        switch (int_config->int_channel)
        {
            case BMX160_INT_CHANNEL_NONE:
                data = temp;
                break;
            case BMX160_INT_CHANNEL_1:
                data = temp | (uint8_t)((int_mask_lookup_table[int_config->int_type]) << 4);
                break;
            case BMX160_INT_CHANNEL_2:
                data = temp | int_mask_lookup_table[int_config->int_type];
                break;
            case BMX160_INT_CHANNEL_BOTH:
                data = temp | int_mask_lookup_table[int_config->int_type];
                data = data | (uint8_t)((int_mask_lookup_table[int_config->int_type]) << 4);
                break;
            default:
                rslt = BMX160_E_OUT_OF_RANGE;
        }
        if (rslt == BMX160_OK)
        {
            rslt = bmx160_write_reg(ctx, BMX160_INT_MAP_1_ADDR, &data, 1, ctx->mode);
        }
    }

    return rslt;
}

/*!
 * @brief This API configure the duration and threshold of any-motion interrupt.
 *
 * @param[in] any_motion_int_cfg  : Structure instance of  bmi160_acc_any_mot_int_cfg.
 * @param[in] ctx        : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_any_dur_threshold(bmx160_ctx_t* ctx, const bmx160_acc_any_mot_int_cfg *any_motion_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;
    uint8_t data_array[2] = { 0 };
    uint8_t dur;

    /* Configure Int Motion 0 register */
    rslt = bmx160_read_reg(ctx, BMX160_INT_MOTION_0_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        /* slope duration */
        dur = (uint8_t)any_motion_int_cfg->anymotion_dur;
        temp = data & ~BMX160_SLOPE_INT_DUR_MASK;
        data = temp | (dur & BMX160_MOTION_SRC_INT_MASK);
        data_array[0] = data;

        /* add slope threshold */
        data_array[1] = any_motion_int_cfg->anymotion_thr;

        /* INT MOTION 0 and INT MOTION 1 address lie consecutively,
         * hence writing data to respective registers at one go */

        /* Writing to Int_motion 0 and
         * Int_motion 1 Address simultaneously */
        rslt = bmx160_write_reg(ctx, BMX160_INT_MOTION_0_ADDR, data_array, 2, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This API sets tap interrupts.Interrupt is fired when tap movements happen.
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] ctx         : Structure instance of bmi160_dev.
 * @retval zero -> Success / -ve value -> Error.
 */

int32_t set_accel_tap_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config)
{
    int32_t rslt;

    if (int_config == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* updating the interrupt structure to local structure */
        bmx160_acc_tap_int_cfg *tap_int_cfg = &(int_config->int_type_cfg.acc_tap_int);
        rslt = enable_tap_int(ctx, int_config, tap_int_cfg);
        if (rslt == BMX160_OK)
        {
            /* Configure Interrupt pins */
            rslt = set_intr_pin_config(ctx, int_config);
            if (rslt == BMX160_OK)
            {
                rslt = config_tap_int_settg(ctx, int_config, tap_int_cfg);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API enables the single/double tap interrupt.
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] dev       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_tap_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_tap_int_cfg *tap_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Enable single tap or double tap interrupt in Int Enable 0 register */
    rslt = bmx160_read_reg(ctx, BMX160_INT_ENABLE_0_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        if (int_config->int_type == BMX160_ACC_SINGLE_TAP_INT)
        {
            temp = data & ~BMX160_SINGLE_TAP_INT_EN_MASK;
            data = temp | ((tap_int_cfg->tap_en << 5) & BMX160_SINGLE_TAP_INT_EN_MASK);
        }
        else
        {
            temp = data & ~BMX160_DOUBLE_TAP_INT_EN_MASK;
            data = temp | ((tap_int_cfg->tap_en << 4) & BMX160_DOUBLE_TAP_INT_EN_MASK);
        }

        /* Write to Enable 0 Address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_0_ADDR, &data, 1, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the interrupt PIN setting for tap interrupt.
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] tap_int_cfg   : Structure instance of bmi160_acc_tap_int_cfg.
 * @param[in] dev       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_tap_int_settg(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_tap_int_cfg *tap_int_cfg)
{
    int32_t rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(ctx, int_config);
    if (rslt == BMX160_OK)
    {
        rslt = map_feature_interrupt(ctx, int_config);
        if (rslt == BMX160_OK)
        {
            rslt = config_tap_data_src(ctx, tap_int_cfg);
            if (rslt == BMX160_OK)
            {
                rslt = config_tap_param(ctx, int_config, tap_int_cfg);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API configure the source of data(filter & pre-filter) for tap interrupt.
 * @param[in] tap_int_cfg   : Structure instance of bmi160_acc_tap_int_cfg.
 * @param[in] dev       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_tap_data_src(bmx160_ctx_t* ctx, const bmx160_acc_tap_int_cfg *tap_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Configure Int data 0 register to add source of interrupt */
    rslt = bmx160_read_reg(ctx, BMX160_INT_DATA_0_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_TAP_SRC_INT_MASK;
        data = temp | ((tap_int_cfg->tap_data_src << 3) & BMX160_TAP_SRC_INT_MASK);

        /* Write data to Data 0 address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_DATA_0_ADDR, &data, 1, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the  parameters of tap interrupt. Threshold, quite, shock, and duration.
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] tap_int_cfg   : Structure instance of bmi160_acc_tap_int_cfg.
 * @param[in] ctx       : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_tap_param(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_tap_int_cfg *tap_int_cfg)
{
    int32_t rslt;
    uint8_t temp = 0;
    uint8_t data = 0;
    uint8_t data_array[2] = { 0 };
    uint8_t count = 0;
    uint8_t dur, shock, quiet, thres;

    /* Configure tap 0 register for tap shock,tap quiet duration
     * in case of single tap interrupt */
    rslt = bmx160_read_reg(ctx, BMX160_INT_TAP_0_ADDR, data_array, 2);
    if (rslt == BMX160_OK)
    {
        data = data_array[count];
        if (int_config->int_type == BMX160_ACC_DOUBLE_TAP_INT)
        {
            dur = (uint8_t)tap_int_cfg->tap_dur;
            temp = (data & ~BMX160_TAP_DUR_MASK);

            /* Add tap duration data in case of
             * double tap interrupt */
            data = temp | (dur & BMX160_TAP_DUR_MASK);
        }
        shock = (uint8_t)tap_int_cfg->tap_shock;
        temp = data & ~BMX160_TAP_SHOCK_DUR_MASK;
        data = temp | ((shock << 6) & BMX160_TAP_SHOCK_DUR_MASK);
        quiet = (uint8_t)tap_int_cfg->tap_quiet;
        temp = data & ~BMX160_TAP_QUIET_DUR_MASK;
        data = temp | ((quiet << 7) & BMX160_TAP_QUIET_DUR_MASK);
        data_array[count++] = data;
        data = data_array[count];
        thres = (uint8_t)tap_int_cfg->tap_thr;
        temp = data & ~BMX160_TAP_THRES_MASK;
        data = temp | (thres & BMX160_TAP_THRES_MASK);
        data_array[count++] = data;

        /* TAP 0 and TAP 1 address lie consecutively,
         * hence writing data to respective registers at one go */

        /* Writing to Tap 0 and Tap 1 Address simultaneously */
        rslt = bmx160_write_reg(ctx, BMX160_INT_TAP_0_ADDR, data_array, count, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API enable the data ready interrupt.
 *
 * @param[in] dev       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_data_ready_int(bmx160_ctx_t* ctx)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Enable data ready interrupt in Int Enable 1 register */
    rslt = bmx160_read_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_DATA_RDY_INT_EN_MASK;
        data = temp | ((1 << 4) & BMX160_DATA_RDY_INT_EN_MASK);

        /* Writing data to INT ENABLE 1 Address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This API configure the interrupt PIN setting for significant motion interrupt.
 *
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] sig_mot_int_cfg   : Structure instance of bmi160_acc_sig_mot_int_cfg.
 * @param[in] ctx       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_sig_motion_int_settg(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_sig_mot_int_cfg *sig_mot_int_cfg)
{
    int32_t rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(ctx, int_config);
    if (rslt == BMX160_OK)
    {
        rslt = map_feature_interrupt(ctx, int_config);
        if (rslt == BMX160_OK)
        {
            rslt = config_sig_motion_data_src(ctx, sig_mot_int_cfg);
            if (rslt == BMX160_OK)
            {
                rslt = config_sig_dur_threshold(ctx, sig_mot_int_cfg);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API configure the source of data(filter & pre-filter)
 * for sig motion interrupt.
 */
int32_t config_sig_motion_data_src(bmx160_ctx_t* ctx, const bmx160_acc_sig_mot_int_cfg *sig_mot_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Configure Int data 1 register to add source of interrupt */
    rslt = bmx160_read_reg(ctx, BMX160_INT_DATA_1_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_MOTION_SRC_INT_MASK;
        data = temp | ((sig_mot_int_cfg->sig_data_src << 7) & BMX160_MOTION_SRC_INT_MASK);

        /* Write data to DATA 1 address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_DATA_1_ADDR, &data, 1, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the threshold, skip and proof time of sig motion interrupt.
 *
 * @param[in] sig_mot_int_cfg   : Structure instance of bmi160_acc_sig_mot_int_cfg.
 * @param[in] ctx       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_sig_dur_threshold(bmx160_ctx_t* ctx, const bmx160_acc_sig_mot_int_cfg *sig_mot_int_cfg)
{
    int32_t rslt;
    uint8_t data;
    uint8_t temp = 0;

    /* Configuring INT_MOTION registers */

    /* Write significant motion threshold.
     * This threshold is same as any motion threshold */
    data = sig_mot_int_cfg->sig_mot_thres;

    /* Write data to INT_MOTION 1 address */
    rslt = bmx160_write_reg(ctx, BMX160_INT_MOTION_1_ADDR, &data, 1, ctx->mode);
    if (rslt == BMX160_OK)
    {
        rslt = bmx160_read_reg(ctx, BMX160_INT_MOTION_3_ADDR, &data, 1);
        if (rslt == BMX160_OK)
        {
            temp = data & ~BMX160_SIG_MOTION_SKIP_MASK;

            /* adding skip time of sig_motion interrupt*/
            data = temp | ((sig_mot_int_cfg->sig_mot_skip << 2) & BMX160_SIG_MOTION_SKIP_MASK);
            temp = data & ~BMX160_SIG_MOTION_PROOF_MASK;

            /* adding proof time of sig_motion interrupt */
            data = temp | ((sig_mot_int_cfg->sig_mot_proof << 4) & BMX160_SIG_MOTION_PROOF_MASK);

            /* configure the int_sig_mot_sel bit to select
             * significant motion interrupt */
            temp = data & ~BMX160_SIG_MOTION_SEL_MASK;
            data = temp | ((sig_mot_int_cfg->sig_en << 1) & BMX160_SIG_MOTION_SEL_MASK);
            rslt = bmx160_write_reg(ctx, BMX160_INT_MOTION_3_ADDR, &data, 1, ctx->mode); //normal mode
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the no motion/slow motion interrupt of the sensor. Slow motion is similar to any motion interrupt.No motion interrupt
 * occurs when slope bet. two accel values falls below preset threshold for preset duration.
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] dev         : Structure instance of bmi160_dev.
 * @retval zero -> Success / -ve value -> Error.
 */
int32_t set_accel_no_motion_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config)
{
    int32_t rslt;

    if (int_config == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* updating the interrupt structure to local structure */
        bmx160_acc_no_motion_int_cfg *no_mot_int_cfg = &(int_config->int_type_cfg.acc_no_motion_int);
        rslt = enable_no_motion_int(ctx, no_mot_int_cfg);
        if (rslt == BMX160_OK)
        {
            /* Configure the INT PIN settings*/
            rslt = config_no_motion_int_settg(ctx, int_config, no_mot_int_cfg);
        }
    }

    return rslt;
}

/*!
 * @brief This API enables the no motion/slow motion interrupt.
 *
 * @param[in] no_mot_int_cfg    : Structure instance of bmi160_acc_no_motion_int_cfg.
 * @param[in] ctx      : Structure instance of bmi160_dev.

 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_no_motion_int(bmx160_ctx_t* ctx, const bmx160_acc_no_motion_int_cfg *no_mot_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Enable no motion x, no motion y, no motion z
     * in Int Enable 2 register */
    rslt = bmx160_read_reg(ctx, BMX160_INT_ENABLE_2_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        if (no_mot_int_cfg->no_motion_x == 1)
        {
            temp = data & ~BMX160_NO_MOTION_X_INT_EN_MASK;

            /* Adding No_motion x axis */
            data = temp | (1 & BMX160_NO_MOTION_X_INT_EN_MASK);
        }
        if (no_mot_int_cfg->no_motion_y == 1)
        {
            temp = data & ~BMX160_NO_MOTION_Y_INT_EN_MASK;

            /* Adding No_motion x axis */
            data = temp | ((1 << 1) & BMX160_NO_MOTION_Y_INT_EN_MASK);
        }
        if (no_mot_int_cfg->no_motion_z == 1)
        {
            temp = data & ~BMX160_NO_MOTION_Z_INT_EN_MASK;

            /* Adding No_motion x axis */
            data = temp | ((1 << 2) & BMX160_NO_MOTION_Z_INT_EN_MASK);
        }

        /* write data to Int Enable 2 register */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_2_ADDR, &data, 1, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the interrupt PIN setting for no motion/slow motion interrupt.
 *
 * @param[in] int_config    : structure instance of bmi160_int_settg.
 * @param[in] no_mot_int_cfg    : Structure instance of  bmi160_acc_no_motion_int_cfg.
 * @param[in] ctx       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_no_motion_int_settg(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_no_motion_int_cfg *no_mot_int_cfg)
{
    int32_t rslt;

    /* Configure Interrupt pins */
    rslt = set_intr_pin_config(ctx, int_config);
    if (rslt == BMX160_OK)
    {
        rslt = map_feature_interrupt(ctx, int_config);
        if (rslt == BMX160_OK)
        {
            rslt = config_no_motion_data_src(ctx, no_mot_int_cfg);
            if (rslt == BMX160_OK)
            {
                rslt = config_no_motion_dur_thr(ctx, no_mot_int_cfg);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API configure the source of interrupt for no motion.
 *
 * @param[in] no_mot_int_cfg    : Structure instance of bmi160_acc_no_motion_int_cfg.
 * @param[in] dev       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_no_motion_data_src(bmx160_ctx_t* ctx, const bmx160_acc_no_motion_int_cfg *no_mot_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Configure Int data 1 register to add source of interrupt */
    rslt = bmx160_read_reg(ctx, BMX160_INT_DATA_1_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_MOTION_SRC_INT_MASK;
        data = temp | ((no_mot_int_cfg->no_motion_src << 7) & BMX160_MOTION_SRC_INT_MASK);

        /* Write data to DATA 1 address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_DATA_1_ADDR, &data, 1, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the duration and threshold of no motion/slow motion interrupt along with selection of no/slow motion.
 *
 * @param[in] no_mot_int_cfg    : Structure instance of bmi160_acc_no_motion_int_cfg.
 * @param[in] ctx       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_no_motion_dur_thr(bmx160_ctx_t* ctx, const bmx160_acc_no_motion_int_cfg *no_mot_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;
    uint8_t temp_1 = 0;
    uint8_t reg_addr;
    uint8_t data_array[2] = { 0 };

    /* Configuring INT_MOTION register */
    reg_addr = BMX160_INT_MOTION_0_ADDR;
    rslt = bmx160_read_reg(ctx, reg_addr, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_NO_MOTION_INT_DUR_MASK;

        /* Adding no_motion duration */
        data = temp | ((no_mot_int_cfg->no_motion_dur << 2) & BMX160_NO_MOTION_INT_DUR_MASK);

        /* Write data to NO_MOTION 0 address */
        rslt = bmx160_write_reg(ctx, reg_addr, &data, 1, ctx->mode);
        if (rslt == BMX160_OK)
        {
            reg_addr = BMX160_INT_MOTION_3_ADDR;
            rslt = bmx160_read_reg(ctx, reg_addr, &data, 1);
            if (rslt == BMX160_OK)
            {
                temp = data & ~BMX160_NO_MOTION_SEL_BIT_MASK;

                /* Adding no_motion_sel bit */
                temp_1 = (no_mot_int_cfg->no_motion_sel & BMX160_NO_MOTION_SEL_BIT_MASK);
                data = (temp | temp_1);
                data_array[1] = data;

                /* Adding no motion threshold */
                data_array[0] = no_mot_int_cfg->no_motion_thres;
                reg_addr = BMX160_INT_MOTION_2_ADDR;

                /* writing data to INT_MOTION 2 and INT_MOTION 3
                 * address simultaneously */
                rslt = bmx160_write_reg(ctx, reg_addr, data_array, 2, ctx->mode); //normal mode
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the step detection interrupt.This interrupt occurs when the single step causes accel values to go above
 * preset threshold.
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] ctx         : Structure instance of bmi160_dev.
 * @retval zero -> Success / -ve value -> Error.
 */
int32_t set_accel_step_detect_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config)
{
    int32_t rslt;

    /* Null-pointer check */
    if (int_config == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* updating the interrupt structure to local structure */
        bmx160_acc_step_detect_int_cfg *step_detect_int_cfg = &(int_config->int_type_cfg.acc_step_detect_int);
        rslt = enable_step_detect_int(ctx, step_detect_int_cfg);
        if (rslt == BMX160_OK)
        {
            /* Configure Interrupt pins */
            rslt = set_intr_pin_config(ctx, int_config);
            if (rslt == BMX160_OK)
            {
                rslt = map_feature_interrupt(ctx, int_config);
                if (rslt == BMX160_OK)
                {
                    rslt = config_step_detect(ctx, step_detect_int_cfg);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API enables the step detector interrupt.
 * @param[in] step_detect_int_cfg   : Structure instance of bmi160_acc_step_detect_int_cfg.
 * @param[in] dev           : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_step_detect_int(bmx160_ctx_t* ctx, const bmx160_acc_step_detect_int_cfg *step_detect_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Enable data ready interrupt in Int Enable 2 register */
    rslt = bmx160_read_reg(ctx, BMX160_INT_ENABLE_2_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_STEP_DETECT_INT_EN_MASK;
        data = temp | ((step_detect_int_cfg->step_detector_en << 3) & BMX160_STEP_DETECT_INT_EN_MASK);

        /* Writing data to INT ENABLE 2 Address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_2_ADDR, &data, 1, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This API configure the step detector parameter.
 * @param[in] step_detect_int_cfg   : Structure instance of bmi160_acc_step_detect_int_cfg.
 * @param[in] dev           : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_step_detect(bmx160_ctx_t* ctx, const bmx160_acc_step_detect_int_cfg *step_detect_int_cfg)
{
    int32_t rslt;
    uint8_t temp = 0;
    uint8_t data_array[2] = { 0 };

    if (step_detect_int_cfg->step_detector_mode == BMX160_STEP_DETECT_NORMAL)
    {
        /* Normal mode setting */
        data_array[0] = 0x15;
        data_array[1] = 0x03;
    }
    else if (step_detect_int_cfg->step_detector_mode == BMX160_STEP_DETECT_SENSITIVE)
    {
        /* Sensitive mode setting */
        data_array[0] = 0x2D;
        data_array[1] = 0x00;
    }
    else if (step_detect_int_cfg->step_detector_mode == BMX160_STEP_DETECT_ROBUST)
    {
        /* Robust mode setting */
        data_array[0] = 0x1D;
        data_array[1] = 0x07;
    }
    else if (step_detect_int_cfg->step_detector_mode == BMX160_STEP_DETECT_USER_DEFINE)
    {
        /* Non recommended User defined setting */
        /* Configuring STEP_CONFIG register */
        rslt = bmx160_read_reg(ctx, BMX160_INT_STEP_CONFIG_0_ADDR, &data_array[0], 2);
        if (rslt == BMX160_OK)
        {
            temp = data_array[0] & ~BMX160_STEP_DETECT_MIN_THRES_MASK;

            /* Adding min_threshold */
            data_array[0] = temp | ((step_detect_int_cfg->min_threshold << 3) & BMX160_STEP_DETECT_MIN_THRES_MASK);
            temp = data_array[0] & ~BMX160_STEP_DETECT_STEPTIME_MIN_MASK;

            /* Adding steptime_min */
            data_array[0] = temp | ((step_detect_int_cfg->steptime_min) & BMX160_STEP_DETECT_STEPTIME_MIN_MASK);
            temp = data_array[1] & ~BMX160_STEP_MIN_BUF_MASK;

            /* Adding steptime_min */
            data_array[1] = temp | ((step_detect_int_cfg->step_min_buf) & BMX160_STEP_MIN_BUF_MASK);
        }
    }

    /* Write data to STEP_CONFIG register */
    rslt = bmx160_write_reg(ctx, BMX160_INT_STEP_CONFIG_0_ADDR, data_array, 2, ctx->mode);

    return rslt;
}

/*!
 * @brief This API sets the orientation interrupt of the sensor.This interrupt occurs when there is orientation change in the sensor
 * with respect to gravitational field vector g.
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] ctx         : Structure instance of bmi160_dev.
 * @retval zero -> Success / -ve value -> Error.
 */
int32_t set_accel_orientation_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config)
{
    int32_t rslt;

    if (int_config == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* updating the interrupt structure to local structure */
        bmx160_acc_orient_int_cfg *orient_int_cfg = &(int_config->int_type_cfg.acc_orient_int);
        rslt = enable_orient_int(ctx, orient_int_cfg);
        if (rslt == BMX160_OK)
        {
            /* Configure Interrupt pins */
            rslt = set_intr_pin_config(ctx, int_config);
            if (rslt == BMX160_OK)
            {
                /* map INT pin to orient interrupt */
                rslt = map_feature_interrupt(ctx, int_config);
                if (rslt == BMX160_OK)
                {
                    /* configure the
                     * orientation setting*/
                    rslt = config_orient_int_settg(ctx, orient_int_cfg);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API enables the orient interrupt.
 * @param[in] orient_int_cfg : Structure instance of bmi160_acc_orient_int_cfg.
 * @param[in] dev        : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_orient_int(bmx160_ctx_t* ctx, const bmx160_acc_orient_int_cfg *orient_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Enable data ready interrupt in Int Enable 0 register */
    rslt = bmx160_read_reg(ctx, BMX160_INT_ENABLE_0_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_ORIENT_INT_EN_MASK;
        data = temp | ((orient_int_cfg->orient_en << 6) & BMX160_ORIENT_INT_EN_MASK);

        /* write data to Int Enable 0 register */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_0_ADDR, &data, 1, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the necessary setting of orientation interrupt.
 * @param[in] orient_int_cfg : Structure instance of bmi160_acc_orient_int_cfg.
 * @param[in] dev        : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_orient_int_settg(bmx160_ctx_t* ctx, const bmx160_acc_orient_int_cfg *orient_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;
    uint8_t data_array[2] = { 0, 0 };

    /* Configuring INT_ORIENT registers */
    rslt = bmx160_read_reg(ctx, BMX160_INT_ORIENT_0_ADDR, data_array, 2);
    if (rslt == BMX160_OK)
    {
        data = data_array[0];
        temp = data & ~BMX160_ORIENT_MODE_MASK;

        /* Adding Orientation mode */
        data = temp | ((orient_int_cfg->orient_mode) & BMX160_ORIENT_MODE_MASK);
        temp = data & ~BMX160_ORIENT_BLOCK_MASK;

        /* Adding Orientation blocking */
        data = temp | ((orient_int_cfg->orient_blocking << 2) & BMX160_ORIENT_BLOCK_MASK);
        temp = data & ~BMX160_ORIENT_HYST_MASK;

        /* Adding Orientation hysteresis */
        data = temp | ((orient_int_cfg->orient_hyst << 4) & BMX160_ORIENT_HYST_MASK);
        data_array[0] = data;
        data = data_array[1];
        temp = data & ~BMX160_ORIENT_THETA_MASK;

        /* Adding Orientation threshold */
        data = temp | ((orient_int_cfg->orient_theta) & BMX160_ORIENT_THETA_MASK);
        temp = data & ~BMX160_ORIENT_UD_ENABLE;

        /* Adding Orient_ud_en */
        data = temp | ((orient_int_cfg->orient_ud_en << 6) & BMX160_ORIENT_UD_ENABLE);
        temp = data & ~BMX160_AXES_EN_MASK;

        /* Adding axes_en */
        data = temp | ((orient_int_cfg->axes_ex << 7) & BMX160_AXES_EN_MASK);
        data_array[1] = data;

        /* Writing data to INT_ORIENT 0 and INT_ORIENT 1
         * registers simultaneously */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ORIENT_0_ADDR, data_array, 2, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API sets the flat interrupt of the sensor.This interrupt occurs in case of flat orientation
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] ctx         : Structure instance of bmi160_dev.
 * @retval zero -> Success / -ve value -> Error.
 */
int32_t set_accel_flat_detect_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config)
{
    int32_t rslt;

    if (int_config == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* updating the interrupt structure to local structure */
        bmx160_acc_flat_detect_int_cfg *flat_detect_int = &(int_config->int_type_cfg.acc_flat_int);

        /* enable the flat interrupt */
        rslt = enable_flat_int(ctx, flat_detect_int);
        if (rslt == BMX160_OK)
        {
            /* Configure Interrupt pins */
            rslt = set_intr_pin_config(ctx, int_config);
            if (rslt == BMX160_OK)
            {
                /* map INT pin to flat interrupt */
                rslt = map_feature_interrupt(ctx, int_config);
                if (rslt == BMX160_OK)
                {
                    /* configure the flat setting*/
                    rslt = config_flat_int_settg(ctx, flat_detect_int);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API enables the flat interrupt.
 *
 * @param[in] flat_int  : Structure instance of bmi160_acc_flat_detect_int_cfg.
 * @param[in] dev       : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_flat_int(bmx160_ctx_t* ctx, const bmx160_acc_flat_detect_int_cfg *flat_int)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Enable flat interrupt in Int Enable 0 register */
    rslt = bmx160_read_reg(ctx, BMX160_INT_ENABLE_0_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_FLAT_INT_EN_MASK;
        data = temp | ((flat_int->flat_en << 7) & BMX160_FLAT_INT_EN_MASK);

        /* write data to Int Enable 0 register */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_0_ADDR, &data, 1, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the necessary setting of flat interrupt.
 * @param[in] flat_int  : Structure instance of bmi160_acc_flat_detect_int_cfg.
 * @param[in] dev   : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_flat_int_settg(bmx160_ctx_t* ctx, const bmx160_acc_flat_detect_int_cfg *flat_int)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;
    uint8_t data_array[2] = { 0, 0 };

    /* Configuring INT_FLAT register */
    rslt = bmx160_read_reg(ctx, BMX160_INT_FLAT_0_ADDR, data_array, 2);
    if (rslt == BMX160_OK)
    {
        data = data_array[0];
        temp = data & ~BMX160_FLAT_THRES_MASK;

        /* Adding flat theta */
        data = temp | ((flat_int->flat_theta) & BMX160_FLAT_THRES_MASK);
        data_array[0] = data;
        data = data_array[1];
        temp = data & ~BMX160_FLAT_HOLD_TIME_MASK;

        /* Adding flat hold time */
        data = temp | ((flat_int->flat_hold_time << 4) & BMX160_FLAT_HOLD_TIME_MASK);
        temp = data & ~BMX160_FLAT_HYST_MASK;

        /* Adding flat hysteresis */
        data = temp | ((flat_int->flat_hy) & BMX160_FLAT_HYST_MASK);
        data_array[1] = data;

        /* Writing data to INT_FLAT 0 and INT_FLAT 1
         * registers simultaneously */
        rslt = bmx160_write_reg(ctx, BMX160_INT_FLAT_0_ADDR, data_array, 2, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API enable the external mode configuration.
 * @param[in] dev   : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_sec_if(bmx160_ctx_t* ctx)
{
    int32_t rslt;
    uint8_t if_conf = 0;
    uint8_t cmd = BMX160_AUX_NORMAL_MODE;

    /* set the aux power mode to normal*/
    rslt = bmx160_write_reg(ctx, BMX160_COMMAND_REG_ADDR, &cmd, 1, 1); //normal mode
    if (rslt == BMX160_OK)
    {
        /* 0.5ms delay - refer datasheet table 24*/
    	bmx160_delay_ms(ctx, 1);
        rslt = bmx160_read_reg(ctx, BMX160_IF_CONF_ADDR, &if_conf, 1);
        if_conf |= (uint8_t)(1 << 5);
        if (rslt == BMX160_OK)
        {
            /*enable the secondary interface also*/
            rslt = bmx160_write_reg(ctx, BMX160_IF_CONF_ADDR, &if_conf, 1, ctx->mode); //normal mode
        }
    }

    return rslt;
}

/*!
 * @brief This API enables the Low-g interrupt.
 * @param[in] low_g_int : Structure instance of bmi160_acc_low_g_int_cfg.
 * @param[in] dev   : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_low_g_int(bmx160_ctx_t* ctx, const bmx160_acc_low_g_int_cfg *low_g_int)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Enable low-g interrupt in Int Enable 1 register */
    rslt = bmx160_read_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_LOW_G_INT_EN_MASK;
        data = temp | ((low_g_int->low_en << 3) & BMX160_LOW_G_INT_EN_MASK);

        /* write data to Int Enable 0 register */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1, ctx->mode);
    }

    return rslt;
}
/*!
 * @brief This API configure the source of data(filter & pre-filter) for low-g interrupt.
 * @param[in] low_g_int : Structure instance of bmi160_acc_low_g_int_cfg.
 * @param[in] dev   : structure instance of bmi160_dev.
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_low_g_data_src(bmx160_ctx_t* ctx, const bmx160_acc_low_g_int_cfg *low_g_int)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Configure Int data 0 register to add source of interrupt */
    rslt = bmx160_read_reg(ctx, BMX160_INT_DATA_0_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_LOW_HIGH_SRC_INT_MASK;
        data = temp | ((low_g_int->low_data_src << 7) & BMX160_LOW_HIGH_SRC_INT_MASK);

        /* Write data to Data 0 address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_DATA_0_ADDR, &data, 1, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This API configure the necessary setting of low-g interrupt.
 * @param[in] low_g_int : Structure instance of bmi160_acc_low_g_int_cfg.
 * @param[in] dev   : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_low_g_int_settg(bmx160_ctx_t* ctx, const bmx160_acc_low_g_int_cfg *low_g_int)
{
    int32_t rslt;
    uint8_t temp = 0;
    uint8_t data_array[3] = { 0, 0, 0 };

    /* Configuring INT_LOWHIGH register for low-g interrupt */
    rslt = bmx160_read_reg(ctx, BMX160_INT_LOWHIGH_2_ADDR, &data_array[2], 1);
    if (rslt == BMX160_OK)
    {
        temp = data_array[2] & ~BMX160_LOW_G_HYST_MASK;

        /* Adding low-g hysteresis */
        data_array[2] = temp | (low_g_int->low_hyst & BMX160_LOW_G_HYST_MASK);
        temp = data_array[2] & ~BMX160_LOW_G_LOW_MODE_MASK;

        /* Adding low-mode */
        data_array[2] = temp | ((low_g_int->low_mode << 2) & BMX160_LOW_G_LOW_MODE_MASK);

        /* Adding low-g threshold */
        data_array[1] = low_g_int->low_thres;

        /* Adding low-g interrupt delay */
        data_array[0] = low_g_int->low_dur;

        /* Writing data to INT_LOWHIGH 0,1,2 registers simultaneously*/
        rslt = bmx160_write_reg(ctx, BMX160_INT_LOWHIGH_0_ADDR, data_array, 3, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This API enables the high-g interrupt.
 * @param[in] high_g_int_cfg : Structure instance of bmi160_acc_high_g_int_cfg.
 * @param[in] dev        : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_high_g_int(bmx160_ctx_t* ctx, const bmx160_acc_high_g_int_cfg *high_g_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Enable low-g interrupt in Int Enable 1 register */
    rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1, 1); //normal mode
    if (rslt == BMX160_OK)
    {
        /* Adding high-g X-axis */
        temp = data & ~BMX160_HIGH_G_X_INT_EN_MASK;
        data = temp | (high_g_int_cfg->high_g_x & BMX160_HIGH_G_X_INT_EN_MASK);

        /* Adding high-g Y-axis */
        temp = data & ~BMX160_HIGH_G_Y_INT_EN_MASK;
        data = temp | ((high_g_int_cfg->high_g_y << 1) & BMX160_HIGH_G_Y_INT_EN_MASK);

        /* Adding high-g Z-axis */
        temp = data & ~BMX160_HIGH_G_Z_INT_EN_MASK;
        data = temp | ((high_g_int_cfg->high_g_z << 2) & BMX160_HIGH_G_Z_INT_EN_MASK);

        /* write data to Int Enable 0 register */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This API configure the source of data(filter & pre-filter)
 * for high-g interrupt.
 * @param[in] high_g_int_cfg : Structure instance of bmi160_acc_high_g_int_cfg.
 * @param[in] dev        : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_high_g_data_src(bmx160_ctx_t* ctx, const bmx160_acc_high_g_int_cfg *high_g_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;

    /* Configure Int data 0 register to add source of interrupt */
    rslt = bmx160_read_reg(ctx, BMX160_INT_DATA_0_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        temp = data & ~BMX160_LOW_HIGH_SRC_INT_MASK;
        data = temp | ((high_g_int_cfg->high_data_src << 7) & BMX160_LOW_HIGH_SRC_INT_MASK);

        /* Write data to Data 0 address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_DATA_0_ADDR, &data, 1, ctx->mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the necessary setting of high-g interrupt.
 * @param[in] high_g_int_cfg : Structure instance of bmi160_acc_high_g_int_cfg.
 * @param[in] dev        : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t config_high_g_int_settg(bmx160_ctx_t* ctx, const bmx160_acc_high_g_int_cfg *high_g_int_cfg)
{
    int32_t rslt;
    uint8_t temp = 0;
    uint8_t data_array[3] = { 0, 0, 0 };

    rslt = bmx160_read_reg(ctx, BMX160_INT_LOWHIGH_2_ADDR, &data_array[0], 1);
    if (rslt == BMX160_OK)
    {
        temp = data_array[0] & ~BMX160_HIGH_G_HYST_MASK;

        /* Adding high-g hysteresis */
        data_array[0] = temp | ((high_g_int_cfg->high_hy << 6) & BMX160_HIGH_G_HYST_MASK);

        /* Adding high-g duration */
        data_array[1] = high_g_int_cfg->high_dur;

        /* Adding high-g threshold */
        data_array[2] = high_g_int_cfg->high_thres;
        rslt = bmx160_write_reg(ctx, BMX160_INT_LOWHIGH_2_ADDR, data_array, 3, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This enable the FIFO full interrupt engine.
 *
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] dev       : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_fifo_full_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
{
    int32_t rslt;
    uint8_t data = 0;

    rslt = bmx160_read_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        data = BMX160_SET_BITS(data, BMX160_FIFO_FULL_INT, int_config->fifo_full_int_en);

        /* Writing data to INT ENABLE 1 Address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1, ctx->mode);
    }

    return rslt;
}

/*!
 * @brief This enable the FIFO watermark interrupt engine.
 *
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] dev       : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_fifo_wtm_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
{
    int32_t rslt;
    uint8_t data = 0;

    rslt = bmx160_read_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        data = BMX160_SET_BITS(data, BMX160_FIFO_WTM_INT, int_config->fifo_wtm_int_en);

        /* Writing data to INT ENABLE 1 Address */
        rslt = bmx160_write_reg(ctx, BMX160_INT_ENABLE_1_ADDR, &data, 1, ctx->mode);
    }

    return rslt;
}

/*!
 *  @brief This API sets FIFO watermark interrupt of the sensor.The FIFO watermark interrupt is fired, when the FIFO fill level is above a fifowatermark.
 *
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] dev       : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t set_fifo_watermark_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
{
    int32_t rslt = BMX160_OK;

    /* Enable fifo-watermark interrupt in Int Enable 1 register */
    rslt = enable_fifo_wtm_int(ctx, int_config);
    if (rslt == BMX160_OK)
    {
    	/* Configure Interrupt pins */
        rslt = set_intr_pin_config(ctx, int_config);
        if (rslt == BMX160_OK)
        {
            rslt = map_hardware_interrupt(ctx, int_config);
         }
     }

    return rslt;
}

/*!
 *  @brief This API sets FIFO full interrupt of the sensor.This interrupt occurs when the FIFO is full and the next full data sample would cause
 *  a FIFO overflow, which may delete the old samples.
 *
 * @param[in] int_config    : Structure instance of bmi160_int_settg.
 * @param[in] dev       : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t set_fifo_full_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config)
{
    int32_t rslt = BMX160_OK;

    /*enable the fifo full interrupt */
    rslt = enable_fifo_full_int(ctx, int_config);
    if (rslt == BMX160_OK)
    {
    	/* Configure Interrupt pins */
        rslt = set_intr_pin_config(ctx, int_config);
        if (rslt == BMX160_OK)
        {
            rslt = map_hardware_interrupt(ctx, int_config);
        }
     }
    return rslt;
}

/*!
 *  @brief This API is used to read number of bytes filled currently in FIFO buffer.
 *
 *  @param[in] bytes_to_read  : Number of bytes available in FIFO at the instant which is obtained from FIFO counter.
 *  @param[in] dev            : Structure instance of bmi160_dev.
 *  @retval zero -> Success / -ve value -> Error.
 *  @retval Any non zero value -> Fail
 *
 */
int32_t get_fifo_byte_counter(bmx160_ctx_t* ctx, uint16_t *bytes_to_read)
{
    int32_t rslt = 0;
    uint8_t data[2];
    uint8_t addr = BMX160_FIFO_LENGTH_ADDR;

    rslt |= bmx160_read_reg(ctx, addr, data, 2);
    data[1] = data[1] & BMX160_FIFO_BYTE_COUNTER_MASK;

    /* Available data in FIFO is stored in bytes_to_read*/
    *bytes_to_read = (((uint16_t)data[1] << 8) | ((uint16_t)data[0]));

    return rslt;
}

/*!
 *  @brief This API is used to configure the offset enable bits in the sensor
 *
 *  @param[in,out] foc_conf   : Structure instance of bmi160_foc_conf which has the FOC and offset configurations
 *  @param[in] dev            : Structure instance of bmi160_dev.
 *  @retval zero -> Success  / -ve value -> Error
 */
int32_t configure_offset_enable(bmx160_ctx_t* ctx, const bmx160_foc_conf *foc_conf)
{
    int32_t rslt;
    uint8_t data;

    rslt = BMX160_E_NULL_PTR;

    /* Read the FOC config from the sensor */
    rslt = bmx160_read_reg(ctx, BMX160_OFFSET_CONF_ADDR, &data, 1);
        if (rslt == BMX160_OK)
        {
            /* Set the offset enable/disable for gyro */
            data = BMX160_SET_BITS(data, BMX160_GYRO_OFFSET_EN, foc_conf->gyro_off_en);

            /* Set the offset enable/disable for accel */
            data = BMX160_SET_BITS(data, BMX160_ACCEL_OFFSET_EN, foc_conf->acc_off_en);

            /* Set the offset config in the sensor */
            rslt = bmx160_write_reg(ctx, BMX160_OFFSET_CONF_ADDR, &data, 1, 1); //normal mode
        }


    return rslt;
}

/*!
 *  @brief This API is used to get the FOC status from the sensor
 *
 *  @param[in,out] foc_status   : Result of FOC status.
 *  @param[in] dev              : Structure instance of bmi160_dev.
 *  @retval zero -> Success  / -ve value -> Error
 */
int32_t get_foc_status(bmx160_ctx_t* ctx, uint8_t *foc_status)
{
    int32_t rslt;
    uint8_t data;

    /* Read the FOC status from sensor */
    rslt = bmx160_read_reg(ctx, BMX160_STATUS_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        /* Get the foc_status bit */
        *foc_status = BMX160_GET_BITS(data, BMX160_FOC_STATUS);
    }

    return rslt;
}

/*!
 * @brief This API reads accel data along with sensor time if time is requested by user. Kindly refer the user guide(README.md) for more info.
 *
 * @param[in] len    : len to read no of bytes
 * @param[out] accel    : Structure pointer to store accel data
 * @param[in] ctx       : Structure instance of bmi160_dev.

 * @retval zero -> Success  / -ve value -> Error
 */
int32_t get_accel_data(bmx160_ctx_t* ctx, uint8_t len, bmx160_sensor_data *accel)
{
    int32_t rslt;
    uint8_t idx = 0;
    uint8_t data_array[9] = { 0 };
    uint8_t time_0 = 0;
    uint16_t time_1 = 0;
    uint32_t time_2 = 0;
    uint8_t lsb;
    uint8_t msb;
    int16_t msblsb;

    /* read accel sensor data along with time if requested */
    rslt = bmx160_read_reg(ctx, BMX160_ACCEL_DATA_ADDR, data_array, 6 + len);
    if (rslt == BMX160_OK)
    {
        /* Accel Data */
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel->x = msblsb; /* Data in X axis */
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel->y = msblsb; /* Data in Y axis */
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel->z = msblsb; /* Data in Z axis */
        if (len == 3)
        {
            time_0 = data_array[idx++];
            time_1 = (uint16_t)(data_array[idx++] << 8);
            time_2 = (uint32_t)(data_array[idx++] << 16);
            accel->sensortime = (uint32_t)(time_2 | time_1 | time_0);
        }
        else
        {
            accel->sensortime = 0;
        }
    }
    else
    {
        rslt = BMX160_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API reads accel data along with sensor time if time is requested by user. Kindly refer the user guide(README.md) for more info.
 *
 * @param[in] len    : len to read no of bytes
 * @param[out] gyro    : Structure pointer to store accel data
 * @param[in] dev       : Structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t get_gyro_data(bmx160_ctx_t* ctx, uint8_t len, bmx160_sensor_data *gyro)
{
    int32_t rslt;
    uint8_t idx = 0;
    uint8_t data_array[15] = { 0 };
    uint8_t time_0 = 0;
    uint16_t time_1 = 0;
    uint32_t time_2 = 0;
    uint8_t lsb;
    uint8_t msb;
    int16_t msblsb;

    if (len == 0)
    {
        /* read gyro data only */
        rslt = bmx160_read_reg(ctx, BMX160_GYRO_DATA_ADDR, data_array, 6);
        if (rslt == BMX160_OK)
        {
            /* Gyro Data */
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro->x = msblsb; /* Data in X axis */
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro->y = msblsb; /* Data in Y axis */
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro->z = msblsb; /* Data in Z axis */
            gyro->sensortime = 0;
        }
        else
        {
            rslt = BMX160_E_COM_FAIL;
        }
    }
    else
    {
        /* read gyro sensor data along with time */
        rslt = bmx160_read_reg(ctx, BMX160_GYRO_DATA_ADDR, data_array, 12 + len);
        if (rslt == BMX160_OK)
        {
            /* Gyro Data */
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro->x = msblsb; /* gyro X axis data */
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro->y = msblsb; /* gyro Y axis data */
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro->z = msblsb; /* gyro Z axis data */
            idx = idx + 6;
            time_0 = data_array[idx++];
            time_1 = (uint16_t)(data_array[idx++] << 8);
            time_2 = (uint32_t)(data_array[idx++] << 16);
            gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);
        }
        else
        {
            rslt = BMX160_E_COM_FAIL;
        }
    }

    return rslt;
}

int32_t get_accel_gyro_data(bmx160_ctx_t* ctx, uint8_t len, bmx160_sensor_data *accel, bmx160_sensor_data *gyro)
{
    int8_t rslt;
    uint8_t idx = 0;
    uint8_t data_array[15] = { 0 };
    uint8_t time_0 = 0;
    uint16_t time_1 = 0;
    uint32_t time_2 = 0;
    uint8_t lsb;
    uint8_t msb;
    int16_t msblsb;

    /* read both accel and gyro sensor data
     * along with time if requested */
    rslt = bmx160_read_reg(ctx, BMX160_GYRO_DATA_ADDR, data_array, 12 + len);
    if (rslt == BMX160_OK)
    {
        /* Gyro Data */
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        gyro->x = msblsb; /* gyro X axis data */
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        gyro->y = msblsb; /* gyro Y axis data */
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        gyro->z = msblsb; /* gyro Z axis data */
        /* Accel Data */
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel->x = (int16_t)msblsb; /* accel X axis data */
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel->y = (int16_t)msblsb; /* accel Y axis data */
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel->z = (int16_t)msblsb; /* accel Z axis data */
        if (len == 3)
        {
            time_0 = data_array[idx++];
            time_1 = (uint16_t)(data_array[idx++] << 8);
            time_2 = (uint32_t)(data_array[idx++] << 16);
            accel->sensortime = (uint32_t)(time_2 | time_1 | time_0);
            gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);
        }
        else
        {
            accel->sensortime = 0;
            gyro->sensortime = 0;
        }
    }
    else
    {
        rslt = BMX160_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API sets the low-g interrupt of the sensor.This interrupt occurs during free-fall.
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int32_t set_accel_low_g_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config)
{
    int32_t rslt;

    if (int_config == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* updating the interrupt structure to local structure */
        bmx160_acc_low_g_int_cfg *low_g_int = &(int_config->int_type_cfg.acc_low_g_int);

        /* Enable the low-g interrupt*/
        rslt = enable_low_g_int(ctx, low_g_int);
        if (rslt == BMX160_OK)
        {
            /* Configure Interrupt pins */
            rslt = set_intr_pin_config(ctx, int_config);
            if (rslt == BMX160_OK)
            {
                /* Map INT pin to low-g interrupt */
                rslt = map_feature_interrupt(ctx, int_config);
                if (rslt == BMX160_OK)
                {
                    /* configure the data source
                     * for low-g interrupt*/
                    rslt = config_low_g_data_src(ctx, low_g_int);
                    if (rslt == BMX160_OK)
                    {
                        rslt = config_low_g_int_settg(ctx, low_g_int);
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the high-g interrupt of the sensor.The interrupt
 * occurs if the absolute value of acceleration data of any enabled axis
 * exceeds the programmed threshold and the sign of the value does not
 * change for a preset duration.
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
int32_t set_accel_high_g_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config)
{
    int32_t rslt;

    /* Null-pointer check */
    if (int_config == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* updating the interrupt structure to local structure */
        bmx160_acc_high_g_int_cfg *high_g_int_cfg = &(int_config->int_type_cfg.acc_high_g_int);

        /* Enable the high-g interrupt */
        rslt = enable_high_g_int(ctx, high_g_int_cfg);
        if (rslt == BMX160_OK)
        {
            /* Configure Interrupt pins */
            rslt = set_intr_pin_config(ctx, int_config);
            if (rslt == BMX160_OK)
            {
                /* Map INT pin to high-g interrupt */
                rslt = map_feature_interrupt(ctx, int_config);
                if (rslt == BMX160_OK)
                {
                    /* configure the data source
                     * for high-g interrupt*/
                    rslt = config_high_g_data_src(ctx, high_g_int_cfg);
                    if (rslt == BMX160_OK)
                    {
                        rslt = config_high_g_int_settg(ctx, high_g_int_cfg);
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API extract the read data from auxiliary sensor.
 *
 * @param[in] map_len     : burst read value.
 * @param[in] reg_addr    : Address of register to read.
 * @param[in] aux_data    : Pointer to store the read data.
 * @param[in] len     	  : length to read the data.
 * @param[in] dev         : Structure instance of bmi160_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */

int32_t extract_aux_read(bmx160_ctx_t* ctx, uint16_t map_len, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
    int32_t rslt = BMX160_OK;
    uint8_t data[8] = { 0, };
    uint8_t read_addr = BMX160_AUX_DATA_ADDR;
    uint8_t count = 0;
    uint8_t read_count;
    uint8_t read_len = (uint8_t)map_len;

    for (; count < len;)
    {
        /* set address to read */
        //rslt = bmx160_write_reg(ctx, BMX160_AUX_IF_2_ADDR, &reg_addr, 1, ctx->mode); BMI
	  rslt = bmx160_write_reg(ctx, BMX160_AUX_IF_1_ADDR, &reg_addr, 1, ctx->mode);
        //dev->delay_ms(BMI160_AUX_COM_DELAY);
        bmx160_delay_ms(ctx, BMX160_AUX_COM_DELAY);
        if (rslt == BMX160_OK)
        {
            rslt = bmx160_read_reg(ctx, read_addr, data, map_len);
            if (rslt == BMX160_OK)
            {
                read_count = 0;

                /* if read len is less the burst read len
                 * mention by user*/
                if (len < map_len)
                {
                    read_len = (uint8_t)len;
                }
                else if ((len - count) < map_len)
                {
                    read_len = (uint8_t)(len - count);
                }

                for (; read_count < read_len; read_count++)
                {
                    aux_data[count + read_count] = data[read_count];
                }
                reg_addr += (uint8_t)map_len;
                count += (uint8_t)map_len;
            }
            else
            {
                rslt = BMX160_E_COM_FAIL;
                break;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API enables the self test bit to trigger self test for gyro
 * @param[in] dev   : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t enable_gyro_self_test(bmx160_ctx_t* ctx)
{
    int32_t rslt;
    uint8_t reg_data;

    /* Enable the Gyro self test bit to trigger the self test */
    rslt = bmx160_read_reg(ctx,BMX160_SELF_TEST_ADDR, &reg_data, 1);
    if (rslt == BMX160_OK)
    {
        reg_data = BMX160_SET_BITS(reg_data, BMX160_GYRO_SELF_TEST, 1);
        rslt = bmx160_write_reg(ctx, BMX160_SELF_TEST_ADDR, &reg_data, 1, 1); //normal mode
        if (rslt == BMX160_OK)
        {
            /* Delay to enable gyro self test */
            //dev->delay_ms(15);
        	bmx160_delay_ms(ctx, 15);
        }
    }

    return rslt;
}

/*!
 * @brief This API validates the self test results of gyro
 * @param[in] dev   : structure instance of bmi160_dev.
 * @retval zero -> Success  / -ve value -> Error
 */
int32_t validate_gyro_self_test(bmx160_ctx_t* ctx)
{
    int32_t rslt;
    uint8_t reg_data;

    /* Validate the Gyro self test result */
    rslt = bmx160_read_reg(ctx, BMX160_STATUS_ADDR, &reg_data, 1);
    if (rslt == BMX160_OK)
    {

        reg_data = BMX160_GET_BITS(reg_data, BMX160_GYRO_SELF_TEST_STATUS);
        if (reg_data == BMX160_ENABLE)
        {
            /* Gyro self test success case */
            rslt = BMX160_OK;
        }
        else
        {
            rslt = BMX160_W_GYRO_SELF_TEST_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API validates the accel self test results
 * @param[in] accel_pos : Structure pointer to store accel data  for positive excitation
 * @param[in] accel_neg : Structure pointer to store accel data for negative excitation
 * @retval zero -> Success  / -ve value -> Error / +ve value -> Self test fail
 */
int32_t validate_accel_self_test(const bmx160_sensor_data *accel_pos, const bmx160_sensor_data *accel_neg)
{
    int32_t rslt;

    /* Validate the results of self test */
    if (((accel_neg->x - accel_pos->x) > BMX160_ACCEL_SELF_TEST_LIMIT) &&
        ((accel_neg->y - accel_pos->y) > BMX160_ACCEL_SELF_TEST_LIMIT) &&
        ((accel_neg->z - accel_pos->z) > BMX160_ACCEL_SELF_TEST_LIMIT))
    {
        /* Self test pass condition */
        rslt = BMX160_OK;
    }
    else
    {
        rslt = BMX160_W_ACCEl_SELF_TEST_FAIL;
    }

    return rslt;
}
