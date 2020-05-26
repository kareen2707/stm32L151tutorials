/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bmi160.h
* @date 10/01/2020
* @version  3.8.1
*
*/

/*!
 * @defgroup bmi160
 * @brief
 * @{*/

#ifndef BMI160_H_
#define BMI160_H_

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C" {
#endif

#include "bmx160_reg.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>


/** @defgroup BMX160_Exported_Types BMX160 Exported Types
 * @{
 */

typedef int32_t (*BMX160_Init_Func)(void);
typedef int32_t (*BMX160_DeInit_Func)(void);
typedef int32_t (*BMX160_WriteReg_Func)(uint8_t, uint8_t, uint8_t *, uint16_t);
typedef int32_t (*BMX160_ReadReg_Func)(uint8_t, uint8_t, uint8_t *, uint16_t);
typedef void 	(*BMX160_Delayms_Func)(uint32_t);

typedef struct
{
  BMX160_Init_Func          Init;
  BMX160_DeInit_Func        DeInit;
  uint32_t                  BusType; /*0 means I2C, 1 means SPI-4-Wires */
  uint8_t                   Address;
  BMX160_WriteReg_Func      WriteReg;
  BMX160_ReadReg_Func       ReadReg;
  BMX160_Delayms_Func		Delayms;
} BMX160_IO_t;

typedef struct BMX160_Object
{
  BMX160_IO_t        IO;
  bmx160_ctx_t       Ctx;
  uint8_t            is_initialized;
  uint8_t 			 chip_id;
  uint8_t			 id;
  /*! Hold active interrupts status for any and sig motion: 0 - Any-motion enable, 1 - Sig-motion enable, -1 neither any-motion nor sig-motion selected */
  bmx160_any_sig_motion_active_interrupt_state any_sig_sel;
  /*! Structure to configure Accel sensor */
  bmx160_cfg accel_cfg;

  /*! Structure to hold previous/old accel config parameters. This is used at driver level to prevent overwriting of same data, hence user does not change it in the code */
  bmx160_cfg prev_accel_cfg;

  /*! Structure to configure Gyro sensor */
  bmx160_cfg gyro_cfg;

  /*! Structure to hold previous/old gyro config parameters. This is used at driver level to prevent overwriting of same data, hence user does not change it in the code */
  bmx160_cfg prev_gyro_cfg;

  /*! Structure to configure the auxiliary sensor */
  bmx160_aux_cfg aux_cfg;

  /*! Structure to hold previous/old aux config parameters. This is used at driver level to prevent overwriting of same data, hence user does not change it in the code */
  bmx160_aux_cfg prev_aux_cfg;

   /*! FIFO related configurations */
  bmx160_fifo_frame *fifo;

   /*! User set read/write length */
  uint16_t read_write_len;

} BMX160_Object_t;

struct bmi160_dev
{
    /*! Chip Id */
    uint8_t chip_id;

    /*! Device Id */
    uint8_t id;

    /*! 0 - I2C , 1 - SPI Interface */
    uint8_t interface;

    /*! Hold active interrupts status for any and sig motion
     *  0 - Any-motion enable, 1 - Sig-motion enable,
     *  -1 neither any-motion nor sig-motion selected */
    bmx160_any_sig_motion_active_interrupt_state any_sig_sel;

    /*! Structure to configure Accel sensor */
    bmx160_cfg accel_cfg;

    /*! Structure to hold previous/old accel config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    bmx160_cfg prev_accel_cfg;

    /*! Structure to configure Gyro sensor */
    bmx160_cfg gyro_cfg;

    /*! Structure to hold previous/old gyro config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    bmx160_cfg prev_gyro_cfg;

    /*! Structure to configure the auxiliary sensor */
    bmx160_aux_cfg aux_cfg;

    /*! Structure to hold previous/old aux config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    bmx160_aux_cfg prev_aux_cfg;

    /*! FIFO related configurations */
    bmx160_fifo_frame *fifo;

    /*! Read function pointer */
    bmi160_com_fptr_t read;

    /*! Write function pointer */
    bmi160_com_fptr_t write;

    /*!  Delay function pointer */
    bmi160_delay_fptr_t delay_ms;

    /*! User set read/write length */
    uint16_t read_write_len;

    /*! For switching from I2C to SPI */
    uint8_t dummy_byte;
};

/*********************** User function prototypes ************************/

int32_t BMX160_RegisterBusIO(BMX160_Object_t *pObj, BMX160_IO_t *pIO);
/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of bmi160 sensor.
 *
 *  @param[in,out] dev : Structure instance of bmi160_dev
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
//int8_t bmi160_init(struct bmi160_dev *dev);
int32_t BMX160_Init(BMX160_Object_t *pObj);

/*!
 * @brief This API reads the data from the given register address of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] data     : Pointer to data buffer to store the read data.
 * @param[in] len       : No of bytes of data to be read.
 * @param[in] dev       : Structure instance of bmi160_dev.
 *
 * @note For most of the registers auto address increment applies, with the
 * exception of a few special registers, which trap the address. For e.g.,
 * Register address - 0x24(BMI160_FIFO_DATA_ADDR)
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev);

/*!
 * @brief This API writes the given data to the register address
 * of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be written.
 * @param[in] data      : Pointer to data buffer which is to be written in the sensor.
 * @param[in] len       : No of bytes of data to write.
 * @param[in] dev       : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmi160_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev);

/*!
 * @brief This API resets and restarts the device. All register values are overwritten with default parameters.
 *
 * @param[in] pObj  : BMX160 object.
 * @retval zero -> Success / -ve value -> Error.
 */
//int8_t bmi160_soft_reset(struct bmi160_dev *dev);
int32_t BMX160_soft_reset(BMX160_Object_t *pObj);
/*!
 * @brief This API configures the power mode, range and bandwidth of sensor.
 *
 * @param[in] pObj  : BMX160 object.
 * @note : Refer user guide for detailed info.
 * @retval zero -> Success / -ve value -> Error.
 */
//int8_t bmi160_set_sens_conf(struct bmi160_dev *dev);
int32_t BMX160_set_sens_conf(BMX160_Object_t *pObj);

/*!
 * @brief This API sets the power mode of the sensor.
 *
 * @param[in] pObj  : BMX160 object.
 * @retval zero -> Success / -ve value -> Error.
 */
//int8_t bmi160_set_power_mode(struct bmi160_dev *dev);
int32_t BMX160_set_power_mode(BMX160_Object_t *pObj);

/*!
 * @brief This API gets the power mode of the sensor.
 *
 * @param[in] power_mode  : Power mode of the sensor
 * @param[in] pObj  : BMX160 object.
 *
 * power_mode Macros possible values for pmu_status->aux_pmu_status :
 *  - BMI160_AUX_PMU_SUSPEND
 *  - BMI160_AUX_PMU_NORMAL
 *  - BMI160_AUX_PMU_LOW_POWER
 *
 * power_mode Macros possible values for pmu_status->gyro_pmu_status :
 *  - BMI160_GYRO_PMU_SUSPEND
 *  - BMI160_GYRO_PMU_NORMAL
 *  - BMI160_GYRO_PMU_FSU
 *
 * power_mode Macros possible values for pmu_status->accel_pmu_status :
 *  - BMI160_ACCEL_PMU_SUSPEND
 *  - BMI160_ACCEL_PMU_NORMAL
 *  - BMI160_ACCEL_PMU_LOW_POWER
 *
 * @retval zero -> Success / -ve value -> Error.
 */
//int8_t bmi160_get_power_mode(bmi160_pmu_status *pmu_status, const struct bmi160_dev *dev);
int32_t BMX160_get_power_mode(BMX160_Object_t *pObj, bmx160_pmu_status *pmu_status);
/*!
 * @brief This API reads sensor data, stores it in the bmx160_sensor_data structure pointer passed by the user.
 * The user can ask for accel data ,gyro data or both sensor data using bmx160_select_sensor enum
 *
 * @param[in] select_sensor    : enum to choose accel, gyro or both sensor data
 * @param[out] accel    	: Structure pointer to store accel data
 * @param[out] gyro     	: Structure pointer to store gyro data
 * @param[in] pObj  		: BMX160 object.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//int8_t bmi160_get_sensor_data(uint8_t select_sensor, bmi160_sensor_data *accel, bmi160_sensor_data *gyro, const struct bmi160_dev *dev);
int32_t BMX160_get_sensor_data(BMX160_Object_t *pObj, uint8_t select_sensor, bmx160_sensor_data *accel, bmx160_sensor_data *gyro);
/*!
 * @brief This API configures the necessary interrupt based on
 *  the user settings in the bmx160_int_settg structure instance.
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] pObj  : BMX160 object.
 * @note : Refer user guide for detailed info.
 *
 * @retval zero -> Success / -ve value -> Error
 */
//int8_t bmi160_set_int_config(bmi160_int_settg *int_config, struct bmi160_dev *dev);
int32_t BMX160_set_int_config(BMX160_Object_t *pObj, bmx160_int_settg *int_config);
/*!
 * @brief This API enables the step counter feature.
 *
 * @param[in] step_cnt_enable   : value to enable or disable
 * @param[in] pObj  : BMX160 object.
 * @note : Refer user guide for detailed info.
 *
 * @retval zero -> Success / -ve value -> Error
 */
//int8_t bmi160_set_step_counter(uint8_t step_cnt_enable, const struct bmi160_dev *dev);
int32_t BMX160_set_step_counter(BMX160_Object_t *pObj, uint8_t step_cnt_enable);
/*!
 * @brief This API reads the step counter value.
 *
 * @param[in] step_val    : Pointer to store the step counter value.
 * @param[in] pObj  : BMX160 object.
 * @note : Refer user guide for detailed info.
 *
 * @retval zero -> Success / -ve value -> Error
 */
//int8_t bmi160_read_step_counter(uint16_t *step_val, const struct bmi160_dev *dev);
int32_t BMX160_read_step_counter(BMX160_Object_t *pObj, uint16_t *step_val);
/*!
 * @brief This API reads the mention no of byte of data from the given register address of magnetometer sensor.
 *
 * @param[in] reg_addr    : Address of register to read.
 * @param[in] aux_data    : Pointer to store the read data.
 * @param[in] len     : No of bytes to read.
 * @param[in] pObj  : BMX160 object.
 * @note : Refer user guide for detailed info.
 *
 * @retval zero -> Success / -ve value -> Error
 */
//int8_t bmi160_aux_read(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, const struct bmi160_dev *dev);
int32_t BMX160_mag_read(BMX160_Object_t *pObj, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
/*!
 * @brief This API writes the mention no of byte of data to the given
 * register address of auxiliary sensor.
 *
 * @param[in] reg_addr    : Address of register to write.
 * @param[in] aux_data    : Pointer to write data.
 * @param[in] len     	  : No of bytes to write.
 * @param[in] pObj  	  : BMX160 object.
 * @note : Refer user guide for detailed info.
 *
 * @retval zero -> Success / -ve value -> Error
 */
//int8_t bmi160_aux_write(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, const struct bmi160_dev *dev);
int32_t BMX160_mag_write(BMX160_Object_t *pObj, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
/*!
 * @brief This API initialize the magnetometer sensor
 * in order to access it.
 *
 * @param[in] pObj  : BMX160 object.
 * @note : Refer user guide for detailed info.
 *
 * @retval zero -> Success / -ve value -> Error
 */
int32_t BMX160_mag_init(BMX160_Object_t *pObj);

/*!
 * @brief This API is used to setup the auxiliary sensor of bmx160 in auto mode
 * Thus enabling the auto update of 8 bytes of data from magnetometer sensor to BMX160 register address 0x04 to 0x0B
 *
 * @param[in] data_addr    : Starting address of mag. sensor's data register (BMX160 registers 0x04 to 0x0B will be updated
 *                           with 8 bytes of data from auxiliary sensor starting from this register address.)
 * @param[in] pObj  : BMX160 object.
 *
 * @note : Set the value of auxiliary polling rate by setting pObj->aux_cfg.aux_odr to the required value from the table
 *         before calling this API
 *
 *   pObj->aux_cfg.aux_odr  |   Auxiliary ODR (Hz)
 *  -----------------------|-----------------------
 *  BMI160_AUX_ODR_0_78HZ  |        25/32
 *  BMI160_AUX_ODR_1_56HZ  |        25/16
 *  BMI160_AUX_ODR_3_12HZ  |        25/8
 *  BMI160_AUX_ODR_6_25HZ  |        25/4
 *  BMI160_AUX_ODR_12_5HZ  |        25/2
 *  BMI160_AUX_ODR_25HZ    |        25
 *  BMI160_AUX_ODR_50HZ    |        50
 *  BMI160_AUX_ODR_100HZ   |        100
 *  BMI160_AUX_ODR_200HZ   |        200
 *  BMI160_AUX_ODR_400HZ   |        400
 *  BMI160_AUX_ODR_800HZ   |        800
 *
 * @note : Other values of  dev->aux_cfg.aux_odr are reserved and not for use
 *
 * @retval zero -> Success  / -ve value -> Error
 */
//int8_t bmi160_set_aux_auto_mode(uint8_t *data_addr, struct bmi160_dev *dev);
int32_t BMX160_set_mag_auto_mode(BMX160_Object_t *pObj, uint8_t *data_addr);
/*!
 * @brief This API configures the MAG_IF_1 register and settings like
 * Auxiliary sensor manual enable/ disable and aux burst read length.
 *
 * @param[in] pObj  : BMX160 object.
 *
 * @retval zero -> Success  / -ve value -> Error
 */
//int8_t bmi160_config_aux_mode(const struct bmi160_dev *dev);
int32_t BMX160_config_mag_mode(BMX160_Object_t *pObj);

/*!
 * @brief This API is used to read the raw uncompensated auxiliary sensor
 * data of 8 bytes from BMX160 register address 0x04 to 0x0B
 *
 * @param[in] aux_data       : Pointer to user array of length 8 bytes
 *                             Ensure that the aux_data array is of
 *                             length 8 bytes
 * @param[in] pObj  : BMX160 object.
 *
 * @retval zero -> Success  / -ve value -> Error
 */
//int8_t bmi160_read_aux_data_auto_mode(uint8_t *aux_data, const struct bmi160_dev *dev);
int32_t BMX160_read_mag_data_auto_mode(BMX160_Object_t *pObj, uint8_t *aux_data);
/*!
 * @brief This is used to perform self test of accel/gyro of the BMX160 sensor
 *
 * @param[in] select_sensor  : enum to choose accel or gyro for self test
 * @param[in] pObj  : BMX160 object.
 *
 * @note self test can be performed either for accel/gyro at any instant.
 *
 *     value of select_sensor       |  Inference
 *----------------------------------|--------------------------------
 *   BMI160_ACCEL_ONLY              | Accel self test enabled
 *   BMI160_GYRO_ONLY               | Gyro self test enabled
 *   BMI160_BOTH_ACCEL_AND_GYRO     | NOT TO BE USED
 *
 * @note The return value of this API gives us the result of self test.
 *
 * @note Performing self test does soft reset of the sensor, User can
 * set the desired settings after performing the self test.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error / +ve value -> Self-test fail
 *
 *   Return value                  |   Result of self test
 * --------------------------------|---------------------------------
 *  BMI160_OK                      |  Self test success
 *  BMI160_W_GYRO_SELF_TEST_FAIL   |  Gyro self test fail
 *  BMI160_W_ACCEl_SELF_TEST_FAIL  |  Accel self test fail
 */
//int8_t bmi160_perform_self_test(uint8_t select_sensor, struct bmi160_dev *dev);
int32_t BMX160_perform_self_test(BMX160_Object_t *pObj, uint8_t select_sensor);
/*!
 *  @brief This API reads data from the fifo buffer.
 *
 *  @note User has to allocate the FIFO buffer along with
 *  corresponding fifo length from his side before calling this API
 *  as mentioned in the readme.md
 *
 *  @note User must specify the number of bytes to read from the FIFO in
 *  pObj->fifo->length , It will be updated by the number of bytes actually
 *  read from FIFO after calling this API
 *
 *  @param[in] pObj  : BMX160 object.
 *
 *  @retval zero -> Success / -ve value -> Error
 *
 */
//int8_t bmi160_get_fifo_data(struct bmi160_dev const *dev);
int32_t BMX160_get_fifo_data(BMX160_Object_t *pObj);
/*!
 *  @brief This API writes fifo_flush command to command register.This
 *  action clears all data in the Fifo without changing fifo configuration
 *  settings.
 *
 *  @param[in] dev     : Structure instance of bmi160_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
//int8_t bmi160_set_fifo_flush(const struct bmi160_dev *dev);
int32_t BMX160_set_fifo_flush(BMX160_Object_t *pObj);

/*! @brief This API sets the FIFO configuration in the sensor.
 *
 *  @param[in] config : variable used to specify the FIFO
 *  configurations which are to be enabled or disabled in the sensor.
 *
 *  @note : User can set either set one or more or all FIFO configurations
 *  by ORing the below mentioned macros.
 *      config                  |   Value
 *      ------------------------|---------------------------
 *      BMI160_FIFO_TIME        |   0x02
 *      BMI160_FIFO_TAG_INT2    |   0x04
 *      BMI160_FIFO_TAG_INT1    |   0x08
 *      BMI160_FIFO_HEADER      |   0x10
 *      BMI160_FIFO_AUX         |   0x20
 *      BMI160_FIFO_ACCEL   |   0x40
 *      BMI160_FIFO_GYRO        |   0x80
 *
 *  @param[in] enable : Parameter used to enable or disable the above
 *  FIFO configuration
 *  @param[in] dev : Structure instance of bmi160_dev.
 *
 *  @return status of bus communication result
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
//int8_t bmi160_set_fifo_config(uint8_t config, uint8_t enable, struct bmi160_dev const *dev);
int32_t BMX160_set_fifo_config(BMX160_Object_t *pObj, uint8_t config, uint8_t enable);

/*! @brief This API is used to configure the down sampling ratios of the accel and gyro data for FIFO.
 *  Also, it configures filtered or pre-filtered data for the fifo for accel and gyro.
 *
 *  @param[in] fifo_down : variable used to specify the FIFO down
 *  configurations which are to be enabled or disabled in the sensor.
 *
 *  @note The user must select one among the following macros to
 *  select down-sampling ratio for accel
 *      config                               |   Value
 *      -------------------------------------|---------------------------
 *      BMI160_ACCEL_FIFO_DOWN_ZERO          |   0x00
 *      BMI160_ACCEL_FIFO_DOWN_ONE           |   0x10
 *      BMI160_ACCEL_FIFO_DOWN_TWO           |   0x20
 *      BMI160_ACCEL_FIFO_DOWN_THREE         |   0x30
 *      BMI160_ACCEL_FIFO_DOWN_FOUR          |   0x40
 *      BMI160_ACCEL_FIFO_DOWN_FIVE          |   0x50
 *      BMI160_ACCEL_FIFO_DOWN_SIX           |   0x60
 *      BMI160_ACCEL_FIFO_DOWN_SEVEN         |   0x70
 *
 *  @note The user must select one among the following macros to
 *  select down-sampling ratio for gyro
 *
 *      config                               |   Value
 *      -------------------------------------|---------------------------
 *      BMI160_GYRO_FIFO_DOWN_ZERO           |   0x00
 *      BMI160_GYRO_FIFO_DOWN_ONE            |   0x01
 *      BMI160_GYRO_FIFO_DOWN_TWO            |   0x02
 *      BMI160_GYRO_FIFO_DOWN_THREE          |   0x03
 *      BMI160_GYRO_FIFO_DOWN_FOUR           |   0x04
 *      BMI160_GYRO_FIFO_DOWN_FIVE           |   0x05
 *      BMI160_GYRO_FIFO_DOWN_SIX            |   0x06
 *      BMI160_GYRO_FIFO_DOWN_SEVEN          |   0x07
 *
 *  @note The user can enable filtered accel data by the following macro
 *      config                               |   Value
 *      -------------------------------------|---------------------------
 *      BMI160_ACCEL_FIFO_FILT_EN            |   0x80
 *
 *  @note The user can enable filtered gyro data by the following macro
 *      config                               |   Value
 *      -------------------------------------|---------------------------
 *      BMI160_GYRO_FIFO_FILT_EN             |   0x08
 *
 *  @note : By ORing the above mentioned macros, the user can select
 *  the required FIFO down config settings
 *
 *  @param[in] pObj  : BMX160 object.
 *
 *  @return status of bus communication result
 *  @retval 0 -> Success. Any non zero value -> Fail
 *
 */
//int8_t bmi160_set_fifo_down(uint8_t fifo_down, const struct bmi160_dev *dev);
int32_t BMX160_set_fifo_down(BMX160_Object_t *pObj, uint8_t fifo_down);
/*!
 *  @brief This API sets the FIFO watermark level in the sensor.
 *
 *  @note The FIFO watermark is issued when the FIFO fill level is
 *  equal or above the watermark level and units of watermark is 4 bytes.
 *
 *  @param[in]  fifo_wm        : Variable used to set the FIFO water mark level
 *  @param[in] pObj  : BMX160 object.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success. Any non zero value -> Fail
 *
 */
//int8_t bmi160_set_fifo_wm(uint8_t fifo_wm, const struct bmi160_dev *dev);
int32_t BMX160_set_fifo_wm(BMX160_Object_t *pObj, uint8_t fifo_wm);
/*!
 *  @brief This API parses and extracts the accelerometer frames from  FIFO data read
 *  by the "bmx160_get_fifo_data" API and stores it in the "accel_data" structure instance.
 *
 *  @note BMX160_extract_accel API should be called only after
 *  reading the FIFO data by calling the bmx160_get_fifo_data() API.
 *
 *  @param[out] accel_data    : Structure instance of bmx160_sensor_data
 *                              where the accelerometer data in FIFO is stored.
 *  @param[in,out] accel_length  : Number of valid accelerometer frames
 *                              (x,y,z axes data) read out from fifo.
 *  @param[in] pObj  : BMX160 object.
 *
 *  @note accel_length is updated with the number of valid accelerometer
 *  frames extracted from fifo (1 accel frame   = 6 bytes) at the end of
 *  execution of this API.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success. Any non zero value -> Fail
 *
 */
//int8_t bmi160_extract_accel(bmi160_sensor_data *accel_data, uint8_t *accel_length, struct bmi160_dev const *dev);
int32_t BMX160_extract_accel(BMX160_Object_t *pObj, bmx160_sensor_data *accel_data, uint8_t *accel_length);
/*!
 *  @brief This API parses and extracts the gyro frames from FIFO data read
 *  by the "bmi160_get_fifo_data" API and stores it in the "gyro_data" structure instance.
 *
 *  @note BMX160_extract_gyro API should be called only after
 *  reading the FIFO data by calling the bmx160_get_fifo_data() API.
 *
 *  @param[out] gyro_data    : Structure instance of bmi160_sensor_data
 *                             where the gyro data in FIFO is stored.
 *  @param[in,out] gyro_length  : Number of valid gyro frames
 *                             (x,y,z axes data) read out from fifo.
 *  @param[in] dev           : Structure instance of bmx160_dev.
 *
 *  @note gyro_length is updated with the number of valid gyro
 *  frames extracted from fifo (1 gyro frame   = 6 bytes) at the end of
 *  execution of this API.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success. Any non zero value -> Fail
 *
 */
//int8_t bmi160_extract_gyro(bmi160_sensor_data *gyro_data, uint8_t *gyro_length, struct bmi160_dev const *dev);
int32_t BMX160_extract_gyro(BMX160_Object_t *pObj, bmx160_sensor_data *gyro_data, uint8_t *gyro_length);
/*!
 *  @brief This API parses and extracts the magnetometer frames from FIFO data read
 *  by the "bmi160_get_fifo_data" API and stores it in the bmx160_aux_data structure instance.
 *
 *  @note The bmi160_extract_aux API should be called only after
 *  reading the FIFO data by calling the bmi160_get_fifo_data() API.
 *
 *  @param[out] aux_data    : Structure instance of bmx160_aux_data
 *                            where the aux data in FIFO is stored.
 *  @param[in,out] aux_len  : Number of valid aux frames (8bytes)
 *                            read out from FIFO.
 *  @param[in] pObj  : BMX160 object.
 *
 *  @note aux_len is updated with the number of valid aux
 *  frames extracted from fifo (1 aux frame = 8 bytes) at the end of
 *  execution of this API.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success. Any non zero value -> Fail
 *
 */
//int8_t bmi160_extract_aux(bmi160_aux_data *aux_data, uint8_t *aux_len, struct bmi160_dev const *dev);
int32_t BMX160_extract_mag(BMX160_Object_t *pObj, bmx160_aux_data *aux_data, uint8_t *aux_len);
/*!
 *  @brief This API starts the Fast Offset Compensation of accel and gyro
 *
 *  @note FOC should not be used in low-power mode of sensor
 *
 *  @note Accel FOC targets values of +1g , 0g , -1g
 *  Gyro FOC always targets value of 0 dps
 *
 *  @param[in] foc_conf    : Structure instance of bmx160_foc_conf which has the FOC configuration
 *  @param[in,out] offset  : Structure instance to store Offset values read from sensor
 *  @param[in] pObj  : BMX160 object.
 *
 *  @note Pre-requisites for triggering FOC in accel , Set the following, Enable the acc_off_en
 *       Ex :  foc_conf.acc_off_en = BMX160_ENABLE;
 *
 *   Set the desired target values of FOC to each axes (x,y,z) by using the
 *   following macros
 *       - BMI160_FOC_ACCEL_DISABLED
 *       - BMI160_FOC_ACCEL_POSITIVE_G
 *       - BMI160_FOC_ACCEL_NEGATIVE_G
 *       - BMI160_FOC_ACCEL_0G
 *
 *   Ex : foc_conf.foc_acc_x  = BMI160_FOC_ACCEL_0G;
 *        foc_conf.foc_acc_y  = BMI160_FOC_ACCEL_0G;
 *        foc_conf.foc_acc_z  = BMI160_FOC_ACCEL_POSITIVE_G;
 *
 *  @note Pre-requisites for triggering FOC in gyro ,
 *  Set the following parameters,
 *
 *   Ex : foc_conf.foc_gyr_en = BMI160_ENABLE;
 *        foc_conf.gyro_off_en = BMI160_ENABLE;
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success. Any non zero value -> Fail
 */
//int8_t bmi160_start_foc(const bmi160_foc_conf *foc_conf, bmi160_offsets *offset, struct bmi160_dev const *dev);
int32_t BMX160_start_foc(BMX160_Object_t *pObj, const bmx160_foc_conf *foc_conf, bmx160_offsets *offset);
/*!
 *  @brief This API reads and stores the offset values of accel and gyro
 *
 *  @param[in,out] offset : Structure instance of bmx160_offsets in which the offset values are read and stored
 *  @param[in] pObj  : BMX160 object.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success. Any non zero value -> Fail
 */
//int8_t bmi160_get_offsets(bmi160_offsets *offset, const struct bmi160_dev *dev);
int32_t BMX160_get_offsets(BMX160_Object_t *pObj, bmx160_offsets *offset);
/*!
 *  @brief This API writes the offset values of accel and gyro to
 *  the sensor but these values will be reset on POR or soft reset.
 *
 *  @param[in] foc_conf    : Structure instance of bmi160_foc_conf which has the FOC configuration
 *  @param[in] offset      : Structure instance in which user updates offset
 *                            values which are to be written in the sensor
 *  @param[in] pObj  : BMX160 object.
 *
 *  @note Offsets can be set by user like offset->off_acc_x = 10;
 *  where 1LSB = 3.9mg and for gyro 1LSB = 0.061degrees/second
 *
 * @note BMX160 offset values for xyz axes of accel should be within range of
 *  BMI160_ACCEL_MIN_OFFSET (-128) to BMI160_ACCEL_MAX_OFFSET (127)
 *
 * @note BMX160 offset values for xyz axes of gyro should be within range of
 *  BMI160_GYRO_MIN_OFFSET (-512) to BMI160_GYRO_MAX_OFFSET (511)
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success.Any non zero value -> Fail
 */
//int8_t bmi160_set_offsets(const bmi160_foc_conf *foc_conf, const bmi160_offsets *offset, struct bmi160_dev const *dev);
int32_t BMX160_set_offsets(BMX160_Object_t *pObj, const bmx160_foc_conf *foc_conf, const bmx160_offsets *offset);

/*!
 *  @brief This API writes the image registers values to Non-Volatile Memory which is
 *  stored even after POR or soft reset
 *
 *  @param[in] pObj  : BMX160 object.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success. Any non zero value -> Fail
 */
//int8_t bmi160_update_nvm(struct bmi160_dev const *dev);
int32_t BMX160_update_nvm(BMX160_Object_t *pObj);

/*!
 *  @brief This API gets the interrupt status from the sensor.
 *
 *  @param[in] int_status_sel       : Enum variable to select either individual or all the
 *  interrupt status bits.
 *  @param[in] int_status           : pointer variable to get the interrupt status
 *  from the sensor.
 *  @param[in] pObj  : BMX160 object.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success. Any non zero value -> Fail
 */
//int8_t bmi160_get_int_status(enum bmi160_int_status_sel int_status_sel, bmi160_int_status *int_status, struct bmi160_dev const *dev);
int32_t BMX160_get_int_status(BMX160_Object_t *pObj, enum bmx160_int_status_sel int_status_sel, bmx160_int_status *int_status);
/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif
/** @}*/
