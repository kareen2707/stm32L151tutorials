/*
 * bmx160.h
 *
 *  Created on: 19 mar. 2020
 *      Author: kareen
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BMX160_BMX160_H_
#define BMX160_BMX160_H_


#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "bmx160_reg.h"
#include <string.h>

/** Error code definitions */
#define BMX160_OK                            INT8_C(0)
#define BMX160_E_NULL_PTR                    INT8_C(-1)
#define BMX160_E_COM_FAIL                    INT8_C(-2)
#define BMX160_E_DEV_NOT_FOUND               INT8_C(-3)
#define BMX160_E_OUT_OF_RANGE                INT8_C(-4)
#define BMI160_E_INVALID_INPUT               INT8_C(-5)
#define BMI160_E_ACCEL_ODR_BW_INVALID        INT8_C(-6)
#define BMI160_E_GYRO_ODR_BW_INVALID         INT8_C(-7)
#define BMI160_E_LWP_PRE_FLTR_INT_INVALID    INT8_C(-8)
#define BMI160_E_LWP_PRE_FLTR_INVALID        INT8_C(-9)
#define BMI160_E_AUX_NOT_FOUND               INT8_C(-10)
#define BMI160_FOC_FAILURE                   INT8_C(-11)
#define BMI160_READ_WRITE_LENGHT_INVALID     INT8_C(-12)

#define BMX160_I2C_BUS                 0U
#define BMX160_SPI_4WIRES_BUS		   1U


typedef int32_t (*BMX160_Init_Func)(void);
typedef int32_t (*BMX160_DeInit_Func)(void);
typedef int32_t (*BMX160_GetTick_Func)(void);
typedef int32_t (*BMX160_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*BMX160_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef void	(*BMX160_Delay_Func)(uint32_t);

typedef struct
{
  BMX160_Init_Func          Init;
  BMX160_DeInit_Func        DeInit;
  uint32_t                  BusType; /*0 means I2C, 1 means SPI 4-Wires*/
  uint8_t                   Address;
  BMX160_WriteReg_Func      WriteReg;
  BMX160_ReadReg_Func       ReadReg;
  BMX160_Delay_Func			Delay;
  BMX160_GetTick_Func       GetTick;
} BMX160_IO_t;

typedef struct
{
  BMX160_IO_t         IO;
  bmx160_ctx_t        Ctx;
  uint8_t             is_initialized;
  /* uint8_t             acc_is_enabled;
  uint8_t             gyro_is_enabled;
  lsm6dsr_odr_xl_t    acc_odr;
  lsm6dsr_odr_g_t     gyro_odr; */

  /*! Chip Id */
    uint8_t chip_id;
  /*! Hold active interrupts status for any and sig motion
     *  0 - Any-motion enable, 1 - Sig-motion enable,
     *  -1 neither any-motion nor sig-motion selected */
    enum bmi160_any_sig_motion_active_interrupt_state any_sig_sel;

    /*! Structure to configure Accel sensor */
    struct bmi160_cfg accel_cfg;

    /*! Structure to hold previous/old accel config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    struct bmi160_cfg prev_accel_cfg;

    /*! Structure to configure Gyro sensor */
    struct bmi160_cfg gyro_cfg;

    /*! Structure to hold previous/old gyro config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    struct bmi160_cfg prev_gyro_cfg;

    /*! Structure to configure the auxiliary sensor */
    struct bmi160_aux_cfg aux_cfg;

    /*! Structure to hold previous/old aux config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    struct bmi160_aux_cfg prev_aux_cfg;

    /*! FIFO related configurations */
    struct bmi160_fifo_frame *fifo;
	/*! User set read/write length */
    uint16_t read_write_len;

    /*! For switching from I2C to SPI */
    uint8_t dummy_byte;

} BMX160_Object_t; //bmi160_dev

int32_t BMX160_RegisterBusIO(BMX160_Object_t *pObj, BMX160_IO_t *pIO);
int32_t BMX160_Init(BMX160_Object_t *pObj);
int32_t BMX160_DeInit(BMX160_Object_t *pObj); //Not implemented yet
int32_t BMX160_ReadID(BMX160_Object_t *pObj, uint8_t *Id); //same
int32_t BMX160_Soft_Reset(BMX160_Object_t *pObj); //same
int32_t BMX160_Set_Sens_Conf(BMX160_Object_t *pObj);

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
    enum bmi160_any_sig_motion_active_interrupt_state any_sig_sel;

    /*! Structure to configure Accel sensor */
    struct bmi160_cfg accel_cfg;

    /*! Structure to hold previous/old accel config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    struct bmi160_cfg prev_accel_cfg;

    /*! Structure to configure Gyro sensor */
    struct bmi160_cfg gyro_cfg;

    /*! Structure to hold previous/old gyro config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    struct bmi160_cfg prev_gyro_cfg;

    /*! Structure to configure the auxiliary sensor */
    struct bmi160_aux_cfg aux_cfg;

    /*! Structure to hold previous/old aux config parameters.
     * This is used at driver level to prevent overwriting of same
     * data, hence user does not change it in the code */
    struct bmi160_aux_cfg prev_aux_cfg;

    /*! FIFO related configurations */
    struct bmi160_fifo_frame *fifo;

    /*! User set read/write length */
    uint16_t read_write_len;

    /*! For switching from I2C to SPI */
    uint8_t dummy_byte;
};

#ifdef __cplusplus
}
#endif


#endif /* BSP_COMPONENTS_BMX160_BMX160_H_ */
