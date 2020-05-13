/*
 * bmx160_defs.h
 *
 *  Created on: Mar 17, 2020
 *      Author: karen@b105.upm.es
 */

#ifndef INC_BMX160_REG_H_
#define INC_BMX160_REG_H_

#include <stdint.h>
#include <stddef.h>

/*************************** Sensor macros   *****************************/

/* Test for an endian machine */
#ifndef __ORDER_LITTLE_ENDIAN__
#define __ORDER_LITTLE_ENDIAN__ 0
#endif

#ifndef __BYTE_ORDER__
#define __BYTE_ORDER__          __ORDER_LITTLE_ENDIAN__
#endif

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN           1
#endif
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#ifndef BIG_ENDIAN
#define BIG_ENDIAN              1
#endif
#else
#error "Code does not support Endian format of the processor"
#endif

/** Mask definitions */
#define BMX160_ACCEL_BW_MASK                 UINT8_C(0x70)
#define BMX160_ACCEL_ODR_MASK                UINT8_C(0x0F)
#define BMX160_ACCEL_UNDERSAMPLING_MASK      UINT8_C(0x80)
#define BMX160_ACCEL_RANGE_MASK              UINT8_C(0x0F)
#define BMX160_GYRO_BW_MASK                  UINT8_C(0x30)
#define BMX160_GYRO_ODR_MASK                 UINT8_C(0x0F)
#define BMX160_GYRO_RANGE_MSK                UINT8_C(0x07)

/** Mask definitions for INT_EN registers */
#define BMX160_ANY_MOTION_X_INT_EN_MASK      UINT8_C(0x01)
#define BMX160_HIGH_G_X_INT_EN_MASK          UINT8_C(0x01)
#define BMX160_NO_MOTION_X_INT_EN_MASK       UINT8_C(0x01)
#define BMX160_ANY_MOTION_Y_INT_EN_MASK      UINT8_C(0x02)
#define BMX160_HIGH_G_Y_INT_EN_MASK          UINT8_C(0x02)
#define BMX160_NO_MOTION_Y_INT_EN_MASK       UINT8_C(0x02)
#define BMX160_ANY_MOTION_Z_INT_EN_MASK      UINT8_C(0x04)
#define BMX160_HIGH_G_Z_INT_EN_MASK          UINT8_C(0x04)
#define BMX160_NO_MOTION_Z_INT_EN_MASK       UINT8_C(0x04)
#define BMX160_SIG_MOTION_INT_EN_MASK        UINT8_C(0x07)
#define BMX160_ANY_MOTION_ALL_INT_EN_MASK    UINT8_C(0x07)
#define BMX160_STEP_DETECT_INT_EN_MASK       UINT8_C(0x08)
#define BMX160_DOUBLE_TAP_INT_EN_MASK        UINT8_C(0x10)
#define BMX160_SINGLE_TAP_INT_EN_MASK        UINT8_C(0x20)
#define BMX160_FIFO_FULL_INT_EN_MASK         UINT8_C(0x20)
#define BMX160_ORIENT_INT_EN_MASK            UINT8_C(0x40)
#define BMX160_FIFO_WATERMARK_INT_EN_MASK    UINT8_C(0x40)
#define BMX160_LOW_G_INT_EN_MASK             UINT8_C(0x08)
#define BMX160_STEP_DETECT_EN_MASK           UINT8_C(0x08)
#define BMX160_FLAT_INT_EN_MASK              UINT8_C(0x80)
#define BMX160_DATA_RDY_INT_EN_MASK          UINT8_C(0x10)

/** PMU status Macros */
#define BMX160_AUX_PMU_SUSPEND               UINT8_C(0x00)	//Acceleration status
#define BMX160_AUX_PMU_NORMAL                UINT8_C(0x01)
#define BMX160_AUX_PMU_LOW_POWER             UINT8_C(0x02)

#define BMX160_GYRO_PMU_SUSPEND              UINT8_C(0x00)
#define BMX160_GYRO_PMU_NORMAL               UINT8_C(0x01)
#define BMX160_GYRO_PMU_FSU                  UINT8_C(0x03)

#define BMX160_ACCEL_PMU_SUSPEND             UINT8_C(0x00)
#define BMX160_ACCEL_PMU_NORMAL              UINT8_C(0x01)
#define BMX160_ACCEL_PMU_LOW_POWER           UINT8_C(0x02)

/** Mask definitions for INT_OUT_CTRL register */
#define BMX160_INT1_EDGE_CTRL_MASK           UINT8_C(0x01)
#define BMX160_INT1_OUTPUT_MODE_MASK         UINT8_C(0x04)
#define BMX160_INT1_OUTPUT_TYPE_MASK         UINT8_C(0x02)
#define BMX160_INT1_OUTPUT_EN_MASK           UINT8_C(0x08)
#define BMX160_INT2_EDGE_CTRL_MASK           UINT8_C(0x10)
#define BMX160_INT2_OUTPUT_MODE_MASK         UINT8_C(0x40)
#define BMX160_INT2_OUTPUT_TYPE_MASK         UINT8_C(0x20)
#define BMX160_INT2_OUTPUT_EN_MASK           UINT8_C(0x80)

/** Mask definitions for INT_LATCH register */
#define BMX160_INT1_INPUT_EN_MASK            UINT8_C(0x10)
#define BMX160_INT2_INPUT_EN_MASK            UINT8_C(0x20)
#define BMX160_INT_LATCH_MASK                UINT8_C(0x0F)

/** Mask definitions for INT_MAP register */
#define BMX160_INT1_LOW_G_MASK               UINT8_C(0x01)
#define BMX160_INT1_HIGH_G_MASK              UINT8_C(0x02)
#define BMX160_INT1_SLOPE_MASK               UINT8_C(0x04)
#define BMX160_INT1_NO_MOTION_MASK           UINT8_C(0x08)
#define BMX160_INT1_DOUBLE_TAP_MASK          UINT8_C(0x10)
#define BMX160_INT1_SINGLE_TAP_MASK          UINT8_C(0x20)
#define BMX160_INT1_FIFO_FULL_MASK           UINT8_C(0x20)
#define BMX160_INT1_FIFO_WM_MASK             UINT8_C(0x40)
#define BMX160_INT1_ORIENT_MASK              UINT8_C(0x40)
#define BMX160_INT1_FLAT_MASK                UINT8_C(0x80)
#define BMX160_INT1_DATA_READY_MASK          UINT8_C(0x80)
#define BMX160_INT2_LOW_G_MASK               UINT8_C(0x01)
#define BMX160_INT1_LOW_STEP_DETECT_MASK     UINT8_C(0x01)
#define BMX160_INT2_LOW_STEP_DETECT_MASK     UINT8_C(0x01)
#define BMX160_INT2_HIGH_G_MASK              UINT8_C(0x02)
#define BMX160_INT2_FIFO_FULL_MASK           UINT8_C(0x02)
#define BMX160_INT2_FIFO_WM_MASK             UINT8_C(0x04)
#define BMX160_INT2_SLOPE_MASK               UINT8_C(0x04)
#define BMX160_INT2_DATA_READY_MASK          UINT8_C(0x08)
#define BMX160_INT2_NO_MOTION_MASK           UINT8_C(0x08)
#define BMX160_INT2_DOUBLE_TAP_MASK          UINT8_C(0x10)
#define BMX160_INT2_SINGLE_TAP_MASK          UINT8_C(0x20)
#define BMX160_INT2_ORIENT_MASK              UINT8_C(0x40)
#define BMX160_INT2_FLAT_MASK                UINT8_C(0x80)

/** Mask definitions for INT_DATA register */
#define BMX160_TAP_SRC_INT_MASK              UINT8_C(0x08)
#define BMX160_LOW_HIGH_SRC_INT_MASK         UINT8_C(0x80)
#define BMX160_MOTION_SRC_INT_MASK           UINT8_C(0x80)

/** Mask definitions for INT_MOTION register */
#define BMX160_SLOPE_INT_DUR_MASK            UINT8_C(0x03)
#define BMX160_NO_MOTION_INT_DUR_MASK        UINT8_C(0xFC)
#define BMX160_NO_MOTION_SEL_BIT_MASK        UINT8_C(0x01)

/** Mask definitions for INT_TAP register */
#define BMX160_TAP_DUR_MASK                  UINT8_C(0x07)
#define BMX160_TAP_SHOCK_DUR_MASK            UINT8_C(0x40)
#define BMX160_TAP_QUIET_DUR_MASK            UINT8_C(0x80)
#define BMX160_TAP_THRES_MASK                UINT8_C(0x1F)

/** Mask definitions for INT_FLAT register */
#define BMX160_FLAT_THRES_MASK               UINT8_C(0x3F)
#define BMX160_FLAT_HOLD_TIME_MASK           UINT8_C(0x30)
#define BMX160_FLAT_HYST_MASK                UINT8_C(0x07)

/** Mask definitions for INT_LOWHIGH register */
#define BMX160_LOW_G_HYST_MASK               UINT8_C(0x03)
#define BMX160_LOW_G_LOW_MODE_MASK           UINT8_C(0x04)
#define BMX160_HIGH_G_HYST_MASK              UINT8_C(0xC0)

/** Mask definitions for INT_SIG_MOTION register */
#define BMX160_SIG_MOTION_SEL_MASK           UINT8_C(0x02)
#define BMX160_SIG_MOTION_SKIP_MASK          UINT8_C(0x0C)
#define BMX160_SIG_MOTION_PROOF_MASK         UINT8_C(0x30)

/** Mask definitions for INT_ORIENT register */
#define BMX160_ORIENT_MODE_MASK              UINT8_C(0x03)
#define BMX160_ORIENT_BLOCK_MASK             UINT8_C(0x0C)
#define BMX160_ORIENT_HYST_MASK              UINT8_C(0xF0)
#define BMX160_ORIENT_THETA_MASK             UINT8_C(0x3F)
#define BMX160_ORIENT_UD_ENABLE              UINT8_C(0x40)
#define BMX160_AXES_EN_MASK                  UINT8_C(0x80)

/** Mask definitions for FIFO_CONFIG register */
#define BMX160_FIFO_GYRO                     UINT8_C(0x80)
#define BMX160_FIFO_ACCEL                    UINT8_C(0x40)
#define BMX160_FIFO_AUX                      UINT8_C(0x20)
#define BMX160_FIFO_TAG_INT1                 UINT8_C(0x08)
#define BMX160_FIFO_TAG_INT2                 UINT8_C(0x04)
#define BMX160_FIFO_TIME                     UINT8_C(0x02)
#define BMX160_FIFO_HEADER                   UINT8_C(0x10)
#define BMX160_FIFO_CONFIG_1_MASK            UINT8_C(0xFE)

/** Mask definitions for STEP_CONF register */
#define BMX160_STEP_COUNT_EN_BIT_MASK        UINT8_C(0x08)
#define BMX160_STEP_DETECT_MIN_THRES_MASK    UINT8_C(0x18)
#define BMX160_STEP_DETECT_STEPTIME_MIN_MASK UINT8_C(0x07)
#define BMX160_STEP_MIN_BUF_MASK             UINT8_C(0x07)

/** Mask definition for FIFO Header Data Tag */
#define BMX160_FIFO_TAG_INTR_MASK            UINT8_C(0xFC)

/** Fifo byte counter mask definitions */
#define BMX160_FIFO_BYTE_COUNTER_MASK        UINT8_C(0x07)

/** Enable/disable bit value */
#define BMX160_ENABLE                        UINT8_C(0x01)
#define BMX160_DISABLE                       UINT8_C(0x00)

/** Latch Duration */
#define BMX160_LATCH_DUR_NONE                UINT8_C(0x00)
#define BMX160_LATCH_DUR_312_5_MICRO_SEC     UINT8_C(0x01)
#define BMX160_LATCH_DUR_625_MICRO_SEC       UINT8_C(0x02)
#define BMX160_LATCH_DUR_1_25_MILLI_SEC      UINT8_C(0x03)
#define BMX160_LATCH_DUR_2_5_MILLI_SEC       UINT8_C(0x04)
#define BMX160_LATCH_DUR_5_MILLI_SEC         UINT8_C(0x05)
#define BMX160_LATCH_DUR_10_MILLI_SEC        UINT8_C(0x06)
#define BMX160_LATCH_DUR_20_MILLI_SEC        UINT8_C(0x07)
#define BMX160_LATCH_DUR_40_MILLI_SEC        UINT8_C(0x08)
#define BMX160_LATCH_DUR_80_MILLI_SEC        UINT8_C(0x09)
#define BMX160_LATCH_DUR_160_MILLI_SEC       UINT8_C(0x0A)
#define BMX160_LATCH_DUR_320_MILLI_SEC       UINT8_C(0x0B)
#define BMX160_LATCH_DUR_640_MILLI_SEC       UINT8_C(0x0C)
#define BMX160_LATCH_DUR_1_28_SEC            UINT8_C(0x0D)
#define BMX160_LATCH_DUR_2_56_SEC            UINT8_C(0x0E)
#define BMI160_LATCHED                       UINT8_C(0x0F)

/** BMI160 Register map */
#define BMX160_CHIP_ID_ADDR                  UINT8_C(0x00)
#define BMX160_ERROR_REG_ADDR                UINT8_C(0x02)
#define BMX160_PMU_STATUS_ADDR               UINT8_C(0x03)
#define BMX160_AUX_DATA_ADDR                 UINT8_C(0x04) // mag and rhall data
#define BMX160_GYRO_DATA_ADDR                UINT8_C(0x0C)
#define BMX160_ACCEL_DATA_ADDR               UINT8_C(0x12)
#define BMX160_STATUS_ADDR                   UINT8_C(0x1B)
#define BMX160_INT_STATUS_ADDR               UINT8_C(0x1C)
#define BMX160_FIFO_LENGTH_ADDR              UINT8_C(0x22)
#define BMX160_FIFO_DATA_ADDR                UINT8_C(0x24)
#define BMX160_ACCEL_CONFIG_ADDR             UINT8_C(0x40)
#define BMX160_ACCEL_RANGE_ADDR              UINT8_C(0x41)
#define BMX160_GYRO_CONFIG_ADDR              UINT8_C(0x42)
#define BMX160_GYRO_RANGE_ADDR               UINT8_C(0x43)
#define BMX160_AUX_ODR_ADDR                  UINT8_C(0x44)
#define BMX160_FIFO_DOWN_ADDR                UINT8_C(0x45)
#define BMX160_FIFO_CONFIG_0_ADDR            UINT8_C(0x46)
#define BMX160_FIFO_CONFIG_1_ADDR            UINT8_C(0x47)
#define BMX160_AUX_IF_0_ADDR                 UINT8_C(0x4C) // BMI: 0x4B, BMX: 0x4C
#define BMX160_AUX_IF_1_ADDR                 UINT8_C(0x4D) // BMI: 0x4C, BMX: 0x4D
#define BMX160_AUX_IF_2_ADDR                 UINT8_C(0x4E) // BMI: 0x4D, BMX: 0x4E
#define BMX160_AUX_IF_3_ADDR                 UINT8_C(0x4F) // BMI: 0x4E, BMX: 0x4F
//#define BMI160_AUX_IF_4_ADDR                 UINT8_C(0x4F) // BMI: 0x4F, BMX: it doesn't exist
#define BMX160_INT_ENABLE_0_ADDR             UINT8_C(0x50)
#define BMX160_INT_ENABLE_1_ADDR             UINT8_C(0x51)
#define BMX160_INT_ENABLE_2_ADDR             UINT8_C(0x52)
#define BMX160_INT_OUT_CTRL_ADDR             UINT8_C(0x53)
#define BMX160_INT_LATCH_ADDR                UINT8_C(0x54)
#define BMX160_INT_MAP_0_ADDR                UINT8_C(0x55)
#define BMX160_INT_MAP_1_ADDR                UINT8_C(0x56)
#define BMX160_INT_MAP_2_ADDR                UINT8_C(0x57)
#define BMX160_INT_DATA_0_ADDR               UINT8_C(0x58)
#define BMX160_INT_DATA_1_ADDR               UINT8_C(0x59)
#define BMX160_INT_LOWHIGH_0_ADDR            UINT8_C(0x5A)
#define BMX160_INT_LOWHIGH_1_ADDR            UINT8_C(0x5B)
#define BMX160_INT_LOWHIGH_2_ADDR            UINT8_C(0x5C)
#define BMX160_INT_LOWHIGH_3_ADDR            UINT8_C(0x5D)
#define BMX160_INT_LOWHIGH_4_ADDR            UINT8_C(0x5E)
#define BMX160_INT_MOTION_0_ADDR             UINT8_C(0x5F)
#define BMX160_INT_MOTION_1_ADDR             UINT8_C(0x60)
#define BMX160_INT_MOTION_2_ADDR             UINT8_C(0x61)
#define BMX160_INT_MOTION_3_ADDR             UINT8_C(0x62)
#define BMX160_INT_TAP_0_ADDR                UINT8_C(0x63)
#define BMX160_INT_TAP_1_ADDR                UINT8_C(0x64)
#define BMX160_INT_ORIENT_0_ADDR             UINT8_C(0x65)
#define BMI160_INT_ORIENT_1_ADDR             UINT8_C(0x66)
#define BMX160_INT_FLAT_0_ADDR               UINT8_C(0x67)
#define BMX160_INT_FLAT_1_ADDR               UINT8_C(0x68)
#define BMX160_FOC_CONF_ADDR                 UINT8_C(0x69)
#define BMX160_CONF_ADDR                     UINT8_C(0x6A)

#define BMX160_IF_CONF_ADDR                  UINT8_C(0x6B)
#define BMX160_SELF_TEST_ADDR                UINT8_C(0x6D)
#define BMX160_OFFSET_ADDR                   UINT8_C(0x71)
#define BMX160_OFFSET_CONF_ADDR              UINT8_C(0x77)
#define BMX160_INT_STEP_CNT_0_ADDR           UINT8_C(0x78)
#define BMX160_INT_STEP_CONFIG_0_ADDR        UINT8_C(0x7A)
#define BMX160_INT_STEP_CONFIG_1_ADDR        UINT8_C(0x7B)
#define BMX160_COMMAND_REG_ADDR              UINT8_C(0x7E)
#define BMX160_SPI_COMM_TEST_ADDR            UINT8_C(0x7F)
#define BMX160_INTL_PULLUP_CONF_ADDR         UINT8_C(0x85)

/** Error code definitions */
#define BMX160_OK                            INT8_C(0)
#define BMX160_E_NULL_PTR                    INT8_C(-1)
#define BMX160_E_COM_FAIL                    INT8_C(-2)
#define BMX160_E_DEV_NOT_FOUND               INT8_C(-3)
#define BMX160_E_OUT_OF_RANGE                INT8_C(-4)
#define BMX160_E_INVALID_INPUT               INT8_C(-5)
#define BMX160_E_ACCEL_ODR_BW_INVALID        INT8_C(-6)
#define BMX160_E_GYRO_ODR_BW_INVALID         INT8_C(-7)
#define BMX160_E_LWP_PRE_FLTR_INT_INVALID    INT8_C(-8)
#define BMX160_E_LWP_PRE_FLTR_INVALID        INT8_C(-9)
#define BMX160_E_AUX_NOT_FOUND               INT8_C(-10)
#define BMX160_FOC_FAILURE                   INT8_C(-11)
#define BMX160_READ_WRITE_LENGHT_INVALID     INT8_C(-12)

/**\name API warning codes */
#define BMX160_W_GYRO_SELF_TEST_FAIL         INT8_C(1)
#define BMX160_W_ACCEl_SELF_TEST_FAIL        INT8_C(2)

/** BMI160 unique chip identifier */
#define BMX160_CHIP_ID                       UINT8_C(0xD8) //Karen: BMX160 id 0xD8, BMI160 id 0xD1

/** Soft reset command */
#define BMX160_SOFT_RESET_CMD                UINT8_C(0xb6)
#define BMX160_SOFT_RESET_DELAY_MS           UINT8_C(1)

/** Start FOC command */
#define BMI160_START_FOC_CMD                 UINT8_C(0x03)

/** NVM backup enabling command */
#define BMX160_NVM_BACKUP_EN                 UINT8_C(0xA0)

/* Delay in ms settings */
#define BMX160_ACCEL_DELAY_MS                UINT8_C(5)
#define BMX160_GYRO_DELAY_MS                 UINT8_C(81)
#define BMX160_ONE_MS_DELAY                  UINT8_C(1)
#define BMX160_AUX_COM_DELAY                 UINT8_C(10)
#define BMX160_GYRO_SELF_TEST_DELAY          UINT8_C(20)
#define BMX160_ACCEL_SELF_TEST_DELAY         UINT8_C(50)

/** Self test configurations */
#define BMX160_ACCEL_SELF_TEST_CONFIG        UINT8_C(0x2C)
#define BMX160_ACCEL_SELF_TEST_POSITIVE_EN   UINT8_C(0x0D)
#define BMX160_ACCEL_SELF_TEST_NEGATIVE_EN   UINT8_C(0x09)
#define BMX160_ACCEL_SELF_TEST_LIMIT         UINT16_C(8192)

/** Power mode settings */
/* Accelerometer power mode */
#define BMX160_ACCEL_NORMAL_MODE             UINT8_C(0x11)
#define BMX160_ACCEL_LOWPOWER_MODE           UINT8_C(0x12)
#define BMX160_ACCEL_SUSPEND_MODE            UINT8_C(0x10)

/* Gyroscope power mode */
#define BMX160_GYRO_SUSPEND_MODE             UINT8_C(0x14)
#define BMX160_GYRO_NORMAL_MODE              UINT8_C(0x15)
#define BMX160_GYRO_FASTSTARTUP_MODE         UINT8_C(0x17)

/* Magnetometer power mode */
#define BMX160_AUX_SUSPEND_MODE              UINT8_C(0x18)
#define BMX160_AUX_NORMAL_MODE               UINT8_C(0x19)
#define BMX160_AUX_LOWPOWER_MODE             UINT8_C(0x1A)


/** Bandwidth, output data rate and read mode settings, BMI160_ACCEL_CONFIG_ADDR  */
/* Accelerometer Bandwidth */
#define BMX160_ACCEL_BW_OSR4_AVG1            UINT8_C(0x00)
#define BMX160_ACCEL_BW_OSR2_AVG2            UINT8_C(0x01)
#define BMX160_ACCEL_BW_NORMAL_AVG4          UINT8_C(0x02)
#define BMX160_ACCEL_BW_RES_AVG8             UINT8_C(0x03)
#define BMX160_ACCEL_BW_RES_AVG16            UINT8_C(0x04)
#define BMX160_ACCEL_BW_RES_AVG32            UINT8_C(0x05)
#define BMX160_ACCEL_BW_RES_AVG64            UINT8_C(0x06)
#define BMX160_ACCEL_BW_RES_AVG128           UINT8_C(0x07)

/* Accelerometer Output data rate */
#define BMX160_ACCEL_ODR_RESERVED            UINT8_C(0x00)
#define BMX160_ACCEL_ODR_0_78HZ              UINT8_C(0x01)
#define BMX160_ACCEL_ODR_1_56HZ              UINT8_C(0x02)
#define BMX160_ACCEL_ODR_3_12HZ              UINT8_C(0x03)
#define BMX160_ACCEL_ODR_6_25HZ              UINT8_C(0x04)
#define BMX160_ACCEL_ODR_12_5HZ              UINT8_C(0x05)
#define BMX160_ACCEL_ODR_25HZ                UINT8_C(0x06)
#define BMX160_ACCEL_ODR_50HZ                UINT8_C(0x07)
#define BMX160_ACCEL_ODR_100HZ               UINT8_C(0x08)
#define BMX160_ACCEL_ODR_200HZ               UINT8_C(0x09)
#define BMX160_ACCEL_ODR_400HZ               UINT8_C(0x0A)
#define BMX160_ACCEL_ODR_800HZ               UINT8_C(0x0B)
#define BMX160_ACCEL_ODR_1600HZ              UINT8_C(0x0C)
#define BMX160_ACCEL_ODR_RESERVED0           UINT8_C(0x0D)
#define BMX160_ACCEL_ODR_RESERVED1           UINT8_C(0x0E)
#define BMX160_ACCEL_ODR_RESERVED2           UINT8_C(0x0F)

/* Accelerometer Range, ACC_RANGE register */
#define BMX160_ACCEL_RANGE_2G                UINT8_C(0x03)
#define BMX160_ACCEL_RANGE_4G                UINT8_C(0x05)
#define BMX160_ACCEL_RANGE_8G                UINT8_C(0x08)
#define BMX160_ACCEL_RANGE_16G               UINT8_C(0x0C)

/* Maximum limits definition */
#define BMX160_ACCEL_ODR_MAX                 UINT8_C(15)
#define BMX160_ACCEL_BW_MAX                  UINT8_C(2)
#define BMX160_ACCEL_RANGE_MAX               UINT8_C(12)

/** Bandwidth, output data rate and read mode settings, BMI160_GYRO_CONFIG_ADDR  */
/* Gyroscope Bandwidth */
#define BMX160_GYRO_BW_OSR4_MODE             UINT8_C(0x00)
#define BMX160_GYRO_BW_OSR2_MODE             UINT8_C(0x01)
#define BMX160_GYRO_BW_NORMAL_MODE           UINT8_C(0x02)

/* Gyroscope Output data rate */
#define BMX160_GYRO_ODR_RESERVED             UINT8_C(0x00)
#define BMX160_GYRO_ODR_25HZ                 UINT8_C(0x06)
#define BMX160_GYRO_ODR_50HZ                 UINT8_C(0x07)
#define BMX160_GYRO_ODR_100HZ                UINT8_C(0x08)
#define BMX160_GYRO_ODR_200HZ                UINT8_C(0x09)
#define BMX160_GYRO_ODR_400HZ                UINT8_C(0x0A)
#define BMX160_GYRO_ODR_800HZ                UINT8_C(0x0B)
#define BMX160_GYRO_ODR_1600HZ               UINT8_C(0x0C)
#define BMX160_GYRO_ODR_3200HZ               UINT8_C(0x0D)

/* Gyroscope Range, GYR_RANGE */
#define BMX160_GYRO_RANGE_2000_DPS           UINT8_C(0x00)
#define BMX160_GYRO_RANGE_1000_DPS           UINT8_C(0x01)
#define BMX160_GYRO_RANGE_500_DPS            UINT8_C(0x02)
#define BMX160_GYRO_RANGE_250_DPS            UINT8_C(0x03)
#define BMX160_GYRO_RANGE_125_DPS            UINT8_C(0x04)

#define BMX160_GYRO_ODR_MAX                  UINT8_C(13)
#define BMX160_GYRO_BW_MAX                   UINT8_C(2)
#define BMX160_GYRO_RANGE_MAX                UINT8_C(4)

/** Magnetometer Output data rate, MAG_CONF **/
#define BMX160_AUX_ODR_RESERVED              UINT8_C(0x00)
#define BMX160_AUX_ODR_0_78HZ                UINT8_C(0x01)
#define BMX160_AUX_ODR_1_56HZ                UINT8_C(0x02)
#define BMX160_AUX_ODR_3_12HZ                UINT8_C(0x03)
#define BMX160_AUX_ODR_6_25HZ                UINT8_C(0x04)
#define BMX160_AUX_ODR_12_5HZ                UINT8_C(0x05)
#define BMX160_AUX_ODR_25HZ                  UINT8_C(0x06)
#define BMX160_AUX_ODR_50HZ                  UINT8_C(0x07)
#define BMX160_AUX_ODR_100HZ                 UINT8_C(0x08)
#define BMX160_AUX_ODR_200HZ                 UINT8_C(0x09)
#define BMX160_AUX_ODR_400HZ                 UINT8_C(0x0A)
#define BMX160_AUX_ODR_800HZ                 UINT8_C(0x0B)

/** FIFO DOWN selection */
/* Accel fifo down-sampling values*/
#define  BMX160_ACCEL_FIFO_DOWN_ZERO         UINT8_C(0x00)
#define  BMX160_ACCEL_FIFO_DOWN_ONE          UINT8_C(0x10)
#define  BMX160_ACCEL_FIFO_DOWN_TWO          UINT8_C(0x20)
#define  BMX160_ACCEL_FIFO_DOWN_THREE        UINT8_C(0x30)
#define  BMX160_ACCEL_FIFO_DOWN_FOUR         UINT8_C(0x40)
#define  BMX160_ACCEL_FIFO_DOWN_FIVE         UINT8_C(0x50)
#define  BMX160_ACCEL_FIFO_DOWN_SIX          UINT8_C(0x60)
#define  BMX160_ACCEL_FIFO_DOWN_SEVEN        UINT8_C(0x70)

/* Gyro fifo down-smapling values*/
#define  BMX160_GYRO_FIFO_DOWN_ZERO          UINT8_C(0x00)
#define  BMX160_GYRO_FIFO_DOWN_ONE           UINT8_C(0x01)
#define  BMX160_GYRO_FIFO_DOWN_TWO           UINT8_C(0x02)
#define  BMX160_GYRO_FIFO_DOWN_THREE         UINT8_C(0x03)
#define  BMX160_GYRO_FIFO_DOWN_FOUR          UINT8_C(0x04)
#define  BMX160_GYRO_FIFO_DOWN_FIVE          UINT8_C(0x05)
#define  BMX160_GYRO_FIFO_DOWN_SIX           UINT8_C(0x06)
#define  BMX160_GYRO_FIFO_DOWN_SEVEN         UINT8_C(0x07)

/** FIFO_CONFIG Definitions. This register can be used for setting the different modes
 * of operation of the FIFO, e.g. which data is going to be stored in in and which
 * format is going to be used (header/less mode).
 **/
#define BMX160_FIFO_TIME_ENABLE              UINT8_C(0x02)
#define BMX160_FIFO_TAG_INT2_ENABLE          UINT8_C(0x04)
#define BMX160_FIFO_TAG_INT1_ENABLE          UINT8_C(0x08)
#define BMX160_FIFO_HEAD_ENABLE              UINT8_C(0x10)
#define BMX160_FIFO_M_ENABLE                 UINT8_C(0x20)
#define BMX160_FIFO_A_ENABLE                 UINT8_C(0x40)
#define BMX160_FIFO_M_A_ENABLE               UINT8_C(0x60)
#define BMX160_FIFO_G_ENABLE                 UINT8_C(0x80)
#define BMX160_FIFO_M_G_ENABLE               UINT8_C(0xA0)
#define BMX160_FIFO_G_A_ENABLE               UINT8_C(0xC0)
#define BMX160_FIFO_M_G_A_ENABLE             UINT8_C(0xE0)

/* Macro to specify the number of bytes over-read from the
 * FIFO in order to get the sensor time at the end of FIFO */
#ifndef BMI160_FIFO_BYTES_OVERREAD
#define BMX160_FIFO_BYTES_OVERREAD           UINT8_C(25)
#endif

/* Accel, gyro and aux. sensor length and also their combined
 * length definitions in FIFO */
#define BMX160_FIFO_G_LENGTH                 UINT8_C(6)
#define BMX160_FIFO_A_LENGTH                 UINT8_C(6)
#define BMX160_FIFO_M_LENGTH                 UINT8_C(8)
#define BMX160_FIFO_GA_LENGTH                UINT8_C(12)
#define BMX160_FIFO_MA_LENGTH                UINT8_C(14)
#define BMX160_FIFO_MG_LENGTH                UINT8_C(14)
#define BMX160_FIFO_MGA_LENGTH               UINT8_C(20)

/** FIFO Header Data definitions */
#define BMX160_FIFO_HEAD_SKIP_FRAME          UINT8_C(0x40)
#define BMX160_FIFO_HEAD_SENSOR_TIME         UINT8_C(0x44)
#define BMX160_FIFO_HEAD_INPUT_CONFIG        UINT8_C(0x48)
#define BMX160_FIFO_HEAD_OVER_READ           UINT8_C(0x80)
#define BMX160_FIFO_HEAD_A                   UINT8_C(0x84)
#define BMX160_FIFO_HEAD_G                   UINT8_C(0x88)
#define BMX160_FIFO_HEAD_G_A                 UINT8_C(0x8C)
#define BMX160_FIFO_HEAD_M                   UINT8_C(0x90)
#define BMX160_FIFO_HEAD_M_A                 UINT8_C(0x94)
#define BMX160_FIFO_HEAD_M_G                 UINT8_C(0x98)
#define BMX160_FIFO_HEAD_M_G_A               UINT8_C(0x9C)

/** FIFO sensor time length definitions */
#define BMX160_SENSOR_TIME_LENGTH            UINT8_C(3)

/* Accel Fifo filter enable*/
#define  BMX160_ACCEL_FIFO_FILT_EN           UINT8_C(0x80)

/* Gyro Fifo filter enable*/
#define  BMX160_GYRO_FIFO_FILT_EN            UINT8_C(0x08)

/** Definitions to check validity of FIFO frames */
#define FIFO_CONFIG_MSB_CHECK                UINT8_C(0x80)
#define FIFO_CONFIG_LSB_CHECK                UINT8_C(0x00)

/** FOC_CONF configurations. Info related to the fast offset
 * compensation for the accelerometer and gyroscope */
#define BMX160_FOC_ACCEL_DISABLED            UINT8_C(0x00)
#define BMX160_FOC_ACCEL_POSITIVE_G          UINT8_C(0x01)
#define BMX160_FOC_ACCEL_NEGATIVE_G          UINT8_C(0x02)
#define BMX160_FOC_ACCEL_0G                  UINT8_C(0x03)

/** Array Parameter DefinItions */
#define BMX160_SENSOR_TIME_LSB_BYTE          UINT8_C(0)
#define BMX160_SENSOR_TIME_XLSB_BYTE         UINT8_C(1)
#define BMX160_SENSOR_TIME_MSB_BYTE          UINT8_C(2)

/** Interface settings */
#define BMX160_SPI_INTF                      UINT8_C(1)
#define BMX160_I2C_INTF                      UINT8_C(0)
#define BMX160_SPI_RD_MASK                   UINT8_C(0x80)
#define BMX160_SPI_WR_MASK                   UINT8_C(0x7F)

/* Sensor & time select definition*/
#define BMX160_ACCEL_SEL                     UINT8_C(0x01)
#define BMX160_GYRO_SEL                      UINT8_C(0x02)
#define BMX160_TIME_SEL                      UINT8_C(0x04)

/* Sensor select mask*/
#define BMX160_SEN_SEL_MASK                  UINT8_C(0x07)

/* Error code mask */
#define BMX160_ERR_REG_MASK                  UINT8_C(0x0F)

/* BMI160 I2C address */
#define BMX160_I2C_ADDR                      UINT8_C(0x68)

/* BMI160 secondary IF address */
#define BMX160_AUX_BMM150_I2C_ADDR           UINT8_C(0x10)

/** BMI160 Length definitions */
#define BMX160_ONE                           UINT8_C(1)
#define BMX160_TWO                           UINT8_C(2)
#define BMX160_THREE                         UINT8_C(3)
#define BMX160_FOUR                          UINT8_C(4)
#define BMX160_FIVE                          UINT8_C(5)

/** BMI160 fifo level Margin */
#define BMX160_FIFO_LEVEL_MARGIN             UINT8_C(16)

/** BMI160 fifo flush Command */
#define BMX160_FIFO_FLUSH_VALUE              UINT8_C(0xB0)

/** BMI160 offset values for xyz axes of accel */
#define BMX160_ACCEL_MIN_OFFSET              INT8_C(-128)
#define BMX160_ACCEL_MAX_OFFSET              INT8_C(127)

/** BMI160 offset values for xyz axes of gyro */
#define BMX160_GYRO_MIN_OFFSET               INT16_C(-512)
#define BMX160_GYRO_MAX_OFFSET               INT16_C(511)

/** BMI160 fifo full interrupt position and mask */
#define BMX160_FIFO_FULL_INT_POS             UINT8_C(5)
#define BMX160_FIFO_FULL_INT_MSK             UINT8_C(0x20)
#define BMX160_FIFO_WTM_INT_POS              UINT8_C(6)
#define BMX160_FIFO_WTM_INT_MSK              UINT8_C(0x40)

#define BMX160_FIFO_FULL_INT_PIN1_POS        UINT8_C(5)
#define BMX160_FIFO_FULL_INT_PIN1_MSK        UINT8_C(0x20)
#define BMX160_FIFO_FULL_INT_PIN2_POS        UINT8_C(1)
#define BMX160_FIFO_FULL_INT_PIN2_MSK        UINT8_C(0x02)

#define BMX160_FIFO_WTM_INT_PIN1_POS         UINT8_C(6)
#define BMX160_FIFO_WTM_INT_PIN1_MSK         UINT8_C(0x40)
#define BMX160_FIFO_WTM_INT_PIN2_POS         UINT8_C(2)
#define BMX160_FIFO_WTM_INT_PIN2_MSK         UINT8_C(0x04)

#define BMX160_MANUAL_MODE_EN_POS            UINT8_C(7)
#define BMX160_MANUAL_MODE_EN_MSK            UINT8_C(0x80)
#define BMX160_AUX_READ_BURST_POS            UINT8_C(0)
#define BMX160_AUX_READ_BURST_MSK            UINT8_C(0x03)

#define BMX160_GYRO_SELF_TEST_POS            UINT8_C(4)
#define BMX160_GYRO_SELF_TEST_MSK            UINT8_C(0x10)
#define BMX160_GYRO_SELF_TEST_STATUS_POS     UINT8_C(1)
#define BMX160_GYRO_SELF_TEST_STATUS_MSK     UINT8_C(0x02)

#define BMX160_GYRO_FOC_EN_POS               UINT8_C(6)
#define BMX160_GYRO_FOC_EN_MSK               UINT8_C(0x40)

#define BMX160_ACCEL_FOC_X_CONF_POS          UINT8_C(4)
#define BMX160_ACCEL_FOC_X_CONF_MSK          UINT8_C(0x30)

#define BMX160_ACCEL_FOC_Y_CONF_POS          UINT8_C(2)
#define BMX160_ACCEL_FOC_Y_CONF_MSK          UINT8_C(0x0C)

#define BMX160_ACCEL_FOC_Z_CONF_MSK          UINT8_C(0x03)

#define BMX160_FOC_STATUS_POS                UINT8_C(3)
#define BMX160_FOC_STATUS_MSK                UINT8_C(0x08)

#define BMX160_GYRO_OFFSET_X_MSK             UINT8_C(0x03)

#define BMX160_GYRO_OFFSET_Y_POS             UINT8_C(2)
#define BMX160_GYRO_OFFSET_Y_MSK             UINT8_C(0x0C)

#define BMX160_GYRO_OFFSET_Z_POS             UINT8_C(4)
#define BMX160_GYRO_OFFSET_Z_MSK             UINT8_C(0x30)

#define BMX160_GYRO_OFFSET_EN_POS            UINT8_C(7)
#define BMX160_GYRO_OFFSET_EN_MSK            UINT8_C(0x80)

#define BMX160_ACCEL_OFFSET_EN_POS           UINT8_C(6)
#define BMX160_ACCEL_OFFSET_EN_MSK           UINT8_C(0x40)

#define BMX160_GYRO_OFFSET_POS               UINT16_C(8)
#define BMX160_GYRO_OFFSET_MSK               UINT16_C(0x0300)

#define BMX160_NVM_UPDATE_POS                UINT8_C(1)
#define BMX160_NVM_UPDATE_MSK                UINT8_C(0x02)

#define BMX160_NVM_STATUS_POS                UINT8_C(4)
#define BMX160_NVM_STATUS_MSK                UINT8_C(0x10)

#define BMX160_MAG_POWER_MODE_MSK            UINT8_C(0x03)

#define BMX160_ACCEL_POWER_MODE_MSK          UINT8_C(0x30)
#define BMX160_ACCEL_POWER_MODE_POS          UINT8_C(4)

#define BMX160_GYRO_POWER_MODE_MSK           UINT8_C(0x0C)
#define BMX160_GYRO_POWER_MODE_POS           UINT8_C(2)

/* BIT SLICE GET AND SET FUNCTIONS */
#define BMX160_GET_BITS(regvar, bitname) \
    ((regvar & bitname##_MSK) >> bitname##_POS)
#define BMX160_SET_BITS(regvar, bitname, val) \
    ((regvar & ~bitname##_MSK) | \
     ((val << bitname##_POS) & bitname##_MSK))

#define BMX160_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BMX160_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

/**\name UTILITY MACROS */
#define BMX160_SET_LOW_BYTE  UINT16_C(0x00FF)
#define BMX160_SET_HIGH_BYTE UINT16_C(0xFF00)

#define BMX160_GET_LSB(var) (uint8_t)(var & BMX160_SET_LOW_BYTE)
#define BMX160_GET_MSB(var) (uint8_t)((var & BMX160_SET_HIGH_BYTE) >> 8)

/*****************************************************************************/
/* type definitions */
typedef int8_t (*bmi160_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bmi160_delay_fptr_t)(uint32_t period);

/*************************** Data structures *********************************/
typedef struct
{
    /*! Power mode status of Accel
     * Possible values :
     *  - BMI160_ACCEL_PMU_SUSPEND
     *  - BMI160_ACCEL_PMU_NORMAL
     *  - BMI160_ACCEL_PMU_LOW_POWER
     */
    uint8_t accel_pmu_status;

    /*! Power mode status of Gyro
     * Possible values :
     *  - BMI160_GYRO_PMU_SUSPEND
     *  - BMI160_GYRO_PMU_NORMAL
     *  - BMI160_GYRO_PMU_FSU
     */
    uint8_t gyro_pmu_status;

    /*! Power mode status of 'Auxiliary sensor interface' whereas the actual
     *  power mode of the aux. sensor should be configured
     *  according to the connected sensor specifications
     * Possible values :
     *  - BMI160_AUX_PMU_SUSPEND
     *  - BMI160_AUX_PMU_NORMAL
     *  - BMI160_AUX_PMU_LOW_POWER
     */
    uint8_t aux_pmu_status;
}bmx160_pmu_status;

/*!
 * @brief bmi160 interrupt status selection enum.
 */
enum bmx160_int_status_sel {
    BMI160_INT_STATUS_0 = 1,
    BMI160_INT_STATUS_1 = 2,
    BMI160_INT_STATUS_2 = 4,
    BMI160_INT_STATUS_3 = 8,
    BMI160_INT_STATUS_ALL = 15
};

/*!
 * @brief bmi160 interrupt status bits structure
 */
typedef struct
{
#if LITTLE_ENDIAN == 1
    uint32_t step : 1;
    uint32_t sigmot : 1;
    uint32_t anym : 1;

    /* pmu trigger will be handled later */
    uint32_t pmu_trigger_reserved : 1;
    uint32_t d_tap : 1;
    uint32_t s_tap : 1;
    uint32_t orient : 1;
    uint32_t flat_int : 1;
    uint32_t reserved : 2;
    uint32_t high_g : 1;
    uint32_t low_g : 1;
    uint32_t drdy : 1;
    uint32_t ffull : 1;
    uint32_t fwm : 1;
    uint32_t nomo : 1;
    uint32_t anym_first_x : 1;
    uint32_t anym_first_y : 1;
    uint32_t anym_first_z : 1;
    uint32_t anym_sign : 1;
    uint32_t tap_first_x : 1;
    uint32_t tap_first_y : 1;
    uint32_t tap_first_z : 1;
    uint32_t tap_sign : 1;
    uint32_t high_first_x : 1;
    uint32_t high_first_y : 1;
    uint32_t high_first_z : 1;
    uint32_t high_sign : 1;
    uint32_t orient_1_0 : 2;
    uint32_t orient_2 : 1;
    uint32_t flat : 1;
#elif BIG_ENDIAN == 1
    uint32_t high_first_x : 1;
    uint32_t high_first_y : 1;
    uint32_t high_first_z : 1;
    uint32_t high_sign : 1;
    uint32_t orient_1_0 : 2;
    uint32_t orient_2 : 1;
    uint32_t flat : 1;
    uint32_t anym_first_x : 1;
    uint32_t anym_first_y : 1;
    uint32_t anym_first_z : 1;
    uint32_t anym_sign : 1;
    uint32_t tap_first_x : 1;
    uint32_t tap_first_y : 1;
    uint32_t tap_first_z : 1;
    uint32_t tap_sign : 1;
    uint32_t reserved : 2;
    uint32_t high_g : 1;
    uint32_t low_g : 1;
    uint32_t drdy : 1;
    uint32_t ffull : 1;
    uint32_t fwm : 1;
    uint32_t nomo : 1;
    uint32_t step : 1;
    uint32_t sigmot : 1;
    uint32_t anym : 1;

    /* pmu trigger will be handled later */
    uint32_t pmu_trigger_reserved : 1;
    uint32_t d_tap : 1;
    uint32_t s_tap : 1;
    uint32_t orient : 1;
    uint32_t flat_int : 1;
#endif
}bmx160_int_status_bits;

/*!
 * @brief bmi160 interrupt status structure
 */
typedef union
{
    uint8_t data[4];
    bmx160_int_status_bits bit;
}bmx160_int_status;

/*!
 * @brief bmi160 sensor data structure which comprises of accel data
 */
typedef struct
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;

    /*! sensor time */
    uint32_t sensortime;
}bmx160_sensor_data;

/*!
 * @brief bmi160 aux data structure which comprises of 8 bytes of accel data
 */
typedef struct
{
    /*! Auxiliary data */
    uint8_t data[8];
}bmx160_aux_data;

/*!
 * @brief bmi160 FOC configuration structure
 */
typedef struct
{
    /*! Enabling FOC in gyro
     * Assignable macros :
     *  - BMI160_ENABLE
     *  - BMI160_DISABLE
     */
    uint8_t foc_gyr_en;

    /*! Accel FOC configurations
     * Assignable macros :
     *  - BMI160_FOC_ACCEL_DISABLED
     *  - BMI160_FOC_ACCEL_POSITIVE_G
     *  - BMI160_FOC_ACCEL_NEGATIVE_G
     *  - BMI160_FOC_ACCEL_0G
     */
    uint8_t foc_acc_x;
    uint8_t foc_acc_y;
    uint8_t foc_acc_z;

    /*! Enabling offset compensation for accel in data registers
     * Assignable macros :
     *  - BMI160_ENABLE
     *  - BMI160_DISABLE
     */
    uint8_t acc_off_en;

    /*! Enabling offset compensation for gyro in data registers
     * Assignable macros :
     *  - BMI160_ENABLE
     *  - BMI160_DISABLE
     */
    uint8_t gyro_off_en;
}bmx160_foc_conf;

/*!
 * @brief bmi160 accel gyro offsets
 */
typedef struct
{
    /*! Accel offset for x axis */
    int8_t off_acc_x;

    /*! Accel offset for y axis */
    int8_t off_acc_y;

    /*! Accel offset for z axis */
    int8_t off_acc_z;

    /*! Gyro offset for x axis */
    int16_t off_gyro_x;

    /*! Gyro offset for y axis */
    int16_t off_gyro_y;

    /*! Gyro offset for z axis */
    int16_t off_gyro_z;
}bmx160_offsets;

/*!
 * @brief FIFO aux. sensor data structure
 */
typedef struct
{
    /*! The value of aux. sensor x LSB data */
    uint8_t aux_x_lsb;

    /*! The value of aux. sensor x MSB data */
    uint8_t aux_x_msb;

    /*! The value of aux. sensor y LSB data */
    uint8_t aux_y_lsb;

    /*! The value of aux. sensor y MSB data */
    uint8_t aux_y_msb;

    /*! The value of aux. sensor z LSB data */
    uint8_t aux_z_lsb;

    /*! The value of aux. sensor z MSB data */
    uint8_t aux_z_msb;

    /*! The value of aux. sensor r for BMM150 LSB data */
    uint8_t aux_r_y2_lsb;

    /*! The value of aux. sensor r for BMM150 MSB data */
    uint8_t aux_r_y2_msb;
}bmx160_aux_fifo_data;

/*!
 * @brief bmi160 sensor select structure
 */
enum bmx160_select_sensor {
    BMX160_ACCEL_ONLY = 1,
    BMX160_GYRO_ONLY,
    BMX160_BOTH_ACCEL_AND_GYRO
};

/*!
 * @brief bmi160 sensor step detector mode structure
 */
enum bmx160_step_detect_mode {
    BMX160_STEP_DETECT_NORMAL,
    BMX160_STEP_DETECT_SENSITIVE,
    BMX160_STEP_DETECT_ROBUST,

    /*! Non recommended User defined setting */
    BMX160_STEP_DETECT_USER_DEFINE
};

/*!
 * @brief enum for auxiliary burst read selection
 */
enum bmx160_aux_read_len {
    BMX160_AUX_READ_LEN_0,
    BMX160_AUX_READ_LEN_1,
    BMX160_AUX_READ_LEN_2,
    BMX160_AUX_READ_LEN_3
};

/*!
 * @brief bmi160 sensor configuration structure
 */
typedef struct
{
    /*! power mode */
    uint8_t power;

    /*! output data rate */
    uint8_t odr;

    /*! range */
    uint8_t range;

    /*! bandwidth */
    uint8_t bw;
}bmx160_cfg;

/*!
 * @brief Aux sensor configuration structure
 */
typedef struct
{
    /*! Aux sensor, 1 - enable 0 - disable */
    uint8_t aux_sensor_enable : 1;

    /*! Aux manual/auto mode status */
    uint8_t manual_enable : 1;

    /*! Aux read burst length */
    uint8_t aux_rd_burst_len : 2;

    /*! output data rate */
    uint8_t aux_odr : 4;

    /*! i2c addr of auxiliary sensor */
    uint8_t aux_i2c_addr;
}bmx160_aux_cfg;

/*!
 * @brief bmi160 interrupt channel selection structure
 */
enum bmx160_int_channel {
    /*! Un-map both channels */
    BMX160_INT_CHANNEL_NONE,

    /*! interrupt Channel 1 */
    BMX160_INT_CHANNEL_1,

    /*! interrupt Channel 2 */
    BMX160_INT_CHANNEL_2,

    /*! Map both channels */
    BMX160_INT_CHANNEL_BOTH
};
enum bmx160_int_types {
    /*! Slope/Any-motion interrupt */
    BMX160_ACC_ANY_MOTION_INT,

    /*! Significant motion interrupt */
    BMX160_ACC_SIG_MOTION_INT,

    /*! Step detector interrupt */
    BMX160_STEP_DETECT_INT,

    /*! double tap interrupt */
    BMX160_ACC_DOUBLE_TAP_INT,

    /*! single tap interrupt */
    BMX160_ACC_SINGLE_TAP_INT,

    /*! orientation interrupt */
    BMX160_ACC_ORIENT_INT,

    /*! flat interrupt */
    BMX160_ACC_FLAT_INT,

    /*! high-g interrupt */
    BMX160_ACC_HIGH_G_INT,

    /*! low-g interrupt */
    BMX160_ACC_LOW_G_INT,

    /*! slow/no-motion interrupt */
    BMX160_ACC_SLOW_NO_MOTION_INT,

    /*! data ready interrupt  */
    BMX160_ACC_GYRO_DATA_RDY_INT,

    /*! fifo full interrupt */
    BMX160_ACC_GYRO_FIFO_FULL_INT,

    /*! fifo watermark interrupt */
    BMX160_ACC_GYRO_FIFO_WATERMARK_INT,

    /*! fifo tagging feature support */
    BMX160_FIFO_TAG_INT_PIN
};

/*!
 * @brief bmi160 active state of any & sig motion interrupt.
 */
typedef enum {
    /*! Both any & sig motion are disabled */
    BMX160_BOTH_ANY_SIG_MOTION_DISABLED = -1,

    /*! Any-motion selected */
    BMX160_ANY_MOTION_ENABLED,

    /*! Sig-motion selected */
    BMX160_SIG_MOTION_ENABLED
}bmx160_any_sig_motion_active_interrupt_state;
typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! tap threshold */
    uint16_t tap_thr : 5;

    /*! tap shock */
    uint16_t tap_shock : 1;

    /*! tap quiet */
    uint16_t tap_quiet : 1;

    /*! tap duration */
    uint16_t tap_dur : 3;

    /*! data source 0- filter & 1 pre-filter*/
    uint16_t tap_data_src : 1;

    /*! tap enable, 1 - enable, 0 - disable */
    uint16_t tap_en : 1;
#elif BIG_ENDIAN == 1

    /*! tap enable, 1 - enable, 0 - disable */
    uint16_t tap_en : 1;

    /*! data source 0- filter & 1 pre-filter*/
    uint16_t tap_data_src : 1;

    /*! tap duration */
    uint16_t tap_dur : 3;

    /*! tap quiet */
    uint16_t tap_quiet : 1;

    /*! tap shock */
    uint16_t tap_shock : 1;

    /*! tap threshold */
    uint16_t tap_thr : 5;
#endif
}bmx160_acc_tap_int_cfg;

typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! 1 any-motion enable, 0 - any-motion disable */
    uint8_t anymotion_en : 1;

    /*! slope interrupt x, 1 - enable, 0 - disable */
    uint8_t anymotion_x : 1;

    /*! slope interrupt y, 1 - enable, 0 - disable */
    uint8_t anymotion_y : 1;

    /*! slope interrupt z, 1 - enable, 0 - disable */
    uint8_t anymotion_z : 1;

    /*! slope duration */
    uint8_t anymotion_dur : 2;

    /*! data source 0- filter & 1 pre-filter*/
    uint8_t anymotion_data_src : 1;

    /*! slope threshold */
    uint8_t anymotion_thr;
#elif BIG_ENDIAN == 1

    /*! slope threshold */
    uint8_t anymotion_thr;

    /*! data source 0- filter & 1 pre-filter*/
    uint8_t anymotion_data_src : 1;

    /*! slope duration */
    uint8_t anymotion_dur : 2;

    /*! slope interrupt z, 1 - enable, 0 - disable */
    uint8_t anymotion_z : 1;

    /*! slope interrupt y, 1 - enable, 0 - disable */
    uint8_t anymotion_y : 1;

    /*! slope interrupt x, 1 - enable, 0 - disable */
    uint8_t anymotion_x : 1;

    /*! 1 any-motion enable, 0 - any-motion disable */
    uint8_t anymotion_en : 1;
#endif
}bmx160_acc_any_mot_int_cfg;

typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! skip time of sig-motion interrupt */
    uint8_t sig_mot_skip : 2;

    /*! proof time of sig-motion interrupt */
    uint8_t sig_mot_proof : 2;

    /*! data source 0- filter & 1 pre-filter*/
    uint8_t sig_data_src : 1;

    /*! 1 - enable sig, 0 - disable sig & enable anymotion */
    uint8_t sig_en : 1;

    /*! sig-motion threshold */
    uint8_t sig_mot_thres;
#elif BIG_ENDIAN == 1

    /*! sig-motion threshold */
    uint8_t sig_mot_thres;

    /*! 1 - enable sig, 0 - disable sig & enable anymotion */
    uint8_t sig_en : 1;

    /*! data source 0- filter & 1 pre-filter*/
    uint8_t sig_data_src : 1;

    /*! proof time of sig-motion interrupt */
    uint8_t sig_mot_proof : 2;

    /*! skip time of sig-motion interrupt */
    uint8_t sig_mot_skip : 2;
#endif
}bmx160_acc_sig_mot_int_cfg;
typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! 1- step detector enable, 0- step detector disable */
    uint16_t step_detector_en : 1;

    /*! minimum threshold */
    uint16_t min_threshold : 2;

    /*! minimal detectable step time */
    uint16_t steptime_min : 3;

    /*! enable step counter mode setting */
    uint16_t step_detector_mode : 2;

    /*! minimum step buffer size*/
    uint16_t step_min_buf : 3;
#elif BIG_ENDIAN == 1

    /*! minimum step buffer size*/
    uint16_t step_min_buf : 3;

    /*! enable step counter mode setting */
    uint16_t step_detector_mode : 2;

    /*! minimal detectable step time */
    uint16_t steptime_min : 3;

    /*! minimum threshold */
    uint16_t min_threshold : 2;

    /*! 1- step detector enable, 0- step detector disable */
    uint16_t step_detector_en : 1;
#endif
}bmx160_acc_step_detect_int_cfg;
typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! no motion interrupt x */
    uint16_t no_motion_x : 1;

    /*! no motion interrupt y */
    uint16_t no_motion_y : 1;

    /*! no motion interrupt z */
    uint16_t no_motion_z : 1;

    /*! no motion duration */
    uint16_t no_motion_dur : 6;

    /*! no motion sel , 1 - enable no-motion ,0- enable slow-motion */
    uint16_t no_motion_sel : 1;

    /*! data source 0- filter & 1 pre-filter*/
    uint16_t no_motion_src : 1;

    /*! no motion threshold */
    uint8_t no_motion_thres;
#elif BIG_ENDIAN == 1

    /*! no motion threshold */
    uint8_t no_motion_thres;

    /*! data source 0- filter & 1 pre-filter*/
    uint16_t no_motion_src : 1;

    /*! no motion sel , 1 - enable no-motion ,0- enable slow-motion */
    uint16_t no_motion_sel : 1;

    /*! no motion duration */
    uint16_t no_motion_dur : 6;

    /* no motion interrupt z */
    uint16_t no_motion_z : 1;

    /*! no motion interrupt y */
    uint16_t no_motion_y : 1;

    /*! no motion interrupt x */
    uint16_t no_motion_x : 1;
#endif
}bmx160_acc_no_motion_int_cfg;
typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! thresholds for switching between the different orientations */
    uint16_t orient_mode : 2;

    /*! blocking_mode */
    uint16_t orient_blocking : 2;

    /*! Orientation interrupt hysteresis */
    uint16_t orient_hyst : 4;

    /*! Orientation interrupt theta */
    uint16_t orient_theta : 6;

    /*! Enable/disable Orientation interrupt */
    uint16_t orient_ud_en : 1;

    /*! exchange x- and z-axis in algorithm ,0 - z, 1 - x */
    uint16_t axes_ex : 1;

    /*! 1 - orient enable, 0 - orient disable */
    uint8_t orient_en : 1;
#elif BIG_ENDIAN == 1

    /*! 1 - orient enable, 0 - orient disable */
    uint8_t orient_en : 1;

    /*! exchange x- and z-axis in algorithm ,0 - z, 1 - x */
    uint16_t axes_ex : 1;

    /*! Enable/disable Orientation interrupt */
    uint16_t orient_ud_en : 1;

    /*! Orientation interrupt theta */
    uint16_t orient_theta : 6;

    /*! Orientation interrupt hysteresis */
    uint16_t orient_hyst : 4;

    /*! blocking_mode */
    uint16_t orient_blocking : 2;

    /*! thresholds for switching between the different orientations */
    uint16_t orient_mode : 2;
#endif
}bmx160_acc_orient_int_cfg;
typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! flat threshold */
    uint16_t flat_theta : 6;

    /*! flat interrupt hysteresis */
    uint16_t flat_hy : 3;

    /*! delay time for which the flat value must remain stable for the
     * flat interrupt to be generated */
    uint16_t flat_hold_time : 2;

    /*! 1 - flat enable, 0 - flat disable */
    uint16_t flat_en : 1;
#elif BIG_ENDIAN == 1

    /*! 1 - flat enable, 0 - flat disable */
    uint16_t flat_en : 1;

    /*! delay time for which the flat value must remain stable for the
     * flat interrupt to be generated */
    uint16_t flat_hold_time : 2;

    /*! flat interrupt hysteresis */
    uint16_t flat_hy : 3;

    /*! flat threshold */
    uint16_t flat_theta : 6;
#endif
}bmx160_acc_flat_detect_int_cfg;
typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! low-g interrupt trigger delay */
    uint8_t low_dur;

    /*! low-g interrupt trigger threshold */
    uint8_t low_thres;

    /*! hysteresis of low-g interrupt */
    uint8_t low_hyst : 2;

    /*! 0 - single-axis mode ,1 - axis-summing mode */
    uint8_t low_mode : 1;

    /*! data source 0- filter & 1 pre-filter */
    uint8_t low_data_src : 1;

    /*! 1 - enable low-g, 0 - disable low-g */
    uint8_t low_en : 1;
#elif BIG_ENDIAN == 1

    /*! 1 - enable low-g, 0 - disable low-g */
    uint8_t low_en : 1;

    /*! data source 0- filter & 1 pre-filter */
    uint8_t low_data_src : 1;

    /*! 0 - single-axis mode ,1 - axis-summing mode */
    uint8_t low_mode : 1;

    /*! hysteresis of low-g interrupt */
    uint8_t low_hyst : 2;

    /*! low-g interrupt trigger threshold */
    uint8_t low_thres;

    /*! low-g interrupt trigger delay */
    uint8_t low_dur;
#endif
}bmx160_acc_low_g_int_cfg;
typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! High-g interrupt x, 1 - enable, 0 - disable */
    uint8_t high_g_x : 1;

    /*! High-g interrupt y, 1 - enable, 0 - disable */
    uint8_t high_g_y : 1;

    /*! High-g interrupt z, 1 - enable, 0 - disable */
    uint8_t high_g_z : 1;

    /*! High-g hysteresis  */
    uint8_t high_hy : 2;

    /*! data source 0- filter & 1 pre-filter */
    uint8_t high_data_src : 1;

    /*! High-g threshold */
    uint8_t high_thres;

    /*! High-g duration */
    uint8_t high_dur;
#elif BIG_ENDIAN == 1

    /*! High-g duration */
    uint8_t high_dur;

    /*! High-g threshold */
    uint8_t high_thres;

    /*! data source 0- filter & 1 pre-filter */
    uint8_t high_data_src : 1;

    /*! High-g hysteresis  */
    uint8_t high_hy : 2;

    /*! High-g interrupt z, 1 - enable, 0 - disable */
    uint8_t high_g_z : 1;

    /*! High-g interrupt y, 1 - enable, 0 - disable */
    uint8_t high_g_y : 1;

    /*! High-g interrupt x, 1 - enable, 0 - disable */
    uint8_t high_g_x : 1;
#endif
}bmx160_acc_high_g_int_cfg;
typedef struct
{
#if LITTLE_ENDIAN == 1

    /*! To enable either INT1 or INT2 pin as output.
     * 0- output disabled ,1- output enabled */
    uint16_t output_en : 1;

    /*! 0 - push-pull 1- open drain,only valid if output_en is set 1 */
    uint16_t output_mode : 1;

    /*! 0 - active low , 1 - active high level.
     * if output_en is 1,this applies to interrupts,else PMU_trigger */
    uint16_t output_type : 1;

    /*! 0 - level trigger , 1 - edge trigger  */
    uint16_t edge_ctrl : 1;

    /*! To enable either INT1 or INT2 pin as input.
     * 0 - input disabled ,1 - input enabled */
    uint16_t input_en : 1;

    /*! latch duration*/
    uint16_t latch_dur : 4;
#elif BIG_ENDIAN == 1

    /*! latch duration*/
    uint16_t latch_dur : 4;

    /*! Latched,non-latched or temporary interrupt modes */
    uint16_t input_en : 1;

    /*! 1 - edge trigger, 0 - level trigger */
    uint16_t edge_ctrl : 1;

    /*! 0 - active low , 1 - active high level.
     * if output_en is 1,this applies to interrupts,else PMU_trigger */
    uint16_t output_type : 1;

    /*! 0 - push-pull , 1 - open drain,only valid if output_en is set 1 */
    uint16_t output_mode : 1;

    /*! To enable either INT1 or INT2 pin as output.
     * 0 - output disabled , 1 - output enabled */
    uint16_t output_en : 1;
#endif
}bmx160_int_pin_settg;

typedef union
{
    /*! Tap interrupt structure */
    bmx160_acc_tap_int_cfg acc_tap_int;

    /*! Slope interrupt structure */
    bmx160_acc_any_mot_int_cfg acc_any_motion_int;

    /*! Significant motion interrupt structure */
    bmx160_acc_sig_mot_int_cfg acc_sig_motion_int;

    /*! Step detector interrupt structure */
    bmx160_acc_step_detect_int_cfg acc_step_detect_int;

    /*! No motion interrupt structure */
    bmx160_acc_no_motion_int_cfg acc_no_motion_int;

    /*! Orientation interrupt structure */
    bmx160_acc_orient_int_cfg acc_orient_int;

    /*! Flat interrupt structure */
    bmx160_acc_flat_detect_int_cfg acc_flat_int;

    /*! Low-g interrupt structure */
    bmx160_acc_low_g_int_cfg acc_low_g_int;

    /*! High-g interrupt structure */
    bmx160_acc_high_g_int_cfg acc_high_g_int;
}bmx160_int_type_cfg;

typedef struct
{
    /*! Interrupt channel */
    enum bmx160_int_channel int_channel;

    /*! Select Interrupt */
    enum bmx160_int_types int_type;

    /*! Structure configuring Interrupt pins */
    bmx160_int_pin_settg int_pin_settg;

    /*! Union configures required interrupt */
    bmx160_int_type_cfg int_type_cfg;

    /*! FIFO FULL INT 1-enable, 0-disable */
    uint8_t fifo_full_int_en : 1;

    /*! FIFO WTM INT 1-enable, 0-disable */
    uint8_t fifo_wtm_int_en : 1;
}bmx160_int_settg;

/*!
 *  @brief This structure holds the information for usage of
 *  FIFO by the user.
 */
typedef struct
{
    /*! Data buffer of user defined length is to be mapped here */
    uint8_t *data;

    /*! While calling the API  "bmi160_get_fifo_data" , length stores
     *  number of bytes in FIFO to be read (specified by user as input)
     *  and after execution of the API ,number of FIFO data bytes
     *  available is provided as an output to user
     */
    uint16_t length;

    /*! FIFO time enable */
    uint8_t fifo_time_enable;

    /*! Enabling of the FIFO header to stream in header mode */
    uint8_t fifo_header_enable;

    /*! Streaming of the Accelerometer, Gyroscope
     * sensor data or both in FIFO */
    uint8_t fifo_data_enable;

    /*! Will be equal to length when no more frames are there to parse */
    uint16_t accel_byte_start_idx;

    /*! Will be equal to length when no more frames are there to parse */
    uint16_t gyro_byte_start_idx;

    /*! Will be equal to length when no more frames are there to parse */
    uint16_t aux_byte_start_idx;

    /*! Value of FIFO sensor time time */
    uint32_t sensor_time;

    /*! Value of Skipped frame counts */
    uint8_t skipped_frame_count;
}bmx160_fifo_frame;


/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*bmx160_ctx_write)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*bmx160_ctx_read) (void *, uint8_t, uint8_t*, uint16_t);
typedef void (*bmx160_ctx_delay)(void *, uint32_t);

typedef struct {
  /** Component mandatory fields **/
  bmx160_ctx_write  write_reg;
  bmx160_ctx_read   read_reg;
  bmx160_ctx_delay	delay_ms;
  uint8_t 			mode;
  /** Customizable optional pointer **/
  void *handle;
} bmx160_ctx_t;

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
int32_t bmx160_read_reg(bmx160_ctx_t *ctx, uint8_t reg_addr, uint8_t *data, uint16_t len);

/*!
 * @brief This API writes the given data to the register address
 * of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be written.
 * @param[in] data      : Pointer to data buffer which is to be written in the sensor.
 * @param[in] len       : No of bytes of data to write..
 * @param[in] ctx       : Interface through which the operation will be performed.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int32_t bmx160_write_reg(bmx160_ctx_t *ctx, uint8_t reg_addr, uint8_t *data, uint16_t len, uint8_t mode);
void bmx160_delay_ms(bmx160_ctx_t *ctx, uint32_t period);

int32_t set_intr_pin_config(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config);
int32_t config_int_out_ctrl(bmx160_ctx_t *ctx, const bmx160_int_settg *int_config);
int32_t config_int_latch (bmx160_ctx_t* ctx, const bmx160_int_settg *int_config);
int32_t disable_sig_motion_int(bmx160_ctx_t* ctx);
int32_t map_feature_interrupt(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config);
int32_t map_hardware_interrupt(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config);
int32_t config_any_dur_threshold(bmx160_ctx_t* ctx, const bmx160_acc_any_mot_int_cfg *any_motion_int_cfg);
int32_t enable_tap_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_tap_int_cfg *tap_int_cfg);
int32_t config_tap_data_src(bmx160_ctx_t* ctx, const bmx160_acc_tap_int_cfg *tap_int_cfg);
int32_t config_tap_param(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_tap_int_cfg *tap_int_cfg);
int32_t enable_data_ready_int(bmx160_ctx_t* ctx);
int32_t config_sig_motion_data_src(bmx160_ctx_t* ctx, const bmx160_acc_sig_mot_int_cfg *sig_mot_int_cfg);
int32_t config_sig_dur_threshold(bmx160_ctx_t* ctx, const bmx160_acc_sig_mot_int_cfg *sig_mot_int_cfg);
int32_t config_no_motion_data_src(bmx160_ctx_t* ctx, const bmx160_acc_no_motion_int_cfg *no_mot_int_cfg);
int32_t config_no_motion_dur_thr(bmx160_ctx_t* ctx, const bmx160_acc_no_motion_int_cfg *no_mot_int_cfg);
int32_t config_no_motion_int_settg(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_no_motion_int_cfg *no_mot_int_cfg);
int32_t enable_step_detect_int(bmx160_ctx_t* ctx, const bmx160_acc_step_detect_int_cfg *step_detect_int_cfg);
int32_t config_step_detect(bmx160_ctx_t* ctx, const bmx160_acc_step_detect_int_cfg *step_detect_int_cfg);
int32_t config_orient_int_settg(bmx160_ctx_t* ctx, const bmx160_acc_orient_int_cfg *orient_int_cfg);
int32_t config_flat_int_settg(bmx160_ctx_t* ctx, const bmx160_acc_flat_detect_int_cfg *flat_int);
int32_t config_sec_if(bmx160_ctx_t* ctx);
int32_t config_any_motion_src(bmx160_ctx_t* ctx, const bmx160_acc_any_mot_int_cfg *any_motion_int_cfg);
int32_t config_sig_motion_int_settg(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_sig_mot_int_cfg *sig_mot_int_cfg);
int32_t config_any_motion_int_settg(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_any_mot_int_cfg *any_motion_int_cfg);
int32_t config_tap_int_settg(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config, const bmx160_acc_tap_int_cfg *tap_int_cfg);
int32_t enable_no_motion_int(bmx160_ctx_t* ctx, const bmx160_acc_no_motion_int_cfg *no_mot_int_cfg);
int32_t enable_flat_int(bmx160_ctx_t* ctx, const bmx160_acc_flat_detect_int_cfg *flat_int);
int32_t enable_orient_int(bmx160_ctx_t* ctx, const bmx160_acc_orient_int_cfg *orient_int_cfg);
int32_t enable_low_g_int(bmx160_ctx_t* ctx, const bmx160_acc_low_g_int_cfg *low_g_int);
int32_t config_low_g_data_src(bmx160_ctx_t* ctx, const bmx160_acc_low_g_int_cfg *low_g_int);
int32_t config_low_g_int_settg(bmx160_ctx_t* ctx, const bmx160_acc_low_g_int_cfg *low_g_int);
int32_t enable_high_g_int(bmx160_ctx_t* ctx, const bmx160_acc_high_g_int_cfg *high_g_int_cfg);
int32_t config_high_g_data_src(bmx160_ctx_t* ctx, const bmx160_acc_high_g_int_cfg *high_g_int_cfg);
int32_t config_high_g_int_settg(bmx160_ctx_t* ctx, const bmx160_acc_high_g_int_cfg *high_g_int_cfg);
int32_t enable_fifo_full_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config);
int32_t enable_fifo_wtm_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config);
int32_t set_fifo_watermark_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config);
int32_t set_fifo_full_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config);
int32_t get_fifo_byte_counter(bmx160_ctx_t* ctx, uint16_t *bytes_to_read);
int32_t configure_offset_enable(bmx160_ctx_t* ctx, const bmx160_foc_conf *foc_conf);
int32_t get_foc_status(bmx160_ctx_t* ctx, uint8_t *foc_status);
int32_t set_accel_tap_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config);
int32_t get_accel_data(bmx160_ctx_t* ctx, uint8_t len, bmx160_sensor_data *accel);
int32_t get_gyro_data(bmx160_ctx_t* ctx, uint8_t len, bmx160_sensor_data *gyro);
int32_t get_accel_gyro_data(bmx160_ctx_t* ctx, uint8_t len, bmx160_sensor_data *accel, bmx160_sensor_data *gyro);
int32_t set_accel_gyro_data_ready_int(bmx160_ctx_t* ctx, const bmx160_int_settg *int_config);
int32_t set_accel_no_motion_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config);
int32_t set_accel_step_detect_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config);
int32_t set_accel_orientation_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config);
int32_t set_accel_tap_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config);
int32_t set_accel_flat_detect_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config);
int32_t set_accel_low_g_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config);
int32_t set_accel_high_g_int(bmx160_ctx_t* ctx, bmx160_int_settg *int_config);
int32_t extract_aux_read(bmx160_ctx_t* ctx, uint16_t map_len, uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int32_t enable_gyro_self_test(bmx160_ctx_t* ctx);
int32_t validate_gyro_self_test(bmx160_ctx_t* ctx);
int32_t validate_accel_self_test(const bmx160_sensor_data *accel_pos, const bmx160_sensor_data *accel_neg);

#endif /* INC_BMX160_REG_H_ */
