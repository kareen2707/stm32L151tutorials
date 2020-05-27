/*
 * bmx160.c
 *
 *  Created on: 17 march. 2020
 *      Author: karen@b105.upm.es
 */


/*!
 * @defgroup bmx160
 * @brief
 * @{*/

#include "bmx160.h"

/*********************************************************************/

//static int8_t set_intr_pin_config(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);

/*!
 * @brief This API sets the any-motion interrupt of the sensor.
 * This interrupt occurs when accel values exceeds preset threshold
 * for a certain period of time.
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t set_accel_any_motion_int(bmi160_int_settg *int_config, struct bmi160_dev *dev);
static int32_t set_accel_any_motion_int(BMX160_Object_t *pObj, bmx160_int_settg *int_config);

//static int8_t set_accel_tap_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev);
//static int8_t set_accel_gyro_data_ready_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);

/*!
 * @brief This API sets the significant motion interrupt of the sensor.This
 * interrupt occurs when there is change in user location.
 *
 * @param[in] int_config  : Structure instance of bmi160_int_settg.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t set_accel_sig_motion_int(bmi160_int_settg *int_config, struct bmi160_dev *dev);
static int32_t set_accel_sig_motion_int(BMX160_Object_t *pObj, bmx160_int_settg *int_config);

//static int8_t set_accel_no_motion_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev);
//static int8_t set_accel_step_detect_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev);
//static int8_t set_accel_orientation_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev);
//static int8_t set_accel_flat_detect_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev);

/*!
 * @brief This API sets the default configuration parameters of accel & gyro.
 * Also maintain the previous state of configurations.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static void default_param_settg(BMX160_Object_t *pObj);

/*!
 * @brief This API set the accel configuration.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t set_accel_conf(struct bmi160_dev *dev);
static int32_t set_accel_conf(BMX160_Object_t *pObj);

/*!
 * @brief This API check the accel configuration.
 *
 * @param[in] data        : Pointer to store the updated accel config.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t check_accel_config(uint8_t *data, const struct bmi160_dev *dev);
static int32_t check_accel_config(BMX160_Object_t *pObj, uint8_t *data);
/*!
 * @brief This API process the accel odr.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t process_accel_odr(uint8_t *data, const struct bmi160_dev *dev);
static int32_t process_accel_odr(BMX160_Object_t *pObj, uint8_t *data);
/*!
 * @brief This API process the accel bandwidth.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t process_accel_bw(uint8_t *data, const struct bmi160_dev *dev);
static int32_t process_accel_bw(BMX160_Object_t *pObj, uint8_t *data);

/*!
 * @brief This API process the accel range.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t process_accel_range(uint8_t *data, const struct bmi160_dev *dev);
static int32_t process_accel_range(BMX160_Object_t *pObj, uint8_t *data);
/*!
 * @brief This API checks the invalid settings for ODR & Bw for Accel and Gyro.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t check_invalid_settg(const struct bmi160_dev *dev);
static int32_t check_invalid_settg(BMX160_Object_t *pObj);
/*!
 * @brief This API set the gyro configuration.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t set_gyro_conf(struct bmi160_dev *dev);
static int32_t set_gyro_conf(BMX160_Object_t *pObj);

/*!
 * @brief This API check the gyro configuration.
 *
 * @param[in] data        : Pointer to store the updated gyro config.
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t check_gyro_config(uint8_t *data, const struct bmi160_dev *dev);
static int32_t check_gyro_config(BMX160_Object_t *pObj, uint8_t *data);
/*!
 * @brief This API process the gyro odr.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t process_gyro_odr(uint8_t *data, const struct bmi160_dev *dev);
static int32_t process_gyro_odr(BMX160_Object_t *pObj, uint8_t *data);

/*!
 * @brief This API process the gyro bandwidth.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t process_gyro_bw(uint8_t *data, const struct bmi160_dev *dev);
static int32_t process_gyro_bw(BMX160_Object_t *pObj, uint8_t *data);
/*!
 * @brief This API process the gyro range.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t process_gyro_range(uint8_t *data, const struct bmi160_dev *dev);
static int32_t process_gyro_range(BMX160_Object_t *pObj, uint8_t *data);

/*!
 * @brief This API sets the accel power mode.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t set_accel_pwr(struct bmi160_dev *dev);
static int32_t set_accel_pwr(BMX160_Object_t *pObj);
/*!
 * @brief This API process the undersampling setting of Accel.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
//static int8_t process_under_sampling(uint8_t *data, const struct bmi160_dev *dev);
static int32_t process_under_sampling(BMX160_Object_t *pObj, uint8_t *data);
/*!
 * @brief This API sets the gyro power mode.
 *
 * @param[in] dev         : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int32_t set_gyro_pwr(BMX160_Object_t *pObj);

//static int8_t get_accel_data(uint8_t len, bmi160_sensor_data *accel, const struct bmi160_dev *dev);
//static int8_t get_gyro_data(uint8_t len, bmi160_sensor_data *gyro, const struct bmi160_dev *dev);

/*!
 * @brief This API reads accel and gyro data along with sensor time
 * if time is requested by user.
 * Kindly refer the user guide(README.md) for more info.
 *
 * @param[in] len    : len to read no of bytes
 * @param[out] accel    : Structure pointer to store accel data
 * @param[out] gyro    : Structure pointer to store accel data
 * @param[in] dev       : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static int8_t get_accel_gyro_data(uint8_t len, bmi160_sensor_data *accel, bmi160_sensor_data *gyro, const struct bmi160_dev *dev);
/*!
 * @brief This API enables the any-motion interrupt for accel.
 *
 * @param[in] any_motion_int_cfg   : Structure instance of
 *                   bmi160_acc_any_mot_int_cfg.
 * @param[in] dev          : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static int8_t enable_accel_any_motion_int(const bmi160_acc_any_mot_int_cfg *any_motion_int_cfg, struct bmi160_dev *dev);
static int32_t enable_accel_any_motion_int(BMX160_Object_t *pObj, const bmx160_acc_any_mot_int_cfg *any_motion_int_cfg);

//static int8_t disable_sig_motion_int(const struct bmi160_dev *dev);
//static int8_t config_any_motion_src(const bmi160_acc_any_mot_int_cfg *any_motion_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_any_dur_threshold(const bmi160_acc_any_mot_int_cfg *any_motion_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_any_motion_int_settg(const bmi160_int_settg *int_config, const bmi160_acc_any_mot_int_cfg *any_motion_int_cfg, const struct bmi160_dev *dev);
//static int8_t enable_data_ready_int(const struct bmi160_dev *dev);
//static int8_t enable_no_motion_int(const bmi160_acc_no_motion_int_cfg *no_mot_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_no_motion_int_settg(const bmi160_int_settg *int_config, const bmi160_acc_no_motion_int_cfg *no_mot_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_no_motion_data_src(const bmi160_acc_no_motion_int_cfg *no_mot_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_no_motion_dur_thr(const bmi160_acc_no_motion_int_cfg *no_mot_int_cfg, const struct bmi160_dev *dev);
/*!
 * @brief This API enables the sig-motion motion interrupt.
 *
 * @param[in] sig_mot_int_cfg   : Structure instance of
 *                bmi160_acc_sig_mot_int_cfg.
 * @param[in] dev       : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static int8_t enable_sig_motion_int(const bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg, struct bmi160_dev *dev);
static int32_t enable_sig_motion_int(BMX160_Object_t *pObj, const bmx160_acc_sig_mot_int_cfg *sig_mot_int_cfg);

//static int8_t config_sig_motion_int_settg(const bmi160_int_settg *int_config, const bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_sig_motion_data_src(const bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_sig_dur_threshold(const bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg, const struct bmi160_dev *dev);
//static int8_t enable_step_detect_int(const bmi160_acc_step_detect_int_cfg *step_detect_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_step_detect(const bmi160_acc_step_detect_int_cfg *step_detect_int_cfg, const struct bmi160_dev *dev);
//static int8_t enable_tap_int(const bmi160_int_settg *int_config, const bmi160_acc_tap_int_cfg *tap_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_tap_int_settg(const bmi160_int_settg *int_config, const bmi160_acc_tap_int_cfg *tap_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_tap_data_src(const bmi160_acc_tap_int_cfg *tap_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_tap_param(const bmi160_int_settg *int_config, const bmi160_acc_tap_int_cfg *tap_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_sec_if(const struct bmi160_dev *dev);

/*!
 * @brief This API configure the ODR of the auxiliary sensor.
 *
 * @param[in] dev   : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static int8_t config_aux_odr(const struct bmi160_dev *dev);
static int32_t config_aux_odr(BMX160_Object_t *pObj);
/*!
 * @brief This API maps the actual burst read length set by user.
 *
 * @param[in] len   : Pointer to store the read length.
 * @param[in] dev   : Structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static int8_t map_read_len(uint16_t *len, const struct bmi160_dev *dev);
static int32_t map_read_len(BMX160_Object_t *pObj, uint16_t *len);

static int32_t config_aux_settg(BMX160_Object_t *pObj);

//static int8_t enable_orient_int(const bmi160_acc_orient_int_cfg *orient_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_orient_int_settg(const bmi160_acc_orient_int_cfg *orient_int_cfg, const struct bmi160_dev *dev);
//static int8_t enable_flat_int(const bmi160_acc_flat_detect_int_cfg *flat_int, const struct bmi160_dev *dev);
//static int8_t config_flat_int_settg(const bmi160_acc_flat_detect_int_cfg *flat_int, const struct bmi160_dev *dev);
//static int8_t enable_low_g_int(const bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev);
//static int8_t config_low_g_data_src(const bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev);
//static int8_t config_low_g_int_settg(const bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev);
//static int8_t enable_high_g_int(const bmi160_acc_high_g_int_cfg *high_g_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_high_g_data_src(const bmi160_acc_high_g_int_cfg *high_g_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_high_g_int_settg(const bmi160_acc_high_g_int_cfg *high_g_int_cfg, const struct bmi160_dev *dev);
//static int8_t config_int_out_ctrl(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);
//static int8_t config_int_latch(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);

/*!
 * @brief This API performs the self test for accelerometer of BMI160
 *
 * @param[in] dev   : structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static int8_t perform_accel_self_test(struct bmi160_dev *dev);
static int32_t perform_accel_self_test(BMX160_Object_t *pObj);
/*!
 * @brief This API enables to perform the accel self test by setting proper
 * configurations to facilitate accel self test
 *
 * @param[in] dev   : structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int32_t enable_accel_self_test(BMX160_Object_t *pObj);

/*!
 * @brief This API performs accel self test with positive excitation
 *
 * @param[in] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] dev   : structure instance of bmi160_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static int8_t accel_self_test_positive_excitation(bmi160_sensor_data *accel_pos, const struct bmi160_dev *dev);
static int32_t accel_self_test_positive_excitation(BMX160_Object_t *pObj, bmx160_sensor_data *accel_pos);
/*!
 * @brief This API performs accel self test with negative excitation
 *
 * @param[in] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 * @param[in] dev   : structure instance of bmi160_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static int8_t accel_self_test_negative_excitation(bmi160_sensor_data *accel_neg, const struct bmi160_dev *dev);
static int32_t accel_self_test_negative_excitation(BMX160_Object_t *pObj, bmx160_sensor_data *accel_neg);

//static int8_t validate_accel_self_test(const bmi160_sensor_data *accel_pos, const bmi160_sensor_data *accel_neg);

/*!
 * @brief This API performs the self test for gyroscope of BMI160
 *
 * @param[in] dev   : structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static int8_t perform_gyro_self_test(const struct bmi160_dev *dev);
static int32_t perform_gyro_self_test(BMX160_Object_t *pObj);

//static int8_t enable_gyro_self_test(const struct bmi160_dev *dev);
//static int8_t validate_gyro_self_test(const struct bmi160_dev *dev);
//static int8_t set_fifo_full_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);
//static int8_t enable_fifo_full_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);
//static int8_t set_fifo_watermark_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);
//static int8_t enable_fifo_wtm_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);

/*!
 * @brief This API is used to reset the FIFO related configurations
 *  in the fifo_frame structure.
 *
 * @param[in] dev       : structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static void reset_fifo_data_structure(const struct bmi160_dev *dev);
static void reset_fifo_data_structure(BMX160_Object_t *pObj);

//static int8_t get_fifo_byte_counter(uint16_t *bytes_to_read, struct bmi160_dev const *dev);

/*!
 *  @brief This API is used to compute the number of bytes of accel FIFO data
 *  which is to be parsed in header-less mode
 *
 *  @param[out] data_index        : The start index for parsing data
 *  @param[out] data_read_length  : Number of bytes to be parsed
 *  @param[in]  acc_frame_count   : Number of accelerometer frames to be read
 *  @param[in]  dev               : Structure instance of bmi160_dev.
 *
 */
//static void get_accel_len_to_parse(uint16_t *data_index, uint16_t *data_read_length, const uint8_t *acc_frame_count, const struct bmi160_dev *dev);
static void get_accel_len_to_parse(BMX160_Object_t *pObj, uint16_t *data_index, uint16_t *data_read_length, const uint8_t *acc_frame_count);
/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 *
 *  @param[in,out] acc      : structure instance of sensor data
 *  @param[in,out] idx      : Index value of number of bytes parsed
 *  @param[in,out] acc_idx  : Index value of accelerometer data
 *                                (x,y,z axes) frames parsed
 *  @param[in] frame_info       : It consists of either fifo_data_enable
 *                                parameter in header-less mode or
 *                                frame header data in header mode
 *  @param[in] dev      : structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void unpack_accel_frame(bmi160_sensor_data *acc, uint16_t *idx, uint8_t *acc_idx, uint8_t frame_info, const struct bmi160_dev *dev);
static void unpack_accel_frame(BMX160_Object_t *pObj, bmx160_sensor_data *acc, uint16_t *idx, uint8_t *acc_idx, uint8_t frame_info);
/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data and store it in the instance of the structure bmi160_sensor_data.
 *
 * @param[in,out] accel_data        : structure instance of sensor data
 * @param[in,out] data_start_index  : Index value of number of bytes parsed
 * @param[in] dev           : structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static void unpack_accel_data(bmi160_sensor_data *accel_data, uint16_t data_start_index, const struct bmi160_dev *dev);
static void unpack_accel_data(BMX160_Object_t *pObj, bmx160_sensor_data *accel_data, uint16_t data_start_index);

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in header mode.
 *
 *  @param[in,out] accel_data    : Structure instance of sensor data
 *  @param[in,out] accel_length  : Number of accelerometer frames
 *  @param[in] dev               : Structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void extract_accel_header_mode(bmi160_sensor_data *accel_data, uint8_t *accel_length, const struct bmi160_dev *dev);
static void extract_accel_header_mode(BMX160_Object_t *pObj, bmx160_sensor_data *accel_data, uint8_t *accel_length);

/*!
 *  @brief This API computes the number of bytes of gyro FIFO data
 *  which is to be parsed in header-less mode
 *
 *  @param[out] data_index       : The start index for parsing data
 *  @param[out] data_read_length : No of bytes to be parsed from FIFO buffer
 *  @param[in] gyro_frame_count  : Number of Gyro data frames to be read
 *  @param[in] dev               : Structure instance of bmi160_dev.
 */
//static void get_gyro_len_to_parse(uint16_t *data_index,  uint16_t *data_read_length, const uint8_t *gyro_frame_count, const struct bmi160_dev *dev);
static void get_gyro_len_to_parse(BMX160_Object_t *pObj, uint16_t *data_index,  uint16_t *data_read_length, const uint8_t *gyro_frame_count);
/*!
 *  @brief This API is used to parse the gyroscope's data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 *
 *  @param[in,out] gyro     : structure instance of sensor data
 *  @param[in,out] idx      : Index value of number of bytes parsed
 *  @param[in,out] gyro_idx : Index value of gyro data
 *                                (x,y,z axes) frames parsed
 *  @param[in] frame_info       : It consists of either fifo_data_enable
 *                                parameter in header-less mode or
 *                                frame header data in header mode
 *  @param[in] dev      : structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void unpack_gyro_frame(bmi160_sensor_data *gyro, uint16_t *idx, uint8_t *gyro_idx, uint8_t frame_info, const struct bmi160_dev *dev);
static void unpack_gyro_frame(BMX160_Object_t *pObj, bmx160_sensor_data *gyro, uint16_t *idx, uint8_t *gyro_idx, uint8_t frame_info);

/*!
 *  @brief This API is used to parse the gyro data from the
 *  FIFO data and store it in the instance of the structure bmi160_sensor_data.
 *
 *  @param[in,out] gyro_data         : structure instance of sensor data
 *  @param[in,out] data_start_index  : Index value of number of bytes parsed
 *  @param[in] dev           : structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void unpack_gyro_data(bmi160_sensor_data *gyro_data, uint16_t data_start_index, const struct bmi160_dev *dev);
static void unpack_gyro_data(BMX160_Object_t *pObj, bmx160_sensor_data *gyro_data, uint16_t data_start_index);
/*!
 *  @brief This API is used to parse the gyro data from the
 *  FIFO data in header mode.
 *
 *  @param[in,out] gyro_data     : Structure instance of sensor data
 *  @param[in,out] gyro_length   : Number of gyro frames
 *  @param[in] dev               : Structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void extract_gyro_header_mode(bmi160_sensor_data *gyro_data, uint8_t *gyro_length, const struct bmi160_dev *dev);
static void extract_gyro_header_mode(BMX160_Object_t *pObj, bmx160_sensor_data *gyro_data, uint8_t *gyro_length);

/*!
 *  @brief This API computes the number of bytes of aux FIFO data
 *  which is to be parsed in header-less mode
 *
 *  @param[out] data_index       : The start index for parsing data
 *  @param[out] data_read_length : No of bytes to be parsed from FIFO buffer
 *  @param[in] aux_frame_count   : Number of Aux data frames to be read
 *  @param[in] dev               : Structure instance of bmi160_dev.
 */
//static void get_aux_len_to_parse(uint16_t *data_index, uint16_t *data_read_length, const uint8_t *aux_frame_count, const struct bmi160_dev *dev);
static void get_aux_len_to_parse(BMX160_Object_t *pObj, uint16_t *data_index, uint16_t *data_read_length, const uint8_t *aux_frame_count);

/*!
 *  @brief This API is used to parse the aux's data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed
 *
 *  @param[in,out] aux_data : structure instance of sensor data
 *  @param[in,out] idx      : Index value of number of bytes parsed
 *  @param[in,out] aux_index    : Index value of gyro data
 *                                (x,y,z axes) frames parsed
 *  @param[in] frame_info       : It consists of either fifo_data_enable
 *                                parameter in header-less mode or
 *                                frame header data in header mode
 *  @param[in] dev      : structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void unpack_aux_frame(bmi160_aux_data *aux_data, uint16_t *idx, uint8_t *aux_index, uint8_t frame_info, const struct bmi160_dev *dev);
static void unpack_aux_frame(BMX160_Object_t *pObj, bmx160_aux_data *aux_data, uint16_t *idx, uint8_t *aux_index, uint8_t frame_info);

/*!
 *  @brief This API is used to parse the aux data from the
 *  FIFO data and store it in the instance of the structure bmi160_aux_data.
 *
 * @param[in,out] aux_data      : structure instance of sensor data
 * @param[in,out] data_start_index  : Index value of number of bytes parsed
 * @param[in] dev           : structure instance of bmi160_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
//static void unpack_aux_data(bmi160_aux_data *aux_data, uint16_t data_start_index, const struct bmi160_dev *dev);
static void unpack_aux_data(BMX160_Object_t *pObj, bmx160_aux_data *aux_data, uint16_t data_start_index);
/*!
 *  @brief This API is used to parse the aux data from the
 *  FIFO data in header mode.
 *
 *  @param[in,out] aux_data     : Structure instance of sensor data
 *  @param[in,out] aux_length   : Number of aux frames
 *  @param[in] dev              : Structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void extract_aux_header_mode(bmi160_aux_data *aux_data, uint8_t *aux_length, const struct bmi160_dev *dev);
static void extract_aux_header_mode(BMX160_Object_t *pObj, bmx160_aux_data *aux_data, uint8_t *aux_length);
/*!
 *  @brief This API checks the presence of non-valid frames in the read fifo data.
 *
 *  @param[in,out] data_index    : The index of the current data to
 *                                be parsed from fifo data
 *  @param[in] dev               : Structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void check_frame_validity(uint16_t *data_index, const struct bmi160_dev *dev);
static void check_frame_validity(BMX160_Object_t *pObj, uint16_t *data_index);

/*!
 *  @brief This API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the user specified data.
 *
 *  @param[in,out] data_index       : Index of the FIFO data which
 *                                  is to be moved ahead of the
 *                                  current_frame_length
 *  @param[in] current_frame_length : Number of bytes in a particular frame
 *  @param[in] dev                  : Structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi160_dev *dev);
static void move_next_frame(BMX160_Object_t *pObj, uint16_t *data_index, uint8_t current_frame_length);
/*!
 *  @brief This API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance dev.
 *
 *  @param[in,out] data_index : Index of the FIFO data which
 *                              has the sensor time.
 *  @param[in] dev            : Structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void unpack_sensortime_frame(uint16_t *data_index, const struct bmi160_dev *dev);
static void unpack_sensortime_frame(BMX160_Object_t *pObj, uint16_t *data_index);
/*!
 *  @brief This API is used to parse and store the skipped_frame_count from
 *  the FIFO data in the structure instance dev.
 *
 *  @param[in,out] data_index   : Index of the FIFO data which
 *                                    has the skipped frame count.
 *  @param[in] dev              : Structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static void unpack_skipped_frame(uint16_t *data_index, const struct bmi160_dev *dev);
static void unpack_skipped_frame(BMX160_Object_t *pObj, uint16_t *data_index);

//static int8_t get_foc_status(uint8_t *foc_status, struct bmi160_dev const *dev);
//static int8_t configure_offset_enable(const bmi160_foc_conf *foc_conf, struct bmi160_dev const *dev);

/*!
 *  @brief This API is used to trigger the FOC in the sensor
 *
 *  @param[in,out] offset     : Structure instance of bmi160_offsets which
 *                              reads and stores the offset values after FOC
 *  @param[in] dev            : Structure instance of bmi160_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 */
//static int8_t trigger_foc(bmi160_offsets *offset, struct bmi160_dev const *dev);
static int32_t trigger_foc(BMX160_Object_t *pObj, bmx160_offsets *offset);

//static int8_t map_hardware_interrupt(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);
//static int8_t map_feature_interrupt(const bmi160_int_settg *int_config, const struct bmi160_dev *dev);

/** @defgroup BMX160_Private_Function_Prototypes BMX160 Private Function Prototypes
 * @{
 */

static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static void DelayWrap(void *Handle, uint32_t Period);


/**
 * @}
 */

/*********************** User function definitions ****************************/

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
int8_t bmi160_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev)
{
    int8_t rslt = BMX160_OK;

    /* Variable to define temporary length */
    uint16_t temp_len = len + dev->dummy_byte;

    /* Variable to define temporary buffer */
    uint8_t temp_buf[temp_len];

    /* Variable to define loop */
    uint16_t indx = 0;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->read == NULL))
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else if (len == 0)
    {
        rslt = BMX160_READ_WRITE_LENGHT_INVALID;
    }
    else
    {
        /* Configuring reg_addr for SPI Interface */
        if (dev->interface == BMX160_SPI_INTF)
        {
            reg_addr = (reg_addr | BMX160_SPI_RD_MASK);
        }
        rslt = dev->read(dev->id, reg_addr, temp_buf, temp_len);

        if (rslt == BMX160_OK)
        {
            /* Read the data from the position next to dummy byte */
            while (indx < len)
            {
                data[indx] = temp_buf[indx];
                indx++;
            }
        }
        else
        {
            rslt = BMX160_E_COM_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of sensor.
 */
int8_t bmi160_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev)
{
    int8_t rslt = BMX160_OK;
    uint8_t count = 0;

    /* Null-pointer check */
    if ((dev == NULL) || (dev->write == NULL))
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else if (len == 0)
    {
        rslt = BMX160_READ_WRITE_LENGHT_INVALID;
    }
    else
    {
        /* Configuring reg_addr for SPI Interface */
        if (dev->interface == BMX160_SPI_INTF)
        {
            reg_addr = (reg_addr & BMX160_SPI_WR_MASK);
        }
        if ((dev->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) ||
            (dev->prev_gyro_cfg.power == BMX160_GYRO_NORMAL_MODE))
        {
            rslt = dev->write(dev->id, reg_addr, data, len);

            /* Kindly refer bmi160 data sheet section 3.2.4 */
            dev->delay_ms(1);

        }
        else
        {
            /*Burst write is not allowed in
             * suspend & low power mode */
            for (; count < len; count++)
            {
                rslt = dev->write(dev->id, reg_addr, &data[count], 1);
                reg_addr++;

                /* Kindly refer bmi160 data sheet section 3.2.4 */
                dev->delay_ms(1);

            }
        }
        if (rslt != BMX160_OK)
        {
            rslt = BMX160_E_COM_FAIL;
        }
    }

    return rslt;
}

int32_t BMX160_RegisterBusIO(BMX160_Object_t *pObj, BMX160_IO_t *pIO){

	int32_t ret;
	if(pObj == NULL){
		ret = BMX160_E_NULL_PTR;
	}
	else{
		pObj->IO.Init 		= pIO->Init;
		pObj->IO.DeInit 	= pIO->DeInit;
		pObj->IO.BusType	= pIO->BusType;
		pObj->IO.Address	= pIO->Address;
		pObj->IO.WriteReg	= pIO->WriteReg;
		pObj->IO.ReadReg	= pIO->ReadReg;
		pObj->IO.Delayms	= pIO->Delayms;

		pObj->Ctx.read_reg	= ReadRegWrap;
		pObj->Ctx.write_reg = WriteRegWrap;
		pObj->Ctx.delay_ms  = DelayWrap;
		pObj->Ctx.mode		= 1; // Normal mode by default
		pObj->Ctx.handle	= pObj;

		if(pObj->IO.Init != NULL){
			ret = pObj->IO.Init();
		}
		else{
			ret = BMX160_E_NULL_PTR;
		}
	}
	return ret;

}
/*!
 *  @brief This API is the entry point for sensor.It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of BMX160 sensor.
 */
//int8_t bmi160_init(struct bmi160_dev *dev)
int32_t BMX160_Init(BMX160_Object_t *pObj)
{
    int32_t rslt;
    uint8_t data;
    //uint8_t try = 3;

    if(pObj->is_initialized == 0U){
        /* Dummy read of 0x7F register to enable SPI Interface
         * if SPI is used */
        if ((pObj->IO.BusType == BMX160_SPI_INTF))
        {
            rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_SPI_COMM_TEST_ADDR, &data, 1);
        }

        if (rslt == BMX160_OK)
        {
            /* Assign chip id as zero */
            pObj->chip_id = 0;

            //while ((try--) && (dev->chip_id != BMI160_CHIP_ID))
            while(pObj->chip_id != BMX160_CHIP_ID)
            {
                /* Read chip_id */
                rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_CHIP_ID_ADDR, &pObj->chip_id, 1);
            }
            if ((rslt == BMX160_OK) && (pObj->chip_id == BMX160_CHIP_ID))
            {
                pObj->any_sig_sel = BMX160_BOTH_ANY_SIG_MOTION_DISABLED;
                /* Soft reset */
                //rslt = bmi160_soft_reset(dev);
                rslt = BMX160_soft_reset(pObj);
            }
            else
            {
                rslt = BMX160_E_DEV_NOT_FOUND;
            }
        }
    }
    pObj->is_initialized = 1;

    return rslt;
}

/*!
 * @brief This API resets and restarts the device.
 * All register values are overwritten with default parameters.
 */
//int8_t bmi160_soft_reset(struct bmi160_dev *dev)
int32_t BMX160_soft_reset(BMX160_Object_t *pObj)
{
    int32_t rslt;
    uint8_t data = BMX160_SOFT_RESET_CMD;
    uint8_t mode = 0; //low power/suspend mode

    /* Reset the device */
    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
    	mode = 1;
    }
    rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_COMMAND_REG_ADDR, &data, 1, mode);
    //dev->delay_ms(BMI160_SOFT_RESET_DELAY_MS);
    bmx160_delay_ms(&(pObj->Ctx), BMX160_SOFT_RESET_DELAY_MS);
    if ((rslt == BMX160_OK) && (pObj->IO.BusType == BMX160_SPI_INTF))
    {
    	/* Dummy read of 0x7F register to enable SPI Interface if SPI is used */
        rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_SPI_COMM_TEST_ADDR, &data, 1);
    }
    if (rslt == BMX160_OK)
    {
        /* Update the default parameters */
        default_param_settg(pObj);
    }

    return rslt;
}

/*!
 * @brief This API configures the power mode, range and bandwidth
 * of sensor.
 */
//int8_t bmi160_set_sens_conf(struct bmi160_dev *dev)
int32_t BMX160_set_sens_conf(BMX160_Object_t *pObj)
{
    int32_t rslt = BMX160_OK;

    rslt = set_accel_conf(pObj);
        if (rslt == BMX160_OK)
        {
            rslt = set_gyro_conf(pObj);
            if (rslt == BMX160_OK)
            {
                /* write power mode for accel and gyro */
                rslt = BMX160_set_power_mode(pObj);
                if (rslt == BMX160_OK)
                {
                    rslt = check_invalid_settg(pObj);
                }
            }
        }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
//int8_t bmi160_set_power_mode(struct bmi160_dev *dev)
int32_t BMX160_set_power_mode(BMX160_Object_t *pObj)
{
    int32_t rslt = 0;

    rslt = set_accel_pwr(pObj);
    if (rslt == BMX160_OK)
    {
        rslt = set_gyro_pwr(pObj);
    }

    return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
//int8_t bmi160_get_power_mode(bmi160_pmu_status *pmu_status, const struct bmi160_dev *dev)
int32_t BMX160_get_power_mode(BMX160_Object_t *pObj, bmx160_pmu_status *pmu_status)
{
    int32_t rslt = 0;
    uint8_t power_mode = 0;

    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_PMU_STATUS_ADDR, &power_mode, 1);
    if (rslt == BMX160_OK)
    {
        /* Power mode of the accel,gyro,aux sensor is obtained */
        pmu_status->aux_pmu_status = BMX160_GET_BITS_POS_0(power_mode, BMX160_MAG_POWER_MODE);
        pmu_status->gyro_pmu_status = BMX160_GET_BITS(power_mode, BMX160_GYRO_POWER_MODE);
        pmu_status->accel_pmu_status = BMX160_GET_BITS(power_mode, BMX160_ACCEL_POWER_MODE);
     }
    return rslt;
}

/*!
 * @brief This API reads sensor data, stores it in
 * the bmi160_sensor_data structure pointer passed by the user.
 */
//int8_t bmi160_get_sensor_data(uint8_t select_sensor, bmi160_sensor_data *accel, bmi160_sensor_data *gyro, const struct bmi160_dev *dev)
int32_t BMX160_get_sensor_data(BMX160_Object_t *pObj, uint8_t select_sensor, bmx160_sensor_data *accel, bmx160_sensor_data *gyro)
{
    int32_t rslt = BMX160_OK;
    uint8_t time_sel;
    uint8_t sen_sel;
    uint8_t len = 0;

    /*Extract the sensor and time select information*/
    sen_sel = select_sensor & BMX160_SEN_SEL_MASK;
    time_sel = ((sen_sel & BMX160_TIME_SEL) >> 2);
    sen_sel = sen_sel & (BMX160_ACCEL_SEL | BMX160_GYRO_SEL);
    if (time_sel == 1)
    {
        len = 3;
    }

    switch (sen_sel){

    	case BMX160_ACCEL_ONLY:

    		/* Null-pointer check */
    		if (accel == NULL){
    			rslt = BMX160_E_NULL_PTR;
    		}
    		else
    		{
    			rslt = get_accel_data(&(pObj->Ctx), len, accel);
    		}
    		break;

    	case BMX160_GYRO_ONLY:

    		/* Null-pointer check */
    		if (gyro == NULL)
    		{
    			rslt = BMX160_E_NULL_PTR;
    		}
    		else
    		{
    			rslt = get_gyro_data(&(pObj->Ctx), len, gyro);
    		}
    		break;
    	case BMX160_BOTH_ACCEL_AND_GYRO:

    		/* Null-pointer check */
    		if ((gyro == NULL) || (accel == NULL))
    		{
    			rslt = BMX160_E_NULL_PTR;
    		}
    		else
    		{
    			rslt = get_accel_gyro_data(&(pObj->Ctx), len, accel, gyro);
            }
    		break;
    	default:
    	        rslt = BMX160_E_INVALID_INPUT;
    	    break;
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary interrupt based on
 *  the user settings in the bmi160_int_settg structure instance.
 */
int32_t BMX160_set_int_config(BMX160_Object_t *pObj, bmx160_int_settg *int_config)
{
    int8_t rslt = BMX160_OK;
    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        pObj->Ctx.mode = 1;
     }

    switch (int_config->int_type)
    {
        case BMX160_ACC_ANY_MOTION_INT:

            /*Any-motion  interrupt*/
            rslt = set_accel_any_motion_int(pObj, int_config);
            break;
        case BMX160_ACC_SIG_MOTION_INT:

            /* Significant motion interrupt */
            rslt = set_accel_sig_motion_int(pObj, int_config);
            break;
        case BMX160_ACC_SLOW_NO_MOTION_INT:

            /* Slow or no motion interrupt */
            rslt = set_accel_no_motion_int(&(pObj->Ctx), int_config);
            break;
        case BMX160_ACC_DOUBLE_TAP_INT:
        case BMX160_ACC_SINGLE_TAP_INT:

            /* Double tap and single tap Interrupt */
            rslt = set_accel_tap_int(&(pObj->Ctx), int_config);
            break;
        case BMX160_STEP_DETECT_INT:

            /* Step detector interrupt */
            rslt = set_accel_step_detect_int(&(pObj->Ctx), int_config);
            break;
        case BMX160_ACC_ORIENT_INT:

            /* Orientation interrupt */
            rslt = set_accel_orientation_int(&(pObj->Ctx), int_config);
            break;
        case BMX160_ACC_FLAT_INT:

            /* Flat detection interrupt */
            rslt = set_accel_flat_detect_int(&(pObj->Ctx), int_config);
            break;
        case BMX160_ACC_LOW_G_INT:

            /* Low-g interrupt */
            rslt = set_accel_low_g_int(&(pObj->Ctx),int_config);
            break;
        case BMX160_ACC_HIGH_G_INT:

            /* High-g interrupt */
            rslt = set_accel_high_g_int(&(pObj->Ctx), int_config);
            break;
        case BMX160_ACC_GYRO_DATA_RDY_INT:

            /* Data ready interrupt */
            rslt = set_accel_gyro_data_ready_int(&(pObj->Ctx), int_config);
            break;
        case BMX160_ACC_GYRO_FIFO_FULL_INT:

            /* Fifo full interrupt */
            rslt = set_fifo_full_int(&(pObj->Ctx), int_config);
            break;
        case BMX160_ACC_GYRO_FIFO_WATERMARK_INT:

            /* Fifo water-mark interrupt */
            rslt = set_fifo_watermark_int(&(pObj->Ctx), int_config);
            break;
        case BMX160_FIFO_TAG_INT_PIN:

            /* Fifo tagging feature support */
            /* Configure Interrupt pins */
            rslt = set_intr_pin_config(&(pObj->Ctx), int_config);
            break;
        default:
            break;
    }

    return rslt;
}

/*!
 * @brief This API enables or disable the step counter feature.
 * 1 - enable step counter (0 - disable)
 */
//int8_t bmi160_set_step_counter(uint8_t step_cnt_enable, const struct bmi160_dev *dev)
int32_t BMX160_set_step_counter(BMX160_Object_t *pObj, uint8_t step_cnt_enable)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        	mode = 1;
        }

    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_INT_STEP_CONFIG_1_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
    	if (step_cnt_enable == BMX160_ENABLE)
        {
    		data |= (uint8_t)(step_cnt_enable << 3);
        }
        else
        {
            data &= ~BMX160_STEP_COUNT_EN_BIT_MASK;
        }
        rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_INT_STEP_CONFIG_1_ADDR, &data, 1, mode); //normal mode
     }

    return rslt;
}

/*!
 * @brief This API reads the step counter value.
 */
//int8_t bmi160_read_step_counter(uint16_t *step_val, const struct bmi160_dev *dev)
int32_t BMX160_read_step_counter(BMX160_Object_t *pObj, uint16_t *step_val)
{
    int32_t rslt;
    uint8_t data[2] = { 0, 0 };
    uint16_t msb = 0;
    uint8_t lsb = 0;

    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_INT_STEP_CNT_0_ADDR, data, 2);
    if (rslt == BMX160_OK)
    {
    	lsb = data[0];
        msb = data[1] << 8;
        *step_val = msb | lsb;
    }

    return rslt;
}

/*!
 * @brief This API reads the mention no of byte of data from the given
 * register address of auxiliary sensor.
 */
//int8_t bmi160_aux_read(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, const struct bmi160_dev *dev)
int32_t BMX160_mag_read(BMX160_Object_t *pObj, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
    int32_t rslt = BMX160_OK;
    uint16_t map_len = 0;
    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        pObj->Ctx.mode = 1;
    }

    if (pObj->aux_cfg.aux_sensor_enable == BMX160_ENABLE)
    {
    	rslt = map_read_len(pObj, &map_len);
        if (rslt == BMX160_OK)
        {
            rslt = extract_aux_read(&(pObj->Ctx), map_len, reg_addr, aux_data, len);
        }
    }
    else
    {
        rslt = BMX160_E_INVALID_INPUT;
    }
    return rslt;
}

/*!
 * @brief This API writes the mention no of byte of data to the given
 * register address of auxiliary sensor.
 */
//int8_t bmi160_aux_write(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, const struct bmi160_dev *dev)
int32_t BMX160_mag_write(BMX160_Object_t *pObj, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
    int32_t rslt = BMX160_OK;
    uint8_t count = 0;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
    	mode = 1;
    }

    for (; count < len; count++)
    {
    	/* set data to write */
        //rslt = bmx160_write_reg(&(pObj->Ctx), BMI160_AUX_IF_4_ADDR, aux_data, 1, 1); // BMI160
    	rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_AUX_IF_3_ADDR, aux_data, 1, mode);
        //dev->delay_ms(BMI160_AUX_COM_DELAY);
        bmx160_delay_ms(&(pObj->Ctx), BMX160_AUX_COM_DELAY);
        if (rslt == BMX160_OK)
        {
        	/* set address to write */
            //rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_AUX_IF_3_ADDR, &reg_addr, 1, mode); // BMI160
        	rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_AUX_IF_2_ADDR, &reg_addr, 1, mode);
            //dev->delay_ms(BMI160_AUX_COM_DELAY);
            bmx160_delay_ms(&(pObj->Ctx), BMX160_AUX_COM_DELAY);
            if (rslt == BMX160_OK && (count < len - 1))
            {
                aux_data++;
                reg_addr++;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API initialize the auxiliary sensor
 * in order to access it.
 */
int32_t BMX160_mag_init(BMX160_Object_t *pObj)
{
    int32_t rslt;
    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        pObj->Ctx.mode = 1;
    }

    if (pObj->aux_cfg.aux_sensor_enable == BMX160_ENABLE)
    {
        /* Configures the auxiliary sensor interface settings */
        rslt = config_aux_settg(pObj);
    }
    else
    {
        rslt = BMX160_E_INVALID_INPUT;
    }
    return rslt;
}

/*!
 * @brief This API is used to setup the auxiliary sensor of bmi160 in auto mode
 * Thus enabling the auto update of 8 bytes of data from auxiliary sensor
 * to BMI160 register address 0x04 to 0x0B
 */
//int8_t bmi160_set_aux_auto_mode(uint8_t *data_addr, struct bmi160_dev *dev)
int32_t BMX160_set_mag_auto_mode(BMX160_Object_t *pObj, uint8_t *data_addr)
{
    int32_t rslt;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
         mode = 1;
    }

    if (pObj->aux_cfg.aux_sensor_enable == BMX160_ENABLE)
    {
    	/* Write the aux. address to read in 0x4D of BMI160*/
        rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_AUX_IF_2_ADDR, data_addr, 1, mode); //normal mode
        //dev->delay_ms(BMI160_AUX_COM_DELAY);
        bmx160_delay_ms(&(pObj->Ctx), BMX160_AUX_COM_DELAY);
        if (rslt == BMX160_OK)
        {
        	/* Configure the polling ODR for auxiliary sensor */
            rslt = config_aux_odr(pObj);
            if (rslt == BMX160_OK)
            {
            	/* Disable the aux. manual mode, i.e aux.sensor is in auto-mode (data-mode) */
                pObj->aux_cfg.manual_enable = BMX160_DISABLE;
                rslt = BMX160_config_mag_mode(pObj);
                /*  Auxiliary sensor data is obtained in auto mode from this point */
            }
        }
     }
     else
     {
            rslt = BMX160_E_INVALID_INPUT;
     }

    return rslt;
}

/*!
 * @brief This API configures the 0x4C register and settings like auxiliary sensor manual enable/ disable and aux burst read length.
 */
//int8_t bmi160_config_aux_mode(const struct bmi160_dev *dev)
int32_t BMX160_config_mag_mode(BMX160_Object_t *pObj)
{
    int32_t rslt;
    uint8_t aux_if[2] = { (uint8_t)(pObj->aux_cfg.aux_i2c_addr * 2), 0 };
    //uint8_t mode = 0;

//    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
//            mode = 1;
//    }

    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_AUX_IF_0_ADDR, &aux_if[1], 1);  //Karen: in BMI the address is AUX_IF_1_ADDR
    if (rslt == BMX160_OK)
    {
        /* update the Auxiliary interface to manual/auto mode */
        aux_if[1] = BMX160_SET_BITS(aux_if[1], BMX160_MANUAL_MODE_EN, pObj->aux_cfg.manual_enable);

        /* update the burst read length defined by user */
        aux_if[1] = BMX160_SET_BITS_POS_0(aux_if[1], BMX160_AUX_READ_BURST, pObj->aux_cfg.aux_rd_burst_len);

        /* Set the secondary interface address and manual mode  along with burst read length */
        //rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_AUX_IF_0_ADDR, &aux_if[0], 2, mode); //  This function not needed in BMX
        //dev->delay_ms(BMI160_AUX_COM_DELAY);
        bmx160_delay_ms(&(pObj->Ctx), BMX160_AUX_COM_DELAY);
    }

    return rslt;
}

/*!
 * @brief This API is used to read the raw uncompensated auxiliary sensor
 * data of 8 bytes from BMI160 register address 0x04 to 0x0B
 */
//int8_t bmi160_read_aux_data_auto_mode(uint8_t *aux_data, const struct bmi160_dev *dev)
int32_t BMX160_read_mag_data_auto_mode(BMX160_Object_t *pObj, uint8_t *aux_data)
{
    int32_t rslt;

    if ((pObj->aux_cfg.aux_sensor_enable == BMX160_ENABLE) && (pObj->aux_cfg.manual_enable == BMX160_DISABLE))
    {
    	/* Read the aux. sensor's raw data */
        rslt = bmx160_read_reg(&(pObj->Ctx),BMX160_AUX_DATA_ADDR, aux_data, 8);
    }
    else
    {
        rslt = BMX160_E_INVALID_INPUT;
    }

    return rslt;
}

/*!
 * @brief This is used to perform self test of accel/gyro of the BMI160 sensor
 */
//int8_t bmi160_perform_self_test(uint8_t select_sensor, struct bmi160_dev *dev)
int32_t BMX160_perform_self_test(BMX160_Object_t *pObj, uint8_t select_sensor)
{
    int32_t rslt;
    int8_t self_test_rslt = 0;

    /* Proceed if null check is fine */
        switch (select_sensor)
        {
            case BMX160_ACCEL_ONLY:
                rslt = perform_accel_self_test(pObj);
                break;
            case BMX160_GYRO_ONLY:

                /* Set the power mode as normal mode */
                pObj->gyro_cfg.power = BMX160_GYRO_NORMAL_MODE;
                rslt = BMX160_set_power_mode(pObj);

                /* Perform gyro self test */
                if (rslt == BMX160_OK)
                {
                    /* Perform gyro self test */
                    rslt = perform_gyro_self_test(pObj);
                }

                break;
            default:
                rslt = BMX160_E_INVALID_INPUT;
                break;
        }

        /* Check to ensure bus error does not occur */
        if (rslt >= BMX160_OK)
        {
            /* Store the status of self test result */
            self_test_rslt = rslt;

            /* Perform soft reset */
            //rslt = bmi160_soft_reset(dev);
            rslt = BMX160_soft_reset(pObj);

        }

        /* Check to ensure bus operations are success */
        if (rslt == BMX160_OK)
        {
            /* Restore self_test_rslt as return value */
            rslt = self_test_rslt;
        }

    return rslt;
}

/*!
 * @brief This API reads the data from fifo buffer.
 */
int32_t BMX160_get_fifo_data(BMX160_Object_t *pObj)
{
    int32_t rslt = 0;
    uint16_t bytes_to_read = 0;
    uint16_t user_fifo_len = 0;

    /* check the bmi160 structure as NULL*/
    if (pObj->fifo->data == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        reset_fifo_data_structure(pObj);

        /* get current FIFO fill-level*/
        rslt = get_fifo_byte_counter(&(pObj->Ctx), &bytes_to_read);
        if (rslt == BMX160_OK)
        {
            user_fifo_len = pObj->fifo->length;
            if ((pObj->fifo->length > bytes_to_read))
            {
                /* Handling the case where user requests
                 * more data than available in FIFO */
                pObj->fifo->length = bytes_to_read;
            }
            if ((pObj->fifo->fifo_time_enable == BMX160_FIFO_TIME_ENABLE) &&
                (bytes_to_read + BMX160_FIFO_BYTES_OVERREAD <= user_fifo_len))
            {
                /* Handling case of sensor time availability*/
            	pObj->fifo->length = pObj->fifo->length + BMX160_FIFO_BYTES_OVERREAD;
            }

            /* read only the filled bytes in the FIFO Buffer */
            rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_FIFO_DATA_ADDR, pObj->fifo->data, pObj->fifo->length);
        }
    }

    return rslt;
}

/*!
 *  @brief This API writes fifo_flush command to command register.This
 *  action clears all data in the Fifo without changing fifo configuration
 *  settings
 */
//int8_t bmi160_set_fifo_flush(const struct bmi160_dev *dev)
int32_t BMX160_set_fifo_flush(BMX160_Object_t *pObj)
{
    int32_t rslt = 0;
    uint8_t data = BMX160_FIFO_FLUSH_VALUE;
    uint8_t reg_addr = BMX160_COMMAND_REG_ADDR;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
    	mode = 1;
    }

    rslt = bmx160_write_reg(&(pObj->Ctx),reg_addr, &data, BMX160_ONE, mode); //normal mode

    return rslt;
}

/*!
 * @brief This API sets the FIFO configuration in the sensor.
 */
//int8_t bmi160_set_fifo_config(uint8_t config, uint8_t enable, struct bmi160_dev const *dev)
int32_t BMX160_set_fifo_config(BMX160_Object_t *pObj, uint8_t config, uint8_t enable)
{
    int32_t rslt = 0;
    uint8_t data = 0;
    uint8_t reg_addr = BMX160_FIFO_CONFIG_1_ADDR;
    uint8_t fifo_config = config & BMX160_FIFO_CONFIG_1_MASK;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    rslt = bmx160_read_reg(&(pObj->Ctx), reg_addr, &data, BMX160_ONE);
    if (rslt == BMX160_OK)
    {
        if (fifo_config > 0)
        {
            if (enable == BMX160_ENABLE)
            {
                data = data | fifo_config;
            }
            else
            {
                data = data & (~fifo_config);
            }
        }

        /* write fifo frame content configuration*/
        rslt = bmx160_write_reg(&(pObj->Ctx), reg_addr, &data, BMX160_ONE, mode);
        if (rslt == BMX160_OK)
        {
            /* read fifo frame content configuration*/
            rslt = bmx160_read_reg(&(pObj->Ctx), reg_addr, &data, BMX160_ONE);
            if (rslt == BMX160_OK)
            {
                /* extract fifo header enabled status */
                pObj->fifo->fifo_header_enable = data & BMX160_FIFO_HEAD_ENABLE;

                /* extract accel/gyr/aux. data enabled status */
                pObj->fifo->fifo_data_enable = data & BMX160_FIFO_M_G_A_ENABLE;

                /* extract fifo sensor time enabled status */
                pObj->fifo->fifo_time_enable = data & BMX160_FIFO_TIME_ENABLE;
            }
        }
    }

    return rslt;
}

/*! @brief This API is used to configure the down sampling ratios of
 *  the accel and gyro data for FIFO.Also, it configures filtered or
 *  pre-filtered data for accel and gyro.
 *
 */
//int8_t bmi160_set_fifo_down(uint8_t fifo_down, const struct bmi160_dev *dev)
int32_t BMX160_set_fifo_down(BMX160_Object_t *pObj, uint8_t fifo_down)
{
    int32_t rslt = 0;
    uint8_t data = 0;
    uint8_t reg_addr = BMX160_FIFO_DOWN_ADDR;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    rslt = bmx160_read_reg(&(pObj->Ctx), reg_addr, &data, BMX160_ONE);
    if (rslt == BMX160_OK)
    {
    	data = data | fifo_down;
        rslt = bmx160_write_reg(&(pObj->Ctx), reg_addr, &data, BMX160_ONE, mode);
    }

    return rslt;
}

/*!
 *  @brief This API sets the FIFO watermark level in the sensor.
 *
 */
//int8_t bmi160_set_fifo_wm(uint8_t fifo_wm, const struct bmi160_dev *dev)
int32_t BMX160_set_fifo_wm(BMX160_Object_t *pObj, uint8_t fifo_wm)
{
    int32_t rslt = 0;
    uint8_t data = fifo_wm;
    uint8_t reg_addr = BMX160_FIFO_CONFIG_0_ADDR;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    rslt = bmx160_write_reg(&(pObj->Ctx), reg_addr, &data, BMX160_ONE, mode);

    return rslt;
}

/*!
 *  @brief This API parses and extracts the accelerometer frames from
 *  FIFO data read by the "bmi160_get_fifo_data" API and stores it in
 *  the "accel_data" structure instance.
 */
//int8_t bmi160_extract_accel(bmi160_sensor_data *accel_data, uint8_t *accel_length, struct bmi160_dev const *dev)
int32_t BMX160_extract_accel(BMX160_Object_t *pObj, bmx160_sensor_data *accel_data, uint8_t *accel_length)
{
    int32_t rslt = 0;
    uint16_t data_index = 0;
    uint16_t data_read_length = 0;
    uint8_t accel_index = 0;
    uint8_t fifo_data_enable = 0;

    if (pObj->fifo == NULL || pObj->fifo->data == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* Parsing the FIFO data in header-less mode */
        if (pObj->fifo->fifo_header_enable == 0)
        {
            /* Number of bytes to be parsed from FIFO */
            get_accel_len_to_parse(pObj, &data_index, &data_read_length, accel_length);
            for (; data_index < data_read_length;)
            {
                /*Check for the availability of next two bytes of FIFO data */
                check_frame_validity(pObj, &data_index);
                fifo_data_enable = pObj->fifo->fifo_data_enable;
                unpack_accel_frame(pObj, accel_data, &data_index, &accel_index, fifo_data_enable);
            }

            /* update number of accel data read*/
            *accel_length = accel_index;

            /*update the accel byte index*/
            pObj->fifo->accel_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            extract_accel_header_mode(pObj, accel_data, accel_length);
        }
    }

    return rslt;
}

/*!
 *  @brief This API parses and extracts the gyro frames from
 *  FIFO data read by the "bmi160_get_fifo_data" API and stores it in
 *  the "gyro_data" structure instance.
 */
//int8_t bmi160_extract_gyro(bmi160_sensor_data *gyro_data, uint8_t *gyro_length, struct bmi160_dev const *dev)
int32_t BMX160_extract_gyro(BMX160_Object_t *pObj, bmx160_sensor_data *gyro_data, uint8_t *gyro_length)
{
    int32_t rslt = 0;
    uint16_t data_index = 0;
    uint16_t data_read_length = 0;
    uint8_t gyro_index = 0;
    uint8_t fifo_data_enable = 0;

    if (pObj->fifo->data == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* Parsing the FIFO data in header-less mode */
        if (pObj->fifo->fifo_header_enable == 0)
        {
            /* Number of bytes to be parsed from FIFO */
            get_gyro_len_to_parse(pObj, &data_index, &data_read_length, gyro_length);
            for (; data_index < data_read_length;)
            {
                /*Check for the availability of next two bytes of FIFO data */
                check_frame_validity(pObj, &data_index);
                fifo_data_enable = pObj->fifo->fifo_data_enable;
                unpack_gyro_frame(pObj, gyro_data, &data_index, &gyro_index, fifo_data_enable);
            }

            /* update number of gyro data read */
            *gyro_length = gyro_index;

            /* update the gyro byte index */
            pObj->fifo->gyro_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            extract_gyro_header_mode(pObj, gyro_data, gyro_length);
        }
    }

    return rslt;
}

/*!
 *  @brief This API parses and extracts the aux frames from
 *  FIFO data read by the "bmi160_get_fifo_data" API and stores it in
 *  the "aux_data" structure instance.
 */
//int8_t bmi160_extract_aux(bmi160_aux_data *aux_data, uint8_t *aux_len, struct bmi160_dev const *dev)
int32_t BMX160_extract_mag(BMX160_Object_t *pObj, bmx160_aux_data *aux_data, uint8_t *aux_len)
{
    int32_t rslt = 0;
    uint16_t data_index = 0;
    uint16_t data_read_length = 0;
    uint8_t aux_index = 0;
    uint8_t fifo_data_enable = 0;

    if ((pObj->fifo->data == NULL) || (aux_data == NULL))
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* Parsing the FIFO data in header-less mode */
        if (pObj->fifo->fifo_header_enable == 0)
        {
            /* Number of bytes to be parsed from FIFO */
            get_aux_len_to_parse(pObj, &data_index, &data_read_length, aux_len);
            for (; data_index < data_read_length;)
            {
                /* Check for the availability of next two
                 * bytes of FIFO data */
                check_frame_validity(pObj, &data_index);
                fifo_data_enable = pObj->fifo->fifo_data_enable;
                unpack_aux_frame(pObj, aux_data, &data_index, &aux_index, fifo_data_enable);
            }

            /* update number of aux data read */
            *aux_len = aux_index;

            /* update the aux byte index */
            pObj->fifo->aux_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            extract_aux_header_mode(pObj, aux_data, aux_len);
        }
    }

    return rslt;
}

/*!
 *  @brief This API starts the FOC of accel and gyro
 *
 *  @note FOC should not be used in low-power mode of sensor
 *
 *  @note Accel FOC targets values of +1g , 0g , -1g
 *  Gyro FOC always targets value of 0 dps
 */
//int8_t bmi160_start_foc(const bmi160_foc_conf *foc_conf,  bmi160_offsets *offset, struct bmi160_dev const *dev)
int32_t BMX160_start_foc(BMX160_Object_t *pObj, const bmx160_foc_conf *foc_conf, bmx160_offsets *offset)
{
    int32_t rslt;
    uint8_t data;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

        /* Set the offset enable bits */
        rslt = configure_offset_enable(&(pObj->Ctx), foc_conf);
        if (rslt == BMX160_OK)
        {
            /* Read the FOC config from the sensor */
            rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_FOC_CONF_ADDR, &data, 1);

            /* Set the FOC config for gyro */
            data = BMX160_SET_BITS(data, BMX160_GYRO_FOC_EN, foc_conf->foc_gyr_en);

            /* Set the FOC config for accel xyz axes */
            data = BMX160_SET_BITS(data, BMX160_ACCEL_FOC_X_CONF, foc_conf->foc_acc_x);
            data = BMX160_SET_BITS(data, BMX160_ACCEL_FOC_Y_CONF, foc_conf->foc_acc_y);
            data = BMX160_SET_BITS_POS_0(data, BMX160_ACCEL_FOC_Z_CONF, foc_conf->foc_acc_z);
            if (rslt == BMX160_OK)
            {
                /* Set the FOC config in the sensor */
                rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_FOC_CONF_ADDR, &data, 1, mode);
                if (rslt == BMX160_OK)
                {
                    /* Procedure to trigger
                     * FOC and check status */
                    rslt = trigger_foc(pObj, offset);
                }
            }
        }

    return rslt;
}

/*!
 *  @brief This API reads and stores the offset values of accel and gyro
 */
//int8_t bmi160_get_offsets(bmi160_offsets *offset, const struct bmi160_dev *dev)
int32_t BMX160_get_offsets(BMX160_Object_t *pObj, bmx160_offsets *offset)
{
    int32_t rslt;
    uint8_t data[7];
    uint8_t lsb, msb;
    int16_t offset_msb, offset_lsb;
    int16_t offset_data;

    /* Read the FOC config from the sensor */
    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_OFFSET_ADDR, data, 7);

    /* Accel offsets */
    offset->off_acc_x = (int8_t)data[0];
    offset->off_acc_y = (int8_t)data[1];
    offset->off_acc_z = (int8_t)data[2];

    /* Gyro x-axis offset */
    lsb = data[3];
    msb = BMX160_GET_BITS_POS_0(data[6], BMX160_GYRO_OFFSET_X);
    offset_msb = (int16_t)(msb << 14);
    offset_lsb = lsb << 6;
    offset_data = offset_msb | offset_lsb;

    /* Divide by 64 to get the Right shift by 6 value */
    offset->off_gyro_x = (int16_t)(offset_data / 64);

    /* Gyro y-axis offset */
    lsb = data[4];
    msb = BMX160_GET_BITS(data[6], BMX160_GYRO_OFFSET_Y);
    offset_msb = (int16_t)(msb << 14);
    offset_lsb = lsb << 6;
    offset_data = offset_msb | offset_lsb;

    /* Divide by 64 to get the Right shift by 6 value */
    offset->off_gyro_y = (int16_t)(offset_data / 64);

    /* Gyro z-axis offset */
    lsb = data[5];
    msb = BMX160_GET_BITS(data[6], BMX160_GYRO_OFFSET_Z);
    offset_msb = (int16_t)(msb << 14);
    offset_lsb = lsb << 6;
    offset_data = offset_msb | offset_lsb;

    /* Divide by 64 to get the Right shift by 6 value */
    offset->off_gyro_z = (int16_t)(offset_data / 64);

    return rslt;
}

/*!
 *  @brief This API writes the offset values of accel and gyro to
 *  the sensor but these values will be reset on POR or soft reset.
 */
//int8_t bmi160_set_offsets(const bmi160_foc_conf *foc_conf, const bmi160_offsets *offset, struct bmi160_dev const *dev)
int32_t BMX160_set_offsets(BMX160_Object_t *pObj, const bmx160_foc_conf *foc_conf, const bmx160_offsets *offset)
{
    int32_t rslt;
    uint8_t data[7];
    uint8_t x_msb, y_msb, z_msb;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    /* Update the accel offset */
    data[0] = (uint8_t)offset->off_acc_x;
    data[1] = (uint8_t)offset->off_acc_y;
    data[2] = (uint8_t)offset->off_acc_z;

    /* Update the LSB of gyro offset */
    data[3] = BMX160_GET_LSB(offset->off_gyro_x);
    data[4] = BMX160_GET_LSB(offset->off_gyro_y);
    data[5] = BMX160_GET_LSB(offset->off_gyro_z);

    /* Update the MSB of gyro offset */
    x_msb = BMX160_GET_BITS(offset->off_gyro_x, BMX160_GYRO_OFFSET);
    y_msb = BMX160_GET_BITS(offset->off_gyro_y, BMX160_GYRO_OFFSET);
    z_msb = BMX160_GET_BITS(offset->off_gyro_z, BMX160_GYRO_OFFSET);
    data[6] = (uint8_t)(z_msb << 4 | y_msb << 2 | x_msb);

    /* Set the offset enable/disable for gyro and accel */
    data[6] = BMX160_SET_BITS(data[6], BMX160_GYRO_OFFSET_EN, foc_conf->gyro_off_en);
    data[6] = BMX160_SET_BITS(data[6], BMX160_ACCEL_OFFSET_EN, foc_conf->acc_off_en);

    /* Set the offset config and values in the sensor */
    rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_OFFSET_ADDR, data, 7, mode);

    return rslt;
}

/*!
 *  @brief This API writes the image registers values to NVM which is
 *  stored even after POR or soft reset
 */
//int8_t bmi160_update_nvm(struct bmi160_dev const *dev)
int32_t BMX160_update_nvm(BMX160_Object_t *pObj)
{
    int32_t rslt;
    uint8_t data;
    uint8_t cmd = BMX160_NVM_BACKUP_EN;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    /* Read the nvm_prog_en configuration */
    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_CONF_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        data = BMX160_SET_BITS(data, BMX160_NVM_UPDATE, 1);

        /* Set the nvm_prog_en bit in the sensor */
        rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_CONF_ADDR, &data, 1, mode);
        if (rslt == BMX160_OK)
        {
            /* Update NVM */
            rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_COMMAND_REG_ADDR, &cmd, 1, mode);
            if (rslt == BMX160_OK)
            {
                /* Check for NVM ready status */
                rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_STATUS_ADDR, &data, 1);
                if (rslt == BMX160_OK)
                {
                    data = BMX160_GET_BITS(data, BMX160_NVM_STATUS);
                    if (data != BMX160_ENABLE)
                    {
                        /* Delay to update NVM */
                        //dev->delay_ms(25);
                    	bmx160_delay_ms(&(pObj->Ctx), 25);
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API gets the interrupt status from the sensor.
 */
//int8_t bmi160_get_int_status(enum bmi160_int_status_sel int_status_sel, bmi160_int_status *int_status, struct bmi160_dev const *dev)
int32_t BMX160_get_int_status(BMX160_Object_t *pObj, enum bmx160_int_status_sel int_status_sel, bmx160_int_status *int_status)
{
    int8_t rslt = 0;

    /* To get the status of all interrupts */
    if (int_status_sel == BMI160_INT_STATUS_ALL)
    {
        rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_INT_STATUS_ADDR, &int_status->data[0], 4);
    }
    else
    {
        if (int_status_sel & BMI160_INT_STATUS_0)
        {
            rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_INT_STATUS_ADDR, &int_status->data[0], 1);
        }
        if (int_status_sel & BMI160_INT_STATUS_1)
        {
            rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_INT_STATUS_ADDR + 1, &int_status->data[1], 1);
        }
        if (int_status_sel & BMI160_INT_STATUS_2)
        {
            rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_INT_STATUS_ADDR + 2, &int_status->data[2], 1);
        }
        if (int_status_sel & BMI160_INT_STATUS_3)
        {
            rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_INT_STATUS_ADDR + 3, &int_status->data[3], 1);
        }
    }

    return rslt;
}

/*********************** Local function definitions ***************************/

/*!
 * @brief This API sets the any-motion interrupt of the sensor.
 * This interrupt occurs when accel values exceeds preset threshold
 * for a certain period of time.
 */
static int32_t set_accel_any_motion_int(BMX160_Object_t *pObj, bmx160_int_settg *int_config)
{
    int32_t rslt;

    if (int_config == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* updating the interrupt structure to local structure */
        bmx160_acc_any_mot_int_cfg *any_motion_int_cfg = &(int_config->int_type_cfg.acc_any_motion_int);
        rslt = enable_accel_any_motion_int(pObj, any_motion_int_cfg);
        if (rslt == BMX160_OK)
        {
            rslt = config_any_motion_int_settg(&(pObj->Ctx),int_config, any_motion_int_cfg);
        }
    }

    return rslt;
}

///*!
// * @brief This API sets tap interrupts.Interrupt is fired when
// * tap movements happen.
// */
//static int8_t set_accel_tap_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Null-pointer check */
//    rslt = null_ptr_check(dev);
//    if ((rslt != BMI160_OK) || (int_config == NULL))
//    {
//        rslt = BMI160_E_NULL_PTR;
//    }
//    else
//    {
//        /* updating the interrupt structure to local structure */
//        bmi160_acc_tap_int_cfg *tap_int_cfg = &(int_config->int_type_cfg.acc_tap_int);
//        rslt = enable_tap_int(int_config, tap_int_cfg, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Configure Interrupt pins */
//            rslt = set_intr_pin_config(int_config, dev);
//            if (rslt == BMI160_OK)
//            {
//                rslt = config_tap_int_settg(int_config, tap_int_cfg, dev);
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API sets the data ready interrupt for both accel and gyro.
 * This interrupt occurs when new accel and gyro data comes.
 */
//static int8_t set_accel_gyro_data_ready_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Null-pointer check */
//    rslt = null_ptr_check(dev);
//    if ((rslt != BMI160_OK) || (int_config == NULL))
//    {
//        rslt = BMI160_E_NULL_PTR;
//    }
//    else
//    {
//        rslt = enable_data_ready_int(dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Configure Interrupt pins */
//            rslt = set_intr_pin_config(int_config, dev);
//            if (rslt == BMI160_OK)
//            {
//                rslt = map_hardware_interrupt(int_config, dev);
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API sets the significant motion interrupt of the sensor.This
 * interrupt occurs when there is change in user location.
 */
static int32_t set_accel_sig_motion_int(BMX160_Object_t *pObj, bmx160_int_settg *int_config)
{
    int32_t rslt;

    if (int_config == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    else
    {
        /* updating the interrupt structure to local structure */
        bmx160_acc_sig_mot_int_cfg *sig_mot_int_cfg = &(int_config->int_type_cfg.acc_sig_motion_int);
        rslt = enable_sig_motion_int(pObj, sig_mot_int_cfg);
        if (rslt == BMX160_OK)
        {
            rslt = config_sig_motion_int_settg(&(pObj->Ctx),int_config, sig_mot_int_cfg);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the no motion/slow motion interrupt of the sensor.
 * Slow motion is similar to any motion interrupt.No motion interrupt
 * occurs when slope bet. two accel values falls below preset threshold
 * for preset duration.
 */
//static int8_t set_accel_no_motion_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Null-pointer check */
//    rslt = null_ptr_check(dev);
//    if ((rslt != BMI160_OK) || (int_config == NULL))
//    {
//        rslt = BMI160_E_NULL_PTR;
//    }
//    else
//    {
//        /* updating the interrupt structure to local structure */
//        bmi160_acc_no_motion_int_cfg *no_mot_int_cfg = &(int_config->int_type_cfg.acc_no_motion_int);
//        rslt = enable_no_motion_int(no_mot_int_cfg, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Configure the INT PIN settings*/
//            rslt = config_no_motion_int_settg(int_config, no_mot_int_cfg, dev);
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API sets the step detection interrupt.This interrupt
 * occurs when the single step causes accel values to go above
 * preset threshold.
 */
//static int8_t set_accel_step_detect_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Null-pointer check */
//    rslt = null_ptr_check(dev);
//    if ((rslt != BMI160_OK) || (int_config == NULL))
//    {
//        rslt = BMI160_E_NULL_PTR;
//    }
//    else
//    {
//        /* updating the interrupt structure to local structure */
//        bmi160_acc_step_detect_int_cfg *step_detect_int_cfg = &(int_config->int_type_cfg.acc_step_detect_int);
//        rslt = enable_step_detect_int(step_detect_int_cfg, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Configure Interrupt pins */
//            rslt = set_intr_pin_config(int_config, dev);
//            if (rslt == BMI160_OK)
//            {
//                rslt = map_feature_interrupt(int_config, dev);
//                if (rslt == BMI160_OK)
//                {
//                    rslt = config_step_detect(step_detect_int_cfg, dev);
//                }
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API sets the orientation interrupt of the sensor.This
 * interrupt occurs when there is orientation change in the sensor
 * with respect to gravitational field vector g.
 */
//static int8_t set_accel_orientation_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Null-pointer check */
//    rslt = null_ptr_check(dev);
//    if ((rslt != BMI160_OK) || (int_config == NULL))
//    {
//        rslt = BMI160_E_NULL_PTR;
//    }
//    else
//    {
//        /* updating the interrupt structure to local structure */
//        bmi160_acc_orient_int_cfg *orient_int_cfg = &(int_config->int_type_cfg.acc_orient_int);
//        rslt = enable_orient_int(orient_int_cfg, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Configure Interrupt pins */
//            rslt = set_intr_pin_config(int_config, dev);
//            if (rslt == BMI160_OK)
//            {
//                /* map INT pin to orient interrupt */
//                rslt = map_feature_interrupt(int_config, dev);
//                if (rslt == BMI160_OK)
//                {
//                    /* configure the
//                     * orientation setting*/
//                    rslt = config_orient_int_settg(orient_int_cfg, dev);
//                }
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API sets the flat interrupt of the sensor.This interrupt
 * occurs in case of flat orientation
 */
//static int8_t set_accel_flat_detect_int(bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Null-pointer check */
//    rslt = null_ptr_check(dev);
//    if ((rslt != BMI160_OK) || (int_config == NULL))
//    {
//        rslt = BMI160_E_NULL_PTR;
//    }
//    else
//    {
//        /* updating the interrupt structure to local structure */
//        bmi160_acc_flat_detect_int_cfg *flat_detect_int = &(int_config->int_type_cfg.acc_flat_int);
//
//        /* enable the flat interrupt */
//        rslt = enable_flat_int(flat_detect_int, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Configure Interrupt pins */
//            rslt = set_intr_pin_config(int_config, dev);
//            if (rslt == BMI160_OK)
//            {
//                /* map INT pin to flat interrupt */
//                rslt = map_feature_interrupt(int_config, dev);
//                if (rslt == BMI160_OK)
//                {
//                    /* configure the flat setting*/
//                    rslt = config_flat_int_settg(flat_detect_int, dev);
//                }
//            }
//        }
//    }
//
//    return rslt;
//}


/*!
 * @brief This API configures the pins to fire the
 * interrupt signal when it occurs.
 */
//static int8_t set_intr_pin_config(const bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* configure the behavioural settings of interrupt pin */
//    rslt = config_int_out_ctrl(int_config, dev);
//    if (rslt == BMI160_OK)
//    {
//        rslt = config_int_latch(int_config, dev);
//    }
//
//    return rslt;
//}


/*!
 * @brief This API sets the default configuration parameters of accel & gyro.
 * Also maintain the previous state of configurations.
 */
static void default_param_settg(BMX160_Object_t *pObj)
{
    /* Initializing accel and gyro params with
     * default values */
	pObj->accel_cfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
	pObj->accel_cfg.odr = BMX160_ACCEL_ODR_100HZ;
	//pObj->accel_cfg.power = BMX160_ACCEL_SUSPEND_MODE; //Karen: I want it works in normal mode by default
	pObj->accel_cfg.power = BMX160_ACCEL_NORMAL_MODE;
	pObj->accel_cfg.range = BMX160_ACCEL_RANGE_2G;
    pObj->gyro_cfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
    pObj->gyro_cfg.odr = BMX160_GYRO_ODR_100HZ;
    //pObj->gyro_cfg.power = BMX160_GYRO_SUSPEND_MODE; //Karen: I want it works in normal mode by default
    pObj->gyro_cfg.power = BMX160_GYRO_NORMAL_MODE;
    pObj->gyro_cfg.range = BMX160_GYRO_RANGE_2000_DPS;

    /* To maintain the previous state of accel configuration */
    pObj->prev_accel_cfg = pObj->accel_cfg;

    /* To maintain the previous state of gyro configuration */
    pObj->prev_gyro_cfg = pObj->gyro_cfg;
}

/*!
 * @brief This API set the accel configuration.
 */
static int32_t set_accel_conf(BMX160_Object_t *pObj)
{
    int32_t rslt;
    uint8_t data[2] = { 0 };
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    rslt = check_accel_config(pObj, data);
    if (rslt == BMX160_OK)
    {
        /* Write output data rate and bandwidth */
        rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, &data[0], 1, mode); //normal mode
        if (rslt == BMX160_OK)
        {
        	pObj->prev_accel_cfg.odr = pObj->accel_cfg.odr;
        	pObj->prev_accel_cfg.bw = pObj->accel_cfg.bw;

            /* write accel range */
            rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_ACCEL_RANGE_ADDR, &data[1], 1, mode); //normal mode
            if (rslt == BMX160_OK)
            {
            	pObj->prev_accel_cfg.range = pObj->accel_cfg.range;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API check the accel configuration.
 */
static int32_t check_accel_config(BMX160_Object_t *pObj, uint8_t *data)
{
    int32_t rslt;

    /* read accel Output data rate and bandwidth */
    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, data, 2);
    if (rslt == BMX160_OK)
    {
        rslt = process_accel_odr(pObj, &data[0]);
        if (rslt == BMX160_OK)
        {
            rslt = process_accel_bw(pObj, &data[0]);
            if (rslt == BMX160_OK)
            {
                rslt = process_accel_range(pObj, &data[1]);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API process the accel odr.
 */
static int32_t process_accel_odr(BMX160_Object_t *pObj, uint8_t *data)
{
    int32_t rslt = 0;
    uint8_t temp = 0;
    uint8_t odr = 0;

    if (pObj->accel_cfg.odr <= BMX160_ACCEL_ODR_MAX)
    {
        if (pObj->accel_cfg.odr != pObj->prev_accel_cfg.odr)
        {
            odr = (uint8_t)pObj->accel_cfg.odr;
            temp = *data & ~BMX160_ACCEL_ODR_MASK;

            /* Adding output data rate */
            *data = temp | (odr & BMX160_ACCEL_ODR_MASK);
        }
    }
    else
    {
        rslt = BMX160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API process the accel bandwidth.
 */
static int32_t process_accel_bw(BMX160_Object_t *pObj, uint8_t *data)
{
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t bw = 0;

    if (pObj->accel_cfg.bw <= BMX160_ACCEL_BW_MAX)
    {
        if (pObj->accel_cfg.bw != pObj->prev_accel_cfg.bw)
        {
            bw = (uint8_t)pObj->accel_cfg.bw;
            temp = *data & ~BMX160_ACCEL_BW_MASK;

            /* Adding bandwidth */
            *data = temp | ((bw << 4) & BMX160_ACCEL_ODR_MASK);
        }
    }
    else
    {
        rslt = BMX160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API process the accel range.
 */
static int32_t process_accel_range(BMX160_Object_t *pObj, uint8_t *data)
{
    int32_t rslt = 0;
    uint8_t temp = 0;
    uint8_t range = 0;

    if (pObj->accel_cfg.range <= BMX160_ACCEL_RANGE_MAX)
    {
        if (pObj->accel_cfg.range != pObj->prev_accel_cfg.range)
        {
            range = (uint8_t)pObj->accel_cfg.range;
            temp = *data & ~BMX160_ACCEL_RANGE_MASK;

            /* Adding range */
            *data = temp | (range & BMX160_ACCEL_RANGE_MASK);
        }
    }
    else
    {
        rslt = BMX160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API checks the invalid settings for ODR & Bw for
 * Accel and Gyro.
 */
static int32_t check_invalid_settg(BMX160_Object_t *pObj)
{
    int32_t rslt;
    uint8_t data = 0;

    /* read the error reg */
    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_ERROR_REG_ADDR, &data, 1);
    data = data >> 1;
    data = data & BMX160_ERR_REG_MASK;
    if (data == 1)
    {
        rslt = BMX160_E_ACCEL_ODR_BW_INVALID;
    }
    else if (data == 2)
    {
        rslt = BMX160_E_GYRO_ODR_BW_INVALID;
    }
    else if (data == 3)
    {
        rslt = BMX160_E_LWP_PRE_FLTR_INT_INVALID;
    }
    else if (data == 7)
    {
        rslt = BMX160_E_LWP_PRE_FLTR_INVALID;
    }

    return rslt;
}

static int32_t set_gyro_conf(BMX160_Object_t *pObj)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    rslt = check_gyro_config(pObj, data);
    if (rslt == BMX160_OK)
    {
        /* Write output data rate and bandwidth */
        rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_GYRO_CONFIG_ADDR, &data[0], 1, mode);
        if (rslt == BMX160_OK)
        {
        	pObj->prev_gyro_cfg.odr = pObj->gyro_cfg.odr;
        	pObj->prev_gyro_cfg.bw = pObj->gyro_cfg.bw;

            /* Write gyro range */
            rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_GYRO_RANGE_ADDR, &data[1], 1, mode);
            if (rslt == BMX160_OK)
            {
            	pObj->prev_gyro_cfg.range = pObj->gyro_cfg.range;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API check the gyro configuration.
 */
static int32_t check_gyro_config(BMX160_Object_t *pObj, uint8_t *data)
{
    int32_t rslt;

    /* read gyro Output data rate and bandwidth */
    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_GYRO_CONFIG_ADDR, data, 2);
    if (rslt == BMX160_OK)
    {
        rslt = process_gyro_odr(pObj, &data[0]);
        if (rslt == BMX160_OK)
        {
            rslt = process_gyro_bw(pObj, &data[0]);
            if (rslt == BMX160_OK)
            {
                rslt = process_gyro_range(pObj, &data[1]);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API process the gyro odr.
 */
static int32_t process_gyro_odr(BMX160_Object_t *pObj, uint8_t *data)
{
    int32_t rslt = 0;
    uint8_t temp = 0;
    uint8_t odr = 0;

    if (pObj->gyro_cfg.odr <= BMX160_GYRO_ODR_MAX)
    {
        if (pObj->gyro_cfg.odr != pObj->prev_gyro_cfg.odr)
        {
            odr = (uint8_t)pObj->gyro_cfg.odr;
            temp = (*data & ~BMX160_GYRO_ODR_MASK);

            /* Adding output data rate */
            *data = temp | (odr & BMX160_GYRO_ODR_MASK);
        }
    }
    else
    {
        rslt = BMX160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API process the gyro bandwidth.
 */
static int32_t process_gyro_bw(BMX160_Object_t *pObj, uint8_t *data)
{
    int32_t rslt = 0;
    uint8_t temp = 0;
    uint8_t bw = 0;

    if (pObj->gyro_cfg.bw <= BMX160_GYRO_BW_MAX)
    {
        bw = (uint8_t)pObj->gyro_cfg.bw;
        temp = *data & ~BMX160_GYRO_BW_MASK;

        /* Adding bandwidth */
        *data = temp | ((bw << 4) & BMX160_GYRO_BW_MASK);
    }
    else
    {
        rslt = BMX160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API process the gyro range.
 */
static int32_t process_gyro_range(BMX160_Object_t *pObj, uint8_t *data)
{
    int32_t rslt = 0;
    uint8_t temp = 0;
    uint8_t range = 0;

    if (pObj->gyro_cfg.range <= BMX160_GYRO_RANGE_MAX)
    {
        if (pObj->gyro_cfg.range != pObj->prev_gyro_cfg.range)
        {
            range = (uint8_t)pObj->gyro_cfg.range;
            temp = *data & ~BMX160_GYRO_RANGE_MSK;

            /* Adding range */
            *data = temp | (range & BMX160_GYRO_RANGE_MSK);
        }
    }
    else
    {
        rslt = BMX160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API sets the accel power.
 */
static int32_t set_accel_pwr(BMX160_Object_t *pObj)
{
    int32_t rslt = 0;
    uint8_t data = 0;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    if ((pObj->accel_cfg.power >= BMX160_ACCEL_SUSPEND_MODE) && (pObj->accel_cfg.power <= BMX160_ACCEL_LOWPOWER_MODE))
    {
        if (pObj->accel_cfg.power != pObj->prev_accel_cfg.power)
        {
            rslt = process_under_sampling(pObj, &data);
            if (rslt == BMX160_OK)
            {
                /* Write accel power */
                rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_COMMAND_REG_ADDR, &pObj->accel_cfg.power, 1, mode);

                /* Add delay of 3.8 ms - refer data sheet table 24*/
                if (pObj->prev_accel_cfg.power == BMX160_ACCEL_SUSPEND_MODE)
                {
                    //dev->delay_ms(BMI160_ACCEL_DELAY_MS);
                	bmx160_delay_ms(&(pObj->Ctx), BMX160_ACCEL_DELAY_MS);
                }
                pObj->prev_accel_cfg.power = pObj->accel_cfg.power;
            }
        }
    }
    else
    {
        rslt = BMX160_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 * @brief This API process the undersampling setting of Accel.
 */
static int32_t process_under_sampling(BMX160_Object_t *pObj, uint8_t *data)
{
    int32_t rslt;
    uint8_t temp = 0;
    uint8_t pre_filter = 0;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, data, 1);
    if (rslt == BMX160_OK)
    {
        if (pObj->accel_cfg.power == BMX160_ACCEL_LOWPOWER_MODE)
        {
            temp = *data & ~BMX160_ACCEL_UNDERSAMPLING_MASK;

            /* Set under-sampling parameter */
            *data = temp | ((1 << 7) & BMX160_ACCEL_UNDERSAMPLING_MASK);

            /* Write data */
            rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, data, 1, mode);

            /* disable the pre-filter data in
             * low power mode */
            if (rslt == BMX160_OK)
            {
                /* Disable the Pre-filter data*/
                rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_INT_DATA_0_ADDR, &pre_filter, 2, mode);
            }
        }
        else if (*data & BMX160_ACCEL_UNDERSAMPLING_MASK)
        {
            temp = *data & ~BMX160_ACCEL_UNDERSAMPLING_MASK;

            /* disable under-sampling parameter
             * if already enabled */
            *data = temp;

            /* Write data */
            rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, data, 1, mode);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the gyro power mode.
 */
static int32_t set_gyro_pwr(BMX160_Object_t *pObj)
{
    int8_t rslt = 0;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    if ((pObj->gyro_cfg.power == BMX160_GYRO_SUSPEND_MODE) || (pObj->gyro_cfg.power == BMX160_GYRO_NORMAL_MODE) ||
        (pObj->gyro_cfg.power == BMX160_GYRO_FASTSTARTUP_MODE))
    {
        if (pObj->gyro_cfg.power != pObj->prev_gyro_cfg.power)
        {
            /* Write gyro power */
            rslt = bmx160_write_reg(&(pObj->Ctx),BMX160_COMMAND_REG_ADDR, &pObj->gyro_cfg.power, 1, mode);
            if (pObj->prev_gyro_cfg.power == BMX160_GYRO_SUSPEND_MODE)
            {
                /* Delay of 80 ms - datasheet Table 24 */
                //dev->delay_ms(BMI160_GYRO_DELAY_MS);
                bmx160_delay_ms(&(pObj->Ctx), BMX160_GYRO_DELAY_MS);
            }
            else if ((pObj->prev_gyro_cfg.power == BMX160_GYRO_FASTSTARTUP_MODE) &&
                     (pObj->gyro_cfg.power == BMX160_GYRO_NORMAL_MODE))
            {
                /* This delay is required for transition from
                 * fast-startup mode to normal mode - datasheet Table 3 */
                //dev->delay_ms(10);
                bmx160_delay_ms(&(pObj->Ctx), 10);
            }
            else
            {
                /* do nothing */
            }
            pObj->prev_gyro_cfg.power = pObj->gyro_cfg.power;
        }
    }
    else
    {
        rslt = BMX160_E_OUT_OF_RANGE;
    }

    return rslt;
}

///*!
// * @brief This API reads accel data along with sensor time if time is requested
// * by user. Kindly refer the user guide(README.md) for more info.
// */
//static int8_t get_accel_data(uint8_t len, bmi160_sensor_data *accel, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t idx = 0;
//    uint8_t data_array[9] = { 0 };
//    uint8_t time_0 = 0;
//    uint16_t time_1 = 0;
//    uint32_t time_2 = 0;
//    uint8_t lsb;
//    uint8_t msb;
//    int16_t msblsb;
//
//    /* read accel sensor data along with time if requested */
//    rslt = bmi160_get_regs(BMI160_ACCEL_DATA_ADDR, data_array, 6 + len, dev);
//    if (rslt == BMI160_OK)
//    {
//        /* Accel Data */
//        lsb = data_array[idx++];
//        msb = data_array[idx++];
//        msblsb = (int16_t)((msb << 8) | lsb);
//        accel->x = msblsb; /* Data in X axis */
//        lsb = data_array[idx++];
//        msb = data_array[idx++];
//        msblsb = (int16_t)((msb << 8) | lsb);
//        accel->y = msblsb; /* Data in Y axis */
//        lsb = data_array[idx++];
//        msb = data_array[idx++];
//        msblsb = (int16_t)((msb << 8) | lsb);
//        accel->z = msblsb; /* Data in Z axis */
//        if (len == 3)
//        {
//            time_0 = data_array[idx++];
//            time_1 = (uint16_t)(data_array[idx++] << 8);
//            time_2 = (uint32_t)(data_array[idx++] << 16);
//            accel->sensortime = (uint32_t)(time_2 | time_1 | time_0);
//        }
//        else
//        {
//            accel->sensortime = 0;
//        }
//    }
//    else
//    {
//        rslt = BMI160_E_COM_FAIL;
//    }
//
//    return rslt;
//}

///*!
// * @brief This API reads accel data along with sensor time if time is requested
// * by user. Kindly refer the user guide(README.md) for more info.
// */
//static int8_t get_gyro_data(uint8_t len, bmi160_sensor_data *gyro, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t idx = 0;
//    uint8_t data_array[15] = { 0 };
//    uint8_t time_0 = 0;
//    uint16_t time_1 = 0;
//    uint32_t time_2 = 0;
//    uint8_t lsb;
//    uint8_t msb;
//    int16_t msblsb;
//
//    if (len == 0)
//    {
//        /* read gyro data only */
//        rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 6, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Gyro Data */
//            lsb = data_array[idx++];
//            msb = data_array[idx++];
//            msblsb = (int16_t)((msb << 8) | lsb);
//            gyro->x = msblsb; /* Data in X axis */
//            lsb = data_array[idx++];
//            msb = data_array[idx++];
//            msblsb = (int16_t)((msb << 8) | lsb);
//            gyro->y = msblsb; /* Data in Y axis */
//            lsb = data_array[idx++];
//            msb = data_array[idx++];
//            msblsb = (int16_t)((msb << 8) | lsb);
//            gyro->z = msblsb; /* Data in Z axis */
//            gyro->sensortime = 0;
//        }
//        else
//        {
//            rslt = BMI160_E_COM_FAIL;
//        }
//    }
//    else
//    {
//        /* read gyro sensor data along with time */
//        rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Gyro Data */
//            lsb = data_array[idx++];
//            msb = data_array[idx++];
//            msblsb = (int16_t)((msb << 8) | lsb);
//            gyro->x = msblsb; /* gyro X axis data */
//            lsb = data_array[idx++];
//            msb = data_array[idx++];
//            msblsb = (int16_t)((msb << 8) | lsb);
//            gyro->y = msblsb; /* gyro Y axis data */
//            lsb = data_array[idx++];
//            msb = data_array[idx++];
//            msblsb = (int16_t)((msb << 8) | lsb);
//            gyro->z = msblsb; /* gyro Z axis data */
//            idx = idx + 6;
//            time_0 = data_array[idx++];
//            time_1 = (uint16_t)(data_array[idx++] << 8);
//            time_2 = (uint32_t)(data_array[idx++] << 16);
//            gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);
//        }
//        else
//        {
//            rslt = BMI160_E_COM_FAIL;
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API reads accel and gyro data along with sensor time
 * if time is requested by user.
 *  Kindly refer the user guide(README.md) for more info.
 */

/*!
 * @brief This API enables the any-motion interrupt for accel.
 */
static int32_t enable_accel_any_motion_int(BMX160_Object_t *pObj, const bmx160_acc_any_mot_int_cfg *any_motion_int_cfg)
{
    int32_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    /* Enable any motion x, any motion y, any motion z
     * in Int Enable 0 register */
    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_INT_ENABLE_0_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        if (any_motion_int_cfg->anymotion_en == BMX160_ENABLE)
        {
            temp = data & ~BMX160_ANY_MOTION_X_INT_EN_MASK;

            /* Adding Any_motion x axis */
            data = temp | (any_motion_int_cfg->anymotion_x & BMX160_ANY_MOTION_X_INT_EN_MASK);
            temp = data & ~BMX160_ANY_MOTION_Y_INT_EN_MASK;

            /* Adding Any_motion y axis */
            data = temp | ((any_motion_int_cfg->anymotion_y << 1) & BMX160_ANY_MOTION_Y_INT_EN_MASK);
            temp = data & ~BMX160_ANY_MOTION_Z_INT_EN_MASK;

            /* Adding Any_motion z axis */
            data = temp | ((any_motion_int_cfg->anymotion_z << 2) & BMX160_ANY_MOTION_Z_INT_EN_MASK);

            /* any-motion feature selected*/
            pObj->any_sig_sel = BMX160_ANY_MOTION_ENABLED;
        }
        else
        {
            data = data & ~BMX160_ANY_MOTION_ALL_INT_EN_MASK;

            /* neither any-motion feature nor sig-motion selected */
            pObj->any_sig_sel = BMX160_BOTH_ANY_SIG_MOTION_DISABLED;
        }

        /* write data to Int Enable 0 register */
        rslt = bmx160_write_reg(&(pObj->Ctx),BMX160_INT_ENABLE_0_ADDR, &data, 1, mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API disable the sig-motion interrupt.
 */
//static int8_t disable_sig_motion_int(const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Disabling Significant motion interrupt if enabled */
//    rslt = bmi160_get_regs(BMI160_INT_MOTION_3_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = (data & BMI160_SIG_MOTION_SEL_MASK);
//        if (temp)
//        {
//            temp = data & ~BMI160_SIG_MOTION_SEL_MASK;
//            data = temp;
//
//            /* Write data to register */
//            rslt = bmi160_set_regs(BMI160_INT_MOTION_3_ADDR, &data, 1, dev);
//        }
//    }
//
//    return rslt;
//}


/*!
 * @brief This API configure the source of data(filter & pre-filter)
 * for any-motion interrupt.
 */
//static int8_t config_any_motion_src(const bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,
//                                    const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Configure Int data 1 register to add source of interrupt */
//    rslt = bmi160_get_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_MOTION_SRC_INT_MASK;
//        data = temp | ((any_motion_int_cfg->anymotion_data_src << 7) & BMI160_MOTION_SRC_INT_MASK);
//
//        /* Write data to DATA 1 address */
//        rslt = bmi160_set_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the duration and threshold of
 * any-motion interrupt.
 */
//static int8_t config_any_dur_threshold(const bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,
//                                       const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//    uint8_t data_array[2] = { 0 };
//    uint8_t dur;
//
//    /* Configure Int Motion 0 register */
//    rslt = bmi160_get_regs(BMI160_INT_MOTION_0_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        /* slope duration */
//        dur = (uint8_t)any_motion_int_cfg->anymotion_dur;
//        temp = data & ~BMI160_SLOPE_INT_DUR_MASK;
//        data = temp | (dur & BMI160_MOTION_SRC_INT_MASK);
//        data_array[0] = data;
//
//        /* add slope threshold */
//        data_array[1] = any_motion_int_cfg->anymotion_thr;
//
//        /* INT MOTION 0 and INT MOTION 1 address lie consecutively,
//         * hence writing data to respective registers at one go */
//
//        /* Writing to Int_motion 0 and
//         * Int_motion 1 Address simultaneously */
//        rslt = bmi160_set_regs(BMI160_INT_MOTION_0_ADDR, data_array, 2, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure necessary setting of any-motion interrupt.
 */
//static int8_t config_any_motion_int_settg(const bmi160_int_settg *int_config,
//                                          const bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,
//                                          const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Configure Interrupt pins */
//    rslt = set_intr_pin_config(int_config, dev);
//    if (rslt == BMI160_OK)
//    {
//        rslt = disable_sig_motion_int(dev);
//        if (rslt == BMI160_OK)
//        {
//            rslt = map_feature_interrupt(int_config, dev);
//            if (rslt == BMI160_OK)
//            {
//                rslt = config_any_motion_src(any_motion_int_cfg, dev);
//                if (rslt == BMI160_OK)
//                {
//                    rslt = config_any_dur_threshold(any_motion_int_cfg, dev);
//                }
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API enable the data ready interrupt.
 */
//static int8_t enable_data_ready_int(const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Enable data ready interrupt in Int Enable 1 register */
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_DATA_RDY_INT_EN_MASK;
//        data = temp | ((1 << 4) & BMI160_DATA_RDY_INT_EN_MASK);
//
//        /* Writing data to INT ENABLE 1 Address */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API enables the no motion/slow motion interrupt.
 */
//static int8_t enable_no_motion_int(const bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,
//                                   const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Enable no motion x, no motion y, no motion z
//     * in Int Enable 2 register */
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        if (no_mot_int_cfg->no_motion_x == 1)
//        {
//            temp = data & ~BMI160_NO_MOTION_X_INT_EN_MASK;
//
//            /* Adding No_motion x axis */
//            data = temp | (1 & BMI160_NO_MOTION_X_INT_EN_MASK);
//        }
//        if (no_mot_int_cfg->no_motion_y == 1)
//        {
//            temp = data & ~BMI160_NO_MOTION_Y_INT_EN_MASK;
//
//            /* Adding No_motion x axis */
//            data = temp | ((1 << 1) & BMI160_NO_MOTION_Y_INT_EN_MASK);
//        }
//        if (no_mot_int_cfg->no_motion_z == 1)
//        {
//            temp = data & ~BMI160_NO_MOTION_Z_INT_EN_MASK;
//
//            /* Adding No_motion x axis */
//            data = temp | ((1 << 2) & BMI160_NO_MOTION_Z_INT_EN_MASK);
//        }
//
//        /* write data to Int Enable 2 register */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the interrupt PIN setting for
 * no motion/slow motion interrupt.
 */
//static int8_t config_no_motion_int_settg(const bmi160_int_settg *int_config,
//                                         const bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,
//                                         const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Configure Interrupt pins */
//    rslt = set_intr_pin_config(int_config, dev);
//    if (rslt == BMI160_OK)
//    {
//        rslt = map_feature_interrupt(int_config, dev);
//        if (rslt == BMI160_OK)
//        {
//            rslt = config_no_motion_data_src(no_mot_int_cfg, dev);
//            if (rslt == BMI160_OK)
//            {
//                rslt = config_no_motion_dur_thr(no_mot_int_cfg, dev);
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the source of interrupt for no motion.
 */
//static int8_t config_no_motion_data_src(const bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,
//                                        const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Configure Int data 1 register to add source of interrupt */
//    rslt = bmi160_get_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_MOTION_SRC_INT_MASK;
//        data = temp | ((no_mot_int_cfg->no_motion_src << 7) & BMI160_MOTION_SRC_INT_MASK);
//
//        /* Write data to DATA 1 address */
//        rslt = bmi160_set_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the duration and threshold of
 * no motion/slow motion interrupt along with selection of no/slow motion.
 */
//static int8_t config_no_motion_dur_thr(const bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,
//                                       const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//    uint8_t temp_1 = 0;
//    uint8_t reg_addr;
//    uint8_t data_array[2] = { 0 };
//
//    /* Configuring INT_MOTION register */
//    reg_addr = BMI160_INT_MOTION_0_ADDR;
//    rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_NO_MOTION_INT_DUR_MASK;
//
//        /* Adding no_motion duration */
//        data = temp | ((no_mot_int_cfg->no_motion_dur << 2) & BMI160_NO_MOTION_INT_DUR_MASK);
//
//        /* Write data to NO_MOTION 0 address */
//        rslt = bmi160_set_regs(reg_addr, &data, 1, dev);
//        if (rslt == BMI160_OK)
//        {
//            reg_addr = BMI160_INT_MOTION_3_ADDR;
//            rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
//            if (rslt == BMI160_OK)
//            {
//                temp = data & ~BMI160_NO_MOTION_SEL_BIT_MASK;
//
//                /* Adding no_motion_sel bit */
//                temp_1 = (no_mot_int_cfg->no_motion_sel & BMI160_NO_MOTION_SEL_BIT_MASK);
//                data = (temp | temp_1);
//                data_array[1] = data;
//
//                /* Adding no motion threshold */
//                data_array[0] = no_mot_int_cfg->no_motion_thres;
//                reg_addr = BMI160_INT_MOTION_2_ADDR;
//
//                /* writing data to INT_MOTION 2 and INT_MOTION 3
//                 * address simultaneously */
//                rslt = bmi160_set_regs(reg_addr, data_array, 2, dev);
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API enables the sig-motion motion interrupt.
 */
static int32_t enable_sig_motion_int(BMX160_Object_t *pObj, const bmx160_acc_sig_mot_int_cfg *sig_mot_int_cfg)
{
    int8_t rslt;
    uint8_t data = 0;
    uint8_t temp = 0;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    /* For significant motion,enable any motion x,any motion y,
     * any motion z in Int Enable 0 register */
    rslt = bmx160_read_reg(&(pObj->Ctx),BMX160_INT_ENABLE_0_ADDR, &data, 1);
    if (rslt == BMX160_OK)
    {
        if (sig_mot_int_cfg->sig_en == BMX160_ENABLE)
        {
            temp = data & ~BMX160_SIG_MOTION_INT_EN_MASK;
            data = temp | (7 & BMX160_SIG_MOTION_INT_EN_MASK);

            /* sig-motion feature selected*/
            pObj->any_sig_sel = BMX160_SIG_MOTION_ENABLED;
        }
        else
        {
            data = data & ~BMX160_SIG_MOTION_INT_EN_MASK;

            /* neither any-motion feature nor sig-motion selected */
            pObj->any_sig_sel = BMX160_BOTH_ANY_SIG_MOTION_DISABLED;
        }

        /* write data to Int Enable 0 register */
        rslt = bmx160_write_reg(&(pObj->Ctx),BMX160_INT_ENABLE_0_ADDR, &data, 1, mode); //normal mode
    }

    return rslt;
}

/*!
 * @brief This API configure the interrupt PIN setting for
 * significant motion interrupt.
 */
//static int8_t config_sig_motion_int_settg(const bmi160_int_settg *int_config,
//                                          const bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg,
//                                          const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Configure Interrupt pins */
//    rslt = set_intr_pin_config(int_config, dev);
//    if (rslt == BMI160_OK)
//    {
//        rslt = map_feature_interrupt(int_config, dev);
//        if (rslt == BMI160_OK)
//        {
//            rslt = config_sig_motion_data_src(sig_mot_int_cfg, dev);
//            if (rslt == BMI160_OK)
//            {
//                rslt = config_sig_dur_threshold(sig_mot_int_cfg, dev);
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the source of data(filter & pre-filter)
 * for sig motion interrupt.
 */
//static int8_t config_sig_motion_data_src(const bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg,
//                                         const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Configure Int data 1 register to add source of interrupt */
//    rslt = bmi160_get_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_MOTION_SRC_INT_MASK;
//        data = temp | ((sig_mot_int_cfg->sig_data_src << 7) & BMI160_MOTION_SRC_INT_MASK);
//
//        /* Write data to DATA 1 address */
//        rslt = bmi160_set_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the threshold, skip and proof time of
 * sig motion interrupt.
 */
//static int8_t config_sig_dur_threshold(const bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg,
//                                       const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data;
//    uint8_t temp = 0;
//
//    /* Configuring INT_MOTION registers */
//
//    /* Write significant motion threshold.
//     * This threshold is same as any motion threshold */
//    data = sig_mot_int_cfg->sig_mot_thres;
//
//    /* Write data to INT_MOTION 1 address */
//    rslt = bmi160_set_regs(BMI160_INT_MOTION_1_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        rslt = bmi160_get_regs(BMI160_INT_MOTION_3_ADDR, &data, 1, dev);
//        if (rslt == BMI160_OK)
//        {
//            temp = data & ~BMI160_SIG_MOTION_SKIP_MASK;
//
//            /* adding skip time of sig_motion interrupt*/
//            data = temp | ((sig_mot_int_cfg->sig_mot_skip << 2) & BMI160_SIG_MOTION_SKIP_MASK);
//            temp = data & ~BMI160_SIG_MOTION_PROOF_MASK;
//
//            /* adding proof time of sig_motion interrupt */
//            data = temp | ((sig_mot_int_cfg->sig_mot_proof << 4) & BMI160_SIG_MOTION_PROOF_MASK);
//
//            /* configure the int_sig_mot_sel bit to select
//             * significant motion interrupt */
//            temp = data & ~BMI160_SIG_MOTION_SEL_MASK;
//            data = temp | ((sig_mot_int_cfg->sig_en << 1) & BMI160_SIG_MOTION_SEL_MASK);
//            rslt = bmi160_set_regs(BMI160_INT_MOTION_3_ADDR, &data, 1, dev);
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API enables the step detector interrupt.
 */
//static int8_t enable_step_detect_int(const bmi160_acc_step_detect_int_cfg *step_detect_int_cfg,
//                                     const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Enable data ready interrupt in Int Enable 2 register */
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_STEP_DETECT_INT_EN_MASK;
//        data = temp | ((step_detect_int_cfg->step_detector_en << 3) & BMI160_STEP_DETECT_INT_EN_MASK);
//
//        /* Writing data to INT ENABLE 2 Address */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the step detector parameter.
 */
//static int8_t config_step_detect(const bmi160_acc_step_detect_int_cfg *step_detect_int_cfg,
//                                 const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t temp = 0;
//    uint8_t data_array[2] = { 0 };
//
//    if (step_detect_int_cfg->step_detector_mode == BMI160_STEP_DETECT_NORMAL)
//    {
//        /* Normal mode setting */
//        data_array[0] = 0x15;
//        data_array[1] = 0x03;
//    }
//    else if (step_detect_int_cfg->step_detector_mode == BMI160_STEP_DETECT_SENSITIVE)
//    {
//        /* Sensitive mode setting */
//        data_array[0] = 0x2D;
//        data_array[1] = 0x00;
//    }
//    else if (step_detect_int_cfg->step_detector_mode == BMI160_STEP_DETECT_ROBUST)
//    {
//        /* Robust mode setting */
//        data_array[0] = 0x1D;
//        data_array[1] = 0x07;
//    }
//    else if (step_detect_int_cfg->step_detector_mode == BMI160_STEP_DETECT_USER_DEFINE)
//    {
//        /* Non recommended User defined setting */
//        /* Configuring STEP_CONFIG register */
//        rslt = bmi160_get_regs(BMI160_INT_STEP_CONFIG_0_ADDR, &data_array[0], 2, dev);
//        if (rslt == BMI160_OK)
//        {
//            temp = data_array[0] & ~BMI160_STEP_DETECT_MIN_THRES_MASK;
//
//            /* Adding min_threshold */
//            data_array[0] = temp | ((step_detect_int_cfg->min_threshold << 3) & BMI160_STEP_DETECT_MIN_THRES_MASK);
//            temp = data_array[0] & ~BMI160_STEP_DETECT_STEPTIME_MIN_MASK;
//
//            /* Adding steptime_min */
//            data_array[0] = temp | ((step_detect_int_cfg->steptime_min) & BMI160_STEP_DETECT_STEPTIME_MIN_MASK);
//            temp = data_array[1] & ~BMI160_STEP_MIN_BUF_MASK;
//
//            /* Adding steptime_min */
//            data_array[1] = temp | ((step_detect_int_cfg->step_min_buf) & BMI160_STEP_MIN_BUF_MASK);
//        }
//    }
//
//    /* Write data to STEP_CONFIG register */
//    rslt = bmi160_set_regs(BMI160_INT_STEP_CONFIG_0_ADDR, data_array, 2, dev);
//
//    return rslt;
//}

/*!
 * @brief This API enables the single/double tap interrupt.
 */
//static int8_t enable_tap_int(const bmi160_int_settg *int_config,
//                             const bmi160_acc_tap_int_cfg *tap_int_cfg,
//                             const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Enable single tap or double tap interrupt in Int Enable 0 register */
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        if (int_config->int_type == BMI160_ACC_SINGLE_TAP_INT)
//        {
//            temp = data & ~BMI160_SINGLE_TAP_INT_EN_MASK;
//            data = temp | ((tap_int_cfg->tap_en << 5) & BMI160_SINGLE_TAP_INT_EN_MASK);
//        }
//        else
//        {
//            temp = data & ~BMI160_DOUBLE_TAP_INT_EN_MASK;
//            data = temp | ((tap_int_cfg->tap_en << 4) & BMI160_DOUBLE_TAP_INT_EN_MASK);
//        }
//
//        /* Write to Enable 0 Address */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the interrupt PIN setting for
 * tap interrupt.
 */
//static int8_t config_tap_int_settg(const bmi160_int_settg *int_config,
//                                   const bmi160_acc_tap_int_cfg *tap_int_cfg,
//                                   const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//
//    /* Configure Interrupt pins */
//    rslt = set_intr_pin_config(int_config, dev);
//    if (rslt == BMI160_OK)
//    {
//        rslt = map_feature_interrupt(int_config, dev);
//        if (rslt == BMI160_OK)
//        {
//            rslt = config_tap_data_src(tap_int_cfg, dev);
//            if (rslt == BMI160_OK)
//            {
//                rslt = config_tap_param(int_config, tap_int_cfg, dev);
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the source of data(filter & pre-filter)
 * for tap interrupt.
 */
//static int8_t config_tap_data_src(const bmi160_acc_tap_int_cfg *tap_int_cfg, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Configure Int data 0 register to add source of interrupt */
//    rslt = bmi160_get_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_TAP_SRC_INT_MASK;
//        data = temp | ((tap_int_cfg->tap_data_src << 3) & BMI160_TAP_SRC_INT_MASK);
//
//        /* Write data to Data 0 address */
//        rslt = bmi160_set_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the  parameters of tap interrupt.
 * Threshold, quite, shock, and duration.
 */
//static int8_t config_tap_param(const bmi160_int_settg *int_config,
//                               const bmi160_acc_tap_int_cfg *tap_int_cfg,
//                               const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t temp = 0;
//    uint8_t data = 0;
//    uint8_t data_array[2] = { 0 };
//    uint8_t count = 0;
//    uint8_t dur, shock, quiet, thres;
//
//    /* Configure tap 0 register for tap shock,tap quiet duration
//     * in case of single tap interrupt */
//    rslt = bmi160_get_regs(BMI160_INT_TAP_0_ADDR, data_array, 2, dev);
//    if (rslt == BMI160_OK)
//    {
//        data = data_array[count];
//        if (int_config->int_type == BMI160_ACC_DOUBLE_TAP_INT)
//        {
//            dur = (uint8_t)tap_int_cfg->tap_dur;
//            temp = (data & ~BMI160_TAP_DUR_MASK);
//
//            /* Add tap duration data in case of
//             * double tap interrupt */
//            data = temp | (dur & BMI160_TAP_DUR_MASK);
//        }
//        shock = (uint8_t)tap_int_cfg->tap_shock;
//        temp = data & ~BMI160_TAP_SHOCK_DUR_MASK;
//        data = temp | ((shock << 6) & BMI160_TAP_SHOCK_DUR_MASK);
//        quiet = (uint8_t)tap_int_cfg->tap_quiet;
//        temp = data & ~BMI160_TAP_QUIET_DUR_MASK;
//        data = temp | ((quiet << 7) & BMI160_TAP_QUIET_DUR_MASK);
//        data_array[count++] = data;
//        data = data_array[count];
//        thres = (uint8_t)tap_int_cfg->tap_thr;
//        temp = data & ~BMI160_TAP_THRES_MASK;
//        data = temp | (thres & BMI160_TAP_THRES_MASK);
//        data_array[count++] = data;
//
//        /* TAP 0 and TAP 1 address lie consecutively,
//         * hence writing data to respective registers at one go */
//
//        /* Writing to Tap 0 and Tap 1 Address simultaneously */
//        rslt = bmi160_set_regs(BMI160_INT_TAP_0_ADDR, data_array, count, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the secondary interface.
 */
//static int8_t config_sec_if(const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t if_conf = 0;
//    uint8_t cmd = BMI160_AUX_NORMAL_MODE;
//
//    /* set the aux power mode to normal*/
//    rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &cmd, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        /* 0.5ms delay - refer datasheet table 24*/
//        dev->delay_ms(1);
//        rslt = bmi160_get_regs(BMI160_IF_CONF_ADDR, &if_conf, 1, dev);
//        if_conf |= (uint8_t)(1 << 5);
//        if (rslt == BMI160_OK)
//        {
//            /*enable the secondary interface also*/
//            rslt = bmi160_set_regs(BMI160_IF_CONF_ADDR, &if_conf, 1, dev);
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the ODR of the auxiliary sensor.
 */
static int32_t config_aux_odr(BMX160_Object_t *pObj)
{
    int32_t rslt;
    uint8_t aux_odr;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }

    rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_AUX_ODR_ADDR, &aux_odr, 1);
    if (rslt == BMX160_OK)
    {
        aux_odr = (uint8_t)(pObj->aux_cfg.aux_odr);

        /* Set the secondary interface ODR
         * i.e polling rate of secondary sensor */
        rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_AUX_ODR_ADDR, &aux_odr, 1, mode);
        bmx160_delay_ms(&(pObj->Ctx), BMX160_AUX_COM_DELAY);
        //dev->delay_ms(BMI160_AUX_COM_DELAY);
    }

    return rslt;
}

/*!
 * @brief This API maps the actual burst read length set by user.
 */
static int32_t map_read_len(BMX160_Object_t *pObj, uint16_t *len)
{
    int32_t rslt = BMX160_OK;

    switch (pObj->aux_cfg.aux_rd_burst_len)
    {
        case BMX160_AUX_READ_LEN_0:
            *len = 1;
            break;
        case BMX160_AUX_READ_LEN_1:
            *len = 2;
            break;
        case BMX160_AUX_READ_LEN_2:
            *len = 6;
            break;
        case BMX160_AUX_READ_LEN_3:
            *len = 8;
            break;
        default:
            rslt = BMX160_E_INVALID_INPUT;
            break;
    }

    return rslt;
}

/*!
 * @brief This API configure the settings of auxiliary sensor.
 */
static int32_t config_aux_settg(BMX160_Object_t *pObj)
{
    int8_t rslt;

    rslt = config_sec_if(&(pObj->Ctx));
    if (rslt == BMX160_OK)
    {
        /* Configures the auxiliary interface settings */
        rslt = BMX160_config_mag_mode(pObj);
    }

    return rslt;
}

/*!
 * @brief This API extract the read data from auxiliary sensor.
 */
//static int32_t extract_aux_read(uint16_t map_len, uint8_t reg_addr, uint8_t *aux_data, uint16_t len, const struct bmi160_dev *dev)
//{
//    int32_t rslt = BMI160_OK;
//    uint8_t data[8] = { 0, };
//    uint8_t read_addr = BMI160_AUX_DATA_ADDR;
//    uint8_t count = 0;
//    uint8_t read_count;
//    uint8_t read_len = (uint8_t)map_len;
//
//    for (; count < len;)
//    {
//        /* set address to read */
//        rslt = bmi160_set_regs(BMI160_AUX_IF_2_ADDR, &reg_addr, 1, dev);
//        dev->delay_ms(BMI160_AUX_COM_DELAY);
//        if (rslt == BMI160_OK)
//        {
//            rslt = bmi160_get_regs(read_addr, data, map_len, dev);
//            if (rslt == BMI160_OK)
//            {
//                read_count = 0;
//
//                /* if read len is less the burst read len
//                 * mention by user*/
//                if (len < map_len)
//                {
//                    read_len = (uint8_t)len;
//                }
//                else if ((len - count) < map_len)
//                {
//                    read_len = (uint8_t)(len - count);
//                }
//
//                for (; read_count < read_len; read_count++)
//                {
//                    aux_data[count + read_count] = data[read_count];
//                }
//                reg_addr += (uint8_t)map_len;
//                count += (uint8_t)map_len;
//            }
//            else
//            {
//                rslt = BMI160_E_COM_FAIL;
//                break;
//            }
//        }
//    }
//
//    return rslt;
//}

/*!
 * @brief This API enables the orient interrupt.
 */
//static int8_t enable_orient_int(const bmi160_acc_orient_int_cfg *orient_int_cfg, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Enable data ready interrupt in Int Enable 0 register */
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_ORIENT_INT_EN_MASK;
//        data = temp | ((orient_int_cfg->orient_en << 6) & BMI160_ORIENT_INT_EN_MASK);
//
//        /* write data to Int Enable 0 register */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the necessary setting of orientation interrupt.
 */
//static int8_t config_orient_int_settg(const bmi160_acc_orient_int_cfg *orient_int_cfg,
//                                      const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//    uint8_t data_array[2] = { 0, 0 };
//
//    /* Configuring INT_ORIENT registers */
//    rslt = bmi160_get_regs(BMI160_INT_ORIENT_0_ADDR, data_array, 2, dev);
//    if (rslt == BMI160_OK)
//    {
//        data = data_array[0];
//        temp = data & ~BMI160_ORIENT_MODE_MASK;
//
//        /* Adding Orientation mode */
//        data = temp | ((orient_int_cfg->orient_mode) & BMI160_ORIENT_MODE_MASK);
//        temp = data & ~BMI160_ORIENT_BLOCK_MASK;
//
//        /* Adding Orientation blocking */
//        data = temp | ((orient_int_cfg->orient_blocking << 2) & BMI160_ORIENT_BLOCK_MASK);
//        temp = data & ~BMI160_ORIENT_HYST_MASK;
//
//        /* Adding Orientation hysteresis */
//        data = temp | ((orient_int_cfg->orient_hyst << 4) & BMI160_ORIENT_HYST_MASK);
//        data_array[0] = data;
//        data = data_array[1];
//        temp = data & ~BMI160_ORIENT_THETA_MASK;
//
//        /* Adding Orientation threshold */
//        data = temp | ((orient_int_cfg->orient_theta) & BMI160_ORIENT_THETA_MASK);
//        temp = data & ~BMI160_ORIENT_UD_ENABLE;
//
//        /* Adding Orient_ud_en */
//        data = temp | ((orient_int_cfg->orient_ud_en << 6) & BMI160_ORIENT_UD_ENABLE);
//        temp = data & ~BMI160_AXES_EN_MASK;
//
//        /* Adding axes_en */
//        data = temp | ((orient_int_cfg->axes_ex << 7) & BMI160_AXES_EN_MASK);
//        data_array[1] = data;
//
//        /* Writing data to INT_ORIENT 0 and INT_ORIENT 1
//         * registers simultaneously */
//        rslt = bmi160_set_regs(BMI160_INT_ORIENT_0_ADDR, data_array, 2, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API enables the flat interrupt.
 */
//static int8_t enable_flat_int(const bmi160_acc_flat_detect_int_cfg *flat_int, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Enable flat interrupt in Int Enable 0 register */
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_FLAT_INT_EN_MASK;
//        data = temp | ((flat_int->flat_en << 7) & BMI160_FLAT_INT_EN_MASK);
//
//        /* write data to Int Enable 0 register */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the necessary setting of flat interrupt.
 */
//static int8_t config_flat_int_settg(const bmi160_acc_flat_detect_int_cfg *flat_int, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//    uint8_t data_array[2] = { 0, 0 };
//
//    /* Configuring INT_FLAT register */
//    rslt = bmi160_get_regs(BMI160_INT_FLAT_0_ADDR, data_array, 2, dev);
//    if (rslt == BMI160_OK)
//    {
//        data = data_array[0];
//        temp = data & ~BMI160_FLAT_THRES_MASK;
//
//        /* Adding flat theta */
//        data = temp | ((flat_int->flat_theta) & BMI160_FLAT_THRES_MASK);
//        data_array[0] = data;
//        data = data_array[1];
//        temp = data & ~BMI160_FLAT_HOLD_TIME_MASK;
//
//        /* Adding flat hold time */
//        data = temp | ((flat_int->flat_hold_time << 4) & BMI160_FLAT_HOLD_TIME_MASK);
//        temp = data & ~BMI160_FLAT_HYST_MASK;
//
//        /* Adding flat hysteresis */
//        data = temp | ((flat_int->flat_hy) & BMI160_FLAT_HYST_MASK);
//        data_array[1] = data;
//
//        /* Writing data to INT_FLAT 0 and INT_FLAT 1
//         * registers simultaneously */
//        rslt = bmi160_set_regs(BMI160_INT_FLAT_0_ADDR, data_array, 2, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API enables the Low-g interrupt.
 */
//static int8_t enable_low_g_int(const bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Enable low-g interrupt in Int Enable 1 register */
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_LOW_G_INT_EN_MASK;
//        data = temp | ((low_g_int->low_en << 3) & BMI160_LOW_G_INT_EN_MASK);
//
//        /* write data to Int Enable 0 register */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the source of data(filter & pre-filter)
 * for low-g interrupt.
 */
//static int8_t config_low_g_data_src(const bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Configure Int data 0 register to add source of interrupt */
//    rslt = bmi160_get_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_LOW_HIGH_SRC_INT_MASK;
//        data = temp | ((low_g_int->low_data_src << 7) & BMI160_LOW_HIGH_SRC_INT_MASK);
//
//        /* Write data to Data 0 address */
//        rslt = bmi160_set_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the necessary setting of low-g interrupt.
 */
//static int8_t config_low_g_int_settg(const bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t temp = 0;
//    uint8_t data_array[3] = { 0, 0, 0 };
//
//    /* Configuring INT_LOWHIGH register for low-g interrupt */
//    rslt = bmi160_get_regs(BMI160_INT_LOWHIGH_2_ADDR, &data_array[2], 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data_array[2] & ~BMI160_LOW_G_HYST_MASK;
//
//        /* Adding low-g hysteresis */
//        data_array[2] = temp | (low_g_int->low_hyst & BMI160_LOW_G_HYST_MASK);
//        temp = data_array[2] & ~BMI160_LOW_G_LOW_MODE_MASK;
//
//        /* Adding low-mode */
//        data_array[2] = temp | ((low_g_int->low_mode << 2) & BMI160_LOW_G_LOW_MODE_MASK);
//
//        /* Adding low-g threshold */
//        data_array[1] = low_g_int->low_thres;
//
//        /* Adding low-g interrupt delay */
//        data_array[0] = low_g_int->low_dur;
//
//        /* Writing data to INT_LOWHIGH 0,1,2 registers simultaneously*/
//        rslt = bmi160_set_regs(BMI160_INT_LOWHIGH_0_ADDR, data_array, 3, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API enables the high-g interrupt.
 */
//static int8_t enable_high_g_int(const bmi160_acc_high_g_int_cfg *high_g_int_cfg, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Enable low-g interrupt in Int Enable 1 register */
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        /* Adding high-g X-axis */
//        temp = data & ~BMI160_HIGH_G_X_INT_EN_MASK;
//        data = temp | (high_g_int_cfg->high_g_x & BMI160_HIGH_G_X_INT_EN_MASK);
//
//        /* Adding high-g Y-axis */
//        temp = data & ~BMI160_HIGH_G_Y_INT_EN_MASK;
//        data = temp | ((high_g_int_cfg->high_g_y << 1) & BMI160_HIGH_G_Y_INT_EN_MASK);
//
//        /* Adding high-g Z-axis */
//        temp = data & ~BMI160_HIGH_G_Z_INT_EN_MASK;
//        data = temp | ((high_g_int_cfg->high_g_z << 2) & BMI160_HIGH_G_Z_INT_EN_MASK);
//
//        /* write data to Int Enable 0 register */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

///*!
// * @brief This API configure the source of data(filter & pre-filter)
// * for high-g interrupt.
// */
//static int8_t config_high_g_data_src(const bmi160_acc_high_g_int_cfg *high_g_int_cfg,
//                                     const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//    uint8_t temp = 0;
//
//    /* Configure Int data 0 register to add source of interrupt */
//    rslt = bmi160_get_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data & ~BMI160_LOW_HIGH_SRC_INT_MASK;
//        data = temp | ((high_g_int_cfg->high_data_src << 7) & BMI160_LOW_HIGH_SRC_INT_MASK);
//
//        /* Write data to Data 0 address */
//        rslt = bmi160_set_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}
//
///*!
// * @brief This API configure the necessary setting of high-g interrupt.
// */
//static int8_t config_high_g_int_settg(const bmi160_acc_high_g_int_cfg *high_g_int_cfg,
//                                      const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t temp = 0;
//    uint8_t data_array[3] = { 0, 0, 0 };
//
//    rslt = bmi160_get_regs(BMI160_INT_LOWHIGH_2_ADDR, &data_array[0], 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        temp = data_array[0] & ~BMI160_HIGH_G_HYST_MASK;
//
//        /* Adding high-g hysteresis */
//        data_array[0] = temp | ((high_g_int_cfg->high_hy << 6) & BMI160_HIGH_G_HYST_MASK);
//
//        /* Adding high-g duration */
//        data_array[1] = high_g_int_cfg->high_dur;
//
//        /* Adding high-g threshold */
//        data_array[2] = high_g_int_cfg->high_thres;
//        rslt = bmi160_set_regs(BMI160_INT_LOWHIGH_2_ADDR, data_array, 3, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the behavioural setting of interrupt pin.
 */
//static int8_t config_int_out_ctrl(const bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t temp = 0;
//    uint8_t data = 0;
//
//    /* Configuration of output interrupt signals on pins INT1 and INT2 are
//     * done in BMI160_INT_OUT_CTRL_ADDR register*/
//    rslt = bmi160_get_regs(BMI160_INT_OUT_CTRL_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        /* updating the interrupt pin structure to local structure */
//        const bmi160_int_pin_settg *intr_pin_sett = &(int_config->int_pin_settg);
//
//        /* Configuring channel 1 */
//        if (int_config->int_channel == BMI160_INT_CHANNEL_1)
//        {
//            /* Output enable */
//            temp = data & ~BMI160_INT1_OUTPUT_EN_MASK;
//            data = temp | ((intr_pin_sett->output_en << 3) & BMI160_INT1_OUTPUT_EN_MASK);
//
//            /* Output mode */
//            temp = data & ~BMI160_INT1_OUTPUT_MODE_MASK;
//            data = temp | ((intr_pin_sett->output_mode << 2) & BMI160_INT1_OUTPUT_MODE_MASK);
//
//            /* Output type */
//            temp = data & ~BMI160_INT1_OUTPUT_TYPE_MASK;
//            data = temp | ((intr_pin_sett->output_type << 1) & BMI160_INT1_OUTPUT_TYPE_MASK);
//
//            /* edge control */
//            temp = data & ~BMI160_INT1_EDGE_CTRL_MASK;
//            data = temp | ((intr_pin_sett->edge_ctrl) & BMI160_INT1_EDGE_CTRL_MASK);
//        }
//        else
//        {
//            /* Configuring channel 2 */
//            /* Output enable */
//            temp = data & ~BMI160_INT2_OUTPUT_EN_MASK;
//            data = temp | ((intr_pin_sett->output_en << 7) & BMI160_INT2_OUTPUT_EN_MASK);
//
//            /* Output mode */
//            temp = data & ~BMI160_INT2_OUTPUT_MODE_MASK;
//            data = temp | ((intr_pin_sett->output_mode << 6) & BMI160_INT2_OUTPUT_MODE_MASK);
//
//            /* Output type */
//            temp = data & ~BMI160_INT2_OUTPUT_TYPE_MASK;
//            data = temp | ((intr_pin_sett->output_type << 5) & BMI160_INT2_OUTPUT_TYPE_MASK);
//
//            /* edge control */
//            temp = data & ~BMI160_INT2_EDGE_CTRL_MASK;
//            data = temp | ((intr_pin_sett->edge_ctrl << 4) & BMI160_INT2_EDGE_CTRL_MASK);
//        }
//        rslt = bmi160_set_regs(BMI160_INT_OUT_CTRL_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API configure the mode(input enable, latch or non-latch) of interrupt pin.
 */
//static int8_t config_int_latch(const bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t temp = 0;
//    uint8_t data = 0;
//
//    /* Configuration of latch on pins INT1 and INT2 are done in
//     * BMI160_INT_LATCH_ADDR register*/
//    rslt = bmi160_get_regs(BMI160_INT_LATCH_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        /* updating the interrupt pin structure to local structure */
//        const bmi160_int_pin_settg *intr_pin_sett = &(int_config->int_pin_settg);
//        if (int_config->int_channel == BMI160_INT_CHANNEL_1)
//        {
//            /* Configuring channel 1 */
//            /* Input enable */
//            temp = data & ~BMI160_INT1_INPUT_EN_MASK;
//            data = temp | ((intr_pin_sett->input_en << 4) & BMI160_INT1_INPUT_EN_MASK);
//        }
//        else
//        {
//            /* Configuring channel 2 */
//            /* Input enable */
//            temp = data & ~BMI160_INT2_INPUT_EN_MASK;
//            data = temp | ((intr_pin_sett->input_en << 5) & BMI160_INT2_INPUT_EN_MASK);
//        }
//
//        /* In case of latch interrupt,update the latch duration */
//
//        /* Latching holds the interrupt for the amount of latch
//         * duration time */
//        temp = data & ~BMI160_INT_LATCH_MASK;
//        data = temp | (intr_pin_sett->latch_dur & BMI160_INT_LATCH_MASK);
//
//        /* OUT_CTRL_INT and LATCH_INT address lie consecutively,
//         * hence writing data to respective registers at one go */
//        rslt = bmi160_set_regs(BMI160_INT_LATCH_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 * @brief This API performs the self test for accelerometer of BMI160
 */
static int32_t perform_accel_self_test(BMX160_Object_t *pObj)
{
    int32_t rslt;
    bmx160_sensor_data accel_pos, accel_neg;

    /* Enable Gyro self test bit */
    rslt = enable_accel_self_test(pObj);
    if (rslt == BMX160_OK)
    {
        /* Perform accel self test with positive excitation */
        rslt = accel_self_test_positive_excitation(pObj, &accel_pos);
        if (rslt == BMX160_OK)
        {
            /* Perform accel self test with negative excitation */
            rslt = accel_self_test_negative_excitation(pObj, &accel_neg);
            if (rslt == BMX160_OK)
            {
                /* Validate the self test result */
                rslt = validate_accel_self_test(&accel_pos, &accel_neg);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API enables to perform the accel self test by setting proper
 * configurations to facilitate accel self test
 */
static int32_t enable_accel_self_test(BMX160_Object_t *pObj)
{
    int32_t rslt;
    uint8_t reg_data;
    uint8_t mode = 0;

    /* Set the Accel power mode as normal mode */
    pObj->accel_cfg.power = BMX160_ACCEL_NORMAL_MODE;

    /* Set the sensor range configuration as 8G */
    pObj->accel_cfg.range = BMX160_ACCEL_RANGE_8G;
    rslt = BMX160_set_sens_conf(pObj);
    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }
    if (rslt == BMX160_OK)
    {
        /* Accel configurations are set to facilitate self test
         * acc_odr - 1600Hz ; acc_bwp = 2 ; acc_us = 0 */
        reg_data = BMX160_ACCEL_SELF_TEST_CONFIG;
        rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_ACCEL_CONFIG_ADDR, &reg_data, 1, mode);
    }

    return rslt;
}

static int32_t accel_self_test_positive_excitation(BMX160_Object_t *pObj, bmx160_sensor_data *accel_pos)
{
    int32_t rslt;
    uint8_t reg_data;
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }
    /* Enable accel self test with positive self-test excitation
     * and with amplitude of deflection set as high */
    reg_data = BMX160_ACCEL_SELF_TEST_POSITIVE_EN;
    rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_SELF_TEST_ADDR, &reg_data, 1, mode);
    if (rslt == BMX160_OK)
    {
        /* Read the data after a delay of 50ms - refer datasheet  2.8.1 accel self test*/
        //dev->delay_ms(BMI160_ACCEL_SELF_TEST_DELAY);
    	bmx160_delay_ms(&(pObj->Ctx), BMX160_ACCEL_SELF_TEST_DELAY);
        //rslt = bmx160_get_sensor_data(BMI160_ACCEL_ONLY, accel_pos, NULL, dev);
    	rslt = BMX160_get_sensor_data(pObj, BMX160_ACCEL_ONLY, accel_pos, NULL);
    }

    return rslt;
}

/*!
 * @brief This API performs accel self test with negative excitation
 */
static int32_t accel_self_test_negative_excitation(BMX160_Object_t *pObj, bmx160_sensor_data *accel_neg)
{
    int32_t rslt;
    uint8_t reg_data;
    uint8_t mode = 0;
    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }
    /* Enable accel self test with negative self-test excitation
     * and with amplitude of deflection set as high */
    reg_data = BMX160_ACCEL_SELF_TEST_NEGATIVE_EN;
    rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_SELF_TEST_ADDR, &reg_data, 1, mode); //normal mode
    if (rslt == BMX160_OK)
    {
        /* Read the data after a delay of 50ms */
        //dev->delay_ms(BMI160_ACCEL_SELF_TEST_DELAY);
    	bmx160_delay_ms(&(pObj->Ctx), BMX160_ACCEL_SELF_TEST_DELAY);
        //rslt = bmi160_get_sensor_data(BMI160_ACCEL_ONLY, accel_neg, NULL, dev);
    	rslt = BMX160_get_sensor_data(pObj, BMX160_ACCEL_ONLY, accel_neg, NULL);
    }

    return rslt;
}

/*!
 * @brief This API performs the self test for gyroscope of BMI160
 */
static int32_t perform_gyro_self_test(BMX160_Object_t *pObj)
{
    int32_t rslt;

    /* Enable Gyro self test bit */
    rslt = enable_gyro_self_test(&(pObj->Ctx));
    if (rslt == BMX160_OK)
    {
        /* Validate the gyro self test a delay of 50ms */
        //dev->delay_ms(50);
    	bmx160_delay_ms(&(pObj->Ctx), 50);

        /* Validate the gyro self test results */
        rslt = validate_gyro_self_test(&(pObj->Ctx));
    }

    return rslt;
}

///*!
// *  @brief This API sets FIFO full interrupt of the sensor.This interrupt
// *  occurs when the FIFO is full and the next full data sample would cause
// *  a FIFO overflow, which may delete the old samples.
// */
//static int8_t set_fifo_full_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt = BMI160_OK;
//
//    /* Null-pointer check */
//    if ((dev == NULL) || (dev->delay_ms == NULL))
//    {
//        rslt = BMI160_E_NULL_PTR;
//    }
//    else
//    {
//        /*enable the fifo full interrupt */
//        rslt = enable_fifo_full_int(int_config, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Configure Interrupt pins */
//            rslt = set_intr_pin_config(ctx, int_config);
//            if (rslt == BMI160_OK)
//            {
//                rslt = map_hardware_interrupt(ctx, int_config);
//            }
//        }
//    }
//
//    return rslt;
//}

///*!
// * @brief This enable the FIFO full interrupt engine.
// */
//static int8_t enable_fifo_full_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        data = BMI160_SET_BITS(data, BMI160_FIFO_FULL_INT, int_config->fifo_full_int_en);
//
//        /* Writing data to INT ENABLE 1 Address */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

///*!
// *  @brief This API sets FIFO watermark interrupt of the sensor.The FIFO
// *  watermark interrupt is fired, when the FIFO fill level is above a fifo
// *  watermark.
// */
//static int8_t set_fifo_watermark_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt = BMI160_OK;
//
//    if ((dev == NULL) || (dev->delay_ms == NULL))
//    {
//        rslt = BMI160_E_NULL_PTR;
//    }
//    else
//    {
//        /* Enable fifo-watermark interrupt in Int Enable 1 register */
//        rslt = enable_fifo_wtm_int(int_config, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Configure Interrupt pins */
//            rslt = set_intr_pin_config(ctx, int_config, dev);
//            if (rslt == BMI160_OK)
//            {
//                rslt = map_hardware_interrupt(ctx, int_config);
//            }
//        }
//    }
//
//    return rslt;
//}

///*!
// * @brief This enable the FIFO watermark interrupt engine.
// */
//static int8_t enable_fifo_wtm_int(const bmi160_int_settg *int_config, const struct bmi160_dev *dev)
//{
//    int8_t rslt;
//    uint8_t data = 0;
//
//    rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        data = BMI160_SET_BITS(data, BMI160_FIFO_WTM_INT, int_config->fifo_wtm_int_en);
//
//        /* Writing data to INT ENABLE 1 Address */
//        rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
//    }
//
//    return rslt;
//}

/*!
 *  @brief This API is used to reset the FIFO related configurations
 *  in the fifo_frame structure.
 */
static void reset_fifo_data_structure(BMX160_Object_t *pObj)
{
    /*Prepare for next FIFO read by resetting FIFO's
     * internal data structures*/
	pObj->fifo->accel_byte_start_idx = 0;
	pObj->fifo->gyro_byte_start_idx = 0;
	pObj->fifo->aux_byte_start_idx = 0;
	pObj->fifo->sensor_time = 0;
	pObj->fifo->skipped_frame_count = 0;
}

/*!
 *  @brief This API is used to read fifo_byte_counter value (i.e)
 *  current fill-level in Fifo buffer.
 */
//static int8_t get_fifo_byte_counter(uint16_t *bytes_to_read, struct bmi160_dev const *dev)
//{
//    int8_t rslt = 0;
//    uint8_t data[2];
//    uint8_t addr = BMI160_FIFO_LENGTH_ADDR;
//
//    rslt |= bmi160_get_regs(addr, data, 2, dev);
//    data[1] = data[1] & BMI160_FIFO_BYTE_COUNTER_MASK;
//
//    /* Available data in FIFO is stored in bytes_to_read*/
//    *bytes_to_read = (((uint16_t)data[1] << 8) | ((uint16_t)data[0]));
//
//    return rslt;
//}

/*!
 *  @brief This API is used to compute the number of bytes of accel FIFO data
 *  which is to be parsed in header-less mode
 */
static void get_accel_len_to_parse(BMX160_Object_t *pObj, uint16_t *data_index, uint16_t *data_read_length, const uint8_t *acc_frame_count)
{
    /* Data start index */
    *data_index = pObj->fifo->accel_byte_start_idx;
    if (pObj->fifo->fifo_data_enable == BMX160_FIFO_A_ENABLE)
    {
        *data_read_length = (*acc_frame_count) * BMX160_FIFO_A_LENGTH;
    }
    else if (pObj->fifo->fifo_data_enable == BMX160_FIFO_G_A_ENABLE)
    {
        *data_read_length = (*acc_frame_count) * BMX160_FIFO_GA_LENGTH;
    }
    else if (pObj->fifo->fifo_data_enable == BMX160_FIFO_M_A_ENABLE)
    {
        *data_read_length = (*acc_frame_count) * BMX160_FIFO_MA_LENGTH;
    }
    else if (pObj->fifo->fifo_data_enable == BMX160_FIFO_M_G_A_ENABLE)
    {
        *data_read_length = (*acc_frame_count) * BMX160_FIFO_MGA_LENGTH;
    }
    else
    {
        /* When accel is not enabled ,there will be no accel data.
         * so we update the data index as complete */
        *data_index = pObj->fifo->length;
    }
    if (*data_read_length > pObj->fifo->length)
    {
        /* Handling the case where more data is requested
         * than that is available*/
        *data_read_length = pObj->fifo->length;
    }
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static void unpack_accel_frame(BMX160_Object_t *pObj, bmx160_sensor_data *acc, uint16_t *idx, uint8_t *acc_idx, uint8_t frame_info)
{
    switch (frame_info)
    {
        case BMX160_FIFO_HEAD_A:
        case BMX160_FIFO_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMX160_FIFO_A_LENGTH) > pObj->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = pObj->fifo->length;
                break;
            }

            /*Unpack the data array into the structure instance "acc" */
            unpack_accel_data(pObj, &acc[*acc_idx], *idx);

            /*Move the data index*/
            *idx = *idx + BMX160_FIFO_A_LENGTH;
            (*acc_idx)++;
            break;
        case BMX160_FIFO_HEAD_G_A:
        case BMX160_FIFO_G_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMX160_FIFO_GA_LENGTH) > pObj->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = pObj->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "acc"*/
            unpack_accel_data(pObj, &acc[*acc_idx], *idx + BMX160_FIFO_G_LENGTH);

            /*Move the data index*/
            *idx = *idx + BMX160_FIFO_GA_LENGTH;
            (*acc_idx)++;
            break;
        case BMX160_FIFO_HEAD_M_A:
        case BMX160_FIFO_M_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMX160_FIFO_MA_LENGTH) > pObj->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = pObj->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "acc"*/
            unpack_accel_data(pObj, &acc[*acc_idx], *idx + BMX160_FIFO_M_LENGTH);

            /*Move the data index*/
            *idx = *idx + BMX160_FIFO_MA_LENGTH;
            (*acc_idx)++;
            break;
        case BMX160_FIFO_HEAD_M_G_A:
        case BMX160_FIFO_M_G_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMX160_FIFO_MGA_LENGTH) > pObj->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = pObj->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "acc"*/
            unpack_accel_data(pObj, &acc[*acc_idx], *idx + BMX160_FIFO_MG_LENGTH);

            /*Move the data index*/
            *idx = *idx + BMX160_FIFO_MGA_LENGTH;
            (*acc_idx)++;
            break;
        case BMX160_FIFO_HEAD_M:
        case BMX160_FIFO_M_ENABLE:
            (*idx) = (*idx) + BMX160_FIFO_M_LENGTH;
            break;
        case BMX160_FIFO_HEAD_G:
        case BMX160_FIFO_G_ENABLE:
            (*idx) = (*idx) + BMX160_FIFO_G_LENGTH;
            break;
        case BMX160_FIFO_HEAD_M_G:
        case BMX160_FIFO_M_G_ENABLE:
            (*idx) = (*idx) + BMX160_FIFO_MG_LENGTH;
            break;
        default:
            break;
    }
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data and store it in the instance of the structure bmi160_sensor_data.
 */
static void unpack_accel_data(BMX160_Object_t *pObj, bmx160_sensor_data *accel_data, uint16_t data_start_index)
{
    uint16_t data_lsb;
    uint16_t data_msb;

    /* Accel raw x data */
    data_lsb = pObj->fifo->data[data_start_index++];
    data_msb = pObj->fifo->data[data_start_index++];
    accel_data->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Accel raw y data */
    data_lsb = pObj->fifo->data[data_start_index++];
    data_msb = pObj->fifo->data[data_start_index++];
    accel_data->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Accel raw z data */
    data_lsb = pObj->fifo->data[data_start_index++];
    data_msb = pObj->fifo->data[data_start_index++];
    accel_data->z = (int16_t)((data_msb << 8) | data_lsb);
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in header mode.
 */
static void extract_accel_header_mode(BMX160_Object_t *pObj, bmx160_sensor_data *accel_data, uint8_t *accel_length)
{
    uint8_t frame_header = 0;
    uint16_t data_index;
    uint8_t accel_index = 0;

    for (data_index = pObj->fifo->accel_byte_start_idx; data_index < pObj->fifo->length;)
    {
        /* extracting Frame header */
        frame_header = (pObj->fifo->data[data_index] & BMX160_FIFO_TAG_INTR_MASK);

        /*Index is moved to next byte where the data is starting*/
        data_index++;
        switch (frame_header)
        {
            /* Accel frame */
            case BMX160_FIFO_HEAD_A:
            case BMX160_FIFO_HEAD_M_A:
            case BMX160_FIFO_HEAD_G_A:
            case BMX160_FIFO_HEAD_M_G_A:
                unpack_accel_frame(pObj, accel_data, &data_index, &accel_index, frame_header);
                break;
            case BMX160_FIFO_HEAD_M:
                move_next_frame(pObj, &data_index, BMX160_FIFO_M_LENGTH);
                break;
            case BMX160_FIFO_HEAD_G:
                move_next_frame(pObj, &data_index, BMX160_FIFO_G_LENGTH);
                break;
            case BMX160_FIFO_HEAD_M_G:
                move_next_frame(pObj, &data_index, BMX160_FIFO_MG_LENGTH);
                break;

            /* Sensor time frame */
            case BMX160_FIFO_HEAD_SENSOR_TIME:
                unpack_sensortime_frame(pObj, &data_index);
                break;

            /* Skip frame */
            case BMX160_FIFO_HEAD_SKIP_FRAME:
                unpack_skipped_frame(pObj, &data_index);
                break;

            /* Input config frame */
            case BMX160_FIFO_HEAD_INPUT_CONFIG:
                move_next_frame(pObj, &data_index, 1);
                break;
            case BMX160_FIFO_HEAD_OVER_READ:

                /* Update the data index as complete in case of Over read */
                data_index = pObj->fifo->length;
                break;
            default:
                break;
        }
        if (*accel_length == accel_index)
        {
            /* Number of frames to read completed */
            break;
        }
    }

    /*Update number of accel data read*/
    *accel_length = accel_index;

    /*Update the accel frame index*/
    pObj->fifo->accel_byte_start_idx = data_index;
}

/*!
 *  @brief This API computes the number of bytes of gyro FIFO data
 *  which is to be parsed in header-less mode
 */
static void get_gyro_len_to_parse(BMX160_Object_t *pObj, uint16_t *data_index,  uint16_t *data_read_length, const uint8_t *gyro_frame_count)
{
    /* Data start index */
    *data_index = pObj->fifo->gyro_byte_start_idx;
    if (pObj->fifo->fifo_data_enable == BMX160_FIFO_G_ENABLE)
    {
        *data_read_length = (*gyro_frame_count) * BMX160_FIFO_G_LENGTH;
    }
    else if (pObj->fifo->fifo_data_enable == BMX160_FIFO_G_A_ENABLE)
    {
        *data_read_length = (*gyro_frame_count) * BMX160_FIFO_GA_LENGTH;
    }
    else if (pObj->fifo->fifo_data_enable == BMX160_FIFO_M_G_ENABLE)
    {
        *data_read_length = (*gyro_frame_count) * BMX160_FIFO_MG_LENGTH;
    }
    else if (pObj->fifo->fifo_data_enable == BMX160_FIFO_M_G_A_ENABLE)
    {
        *data_read_length = (*gyro_frame_count) * BMX160_FIFO_MGA_LENGTH;
    }
    else
    {
        /* When gyro is not enabled ,there will be no gyro data.
         * so we update the data index as complete */
        *data_index = pObj->fifo->length;
    }
    if (*data_read_length > pObj->fifo->length)
    {
        /* Handling the case where more data is requested
         * than that is available*/
        *data_read_length = pObj->fifo->length;
    }
}

/*!
 *  @brief This API is used to parse the gyroscope's data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static void unpack_gyro_frame(BMX160_Object_t *pObj, bmx160_sensor_data *gyro, uint16_t *idx, uint8_t *gyro_idx, uint8_t frame_info)
{
    switch (frame_info)
    {
        case BMX160_FIFO_HEAD_G:
        case BMX160_FIFO_G_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMX160_FIFO_G_LENGTH) > pObj->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = pObj->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "gyro"*/
            unpack_gyro_data(pObj, &gyro[*gyro_idx], *idx);

            /*Move the data index*/
            (*idx) = (*idx) + BMX160_FIFO_G_LENGTH;
            (*gyro_idx)++;
            break;
        case BMX160_FIFO_HEAD_G_A:
        case BMX160_FIFO_G_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMX160_FIFO_GA_LENGTH) > pObj->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = pObj->fifo->length;
                break;
            }

            /* Unpack the data array into structure instance "gyro" */
            unpack_gyro_data(pObj, &gyro[*gyro_idx], *idx);

            /* Move the data index */
            *idx = *idx + BMX160_FIFO_GA_LENGTH;
            (*gyro_idx)++;
            break;
        case BMX160_FIFO_HEAD_M_G_A:
        case BMX160_FIFO_M_G_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMX160_FIFO_MGA_LENGTH) > pObj->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = pObj->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "gyro"*/
            unpack_gyro_data(pObj, &gyro[*gyro_idx], *idx + BMX160_FIFO_M_LENGTH);

            /*Move the data index*/
            *idx = *idx + BMX160_FIFO_MGA_LENGTH;
            (*gyro_idx)++;
            break;
        case BMX160_FIFO_HEAD_M_A:
        case BMX160_FIFO_M_A_ENABLE:

            /* Move the data index */
            *idx = *idx + BMX160_FIFO_MA_LENGTH;
            break;
        case BMX160_FIFO_HEAD_M:
        case BMX160_FIFO_M_ENABLE:
            (*idx) = (*idx) + BMX160_FIFO_M_LENGTH;
            break;
        case BMX160_FIFO_HEAD_M_G:
        case BMX160_FIFO_M_G_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMX160_FIFO_MG_LENGTH) > pObj->fifo->length)
            {
                /*Update the data index as complete*/
                *idx = pObj->fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "gyro"*/
            unpack_gyro_data(pObj, &gyro[*gyro_idx], *idx + BMX160_FIFO_M_LENGTH);

            /*Move the data index*/
            (*idx) = (*idx) + BMX160_FIFO_MG_LENGTH;
            (*gyro_idx)++;
            break;
        case BMX160_FIFO_HEAD_A:
        case BMX160_FIFO_A_ENABLE:

            /*Move the data index*/
            *idx = *idx + BMX160_FIFO_A_LENGTH;
            break;
        default:
            break;
    }
}

/*!
 *  @brief This API is used to parse the gyro data from the
 *  FIFO data and store it in the instance of the structure bmi160_sensor_data.
 */
static void unpack_gyro_data(BMX160_Object_t *pObj, bmx160_sensor_data *gyro_data, uint16_t data_start_index)
{
    uint16_t data_lsb;
    uint16_t data_msb;

    /* Gyro raw x data */
    data_lsb = pObj->fifo->data[data_start_index++];
    data_msb = pObj->fifo->data[data_start_index++];
    gyro_data->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyro raw y data */
    data_lsb = pObj->fifo->data[data_start_index++];
    data_msb = pObj->fifo->data[data_start_index++];
    gyro_data->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyro raw z data */
    data_lsb = pObj->fifo->data[data_start_index++];
    data_msb = pObj->fifo->data[data_start_index++];
    gyro_data->z = (int16_t)((data_msb << 8) | data_lsb);
}

/*!
 *  @brief This API is used to parse the gyro data from the
 *  FIFO data in header mode.
 */
static void extract_gyro_header_mode(BMX160_Object_t *pObj, bmx160_sensor_data *gyro_data, uint8_t *gyro_length)
{
    uint8_t frame_header = 0;
    uint16_t data_index;
    uint8_t gyro_index = 0;

    for (data_index = pObj->fifo->gyro_byte_start_idx; data_index < pObj->fifo->length;)
    {
        /* extracting Frame header */
        frame_header = (pObj->fifo->data[data_index] & BMX160_FIFO_TAG_INTR_MASK);

        /*Index is moved to next byte where the data is starting*/
        data_index++;
        switch (frame_header)
        {
            /* GYRO frame */
            case BMX160_FIFO_HEAD_G:
            case BMX160_FIFO_HEAD_G_A:
            case BMX160_FIFO_HEAD_M_G:
            case BMX160_FIFO_HEAD_M_G_A:
                unpack_gyro_frame(pObj, gyro_data, &data_index, &gyro_index, frame_header);
                break;
            case BMX160_FIFO_HEAD_A:
                move_next_frame(pObj, &data_index, BMX160_FIFO_A_LENGTH);
                break;
            case BMX160_FIFO_HEAD_M:
                move_next_frame(pObj, &data_index, BMX160_FIFO_M_LENGTH);
                break;
            case BMX160_FIFO_HEAD_M_A:
                move_next_frame(pObj, &data_index, BMX160_FIFO_M_LENGTH);
                break;

            /* Sensor time frame */
            case BMX160_FIFO_HEAD_SENSOR_TIME:
                unpack_sensortime_frame(pObj, &data_index);
                break;

            /* Skip frame */
            case BMX160_FIFO_HEAD_SKIP_FRAME:
                unpack_skipped_frame(pObj, &data_index);
                break;

            /* Input config frame */
            case BMX160_FIFO_HEAD_INPUT_CONFIG:
                move_next_frame(pObj, &data_index, 1);
                break;
            case BMX160_FIFO_HEAD_OVER_READ:

                /* Update the data index as complete in case of over read */
                data_index = pObj->fifo->length;
                break;
            default:
                break;
        }
        if (*gyro_length == gyro_index)
        {
            /*Number of frames to read completed*/
            break;
        }
    }

    /*Update number of gyro data read*/
    *gyro_length = gyro_index;

    /*Update the gyro frame index*/
    pObj->fifo->gyro_byte_start_idx = data_index;
}

/*!
 *  @brief This API computes the number of bytes of aux FIFO data
 *  which is to be parsed in header-less mode
 */
//static void get_aux_len_to_parse(uint16_t *data_index, uint16_t *data_read_length, const uint8_t *aux_frame_count, const struct bmi160_dev *dev)
static void get_aux_len_to_parse(BMX160_Object_t *pObj, uint16_t *data_index, uint16_t *data_read_length, const uint8_t *aux_frame_count)
{
    /* Data start index */
    *data_index = pObj->fifo->gyro_byte_start_idx;
    if (pObj->fifo->fifo_data_enable == BMX160_FIFO_M_ENABLE)
    {
        *data_read_length = (*aux_frame_count) * BMX160_FIFO_M_LENGTH;
    }
    else if (pObj->fifo->fifo_data_enable == BMX160_FIFO_M_A_ENABLE)
    {
        *data_read_length = (*aux_frame_count) * BMX160_FIFO_MA_LENGTH;
    }
    else if (pObj->fifo->fifo_data_enable == BMX160_FIFO_M_G_ENABLE)
    {
        *data_read_length = (*aux_frame_count) * BMX160_FIFO_MG_LENGTH;
    }
    else if (pObj->fifo->fifo_data_enable == BMX160_FIFO_M_G_A_ENABLE)
    {
        *data_read_length = (*aux_frame_count) * BMX160_FIFO_MGA_LENGTH;
    }
    else
    {
        /* When aux is not enabled ,there will be no aux data.
         * so we update the data index as complete */
        *data_index = pObj->fifo->length;
    }
    if (*data_read_length > pObj->fifo->length)
    {
        /* Handling the case where more data is requested
         * than that is available */
        *data_read_length = pObj->fifo->length;
    }
}

/*!
 *  @brief This API is used to parse the aux's data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed
 */
static void unpack_aux_frame(BMX160_Object_t *pObj, bmx160_aux_data *aux_data, uint16_t *idx, uint8_t *aux_index, uint8_t frame_info)
{
    switch (frame_info)
    {
        case BMX160_FIFO_HEAD_M:
        case BMX160_FIFO_M_ENABLE:

            /* Partial read, then skip the data */
            if ((*idx + BMX160_FIFO_M_LENGTH) > pObj->fifo->length)
            {
                /* Update the data index as complete */
                *idx = pObj->fifo->length;
                break;
            }

            /* Unpack the data array into structure instance */
            unpack_aux_data(pObj, &aux_data[*aux_index], *idx);

            /* Move the data index */
            *idx = *idx + BMX160_FIFO_M_LENGTH;
            (*aux_index)++;
            break;
        case BMX160_FIFO_HEAD_M_A:
        case BMX160_FIFO_M_A_ENABLE:

            /* Partial read, then skip the data */
            if ((*idx + BMX160_FIFO_MA_LENGTH) > pObj->fifo->length)
            {
                /* Update the data index as complete */
                *idx = pObj->fifo->length;
                break;
            }

            /* Unpack the data array into structure instance */
            unpack_aux_data(pObj, &aux_data[*aux_index], *idx);

            /* Move the data index */
            *idx = *idx + BMX160_FIFO_MA_LENGTH;
            (*aux_index)++;
            break;
        case BMX160_FIFO_HEAD_M_G:
        case BMX160_FIFO_M_G_ENABLE:

            /* Partial read, then skip the data */
            if ((*idx + BMX160_FIFO_MG_LENGTH) > pObj->fifo->length)
            {
                /* Update the data index as complete */
                *idx = pObj->fifo->length;
                break;
            }

            /* Unpack the data array into structure instance */
            unpack_aux_data(pObj, &aux_data[*aux_index], *idx);

            /* Move the data index */
            (*idx) = (*idx) + BMX160_FIFO_MG_LENGTH;
            (*aux_index)++;
            break;
        case BMX160_FIFO_HEAD_M_G_A:
        case BMX160_FIFO_M_G_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMX160_FIFO_MGA_LENGTH) > pObj->fifo->length)
            {
                /* Update the data index as complete */
                *idx = pObj->fifo->length;
                break;
            }

            /* Unpack the data array into structure instance */
            unpack_aux_data(pObj, &aux_data[*aux_index], *idx);

            /*Move the data index*/
            *idx = *idx + BMX160_FIFO_MGA_LENGTH;
            (*aux_index)++;
            break;
        case BMX160_FIFO_HEAD_G:
        case BMX160_FIFO_G_ENABLE:

            /* Move the data index */
            (*idx) = (*idx) + BMX160_FIFO_G_LENGTH;
            break;
        case BMX160_FIFO_HEAD_G_A:
        case BMX160_FIFO_G_A_ENABLE:

            /* Move the data index */
            *idx = *idx + BMX160_FIFO_GA_LENGTH;
            break;
        case BMX160_FIFO_HEAD_A:
        case BMX160_FIFO_A_ENABLE:

            /* Move the data index */
            *idx = *idx + BMX160_FIFO_A_LENGTH;
            break;
        default:
            break;
    }
}

/*!
 *  @brief This API is used to parse the aux data from the
 *  FIFO data and store it in the instance of the structure bmi160_aux_data.
 */
static void unpack_aux_data(BMX160_Object_t *pObj, bmx160_aux_data *aux_data, uint16_t data_start_index)
{
    /* Aux data bytes */
    aux_data->data[0] = pObj->fifo->data[data_start_index++];
    aux_data->data[1] = pObj->fifo->data[data_start_index++];
    aux_data->data[2] = pObj->fifo->data[data_start_index++];
    aux_data->data[3] = pObj->fifo->data[data_start_index++];
    aux_data->data[4] = pObj->fifo->data[data_start_index++];
    aux_data->data[5] = pObj->fifo->data[data_start_index++];
    aux_data->data[6] = pObj->fifo->data[data_start_index++];
    aux_data->data[7] = pObj->fifo->data[data_start_index++];
}

/*!
 *  @brief This API is used to parse the aux data from the
 *  FIFO data in header mode.
 */
static void extract_aux_header_mode(BMX160_Object_t *pObj, bmx160_aux_data *aux_data, uint8_t *aux_length)
{
    uint8_t frame_header = 0;
    uint16_t data_index;
    uint8_t aux_index = 0;

    for (data_index = pObj->fifo->aux_byte_start_idx; data_index < pObj->fifo->length;)
    {
        /* extracting Frame header */
        frame_header = (pObj->fifo->data[data_index] & BMX160_FIFO_TAG_INTR_MASK);

        /*Index is moved to next byte where the data is starting*/
        data_index++;
        switch (frame_header)
        {
            /* Aux frame */
            case BMX160_FIFO_HEAD_M:
            case BMX160_FIFO_HEAD_M_A:
            case BMX160_FIFO_HEAD_M_G:
            case BMX160_FIFO_HEAD_M_G_A:
                unpack_aux_frame(pObj, aux_data, &data_index, &aux_index, frame_header);
                break;
            case BMX160_FIFO_HEAD_G:
                move_next_frame(pObj, &data_index, BMX160_FIFO_G_LENGTH);
                break;
            case BMX160_FIFO_HEAD_G_A:
                move_next_frame(pObj, &data_index, BMX160_FIFO_GA_LENGTH);
                break;
            case BMX160_FIFO_HEAD_A:
                move_next_frame(pObj, &data_index, BMX160_FIFO_A_LENGTH);
                break;

            /* Sensor time frame */
            case BMX160_FIFO_HEAD_SENSOR_TIME:
                unpack_sensortime_frame(pObj, &data_index);
                break;

            /* Skip frame */
            case BMX160_FIFO_HEAD_SKIP_FRAME:
                unpack_skipped_frame(pObj, &data_index);
                break;

            /* Input config frame */
            case BMX160_FIFO_HEAD_INPUT_CONFIG:
                move_next_frame(pObj, &data_index, 1);
                break;
            case BMX160_FIFO_HEAD_OVER_READ:

                /* Update the data index as complete in case
                 * of over read */
                data_index = pObj->fifo->length;
                break;
            default:

                /* Update the data index as complete in case of
                 * getting other headers like 0x00 */
                data_index = pObj->fifo->length;
                break;
        }
        if (*aux_length == aux_index)
        {
            /*Number of frames to read completed*/
            break;
        }
    }

    /* Update number of aux data read */
    *aux_length = aux_index;

    /* Update the aux frame index */
    pObj->fifo->aux_byte_start_idx = data_index;
}

/*!
 *  @brief This API checks the presence of non-valid frames in the read fifo data.
 */
static void check_frame_validity(BMX160_Object_t *pObj, uint16_t *data_index)
{
    if ((*data_index + 2) < pObj->fifo->length)
    {
        /* Check if FIFO is empty */
        if ((pObj->fifo->data[*data_index] == FIFO_CONFIG_MSB_CHECK) &&
            (pObj->fifo->data[*data_index + 1] == FIFO_CONFIG_LSB_CHECK))
        {
            /*Update the data index as complete*/
            *data_index = pObj->fifo->length;
        }
    }
}

/*!
 *  @brief This API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the user specified data.
 */
static void move_next_frame(BMX160_Object_t *pObj, uint16_t *data_index, uint8_t current_frame_length)
{
    /*Partial read, then move the data index to last data*/
    if ((*data_index + current_frame_length) > pObj->fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = pObj->fifo->length;
    }
    else
    {
        /*Move the data index to next frame*/
        *data_index = *data_index + current_frame_length;
    }
}

/*!
 *  @brief This API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance dev.
 */
static void unpack_sensortime_frame(BMX160_Object_t *pObj, uint16_t *data_index)
{
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /*Partial read, then move the data index to last data*/
    if ((*data_index + BMX160_SENSOR_TIME_LENGTH) > pObj->fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = pObj->fifo->length;
    }
    else
    {
        sensor_time_byte3 = pObj->fifo->data[(*data_index) + BMX160_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = pObj->fifo->data[(*data_index) + BMX160_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = pObj->fifo->data[(*data_index)];

        /* Sensor time */
        pObj->fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);
        *data_index = (*data_index) + BMX160_SENSOR_TIME_LENGTH;
    }
}

/*!
 *  @brief This API is used to parse and store the skipped_frame_count from
 *  the FIFO data in the structure instance dev.
 */
static void unpack_skipped_frame(BMX160_Object_t *pObj, uint16_t *data_index)
{
    /*Partial read, then move the data index to last data*/
    if (*data_index >= pObj->fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = pObj->fifo->length;
    }
    else
    {
    	pObj->fifo->skipped_frame_count = pObj->fifo->data[*data_index];

        /*Move the data index*/
        *data_index = (*data_index) + 1;
    }
}

///*!
// *  @brief This API is used to get the FOC status from the sensor
// */
//static int8_t get_foc_status(uint8_t *foc_status, struct bmi160_dev const *dev)
//{
//    int8_t rslt;
//    uint8_t data;
//
//    /* Read the FOC status from sensor */
//    rslt = bmi160_get_regs(BMI160_STATUS_ADDR, &data, 1, dev);
//    if (rslt == BMI160_OK)
//    {
//        /* Get the foc_status bit */
//        *foc_status = BMI160_GET_BITS(data, BMI160_FOC_STATUS);
//    }
//
//    return rslt;
//}

/*!
 *  @brief This API is used to configure the offset enable bits in the sensor
 */
//static int8_t configure_offset_enable(const bmi160_foc_conf *foc_conf, struct bmi160_dev const *dev)
//{
//    int8_t rslt;
//    uint8_t data;
//
//    /* Null-pointer check */
//    rslt = null_ptr_check(dev);
//    if (rslt != BMI160_OK)
//    {
//        rslt = BMI160_E_NULL_PTR;
//    }
//    else
//    {
//        /* Read the FOC config from the sensor */
//        rslt = bmi160_get_regs(BMI160_OFFSET_CONF_ADDR, &data, 1, dev);
//        if (rslt == BMI160_OK)
//        {
//            /* Set the offset enable/disable for gyro */
//            data = BMI160_SET_BITS(data, BMI160_GYRO_OFFSET_EN, foc_conf->gyro_off_en);
//
//            /* Set the offset enable/disable for accel */
//            data = BMI160_SET_BITS(data, BMI160_ACCEL_OFFSET_EN, foc_conf->acc_off_en);
//
//            /* Set the offset config in the sensor */
//            rslt = bmi160_set_regs(BMI160_OFFSET_CONF_ADDR, &data, 1, dev);
//        }
//    }
//
//    return rslt;
//}
static int32_t trigger_foc(BMX160_Object_t *pObj, bmx160_offsets *offset)
{
    int32_t rslt;
    uint8_t foc_status;
    uint8_t cmd = BMI160_START_FOC_CMD;
    uint8_t timeout = 0;
    uint8_t data_array[20];
    uint8_t mode = 0;

    if((pObj->prev_accel_cfg.power == BMX160_ACCEL_NORMAL_MODE) || (pObj->prev_gyro_cfg.power == BMX160_ACCEL_NORMAL_MODE)){
        mode = 1;
    }
    /* Start the FOC process */
    rslt = bmx160_write_reg(&(pObj->Ctx), BMX160_COMMAND_REG_ADDR, &cmd, 1, mode); //normal mode
    if (rslt == BMX160_OK)
    {
        /* Check the FOC status*/
        rslt = get_foc_status(&(pObj->Ctx), &foc_status);
        if ((rslt != BMX160_OK) || (foc_status != BMX160_ENABLE))
        {
            while ((foc_status != BMX160_ENABLE) && (timeout < 11))
            {
                /* Maximum time of 250ms is given in 10
                 * steps of 25ms each - 250ms refer datasheet 2.9.1 */
                //dev->delay_ms(25);
            	bmx160_delay_ms(&(pObj->Ctx), 25);

                /* Check the FOC status*/
                rslt = get_foc_status(&(pObj->Ctx), &foc_status);
                timeout++;
            }
            if ((rslt == BMX160_OK) && (foc_status == BMX160_ENABLE))
            {
                /* Get offset values from sensor */
                rslt = BMX160_get_offsets(pObj, offset);
            }
            else
            {
                /* FOC failure case */
                rslt = BMX160_FOC_FAILURE;
            }
        }
        if (rslt == BMX160_OK)
        {
            /* Read registers 0x04-0x17 */
            rslt = bmx160_read_reg(&(pObj->Ctx), BMX160_GYRO_DATA_ADDR, data_array, 20);
        }
    }

    return rslt;
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

  if (pObj->IO.BusType == (uint32_t)BMX160_I2C_INTF) /* I2C */
  {
    /* Enable Multi-byte read */
    return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
  }
  else /* SPI 4-Wires */
  {
    /* Enable Multi-byte read */
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | BMX160_SPI_RD_MASK), pData, Length);
  }
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

  if (pObj->IO.BusType == (uint32_t)BMX160_I2C_INTF) /* I2C */
  {
    /* Enable Multi-byte write */
    return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
  }
  else /* SPI 4-Wires */
  {
    /* Enable Multi-byte write */
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg & BMX160_SPI_WR_MASK), pData, Length);
  }
}

/**
 * @brief  Wrap Delay component function to Bus IO function
 * @param  Handle the device handler
 * @param  Period in milliseconds to wait
 */

static void DelayWrap(void *Handle, uint32_t Period)
{
	BMX160_Object_t *pObj = (BMX160_Object_t *)Handle;
	return pObj->IO.Delayms(Period);
}

/** @}*/
