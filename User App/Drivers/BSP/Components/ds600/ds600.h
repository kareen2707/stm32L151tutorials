/*
 * ds600.h
 *
 *  Created on: 15 abr. 2020
 *      Author: karen@b105.upm.es
 */

#ifndef DS600_H_
#define DS600_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

#define OUTPUT_GAIN 			(float) 6.45 // ΔV/ΔT (mV/ºC)
#define OUTPUT_VOLTAGE_OFFSET	(float)	509 //mV
#define ADC_RESOLUTION			4096 // 2^N bits
#define POWER_SUPPLY 			3300 // mV (3.3V)

/** DS600 error codes  **/
#define DS600_OK                 0
#define DS600_ERROR             -1

typedef int32_t (*DS600_Init_Func)(void);
typedef int32_t (*DS600_DeInit_Func)(void);
typedef int32_t (*DS600_Read_Func)(uint16_t *);

typedef struct
{
  DS600_Init_Func          Init;
  DS600_DeInit_Func        DeInit;
  DS600_Read_Func          Read;
} DS600_IO_t;

typedef struct
{
  DS600_IO_t        IO;
  uint8_t           is_initialized;
} DS600_Object_t;

int32_t DS600_RegisterBusIO(DS600_Object_t *pObj, DS600_IO_t *pIO);
int32_t DS600_GET_RawData(DS600_Object_t *pObj, uint16_t *adc_reading);
int32_t DS600_GET_Temperature(DS600_Object_t *pObj, float *temperature);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COMPONENTS_DS600_DS600_H_ */
