/*
 * sph0644.h
 *
 *  Created on: 16 abr. 2020
 *      Author: karen
 */

#ifndef BSP_COMPONENTS_MEMS_MPHONE_SPH0644_H_
#define BSP_COMPONENTS_MEMS_MPHONE_SPH0644_H_

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdint.h>

/** @defgroup BMX160_Exported_Types BMX160 Exported Types
 * @{
 */

typedef int32_t (*SPH0644_Init_Func)(void);
typedef int32_t (*SPH0644_DeInit_Func)(void);
typedef int32_t (*SPH0644_Read_Func)(uint8_t *, uint16_t);

/** SPH0644 error codes  **/
#define SPH0644_OK                 0
#define SPH0644_ERROR             -1

typedef struct
{
  SPH0644_Init_Func          Init;
  SPH0644_DeInit_Func        DeInit;
  uint32_t                   BusType; /*0 means I2C, 1 means SPI-4-Wires */
  SPH0644_Read_Func       	 Read;
} SPH06440_IO_t;

typedef struct SPH06440_Object
{
  SPH06440_IO_t        IO;
  uint8_t              is_initialized;

} SPH06440_Object_t;

int32_t SPH06440_RegisterBusIO(SPH06440_Object_t *pObj, SPH06440_IO_t *pIO);
int32_t SPH06440_GET_samples(SPH06440_Object_t *pObj, uint8_t *pData, uint16_t num_samples);
#ifdef __cplusplus
}
#endif

#endif /* BSP_COMPONENTS_MEMS_MPHONE_SPH0644_H_ */
