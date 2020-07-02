/*
 * se868k3.h
 *
 *  Created on: Feb 4, 2020
 *      Author: karen@b105.upm.es
 */

#ifndef SE868K3_H
#define SE868K3_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <se868k3_reg.h>
#include <string.h>
#include "cmsis_os.h"


 /** @defgroup SE868K3_Exported_Types SE868K3-AL Exported Types
 * @{
*/

typedef int32_t (*SE868K3_Init_Func)(void);
typedef int32_t (*SE868K3_DeInit_Func)(void);
typedef int32_t (*SE868K3_Write_Func)(uint8_t *, uint16_t, uint32_t); //Parameters data, size and timeout
typedef int32_t (*SE868K3_Read_Func)(uint8_t *, uint16_t, char*); //Warning this must be implemented using DMA channel. Parameters: data and size

typedef struct
  {
    SE868K3_Init_Func       Init; // to initialize the peripheral interface
    SE868K3_DeInit_Func     DeInit;
    SE868K3_Write_Func      Write; // to send commands to the module
    SE868K3_Read_Func       Read; // to read data from the module
  } SE868K3_IO_t;

  typedef struct // This is equivalent to module_
  {
	SE868K3_IO_t        IO;
	se868k3_ctx_t       Ctx;
    uint8_t             is_initialized;
    uint8_t				*pileUART;
  } SE868K3_Object_t;



 /** SE868K3 error codes  **/
#define SE868K3_OK                 0
#define SE868K3_ERROR             -1

 int32_t SE868K3_RegisterBusIO(SE868K3_Object_t *pObj, SE868K3_IO_t *pIO);
 int32_t SE868K3_Init(SE868K3_Object_t *pObj);
 int32_t SE868K3_DeInit(SE868K3_Object_t *pObj);
 int32_t SE868K3_Test(SE868K3_Object_t *pObj);
 //int32_t SE868K3_Read_Packet(SE868K3_Object_t* pObj, uint8_t len);
 int32_t SE868K3_Read_RMC(SE868K3_Object_t *pObj);
 int32_t SE868K3_Read_GGA(SE868K3_Object_t *pObj);
 int32_t SE868K3_Read_GLL(SE868K3_Object_t *pObj);
 int32_t SE868K3_Read_GSV(SE868K3_Object_t *pObj);
 int32_t SE868K3_Read_RLM(SE868K3_Object_t *pObj);
 int32_t SE868K3_Read_VTG(SE868K3_Object_t *pObj);
 int32_t SE868K3_Read_ZDA(SE868K3_Object_t *pObj);
 int32_t SE868K3_Read_GSA(SE868K3_Object_t *pObj);

#ifdef __cplusplus
}
#endif


#endif /* SE868K3_H_ */
