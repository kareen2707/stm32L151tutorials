/*
 * se868k3.h
 *
 *  Created on: Feb 4, 2020
 *      Author: karen
 */

#ifndef SE868K3_H
#define SE868K3_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "SE868K3-reg.h"
#include <string.h>
#include "cmsis_os.h" //karen


 /** @defgroup SE868K3_Exported_Types SE868K3-AL Exported Types
 * @{
*/

typedef int32_t (*SE868K3_Init_Func)(void); //Karen: parameters are the pointer and size of the RX buffer
typedef int32_t (*SE868K3_DeInit_Func)(void);
typedef int32_t (*SE868K3_Write_Func)(uint8_t *, uint16_t, uint32_t); //Parameters data, size and timeout
typedef int32_t (*SE868K3_Read_Func)(uint8_t *, uint16_t); //Warning this must be implemented using DMA channel. Parameters: data and size

typedef struct
  {
    SE868K3_Init_Func       Init; // to initialize the peripheral interface
    SE868K3_DeInit_Func     DeInit;
    SE868K3_Write_Func      Write; // to send commands to the module
    SE868K3_Read_Func       Read; // to read data from the module
    //SE868K3_GetTick_Func       GetTick;
  } SE868K3_IO_t;

  typedef struct // This is equivalent to module_
  {
	SE868K3_IO_t        IO;
	se868k3_ctx_t       Ctx;
    uint8_t             is_initialized; //this could be part of the flags
    uint16_t			*flags;
    osMutexId			pileLock;
    uint8_t				*pileUART; //add the circular buffer files needed
  } SE868K3_Object_t;



 /** SE868K3 error codes  **/
#define SE868K3_OK                 0
#define SE868K3_ERROR             -1

 int32_t SE868K3_RegisterBusIO(SE868K3_Object_t *pObj, SE868K3_IO_t *pIO);
 int32_t SE868K3_Init(SE868K3_Object_t *pObj);
 int32_t SE868K3_DeInit(SE868K3_Object_t *pObj);
 int32_t SE868K3_Test(SE868K3_Object_t *pObj);
 int32_t SE868K3_Read_Packet(SE868K3_Object_t* pObj, uint8_t len);



#ifdef __cplusplus
}
#endif


#endif /* SE868K3_H_ */
