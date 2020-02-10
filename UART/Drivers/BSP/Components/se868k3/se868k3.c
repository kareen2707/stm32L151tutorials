/*
 * se868k3.c
 *
 *  Created on: Feb 4, 2020
 *      Author: karen@b105.upm.es
 */

/* Includes ------------------------------------------------------------------*/
#include "se868k3.h"


static int32_t WriteWrap(void *Handle, uint8_t *pData, uint16_t Length, uint32_t Timeout);
static int32_t ReadWrap(void *Handle, uint8_t *pData, uint16_t Length);

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t SE868K3_RegisterBusIO(SE868K3_Object_t *pObj, SE868K3_IO_t *pIO)
{
  int32_t ret;

  if (pObj == NULL)
  {
    ret = SE868K3_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.Write  = 	 pIO->Write;
    pObj->IO.Read   = 	 pIO->Read;
    //pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read  = ReadWrap;
    pObj->Ctx.write = WriteWrap;
    pObj->Ctx.handle   = pObj;

    if (pObj->IO.Init != NULL)
    {
      ret = pObj->IO.Init();
    }
    else
    {
      ret = SE868K3_ERROR;
    }
  }

  return ret;
}

int32_t SE868K3_Init(SE868K3_Object_t *pObj){

	int32_t ret = SE868K3_ERROR;
	//uint16_t *flags;
	//circular_buf_t *pileUART;

	setupState_t state = GNSS_WAIT_INIT;

	if (pObj->is_initialized == 0U)
	  {
		pObj->state = state;
		//ret =  se868k3_test(&(pObj->Ctx));
	  }

	  pObj->is_initialized = 1;

	  return ret;
}

int32_t SE868K3_Test(SE868K3_Object_t *pObj)
{
	int32_t ret = SE868K3_ERROR;
	ret =  se868k3_send_TEST(&(pObj->Ctx));
	return ret;
}


static int32_t WriteWrap(void *Handle, uint8_t *pData, uint16_t Length, uint32_t Timeout){

	SE868K3_Object_t *pObj = (SE868K3_Object_t *)Handle;
	return pObj->IO.Write(pData, Length, Timeout);
}

static int32_t ReadWrap(void *Handle, uint8_t *pData, uint16_t Length){

	SE868K3_Object_t *pObj = (SE868K3_Object_t *)Handle;
	return pObj->IO.Read(pData, Length);

}

