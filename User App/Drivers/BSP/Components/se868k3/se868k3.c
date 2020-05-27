/*
 * se868k3.c
 *
 *  Created on: Feb 4, 2020
 *      Author: karen@b105.upm.es
 */

/* Includes ------------------------------------------------------------------*/
#include "se868k3.h"
//#include "cmsis_os.h" //karen

static int32_t WriteWrap(void *Handle, uint8_t *pData, uint16_t Length, uint32_t Timeout);
static int32_t ReadWrap(void *Handle, uint8_t *pData, uint16_t Length, char* message);


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
    pObj->IO.Write     = pIO->Write;
    pObj->IO.Read      = pIO->Read;

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


	if (pObj->is_initialized == 0U)
	  {
			se868k3_SET_port_output_message_intervals(&(pObj->Ctx));
			osDelay(FIRST_TIMER_GNSS);
			se868k3_SET_MULTIPLE_constellation(&(pObj->Ctx));
			osDelay(TIMER_GNSS);
			se868k3_SET_navigation_mode(&(pObj->Ctx));
			osDelay(TIMER_GNSS);
			pObj->is_initialized = 1;
			ret = SE868K3_OK;
	  }

	  return ret;
}

//int32_t SE868K3_Read_Packet(SE868K3_Object_t* pObj, uint8_t len)
//{
//	return se868k3_read(&(pObj->Ctx), pObj->pileUART, len);
//}

int32_t SE868K3_Read_RMC(SE868K3_Object_t *pObj){
	return se868k3_read(&(pObj->Ctx), pObj->pileUART, RMC_MAX_SIZE, "GNRMC");
}

int32_t SE868K3_Read_GGA(SE868K3_Object_t *pObj){
	return se868k3_read(&(pObj->Ctx), pObj->pileUART, GGA_MAX_SIZE, "GNGGA");
}

 int32_t SE868K3_Read_GLL(SE868K3_Object_t *pObj){
	 return se868k3_read(&(pObj->Ctx), pObj->pileUART, GLL_MAX_SIZE, "GNGLL");
 }

 int32_t SE868K3_Read_GSV(SE868K3_Object_t *pObj){
	 return se868k3_read(&(pObj->Ctx), pObj->pileUART, GSV_MAX_SIZE, "GPGSV");
 }

 int32_t SE868K3_Read_RLM(SE868K3_Object_t *pObj){
	 return se868k3_read(&(pObj->Ctx), pObj->pileUART, RLM_MAX_SIZE, "GNRLM");
 }

 int32_t SE868K3_Read_VTG(SE868K3_Object_t *pObj){
	 return se868k3_read(&(pObj->Ctx), pObj->pileUART, VTG_MAX_SIZE, "GNVTG");
 }

 int32_t SE868K3_Read_ZDA(SE868K3_Object_t *pObj){
	 return se868k3_read(&(pObj->Ctx), pObj->pileUART, ZDA_MAX_SIZE, "GNZDA");
 }

 int32_t SE868K3_Read_GSA(SE868K3_Object_t *pObj){
	 return se868k3_read(&(pObj->Ctx), pObj->pileUART, GSA_MAX_SIZE, "GNGSA");
 }

static int32_t WriteWrap(void *Handle, uint8_t *pData, uint16_t Length, uint32_t Timeout){

	SE868K3_Object_t *pObj = (SE868K3_Object_t *)Handle;
	return pObj->IO.Write(pData, Length, Timeout);
}

static int32_t ReadWrap(void *Handle, uint8_t *pData, uint16_t Length, char* message){

	SE868K3_Object_t *pObj = (SE868K3_Object_t *)Handle;
	return pObj->IO.Read(pData, Length, message);

}


