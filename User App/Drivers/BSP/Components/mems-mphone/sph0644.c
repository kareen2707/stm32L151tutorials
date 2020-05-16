/*
 * sph0644.c
 *
 *  Created on: 16 abr. 2020
 *      Author: karen@b105.upm.es
 *
 */

#include "sph0644.h"

int32_t SPH06440_RegisterBusIO(SPH06440_Object_t *pObj, SPH06440_IO_t *pIO){

	int32_t ret;
	if (pObj == NULL)
	{
		ret = SPH0644_ERROR;
	}
	else
	{
		pObj->IO.Init      = pIO->Init;
		pObj->IO.DeInit    = pIO->DeInit;
		pObj->IO.Read 	   = pIO->Read;

		if (pObj->IO.Init != NULL)
		{
		    ret = pObj->IO.Init();
		}
		else
		{
		    ret = SPH0644_ERROR;
		}
	}

	return ret;
}
int32_t SPH06440_GET_samples(SPH06440_Object_t *pObj, uint8_t *pData, uint16_t num_samples){

	int32_t ret;
	if (pObj == NULL)
	{
		ret = SPH0644_ERROR;
	}
	else{

		ret = pObj->IO.Read(pData, num_samples);
	}
	return ret;
}
