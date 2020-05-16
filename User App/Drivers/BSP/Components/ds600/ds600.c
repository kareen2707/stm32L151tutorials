/*
 * ds600.c
 *
 *  Created on: 15 abr. 2020
 *      Author: karen@b105.upm.es
 *
 */

#include "ds600.h"

int32_t DS600_RegisterBusIO(DS600_Object_t *pObj, DS600_IO_t *pIO){

	int32_t ret;
	if (pObj == NULL)
	{
	    ret = DS600_ERROR;
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
	    	ret = DS600_ERROR;
	    }
	  }

	return ret;
}

int32_t DS600_GET_RawData(DS600_Object_t *pObj, uint16_t *adc_reading){
	int32_t ret;

	if (pObj == NULL)
	{
		ret = DS600_ERROR;
	}
	else{

		ret = pObj->IO.Read(adc_reading);
	}
	return ret;
}

int32_t DS600_GET_Temperature(DS600_Object_t *pObj, float *temperature){

	int32_t ret;
	uint16_t adc_reading = 0;
	float volt_res = (float) POWER_SUPPLY/ADC_RESOLUTION;
	float volt_read;

	if(pObj == NULL){
		ret = DS600_ERROR;
	}
	else{

		ret = pObj->IO.Read(adc_reading);
		if(ret == DS600_OK){
			volt_read = adc_reading * volt_res;
			*temperature = (volt_read - OUTPUT_VOLTAGE_OFFSET)/OUTPUT_GAIN;
		}
		else{
			ret = DS600_ERROR;
		}
	}

	return ret;

}
