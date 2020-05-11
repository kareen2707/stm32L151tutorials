/*
 * se868k3-reg.c
 *
 *  Created on: Feb 4, 2020
 *      Author: karen@b105.upm.es
 */

#include "SE868K3-reg.h"
#include "FreeRTOS.h"
#include <stdio.h>
#include <string.h>

static void checksum(char *cmd, char *checksum1, char *checksum2);
static int32_t createPacket(se868k3_ctx_t *ctx, uint8_t* cmd);

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t se868k3_write(se868k3_ctx_t *ctx, uint8_t* data, uint16_t len, uint32_t timeout)
{
  int32_t ret;
  ret = ctx->write(ctx->handle, data, len, timeout);
  return ret;
}

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t se868k3_read(se868k3_ctx_t* ctx, uint8_t* data, uint16_t len, char* message)
{
  int32_t ret;
  ret = ctx->read(ctx->handle, data, len);
  return ret;
}

int32_t se868k3_TEST(se868k3_ctx_t *ctx)
{
	return createPacket(ctx, (uint8_t *)GNSS_TEST_CMD);
}

int32_t se868k3_SET_port_output_message_intervals(se868k3_ctx_t *ctx){

	return createPacket(ctx, (uint8_t *) GNSS_OUTPUT_MSSG_CMD);
}

int32_t se868k3_SET_output_datarates(se868k3_ctx_t *ctx){

	return createPacket(ctx, (uint8_t *) GNSS_OUTPUT_DATA_RATES);
}

int32_t se868k3_SET_MULTIPLE_constellation(se868k3_ctx_t *ctx){

	return createPacket(ctx, (uint8_t *) GNSS_MULTIPLE_CMD);
}

int32_t se868k3_SET_GPSS_constellation(se868k3_ctx_t *ctx){

	return createPacket(ctx, (uint8_t *) GNSS_GPS_CMD);
}

int32_t se868k3_SET_GALILEO_constellation(se868k3_ctx_t *ctx){

	return createPacket(ctx, (uint8_t *) GNSS_GALILEO_CMD);
}

int32_t se868k3_SET_GLONASS_constellation(se868k3_ctx_t *ctx){

	return createPacket(ctx, (uint8_t *) GNSS_GLONASS_CMD);
}

int32_t se868k3_SET_speed_threshold(se868k3_ctx_t *ctx){

	return createPacket(ctx, (uint8_t *) GNSS_STATIC_CMD);
}


int32_t createPacket(se868k3_ctx_t *ctx, uint8_t* cmd){

	char checksum1, checksum2;
	int32_t ret;
	uint8_t* pckt = (uint8_t*) pvPortMalloc(NMEA_PCKT_MAX_SIZE);
	checksum((char *)cmd, &checksum1, &checksum2);
	sprintf((char *)pckt, "$%s*%c%c\r\n", cmd, checksum1, checksum2);
	ret = se868k3_write(ctx, pckt, strlen((char *)pckt), TIMEOUT_UART);
	vPortFree(pckt);
	return ret;
}

static void checksum(char *cmd, char *checksum1, char *checksum2)
{
	uint8_t i, checksum;
	i = 0;
	checksum = 0;
	do {
		checksum ^= cmd[i++];
	} while (cmd[i] != '\0');
		*checksum1 = (checksum >> 4);
		*checksum2 = (checksum % 16);
		*checksum1 += (*checksum1 > 9)? ASCII_LETTER_THRESHOLD - 10 : ASCII_NUMBER_THRESHOLD;
		*checksum2 += (*checksum2 > 9)? ASCII_LETTER_THRESHOLD - 10 : ASCII_NUMBER_THRESHOLD;
}
