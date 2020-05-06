/*
 * se868k3-reg.c
 *
 *  Created on: Feb 4, 2020
 *      Author: karen@b105.upm.es
 */

#include "SE868K3-reg.h"
#include <stdio.h>
#include <string.h>

static void checksum(char *cmd, char *checksum1, char *checksum2);
static void createPackage (char cmd, char *packet, uint8_t *packet_len);

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
int32_t se868k3_read(se868k3_ctx_t* ctx, uint8_t* data, uint16_t len)
{
  int32_t ret;
  ret = ctx->read(ctx->handle, data, len);
  return ret;
}

int32_t se868k3_send_TEST(se868k3_ctx_t *ctx)
{
	uint16_t cmd_len;
	int32_t ret;
	char *command;
	char checksum1, checksum2;
	cmd_len = strlen(GNSS_TEST_CMD) + GNSS_CMD_STRUCT-1; // -1 because it considers '\0' char character
	checksum(GNSS_TEST_CMD, &checksum1, &checksum2);
	sprintf(command, "$%s*%c%c\r\n", GNSS_TEST_CMD, checksum1, checksum2);
	ret = se868k3_write(ctx, (uint8_t *) command, cmd_len, TIMEOUT_UART);
	return ret;
}

//int32_t se868k3_send_TEST(se868k3_ctx_t *ctx)
//{
//	int32_t ret;
//	char *packet = NULL;
//	uint8_t packet_len;
//	createPackage(GNSS_TEST_CMD, packet, &packet_len);
//	ret = se868k3_write(ctx, (uint8_t *) packet, packet_len, TIMEOUT_UART);
//	return ret;
//}

//int32_t se868k3_send_START(se868k3_ctx_t *ctx)
//{
//	uint8_t cmd_len;
//	int32_t ret;
//	char *command;
//	char checksum1, checksum2;
//	//char format;
//	cmd_len = strlen(GNSS_START_CMD) + GNSS_CMD_STRUCT-1; // -1 because it considers '\0' char character
//	checksum(GNSS_START_CMD, &checksum1, &checksum2);
//	sprintf(command, "$%s*%c%c\r\n", GNSS_START_CMD, checksum1, checksum2);
//	ret = se868k3_write(ctx, (uint8_t *) command, cmd_len, TIMEOUT_UART);
//	return ret;
//}


//static void createPackage (char cmd, char *packet, uint8_t *packet_len)
//{
//	char checksum1, checksum2;
//	*packet_len = strlen(cmd) + GNSS_CMD_STRUCT -1;
//	checksum(cmd, &checksum1, &checksum2);
//	sprintf(packet, "$%s*%c%c\r\n", cmd, checksum1, checksum2);
//}



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
