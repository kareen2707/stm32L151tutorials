/*
 * se868k3-reg.h
 *
 *  Created on: Feb 4, 2020
 *      Author: karen
 */

#ifndef SE868K3_REG_H
#define SE868K3_REG_H

#include <stdint.h>
#include <math.h>


// Size of  GNSS
#define GNSS_STRUCT					5		// "gnss [...]"
#define GNSS_CMD_STRUCT				7		// "$[...]*XX\r\n\0"
// ASCII table conversion
#define ASCII_NUMBER_THRESHOLD		48
#define ASCII_LETTER_THRESHOLD		65
#define TIMEOUT_UART				500

// MTK NMEA Input Messages - For deeper information read the SE868K3 SW guide
#define GNSS_TEST_CMD						"PMTK000"
#define GNSS_HOT_RESTART_CMD				"PMTK101"
#define GNSS_WARM_RESTART_CMD				"PMTK102"
#define GNSS_COLD_RESTART_CMD				"PMTK103"
#define GNSS_FULL_COLD_RESTART_CMD			"PMTK104"
#define GNSS_CLEAR_FLASH_AID_DATA_CMD		"PMTK120"
#define GNSS_CLEAR_EPO_DATA_CMD				"PMTK127"
#define GNSS_STANDBY_MODE_CMD				"PMTK161"
#define GNSS_QUERY_LOG_CMD					"PMTK183"
#define	GNSS_LOGGER_FLASH_CMD				"PMTK184"
#define GNSS_STOP_LOGGING_CMD				"PMKT185"
#define GNSS_START_CMD						"PMTK324,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
#define GNSS_GALILEO_CMD					"PMTK353,0,0,1,0,0"
#define GNSS_GPS_CMD						"PMTK353,1,0,1,1,0"
#define GNSS_GLONASS_CMD					"PMTK353,0,1,1,1,0"
#define GNSS_MULTIPLE_CMD					"PMTK353,1,1,0,0,0"
#define GNSS_STATIC_CMD						"PMTK386,1.4"

// Set up states of SE868K3-AL module
typedef enum {
	GNSS_READY 				= 0,
	GNSS_NOP	 			= 1,
	GNSS_SET_STATIC_SPEED	= 2,
	GNSS_SET_CONSTELLATION	= 3,
	GNSS_SET_DATARATE		= 4,
	GNSS_WAIT_INIT			= 5,
} setupState_t;

// Structure of the location message sent by the SE868K3-AL module
typedef struct
{
	  float		latitude;
	  float		longitude;
	  uint32_t	timestamp;
} location_t;


/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */
typedef int32_t (*stmdev_write)(void *, uint8_t*, uint16_t, uint32_t);
typedef int32_t (*stmdev_read) (void *, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  stmdev_write  write;
  stmdev_read   read;
  /** Customizable optional pointer **/
  void *handle;
} se868k3_ctx_t;

int32_t se868k3_write(se868k3_ctx_t *ctx, uint8_t* data, uint16_t len, uint32_t timeout);
int32_t se868k3_read(se868k3_ctx_t *ctx, uint8_t* data, uint16_t len); // Karen: start with the write function
int32_t se868k3_send_TEST(se868k3_ctx_t *ctx);
int32_t se868k3_send_START(se868k3_ctx_t *ctx); //Karen: add the parameter string
//int32_t se868k3_send_START(se868k3_ctx_t *ctx); //Karen: add the parameter string

#endif /* BSP_COMPONENTS_SE868K3_SE868K3_REG_H_ */
