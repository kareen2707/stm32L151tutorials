/*
 * se868k3_reg.h
 *	Header register file for the GNSS module SE868K3-AL integrated in Grizz Board.
 *  Created on: Feb 4, 2020
 *      Author: karen@b105.upm.es
 */

#ifndef SE868K3_REG_H
#define SE868K3_REG_H

#include <stdint.h>
#include <math.h>


// Size of  GNSS
//#define GNSS_STRUCT					5		// "gnss [...]"
#define MTK_NMEA_CMD_MAX_SIZE		255		// Maximum mtk packet size according to SW USER GUIDE r5 of Telit SE868K3-AL module
//#define GNSS_CMD_STRUCT				7		// "$[...]*XX\r\n\0"
/** ASCII table conversion **/
#define ASCII_NUMBER_THRESHOLD		48
#define ASCII_LETTER_THRESHOLD		65
/** TIME DEFINITIONS **/
#define TIMEOUT_UART				500
#define FIRST_TIMER_GNSS			5000 	//ms
#define TIMER_GNSS					3000	//ms
/** NMEA SENTENCES PACKETS SIZES **/
#define BUFFER_PCKT_SIZE			68
#define GGA_MAX_SIZE				71
#define GLL_MAX_SIZE				50
#define GSA_MAX_SIZE				53
#define GSV_MAX_SIZE				69
#define RMC_MAX_SIZE				82
#define VTG_MAX_SIZE				36
#define ZDA_MAX_SIZE				34
#define RLM_MAX_SIZE				42

// MTK NMEA Input Messages - For further information read SW USER GUIDE r5 of Telit SE868K3-AL module
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

/** MTK NMEA Commands with variable parameters	**/
/* Set NMEA Sentence Output Rates Message 	PMTK314,GLL|RMC|VTG|GGA|GSA|GSV|0|0|0|0|0|0|0|0|0|0|0|ZDA|0|0|0|0|0|0
 * Possible values to set: 0 -> disabled, 1 -> output every position fix, 2 -> every 2 positions fixes
 * 						   3 -> every 3 position fixes, 4 -> every 4 positions, 5 -> every five*/
#define GNSS_OUTPUT_DATA_RATES				"PMTK314,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
/* Set Com Port NMEA Sentence Output Rates 	PMTK324,PORT|GLL|RMC|VTG|GGA|GSA|GSV|0|0|0|0|0|0|0|0|0|0|0|ZDA|0|0|0|0|0|0*/
/* For the PORT field tha possible values are: 1 -> Port0, 2 -> Port1*/
#define GNSS_OUTPUT_MSSG_CMD				"PMTK324,1,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"

/** Set GNSS Search mode 					PMTK353, GPS|GLONASS|GALILEO|0|BEIDOU
 * 	to enable a specific system write 1, otherwise disable it writing 0
 * 	for MULTI-Constellations, maximum 3 different systems enabled at the same time	**/
#define GNSS_MULTIPLE_CMD					"PMTK353,1,0,1,0,0"
#define GNSS_GALILEO_CMD					"PMTK353,0,0,1,0,0"
#define GNSS_GPS_CMD						"PMTK353,1,0,0,0,0"
#define GNSS_GLONASS_CMD					"PMTK353,0,1,0,0,0"

#define GNSS_STATIC_CMD						"PMTK386,1.4"
/** Set Navigation Mode					   PMTK886,MODE
 *  possible modes: 0 -> normal mode, 1 -> fitness mode (speed < 10m/s), 2 -> aviation mode, 3 -> ballon mode, 4 -> stationary mode **/
#define GNSS_NAV_MODE						"PMT886,1"
/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to receive and transmit information.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */
typedef int32_t (*se868k3_ctx_write)(void *, uint8_t*, uint16_t, uint32_t);
typedef int32_t (*se868k3_ctx_read) (void *, uint8_t*, uint16_t, char*); // Char was added

typedef struct {
  /** Component mandatory fields **/
	se868k3_ctx_write  write;
	se868k3_ctx_read   read;
  /** Customizable optional pointer **/
	void *handle;
} se868k3_ctx_t;


int32_t se868k3_write(se868k3_ctx_t *ctx, uint8_t* data, uint16_t len, uint32_t timeout);
int32_t se868k3_read(se868k3_ctx_t *ctx, uint8_t* data, uint16_t len, char* message); // char added
int32_t se868k3_TEST(se868k3_ctx_t *ctx);
int32_t se868k3_SET_port_output_message_intervals(se868k3_ctx_t *ctx);
int32_t se868k3_SET_output_datarates(se868k3_ctx_t *ctx);
int32_t se868k3_SET_MULTIPLE_constellation(se868k3_ctx_t *ctx);
int32_t se868k3_SET_GPSS_constellation(se868k3_ctx_t *ctx);
int32_t se868k3_SET_GALILEO_constellation(se868k3_ctx_t *ctx);
int32_t se868k3_SET_GLONASS_constellation(se868k3_ctx_t *ctx);
int32_t se868k3_SET_speed_threshold(se868k3_ctx_t *ctx);
int32_t se868k3_SET_navigation_mode(se868k3_ctx_t *ctx);



#endif /* SE868K3_REG_H_ */
