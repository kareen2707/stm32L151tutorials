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
#define NMEA_PCKT_MAX_SIZE			15		//82 according to datasheet, let start with this value (40)
#define GNSS_CMD_STRUCT				7		// "$[...]*XX\r\n\0"
// ASCII table conversion
#define ASCII_NUMBER_THRESHOLD		48
#define ASCII_LETTER_THRESHOLD		65
#define TIMEOUT_UART				500
#define FIRST_TIMER_GNSS			5000 	//ms
#define TIMER_GNSS					3000	//ms
#define BUFFER_PCKT_SIZE			64



// MTK NMEA Input Messages - For deeper information read the SE868K3 SW guide
#define GNSS_TEST_CMD							"PMTK000"
//#define 	GNSS_HOT_RESTART_CMD				"PMTK101"
//#define 	GNSS_WARM_RESTART_CMD				"PMTK102"
//#define 	GNSS_COLD_RESTART_CMD				"PMTK103"
//#define 	GNSS_FULL_COLD_RESTART_CMD			"PMTK104"
//#define 	GNSS_CLEAR_FLASH_AID_DATA_CMD		"PMTK120"
//#define 	GNSS_CLEAR_EPO_DATA_CMD				"PMTK127"
//#define 	GNSS_STANDBY_MODE_CMD				"PMTK161"
//#define 	GNSS_QUERY_LOG_CMD					"PMTK183"
//#define	GNSS_LOGGER_FLASH_CMD				"PMTK184"
//#define 	GNSS_STOP_LOGGING_CMD				"PMKT185"
#define GNSS_OUTPUT_MSSG_CMD					"PMTK324,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
#define GNSS_OUTPUT_DATA_RATES					"PMTK314,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
#define GNSS_GALILEO_CMD						"PMTK353,0,0,1,0,0"
#define GNSS_GPS_CMD							"PMTK353,1,0,1,1,0"
#define GNSS_GLONASS_CMD						"PMTK353,0,1,1,1,0"
#define GNSS_MULTIPLE_CMD						"PMTK353,1,1,0,0,0"
#define GNSS_STATIC_CMD							"PMTK386,1.4"


/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */
typedef int32_t (*se868k3_ctx_write)(void *, uint8_t*, uint16_t, uint32_t);
typedef int32_t (*se868k3_ctx_read) (void *, uint8_t*, uint16_t, uint32_t);

typedef struct {
  /** Component mandatory fields **/
	se868k3_ctx_write  write;
	se868k3_ctx_read   read;
  /** Customizable optional pointer **/
  void *handle;
} se868k3_ctx_t;

typedef struct UTC_Info {
  int utc; /**< UTC Info */
  int hh;  /**< Hours */
  int mm;  /**< Minutes */
  int ss;  /**< Seconds */
} UTC_Info;

typedef struct Coords {
  float lat;   /**< Latitude */
  uint8_t ns;  /**< Nord / Sud latitude type */
  float lon;   /**< Longitude */
//  float alt;   /**< Altitude */

  uint8_t ew;  /**< East / West longitude type */
//  uint8_t mis; /**< Altitude unit misure */
} Coords;


typedef struct GPRMC_Infos {
  UTC_Info *utc;           /**< UTC Time */
  uint8_t status;            /**< “A” = valid, “V” = Warning */
  Coords *xyz;             /**< Coords data member */
//  float speed;            /**< Speed over ground in knots */
//  float trackgood;        /**< Course made good */
  int date;               /**< Date of Fix */
//  float mag_var;          /**< Magnetic Variation */
//  char mag_var_dir;       /**< Magnetic Variation Direction */
//  int checksum;           /**< Checksum of the message bytes */
} GPRMC_Infos;



int32_t se868k3_write(se868k3_ctx_t *ctx, uint8_t* data, uint16_t len, uint32_t timeout);
int32_t se868k3_read(se868k3_ctx_t *ctx, uint8_t* data, uint16_t len, uint32_t timeout);
int32_t se868k3_TEST(se868k3_ctx_t *ctx);
int32_t se868k3_SET_port_output_message_intervals(se868k3_ctx_t *ctx);
int32_t se868k3_SET_output_datarates(se868k3_ctx_t *ctx);
int32_t se868k3_SET_MULTIPLE_constellation(se868k3_ctx_t *ctx);
int32_t se868k3_SET_GPSS_constellation(se868k3_ctx_t *ctx);
int32_t se868k3_SET_GALILEO_constellation(se868k3_ctx_t *ctx);
int32_t se868k3_SET_GLONASS_constellation(se868k3_ctx_t *ctx);
int32_t se868k3_SET_speed_threshold(se868k3_ctx_t *ctx);



#endif /* SE868K3_REG_H_ */


