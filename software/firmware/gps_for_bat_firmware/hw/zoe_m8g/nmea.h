/*
 * nmea.h
 *
 *  Created on: 29. 10. 2018
 *      Author: David Rau
 */

#ifndef NMEA_H_
#define NMEA_H_

#include <stdint.h>
#include <string.h>

#include "hw_config.h"

enum Eauto_manual_2D_3D_Fix_t {
   eAuto2D3Dfix   = 'A',
   eManual2D3Dfix   = 'M'
   };
typedef enum Eauto_manual_2D_3D_Fix_t Eauto_manual_2D_3D_Fix;

enum E2D_3D_Fix_t {
   eNofix   = '1',
   e2Dfix   = '2',
   e3Dfix   = '3'
   };
typedef enum E2D_3D_Fix_t E2D_3D_Fix;

typedef enum _NMEA_Message_e_
{
	NMEA_Message_Error,
	NMEA_Message_Unknown,
	NMEA_Message_GGA,
	NMEA_Message_GLL,
	NMEA_Message_GSA,
	NMEA_Message_GSV,
	NMEA_Message_RMC,
	NMEA_Message_VTG,
	NMEA_Message_GRS,
	NMEA_Message_GST,
	NMEA_Message_ZDA,
	NMEA_Message_GBS,
	NMEA_Message_DTM,
	NMEA_Message_GNS,
	NMEA_Message_VLM,
	NMEA_Message_TXT,
} NMEA_Message_e;

typedef enum
{
	NMEA_System_Error,
	NMEA_System_Unknown,
	NMEA_System_Main,
	NMEA_System_GPS,
	NMEA_System_Glonass,
} NMEA_System_e;

typedef struct {

	NMEA_Message_e message;
	NMEA_System_e system;
} NMEA_Params_s;

#define MICRODEGREES_MULTIPLY 1000000


#define GSA_SATELLITE_SPACE_MAX 12
#define GSA_SATELLITE_INFO_SPACE_MAX 12

typedef struct
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint32_t utcDaySeconds;
} NMEA_Time_s;

#define NMEA_TIME_INIT {0, 0, 0, 0}

typedef struct
{
	uint8_t month;
	uint8_t day;
	uint16_t year;
} NMEA_Date_s;

#define NMEA_DATE_INIT {0, 0, 0}

typedef struct
{
	float latitude;
	int16_t latitudeDegMin;
	int16_t latitudeDegMinFraction;
	int32_t	latitudeMicroDegree;
} NMEA_Latitude_s;

#define NMEA_LATITUDE_INIT {0.0, 0, 0, 0}

typedef struct
{
	float longitude;
	int16_t longitudeDegMin;
	int16_t longitudeDegMinFraction;
	int32_t	longitudeMicroDegree;
} NMEA_Longitude_s;

#define NMEA_LONGITUDE_INIT {0.0, 0, 0, 0}

typedef struct
{
	int32_t	speedMPS10;
	float speedKmPH;
} NMEA_Speed_s;

#define NMEA_SPEED_INIT {0, 0.0}

typedef struct
{
	float altitude;
	int32_t	altitudeUInt;
	float geoidHeight;
	int32_t	altitudeUIntWithGeoid;
} NMEA_Altitude_s;

#define NMEA_ALTITUDE_INIT {0.0, 0, 0.0, 0}

typedef struct
{
	NMEA_Time_s time;
	NMEA_Date_s date;

	NMEA_Latitude_s latitude;
	NMEA_Longitude_s longitude;
	uint8_t status;
	int8_t gpsValid;
	int8_t timeValid;
	uint8_t posMode;
	NMEA_Speed_s speed;

	float trackAngle;
	float magneticVariation;
} RMC_Data_s;

#define RMC_DATA_INIT { \
						NMEA_TIME_INIT, NMEA_DATE_INIT, NMEA_LATITUDE_INIT, NMEA_LONGITUDE_INIT, 0, \
						0, 0, 0, NMEA_SPEED_INIT, 0.0, 0.0 \
					  }

typedef struct
{
	NMEA_Time_s time;
	NMEA_Latitude_s latitude;
	NMEA_Longitude_s longitude;
	uint8_t quality;

	int8_t gpsValid;
	int8_t timeValid;

	uint8_t satTracked;

	float horizontalDilutionOfPrecision;
	NMEA_Altitude_s altitude;
} GGA_Data_s;

#define GGA_DATA_INIT { \
						NMEA_TIME_INIT, NMEA_LATITUDE_INIT, NMEA_LONGITUDE_INIT, 0, \
						0, 0, 0, 0.0, NMEA_ALTITUDE_INIT \
					  }


struct SatelliteInfo_s {
    unsigned char prn;
    unsigned char elevation;
    unsigned short azimuth;
    unsigned char snr;
};
typedef struct SatelliteInfo_s SatelliteInfo;


typedef struct
{
	uint8_t activeSatelliteList[GSA_SATELLITE_SPACE_MAX];
	uint8_t activeSatelliteCount;
//	SatelliteInfo satelliteInfoList[GSA_SATELLITE_INFO_SPACE_MAX];
//	uint8_t satelliteInfoCount;// = 0;
} NMEA_Satellite_s;

#define NMEA_SATELLITE_INIT {{0}, 0}

typedef struct
{
	NMEA_Time_s time;
	NMEA_Date_s date;
} ZDA_Data_s;

#define ZDA_DATA_INIT { NMEA_TIME_INIT, NMEA_DATE_INIT }

typedef struct
{
	Eauto_manual_2D_3D_Fix eAutoManualFix;// = (Eauto_manual_2D_3D_Fix)'A';
	E2D_3D_Fix e2D3Dfix;// = (E2D_3D_Fix)'1';

	NMEA_Satellite_s satellite;

	float positionDilutionOfPrecision;
	float horizontalDilutionOfPrecision;
	float verticalDilutionOfPrecision;
} GSA_Data_s;

#define GSA_DATA_INIT {'A', 1, NMEA_SATELLITE_INIT, 0.0, 0.0, 0.0}

//typedef struct
//{
////	uint8_t dilutionPosition;
//	uint8_t hour,min,sec;
//	uint8_t month, day;
//	uint16_t year;
//
//	uint32_t utcDaySeconds;
//	uint8_t satTracked;
//	float horizontalDilution;
//	float geoidHeight;
//	int8_t gpsValid;
//	float altitude;
//	float latitude;
//	float longitude;
//	int16_t longitudeDegMin;
//	int16_t longitudeDegMinFraction;
//	int16_t latitudeDegMin;
//	int16_t latitudeDegMinFraction;
//	uint8_t qualityFixFromGGA;
//
//	int32_t	latitudeMicroDegree;
//	int32_t	longitudeMicroDegree;
//	int32_t	altitudeUInt;
//	int32_t	altitudeUIntWithGeoid;
//
//	int32_t	speedMPS10;
//	int8_t	gpsDOP;
//
//	float dilutionOfPrecision;
//	float verticalDilutionOfPrecision;
//
//	float speedKmPH;
//	float trackAngle;
//	float magneticVariation;
//
////	uint8_t activeSatelliteList[GSA_SATELLITE_SPACE_MAX];
////	uint8_t activeSatelliteCount;
////
////	SatelliteInfo satelliteInfoList[GSA_SATELLITE_INFO_SPACE_MAX];
////	uint8_t satelliteInfoCount;// = 0;
////
////	Eauto_manual_2D_3D_Fix eAutoManualFix;// = (Eauto_manual_2D_3D_Fix)'A';
////	E2D_3D_Fix e2D3Dfix;// = (E2D_3D_Fix)'1';
//
//}Gps_Data_s;


uint8_t parseHeader(uint8_t *buffer, uint8_t iComma);
uint8_t parseStatus(uint8_t *buffer, uint8_t *data, int8_t *valid, uint8_t iComma);
//uint8_t parseTimeFromGGA(uint8_t * buffer, GGA_Data_s *data, uint8_t iComma);
//uint8_t parseLatitudeFromGGA(uint8_t *buffer, GGA_Data_s *data, uint8_t iComma);
//uint8_t parseLongitudeFromGGA(uint8_t * buffer, GGA_Data_s *data, uint8_t iComma);
uint8_t parseLongitude(uint8_t *buffer, NMEA_Longitude_s *data, uint8_t iComma);
uint8_t parseLatitude(uint8_t *buffer, NMEA_Latitude_s *data, uint8_t iComma);
uint8_t parseTime(uint8_t *buffer, NMEA_Time_s *data, int8_t* timeValid,  uint8_t iComma);
uint8_t parseSatelitesTracked(uint8_t * buffer, uint8_t *data, uint8_t iComma);
uint8_t parseQuality(uint8_t *buffer, uint8_t *data, int8_t *valid, uint8_t iComma);
uint8_t parseHorizDilutionOfPosition(uint8_t * buffer, float *data, uint8_t iComma);
uint8_t parseAltitudeMeters(uint8_t * buffer, NMEA_Altitude_s *data, uint8_t iComma);
uint8_t parseHeightOfGeoid(uint8_t * buffer, NMEA_Altitude_s *data, uint8_t iComma);
uint8_t parseEmpty(uint8_t *buffer, uint8_t index);
uint8_t parseGroundSpeed(uint8_t * buffer, NMEA_Speed_s *data, uint8_t iComma);
uint8_t parseTrackAngle(uint8_t * buffer, float *data, uint8_t iComma);
uint8_t parseDate(uint8_t * buffer, NMEA_Date_s *data, uint8_t iComma);
uint8_t parseMagneticVariation(uint8_t * buffer, float *data, uint8_t iComma);
uint8_t parsePosMode(uint8_t *buffer, uint8_t *data, uint8_t iComma);
//uint8_t validityFixFromGGA(uint8_t *buffer, GGA_Data_s *data);
GGA_Data_s proceedGGAparsing(uint8_t * buffer);
RMC_Data_s proceedRMCparsing(uint8_t *buffer);
//void proceedNMEABuffer(uint8_t *buffer);
uint8_t parseActiveSatelliteList(uint8_t* buffer, NMEA_Satellite_s *data, uint8_t iComma);
uint8_t parsePositionDilutionOfPrecision(uint8_t* buffer, float *data, uint32_t iComma);
uint8_t parseVerticalDilutionOfPrecision(uint8_t* buffer, float *data, uint32_t iComma);
uint8_t parseDilutionOfPrecision(uint8_t * buffer, float *data, uint8_t iComma);
GSA_Data_s proceedGSAparsing(uint8_t * buffer);
NMEA_Params_s checkMessage_NMEA(uint8_t * buffer);
int8_t checkChecksum(uint8_t * buffer);
uint8_t parseDay(uint8_t * buffer, NMEA_Date_s *data, uint8_t iComma);
uint8_t parseMonth(uint8_t * buffer, NMEA_Date_s *data, uint8_t iComma);
uint8_t parseYear(uint8_t * buffer, NMEA_Date_s *data, uint8_t iComma);
NMEA_System_e getGnssSystemInNmea(uint8_t *buffer);
NMEA_Message_e getTypeInNmea(uint8_t *buffer);

#endif /* NMEA_H_ */
