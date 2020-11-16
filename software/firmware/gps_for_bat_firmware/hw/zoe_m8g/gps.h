/*
 * gps.h
 *
 *  Created on: 29. 10. 2018
 *      Author: David Rau
 */

#ifndef GPS_H_
#define GPS_H_

#include "nmea.h"
#include <stdint.h>
#include "ubx.h"
#include "usart.h"
#include "rtc.h"


typedef struct
{
	uint8_t rate;
	uint8_t mask;
}Gps_Config_Msg_s;


#define GPS_CONFIG_MSG_LENGTH 4

#define UBX_START_TO_PL_CNT	6
#define UBX_PL_TO_END_CNT 2

typedef enum Packet_Type_ {
	No_Packet_Type,
	NMEA_Packet_Type,
	UBX_Packet_Type,
} Packet_Type_e;

enum {
	NMEA_Head = '$',
	NMEA_End = '\n',
} Packet_Sync_e;

typedef struct
{
	NMEA_Time_s time;
	NMEA_Date_s date;
	NMEA_Latitude_s latitude;
	NMEA_Longitude_s longitude;
	NMEA_Speed_s speed;
	NMEA_Altitude_s altitude;
	NMEA_Satellite_s satellite;
	float horizontalDOP;
	float verticalDOP;
	float positionDOP;
	float trackAngle;
	float magneticVariation;

	uint8_t satTracked;
	int8_t gpsValid;
	int8_t timeValid;

	Eauto_manual_2D_3D_Fix eAutoManualFix;// = (Eauto_manual_2D_3D_Fix)'A';
	E2D_3D_Fix e2D3Dfix;// = (E2D_3D_Fix)'1';

}Gps_Data_s;

typedef struct
{
	UBX_Time_s time;
	UBX_Date_s date;
	float horizontalDOP;
	float verticalDOP;
	float positionDOP;
	float trackAngle;
	float magneticVariation;

	uint8_t satTracked;
	uint8_t satActive;
	int8_t gpsValid;
	int8_t timeValid;

	Eauto_manual_2D_3D_Fix eAutoManualFix;
	E2D_3D_Fix_Ubx e2D3Dfix;

}Gps_Data_Ubx_s;

#define GPS_DATA_INIT  {NMEA_TIME_INIT, NMEA_DATE_INIT, NMEA_LATITUDE_INIT, NMEA_LONGITUDE_INIT, NMEA_SPEED_INIT, NMEA_ALTITUDE_INIT, NMEA_SATELLITE_INIT, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 'A', '1'}

typedef struct
{
	CFG_Protocol_UART_s protocolUART;
	CFG_GNSS_System_s gnssSystem;
	CFG_Message_All_s message[NMEA_ALL_MSG_COUNT];
	CFG_Power_Management_s powerManagement;
	CFG_Power_Mode_s powerMode;
	CFG_Receiver_Management_s receiverManagment;
	NAV_Orbit_s orbit;
}
Gps_Config_s;

enum
{
	GPS = 0x01,
	SBAS = 0x02,
	Galileo = 0x04,
	BeiDou = 0x08,
	IMES = 0x10,
	QZSS = 0x20,
	GLONASS = 0x40,
} Gps_Gnss_e;


void handleBaudRateChanged(int8_t tendency);
void gpsInit();
uint8_t messageRate(uint8_t msgMask);
void registerPrepareUpdateBaudRate(void *callaback);
void proceedNMEABuffer(uint8_t *buffer);
void proceedSnifferUbxBuffer(uint8_t *buffer, uint16_t buffer_length);
void copyStruct(uint8_t *source, uint8_t *dest, uint8_t length);
Packet_State_s proceedUBXBuffer(uint8_t *buffer, uint16_t length);
uint8_t messageReceived();
uint8_t WaitToReceive(void);
uint8_t WaitToSend(void);
void InitGps();

typedef enum
{
	eGpsIdle = 0,
	eGpsSyncStart,
	eGpsSyncReady,
	eGpsWakeUp,
	eGpsWakeUpTimeout,
	eGpsSetMessages,
	eGpsSetGnss,
	eGpsSetPowerMode,
	eGpsSetRecManagement,
	eGpsSetConfiguration,
	eGpsSuccess,
	eGpsFailed,
} GPS_Init_State_e;

void gps_init_intern();
GPS_Init_State_e getGpsInitState();
void receivedGPS(uint8_t c);
void handleInitGps();
void snifferGPS(uint8_t c);

void setMessages();

#endif /* GPS_H_ */
