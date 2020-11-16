/*
 * gpc.c
 *
 *  Created on: 29. 10. 2018
 *      Author: David Rau
 */

#ifndef GPC_C_
#define GPC_C_

#include "gps.h"

static uint8_t msgReceived = 0;

static GPS_Init_State_e gpsInitState = eGpsIdle;
static FlagStatus secondCallState = RESET;
static uint64_t gpsTimestamp = 0;
static uint8_t gpsMsgsOrder = 0;
static uint8_t gpsMsgsAll[NMEA_ALL_MSG_COUNT] = UBX_MSG_NMEA_MSG_IDS;
static Packet_State_s packetState = Packet_State_Idle;

uint8_t SendFlag = 0;
uint8_t CorrectMsg = 0;
uint8_t gps_fix;

Gps_Data_s gGpsData;
Gps_Data_Ubx_s gGpsDataUbx;

volatile uint8_t ubx_received_flag = 0;
volatile uint8_t nmea_received_flag = 0;

static Gps_Config_Msg_s gGpsConfigMsgs[GPS_CONFIG_MSG_LENGTH] = {{1, NMEA_RMC}/*, {1, NMEA_GSA}, {1, NMEA_GGA}, {1, NMEA_GLL}*/};

static uint8_t gBuffer0 [MAXIMUM_GPS_BUFFER_LENGTH];
static uint8_t gBuffer1 [MAXIMUM_GPS_BUFFER_LENGTH];
static uint8_t gUsedBuffer = 0;

static Packet_Type_e packetType = No_Packet_Type;

static uint8_t *bufferToProceed = 0;

void (* gCallbackPrepareUpdateBaudRate)(void) = 0;



void registerPrepareUpdateBaudRate(void *callback)
{
	gCallbackPrepareUpdateBaudRate = callback;
}


void gps_init_intern()
{
	gpsInitState = eGpsSyncStart;
	gpsTimestamp = rtc_getMs();
}

GPS_Init_State_e getGpsInitState()
{
	return gpsInitState;
}

void handleInitGps()
{
	uint64_t timeNowMs = rtc_getMs();
	switch(gpsInitState)
	{
		case eGpsIdle:
			break;
		case eGpsSyncStart:
			if((gpsTimestamp + GPSI_MSG_INIT_REC_TIMEOUT_MS) <= timeNowMs)
			{
				gpsInitState = eGpsWakeUp;
			}
			break;
		case eGpsWakeUp:
			{
				uint8_t b = 0x00;
				USART2_PutBuffer(&b, 1);
				gpsTimestamp = timeNowMs;
				gpsInitState = eGpsWakeUpTimeout;
			}
			break;
		case eGpsWakeUpTimeout:
		case eGpsSyncReady:
			if((gpsTimestamp + GPSI_DELAY_AFTER_SYNC_MS) <= timeNowMs)
			{
				secondCallState = RESET;
				gpsInitState = eGpsSetMessages;
				gpsMsgsOrder = 0;
				packetState = setMessageRateForCurrentPort(UBX_MSG_NMEA_CLASS_ID, gpsMsgsAll[gpsMsgsOrder], messageRate(gpsMsgsAll[gpsMsgsOrder]));
				gpsTimestamp = timeNowMs;
			}
			break;

		case eGpsSetMessages:
			if(packetState == Packet_State_Accepted)
			{
				secondCallState = RESET;
				gpsMsgsOrder++;
				if(gpsMsgsOrder >= NMEA_ALL_MSG_COUNT)
				{
					gpsInitState = eGpsSetGnss;
					packetState = setGnssSystem(1, 0, 0, 0, 0, 0);
				}
				else
				{
					packetState = setMessageRateForCurrentPort(UBX_MSG_NMEA_CLASS_ID, gpsMsgsAll[gpsMsgsOrder], messageRate(gpsMsgsAll[gpsMsgsOrder]));
				}
				gpsTimestamp = timeNowMs;
			}
			else if((gpsTimestamp + GPSI_NO_ANS_REC_TIMEOUT_MS) <= timeNowMs)
			{
				if(secondCallState == SET)
				{
					secondCallState = RESET;
					gpsInitState = eGpsFailed;
				}
				else
				{
					secondCallState = SET;
					gpsInitState = eGpsSetMessages;
					packetState = setMessageRateForCurrentPort(UBX_MSG_NMEA_CLASS_ID, gpsMsgsAll[gpsMsgsOrder], messageRate(gpsMsgsAll[gpsMsgsOrder]));
					gpsTimestamp = timeNowMs;
				}
			}
			break;
		case eGpsSetGnss:
			if(packetState == Packet_State_Accepted)
			{
				secondCallState = RESET;
				gpsInitState = eGpsSetPowerMode;
				packetState = setPowerManagement();
				gpsTimestamp = timeNowMs;
			}
			else if((gpsTimestamp + GPSI_NO_ANS_REC_TIMEOUT_MS) <= timeNowMs)
			{

				if(secondCallState == SET)
				{
					secondCallState = RESET;
					gpsInitState = eGpsFailed;
				}
				else
				{
					secondCallState = SET;
					gpsInitState = eGpsSetGnss;
					packetState = setGnssSystem(1, 0, 0, 0, 0, 0);
				}
			}
			break;
		case eGpsSetPowerMode:
			if(packetState == Packet_State_Accepted)
			{
				secondCallState = RESET;
				gpsInitState = eGpsSetRecManagement;
				packetState = setReceiverManagement(Low_Power_Mode_Power_Save);
				gpsTimestamp = timeNowMs;
			}
			else if((gpsTimestamp + GPSI_NO_ANS_REC_TIMEOUT_MS) <= timeNowMs)
			{
				if(secondCallState == SET)
				{
					secondCallState = RESET;
					gpsInitState = eGpsFailed;
				}
				else
				{
					secondCallState = SET;
					gpsInitState = eGpsSetPowerMode;
					packetState = setPowerManagement();
					gpsTimestamp = timeNowMs;
				}
			}
			break;
		case eGpsSetRecManagement:
			if(packetState == Packet_State_Accepted)
			{
				secondCallState = RESET;
				gpsInitState = eGpsSetConfiguration;
				uint32_t saveMask = Configuration_ioPort | Configuration_msgConf | Configuration_rxmConf;
				packetState = setConfiguration(saveMask, 0, 0);
				gpsTimestamp = timeNowMs;
			}
			else if((gpsTimestamp + GPSI_NO_ANS_REC_TIMEOUT_MS) <= timeNowMs)
			{
				if(secondCallState == SET)
				{
					secondCallState = RESET;
					gpsInitState = eGpsFailed;
				}
				else
				{
					secondCallState = SET;
					gpsInitState = eGpsSetRecManagement;
					packetState = setReceiverManagement(Low_Power_Mode_Power_Save);
					gpsTimestamp = timeNowMs;
				}
			}
			break;
		case eGpsSetConfiguration:
			if(packetState == Packet_State_Accepted)
			{
				secondCallState = RESET;
				gpsInitState = eGpsSuccess;
			}
			if((gpsTimestamp + GPSI_NO_ANS_REC_TIMEOUT_MS) <= timeNowMs)
			{
				if(secondCallState == SET)
				{
					secondCallState = RESET;
					gpsInitState = eGpsFailed;
				}
				else
				{
					secondCallState = SET;
					gpsInitState = eGpsSetConfiguration;
					uint32_t saveMask = Configuration_ioPort | Configuration_msgConf | Configuration_rxmConf;
					packetState = setConfiguration(saveMask, 0, 0);
					gpsTimestamp = timeNowMs;
				}
			}
			break;
		case eGpsSuccess:
		case eGpsFailed:
			gpsInitState = eGpsIdle;
			break;
	}
}

void receivedGPS(uint8_t c)
{
	static uint8_t bufferPointer = 0;
    static uint8_t previousChar = 0;
    static uint16_t UBXpacketEndPos = 0;

    uint8_t * buffer;
    buffer = gUsedBuffer ? gBuffer1 : gBuffer0;
    if(c == NMEA_Head && packetType != UBX_Packet_Type)
    {
        packetType = NMEA_Packet_Type;
    	bufferPointer = 0;
    	if(gpsInitState == eGpsSyncReady)
		{
			gpsInitState = eGpsSyncStart;
			gpsTimestamp = rtc_getMs();
		}
    }
    else if(previousChar == UBX_Packet_Head_Part_1 && c == UBX_Packet_Head_Part_2 && packetType == No_Packet_Type)
    {
    	packetType = UBX_Packet_Type;
    	UBXpacketEndPos = 0;
    	bufferPointer = 1;
    }

    if(bufferPointer > MAXIMUM_GPS_BUFFER_LENGTH)
    {
        bufferPointer = 0;
        return;
    }

    buffer[bufferPointer++] = c;

    if(packetType == UBX_Packet_Type && bufferPointer == UBX_START_TO_PL_CNT)
    {
    	UBXpacketEndPos = c;
    	UBXpacketEndPos <<= 8;
    	UBXpacketEndPos |= previousChar;
    	UBXpacketEndPos += UBX_START_TO_PL_CNT + UBX_PL_TO_END_CNT;
    }
    if(UBXpacketEndPos == bufferPointer && packetType == UBX_Packet_Type)
    {
    	bufferPointer = 0;
    	packetType = No_Packet_Type;

    	if(gUsedBuffer==1)
		{
			gUsedBuffer = 0;
		}
		else
		{
			gUsedBuffer = 1;
		}
		bufferToProceed = buffer;

		packetState = proceedUbxBuffer(buffer, UBXpacketEndPos);
    }

    if(c == NMEA_End && packetType == NMEA_Packet_Type)
    {
    	bufferPointer = 0;
        packetType = No_Packet_Type;
		if(gUsedBuffer==1)
		{
			gUsedBuffer = 0;
		}
		else
		{
			gUsedBuffer = 1;
		}
        bufferToProceed = buffer;
        proceedNMEABuffer(buffer);

    }

    previousChar = c;
}


void snifferGPS(uint8_t c)
{
	static uint8_t bufferPointer = 0;
    static uint8_t previousChar = 0;
    static uint16_t UBXpacketEndPos = 0;

    uint8_t * buffer;
    buffer = gUsedBuffer ? gBuffer1 : gBuffer0;

    if(c == NMEA_Head && packetType != UBX_Packet_Type)
    {
        packetType = NMEA_Packet_Type;
    	bufferPointer = 0;
    	if(gpsInitState == eGpsSyncReady)
		{
			gpsInitState = eGpsSyncStart;
			gpsTimestamp = rtc_getMs();
		}
    }
    else if(previousChar == UBX_Packet_Head_Part_1 && c == UBX_Packet_Head_Part_2 && packetType == No_Packet_Type)
    {
    	packetType = UBX_Packet_Type;
    	UBXpacketEndPos = 0;
    	bufferPointer = 1;
    }

    if(bufferPointer > MAXIMUM_GPS_BUFFER_LENGTH)
    {
        bufferPointer = 0;
        return;
    }

    buffer[bufferPointer++] = c;

    if(packetType == UBX_Packet_Type && bufferPointer == UBX_START_TO_PL_CNT)
    {
    	UBXpacketEndPos = c;
    	UBXpacketEndPos <<= 8;
    	UBXpacketEndPos |= previousChar;
    	UBXpacketEndPos += UBX_START_TO_PL_CNT + UBX_PL_TO_END_CNT;
    }
    if(UBXpacketEndPos == bufferPointer && packetType == UBX_Packet_Type)
    {
    	bufferPointer = 0;
    	packetType = No_Packet_Type;

    	if(gUsedBuffer==1)
		{
			gUsedBuffer = 0;
		}
		else
		{
			gUsedBuffer = 1;
		}
		bufferToProceed = buffer;

		proceedSnifferUbxBuffer(buffer, UBXpacketEndPos);
    }

    if(c == NMEA_End && packetType == NMEA_Packet_Type)
    {
    	bufferPointer = 0;
        packetType = No_Packet_Type;
		if(gUsedBuffer==1)
		{
			gUsedBuffer = 0;
		}
		else
		{
			gUsedBuffer = 1;
		}
        bufferToProceed = buffer;
        proceedNMEABuffer(buffer);
    }

    previousChar = c;
}


uint8_t messageRate(uint8_t msgMask)
{
	uint8_t i;
	for(i = 0; i < GPS_CONFIG_MSG_LENGTH; i++)
	{
		if(msgMask == gGpsConfigMsgs[i].mask)
		{
			return gGpsConfigMsgs[i].rate;
		}
	}
	return 0;
}


uint16_t gWhichSystem = 0;

void proceedNMEABuffer(uint8_t *buffer)
{
	NMEA_Params_s msg = checkMessage_NMEA(buffer);

	if(msg.system == NMEA_System_Error || msg.message == NMEA_Message_Error )
	{
		return;
	}
	if(msg.system == NMEA_System_Main)
	{
		gWhichSystem |= 0x01;
	}
	else if(msg.system == NMEA_System_Glonass)
	{
		gWhichSystem |= 0x02;
	}
	else if(msg.system == NMEA_System_GPS)
	{
		gWhichSystem |= 0x04;
	}

	nmea_received_flag = 0;

	if(msg.message == NMEA_Message_GSA)
	{
		GSA_Data_s gsaData = proceedGSAparsing(buffer);
		uint8_t i;

		gGpsData.e2D3Dfix = gsaData.e2D3Dfix;
		gGpsData.eAutoManualFix = gsaData.eAutoManualFix;
		gGpsData.horizontalDOP = gsaData.horizontalDilutionOfPrecision;
		gGpsData.verticalDOP = gsaData.verticalDilutionOfPrecision;
		gGpsData.positionDOP = gsaData.positionDilutionOfPrecision;
		for(i = 0; i < sizeof(NMEA_Satellite_s); i++)
		{
			*((uint8_t*)&gGpsData.satellite + i) = *((uint8_t*)&gsaData.satellite + i);
		}
	}
	else if(msg.message == NMEA_Message_RMC)
	{
		RMC_Data_s rmcData = proceedRMCparsing(buffer);
		gGpsData.time = rmcData.time;
		gGpsData.date = rmcData.date;
		gGpsData.gpsValid = rmcData.gpsValid;
		gGpsData.timeValid = rmcData.timeValid;
		if(rmcData.gpsValid)
		{
			gGpsData.latitude = rmcData.latitude;
			gGpsData.longitude = rmcData.longitude;
			gGpsData.speed = rmcData.speed;
			gGpsData.trackAngle = rmcData.trackAngle;
			gGpsData.magneticVariation = rmcData.magneticVariation;
		}
	}
	else if(msg.message == NMEA_Message_GGA)
	{
		GGA_Data_s ggaData = proceedGGAparsing(buffer);
		gGpsData.time = ggaData.time;
		gGpsData.gpsValid = ggaData.gpsValid;
		if(ggaData.gpsValid)
		{
			gGpsData.latitude = ggaData.latitude;
			gGpsData.longitude = ggaData.longitude;
			gGpsData.satTracked = ggaData.satTracked;
			gGpsData.altitude = ggaData.altitude;
		}


	}
	else if(msg.message == NMEA_Message_GLL)
	{

	}
	msgReceived = 1;

	if(gpsInitState == eGpsSyncStart && msg.message != NMEA_Message_TXT)
	{
		gpsInitState = eGpsSyncReady;
		gpsTimestamp = rtc_getMs();
	}
}

void proceedSnifferUbxBuffer(uint8_t *buffer, uint16_t buffer_length)
{
	UBX_Message_s msg;

	if(buffer_length > 100)
	{
		return;
	}
	else
	{
		if(parseUbxMessage(buffer, buffer_length, &msg))
		{
			return;
		}
	}

	ubx_received_flag = 0;

	if(msg.classId == UBX_NAV)
	{
		if(msg.msgId == UBX_NAV_DOP)
		{
			UBX_DOP_s gGPSdop = parseUbxNavDop(&msg);

			gGpsDataUbx.horizontalDOP = gGPSdop.hDOP;
			gGpsDataUbx.verticalDOP = gGPSdop.vDOP;
			gGpsDataUbx.positionDOP = gGPSdop.pDOP;
		}
		else if(msg.msgId == UBX_NAV_PVT)
		{
			UBX_Time_s gTime = parseUbxNavPvtTime(&msg);
			gGpsDataUbx.time.sec = gTime.sec;
			gGpsDataUbx.time.min = gTime.min;
			gGpsDataUbx.time.hour = gTime.hour;

			UBX_Date_s gDate = parseUbxNavPvtDate(&msg);
			gGpsDataUbx.date.day = gDate.day;
			gGpsDataUbx.date.month = gDate.month;
			gGpsDataUbx.date.year = gDate.year;

			gGpsDataUbx.e2D3Dfix = parseUbxNavPvtFix(&msg);
			gGpsDataUbx.gpsValid = parseUbxNavPvtGpsValid(&msg);
			gGpsDataUbx.timeValid = parseUbxNavPvtTimeValid(&msg);
			gGpsDataUbx.satTracked = parseUbxNavPvtSatUsed(&msg);
		}
	}
}

uint8_t messageReceived()
{
	uint8_t m = msgReceived;
	msgReceived = 0;
	return m;
}


void InitGps()
{
	uint8_t i = 0;
	uint64_t time = 0;

	time = rtc_getMs();

	while (((rtc_getMs() - time) < 1000) && (!CorrectMsg))
	{
		USART2_CheckDmaReception();
		//i++;
	}

	if (!CorrectMsg)
	{
		//USART2_UpdateBaudRate(UART_BaudRate_9_6k);
		time = rtc_getMs();
		while((rtc_getMs() - time) < 50){}

		packetState = setProtocolUART(UART_BaudRate_115_2k, UART_CharLen_8bit,UART_StopBits_1, UART_Parity_No);
		while((rtc_getMs() - time) < 100){}

		USART2_UpdateBaudRate(UART_BaudRate_115_2k);
		while((rtc_getMs() - time) < 100){}

		if (WaitToSend())
		{
			packetState = setRate(100, 1, GPS_time);
		}
		else
		{
			WaitToSend();
			packetState = setRate(100, 1, GPS_time);
		}

		if(WaitToReceive() == Packet_State_Accepted)
		{

			for (i = 0; i < GPS_CONFIG_MSG_LENGTH; i++)
			{

				if (WaitToSend())
				{
					packetState = setMessageRateForCurrentPort(UBX_MSG_NMEA_CLASS_ID, gpsMsgsAll[i],messageRate(gpsMsgsAll[i]));
				}
				if(WaitToReceive() == Packet_State_Accepted)
				{

				}
				else
				{
					WaitToSend();
					packetState = setMessageRateForCurrentPort(UBX_MSG_NMEA_CLASS_ID, gpsMsgsAll[i],messageRate(gpsMsgsAll[i]));
				}

			}
		}
	}
}


uint8_t WaitToReceive(void)
{
	volatile uint64_t Time=0;
	Time = rtc_getMs();

	while((packetState != Packet_State_Accepted))
	{
		USART2_CheckDmaReception();
		if(!((rtc_getMs() - Time) < 3000))
		{
			return packetState;
		}
	}

	return packetState;
}


uint8_t WaitToSend(void)
{
	volatile uint64_t Time=0;
	Time = rtc_getMs();
	SendFlag=0;

	    while(!SendFlag)
	    {
	    	USART2_CheckDmaReception();

	        if(!((rtc_getMs() - Time) < 3000))
			{
	        	return 0;
			}
	    }

	return SendFlag;
}

#endif /* GPC_C_ */
