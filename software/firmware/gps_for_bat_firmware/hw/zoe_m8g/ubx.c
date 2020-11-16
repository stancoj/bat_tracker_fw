/*
 * ubx.c
 *
 *  Created on: 29. 10. 2018
 *      Author: David Rau
 */

#ifndef UBX_C_
#define UBX_C_

#include "ubx.h"

#include "utilities.h"
#include "hw_config.h"

#define CFG_MSG_RATE_MAX 6

uint8_t gBufferToSend[82];
uint8_t gBufferToSendLength = 0;
uint8_t ubx_msgs_id[100];
static UBX_Message_s lastDataMsg;

//sniffer
//static UBX_Message_s gpsData;



void (* gCallbackUBXsendBuffer)(uint8_t *msg, uint16_t length) = 0;


ACK_s requestId = {0x00};



CFG_Protocol_UART_s gConfigProtocolUART;
CFG_GNSS_System_s gConfigGnssSystem;

static Packet_State_s packetState = Packet_State_Idle;





void registerUBXsendCallback(void *callback)
{
	gCallbackUBXsendBuffer = callback;
}

void payloadToStruct(uint8_t *destStruct, uint8_t *sourcePayload, uint16_t length)
{
	uint8_t i;

	for(i = 0; i < length; i++)
	{
		destStruct[i] = sourcePayload[i];
	}
}

void sendGnssSystem(CFG_GNSS_System_s *msg)
{
	sendMessageUbx(UBX_CFG, UBX_CFG_GNSS, (uint8_t*)msg, sizeof(CFG_GNSS_System_s));
}

Packet_State_s setGnssSystem(uint8_t gpsAndQzss, uint8_t sbas, uint8_t galileo, uint8_t beiDou, uint8_t imes, uint8_t glonass)
{
	CFG_GNSS_System_s msg = CFG_GNSS_SYSTEM_INIT;
	uint8_t i;
	uint8_t enable[7] = {gpsAndQzss, sbas, galileo, beiDou, imes, gpsAndQzss, glonass};
	uint8_t maxChannel[] = CFG_GNSS_SYSTEM_CONFIG_BLOCK_MAX_CHAN;
	uint8_t minChannel[] = CFG_GNSS_SYSTEM_CONFIG_BLOCK_MIN_CHAN;

	for(i = 0; i < GNSS_SYSTEM_COUNT; i++)
	{
		msg.configBlocks[i].flags |= enable[i];
		msg.configBlocks[i].flags |= L1_L1CA_E1_B1;
		msg.configBlocks[i].gnssId = i;
		msg.configBlocks[i].maxTrackChannel = maxChannel[i];
		msg.configBlocks[i].minTrackChannel = minChannel[i];
	}

	return sendMessageUbx(UBX_CFG, UBX_CFG_GNSS, (uint8_t*)&msg, sizeof(CFG_GNSS_System_s));
}

Packet_State_s pollProtocol(I_O_Port_e port)
{
	uint8_t payload[1] = {port};
	return sendMessageUbx(UBX_CFG, UBX_CFG_PRT, payload, 1);
}



void processProtocolUART(uint8_t * payload)
{
	payloadToStruct((uint8_t*)&gConfigProtocolUART, payload, sizeof(CFG_Protocol_UART_s));
	packetState = Packet_State_Data_Received;
}

Packet_State_s setProtocolUART(UART_BaudRate baudRate, UART_CharLen charLen, UART_StopBits stopBits, UART_Parity parity)
{
	CFG_Protocol_UART_s msg = CFG_PROTOCOL_UART_INIT;
	msg.port = I_O_Port_UART_1;
	msg.baudRate = baudRate;
	msg.mode = (charLen | stopBits | parity) & CFG_PROTOCOL_UART_MODE_MASK;
	msg.inProtoMask = (In_Protocol_UBX | In_Protocol_NMEA) & CFG_PROTOCOL_IN_MASK;
	msg.outProtoMask = (Out_Protocol_UBX | Out_Protocol_NMEA) & CFG_PROTOCOL_OUT_MASK;
	return sendMessageUbx(UBX_CFG, UBX_CFG_PRT, (uint8_t*)&msg, sizeof(CFG_Protocol_UART_s));
}

Packet_State_s setMessageRateForCurrentPort(uint8_t msgClass, uint8_t msgID, uint8_t rate)
{
	CFG_Message_s msg;

	msg.msgClass = msgClass;
	msg.msgID = msgID;
	msg.rate = rate;
	return sendMessageUbx(UBX_CFG, UBX_CFG_MSG, (uint8_t*)&msg, sizeof(CFG_Message_s));
}

void setMessageRateForAllPorts(uint8_t msgClass, uint8_t msgID, uint8_t *rate)
{
	CFG_Message_All_s msg;
	uint8_t i;

	msg.msgClass = msgClass;
	msg.msgID = msgID;
	for(i = 0; i < sizeof(msg.rate); i++)
	{
		msg.rate[i] = rate[i];
	}
	sendMessageUbx(UBX_CFG, UBX_CFG_MSG, (uint8_t*)&msg, sizeof(CFG_Message_All_s));
}

void pollOrbit()
{
	uint8_t payload[0] = {};
	sendMessageUbx(UBX_NAV, UBX_NAV_ORB, payload, 0);
}

void getMessage(uint8_t *payload)
{
	CFG_Message_s msg;
	uint8_t i;

	for(i = 0; i < sizeof(CFG_Message_s); i++)
	{
		((uint8_t*)&msg)[i] = payload[i];
	}
	return;
}

void setInactiveState(uint32_t durationInMs) //duration = 0 - infinity time
{
	RXM_PowerManagement_0_s msg = RXM_POWER_MANAGEMENT_0_INIT;
	msg.duration = durationInMs;
	msg.flags = RXM_PM_FLAG_BACKUP;
	sendMessageUbx(UBX_RXM, UBX_RXM_PMREQ, (uint8_t*)&msg, sizeof(RXM_PowerManagement_0_s));
}

void setInactiveStateWithWakeUpSource(uint32_t durationInMs, WakeUp_Source wakeUpSrc)
{
	RXM_PowerManagement_1_s msg = RXM_POWER_MANAGEMENT_1_INIT;
	msg.duration = durationInMs;
	msg.wakeupSources = wakeUpSrc & WAKEUP_SOURCE_MASK;
	msg.flags = RXM_PM_FLAG_BACKUP;
	sendMessageUbx(UBX_RXM, UBX_RXM_PMREQ, (uint8_t*)&msg, sizeof(RXM_PowerManagement_1_s));
}

Packet_State_s setConfiguration(uint32_t saveMask, uint32_t loadMask, uint32_t clearMask)
{
	CFG_Configuration_s msg = CFG_CONFIGURATION_INIT;
	msg.saveMask = saveMask;
	msg.loadMask = loadMask;
	msg.clearMask = clearMask;
	return sendMessageUbx(UBX_CFG, UBX_CFG_CFG, (uint8_t*)&msg, sizeof(CFG_Configuration_s));
}

Packet_State_s pollMessage(uint8_t classId, uint8_t msgId)
{
	uint8_t payload[2] = {classId, msgId};
	return sendMessageUbx(UBX_CFG, UBX_CFG_MSG, payload, 2);
}

Packet_State_s getGnssSystem()
{
	uint8_t payload[0];
	return sendMessageUbx(UBX_CFG, UBX_CFG_GNSS, payload, 0);
}

Packet_State_s getPowerManagement()
{
	uint8_t payload[0];
	return sendMessageUbx(UBX_CFG, UBX_CFG_PM2, payload, 0);
}

Packet_State_s setPowerManagement()
{
	CFG_Power_Management_s msg = CFG_POWER_MANAGEMENT_INIT;
	msg.maxStartupStateDur = 0;//MAX_TIME_TRACK_FIX_S;
	msg.flags = UPDATE_EPH_WAKE_UP | MODE_ON_OFF | EXTINT_WAKE_ENABLED | EXTINT_BACKUP_ENABLED;
	msg.updatePeriod = 0;
	msg.searchPeriod = 0;//SHORT_SEARCH_PERIOD_MS;
	msg.onTime = 5;
//	msg.powerSetupValue = (uint8_t)powerSetup;
//	if(powerSetup == Power_Setup_Interval)
//	{
//		msg.period = period;
//		msg.onTime = msg.onTime;
//	}
	return sendMessageUbx(UBX_CFG, UBX_CFG_PM2, (uint8_t*)&msg, sizeof(CFG_Power_Management_s));
}

Packet_State_s getPowerMode()
{
	uint8_t payload[0];
	return sendMessageUbx(UBX_CFG, UBX_CFG_PMS, payload, 0);
}

Packet_State_s setPowerMode(Power_Setup_e powerSetup, uint16_t period, uint16_t onTime)
{
	CFG_Power_Mode_s msg = CFG_POWER_MODE_INIT;

	msg.powerSetupValue = (uint8_t)powerSetup;
	if(powerSetup == Power_Setup_Interval)
	{
		msg.period = period;
		msg.onTime = msg.onTime;
	}
	return sendMessageUbx(UBX_CFG, UBX_CFG_PMS, (uint8_t*)&msg, sizeof(CFG_Power_Mode_s));
}


/*
 * measR - 1000ms default
 * navR - 1 cycle default (10Hz)
 * timeR - time source
 */
Packet_State_s setRate(uint16_t measR, uint16_t navR, CFG_Rate_TimeSource timeR)
{
	CFG_Rate msg;
	msg.measRate=measR;
	msg.navRate=navR;
	msg.timeRef=timeR;
	return sendMessageUbx(UBX_CFG, UBX_CFG_RATE, (uint8_t*)&msg, sizeof(CFG_Rate));
}


Packet_State_s getReceiverManagement()
{
	uint8_t payload[0];
	return sendMessageUbx(UBX_CFG, UBX_CFG_RXM, payload, 0);
}

Packet_State_s setReceiverManagement(Low_Power_Mode_e powerMode)
{
	CFG_Receiver_Management_s msg = CFG_RECEIVER_MANAGEMENT_INIT;

	msg.lpMode = (uint8_t)powerMode;
	return sendMessageUbx(UBX_CFG, UBX_CFG_RXM, (uint8_t*)&msg, sizeof(CFG_Receiver_Management_s));
}


uint8_t processAcknowledge(ACK_s *answer, ACK_s *request)
{
	if(answer->classID == request->classID && answer->msgID == request->msgID)
	{
		request->classID = UBX_Class_Id_None;
		request->msgID = 0x00;
		return 1;
	}
	else
	{
		return 0;
	}
}

Packet_State_s sendMessageUbx(CLASS_ID classId, uint8_t msgId, uint8_t *payload, uint16_t payload_length)
{
	uint16_t i;
	ACK_s ret;

	gBufferToSend[0] = 0xb5;
	gBufferToSend[1] = 0x62;
	gBufferToSend[2] = ret.classID = requestId.classID = classId;
	gBufferToSend[3] = ret.msgID = requestId.msgID = msgId;
	gBufferToSend[4] = payload_length & 0xff;
	gBufferToSend[5] = (payload_length >> 8) & 0xff;
	for(i = 0; i < payload_length; i++)
	{
		gBufferToSend[i + 6] = payload[i];
	}

	appendCheckSumToMessage(gBufferToSend, 2, 6 + payload_length);
	gBufferToSendLength = 8 + payload_length;
	if(gCallbackUBXsendBuffer != 0)
	{
		gCallbackUBXsendBuffer(gBufferToSend, 8 + payload_length);
		return Packet_State_Sent;
	}
	return Packet_State_Unsent;
}

uint8_t parseUbxMessage(uint8_t *buffer, uint8_t length, UBX_Message_s *msg)
{
	uint8_t i;

	if(checkChecksumUBX(buffer, length))
	{
		return 1;
	}
	msg->classId = buffer[2];
	msg->msgId = buffer[3];
	msg->payload_length = buffer[4] | (buffer[5] << 8);
	for(i = 0; i < msg->payload_length; i++)
	{
		msg->payload[i] = buffer[6 + i];
	}
	return 0;
}

Packet_State_s proceedUbxBuffer(uint8_t *buffer, uint16_t buffer_length)
{
	UBX_Message_s msg;

	if (parseUbxMessage(buffer, buffer_length, &msg))
	{
		return Packet_State_Failed;
	}

	if(msg.classId == UBX_NAV)
	{
		copyMemory((uint8_t*)&msg, (uint8_t*)&lastDataMsg, sizeof(UBX_Message_s));
		return Packet_State_Accepted;
	}
	else if(msg.classId == UBX_ACK)
	{
		if(requestId.classID == msg.payload[0] && requestId.msgID == msg.payload[1])
		{
			requestId.classID = UBX_Class_Id_None;
			if(msg.msgId)
			{
				return Packet_State_Accepted;
			}
			else
			{
				return Packet_State_Not_Accepted;
			}
		}
		else
		{
			return Packet_State_Failed;
		}
	}
	else
	{
		return Packet_State_Data_Received;
	}
}

/*
Packet_State_s proceedSnifferUbxBuffer(uint8_t *buffer, uint16_t buffer_length)
{

	UBX_Message_s msg;

	if(buffer_length > 92)
	{
		return Packet_State_Not_Accepted;
	}
	else
	{
		if(parseUbxMessage(buffer, buffer_length, &msg))
		{
			return Packet_State_Failed;
		}
	}

	if(msg.classId == UBX_NAV)
	{
		if(msg.msgId == UBX_NAV_DOP)
		{
			UBX_DOP_s gGPSdop = parseUbxNavDop(&msg);

			gGpsData.horizontalDOP = gGPSdop.hDOP;
			gGpsData.verticalDOP = gGPSdop.vDOP;
			gGpsData.positionDOP = gGPSdop.pDOP;
		}

		copyMemory((uint8_t*)&msg, (uint8_t*)&gpsData, sizeof(UBX_Message_s));
		return Packet_State_Accepted;
	}

	copyMemory((uint8_t*)&msg, (uint8_t*)&lastDataMsg, sizeof(UBX_Message_s));
	return Packet_State_Accepted;
}
*/

void getReceivedUbxId(CLASS_ID *classId, uint8_t *msgId)
{
	*classId = lastDataMsg.classId;
	*msgId = lastDataMsg.msgId;
}

void getReceivedUbxMessage(uint8_t *data)
{
	copyMemory(lastDataMsg.payload, data, lastDataMsg.payload_length);
}

int8_t checkChecksumUBX(uint8_t * buffer, uint16_t buffer_length)
{
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	uint8_t i;

	for(i = 2; i < buffer_length - 2; i++)
	{
		ck_a += buffer[i];
		ck_b += ck_a;
	}

	if(ck_a == buffer[buffer_length - 2] && ck_b == buffer[buffer_length - 1]) return 0;
	else return 1;
}

uint16_t appendCheckSumToMessage(uint8_t *buffer, uint16_t startPos, uint16_t endPos)
{
	uint16_t i;
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;

	for (i = startPos; i < endPos; i++)
    {
    	ck_a += buffer[i];
    	ck_b += ck_a;
    }

    buffer[i++] = ck_a;
    buffer[i++] = ck_b;

    return i;
}

UBX_DOP_s parseUbxNavDop(UBX_Message_s* msg)
{
	UBX_DOP_s data;

	data.iTOW = (msg->payload[3] << 24) | (msg->payload[2] << 16) | (msg->payload[1] << 8) | msg->payload[0];
	data.gDOP = ((msg->payload[5] << 8) | msg->payload[4])/100.0f;
	data.pDOP = ((msg->payload[7] << 8) | msg->payload[6])/100.0f;
	data.tDOP = ((msg->payload[9] << 8) | msg->payload[8])/100.0f;
	data.vDOP = ((msg->payload[11] << 8) | msg->payload[10])/100.0f;
	data.hDOP = ((msg->payload[13] << 8) | msg->payload[12])/100.0f;
	data.nDOP = ((msg->payload[15] << 8) | msg->payload[14])/100.0f;
	data.eDOP = ((msg->payload[17] << 8) | msg->payload[16])/100.0f;

	return data;
}

uint8_t parseUbxNavPvtFix(UBX_Message_s* msg)
{
	uint8_t fix_type;

	fix_type = '0' + msg->payload[20];

	return fix_type;
}

UBX_Time_s parseUbxNavPvtTime(UBX_Message_s* msg)
{
	UBX_Time_s time;

	time.sec = msg->payload[10];
	time.min = msg->payload[9];
	time.hour = msg->payload[8];

	return time;
}

UBX_Date_s parseUbxNavPvtDate(UBX_Message_s* msg)
{
	UBX_Date_s date;

	date.day = msg->payload[7];
	date.month = msg->payload[6];
	date.year = msg->payload[5] << 8 | msg->payload[4];

	return date;
}

uint8_t parseUbxNavPvtGpsValid(UBX_Message_s* msg)
{
	return (msg->payload[21] & 0x01);
}

uint8_t parseUbxNavPvtTimeValid(UBX_Message_s* msg)
{
	return (msg->payload[11] & (1 << 1) >> 1);
}

uint8_t parseUbxNavPvtSatUsed(UBX_Message_s* msg)
{
	return (msg->payload[23]);
}

#endif /* UBX_C_ */
