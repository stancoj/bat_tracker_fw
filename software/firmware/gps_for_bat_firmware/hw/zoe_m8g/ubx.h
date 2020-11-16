/*
 * ubx.h
 *
 *  Created on: 29. 10. 2018
 *      Author: David Rau
 */

#ifndef UBX_H_
#define UBX_H_

#include <stdint.h>

typedef enum
{
	WakeUp_Source_UART_RX = 0x08,
	WakeUp_Source_EXTINT0 = 0x20,
	WakeUp_Source_EXTINT1 = 0x40,
	WakeUp_Source_SPI_CS = 0x80,
} WakeUp_Source;

enum
{
	UBX_Packet_Head_Part_1 = 0xb5,
	UBX_Packet_Head_Part_2 = 0x62
} UBX_Packet;

#define WAKEUP_SOURCE_MASK 0xe8

typedef struct
{
	uint32_t ioPort		:1;
	uint32_t msgConf	:1;
	uint32_t infMsg		:1;
	uint32_t navConf	:1;
	uint32_t rwmConf	:1;
	uint32_t rsvrd_1	:3;
	uint32_t senConf 	:1;
	uint32_t rinvConf	:1;
	uint32_t antConf	:1;
	uint32_t logConf	:1;
	uint32_t ftsConf	:1;
	uint32_t rsvrd_2	:19;
} Clear_Save_Load_Mask_Bits;

#define RXM_PM_FLAG_BACKUP	0x02
#define RXM_PM_FLAG_FORCE	0x04

typedef union
{
	uint32_t All;
	Clear_Save_Load_Mask_Bits Bits;
} ClearMask_u, SaveMask_u, LoadMask_u;

typedef enum _I_O_Ports_e_
{
	I_O_Port_I2C = 0,
	I_O_Port_UART_1 = 1,
	I_O_Port_UART_2 = 2,//not used
	I_O_Port_USB = 3,
	I_O_Port_SPI = 4,
} I_O_Port_e;

typedef enum _E2D_3D_Fix_Ubx
{
	uNofix = '0',
	uDeadReckoning = '1',
	u2Dfix = '2',
	u3Dfix = '3',
	uGNSSdeadReckoningCombined = '4',
	uTimeOnlyfix = '5',
}E2D_3D_Fix_Ubx;

typedef struct
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint32_t utcDaySeconds;
} UBX_Time_s;

#define NMEA_TIME_INIT {0, 0, 0, 0}

typedef struct
{
	uint8_t month;
	uint8_t day;
	uint16_t year;
} UBX_Date_s;

typedef struct
{
	uint8_t msgClass;
	uint8_t msgID;
	uint8_t rate[6];
} CFG_Message_All_s;

typedef struct
{
	uint8_t msgClass;
	uint8_t msgID;
	uint8_t rate;
} CFG_Message_s;

//typedef struct _CFG_Configuration_s
//{
//	ClearMask_u clearMask;
//	SaveMask_u saveMask;
//	LoadMask_u loadMask;
////	uint8_t deviceMask;
//} CFG_Configuration_s;

typedef struct
{
	uint32_t duration;
	uint32_t flags;
} RXM_PowerManagement_0_s;

#define RXM_POWER_MANAGEMENT_0_INIT {0, 0}

typedef struct
{
	uint8_t version;
	uint8_t reserved_1[3];
	uint32_t duration;
	uint32_t flags;
	uint32_t wakeupSources;
} RXM_PowerManagement_1_s;

#define RXM_POWER_MANAGEMENT_1_INIT {0, {0, 0, 0}, 0, 0, 0}

enum In_Protocol_e
{
	In_Protocol_UBX = 0x01,
	In_Protocol_NMEA = 0x02,
	In_Protocol_RTCM2 = 0x04,
	In_Protocol_RTCM3 = 0x20,
};

#define CFG_PROTOCOL_IN_MASK 0x27

enum Out_Protocol_e
{
	Out_Protocol_UBX = 0x01,
	Out_Protocol_NMEA = 0x02,
	Out_Protocol_RTCM3 = 0x20,
};

#define CFG_PROTOCOL_OUT_MASK 0x23

typedef enum
{
	UART_BaudRate_4_8k = 4800,
	UART_BaudRate_9_6k = 9600,
	UART_BaudRate_19_2k = 19200,
	UART_BaudRate_38_4k = 38400,
	UART_BaudRate_57_6k = 57600,
	UART_BaudRate_115_2k = 115200,
	UART_BaudRate_230_4k = 230400,
	UART_BaudRate_460_8k = 460800,
} UART_BaudRate;

typedef enum
{
	UART_CharLen_7bit_with_parity = 0x80,
	UART_CharLen_8bit = 0xc0,
} UART_CharLen;

typedef enum
{
	UART_Parity_Even = 0x0000,
	UART_Parity_Odd = 0x0200,
	UART_Parity_No = 0x0800,
} UART_Parity;

typedef enum
{
	UART_StopBits_0_5 = 0x3000,
	UART_StopBits_1 = 0x0000,
	UART_StopBits_1_5 = 0x1000,
	UART_StopBits_2 = 0x2000,
} UART_StopBits;

#define CFG_PROTOCOL_UART_MODE_MASK 0x3ac0

typedef struct
{
	uint8_t port;
	uint8_t reserved_1;
	uint16_t txReady;
	uint32_t mode;
	uint32_t baudRate;
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	uint8_t reserved_2[2];
} CFG_Protocol_UART_s;

typedef enum
{
	UTC_time = 0x00,
	GPS_time = 0x01,
	GLO_time = 0x02,
	BDS_time = 0x03,
	GAL_time = 0x04,
} CFG_Rate_TimeSource;

enum UBX_CFG_e
{
	UBX_CFG_PRT = 0x00, //set protocol
	UBX_CFG_MSG = 0x01, //
	UBX_CFG_RATE = 0x08,
	UBX_CFG_CFG = 0x09,
	UBX_CFG_RXM = 0x11,
	UBX_CFG_PM2 = 0x3b,
	UBX_CFG_GNSS = 0x3e,
	UBX_CFG_PMS = 0x86,
};

typedef struct
{
	uint16_t measRate;  // 1000ms default
	uint16_t navRate;   // 1 default
	uint16_t timeRef;   // 1 default
} CFG_Rate;

#define CFG_PROTOCOL_UART_INIT {0, 0, 0, 0, 0, 0, 0, 0, {0, 0}}

typedef enum
{
	GNSS_System_GPS = 0,
	GNSS_System_SBAS = 1,
	GNSS_System_Galileo = 2,
	GNSS_System_BeiDou = 3,
	GNSS_System_IMES = 4,
	GNSS_System_QZSS = 5,
	GNSS_System_GLONASS = 6,
} GNSS_System_e;

typedef struct
{
	uint16_t mask;
	uint8_t dynModel;
	uint8_t fixMode;
	int32_t fixedAlt;
	uint32_t fixedAltVar;
	int8_t minElev;
	uint8_t drLimit;
	uint16_t pDOP;
	uint16_t tDOP;
	uint16_t pAcc;
	uint16_t tAcc;
	uint8_t staticHoldThresh;

} CFG_Navigation_Engine;

#define GNSS_SYSTEM_COUNT 7

#define CFG_GNSS_SYSTEM_FLAG_ENABLE_MASK 0x01;

#define CFG_GNSS_SYSTEM_CONFIG_BLOCK_MIN_CHAN {8, 1, 4, 8, 0, 0, 8}
#define CFG_GNSS_SYSTEM_CONFIG_BLOCK_MAX_CHAN {16, 3, 8, 16, 8, 3, 14}

typedef struct
{
	uint8_t gnssId;
	uint8_t minTrackChannel;
	uint8_t maxTrackChannel;
	uint8_t reserved_1;
	uint32_t flags;
}GNSS_Config_Block_s;

enum GNSS_Frequency_e
{
	L1_L1CA_E1_B1 = 0x00010000,
	L1S = 0x00040000,
};

#define GNSS_CONFIG_BLOCK_INIT {0, 0, 0, 0, 0}


#define GNSS_CONFIG_BLOCK_COUNT 7

typedef struct
{
	uint8_t gnssId;
	uint8_t svId;
	uint8_t svFlag;
	uint8_t eph;
	uint8_t alm;
	uint8_t otherOrb;
} NAV_Orbit_Satellite_s;

#define NAV_ORBIT_SATELLITE_INIT {0, 0, 0, 0, 0, 0}

typedef struct
{
	uint32_t iTOW;
	uint8_t version;
	uint8_t numSv;
	uint8_t reserved_1[2];
	NAV_Orbit_Satellite_s satellites[12];
} NAV_Orbit_s;

#define NAV_ORBIT_INIT {0, 0, 0, {0}, {NAV_ORBIT_SATELLITE_INIT}}

typedef struct
{
	uint8_t classId;
	uint8_t msgId;
	uint16_t payload_length;
	uint8_t payload[92];
	uint16_t crc;
} UBX_Message_s;

typedef enum
{
	Power_Setup_Full_Power = 0x00,
	Power_Setup_Balanced = 0x01,
	Power_Setup_Interval = 0x02,
	Power_Setup_Aggresive_1Hz = 0x03,
	Power_Setup_Aggresive_2Hz = 0x04,
	Power_Setup_Aggresive_4Hz = 0x05,
	Power_Setup_Invalid = 0xff,
} Power_Setup_e;

#define OPT_PERFORMANCE				0 << 1
#define OPT_POWER_SAVE				1 << 1

#define EXTINT_SEL_EXTINT0 			0 << 4

#define EXTINT_WAKE_DISABLED 		0 << 5
#define EXTINT_WAKE_ENABLED			1 << 5

#define EXTINT_BACKUP_DISABLED		0 << 6
#define EXTINT_BACKUP_ENABLED		1 << 6

#define EXTINT_INACTIVE_DISABLED	0 << 7
#define EXTINT_INACTIVE_ENABLED		1 << 7

#define LIMIT_PEAK_CURR_DISABLED	0 << 8
#define LIMIT_PEAK_CURR_ENALED		1 << 8

#define WAIT_FOR_NORMAL_FIX			0 << 10
#define WAIT_FOR_TIME_FIX			1 << 10

#define UPDATE_RTC_NO_WAKE_UP		0 << 11
#define UPDATE_RTC_WAKE_UP			1 << 11

#define UPDATE_EPH_NO_WAKE_UP		0 << 12
#define UPDATE_EPH_WAKE_UP			1 << 12

#define ENTER_TO_INACTIVE			0 << 16
#define DO_NOT_ENTER_TO_INACTIVE	1 << 16

#define MODE_ON_OFF					0 << 17
#define MODE_CYCLIC					1 << 17


typedef struct
{
	const uint8_t version;
	const uint8_t reserved_1;
	uint8_t maxStartupStateDur;
	const uint8_t reserved_2;
	uint32_t flags;
	uint32_t updatePeriod;
	uint32_t searchPeriod;
	uint32_t gridOffset;
	uint16_t onTime;
	uint16_t minAcqTime;
	const uint8_t reserved_3[20];
	uint32_t extintInactivityMs;
} CFG_Power_Management_s;

#define CFG_POWER_MANAGEMENT_INIT {2, 0, 0, 0, 0, 1000, 10000, 0, 0, 0, {0}, 0}

typedef struct
{
	const uint8_t version;
	uint8_t powerSetupValue;
	uint16_t period;
	uint16_t onTime;
	const uint8_t reserved_1[2];
} CFG_Power_Mode_s;

#define CFG_POWER_MODE_INIT {0, 1, 0, 0, {0}}

typedef enum
{
	Low_Power_Mode_Continous_ver0 = 0,
	Low_Power_Mode_Power_Save = 1,
	Low_Power_Mode_Continous_ver4 = 4,
} Low_Power_Mode_e;

typedef struct
{
	const uint8_t reserved_1;
	uint8_t lpMode;
} CFG_Receiver_Management_s;

#define CFG_RECEIVER_MANAGEMENT_INIT {0, 0}

typedef struct
{
	const uint8_t msgVer;
	const uint8_t numTrackChannelHw;
	const uint8_t numTrackChannelUse;
	const uint8_t numConfigBlocks;
	GNSS_Config_Block_s configBlocks[GNSS_CONFIG_BLOCK_COUNT];
} CFG_GNSS_System_s;

#define CFG_GNSS_SYSTEM_INIT {0, 0, 0xff, GNSS_CONFIG_BLOCK_COUNT, {GNSS_CONFIG_BLOCK_INIT}}

typedef struct
{
	uint32_t clearMask;
	uint32_t saveMask;
	uint32_t loadMask;
//	uint8_t deviceMask;//optional
} CFG_Configuration_s;

#define CFG_CONFIGURATION_INIT {0, 0, 0/*, 0*/}

enum
{
	Configuration_empty = 0x0000,
	Configuration_ioPort = 0x0001,
	Configuration_msgConf = 0x0002,
	Configuration_infMsg = 0x0004,
	Configuration_navConf = 0x0008,
	Configuration_rxmConf = 0x0010,
	Configuration_senConf = 0x0100,
	Configuration_rinvConf = 0x0200,
	Configuration_antConf = 0x0400,
	Configuration_logConf = 0x0800,
	Configuration_ftsConf = 0x1000,
	Configuration_all = 0x1f1f,
} Configuration_Mask_e;


//#define CFG_GNSS_SYSTEM_INIT {0, 0, 0, GNSS_CONFIG_BLOCK_COUNT, {[0 ... (GNSS_CONFIG_BLOCK_COUNT -1)] = GNSS_CONFIG_BLOCK_INIT}}





typedef enum CLASS_ID_ {
	UBX_Class_Id_None = 0x00,
	UBX_NAV = 0x01,
	UBX_RXM = 0x02,
	UBX_INF = 0x03,
	UBX_ACK = 0x05,
	UBX_CFG = 0x06,
	UBX_UPD = 0x09,
	UBX_MON = 0x0a,
	UBX_AID = 0x0b,
	UBX_TIM = 0x0d,
	UBX_ESF = 0x10,
	UBX_MGA = 0x13,
	UBX_LOG = 0x21,
	UBX_SEC = 0x27,
	UBX_HNR = 0x28,
//	NMEA = 0xf0,
//	NMEA_PUBX = 0xf1
} CLASS_ID;


enum NMEA_ID_e
{
	NMEA_GGA = 0x00,
	NMEA_GLL = 0x01,
	NMEA_GSA = 0x02,
	NMEA_GSV = 0x03,
	NMEA_RMC = 0x04,
	NMEA_VTG = 0x05,
	NMEA_GRS = 0x06,
	NMEA_GST = 0x07,
	NMEA_ZDA = 0x08,
	NMEA_GBS = 0x09,
	NMEA_DTM = 0x0a,
	NMEA_GNS = 0x0b,
	NMEA_VLM = 0x0c,
//	NMEA_GPQ_ID = 0x40,
//	NMEA_TXT_ID = 0x41,
//	NMEA_GNQ_ID = 0x42,
//	NMEA_GLQ_ID = 0x43,
//	NMEA_GBQ_ID = 0x44,
};

#define UBX_MSG_NMEA_CLASS_ID 0xf0
#define UBX_MSG_NMEA_MSG_IDS {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0d, 0x0f}

#define NMEA_ALL_MSG_COUNT 13

typedef struct _ACK_s_
{
	CLASS_ID classID;
	uint8_t msgID;
} ACK_s;

typedef struct _UBX_DOP_s
{
	uint32_t iTOW;
	float gDOP;
	float pDOP;
	float tDOP;
	float vDOP;
	float hDOP;
	float nDOP;
	float eDOP;
}UBX_DOP_s;

//, 0x40, 0x41, 0x42, 0x43, 0x44}

enum UBX_ACK_e
{
	UBX_ACK_NACK = 0x00,
	UBX_ACK_ACK = 0x01,
};

enum UBX_NAV_e
{
	UBX_NAV_ORB = 0x34,
	UBX_NAV_DOP = 0x04,
	UBX_NAV_PVT = 0x07,
	UBX_NAV_SAT = 0x35,
};

enum UBX_RXM_e
{
	UBX_RXM_PMREQ = 0x41,
};

typedef enum
{
	Packet_State_Idle,
	Packet_State_Sent,
	Packet_State_Unsent,
	Packet_State_Data_Received,
	Packet_State_Unreadable,
	Packet_State_Failed,
	Packet_State_Accepted,
	Packet_State_Not_Accepted,
//	INVALID_PACKET = 0,
//	UNSOPPORTED_PACKET = 1,
//	Packet_State_Failed = 2,
//	Packet_State_Accepted = 3,
//	Packet_State_Not_Received = 9
} Packet_State_s;


enum
{
	Active_Struct_UART_Protocol,
} CFG_Active_Struct;

uint16_t appendCheckSumToMessage(uint8_t *buffer, uint16_t startPos, uint16_t endPos);
int8_t checkChecksumUBX(uint8_t * buffer, uint16_t buffer_length);
Packet_State_s sendMessageUbx(CLASS_ID classId, uint8_t msgId, uint8_t *payload, uint16_t payload_length);
uint8_t proceedUbxBuffer(uint8_t *buffer, uint16_t buffer_length);
//Packet_State_s proceedSnifferUbxBuffer(uint8_t *buffer, uint16_t buffer_length);
void registerUBXsendCallback(void *callback);
void getReceivedUbxId(CLASS_ID *classId, uint8_t *msgId);
void getMessage(uint8_t *payload);
void pollOrbit();
Packet_State_s pollMessage(uint8_t classId, uint8_t msgId);
uint8_t processAcknowledge(ACK_s *answer, ACK_s *request);
void processProtocolUART(uint8_t * payload);
Packet_State_s pollProtocol(I_O_Port_e port);
Packet_State_s setProtocolUART(UART_BaudRate baudRate, UART_CharLen charLen, UART_StopBits stopBits, UART_Parity parity);
void setInactiveStateWithWakeUpSource(uint32_t durationInMs, WakeUp_Source wakeUpSrc);
void setInactiveState(uint32_t durationInMs); //duration = 0 - infinity time
Packet_State_s setMessageRateForCurrentPort(uint8_t msgClass, uint8_t msgID, uint8_t rate);
Packet_State_s getGnssSystem();
uint8_t parseUbxMessage(uint8_t *buffer, uint8_t length, UBX_Message_s *msg);
Packet_State_s setGnssSystem(uint8_t gpsAndQzss, uint8_t sbas, uint8_t galileo, uint8_t beiDou, uint8_t imes, uint8_t glonass);
Packet_State_s setConfiguration(uint32_t saveMask, uint32_t loadMask, uint32_t clearMask);
Packet_State_s getPowerManagement();
Packet_State_s setPowerManagement();
Packet_State_s getReceiverManagement();
Packet_State_s setReceiverManagement(Low_Power_Mode_e powerMode);
Packet_State_s getPowerMode();
Packet_State_s setPowerMode(Power_Setup_e powerSetup, uint16_t period, uint16_t onTime);
void getReceivedUbxMessage(uint8_t *data);
Packet_State_s setRate(uint16_t measR, uint16_t navR, CFG_Rate_TimeSource timeR);

UBX_DOP_s parseUbxNavDop(UBX_Message_s* msg);
uint8_t parseUbxNavPvtFix(UBX_Message_s* msg);
UBX_Date_s parseUbxNavPvtDate(UBX_Message_s* msg);
UBX_Time_s parseUbxNavPvtTime(UBX_Message_s* msg);
uint8_t parseUbxNavPvtTimeValid(UBX_Message_s* msg);
uint8_t parseUbxNavPvtGpsValid(UBX_Message_s* msg);
uint8_t parseUbxNavPvtSatUsed(UBX_Message_s* msg);

#endif /* UBX_H_ */
