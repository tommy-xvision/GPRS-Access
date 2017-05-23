/*
 *  Copyright (c) 2017, xVision, Inc.
 *  All rights reserved.
 *
 *
 */

/* Author: Tommy.Cao */

#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"

#include "main.h"
#include "uart_if.h"
#include "data_if.h"
#include "quectel_if.h"


//#define QUECTEL_IF_DEBUG
#ifdef QUECTEL_IF_DEBUG
#define QUECTEL_DEBUG(...) printf(__VA_ARGS__)
#else
#define QUECTEL_DEBUG(...)
#endif

#define NULL_CMD		0
#define AT_TEST_CMD		1
#define GET_SIM_CMD		2
#define GET_SIGNAL_CMD	3
#define GET_GSM_REG_CMD	4
#define GET_GPRS_REG_CMD	5
#define GET_CARRIER_CMD		6
#define SET_IP_MODE_CMD		7
#define SET_APN_CON_CMD		8
#define SET_IP_TASK_CMD		9
#define GET_CON_MODE_CMD	10
#define SET_ACT_GPRS_CMD		11
#define GET_LOCAL_IP_CMD	12
#define SET_RX_HEADER_CMD		13
#define SET_DNS_MODE_CMD	14
#define SET_IP_CON_PEER_CMD		15
#define GET_GPRS_ATTACH_CMD		16
#define SET_GPRS_ATTACH_CMD		17
#define SET_CON_MODE_CMD	18
#define SET_IP_CLOSE_PEER_CMD		19
#define SET_DEACT_GPRS_CMD		20
#define GET_IP_CON_STATE		21
#define READY_SEND_CMD		22
#define SEND_FRAME_CMD		23
#define GET_SEND_CMD		24
#define SET_RX_IN_CMD		25
#define GET_RX_BUFFER_CMD		26
#define GET_IP_MODE_CMD		27
#define GET_IMEI_CMD		28

QuectelIfConfig QuectelIfRec;

extern DataIfConfig	DataIfRec;
extern SystemParamConfig SystemParamRec;

#define MaxArgs 3
uint8_t DataArgc;                //< count of args parsed so far
char* DataArgv[MaxArgs];    //< pointers to arguments, in input[]
bool DataArgStrType[MaxArgs];
CmdFuncItem* CmdFuncList;

typedef void (*CmdFunc)(void);
typedef void (*CmdFunc)(void);

#define ATGETWAITTIME(cmd_idx) ((CmdFuncItem*)(CmdFuncList+cmd_idx))->WaitTime
#define ATWRITEARG(count, ptr, strptr, strType)	do{ \
														count = snprintf(ptr, DATA_TX_BUFFER_SIZE-DataTxBufferLen, (strType)?"\"%s\"":"%s", strptr);	\
														DataTxBufferLen += count;	\
														ptr += count;}while(0)
#define RIGHT_AT_RESULT		"OK"


/******************Data Task*******************/
/* Buffer for parse receive UART bytes from GSM module */
#define DATA_TASK_PARSE_SIZE	64
char DataTaskParseBuffer[DATA_TASK_PARSE_SIZE];
uint8_t DataTaskParseBufferLen;
uint16_t DataTaskParseCount;

extern char DataTxBuffer[DATA_TX_BUFFER_SIZE];
extern char DataRxBuffer[DATA_RX_BUFFER_SIZE];
extern uint8_t DataTxBufferLen;
extern UartBufferNode DataRxNode;

extern char DataTaskMsgBuffer[DATA_RX_BUFFER_SIZE];
extern uint16_t DataTaskMsgBufferLen;
extern char* DataTaskMsgSendBufferPtr;
extern uint16_t DataTaskMsgSendBufferLen;

/*************** Feature Command **************/
static void Quectel_Print(char* PreFix, char* StrPtr, uint8_t Len)
{
	uint8_t i;
	QUECTEL_DEBUG("%s[%d]: [ ", PreFix, Len);
	for(i = 0; i < Len; i ++)
	{
		QUECTEL_DEBUG("%c", StrPtr[i]);
	}

	QUECTEL_DEBUG(" ]\r\n");
}

static void Quectel_Copy_Rx_Buffer(bool FirstTime)
{
	if(FirstTime)
	{
		/* Initialize buffer if first time */
		DataTaskMsgBufferLen = 0;
		DataTaskParseCount = 0;
	}
	
	if(DataRxNode.front == DataRxNode.rear)
	{
		/* Nothing received */
		return;
	}

	if(DataRxNode.rear > DataRxNode.front)
	{
		memcpy(DataTaskMsgBuffer+DataTaskMsgBufferLen, DataRxNode.front, DataRxNode.rear - DataRxNode.front);
		DataTaskMsgBufferLen += (DataRxNode.rear - DataRxNode.front);
	}
	else if(DataRxNode.rear < DataRxNode.front)
	{
		memcpy(DataTaskMsgBuffer+DataTaskMsgBufferLen, DataRxNode.front, DataRxBuffer + DATA_RX_BUFFER_SIZE - DataRxNode.front);
		DataTaskMsgBufferLen += (DataRxBuffer + DATA_RX_BUFFER_SIZE - DataRxNode.front);
		
		memcpy(DataTaskMsgBuffer+DataTaskMsgBufferLen, DataRxBuffer, DataRxNode.rear-DataRxBuffer);
		DataTaskMsgBufferLen += (DataRxNode.rear - DataRxBuffer);
	}
	DataRxNode.front = DataRxNode.rear;
}

static bool Quectel_Parse_Reply(bool NeedPreFix)
{
	uint16_t i;
	bool CmdPreFixFound = false;

	if(!NeedPreFix)
	{
		CmdPreFixFound = true;
	}

	DataTaskParseBufferLen = 0;
	for(i = DataTaskParseCount; i < DataTaskMsgBufferLen; i ++)
	{
		if((CmdPreFixFound) && (DataTaskMsgBuffer[i] != CHAR_CR))
		{
			/* Copy data to buffer */
			DataTaskParseBuffer[DataTaskParseBufferLen++] = DataTaskMsgBuffer[i];
		}

		if((DataTaskMsgBuffer[i] == CHAR_CR) && (DataTaskMsgBuffer[i+1] == CHAR_LF))
		{
			if(CmdPreFixFound)
			{
				/* found command suffix */
				DataTaskParseBuffer[DataTaskParseBufferLen++] = '\0';
				DataTaskParseCount = i+2;

				QUECTEL_DEBUG("Get: %s\r\n", DataTaskParseBuffer);
				return true;
			}
			else
			{
				/* found a AT command prefix */
				CmdPreFixFound = true;
			}
			i ++;
		}
	}
	
	return false;
}

/************ AT Command handler defintion **********/
static void Quectel_Send_AT(uint8_t CmdIdx)
{
  /* combine all the command string */
	char WCount;
	char* WBuffPtr;
	CmdFuncItem* CmdPtr = (CmdFuncItem*)(CmdFuncList+CmdIdx);

	DataTxBufferLen = 0;
	WBuffPtr = DataTxBuffer;
	
	/* write AT */
	ATWRITEARG(WCount, WBuffPtr, "AT", false);

  /* write command line string */
	ATWRITEARG(WCount, WBuffPtr, CmdPtr->CmdLine, false);

	if (CmdPtr->AnyArgu)
	{
	  uint8_t i;
		
		/* add equal character if needed*/
		ATWRITEARG(WCount, WBuffPtr, "=", false);

		/* add all the arguments */
		for (i = 0; i < DataArgc; i ++)
		{
			if(i != 0)
			{
				/* needs character "," except the first one */
				ATWRITEARG(WCount, WBuffPtr, ",", false);
			}

			/* add argument */
			ATWRITEARG(WCount, WBuffPtr, DataArgv[i], DataArgStrType[i]);
		}
	}

	/* Add CR LF for command completion */
	ATWRITEARG(WCount, WBuffPtr, "\r", false);
	ATWRITEARG(WCount, WBuffPtr, "\n", false);
	*(WBuffPtr) = '\0';
	
	Data_UART_Send(DataTxBuffer, DataTxBufferLen);
	QuectelIfRec.TimerCount = 0;
}


void GsmAtTest(void)
{
	DataArgc = 0;
	Quectel_Send_AT(AT_TEST_CMD);
}

void GsmGetSim(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_SIM_CMD);
}

void GsmGetSignal(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_SIGNAL_CMD);
}

void GsmGetGsmReg(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_GSM_REG_CMD);
}

void GsmGetGprsReg(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_GPRS_REG_CMD);
}

void GsmGetCarrier(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_CARRIER_CMD);
}

void GsmSetIpMode(void)
{
	DataArgc = 0;
	DataArgv[DataArgc] = "0";
	DataArgStrType[DataArgc++] = false;
	Quectel_Send_AT(SET_IP_MODE_CMD);
}

void GsmSetApnCon(void)
{
	DataArgc = 0;
	DataArgv[DataArgc] = "1";
	DataArgStrType[DataArgc++] = false;
	DataArgv[DataArgc] = "CMNET";
	DataArgStrType[DataArgc++] = true;
	Quectel_Send_AT(SET_APN_CON_CMD);
}

void GsmSetIpTask(void)
{
	DataArgc = 0;
	Quectel_Send_AT(SET_IP_TASK_CMD);
}

void GsmGetConMode(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_CON_MODE_CMD);
}

void GsmSetActGprs(void)
{
	DataArgc = 0;
	Quectel_Send_AT(SET_ACT_GPRS_CMD);
}

void GsmGetLocalIp(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_LOCAL_IP_CMD);
}

void GsmSetRxHeader(void)
{
	DataArgc = 0;
	DataArgv[DataArgc] = "1";
	DataArgStrType[DataArgc++] = false;
	Quectel_Send_AT(SET_RX_HEADER_CMD);
}

void GsmSetDnsMode(void)
{
	DataArgc = 0;
	DataArgv[DataArgc] = "1";
	DataArgStrType[DataArgc++] = false;
	Quectel_Send_AT(SET_DNS_MODE_CMD);
}

void GsmSetIpConPeer(void)
{
	char DstPort[8];
	memset(DstPort, 0, sizeof(DstPort));
	
	DataArgc = 0;
	DataArgv[DataArgc] = "tcp";
	DataArgStrType[DataArgc++] = true;
	DataArgv[DataArgc] = (char*)SystemParamRec.ServerUrl;
	DataArgStrType[DataArgc++] = true;
	sprintf(DstPort, "%d", SystemParamRec.ServerDstPort);
	DataArgv[DataArgc] = DstPort;
	DataArgStrType[DataArgc++] = true;
	Quectel_Send_AT(SET_IP_CON_PEER_CMD);
}

void GsmGetGprsAttach(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_GPRS_ATTACH_CMD);
}

void GsmSetGprsAttach(void)
{
	DataArgc = 0;
	DataArgv[DataArgc] = "1";
	DataArgStrType[DataArgc++] = false;
	Quectel_Send_AT(SET_GPRS_ATTACH_CMD);
}

void GsmSetConMode(void)
{
	DataArgc = 0;
	DataArgv[DataArgc] = "1";
	DataArgStrType[DataArgc++] = false;
	Quectel_Send_AT(SET_CON_MODE_CMD);
}

void GsmSetIpClosePeer(void)
{
	DataArgc = 0;
	Quectel_Send_AT(SET_IP_CLOSE_PEER_CMD);
}

void GsmSetDeActGprs(void)
{
	DataArgc = 0;
	Quectel_Send_AT(SET_DEACT_GPRS_CMD);
}

void GsmGetIpConState(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_IP_CON_STATE);
}

void GsmReadySend(void)
{
	char LenStr[6];
	memset(LenStr, 0, sizeof(LenStr));
	
	DataArgc = 0;
	sprintf(LenStr, "%d", DataTaskMsgSendBufferLen);
	DataArgv[DataArgc] = LenStr;
	DataArgStrType[DataArgc++] = false;
	Quectel_Send_AT(READY_SEND_CMD);
}

void GsmSendFrame(void)
{
	Data_UART_Send(DataTaskMsgSendBufferPtr, DataTaskMsgSendBufferLen);
}

void GsmGetSend(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_SEND_CMD);
}

void GsmSetRxInMode(void)
{
	DataArgc = 0;
	DataArgv[DataArgc] = "1";
	DataArgStrType[DataArgc++] = false;
	Quectel_Send_AT(SET_RX_IN_CMD);
}

void GsmGetRxBuffer(void)
{
	DataArgc = 0;
	DataArgv[DataArgc] = "0,1,0";
	DataArgStrType[DataArgc++] = false;
	DataArgv[DataArgc] = "1024";
	DataArgStrType[DataArgc++] = false;
	Quectel_Send_AT(GET_RX_BUFFER_CMD);
}

void GsmGetIpMode(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_IP_MODE_CMD);
}

void GsmGetImei(void)
{
	DataArgc = 0;
	Quectel_Send_AT(GET_IMEI_CMD);
}

/************ AT Command handler defintion for reply **********/

extern char DataTaskParseBuffer[DATA_TASK_PARSE_SIZE];
extern uint8_t DataTaskParseBufferLen;

extern char DataTxBuffer[DATA_TX_BUFFER_SIZE];
extern uint8_t DataTxBufferLen;

#define SIZEOF_CR_LF 2

/******************** Functions **********************/
static bool GsmSetReply(void)
{
	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return false;
	}
	
	/* check the result */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, 2)))
	{
		/* no valid command string or doesn't match with send command */
		return false;
	}

	return true;
}

#define AT_TEST_RESET_TRY	10
static void GsmAtReply(void)
{
	if(GsmSetReply())
	{
		/* Link GSM module correctly */
#ifdef DATA_MODULE_RESET
		if(QuectelIfRec.State == QuectelIfStateReceived)
		{
			/* Only modify state after reset done */
			QuectelIfRec.State = QuectelIfStateLinked;
			DataIfRec.ToReset = false;
			QUECTEL_DEBUG("Quectel module linked\r\n");
			/* Only to check IMEI after startup */
			QuectelIfRec.CmdIdxNext = GET_IMEI_CMD;
		}
#else
		if((DataIfRec.State == DataIfStateHeartBeat) &&
			(DataIfRec.ToSend == false))
			{
				/* AT testing in Heartbeat state */
				return;
			}
			
		if(QuectelIfRec.State <= QuectelIfStateReceived)
		{
			/* Only modify state after reset done */
			QUECTEL_DEBUG("Quectel module linked\r\n");
			/* Only to check IMEI after startup */
			QuectelIfRec.CmdIdxNext = GET_IMEI_CMD;
		}
#endif

	}
	else 
	{
		QUECTEL_DEBUG("Can't link GSM module with try: %d\r\n", QuectelIfRec.AtCount);
		if(QuectelIfRec.AtCount >= AT_TEST_RESET_TRY)
		{
			/* need to reset Quectel module */
			QuectelIfRec.ToReset = true;
			DataIfRec.ToReset = true;
			DataIfRec.State = DataIfStateStartup;
		}
		else
		{
			QuectelIfRec.AtCount ++;
			QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
		}
	}
}

static void GsmGetSimReply(void)
{
#define SIM_PREFIX_OFFSET	7
#define SIM_READY_REPLY	"READY"
	bool SimReady = false;

	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  +CPIN: READY*/
	if (Quectel_Parse_Reply(false))
			
	{
		if(!memcmp(DataTaskParseBuffer+SIM_PREFIX_OFFSET, SIM_READY_REPLY, strlen(SIM_READY_REPLY)))
		{
			/* SIM is ready, move to next step */
			QuectelIfRec.CmdIdxNext = GET_SIGNAL_CMD;
			SimReady = true;
		}
		else
		{
			/* Error happen, need to check SIM or reboot GSM module */
			QUECTEL_DEBUG("SIM error then to restart GSM module\r\n");
		}
	}

	/* Read the result: OK */
	if(!Quectel_Parse_Reply(true) ||
		(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, strlen(RIGHT_AT_RESULT))) ||
		(SimReady == false))
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmGetSignalReply(void)
{
#define SIGNAL_PREFIX_OFFSET	6
	uint8_t SignalDb;

	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should like  +CSQ: 30,0*/
	if (Quectel_Parse_Reply(false))
	{
		uint8_t i, j;
		char SigTemp[3];
		for(i = SIGNAL_PREFIX_OFFSET, j = 0; i < DataTaskParseBufferLen; i ++, j ++)
		{
			if (DataTaskParseBuffer[i] == ',')
			{
				SigTemp[j] = '\0';
				break;
			}
			else
			{
				SigTemp[j] = DataTaskParseBuffer[i];
			}
		}
		SignalDb = (uint8_t)strtoul(SigTemp, NULL, 10);

		QUECTEL_DEBUG("SignalDb: %d\r\n", SignalDb);
		QuectelIfRec.CmdIdxNext = GET_GSM_REG_CMD;
	}

	/* Read the result: OK */
	if(!Quectel_Parse_Reply(true) ||
		(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, strlen(RIGHT_AT_RESULT))) ||
		(SignalDb <= 15))
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}

}

static void GsmGetGsmRegReply(void)
{
#define GSM_PREFIX_OFFSET	7
	uint8_t GsmRegState = 0;

	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  +CREG: 0,1 */
	if (Quectel_Parse_Reply(false))
	{
		uint8_t	i, j;
		char GsmTemp[3];
		bool FoundComma = false;
		for(i = GSM_PREFIX_OFFSET, j = 0; i < DataTaskParseBufferLen; i ++)
		{
			if (DataTaskParseBuffer[i] == ',')
			{
				FoundComma = true;
			}
			
			if(FoundComma)
			{
				GsmTemp[j++] = DataTaskParseBuffer[++i];
				FoundComma = false;
			}
		}
		GsmTemp[j] = '\0';
		GsmRegState = (uint8_t)strtoul(GsmTemp, NULL, 10);
		QUECTEL_DEBUG("GsmRegState: %d\r\n", GsmRegState);
		if(GsmRegState > 0)
		{
			QuectelIfRec.State = QuectelIfStateRegistered;
			QuectelIfRec.CmdIdxNext = GET_GPRS_REG_CMD;
		}
	}

	/* Read the result: OK */
	if(!Quectel_Parse_Reply(true) ||
		(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, strlen(RIGHT_AT_RESULT))) ||
		(GsmRegState == 0))
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmGetGprsRegReply(void)
{
#define GPRS_PREFIX_OFFSET	8
	uint8_t GprsRegState = 0;

	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  +CGREG: 0,1 */
	if (Quectel_Parse_Reply(false))
	{
		uint8_t	i, j;
		char GprsTemp[3];
		bool FoundComma = false;
		for(i = GPRS_PREFIX_OFFSET, j = 0; i < DataTaskParseBufferLen; i ++)
		{
			if (DataTaskParseBuffer[i] == ',')
			{
				FoundComma = true;
			}
			
			if(FoundComma)
			{
				GprsTemp[j++] = DataTaskParseBuffer[++i];
				FoundComma = false;
			}
		}
		GprsTemp[j] = '\0';
		GprsRegState = (uint8_t)strtoul(GprsTemp, NULL, 10);
		QUECTEL_DEBUG("GprsRegState: %d\r\n", GprsRegState);
		if(GprsRegState > 0)
		{
			QuectelIfRec.State = QuectelIfStateAttached;
			QuectelIfRec.CmdIdxNext = GET_CARRIER_CMD;
		}
	}

	/* Read the result: OK */
	if(!Quectel_Parse_Reply(true) ||
		(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, strlen(RIGHT_AT_RESULT))) ||
		(GprsRegState == 0))
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmGetCarrierReply(void)
{
#define CARRIER_PREFIX_OFFSET	18

	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  GsmGetConMode */
	if (Quectel_Parse_Reply(false))
	{
		if(!strncmp(DataTaskParseBuffer, "UNICOM", 6))
		{
		}
		else if(!strncmp(DataTaskParseBuffer, "MOBILE", 6))
		{
		}
		QuectelIfRec.CmdIdxNext = GET_IP_MODE_CMD;
	}

	/* Read the result: OK */
	if(!Quectel_Parse_Reply(true) ||
		(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, strlen(RIGHT_AT_RESULT))))
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetIpModeReply(void)
{
	if(GsmSetReply())
	{
		QuectelIfRec.CmdIdxNext = SET_APN_CON_CMD;
	}
	else
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetApnConReply(void)
{
	if(GsmSetReply())
	{
		QuectelIfRec.CmdIdxNext = SET_IP_TASK_CMD;
	}
	else
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetIpTaskReply(void)
{
	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	Quectel_Parse_Reply(false);
	if(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, 2))
	{
		QUECTEL_DEBUG("Ip Task Initialized\r\n");
	}
	QuectelIfRec.CmdIdxNext = GET_CON_MODE_CMD;
}

static void GsmGetConModeReply(void)
{
#define CON_PREFIX_OFFSET	9
	uint8_t ConMode = 0;

	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  +QICSGP: 1 */
	if (Quectel_Parse_Reply(false))
	{
		uint8_t	i, j;
		char ConTemp[3];
		for(i = CON_PREFIX_OFFSET, j = 0; i < DataTaskParseBufferLen; i ++)
		{
			ConTemp[j++] = DataTaskParseBuffer[i];
		}
		ConTemp[j] = '\0';
		ConMode = (uint8_t)strtoul(ConTemp, NULL, 10);
		QUECTEL_DEBUG("ConMode: %d\r\n", ConMode);
		if(ConMode)
		{
			QuectelIfRec.CmdIdxNext = SET_ACT_GPRS_CMD;
		}
	}

	/* Read the result: OK */
	if(!Quectel_Parse_Reply(true) ||
		(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, strlen(RIGHT_AT_RESULT))) ||
		(ConMode == 0))
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetActGprsReply(void)
{
	if(GsmSetReply())
	{
		QuectelIfRec.CmdIdxNext = GET_LOCAL_IP_CMD;
	}
	else 
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmGetLocalIpReply(void)
{
	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  xx.xx.xx.xx */
	if (Quectel_Parse_Reply(false))
	{
		QUECTEL_DEBUG("LocalIp: %s\r\n", DataTaskParseBuffer);
		if(QuectelIfRec.State == QuectelIfStateAttached)
		{
			QuectelIfRec.CmdIdxNext = SET_RX_HEADER_CMD;
		}
		QuectelIfRec.State = QuectelIfStateActivated;
	}
	else
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetRxHeaderReply(void)
{
	if(GsmSetReply())
	{
		QuectelIfRec.CmdIdxNext = SET_DNS_MODE_CMD;
	}
	else
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetDnsModeReply(void)
{
	if(GsmSetReply())
	{
		QuectelIfRec.CmdIdxNext = SET_RX_IN_CMD;
	}
	else
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetIpConPeerReply(void)
{
#define IPCON_OK_RESULT	"CONNECT OK"
	if(GsmSetReply())
	{
		/* check the result */
		/* result should be  CONNECT OK */
		if (Quectel_Parse_Reply(true))
		{
			if(!memcmp(DataTaskParseBuffer, IPCON_OK_RESULT, strlen(IPCON_OK_RESULT)))
			{
				QUECTEL_DEBUG("Ip connection ok\r\n");	
				if(DataIfRec.ToSend)
				{
					QuectelIfRec.CmdIdxNext = READY_SEND_CMD;
				}
				DataIfRec.Connected = true;
				QuectelIfRec.State = QuectelIfStateIpConnected;
				return;
			}
		}
	}

	QuectelIfRec.CmdIdxNext = GET_IP_CON_STATE;
}

static void GsmGetGprsAttachReply(void)
{
#define CGATT_PREFIX_OFFSET	8
	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  +CGATT: 1 */
	if (Quectel_Parse_Reply(false))
	{
		uint8_t i, j;
		char CgattTemp[3];
		bool CgattMode = false;
		for(i = CGATT_PREFIX_OFFSET, j = 0; i < DataTaskParseBufferLen; i ++)
		{
			CgattTemp[j++] = DataTaskParseBuffer[i];
		}
		CgattTemp[j] = '\0';
		CgattMode = (uint8_t)strtoul(CgattTemp, NULL, 10);
		QUECTEL_DEBUG("CgattMode: %d\r\n", CgattMode);
		if(CgattMode)
		{
			QuectelIfRec.CmdIdxNext = GET_GPRS_REG_CMD;
		}
	}
	else
	{
		/* Re-attach GPRS */
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetGprsAttachReply(void)
{
	if(GsmSetReply())
	{
		QuectelIfRec.CmdIdxNext = GET_GPRS_REG_CMD;
	}
	else 
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetConModeReply(void)
{
	if(GsmSetReply())
	{
		QuectelIfRec.CmdIdxNext = SET_ACT_GPRS_CMD;
	}
	else 
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetIpClosePeerReply(void)
{
#define IPCLOSE_OK_RESULT	"CLOSE OK"
	if ((Quectel_Parse_Reply(false) == true) || 
			(!memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* check the result */
		/* result should be  CLOSE OK */
		if (Quectel_Parse_Reply(false))
		{
			if(!memcmp(DataTaskParseBuffer, IPCLOSE_OK_RESULT, strlen(IPCLOSE_OK_RESULT)))
			{
				QUECTEL_DEBUG("Ip connection close ok\r\n");	
#ifndef DATA_MODULE_RESET
				if((DataIfRec.ToReset) && (QuectelIfRec.State <= QuectelIfStateReceived))
				{
					/* To make sure PDP de-activated */
					QuectelIfRec.CmdIdxNext = SET_DEACT_GPRS_CMD;
					return;
				}
#endif
				if((QuectelIfRec.ToReset) || (DataIfRec.ToReset))
				{
					QuectelIfRec.CmdIdxNext = SET_DEACT_GPRS_CMD;
					QuectelIfRec.State = QuectelIfStateIpDisConnected;
					return;
				}
				
				if((DataIfRec.ToSend) || (DataIfRec.State >= DataIfStateConnecting))
				{
					QuectelIfRec.CmdIdxNext = SET_IP_CON_PEER_CMD;
					QuectelIfRec.State = QuectelIfStateIpDisConnected;
					return;
				}
			}
		}
	}
	
	QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
}

static void GsmSetDeActGprsReply(void)
{
#define DEACT_OK_RESULT	"DEACT OK"
	if ((Quectel_Parse_Reply(false) == true) || 
			(!memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* check the result */
		/* result should be  CLOSE OK */
		if (Quectel_Parse_Reply(false))
		{
			if(!memcmp(DataTaskParseBuffer, DEACT_OK_RESULT, strlen(DEACT_OK_RESULT)))
			{
				QUECTEL_DEBUG("PDP close ok\r\n");		
#ifndef DATA_MODULE_RESET
			if((DataIfRec.ToReset) && (QuectelIfRec.State <= QuectelIfStateReceived))
			{
				/* To make sure PDP de-activated */
				DataIfRec.ToReset = false;
				QuectelIfRec.State = QuectelIfStateLinked;
				return;
			}
#endif
				if(QuectelIfRec.ToReset)
				{
					QuectelIfRec.State = QuectelIfStateDeActivated;
					return;
				}
			}
		}
	}
	
	QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
}

static void GsmGetIpConStateReply(void)
{
#define IPCON_PREFIX_OFFSET	7
#define IPCON_INITIAL_REPLY	"IP INITIAL"
#define IPCON_CLOSE_REPLY	"IP CLOSE"
#define IPCON_CONNECTING_REPLY	"TCP CONNECTING"
#define IPCON_CONNECTED_REPLY	"CONNECT OK"
	/* check the command included in reply */
	
	if(!GsmSetReply())
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
	
	/* check the result */
	/* result should be  
	STATE: IP INITIAL
	STATE: IP CLOSE
	STATE: TCP CONNECTING
	STATE: CONNECT OK*/
	if (Quectel_Parse_Reply(true))
	{
		if(!memcmp(DataTaskParseBuffer+IPCON_PREFIX_OFFSET, IPCON_CONNECTED_REPLY, strlen(IPCON_CONNECTED_REPLY)))
		{
#ifndef DATA_MODULE_RESET
			if((DataIfRec.ToReset) && (QuectelIfRec.State <= QuectelIfStateReceived))
			{
				/* To close Ip Connection */
				QuectelIfRec.CmdIdxNext = SET_IP_CLOSE_PEER_CMD;
				return;
			}
#endif
			/* connected! */
			QuectelIfRec.State = QuectelIfStateIpConnected;
			if(DataIfRec.ToSend)
			{
				QuectelIfRec.CmdIdxNext = READY_SEND_CMD;
			}
			QUECTEL_DEBUG("Connected\r\n");
		}
		else if(!memcmp(DataTaskParseBuffer+IPCON_PREFIX_OFFSET, IPCON_INITIAL_REPLY, strlen(IPCON_INITIAL_REPLY)) || 
			!memcmp(DataTaskParseBuffer+IPCON_PREFIX_OFFSET, IPCON_CLOSE_REPLY, strlen(IPCON_CLOSE_REPLY)))
		{
#ifndef DATA_MODULE_RESET
			if((DataIfRec.ToReset) && (QuectelIfRec.State <= QuectelIfStateReceived))
			{
				/* To make sure PDP de-activated */
				QuectelIfRec.CmdIdxNext = SET_DEACT_GPRS_CMD;
				return;
			}
#endif
			/* need to re-connect remote server */
			QuectelIfRec.CmdIdxNext = SET_IP_CON_PEER_CMD;
			QuectelIfRec.State = QuectelIfStateIpDisConnected;
			QUECTEL_DEBUG("DisConnected\r\n");
		}
		else if(!memcmp(DataTaskParseBuffer+IPCON_PREFIX_OFFSET, IPCON_CONNECTING_REPLY, strlen(IPCON_CONNECTING_REPLY)))
		{
#ifndef DATA_MODULE_RESET
			if((DataIfRec.ToReset) && (QuectelIfRec.State <= QuectelIfStateReceived))
			{
				/* To close Ip Connection */
				QuectelIfRec.CmdIdxNext = SET_IP_CLOSE_PEER_CMD;
				return;
			}
#endif
			/* close previous connection and re-connect */
			QuectelIfRec.CmdIdxNext = SET_IP_CLOSE_PEER_CMD;
			QuectelIfRec.State = QuectelIfStateIpDisConnected;
			QUECTEL_DEBUG("To DisConnect\r\n");
		}
		else
		{
			/* something like STATE: IP STATUS */
			QuectelIfRec.CmdIdxNext = SET_IP_CON_PEER_CMD;
		}
	}
}

static void GsmReadySendReply(void)
{
	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	Quectel_Parse_Reply(false);
	if(DataTaskParseBuffer[0] == '>')
	{
		/* fill send buffer */
		QuectelIfRec.State = QuectelIfStateSendReady;
		QuectelIfRec.CmdIdxNext = SEND_FRAME_CMD;
		QUECTEL_DEBUG("Sending-------->\r\n");
	}
	else
	{
		/* to check TCP connection status */
		QuectelIfRec.CmdIdxNext = GET_IP_CON_STATE;
	}
}

static void GsmSendFrameReply(void)
{
#define SEND_DONE_REPLY	"SEND OK"
	
	/* check the result */
	/* result should be  SEND OK */
	if (Quectel_Parse_Reply(true))
	{
		if(!memcmp(DataTaskParseBuffer, SEND_DONE_REPLY, strlen(SEND_DONE_REPLY)))
		{
			/* connected! */
			QuectelIfRec.State = QuectelIfStateSendOut;
			QuectelIfRec.CmdIdxNext = GET_SEND_CMD;
		}
	}
}

static void GsmGetSendReply(void)
{
#define SEND_PREFIX_OFFSET	9

	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  +QISACK: 9, 0, 9 or +QISACK: 9, 9, 0*/
	if (Quectel_Parse_Reply(false))
	{
		uint8_t	i, j;
		char NumBuf[8];
		uint32_t SentOutCount, SureSent;
		for(i = SEND_PREFIX_OFFSET, j = 0; i < DataTaskParseBufferLen; i ++)
		{
			if (DataTaskParseBuffer[i] == ',')
			{
				NumBuf[j++] = '\0';
				break;
			}
			else
			{
				NumBuf[j++] = DataTaskParseBuffer[i];
			}
		}
		NumBuf[j] = '\0';
		SentOutCount = (uint8_t)strtoul(NumBuf, NULL, 10);
		i ++;
		
		for(j = 0; i < DataTaskParseBufferLen; i ++)
		{
			if (DataTaskParseBuffer[i] == ' ')
			{
				continue;
			}
			else if (DataTaskParseBuffer[i] == ',')
			{
				break;
			}
			else
			{
				NumBuf[j++] = DataTaskParseBuffer[i];
			}
		}
		NumBuf[j] = '\0';
		SureSent = (uint8_t)strtoul(NumBuf, NULL, 10);
		QUECTEL_DEBUG("Sent %d:%d\r\n", SentOutCount, SureSent);
		
		if(SentOutCount > SureSent)
		{
			/* Continue check */
			QuectelIfRec.CmdIdxNext = GET_SEND_CMD;
		}
		else 
		{
			/* make sure sent out */
			QuectelIfRec.State = QuectelIfStateSendDone;
		}
	}
	
	/* Read the result: OK */
	if((!Quectel_Parse_Reply(true)) ||
		(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, strlen(RIGHT_AT_RESULT))))
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmSetRxInModeReply(void)
{
	if(GsmSetReply())
	{
		QuectelIfRec.CmdIdxNext = SET_IP_CON_PEER_CMD;
	}
	else
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

static void GsmGetRxBufferReply(void)
{
	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* Copy UART buffer to message buffer */
	QuectelIfRec.IpReceived = true;
}

static void GsmGetIpModeReply(void)
{
#define IPMODE_PREFIX_OFFSET	9
	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  +QIMODE: 0 */
	if(Quectel_Parse_Reply(false))
	{
#ifdef QUECTEL_IF_DEBUG
		uint8_t i, j;
		char CgattTemp[3];
		for(i = IPMODE_PREFIX_OFFSET, j = 0; i < DataTaskParseBufferLen; i ++)
		{
			CgattTemp[j++] = DataTaskParseBuffer[i];
		}
		CgattTemp[j] = '\0';
		QUECTEL_DEBUG("IpMode: %d\r\n", CgattTemp);
#endif
		QuectelIfRec.CmdIdxNext = SET_APN_CON_CMD;
	}

	/* Read the result: OK */
	if(!Quectel_Parse_Reply(true) ||
		(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, strlen(RIGHT_AT_RESULT))))
	{
		QuectelIfRec.CmdIdxNext = SET_IP_MODE_CMD;
	}
}

static void GsmGetImeiReply(void)
{
	/* check the command included in reply */
	if ((Quectel_Parse_Reply(false) == false) || 
			(memcmp(DataTxBuffer, DataTaskParseBuffer, DataTxBufferLen - SIZEOF_CR_LF)))
	{
		/* no valid command string or doesn't match with send command */
		return;
	}
	
	/* check the result */
	/* result should be  861358030152342 */
	if(Quectel_Parse_Reply(false))
	{
		Data_Decide_Sn(DataTaskParseBuffer);
#ifndef DATA_MODULE_RESET
		/* Need to Disable Ip connection and PDP if needed */
		QuectelIfRec.CmdIdxNext = GET_IP_CON_STATE;
#endif
	}

	/* Read the result: OK */
	if(!Quectel_Parse_Reply(true) ||
		(memcmp(DataTaskParseBuffer, RIGHT_AT_RESULT, strlen(RIGHT_AT_RESULT))))
	{
		QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
	}
}

/**************************************************
Command Line
Wait Time after send Command
Wait how many times
Add arguments?
End fix likes "OK"?
Dedicated fix likes "CONNECT" "DEACT"
Will reset Quectel module if max try failed
Send Command function
Reply handler
***************************************************/
CmdFuncItem QuectelGsmCmdList[] = 
{
	{"", 2, 2,false, NULL, NULL, NULL},	/* 0 */
	{"", 1, 2,false, true, NULL, GsmAtTest, GsmAtReply},	/* 1 */
	{"+CPIN?", 1, 2, false, true, NULL, GsmGetSim, GsmGetSimReply},/* 2 */
	{"+CSQ", 1, 2, false, true, NULL, GsmGetSignal, GsmGetSignalReply},/* 3 */
	{"+CREG?", 1, 2, false, true, NULL, GsmGetGsmReg, GsmGetGsmRegReply},/* 4 */
	{"+CGREG?", 1, 2, false, true, NULL, GsmGetGprsReg, GsmGetGprsRegReply},/* 5 */
	{"+COPS?", 1, 2, false, true, NULL, GsmGetCarrier, GsmGetCarrierReply},/* 6 */
	{"+QIMODE", 1, 2, true, true, NULL, GsmSetIpMode, GsmSetIpModeReply},/* 7 */
	{"+QICSGP", 1, 2, true, true, NULL, GsmSetApnCon, GsmSetApnConReply},/* 8 */
	{"+QIREGAPP", 1, 2, false, true, NULL, GsmSetIpTask, GsmSetIpTaskReply},/* 9 */
	{"+QICSGP?", 1, 2, false, true, NULL, GsmGetConMode, GsmGetConModeReply},/* 10 */
	{"+QIACT", 2, 5, false, true, NULL, GsmSetActGprs, GsmSetActGprsReply},/* 11 */
	{"+QILOCIP", 1, 2, false, false, NULL, GsmGetLocalIp, GsmGetLocalIpReply},/* 12 */
	{"+QIHEAD", 1, 2, true, true, NULL, GsmSetRxHeader, GsmSetRxHeaderReply},/* 13 */
	{"+QIDNSIP", 1, 2, true, true, NULL, GsmSetDnsMode, GsmSetDnsModeReply},/* 14 */
	{"+QIOPEN", 2, 5, true, true, "CONNECT", GsmSetIpConPeer, GsmSetIpConPeerReply},/* 15 */
	{"+CGATT?", 1, 2, false, true, NULL, GsmGetGprsAttach, GsmGetGprsAttachReply},/* 16 */
	{"+CGATT", 2, 5, true, true, NULL, GsmSetGprsAttach, GsmSetGprsAttachReply},/* 17 */
	{"+QICSGP", 1, 2, true, true, NULL, GsmSetConMode, GsmSetConModeReply},/* 18 */
	{"+QICLOSE", 2, 2, false, true, "CLOSE", GsmSetIpClosePeer, GsmSetIpClosePeerReply},/* 19*/
	{"+QIDEACT", 2, 10, false, true, "DEACT", GsmSetDeActGprs, GsmSetDeActGprsReply},/* 20 */
	{"+QISTAT", 1, 2, false, true, "STATE", GsmGetIpConState, GsmGetIpConStateReply},/* 21 */
	{"+QISEND", 1, 5, true, false, NULL, GsmReadySend, GsmReadySendReply},/* 22 */
	{"", 2, 15, false, true, "SEND", GsmSendFrame, GsmSendFrameReply},/* 23 */
	{"+QISACK", 2, 5, false, true, NULL, GsmGetSend, GsmGetSendReply},/* 24 */
	{"+QINDI", 1, 2, true, true, NULL, GsmSetRxInMode, GsmSetRxInModeReply},/* 25 */
	{"+QIRD", 2, 5, true, true, NULL, GsmGetRxBuffer, GsmGetRxBufferReply},/* 26 */
	{"+QIMODE?", 1, 2, false, true, NULL, GsmGetIpMode, GsmGetIpModeReply},/* 27 */
	{"+GSN", 1, 2, false, true, NULL, GsmGetImei, GsmGetImeiReply},/* 28 */
};


/*************************Quectel Data Interface**************************/
static void Quectel_Run_Cmd(uint8_t cmd_idx)
{
	if(cmd_idx != NULL_CMD)
	{	
		CmdFuncItem* ItemPtr = (CmdFuncItem*)(CmdFuncList+cmd_idx);
		
		((CmdFunc)(ItemPtr->FuncPtr))();
		QuectelIfRec.WaitReply = true;
		QuectelIfRec.AtRuning = true;
	}
	else
	{
		QuectelIfRec.WaitReply = false;
		QuectelIfRec.AtRuning = false;
	}
	
	QuectelIfRec.TimerCount = 0;
	QuectelIfRec.WaitTry = 0;
	QuectelIfRec.MaxTry = 0;
	QuectelIfRec.CmdIdxCur = cmd_idx;
	QuectelIfRec.CmdIdxNext = NULL_CMD;
}

#define REPLY_END_FIX_SIZE 4
char QuectelReplyResult[4] = {0x4f, 0x4b, 0x0d, 0x0a};
static bool Quectel_Wait_Reply(bool EndFix, char* AddReply, uint8_t AddReplyLen, bool FirstTime)
{
	uint16_t i;
	bool EndFound = false;

	/* Copy UART buffer to DataTaskBuffer */
	Quectel_Copy_Rx_Buffer(FirstTime);

	if(DataTaskMsgBufferLen == 0)
	{
		/* didn't receive any thing */
		return false;
	}

	/* Try to find EndFix */
	if(!EndFix)
	{
		/* No need to find Endfix, for example get local ip, return directly */
		return true;
	}

	/* whether to find more reply? */
	if(AddReply)
	{
		for(i = 0; i < DataTaskMsgBufferLen; i ++)
		{
			if(DataTaskMsgBuffer[i] == AddReply[0])
			{
				/* Get a Prefix, compare other characters */
				if((((DataTaskMsgBufferLen-i) > AddReplyLen) &&
					(!memcmp(&DataTaskMsgBuffer[i], AddReply, AddReplyLen))) &&
					(i >= 1) && 
					(DataTaskMsgBuffer[i-1] != 'I'))
				{
					QUECTEL_DEBUG("Cmd[%d] Found %s\r\n", QuectelIfRec.CmdIdxCur, AddReply);
					return true;
				}
			}
		}
		
		QUECTEL_DEBUG("Cmd[%d] not Found %s\r\n", QuectelIfRec.CmdIdxCur, AddReply);
		return false;
	}

	/* EndFix should be OK\r\n */
	for(i = 0; i < DataTaskMsgBufferLen; i ++)
	{
		if(DataTaskMsgBuffer[i] == 'O')
		{
			/* Get a O, compare other characters */
			if(((DataTaskMsgBufferLen-i) >= REPLY_END_FIX_SIZE) &&
				(!memcmp(&DataTaskMsgBuffer[i], QuectelReplyResult, REPLY_END_FIX_SIZE)))
			{
				EndFound = true;
				break;
			}
		}
	}
	
	QUECTEL_DEBUG("Cmd[%d] %s Found OK\r\n", QuectelIfRec.CmdIdxCur, EndFound?"":"not");
	return EndFound;
}

#define QUECTEL_WAIT_REPLY_MAX_TRY 5
static void Quectel_Handle_Reply(uint8_t cmd_idx, bool FirstTime)
{
	bool Timeout = false;
	CmdFuncItem* ItemPtr = (CmdFuncItem*)(CmdFuncList+cmd_idx);
	
	if(!Quectel_Wait_Reply(ItemPtr->EndFix, ItemPtr->AddReply, 
				(ItemPtr->AddReply)?strlen(ItemPtr->AddReply):0, FirstTime))
	{
		QuectelIfRec.WaitTry ++;
		if(QuectelIfRec.WaitTry >= ItemPtr->WaitTry)
		{
			Timeout = true;
			QuectelIfRec.MaxTry ++;
		}
		else
		{
			QuectelIfRec.TimerCount = 0; 
			return;
		}
	}

	QuectelIfRec.AtRuning = false;
	QuectelIfRec.WaitReply = false;

	if(Timeout)
	{
		if(QuectelIfRec.MaxTry >= QUECTEL_WAIT_REPLY_MAX_TRY)
		{
			/* Need to reset Quectel module */
			QuectelIfRec.ToReset = true;
			DataIfRec.ToSend = true;
			DataIfRec.State = DataIfStateStartup;
			QuectelIfRec.CmdIdxCur = NULL_CMD;
		}
		else
		{
			QuectelIfRec.CmdIdxNext = QuectelIfRec.CmdIdxCur;
		}
		return;
	}
	
	((CmdFunc)(ItemPtr->ReplyFuncPtr))();
}

#define DATA_RX_INDI_PREFIX	"+QIRDI:"
#define DATA_RX_CPIN_READY	"+CPIN: READY"
#define DATA_RX_CALL_READY	"Call Ready"
#define DATA_RX_SMS_READY		"SMS Ready"
#define DATA_RX_IP_CLOSED		"CLOSED"
static void Quectel_Notice_Handler(char* StrPtr, uint8_t Len)
{
	if(!strncmp(StrPtr, DATA_RX_CPIN_READY, strlen(DATA_RX_CPIN_READY)))
	{
		/* CPIN ready */
		return;
	}
	
	if(!strncmp(StrPtr, DATA_RX_CALL_READY, strlen(DATA_RX_CALL_READY)))
	{
		/* Call ready */
		return;
	}
	
	if(!strncmp(StrPtr, DATA_RX_SMS_READY, strlen(DATA_RX_SMS_READY)))
	{
		/* Sms ready */
		QuectelIfRec.State = QuectelIfStateReceived;
		return;
	}

	if(!strncmp(StrPtr, DATA_RX_INDI_PREFIX, strlen(DATA_RX_INDI_PREFIX)))
	{
		/* indication that receive TCP message from remote server */
		/* save the indication context */
		QUECTEL_DEBUG("Receive frame notify...\r\n");
		Quectel_Run_Cmd(GET_RX_BUFFER_CMD);
		return;
	}

	if(!strncmp(StrPtr, DATA_RX_IP_CLOSED, strlen(DATA_RX_IP_CLOSED)))
	{
		/* indication that ip connection closed */
		QUECTEL_DEBUG("ip connection closed notify...\r\n");
		if((QuectelIfRec.State >= QuectelIfStateIpConnected) && 
			(QuectelIfRec.State <= QuectelIfStateSendDone))
		{
			QuectelIfRec.State = QuectelIfStateIpDisConnected;
		}
		return;
	}
}

static void Quectel_UART_Rx_Handler(void)
{
	Quectel_Copy_Rx_Buffer(true);

	if(DataTaskMsgBufferLen == 0)
	{
		/* didn't receive any thing */
		return;
	}

	Quectel_Print("Notice", DataTaskMsgBuffer, DataTaskMsgBufferLen);

	while(Quectel_Parse_Reply(true))
	{
		Quectel_Notice_Handler(DataTaskParseBuffer, DataTaskParseBufferLen);
	}
}

/************** Data Task ****************/
void Quectel_If_Start(void)
{
	memset(&QuectelIfRec, 0, sizeof(QuectelIfConfig));

	CmdFuncList = QuectelGsmCmdList;

	/* start receiving character from GPRS module */
	Data_UART_Start();
}

static void Quectel_PowerKey_Set(bool Set)
{
}

#define RESET_POWER_KEY_TIMES 5
#define HEARTBEAT_AT_LINK_PERIOD 600
void Quectel_If_Poll(QuectelPollType Cmd)
{
	switch (Cmd)
	{
		case QuectelPollReset:
			switch (QuectelIfRec.State)
			{
				case QuectelIfStateReseting:
					/* check timer counter */
					QuectelIfRec.TimerCount ++;
					if(QuectelIfRec.TimerCount >= RESET_POWER_KEY_TIMES)
					{
						/* Relese GPIO */
						Quectel_PowerKey_Set(false);
						/* Reset done, need to wait and handle module notice */
						QuectelIfRec.State = QuectelIfStateInitialized;
						QuectelIfRec.ToReset = false;
						QuectelIfRec.AtCount = 0;
						QUECTEL_DEBUG("Reset Quectel done...\r\n");
					}
					break;
				
				case QuectelIfStateActivated:
				case QuectelIfStateIpDisConnected:
					/*Need to de-activate if activated*/
					Quectel_Run_Cmd(SET_DEACT_GPRS_CMD);
					break;
				
				case QuectelIfStateIpConnected:
				case QuectelIfStateSendReady:
				case QuectelIfStateSendOut:
				case QuectelIfStateSendDone:
					/*Need to ip close if connected*/
					Quectel_Run_Cmd(SET_IP_CLOSE_PEER_CMD);
					QuectelIfRec.TimerCount = 0;
					break;
					
				case QuectelIfStateInitialized:
					/* waiting for module notification */
					Quectel_UART_Rx_Handler();
					break;
					
				case QuectelIfStateNull:
				case QuectelIfStateReceived:
				case QuectelIfStateRegistered:
				case QuectelIfStateAttached:
				case QuectelIfStateDeActivated:
				default:
					{
						/* To reset Quectel module, just set PowerKey GPIO for N second*/
						QuectelIfRec.State = QuectelIfStateReseting;
						QuectelIfRec.TimerCount = 0;
						/* Set associated GPIO */
						Quectel_PowerKey_Set(true);
						QUECTEL_DEBUG("Reset Quectel start...\r\n");
					}
					break;
			}
			break;
			
		case QuectelPollLink:
			if(QuectelIfRec.State == QuectelIfStateInitialized)
			{
				/* reset done. meed tp waiting for notice */
				Quectel_UART_Rx_Handler();
			}
			else
			{
				/* Send AT to make sure module connected*/
				Quectel_Run_Cmd(AT_TEST_CMD);
			}
			break;
			
		case QuectelPollIpConnect:
			/* Send AT to make sure module connected*/
			Quectel_Run_Cmd(GET_SIM_CMD);
			break;
			
		case QuectelPollSend:
			switch (QuectelIfRec.State)
			{
				case QuectelIfStateIpConnected:
					/* Prepare to Send */
					Quectel_Run_Cmd(READY_SEND_CMD);
					break;
				
				case QuectelIfStateSendReady:
					/* Send buffer */
					Quectel_Run_Cmd(SEND_FRAME_CMD);
					break;
					
				case QuectelIfStateSendOut:
					/* Get send feedback */
					Quectel_Run_Cmd(GET_SEND_CMD);
					break;
					
				case QuectelIfStateSendDone:
					/* Send done */
					QuectelIfRec.State = QuectelIfStateIpConnected;
					break;

				default:
					/* need to check ip connection status */
					Quectel_Run_Cmd(GET_IP_CON_STATE);
					break;
			}
			
		case QuectelPollAt:
			if(QuectelIfRec.WaitReply)
			{
				/* wait for reply, check waiting period */
				if(QuectelIfRec.TimerCount < ATGETWAITTIME(QuectelIfRec.CmdIdxCur))
				{	
					/* continue waiting */
					break;
				}
				
				/* handle reply */
				Quectel_Handle_Reply(QuectelIfRec.CmdIdxCur, (QuectelIfRec.WaitTry==0)?true:false);
			}

			if(QuectelIfRec.CmdIdxNext)
			{
				Quectel_Run_Cmd(QuectelIfRec.CmdIdxNext);
			}
			break;
			
		case QuectelPollNormal:
			if((DataIfRec.State == DataIfStateHeartBeat) && 
				((DataIfRec.TimerCount%HEARTBEAT_AT_LINK_PERIOD) == 0))
			{
				/* Run AT to make sure Quectel module alive */
				Quectel_Run_Cmd(AT_TEST_CMD);
			}
		default:
			/* check Uart Notice 
				for example, networking disconnect unexpectly, 
				ip frame received notification*/
			Quectel_UART_Rx_Handler();
			break;
	}
}

