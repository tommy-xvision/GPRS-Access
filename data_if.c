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

#include "main.h"
#include "uart_if.h"
#include "data_if.h"
#include "timer_if.h"
#include "quectel_if.h"
#include "state_if.h"
#include "utility.h"

/* Reset Qutetel module frequently is not a good idea,
	before communication we can send AT to make module normal*/
//#define DATA_MODULE_RESET

#define DATA_IF_DEBUG
#ifdef DATA_IF_DEBUG
#define DATA_DEBUG(...) printf(__VA_ARGS__)
#else
#define DATA_DEBUG(...)
#endif

/* CRC handler declaration */
CRC_HandleTypeDef CrcHandle;

char DataTaskMsgBuffer[DATA_RX_BUFFER_SIZE];
uint16_t DataTaskMsgBufferLen;

#define DATA_TASK_MSG_OFFSET 792
char* DataTaskMsgSendBufferPtr = DataTaskMsgBuffer+DATA_TASK_MSG_OFFSET;
uint16_t DataTaskMsgSendBufferLen;
extern uint16_t DataTaskParseCount;

/* Variable used for System parameters */
extern SystemParamConfig SystemParamRec;

/* Buffer used for transmission */
extern char DataTxBuffer[DATA_TX_BUFFER_SIZE];
extern uint8_t DataTxBufferLen;


#define DATA_IF_MSG_HEADER_SIZE	4
#define DATA_IF_MSG_VALUE_PTR(x) ((uint8_t*)x+DATA_IF_MSG_HEADER_SIZE)
/****************** Message Handler ***********************/
static void Data_Print(char* PreFix, char* StrPtr, uint8_t Len)
{
	uint8_t i;
	DATA_DEBUG("%s[%d]: ", PreFix, Len);
	for(i = 0; i < Len; i ++)
	{
		DATA_DEBUG("%02X ", StrPtr[i]);
	}

	DATA_DEBUG("\r\n");
}

/*all of the messages includes SN*/
static void Data_Construct_Msg(DataMsgOpCode OpCode)
{
	DataIfMsgHeader* MsgPtr = (DataIfMsgHeader*)DataTaskMsgSendBufferPtr;
	DataIfMsgHeader* SubMsgPtr = (DataIfMsgHeader*)(DataTaskMsgSendBufferPtr+DATA_IF_MSG_HEADER_SIZE);
	uint8_t* SubValuePtr = DATA_IF_MSG_VALUE_PTR(SubMsgPtr);

	memset(DataTaskMsgSendBufferPtr, 0, DATA_RX_BUFFER_SIZE-DATA_TASK_MSG_OFFSET);
	
	/* Fill header info */
	MsgPtr->Code = OpCode;

	/* Fill SN */
	SubMsgPtr->Code = SubCodeSn;
	SubMsgPtr->Len = 0x16;
	memcpy(SubValuePtr, SystemParamRec.SerialNum, SubMsgPtr->Len);
	MsgPtr->Len = SubMsgPtr->Len+DATA_IF_MSG_HEADER_SIZE;
	DataTaskMsgSendBufferLen = MsgPtr->Len + DATA_IF_MSG_HEADER_SIZE;
}

static void Data_Fill_Sub_Msg(DataMsgSubCode SubCode, uint8_t* DataPtr, uint16_t	Len)
{
	DataIfMsgHeader* MsgPtr = (DataIfMsgHeader*)DataTaskMsgSendBufferPtr;
	DataIfMsgHeader* SubMsgPtr = (DataIfMsgHeader*)(DataTaskMsgSendBufferPtr+DataTaskMsgSendBufferLen);
	uint8_t* SubValuePtr = DATA_IF_MSG_VALUE_PTR(SubMsgPtr);

	
	/* Fill Sub msg*/
	SubMsgPtr->Code = SubCode;
	switch (SubCode)
	{
		case SubCodeProfile:
			SubMsgPtr->Len = Len;
			memcpy(SubValuePtr, DataPtr, Len);
			break;
			
		case SubCodeVr:
		case SubCodeEvent:
		case SubCodeAction:
		default:
			SubMsgPtr->Len = 1;
			*SubValuePtr = *DataPtr;
			break;
	}
	MsgPtr->Len += (SubMsgPtr->Len+DATA_IF_MSG_HEADER_SIZE);
	DataTaskMsgSendBufferLen = MsgPtr->Len + DATA_IF_MSG_HEADER_SIZE;
}

void Data_Fill_CRC(void)
{
	uint32_t CrcRes;
	
	CrcRes = HAL_CRC_Accumulate(&CrcHandle, (uint32_t*)DataTaskMsgSendBufferPtr, DataTaskMsgSendBufferLen);
	memcpy(DataTaskMsgSendBufferPtr+DataTaskMsgSendBufferLen, (uint8_t*)&CrcRes, sizeof(uint32_t));
	DataTaskMsgSendBufferLen += sizeof(uint32_t);
}

static void Data_Handle_Msg(char* DataPtr, uint16_t Len)
{
#if 0
	DataIfMsgHeader* MsgPtr = (DataIfMsgHeader*)(DataPtr);
	DataIfMsgHeader* SubMsgPtr = (DataIfMsgHeader*)(DataPtr+sizeof(DataIfMsgHeader));
	uint8_t* SubValuePtr = DATA_IF_MSG_VALUE_PTR(SubMsgPtr);

	if(MsgPtr->Len < 0x1D)
	{
		/* Too short, invalid message */
		DATA_DEBUG("To short message\r\n");
		return;
	}

	/* First need to check SN */
	if((SubMsgPtr->Code != SubCodeSn) || 
		(SubMsgPtr->Len != 0x16) || 
		(memcmp(SubValuePtr, SystemParamRec.SerialNum, 0x16)))
	{
		/* Wrong SN, invalid message */
		DATA_DEBUG("Wrong SerialNum\r\n");
		return;
	}
#else
	Data_Print("Msg Receive", DataPtr, Len);
#endif
}

static void Data_Pick_Msg(char* DataPtr, uint16_t Len)
{
	uint16_t i;
	bool ParseFlg = false;
	
	/* Frame from Quectel module looks like:
		+QIRD: 116.226.32.138:9897,TCP,22
		Feedback to SmartCover
		OK */
#define DATA_RECEIVE_PREFIX "+QIRD"	
#define DATA_RECEIVE_PREFIX_LEN 5
	/* To make sure the prefix is right */
	if(Len < DATA_RECEIVE_PREFIX_LEN)
	{
		/* To short, invalid frame */
		return;
	}
	
	for(i = 0; i < Len; i++)
	{
		if(!memcmp(DataPtr+i, DATA_RECEIVE_PREFIX, DATA_RECEIVE_PREFIX_LEN))
		{
			ParseFlg = true;
			break;
		}
	}

	if(ParseFlg)
	{
		uint8_t CommaCount = 0;
		uint8_t LenCount = 0;
		uint32_t MsgLen;
		char LenStr[6];

		memset(LenStr, 0, sizeof(LenStr));
		
		for(i = DATA_RECEIVE_PREFIX_LEN; i < (Len-DATA_RECEIVE_PREFIX_LEN); i++)
		{
			if(CommaCount == 2)
			{
				/* Message Length found */
				if(*(DataPtr+i) == CHAR_CR)
				{
					LenStr[LenCount++] = '\0';
					break;
				}
				else
				{
					LenStr[LenCount++] = *(DataPtr+i);
				}
			}
			
			if(*(DataPtr+i) == ',')
			{
				CommaCount ++;
			}
		}

		Str2Int((uint8_t*)LenStr, &MsgLen);
		/* Pass character: CHAR_LF */
		i += 2;

		Data_Handle_Msg(DataPtr+i, MsgLen);
	}
}

DataIfConfig	DataIfRec;
DataTaskConfig DataTaskRec;

extern QuectelIfConfig QuectelIfRec;

/**************************** Data Task *****************************/
void Data_Decide_Sn(char* ImeiStr)
{
	/* IMEI will be 15 characters as 861358030152342 */
#define SN_VENDOR_OFFSET 7
#define IMEI_LENGTH 15
	if(memcmp((char*)(SystemParamRec.SerialNum)+SN_VENDOR_OFFSET, ImeiStr, IMEI_LENGTH))
	{
		/* IMEI mismatch, overwrite new IMEI, and write to flash */
		memcpy((char*)(SystemParamRec.SerialNum)+SN_VENDOR_OFFSET, ImeiStr, IMEI_LENGTH);
		SystemParamRec.SerialNum[SN_VENDOR_OFFSET+IMEI_LENGTH] = '\0';
		memcpy((char*)SystemParamRec.SerialNum, "xVision", SN_VENDOR_OFFSET);
		/* Save into Internel Flash */
	}
}

void Data_To_Alert(DataMsgEventType Event)
{
	DATA_DEBUG("Alert type[%d]\r\n", Event);
	DataIfRec.ToAlert = true;
	DataTaskRec.EventType = Event;
}

bool Data_Get_Alert(void)
{
	return DataIfRec.ToAlert;
}

bool Data_Dev_DeAct(void)
{
	QuectelIfRec.ToReset = true;
	DataIfRec.ToReset = true;
	DataIfRec.State = DataIfStateStartup;
	if((QuectelIfRec.State == QuectelIfStateReseting) || 
		(QuectelIfRec.State <= QuectelIfStateDeActivated))
	{
		return true;
	}
	else
	{
		return false;
	}
}

static void Data_Quectel_Poll(QuectelPollType Cmd)
{
	QuectelIfRec.TimerCount ++;
	Quectel_If_Poll(Cmd);
}

void Data_Dev_Link(void)
{
	DataIfRec.ToReset = true;
#ifdef DATA_MODULE_RESET
	QuectelIfRec.ToReset = true;
	DATA_DEBUG("To Reset module...\r\n");
	Data_Quectel_Poll(QuectelPollReset);
#else
	DATA_DEBUG("To Link module...\r\n");
	Data_Quectel_Poll(QuectelPollLink);
#endif
}

void Data_If_Start(void)
{
  CrcHandle.Instance = CRC;
  if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
  {
    Error_Handler();
  }
	
	/* Initialization */
	memset(&DataIfRec, 0, sizeof(DataIfConfig));
	memset(&DataTaskRec, 0, sizeof(DataTaskConfig));

	/* Start task */
	Quectel_If_Start();

	/* Start polling timer */
	Data_Timer_Start();

	/* Link Data module */
	Data_Dev_Link();
}

void Data_If_Poll(void)
{
	if(DataIfRec.TimNotify == false)
	{
		/* wait time notification */
		return;
	}
	
	BSP_LED_Toggle(LED4);
	
	/* Quectecl module is running AT command, just let it go */
	if(QuectelIfRec.AtRuning)
	{
		Data_Quectel_Poll(QuectelPollAt);
		goto Poll_Exit;
	}

	switch (DataIfRec.State)
	{
		case DataIfStateStartup:
			/* Need to reset data module and make sure AT communication */
			if(DataIfRec.ToReset)
			{
				if(QuectelIfRec.ToReset)
				{
					Data_Quectel_Poll(QuectelPollReset);
				}
				else
				{
					Data_Quectel_Poll(QuectelPollLink);
				}
			}
			else
			{
				/* Module initialization done, to do ip connection */
				DataIfRec.State = DataIfStateConnecting;
				DATA_DEBUG("To Connect remote...\r\n");
			}
		break;
		
		case DataIfStateConnecting:
			/* Need to register to GPRS networking and connect remote server */
			if(DataIfRec.Connected)
			{
				DataIfRec.State = DataIfStateRegistering;
				DATA_DEBUG("To Register remote...\r\n");
			}
			else
			{
				Data_Quectel_Poll(QuectelPollIpConnect);
			}
			break;
		
		case DataIfStateRegistering:
			/* Need to send registration frame to server */
			if(DataIfRec.WaitReply == true)
			{
				if(!QuectelIfRec.IpReceived)
				{
					Data_Quectel_Poll(QuectelPollNormal);
				}
				else
				{
					/* Handle received ip frame */
					Data_Pick_Msg(DataTaskMsgBuffer+DataTaskParseCount, DataTaskMsgBufferLen-DataTaskParseCount);
					DataIfRec.State = DataIfStateConfirm;
					QuectelIfRec.IpReceived = false;
					DataIfRec.WaitReply = false;
				}
			}
			else 
			{
				if(!DataIfRec.ToSend)
				{
					/* send AT to send */
					DataIfRec.ToSend = true;
					/* Fill registration frame */
					Data_Construct_Msg(OpCodeReg);
				}
				else
				{
					
					if(QuectelIfRec.State == QuectelIfStateSendDone)
					{
						/* send done, waiting for reply */
						DataIfRec.ToSend = false;
						DataIfRec.WaitReply = true;
					}
				}
				Data_Quectel_Poll(QuectelPollSend);
			}
			break;
		
		case DataIfStateConfirm:
			/* Need to send confirmation to server */
			if(!DataIfRec.ToSend)
			{
				uint8_t VrRes = 1;
				/* send AT to send */
				DataIfRec.ToSend = true;
				/* Fill confirmation frame */
				Data_Construct_Msg(OpCodeCim);
				Data_Fill_Sub_Msg(SubCodeVr, &VrRes, 1);
			}
			else
			{
				if(QuectelIfRec.State == QuectelIfStateSendDone)
				{
					/* send done, registration done */
					DataIfRec.ToSend = false;
					/* start heartbeat timer */
					DataIfRec.State = DataIfStateHeartBeat;
					DataIfRec.TimerCount = 0;
					DATA_DEBUG("Reg to start heartbeat...\r\n");
				}
				Data_Quectel_Poll(QuectelPollSend);
			}
			break;
		
		case DataIfStateHeartBeat:
			/* Need to send heartbeat to keep alive */
#if 0
			if(DataIfRec.WaitReply == true)
			{
				if(!QuectelIfRec.IpReceived)
				{
					Data_Quectel_Poll(QuectelPollNormal);
				}
				else
				{
					/* Handle received ip frame */
					//TBD
					DataIfRec.State = DataIfStateHeartBeat;
					QuectelIfRec.IpReceived = false;
					DataIfRec.WaitReply = false;
					DataIfRec.TimerCount = 0;
					DATA_DEBUG("Restart heartbeat...\r\n");
				}
			}
			else 
#endif
			{
				if(DataIfRec.ToSend)
				{
					if(QuectelIfRec.State == QuectelIfStateSendDone)
					{
						/* send done, waiting for reply */
						DataIfRec.ToSend = false;
						DataIfRec.WaitReply = false;
						DataIfRec.TimerCount = 0;
						DATA_DEBUG("Restart heartbeat...\r\n");
					}
					else
					{
						Data_Quectel_Poll(QuectelPollSend);
					}
				}
				else 
				{
					if(DataIfRec.ToAlert)
					{
						/* Need to alert */
						DataIfRec.State = DataIfStateAlert;
						break;
					}
					
					DataIfRec.TimerCount ++;
					if(DataIfRec.TimerCount > SystemParamRec.HbPeriod)
					{
						/* timeout, need to send heartbeat frame and wait for feedback */
						DataIfRec.ToSend = true;
						/* Fill heart beat frame */
						Data_Construct_Msg(OpCodeHb);
						Data_Quectel_Poll(QuectelPollSend);
					}
					else
					{
						Data_Quectel_Poll(QuectelPollNormal);
					}
				}
			}
			break;
		
		case DataIfStateAlert:
			/* Need to send Alert to server */
			if(DataIfRec.WaitReply == true)
			{
				if(!QuectelIfRec.IpReceived)
				{
					Data_Quectel_Poll(QuectelPollNormal);
				}
				else
				{
					/* Handle received ip frame */		
					Data_Pick_Msg(DataTaskMsgBuffer+DataTaskParseCount, DataTaskMsgBufferLen-DataTaskParseCount);
					QuectelIfRec.IpReceived = false;
					DataIfRec.WaitReply = false;
					DataIfRec.State = DataIfStateHeartBeat;
					DataIfRec.TimerCount = 0;
					State_Task_Refresh();
					DataIfRec.ToAlert = false;
					DATA_DEBUG("Alert to start heartbeat...\r\n");
				}
			}
			else 
			{
				if(DataIfRec.ToSend)
				{
					if(QuectelIfRec.State == QuectelIfStateSendDone)
					{
						/* send done, waiting for reply */
						DataIfRec.ToSend = false;
						DataIfRec.WaitReply = true;
					}
					else
					{
						Data_Quectel_Poll(QuectelPollSend);
					}
				}
				else 
				{
					DataIfRec.ToSend = true;
					/* Fill alert frame */
					Data_Construct_Msg(OpCodeAlert);
					Data_Fill_Sub_Msg(SubCodeEvent, &(DataTaskRec.EventType), 1);
					Data_Quectel_Poll(QuectelPollSend);
				}
			}
			break;
		
		case DataIfStateCommand:
			/* Got ip frame received notification, need to get the frame buffer */
			
			break;

		case DataIfStateNormal:
		default:
			/* Normal polling */
			if(DataIfRec.ToAlert)
			{
				/* Need to alert */
				DataIfRec.State = DataIfStateAlert;
			}
			Data_Quectel_Poll(QuectelPollNormal);
		break;
	}

Poll_Exit:
	DataIfRec.TimNotify = false;
}

void Data_If_Set(uint8_t Type)
{
	if(Type == 1)
	{
	  QuectelIfRec.State = QuectelIfStateReceived;
	}
	else 
	{
		DataIfRec.State = DataIfStateNormal;
		QuectelIfRec.CmdIdxCur = 0;
		QuectelIfRec.CmdIdxNext = 0;
		QuectelIfRec.WaitReply = false;
		QuectelIfRec.AtRuning = false;
		QuectelIfRec.TimerCount = 0;
		QuectelIfRec.WaitTry = 0;
	}
}


/*********************For CLI Command***********************/
void Data_Cmd_SendAT(char* StrPtr)
{
	DataTxBufferLen = 0;
	if(StrPtr)
	{
		memcpy((void*)&(DataTxBuffer[DataTxBufferLen]), (void*)StrPtr, strlen(StrPtr));
		DataTxBufferLen += strlen(StrPtr);
	}
	else
	{
		DataTxBuffer[DataTxBufferLen++] = 'A';
		DataTxBuffer[DataTxBufferLen++] = 'T';
	}
	
	DataTxBuffer[DataTxBufferLen++] = CHAR_CR;
	DataTxBuffer[DataTxBufferLen++] = CHAR_LF;

  Data_UART_Send(DataTxBuffer, DataTxBufferLen);
}


