/*
 *  Copyright (c) 2017, xVision, Inc.
 *  All rights reserved.
 *
 *
 */

/* Author: Tommy.Cao */

#ifndef _DATA_IF_H
#define _DATA_IF_H

#include "main.h"
#include "stdbool.h"

/* Reset Qutetel module frequently is not a good idea,
	before communication we can send AT to make module normal*/
//#define DATA_MODULE_RESET

/****************************************/
/* Message between Client and Server */
typedef struct
{
	uint16_t	Code;
	uint16_t	Len;
}DataIfMsgHeader;

typedef enum
{
	OpCodeNull,
	OpCodeReg,
	OpCodeAck,
	OpCodeCim,
	OpCodeHb,
	OpCodeAlert,
	OpCodeCmd,
	OpCodeRst,
	OpCodeFw,

	OpCodeMax = 0xFFFF
}DataMsgOpCode;

typedef enum
{
	SubCodeNull,
	SubCodeSn,
	SubCodeVr,
	SubCodeProfile,
	SubCodeEvent,
	SubCodeAction,

	SubCodeMax = 0xFFFF
}DataMsgSubCode;

typedef enum
{
	EventTilt = 0x00,
	EventMove,
	EventLowPower
}DataMsgEventType;

typedef enum
{
	ActionStartMonitor = 0x00,
	ActionStopMonitor,
	ActionStartBeep,
	ActionStopBeep,
	ActionFactoryRestore,
	ActionUpdateProfile,
	ActionUpgradeFw
}DataMsgActionType;

/********************************************/
/* Task configurtion and flag */
typedef struct
{
	DataMsgEventType EventType;
	DataMsgActionType ActionType;
}DataTaskConfig;

typedef enum
{
	DataIfStateStartup,
	DataIfStateConnecting,
	DataIfStateRegistering,
	DataIfStateConfirm,
	DataIfStateHeartBeat,
	DataIfStateAlert,
	DataIfStateReset,
	DataIfStateCommand,
	DataIfStateNormal,
}DataIfStateNum;

typedef struct
{
	DataIfStateNum State;
	bool ToReset;
	bool ToSend;
	bool ToAlert;
	bool ToCommand;
	bool Connected;
	bool Registered;
	uint16_t TimerCount;
	bool WaitReply;
	bool TimNotify;
} DataIfConfig;

/********************************************/
void Data_If_Start(void);
void Data_If_Poll(void);
void Data_If_Set(uint8_t Type);

void Data_Decide_Sn(char* ImeiStr);
void Data_To_Alert(DataMsgEventType Event);
bool Data_Get_Alert(void);
bool Data_Dev_DeAct(void);
void Data_Cmd_SendAT(char* StrPtr);
#endif

