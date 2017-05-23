/*
 *  Copyright (c) 2017, xVision, Inc.
 *  All rights reserved.
 *
 *
 */

/* Author: Tommy.Cao */

#ifndef _QUECTEL_IF_H
#define _QUECTEL_IF_H

#include "main.h"
#include "stdbool.h"

/****************************************/
typedef struct
{
	char* CmdLine;
	uint8_t WaitTime;
	uint8_t WaitTry;
	bool AnyArgu;
	bool EndFix;
	char* AddReply;
	void* FuncPtr;
	void* ReplyFuncPtr;
} CmdFuncItem;

typedef enum
{	
	QuectelCarrierUnicom,
	QuectelCarrierCmcc
} QuectelCarrierNum;

typedef enum
{
	QuectelIfStateNull,
	QuectelIfStateInitialized,
	QuectelIfStateReceived,
	QuectelIfStateLinked,
	QuectelIfStateRegistered,
	QuectelIfStateAttached,
	QuectelIfStateDeActivated,
	QuectelIfStateActivated,
	QuectelIfStateIpDisConnected,
	QuectelIfStateIpConnected,
	QuectelIfStateSendReady,
	QuectelIfStateSendOut,
	QuectelIfStateSendDone,
	QuectelIfStateReseting
} QuectelIfStateNum;

typedef struct
{
	QuectelIfStateNum State;
	bool AtRuning;
	bool ToReset;
	bool IpReceived;
	uint8_t	CmdIdxCur;
	uint8_t	CmdIdxNext;
	uint8_t	TimerCount;
	uint8_t AtCount;
	uint8_t WaitTry;
	bool WaitReply;
	uint8_t MaxTry;
} QuectelIfConfig;

typedef enum
{
	QuectelPollReset,
	QuectelPollLink,
	QuectelPollIpConnect,
	QuectelPollSend,
	QuectelPollNormal,
	QuectelPollAt
} QuectelPollType;

void Quectel_If_Start(void);
void Quectel_If_Poll(QuectelPollType Cmd);

#endif

