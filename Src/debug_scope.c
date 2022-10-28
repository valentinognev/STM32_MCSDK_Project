
/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
//cstat -MISRAC2012-Rule-21.1
#include "debug_scope.h" //cstat !MISRAC2012-Rule-21.1

int64_t TickMSCounter = 0;

int64_t getTickMSCounter()
{
	return TickMSCounter;
}

DebugWriteState DebugScopeInsertData(DebugScope_Handle_t *pHandle, const int8_t chid, const int16_t data)
{
	if (!pHandle->startWriteFlag)
		return START_FLAG_IS_OFF ;
	if (chid < 1 || DEBUGSCOPENUMOFCH < chid) return CHID_IS_OUT_OF_BOUNDS;
	if (chid == 1)
	{
		if (pHandle->i1 == DEBUGSCOPESIZE)
			return NO_MORE_PLACE_TO_WRITE; // pHandle->i1 = 0;
		pHandle->Ch1[pHandle->i1++]=data;
	}
	else if (chid == 2)
	{
		if (pHandle->i2 == DEBUGSCOPESIZE)
			return NO_MORE_PLACE_TO_WRITE; // pHandle->i2 = 0;
		pHandle->Ch2[pHandle->i2++] = data;
	}
	else if (chid == 3)
	{
		if (pHandle->i3 == DEBUGSCOPESIZE)
			return NO_MORE_PLACE_TO_WRITE; // pHandle->i3 = 0;
		pHandle->Ch3[pHandle->i3++] = data;
	}
	else if (chid == 4)
	{
		if (pHandle->i4 == DEBUGSCOPESIZE)
			return NO_MORE_PLACE_TO_WRITE; // pHandle->i4 = 0;
		pHandle->Ch4[pHandle->i4++] = data;
	}
	else if (chid == 5)
	{
		if (pHandle->i5 == DEBUGSCOPESIZE)
			return NO_MORE_PLACE_TO_WRITE; // pHandle->i5 = 0;
		pHandle->Ch5[pHandle->i5++] = data;
	}
	return NO_ERROR;
}

void DebugScopeStartWrite(DebugScope_Handle_t *pHandle)
{
	pHandle->startWriteFlag = true;
	pHandle->i1 = pHandle->i2 = pHandle->i3 = pHandle->i4 = pHandle->i5 = 0;
}