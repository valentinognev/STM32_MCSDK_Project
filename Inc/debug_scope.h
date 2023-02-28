/**
  ******************************************************************************
  * @file    mc_type.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control SDK global types definitions
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
  * @ingroup MC_Type
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEBUG_SCOPE_H
#define DEBUG_SCOPE_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MC_Type Motor Control types
  * @{
  */

/**
  * @define MISRA_C_2004_BUILD
  * @brief Used to build the library with MISRA C support
  *
  * Uncomment #define MISRA_C_2004_BUILD to build the library including
  * "stm32fxxx_MisraCompliance.h" instead of "stm32fxxx.h".
  *
  * This will build the library in 'strict ISO/ANSI C' and in
  * compliance with MISRA C 2004 rules (check project options).
  *
  * @note Do not use this flag with the current version of the SDK.
  */
#define DEBUG_SCOPE 1
#define DEBUGSCOPESIZE 1024
#define DEBUGSCOPENUMOFCH 3

typedef struct
{
  int16_t mechAnglePhaseDeg;
  int16_t mechMeanFreqHz;
  int16_t mechAmpFreqHz;
} OuterControlParameters_Handle_t;

typedef enum
{
  NO_ERROR = 0,
  CHID_IS_OUT_OF_BOUNDS,
  NO_MORE_PLACE_TO_WRITE,
  START_FLAG_IS_OFF
} DebugWriteState;
/**
  * @brief  ADConv_t type definition, it is used by PWMC_ADC_SetSamplingTime method of PWMnCurrFdbk class for user defined A/D regular conversions
  */
typedef struct
{
  const int16_t sz;
	int8_t curCh;
	int16_t i1, i2, i3, i4, i5;
  int16_t Ch1[DEBUGSCOPESIZE];
  int16_t Ch2[DEBUGSCOPESIZE];
  int16_t Ch3[DEBUGSCOPESIZE];
  int16_t Ch4[1];
	int16_t Ch5[1];
  uint8_t startWriteFlag;
} DebugScope_Handle_t;

int64_t getTickMSCounter();
DebugWriteState DebugScopeInsertData(DebugScope_Handle_t *pHandle, const int8_t chid, const int16_t data);
void DebugScopeStartWrite(DebugScope_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MC_TYPE_H */
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
