/**
******************************************************************************
* @file    ics_f7xx_pwm_curr_fdbk.h
* @author  Motor Control SDK Team, ST Microelectronics
* @brief   This file contains all definitions and functions prototypes for the
*          ICS PWM current feedback component for F7XX of the Motor Control SDK.
******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
******************************************************************************
  * @ingroup ICS_F7XX_pwm_curr_fdbk
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ICS_F7XX_PWMNCURRFDBK_H
#define __ICS_F7XX_PWMNCURRFDBK_H

#include "pwm_curr_fdbk.h"
#include "ics_dd_pwmncurrfdbk.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup ICS_pwm_curr_fdbk
  * @{
  */


#define SOFOC 0x0008u /* This flag is reset at the beginning of FOC
                           and it is set in the TIM UP IRQ. If at the end of
                           FOC this flag is set, it means that FOC rate is too 
                           high and thus an error is generated */

/*
 *  PWM and Current Feedback ICS handle.
 */
typedef struct
{
  PWMC_Handle_t _Super;              /* Base component handler. */
  uint32_t PhaseAOffset;             /* Offset of Phase A current sensing network. */
  uint32_t PhaseBOffset;             /* Offset of Phase B current sensing network. */
  uint16_t Half_PWMPeriod;           /* Half PWM Period in timer clock counts. */
  volatile uint8_t PolarizationCounter; /* Number of conversions performed during the calibration phase. */
  bool OverCurrentFlag;              /* This flag is set when an overcurrent occurs. */
  ICS_Params_t *pParams_str;
} PWMC_ICS_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*
 * Initializes TIMx, ADC, GPIO and NVIC for current reading
 * in ICS configuration using STM32F4XX.
 */
void ICS_Init( PWMC_ICS_Handle_t * pHandle );

/*
 * Sums up injected conversion data into wPhaseXOffset.
 */
void ICS_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

/*
 * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
 */
void ICS_GetPhaseCurrents( PWMC_Handle_t * pHandle, ab_t * pStator_Currents );

/*
 * Turns on low sides switches.
 */
void ICS_TurnOnLowSides( PWMC_Handle_t * pHandle, uint32_t ticks );

/*
 * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
 */
void ICS_SwitchOnPWM( PWMC_Handle_t * pHandle );

/*
 * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
 */
void ICS_SwitchOffPWM( PWMC_Handle_t * pHandle );

/*
 * Writes into peripheral registers the new duty cycle and sampling point.
 */
uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHandle );

void * ICS_IRQHandler( PWMC_Handle_t * pHandle, unsigned char flag );

/*
 * Checks if an overcurrent occurred since last call.
 */
uint16_t ICS_IsOverCurrentOccurred( PWMC_Handle_t * pHandle );

/*
 * Contains the TIMx Update event interrupt.
 */
void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHandle );

/*
 * Contains the TIMx Break1 event interrupt.
 */
void * ICS_BRK_IRQHandler( PWMC_ICS_Handle_t * pHdl );

/*
 * Stores in the handler the calibrated offsets.
 */
void ICS_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
 * Reads the calibrated offsets stored in the handler.
 */
void ICS_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__ICS_F7XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
