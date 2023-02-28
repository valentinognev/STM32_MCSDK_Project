/**
  ******************************************************************************
  * @file    acim_vf_control.c
  * @author  STMicroelectronics - SRA - System Development Unit - MC Team
  * @brief   This file provides firmware functions that implement the features
  *          of the V/f control of Induction Motor of the Motor Control SDK
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"
#include "acim_vf_control.h"
#include "math.h"    
#include "arm_math.h"

  /**
  * @brief  Software initialization of V/F component
  * @param  pHandle: handler of the current instance of the V/F component
  * @retval none
  */
void ACIM_VF_Init(ACIM_VF_Handle_t *pHandle)
{

  REMNG_Init(pHandle->pRMNGR);
  
  // Only for Closed loop V/F control  
  PI_Float_HandleInit(pHandle->fPI);
  
#if defined(MAIN_VF_OL)
  pHandle->bVF_Mode = VF_OL;
#elif defined(MAIN_VF_CL)
  pHandle->bVF_Mode = VF_CL;
#endif  
  
}

/**
* @brief  Software initialization of V/F object to be performed at each restart
*         of the motor.
* @param  pHandle: handler of the current instance of the V/F component
* @retval none
*/
void ACIM_VF_Clear(ACIM_VF_Handle_t *pHandle, BusVoltageSensor_Handle_t *pBVS)
{
  pHandle->fFlux_Angle_rad = 0;
  pHandle->fFlux_Freq_rads = 0;
  pHandle->fFlux_Freq_Hz = 0;
  pHandle->fSlipFrequency_rads = 0;
  pHandle->hElAngle = 0;
  
  pHandle->VoltageAmpl = 0;
  
  REMNG_Init(pHandle->pRMNGR);
  
  if(pHandle->bVF_Mode==VF_CL)
  {
    pHandle->fPI->fIntegralTerm = 0;
  }  
  
  uint16_t hVbus_V = VBS_GetAvBusVoltage_V(pBVS);
   
 if(hVbus_V != 0)
  {
    
    pHandle->fVoutMax_PWM_Mod_V = (float)hVbus_V/2.0f;
  }
}

#if (defined (CCMRAM) || defined(CCMRAM_ENABLED))
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#elif defined (__GNUC__)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  Update the rotor electrical angle integrating the last setled 
*         instantaneous electrical speed.
* @param  pHandle: handler of the current instance of the V/F component
* @retval none
*/
void ACIM_VF_CalcAngle(ACIM_VF_Handle_t *pHandle)
{
  
  pHandle->fFlux_Angle_rad += (pHandle->fFlux_Freq_rads*pHandle->fCalcAngleExecTime_s);
  
  pHandle->fFlux_Angle_rad = (float) fmod((double)pHandle->fFlux_Angle_rad , (double) (2.0f*(float)PI));
  
  pHandle->hElAngle = Convert_Rad_to16bit(pHandle->fFlux_Angle_rad);
  
}

#if (defined (CCMRAM) || defined(CCMRAM_ENABLED))
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#elif defined (__GNUC__)
__attribute__((section ("ccmram")))
#endif
#endif
/**
* @brief  Get the electrical angle for the sinusoidal voltage signals
* @param  pHandle: handler of the current instance of the V/F component
* @retval float fElAngle
*/
float ACIM_VF_GetElAngle(ACIM_VF_Handle_t *pHandle)
{
 return pHandle->fFlux_Angle_rad;
}


__weak int16_t ACIM_VF_GetMecSpeedRefUnit( ACIM_VF_Handle_t * pHandle ) {
  return (REMNG_GetValue(pHandle->pRMNGR) / pHandle->pACIM_MotorParams->bPP);
}


/**
* @brief  It configures the frequency ramp allowing rotor speed variation
* @param  pHandle: handler of the current instance of the ACIM_VF component
* @param  hTargetFinal: is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in tenths of HZ.
* @retval bool It returns true is command is valid, false otherwise
*/
bool ACIM_VF_ExecRamp(ACIM_VF_Handle_t *pHandle, int16_t hTargetFinal, uint32_t Durationms)
{
  return REMNG_ExecRamp(pHandle->pRMNGR, (hTargetFinal)*pHandle->pACIM_MotorParams->bPP,Durationms); 
}

/**
  * @brief  Check if the settled ramp has been completed.
  * @param  pHandle handler of the current instance of the ACIM_VF component.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool ACIM_VF_RampCompleted( ACIM_VF_Handle_t *pHandle )
{
  return REMNG_RampCompleted(pHandle->pRMNGR);
}

/**
  * @brief  Stop the execution of the ramp keeping the last reached value.
  * @param  pHandle handler of the current instance of the ACIM_VF component.
  * @retval none
  */
void ACIM_VF_StopRamp( ACIM_VF_Handle_t *pHandle )
{
  REMNG_StopRamp(pHandle->pRMNGR);
}

/**
* @brief  It calculates the flux frequency according to the V/f mode selected
* @param  pHandle: handler of the current instance of the ACIM_VF component
* @retval float CodeError
*/
float ACIM_VF_FluxFreqCalc(ACIM_VF_Handle_t *pHandle)
{
  if(pHandle->bVF_Mode == VF_OL)
  {  
    pHandle->fFlux_Freq_Hz = (float)REMNG_Calc(pHandle->pRMNGR)/SPEED_UNIT;
    pHandle->fFlux_Freq_rads = pHandle->fFlux_Freq_Hz*2.0f*(float)PI;
  }
  else // It's is CL
  {
    float fSpeedError_rads = 0.0;
    float fMeasSpeed_rads = (2.0f*(float)PI*(float)pHandle->SPD_real->hAvrMecSpeedUnit)/SPEED_UNIT;
    
    pHandle->fRotorFreqRef_rads = (2.0f*(float)PI*(float)REMNG_Calc(pHandle->pRMNGR))/SPEED_UNIT;
    
  pHandle->fRotorFreqRef_RPM =  (pHandle->fRotorFreqRef_rads*60.0f)/(2.0f*PI);
  
    fSpeedError_rads = pHandle->fRotorFreqRef_rads - fMeasSpeed_rads;
   
    pHandle->fSlipFrequency_rads = PI_Float_Calc(pHandle->fPI, fSpeedError_rads);
    
    pHandle->fFlux_Freq_rads = pHandle->fSlipFrequency_rads + fMeasSpeed_rads;
    pHandle->fFlux_Freq_Hz = pHandle->fFlux_Freq_rads/(2.0f*(float)PI);
  }  
    
  return pHandle->fFlux_Freq_rads;
}


       
#if (defined (CCMRAM) || defined(CCMRAM_ENABLED))
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#elif defined (__GNUC__)
__attribute__((section ("ccmram")))
#endif
#endif


/**
* @brief  It computes the voltage amplitude according to the constant ratio Vn/fn
*         and the configured frequency. It generates the sinusoidal voltages and
*         the duty cycles used for sine-triangle modulation performed by PWM component   
* @param  pHandle: handler of the current instance of the ACIM_VF component
* @retval uint16_t It returns MC_NO_FAULTS if the V/f has been ended before
*         next PWM Update event, MC_DURATION otherwise
*/

uint16_t ACIM_VF_Controller(ACIM_VF_Handle_t *pHandle)
{
  float vas, vbs, vcs;
  float fdutyA, fdutyB, fdutyC;
  
   uint16_t hCodeError;  
  float fFreqHzAbs = pHandle->fFlux_Freq_Hz;
  
  if(fFreqHzAbs<0.0f)
  {
   fFreqHzAbs = -fFreqHzAbs;
  }
  
  if(fFreqHzAbs >= pHandle->fMinFreq_TH_Hz)
  {
    pHandle->VoltageAmpl = 2.0f*PI*fFreqHzAbs * pHandle->fFlux_K;
  }
  else
  {
   pHandle->VoltageAmpl = pHandle->fVoltage_Offset;
  }
  
  float fDeg = ((pHandle->fFlux_Angle_rad)*180.0f)/PI;
  
  if(pHandle->VoltageAmpl>pHandle->fNominalPhaseVoltagePeak_V)
  {
   pHandle->VoltageAmpl = pHandle->fNominalPhaseVoltagePeak_V;
  }
  
 
  if(pHandle->VoltageAmpl > pHandle->fVoutMax_PWM_Mod_V)
  {
   pHandle->VoltageAmpl = pHandle->fVoutMax_PWM_Mod_V;
  }
  
  /* Voltage reference generator */ 
  vas = pHandle->VoltageAmpl * MCM_floatTrig_Functions(fDeg).fSin;  
  vbs = pHandle->VoltageAmpl * MCM_floatTrig_Functions(fDeg+240.0f).fSin;
  vcs = pHandle->VoltageAmpl * MCM_floatTrig_Functions(fDeg+120.0f).fSin;
   
  
  /* Normalization between 0 and 1 considering the maximum voltage available
     at the output of inverter using the sin-tri PWM modulation */
  
  fdutyA = ((vas/pHandle->fVoutMax_PWM_Mod_V)+1.0f)/2.0f;
  fdutyB = ((vbs/pHandle->fVoutMax_PWM_Mod_V)+1.0f)/2.0f;
  fdutyC = ((vcs/pHandle->fVoutMax_PWM_Mod_V)+1.0f)/2.0f;
  
  
  float fMax = 1.0f;
  
  if(fdutyA > fMax)
  {
    fdutyA = fMax;
  }
  if(fdutyB > fMax)
  {
    fdutyB = fMax;
  }
  
  if(fdutyC > fMax)
  {
    fdutyC = fMax;
  }
  
  hCodeError = PWM_SetPhasePWMOutputs(pHandle->pPWM,fdutyA,fdutyB,fdutyC);
  
  return(hCodeError);
    
}


/************************ (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
