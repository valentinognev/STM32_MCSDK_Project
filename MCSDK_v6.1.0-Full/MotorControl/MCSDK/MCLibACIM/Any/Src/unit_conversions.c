/**
  ******************************************************************************
  * @file    unit_conversions.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of units conversion of the Motor Control SDK.
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
#include "unit_conversions.h"
#include "arm_math.h"

#define RAD3DIV3        0.577350269f

#if (defined (CCMRAM) || defined(CCMRAM_ENABLED))
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#elif defined (__GNUC__)
__attribute__((section ("ccmram")))
#endif
#endif
int16_t Convert_Rad_to16bit(float fAngle_rad)
{
  int16_t hElAngle;
       
  hElAngle =  (int16_t) ((fAngle_rad*32767.0f)/(float)PI);
  
  return (hElAngle);
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
Signal_Components Convert_s16_to_A(qd_t hIqd)
{
  float fConvfactor = (float)ADC_REFERENCE_VOLTAGE/(65536.0f*(float)RSHUNT*(float)AMPLIFICATION_GAIN);
  
  Signal_Components  fIqdA;
 
  fIqdA.fS_Component1 = (float)hIqd.q*fConvfactor;
  fIqdA.fS_Component2 = (float)hIqd.d*fConvfactor;
  
  return fIqdA;
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
qd_t Convert_A_to_s16(Signal_Components fIqdA)
{
  float fConvfactor = (float)ADC_REFERENCE_VOLTAGE/(65536.0f*(float)RSHUNT*(float)AMPLIFICATION_GAIN);
  
  qd_t hIqd;
 
  hIqd.q  = (int16_t)(fIqdA.fS_Component1/fConvfactor);
  hIqd.d  = (int16_t)(fIqdA.fS_Component2/fConvfactor);
  
  return hIqd;
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
Signal_Components Convert_s16_to_V(BusVoltageSensor_Handle_t *pBVS, qd_t hVqd)
{
  float fConvfactor = ((float)RAD3DIV3* (float)VBS_GetAvBusVoltage_V(pBVS))/32767.0f;
  
  Signal_Components  fVqd_V;
 
  fVqd_V.fS_Component1 = (float)hVqd.q*fConvfactor;
  fVqd_V.fS_Component2 = (float)hVqd.d*fConvfactor;
  
  return fVqd_V;
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
qd_t Convert_V_to_s16(BusVoltageSensor_Handle_t *pBVS, Signal_Components fVqd_V)
{
  float fConvfactor = 32767.0f/((float)RAD3DIV3* (float)VBS_GetAvBusVoltage_V(pBVS));
  
  qd_t hVqd;
  
  float ftempVq = (fVqd_V.fS_Component1*fConvfactor);
  float ftempVd = (fVqd_V.fS_Component2*fConvfactor);
  
  if(ftempVq >32767.0f)
  {
    ftempVq = 32767.0f;
  }
  
  if(ftempVd >32767.0f)
  {
    ftempVd = 32767.0f;
  }
    
  hVqd.q = (int16_t)ftempVq;    
  hVqd.d = (int16_t)ftempVd;  

  return hVqd;
}
















