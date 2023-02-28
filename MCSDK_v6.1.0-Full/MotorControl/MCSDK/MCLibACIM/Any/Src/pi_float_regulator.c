/**
 ******************************************************************************
 * @file    pid_regulator.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the following features
 *          of the PID regulator component of the Motor Control SDK:
 *
 *           * proportional, integral and derivative computation funcions
 *           * read and write gain functions
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

#include "pi_float_regulator.h"



/** @addtogroup MCSDK
 * @{
 */

/**
 * @defgroup PI Float Regulator PI Float Regulator
 * @brief PI Float regulator component of the Motor Control SDK
 *
 * The PI float regulator component implements the following control function:
 *
 * @f[
 * u(t) = K_{p} e(t) + K_{i} \int_0^t e(\tau) \,d\tau
 * @f]
 *
 *
 * @{
 */

/**
 * @brief  It initializes the handle
 * @param  pHandle: handler of the current instance of the PID component
 * @retval None
 */



void PI_Float_HandleInit(PI_Float_Handle_t *pHandle)
{
  pHandle->fKpGain = pHandle->fDefKpGain;
  pHandle->fKiGain = pHandle->fDefKiGain;
  
  pHandle->fIntegralTerm = 0.0f;
  pHandle->fAntiWindTerm = 0.0f;  
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
  * @brief  This function computes the output of a PI regulator sum of its 
  *         proportional and integral terms
  * @param  Float_PIReg PI regulator object
  * @param  float Actual process variable error, intended as the reference 
  *         value minus the actual process variable value
  * @retval float PI output
  */
float PI_Float_Calc(PI_Float_Handle_t *pHandle, float fProcessVarError)
{
  float fProportional_Term, fOutput,fOutputSat;
  
    /* Proportional term computation*/
  fProportional_Term = pHandle->fKpGain * fProcessVarError;
  fOutput =   fProportional_Term;


  /* Integral term computation */
  if (pHandle->fKiGain == 0.0f)
  {
    pHandle->fIntegralTerm = 0.0f;
  }
  else
  { 
//    pHandle->fIntegralTerm+= ((pHandle->fKiGain *(fProcessVarError)) - (pHandle->fKiGain*pHandle->fAntiWindTerm));
    pHandle->fIntegralTerm+= ((pHandle->fKiGain *(fProcessVarError))/pHandle->fExecFrequencyHz);
    
    if(pHandle->fIntegralTerm > pHandle->fUpperIntegralLimit)
    {
      pHandle->fIntegralTerm =  pHandle->fUpperIntegralLimit;
    }
    else if(pHandle->fIntegralTerm < pHandle->fLowerIntegralLimit)
    {
      pHandle->fIntegralTerm =  pHandle->fLowerIntegralLimit;
    }
    
    fOutput += pHandle->fIntegralTerm;
  }
   
  fOutputSat = fOutput;
  
  /* Saturation */
  if (fOutput > pHandle->fUpperLimit)
  {
    fOutputSat = pHandle->fUpperLimit;   
  } 
  
  if (fOutput < pHandle->fLowerLimit)
  {
    fOutputSat = pHandle->fLowerLimit;
  }
  
  if(pHandle->bAntiWindUpActivation == ((FunctionalState)ENABLE))
  {
    pHandle->fAntiWindTerm = (fOutput - fOutputSat) * pHandle->fKs;
  }
  else
  {
    pHandle->fAntiWindTerm =0.0f;
  }
  
  fOutput = fOutputSat;
  
  return(fOutput); 	
}
