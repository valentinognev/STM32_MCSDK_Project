/**
  ******************************************************************************
  * @file    acim_lso_foc.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @brief   This file provides firmware functions that implement the  features
  *          of the Rotor Flux Position Feedback component of the Motor Control SDK.
  *           - estimates the rotor flux electrical angle and mechanical rotor speed 
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
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
#include "acim_lso_foc.h"
#include "math.h"


/* Variables ---------------------------------------------------------*/


/* Function prototypes -----------------------------------------------*/
bool ACIM_LSO_IsNotValidNumber(float fVar);





/*
* @brief  It initializes the handle
* @param  pHandle: handler of the 
* @param  
* @retval none.
*/
void ACIM_LSO_Init(ACIM_LSO_Handle_t *pHandle, BusVoltageSensor_Handle_t *pBVS)
{
 pHandle->pBVS = pBVS;
 
 PI_Float_HandleInit(pHandle->fPI);
  
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
  * @brief   
  *         
  * @param  
  * @param  
  *         
  * @retval 
  */
void ACIM_LSO_CalcAngle(ACIM_LSO_Handle_t *pHandle)
{
  
  float fSlipFreq_rads = 0.0f;
  float fRotorFluxFreq_rads = 0.0f;
  float fRotorElSpeed_obs_rads = 0.0f;
  Signal_Components fIqds_obs_A;
  Signal_Components fFlux_qdr_obs;
  
  
  Signal_Components fVqds_meas_V;
  
  float fSpeedEstError = 0.0f;
    
  /*Local variables */
  
  float k = pHandle->k;
  float k1;
  float k2;
  float k3;
  float k4;
  
  /* Motor params */
  float fsigma = pHandle->pACIM_MotorParams->fsigma;
  float ftaur  = pHandle->pACIM_MotorParams->ftaur;
  float ftaus  = pHandle->pACIM_MotorParams->ftaus;
  float fLs    = pHandle->pACIM_MotorParams->fLs;
  float fPP    = pHandle->pACIM_MotorParams->bPP;
  
  
  /* Previous values for estimated quantities */
  fRotorFluxFreq_rads    = pHandle->fRotorFluxFreq_rads;
  fRotorElSpeed_obs_rads = pHandle->fRotorElSpeed_obs_rads;
  fIqds_obs_A            = pHandle->fIqds_obs_A;
  fFlux_qdr_obs          = pHandle->fFlux_qdr_obs;
    
  fVqds_meas_V = Convert_s16_to_V(pHandle->pBVS, pHandle->pFOCVars->Vqd);
  
  Signal_Components Iqdmeas_A = Convert_s16_to_A(pHandle->pFOCVars[M1].Iqd);
  
  float ids_obs_A = fIqds_obs_A.fS_Component2;
  
  
  float divftaur = 1/ftaur;
  float divftaus = 1/ftaus;
  float divfsigma = 1/fsigma;
  float fsigmadiff = 1-fsigma;
  
  /* Online computation of Observer parameters */
  k1 = -((k-1)/fsigma)*(divftaur + divftaus);
  k2 =  (k-1)*fRotorElSpeed_obs_rads;
  
  k3 = ((k-1)/(1-fsigma))*(divftaur - k*divftaus);
  k4 = ((k-1)*fRotorElSpeed_obs_rads*fsigma)/(1-fsigma);
  
  /* STEP 1 - Computing of prediction errors --------------------------------------------------------- */
  /* Iq error(k) */
  pHandle->fIqds_error_A.fS_Component1 = Iqdmeas_A.fS_Component1 - fIqds_obs_A.fS_Component1;
  /* Id error(k) */
  pHandle->fIqds_error_A.fS_Component2 = Iqdmeas_A.fS_Component2 - fIqds_obs_A.fS_Component2;
  
  /* STEP  2 - Computing of mech. rotor speed estimation --------------------------------------------- */
  /*  SpeedEstError = -(Iq_error * Flux_dr) */
  fSpeedEstError = -(pHandle->fIqds_error_A.fS_Component1 * pHandle->fFlux_qdr_obs.fS_Component2);
  
  pHandle->fRotorElSpeed_obs_rads = PI_Float_Calc(pHandle->fPI, fSpeedEstError);
 
  
  pHandle->_SpeedEstimator.hAvrMecSpeedUnit =    (int16_t)((pHandle->fRotorElSpeed_obs_rads*10.0f)/(2.0f*(float)PI* fPP));
  pHandle->fRotorSpeed_RPM = (float)pHandle->_SpeedEstimator.hAvrMecSpeedUnit*6.0f;
  
  
  /* STEP  3 - Computing of rotor frequency */
  
  float fFlux_dr_tmp = pHandle->fFlux_qdr_obs.fS_Component2;

  if(fFlux_dr_tmp==0.0f)
  {
   fFlux_dr_tmp = pHandle->pACIM_MotorParams->fImagn_A;
  }
  
  if(ids_obs_A==0.0f)
  {
    ids_obs_A = pHandle->pACIM_MotorParams->fImagn_A;
  }
  
  /* Slip frequency estimation */
  fSlipFreq_rads = fIqds_obs_A.fS_Component1/(fFlux_dr_tmp * ftaur);
   
  /* Rotor Flux frequency computation */
  fRotorFluxFreq_rads = fSlipFreq_rads + pHandle->fRotorElSpeed_obs_rads;
  pHandle->fRotorFluxFreq_rads = fRotorFluxFreq_rads;
  /* STEP  4 - Computing and output of field angle */
  pHandle->fRotorFlux_Angle_rad +=  fRotorFluxFreq_rads*pHandle->fCalcAngleExecTime_s;
    
  /* Observed Rotor Flux angle between 0 and 2pi */
  pHandle->fRotorFlux_Angle_rad = (float) fmod((double)pHandle->fRotorFlux_Angle_rad , (double) (2.0f*(float)PI));
   
  /* STEP 5 and 6 - Prediction of currents and flux */
 
  fIqds_obs_A.fS_Component1 +=  ((-fRotorFluxFreq_rads* pHandle->fIqds_obs_A.fS_Component2) + (((-divfsigma)*(divftaus +(fsigmadiff)*divftaur))*pHandle->fIqds_obs_A.fS_Component1)
                                -((fsigmadiff)*divfsigma*fRotorElSpeed_obs_rads)*pHandle->fFlux_qdr_obs.fS_Component2 
                                +fVqds_meas_V.fS_Component1/(fsigma* fLs)
                                +k2*((pHandle->fIqds_error_A.fS_Component2)) + k1*((pHandle->fIqds_error_A.fS_Component1)))*pHandle->fCalcAngleExecTime_s;
    
  fIqds_obs_A.fS_Component2 += ((-divfsigma*(divftaus +(fsigmadiff*divftaur))*(pHandle->fIqds_obs_A.fS_Component2))+ fRotorFluxFreq_rads*pHandle->fIqds_obs_A.fS_Component1                                     
                                +((fsigmadiff)/(fsigma*ftaur))*(pHandle->fFlux_qdr_obs.fS_Component2)
                                +fVqds_meas_V.fS_Component2/(fsigma* fLs) 
                                +k1*(pHandle->fIqds_error_A.fS_Component2) - k2*(pHandle->fIqds_error_A.fS_Component1))*pHandle->fCalcAngleExecTime_s;

  fFlux_qdr_obs.fS_Component2 += ((ids_obs_A*divftaur)
                                  -(pHandle->fFlux_qdr_obs.fS_Component2*divftaur)
                                  +k3*pHandle->fIqds_error_A.fS_Component2 - k4*pHandle->fIqds_error_A.fS_Component1)*pHandle->fCalcAngleExecTime_s;
  

  pHandle->fIqds_obs_A = fIqds_obs_A;
  pHandle->fFlux_qdr_obs.fS_Component2 = fFlux_qdr_obs.fS_Component2;

  
  pHandle->hElAngle = Convert_Rad_to16bit(pHandle->fRotorFlux_Angle_rad);
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
int16_t ACIM_LSO_GetElAngle(ACIM_LSO_Handle_t *pHandle)
{
 return pHandle->hElAngle;
}



void ACIM_LSO_Clear(ACIM_LSO_Handle_t *pHandle)
{
  Signal_Components fnull = {0.0f,0.0f};
  
  
  pHandle->fFlux_qdr_obs = fnull;   
  pHandle->fIqds_error_A = fnull;
  pHandle->fIqds_obs_A = fnull;
  
  
   PI_Float_HandleInit(pHandle->fPI);
  
  pHandle->fRotorElSpeed_obs_rads = 0.0f;
  pHandle->fRotorFluxFreq_rads = 0.0f;
  pHandle->fRotorFlux_Angle_rad = 0.0f;
  
  pHandle->hElAngle = 0.0f;
  
  
  pHandle->fRotorSpeed_RPM = 0.0f;
  pHandle->_SpeedEstimator.hAvrMecSpeedUnit = 0;
  
  
  pHandle->fdbg_Vampl = 0.0f;
  pHandle->fdbg_ElAngle_rad = 0.0f;
  pHandle->fdbg_Frequency_rads = 0.0f;
  pHandle->hdbg_ElAngle = 0.0f;

  
  
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
/*
   It returns true if LSO variables are valid numerical values 
   It return false as soon as one of the LSO variables becomes Nan or INF
*/

bool ACIM_LSO_CheckIntegrity(ACIM_LSO_Handle_t *pHandle)
{
  bool bRetVal = true;
  
  if( ACIM_LSO_IsNotValidNumber(pHandle->fFlux_qdr_obs.fS_Component1)||
      ACIM_LSO_IsNotValidNumber(pHandle->fFlux_qdr_obs.fS_Component2)||  
      ACIM_LSO_IsNotValidNumber(pHandle->fIqds_obs_A.fS_Component1)||
      ACIM_LSO_IsNotValidNumber(pHandle->fIqds_obs_A.fS_Component2)||  
      ACIM_LSO_IsNotValidNumber(pHandle->fRotorElSpeed_obs_rads)||
      ACIM_LSO_IsNotValidNumber(pHandle->fRotorFluxFreq_rads)||
      ACIM_LSO_IsNotValidNumber(pHandle->fRotorFlux_Angle_rad)||  
      ACIM_LSO_IsNotValidNumber(pHandle->fRotorSpeed_RPM)||
      (pHandle->fRotorElSpeed_obs_rads >=  pHandle->fMaxObsRotorSpeed_RPM)||
      (pHandle->fRotorElSpeed_obs_rads <= -pHandle->fMaxObsRotorSpeed_RPM)  
   )
  {
    bRetVal = false;
  }
  
  
  return bRetVal;
  
}

bool ACIM_LSO_IsNotValidNumber(float fVar)
{
  bool bRetVar = false;
  
  if(isnan(fVar)||isnan(fVar))
  {
   bRetVar = true;
  }
 
    return bRetVar;
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
void ACIM_DBG_LSO_CalcAngle(ACIM_LSO_Handle_t *pHandle, float fFrequency_Hz)
{
    
  
  pHandle->fdbg_Frequency_rads = fFrequency_Hz*2.0f*PI;
  
  /* Discrete Integration */
  pHandle->fdbg_ElAngle_rad += (pHandle->fdbg_Frequency_rads * pHandle->fCalcAngleExecTime_s);

  pHandle->fdbg_ElAngle_rad = (float) fmod((double)pHandle->fdbg_ElAngle_rad , (double) (2.0f*(float)PI));

  pHandle->hdbg_ElAngle = Convert_Rad_to16bit(pHandle->fdbg_ElAngle_rad);
    
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
int16_t ACIM_DBG_LSO_GetdbgElAngle(ACIM_LSO_Handle_t *pHandle)
{ 
 return (pHandle->hdbg_ElAngle);
 
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
qd_t ACIM_DBG_LSO_CalcVoltage(ACIM_LSO_Handle_t *pHandle)
{
  qd_t Vqd = {0,0};
  float fFreq_rads_abs=0;
  
  fFreq_rads_abs = pHandle->fdbg_Frequency_rads;
  
  if(fFreq_rads_abs <0.0f)
  {
    fFreq_rads_abs = -fFreq_rads_abs;
  }	  

  if(fFreq_rads_abs < pHandle->fdbg_MinFreqTH_rads) 
  {   
    pHandle->fdbg_Vampl = pHandle->fdbg_VoltageOffset_V;
  }
  else
  {
    pHandle->fdbg_Vampl = (fFreq_rads_abs * pHandle->fdbg_Flux_K);
  }
  
  Signal_Components fVqd_V = {pHandle->fdbg_Vampl, 0.0f};
  
  Vqd = Convert_V_to_s16(pHandle->pBVS, fVqd_V);
  
  return Vqd;
}
