/**
  ******************************************************************************
  * @file    acim_vf_control.h
  * @author  STMicroelectronics - SRA - System Development Unit - MC Team
  * @brief   This file provides firmware functions that implement the  features
  *          of the V/f control of Induction Motor of the Motor Control SDK.
  *          
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ACIM_VF_CONTROL_H
#define ACIM_VF_CONTROL_H


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#include "mc_type.h"
#include "parameters_conversion.h"  
#include "speed_pos_fdbk.h"
#include "bus_voltage_sensor.h"
#include "pi_float_regulator.h"
#include "ramp_ext_mngr.h"
#include "pwm.h"  
#include "unit_conversions.h"
#include "acim_motor.h"

/* Exported type definition -----------------------------------------------------------------------------------------------*/


   

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup ACIM_VF
  * @{
  */

/* Exported types ------------------------------------------------------------*/


typedef enum 
{
  VF_OL = 1,        /*!< Open Loop control   */
  VF_CL = 2         /*!< Closed Loop control */
  
}ACIM_VF_Mode_t;


/**
  * @brief This structure is used to handle the instance of the ACIM_VF component
  *
  */

typedef struct
{
  ACIM_MotorParams_Handle_t *pACIM_MotorParams;  
  PWM_Handle_t *pPWM;
   
  /* Control quantities  */
  float fNominalPhaseVoltagePeak_V;
  float VoltageAmpl;                             
  float fFlux_K;                                /*!<  Flux constant value Vs,rate/fe,rate (Wb) */
  float fVoltage_Offset;                        /*!<  Voltage offset used for frequencies lower than the fMinFreq_TH_Hz (Wb)
                                                      to compensate the stator voltage drop  */
  float fMinFreq_TH_Hz;                         /*!<  Threshold to fix the Voltage offset (Hz) */
  
  float fFlux_Freq_Hz;                          /*!<  Electrical Flux frequency (Hz)*/
  float fFlux_Freq_rads;                        /*!<  Electrical Flux speed (rad/s)*/
  float fFlux_Angle_rad;                        /*!<  Electrical Flux angle (rad)*/     
  int16_t hElAngle;                             /*!<  Electrical angle used by MCLib to perform the FOC transormations [s16 degrees]*/
    
  float fCalcAngleExecFreqHz;                   /*!<  Execution frequency [Hz] */
  float fCalcAngleExecTime_s;                   /*!<  Execution time [s] */
  
  RampExtMngr_Handle_t* pRMNGR;
  
  ACIM_VF_Mode_t  bVF_Mode;
  
  /* Closed Loop control */
  PI_Float_Handle_t* fPI;                        /*!< The PI regulatore used to perfom the rotor speed estimation */
  float fSlipFrequency_rads;
  SpeednPosFdbk_Handle_t* SPD_real;              /*!< The speed sensor used to perform the speed 
                                                      regulation.*/
  float fRotorFreqRef_rads;                      /*!< Reference rotor speed - Speed Loop (rad/s) */
  float fRotorFreqRef_RPM;                       /*!< Reference rotor speed - Speed Loop (rpm) */
  
  float fVoutMax_PWM_Mod_V;                       /*!< Maximum inverter output voltage using sin-tri PWM modulation */
  
}ACIM_VF_Handle_t;



/* Exported Functions ------------------------------------------------------- */
/**
  * @brief  It initializes the variables required for V/f control execution.
  * @param  pHandle: handler of the current instance of the ACIM_VF component
  * @retval none
  */
void ACIM_VF_Init(ACIM_VF_Handle_t *pHandle);

/**
  * @brief  It resets the variables required for V/f control execution.
  * @param  pHandle: handler of the current instance of the ACIM_VF component
  * @param  pBVS: handler of the bus voltage sensor component
  * @retval none
  */
void ACIM_VF_Clear(ACIM_VF_Handle_t *pHandle, BusVoltageSensor_Handle_t *pBVS);

/**
  * @brief  It calculates the angle of the sinusoidal electromagnetic field.
  * @param  pHandle: handler of the current instance of the ACIM_VF component
  * @retval none
  */
void ACIM_VF_CalcAngle(ACIM_VF_Handle_t *pHandle);

/**
* @brief  Get the electrical angle of the sinusoidal electromagnetic field
* @param  pHandle: handler of the current instance of the ACIM_VF component
* @retval float fElAngle
*/
float ACIM_VF_GetElAngle(ACIM_VF_Handle_t *pHandle);

/**
* @brief  It configures the frequency ramp 
* @param  pHandle: handler of the current instance of the ACIM_VF component
* @retval float fElAngle
*/
bool ACIM_VF_ExecRamp(ACIM_VF_Handle_t *pHandle, int16_t hTargetFinal, uint32_t Durationms);

/**
  * @brief  Check if the settled ramp has been completed.
  * @param  pHandle handler of the current instance of the ACIM_VF component.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
bool ACIM_VF_RampCompleted( ACIM_VF_Handle_t *pHandle );

/**
  * @brief  Stop the execution of the ramp keeping the last reached value.
  * @param  pHandle handler of the current instance of the ACIM_VF component.
  * @retval none
  */
void ACIM_VF_StopRamp( ACIM_VF_Handle_t *pHandle );

float ACIM_VF_FluxFreqCalc(ACIM_VF_Handle_t *pHandle);

uint16_t ACIM_VF_Controller(ACIM_VF_Handle_t *pHandle);

int16_t ACIM_VF_GetMecSpeedRefUnit( ACIM_VF_Handle_t * pHandle );

/**
  * @}
  */

/** @} */


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__ACIM_VF_CONTROl_H*/


/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/





