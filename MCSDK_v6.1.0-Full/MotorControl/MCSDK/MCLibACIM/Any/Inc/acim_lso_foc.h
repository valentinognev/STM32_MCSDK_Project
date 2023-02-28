/**
  ******************************************************************************
  * @file    acim_lso_foc.h
  * @author  STMicroelectronics - SRA - System Development Unit - MC Team
  * @brief   This file provides firmware functions that implement the  features
  *          of the Rotor Flux Position Feedback component of the Motor Control SDK.
  *           - estimates the rotor flux electrical angle and mechanical rotor speed
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
#ifndef ACIM_LSO_FOC_H
#define ACIM_LSO_FOC_H

#include "mc_type.h"
#include "parameters_conversion.h"     
#include "speed_pos_fdbk.h"
#include "bus_voltage_sensor.h"
#include "pi_float_regulator.h"
#include "unit_conversions.h"
#include "acim_motor.h"

#define MAX_SPEED_RPM_MARGIN  1.5f           /* LSO PI regulator Output saturation margin (50% Max Application Speed) */    

#define PI_LSO_MAX_OUT        (((float)MOTOR_MAX_SPEED_RPM*MAX_SPEED_RPM_MARGIN*2.0f*PI*POLE_PAIR_NUM)/60.0f)       //rad/s
   
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup ACIM_LSO-FOC ACIM LSO FOC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief This structure is used to handle the ACIM_LSO-FOC component
  *
  */
typedef struct
{
  SpeednPosFdbk_Handle_t _SpeedEstimator;       /*!< The speed sensor used to perform the speed 
                                                      regulation.*/
  BusVoltageSensor_Handle_t *pBVS;              /*!< Pointer to BusVoltageSensor Handle */
  
  pFOCVars_t pFOCVars;                          /*!< Pointer to FOC vars */
  
  ACIM_MotorParams_Handle_t *pACIM_MotorParams; /*!< Pointer to ACIM Motor Params Handle */
  
  PI_Float_Handle_t* fPI;                       /*!< The PI regulator used to perfom the rotor speed estimation */
  
  Signal_Components fIqds_error_A;              /*!< Current error. Iqds_error_A = Iqd_meas_A - Iqds_obs_A [A]*/
  
  Signal_Components fIqds_obs_A;                /*!< Observed quantities - States of State Model */
  Signal_Components fFlux_qdr_obs;              /*!<  Rotor flux observed divided by magnetizing inductance, LM. [Vs/H]
                                                      It corresponds to the magnetizing rotor current [A]  */
  
  float fRotorFluxFreq_rads;                    /*!<  Observed Electrical Rotor Flux speed (rad/s)*/
  float fRotorFlux_Angle_rad;                   /*!<  Observed electrical rotor Flux angle [rad]*/     
  int16_t hElAngle;                             /*!<  Electrical angle used by MCLib to perform the FOC transormations [s16 degrees]*/
  
  float fRotorElSpeed_obs_rads;                 /*!<  Observed electrical Rotor speed  (rad/s) */
  float fRotorSpeed_RPM;                        /*!<  Observed mechanical Rotor speed  (rpm) */
  float fMaxObsRotorSpeed_RPM;                  /*!<  Maximum Observed mechanical Rotor speed  (rpm) */        
  
  float k;                                      /*!< Common parameter for Luenberger's Matrix coeffients computation*/
  
  float fCalcAngleExecFreqHz;                   /*!< Execution frequency expressed in Hz */
  float fCalcAngleExecTime_s;                   /*!< Execution time expressed in seconds */
  
  /* LSO debug variables */
  
  float fdbg_ElAngle_rad;                       /*!< Angle used for performing V/f open loop control (rad)*/
  int16_t hdbg_ElAngle;                         /*!< Angle used for performing V/f open loop control (s16)*/
  float fdbg_Frequency_rads;                    /*!< Forced flux frequency (rad/s) */
  float fdbg_Flux_K;                            /*!< Flux constant value Vs,rate/fe,rate (Wb) */
  float fdbg_Vampl;                             /*!< Voltage amplitude computed by V/f (Vpeak) */
  float fdbg_VoltageOffset_V;                   /*!< */
  float fdbg_MinFreqTH_rads;                    /*!< */
} ACIM_LSO_Handle_t;



/* Exported Functions ------------------------------------------------------- */

/**
  * @brief  It initializes the variables required for the rotor flux angle and rotor speed estimation.
  * @param  pHandle: handler of the current instance of the ACIM_LSO component
  * @retval none
  */
void ACIM_LSO_Init(ACIM_LSO_Handle_t *pHandle, BusVoltageSensor_Handle_t *pBVS);

/**
  * @brief  It clears State Observer object by re-initializing private variables.
  * @param  pHandle: handler of the current instance of the ACIM_LSO component
  * @retval none
  */
void ACIM_LSO_Clear(ACIM_LSO_Handle_t *pHandle);

/**
  * @brief  It executes Luenberger state observer and computes a new speed 
  *         estimation and update the estimated electrical angle.
  * @param  pHandle: handler of the current instance of the ACIM_LSO component
  * @retval none
  */
void ACIM_LSO_CalcAngle(ACIM_LSO_Handle_t *pHandle);


/**
  * @brief  It returns the flux electrical angle in digital format
  * @param  pHandle: handler of the current instance of the ACIM_LSO component
  * @retval int16_t hElAngle
  */
int16_t ACIM_LSO_GetElAngle(ACIM_LSO_Handle_t *pHandle);

/**
  * @brief  It checks the validity of LSO variables 
  * @param  pHandle: handler of the current instance of the ACIM_LSO component
  * @retval bool: true or false
  */
bool ACIM_LSO_CheckIntegrity(ACIM_LSO_Handle_t *pHandle);

/**
  * @brief  It performs the voltage calculation used by LSO-Debug V/f control
  * @param  pHandle: handler of the current instance of the ACIM_LSO component
  * @retval Volt_Components Vqd
  */
qd_t ACIM_DBG_LSO_CalcVoltage(ACIM_LSO_Handle_t *pHandle);

/**
  * @brief  It returns the flux electrical angle in digital format computed by 
  *         LSO-Debug V/f control
  * @param  pHandle: handler of the current instance of the ACIM_LSO component
  * @retval int16_t hElAngle
  */
int16_t ACIM_DBG_LSO_GetdbgElAngle(ACIM_LSO_Handle_t *pHandle);

/**
  * @brief  It calculates the angle to perform the LSO-Debug V/f control
  * @param  pHandle: handler of the current instance of the ACIM_LSO component
  * @retval none
  */
void ACIM_DBG_LSO_CalcAngle(ACIM_LSO_Handle_t *pHandle, float fFrequency_Hz);


/**
  * @}
  */

/** @} */

#endif /*ACIM_LSO_FOC_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
