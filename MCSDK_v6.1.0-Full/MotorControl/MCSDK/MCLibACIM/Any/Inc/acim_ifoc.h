/**
  ******************************************************************************
  * @file    acim_ifoc.h
  * @author  STMicroelectronics - SRA - System Development Unit - MC Team
  * @brief   This file provides firmware functions that implement the  features
  *          of the Rotor Flux Position Feedback component of the Motor Control SDK.
  *           - estimates the rotor flux electrical angle (IFOC)
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
#ifndef __ACIM_IFOC_H
#define __ACIM_IFOC_H

#include "mc_type.h"
#include "parameters_conversion.h"  
#include "speed_pos_fdbk.h"
#include "unit_conversions.h"
#include "acim_motor.h"
   
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup ACIM_IFOC ACIM IFOC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief This structure is used to handle the ACIM_IFOC component
  *
  */
typedef struct
{
  SpeednPosFdbk_Handle_t * SPD; /*!< The speed sensor used to perform the speed 
                                     regulation.*/
  pFOCVars_t pFOCVars;                          /*!< Pointer to FOC vars */
  
  ACIM_MotorParams_Handle_t *pACIM_MotorParams;  /*!< Pointer to ACIM Motor Params Handle */
  
  float fRotorFluxFreq_rads;					  /*!< Observed electrical rotor Flux pulsation (rad/s)*/
  float fRotorFlux_Angle_rad;                     /*!<  Observed electrical rotor Flux angle (rad) */
  
  float fCalcAngleExecFreqHz;                    /*!< Execution frequency expressed in Hz*/
  float fCalcAngleExecTime_s;				      /*!< Execution time expressed in seconds */

  
  int16_t hElAngle;                             /*!< Electrical angle used by FOC transormations */

} ACIM_IFOC_Handle_t;



/* Exported Functions ------------------------------------------------------- */

/**
  * @brief  It initializes the variables required for the rotor flux angle estimation.
  * @param  pHandle: handler of the current instance of the ACIM_IFOC component
  * @retval none
  */
void ACIM_IFOC_Init(ACIM_IFOC_Handle_t *pHandle , SpeednPosFdbk_Handle_t * oSPD);

/**
  * @brief  It performs the rotor flux angle estimation.
  * @param  pHandle: handler of the current instance of the ACIM_IFOC component
  * @retval none
  */
void ACIM_IFOC_CalcAngle(ACIM_IFOC_Handle_t *pHandle);

/**
  * @brief  It returns the flux electrical angle in digital format
  * @param  pHandle: handler of the current instance of the ACIM_IFOC component
  * @retval int16_t hElAngle
  */
int16_t ACIM_IFOC_GetElAngle(ACIM_IFOC_Handle_t *pHandle);


/**
  * @}
  */

/** @} */

#endif /*__ACIM_IFOC_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
