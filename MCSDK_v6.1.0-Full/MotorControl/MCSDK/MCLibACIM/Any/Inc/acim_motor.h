/**
  ******************************************************************************
  * @file    acim_motor.h
  * @author  Motion Control Team, ST Microelectronics
  * @brief   This file provides definition and functions related to a generic
  *          AC induction motor
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
#ifndef __ACIM_MOTOR_H
#define __ACIM_MOTOR_H

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup ACIM_MOTOR ACIM Motor
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief This structure is used to handle the ACIM_MOTOR component
  *
  */

typedef struct
{
  /* Motor parameters - Steady state model */
  float fRs;                    /*!< Stator resistance, (Ohm) */
  float fRr;                    /*!< Rotor resistance referred to stator equivalent circuit, (Ohm)*/
  float fLls;                   /*!< Stator leakage inductance, (H)*/
  float fLlr;                   /*!< Rotor leakage  inductance referred to the stator equivalent circuit, (H)*/
  float fLms;                   /*!< Motor Magnetizing inductance ((Steady-state model), (H) */        
  uint8_t bPP;                  /*!< Motor Pole Pairs*/
  
  /* Dynamic model's parameters after applying the 
     Reference Frame Theory transformation */
  float fLM;                    /*!< Motor Magnetizing inductance (Dynamic model), (H) */        
  float fLr;                    /*!< Global rotor inductance (Dynamic model), Lr = Llr + LM, (H)*/
  float fLs;                    /*!< Global stator inductance (Dynamic model), Ls = Lls + LM, (H)*/
  float ftaur;                  /*!< Rotor Time constant, Lr/Rr, (s) */
  float ftaus;                  /*!< Statot Time constant, Ls/Rs, (s) */
  float fsigma;                 /*!< Total leakage factor, (dimensionless)*/

  float fImagn_A;              /*!<   Magnetizing Current (No-Load current) expressed in Ampere */
  float fNominalFrequency_rads;      
} ACIM_MotorParams_Handle_t;


/**
  * @}
  */

/** @} */

#endif /*__ACIM_MOTOR_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
