/**
  ******************************************************************************
  * @file    pwm.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component of the Motor Control SDK
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
#ifndef __PWM_H
#define __PWM_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

   
#define TIMxCCER_MASK_CH123        ((uint32_t)  0x00000555u)   
   
   
/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm PWM
  * @{
  */

typedef struct
{
  
  PWMC_Handle_t _Super;                 /*!< Offset of current sensing network  */
  
  TIM_TypeDef*  TIMx;                   /*!< It contains the pointer to the timer 
                                            used for PWM generation. It must 
                                            equal to TIM1 if bInstanceNbr is 
                                            equal to 1, to TIM8 otherwise */
  /* Regular conversion --------------------------------------------------------*/
  ADC_TypeDef * regconvADCx;            /*!< ADC peripheral used for regular 
                                           conversion.*/
  
  uint8_t  bRepetitionCounter;          /*!< It expresses the number of PWM 
                                            periods to be elapsed before compare 
                                            registers are updated again. In 
                                            particular: 
                                            RepetitionCounter= (2* #PWM periods)-1*/
  float fDuty1;
  float fDuty2;
  float fDuty3;

  uint16_t hCCR1;
  uint16_t hCCR2;  
  uint16_t hCCR3;  


}PWM_Handle_t;


/**
  * @brief  It initializes the PWM Timer capture compare channels to 50% of duty cycle
  *         and enables the PWM Timer capture compare channels.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval none 
  *         
  */
void PWM_Init(PWM_Handle_t *pHandle);

/**
  * @brief  It converts the duty cycles between (0 and 1) to digital format for 
  *         PWM Timer capture compare channels setting
  * @param  pHandle: handler of the current instance of the PWMC component
  * @param  float fDutyX: Duty cycle of inverter leg X:
  *         X can be: 1 for phase A, 2 for phase B, 3 for phase C    
  *        
  * @retval uint16_t It returns MC_NO_FAULTS if the V/f has been ended before
  *         next PWM Update event, MC_DURATION otherwise 
  */

uint16_t PWM_SetPhasePWMOutputs(PWM_Handle_t *pHandle, float fDuty1, float fDuty2, float fDuty3);


/**
  * @brief  It enables the output of the PWM Timer channels
  * @param  pHandle: handler of the current instance of the PWMC component    
  * @retval none
  */
void PWM_SwitchOnPWM(PWM_Handle_t *pHandle);
    

/**
  * @brief  It disable the output of the PWM Timer channels
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval none
  */
void PWM_SwitchOffPWM(PWM_Handle_t *pHandle);



/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __PWMNCURRFDBK_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
