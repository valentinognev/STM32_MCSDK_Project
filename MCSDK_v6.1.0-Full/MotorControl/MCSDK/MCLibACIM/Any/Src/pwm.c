/**
  ******************************************************************************
  * @file    pwm.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Analog Feedback component of the Motor Control SDK:
  *
  *           * regular ADC conversion execution
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
#include "pwm.h"

#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup pwm PWM 
  *
  * @brief PWM components of the Motor Control SDK
  *
  * These components fulfill two functions in a Motor Control subsystem:
  *
  * - The generation of the Pulse Width Modulation on the motor's phases
  *
  * See PWMC_Handle for more details on this mechanism.
  * @{
  */

/**
  * @brief  It initializes the PWM Timer capture compare channels to 50% of duty cycle
  *         and enables the PWM Timer capture compare channels.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval none 
  *         
  */
void PWM_Init(PWM_Handle_t *pHandle)
{
  /* Duty cycle initialized to 50% */
  pHandle->TIMx->CCR1 = pHandle->TIMx->ARR/2;
  pHandle->TIMx->CCR2 = pHandle->TIMx->ARR/2;
  pHandle->TIMx->CCR3 = pHandle->TIMx->ARR/2;
  
  /* Enable of TIMx capture compare channels: Ch1, Ch2 Ch3 */
  pHandle->TIMx->CCER |= TIMxCCER_MASK_CH123; 
  
  
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
  * @brief  It converts the duty cycles between (0 and 1) to digital format for 
  *         PWM Timer capture compare channels setting
  * @param  pHandle: handler of the current instance of the PWMC component
  * @param  float fDutyX: Duty cycle of inverter leg X:
  *         X can be: 1 for phase A, 2 for phase B, 3 for phase C    
  *        
  * @retval uint16_t It returns MC_NO_FAULTS if the V/f has been ended before
  *         next PWM Update event, MC_DURATION otherwise 
  */
uint16_t PWM_SetPhasePWMOutputs(PWM_Handle_t *pHandle, float fDuty1, float fDuty2, float fDuty3)
{

  uint16_t hAux;
  
  
  uint16_t hCCR1, hCCR2, hCCR3;
  
  /* Timer capture compare register values computation */
  hCCR1 = (uint16_t)(fDuty1*(float)(pHandle->TIMx->ARR-1));
  hCCR2 = (uint16_t)(fDuty2*(float)(pHandle->TIMx->ARR-1));
  hCCR3 = (uint16_t)(fDuty3*(float)(pHandle->TIMx->ARR-1));
  
  
  /* Store */
  pHandle->fDuty1 = fDuty1;
  pHandle->fDuty2 = fDuty2;
  pHandle->fDuty3 = fDuty3;
  
  pHandle->hCCR1 = hCCR1;
  pHandle->hCCR2 = hCCR2;
  pHandle->hCCR3 = hCCR3;
  
  
  LL_TIM_OC_SetCompareCH1 (pHandle->TIMx,hCCR1);
  LL_TIM_OC_SetCompareCH2 (pHandle->TIMx,hCCR2);
  LL_TIM_OC_SetCompareCH3 (pHandle->TIMx,hCCR3);
 

  if (LL_TIM_IsActiveFlag_UPDATE(pHandle->TIMx))
  {
    hAux = MC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  
  return hAux;
}



/**
  * @brief  It enables the output of the PWM Timer channels
  * @param  pHandle: handler of the current instance of the PWMC component    
  * @retval none
  */
void PWM_SwitchOnPWM(PWM_Handle_t *pHandle)
{

  /* Clear the TIMx Update */
  LL_TIM_ClearFlag_UPDATE(pHandle->TIMx);
  
  /* Enable the TIMx Update - it is used as HFT time base to perform the V/F conntrol */  
  LL_TIM_EnableIT_UPDATE(pHandle->TIMx);
 
  /* Main PWM Output Enable */
  pHandle->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs (pHandle->TIMx);  
  
}


/**
  * @brief  It disable the output of the PWM Timer channels
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval none
  */
void PWM_SwitchOffPWM(PWM_Handle_t *pHandle)
{
  /* Main PWM Output Disable */
  pHandle->TIMx->BDTR &= ~((uint32_t)(LL_TIM_OSSI_ENABLE));
  LL_TIM_DisableAllOutputs(pHandle->TIMx);
  
  /* Disable the TIMx Update - it is used as HFT time base to perform the V/F conntrol */ 
  LL_TIM_DisableIT_UPDATE(pHandle->TIMx);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
