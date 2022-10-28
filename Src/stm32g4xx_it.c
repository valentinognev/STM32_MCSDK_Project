/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mc_type.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMOFANGS 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern EncoderReference_Handle_t EncRefM1;
extern MCI_Handle_t Mci[NBR_OF_MOTORS];
extern STO_CR_Handle_t STO_CR_M1;
extern STO_PLL_Handle_t STO_PLL_M1;

int32_t angArr[NUMOFANGS];
//int16_t angArrS[NUMOFANGS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
 
  /* USER CODE END EXTI9_5_IRQn 0 */
  // if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) != RESET && (EncRefM1.enc_I_counter <= NUMOFANGS))
  // {
  //   LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
  //   /* USER CODE BEGIN LL_EXTI_LINE_8 */
  //   if (EncRefM1.enc_I_counter < NUMOFANGS)
  //   {
  //     //angArr[EncRefM1.enc_I_counter]=STO_CR_M1._Super.hElAngle;
  //     angArr[EncRefM1.enc_I_counter]=STO_PLL_M1._Super.hElAngle;
  //     EncRefM1.enc_I_counter++;
  //   }
  //   else
  //   {
  //     EncRefM1.enc_I_angle = 0;
  //     int32_t var;
  //     for (int16_t i=NUMOFANGS/2;i<NUMOFANGS;i++)
  //       EncRefM1.enc_I_angle += angArr[i];//(EncRefM1.enc_I_angle*EncRefM1.enc_I_counter+Mci[M1].pFOCVars->hElAngle)/(EncRefM1.enc_I_counter+1);
  //     EncRefM1.enc_I_angle /= (NUMOFANGS-NUMOFANGS/2);
  //     for (int16_t i=NUMOFANGS/2;i<NUMOFANGS;i++)
  //       var += (angArr[i]-EncRefM1.enc_I_angle)*(angArr[i]-EncRefM1.enc_I_angle);//(EncRefM1.enc_I_angle*EncRefM1.enc_I_counter+Mci[M1].pFOCVars->hElAngle)/(EncRefM1.enc_I_counter+1);
  //     var /= (NUMOFANGS-NUMOFANGS/2);
  //     // float sd = sqrt(var);
  //     NVIC_DisableIRQ(M1_ENCODER_I_EXTI_IRQn);
  //     EncRefM1.enc_I_counter++;
  //   }
  //   /* USER CODE END LL_EXTI_LINE_8 */
  // }
  // else 
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
    //NVIC_DisableIRQ(M1_ENCODER_I_EXTI_IRQn);
  }
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */

 /* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
  if (LL_TIM_IsEnabledIT_UPDATE (TIM4) != 0U)
  {
    if (LL_TIM_IsActiveFlag_UPDATE (TIM4) != 0U)
    {
      LL_TIM_ClearFlag_UPDATE(TIM4);
//      (void)ENC_IRQHandler(&ENCODER_M1);
      /* USER CODE BEGIN M1 ENCODER_Update */

      /* USER CODE END M1 ENCODER_Update   */
    }
    else
    {
      /* No other IT to manage for encoder config */
    }
  }
  else
  {
    /* No other IT to manage for encoder config */
  }
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
