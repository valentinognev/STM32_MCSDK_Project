/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMOFANGS 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/**************** PWM INPUT **************/
extern bool UART_Input;
/* define the capturing TIMER's CLOCK and the Prescalar you are using */ 
#define TIMCLOCK   170000000
#define PSCALAR    0
#define min(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

/* Define the number of samples to be taken by the DMA 
   For lower Frequencies, keep the less number for samples 
*/
int riseMEANCaptured = 0, riseAMPCaptured = 0, riseAZIMUTHCaptured = 0;
int fallMEANCaptured = 0, fallAMPCaptured = 0, fallAZIMUTHCaptured = 0;
float frequencyMEAN = 0, frequencyAMP = 0, frequencyAZIMUTH = 0;
float widthMEAN = 0, widthAMP = 0, widthAZIMUTH = 0;
uint32_t riseDataMEAN[PWMNUMVAL], riseDataAMP[PWMNUMVAL], riseDataAZIMUTH[PWMNUMVAL];
uint32_t fallDataMEAN[PWMNUMVAL], fallDataAMP[PWMNUMVAL], fallDataAZIMUTH[PWMNUMVAL];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern EncoderReference_Handle_t EncRefM1;
extern MCI_Handle_t Mci[NBR_OF_MOTORS];

int32_t angArr[NUMOFANGS];
//int16_t angArrS[NUMOFANGS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM_IC_CaptureCallback(TIM_TypeDef *tim, const int riseCaptured, const int fallCaptured, 
                            uint32_t riseData[], uint32_t fallData[], float* frequency, float *width);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;

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
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIMAMP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if (!UART_Input)
  {
    if (LL_TIM_IsActiveFlag_CC2(TIMAMP)==1)
    {
      LL_TIM_ClearFlag_CC2(TIMAMP);
      riseAMPCaptured = 1;
    }

    // If the Interrupt is triggered by 2nd Channel
    if (LL_TIM_IsActiveFlag_CC1(TIMAMP) ==1)
    {
      LL_TIM_ClearFlag_CC1(TIMAMP);
      fallAMPCaptured = 1;
    }
    
    TIM_IC_CaptureCallback(TIMAMP, riseAMPCaptured, fallAMPCaptured, 
                          riseDataAMP, fallDataAMP, &frequencyAMP, &widthAMP);
    riseAMPCaptured = 0;
    fallAMPCaptured = 0;
  }
  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIMAZIMUTH_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	// If the Interrupt is triggered by 1st Channel
  if (!UART_Input)
  {
    if (LL_TIM_IsActiveFlag_CC1(TIMAZIMUTH)==1)
    {
      LL_TIM_ClearFlag_CC1(TIMAZIMUTH);
      riseAZIMUTHCaptured = 1;
    }

    // If the Interrupt is triggered by 2nd Channel
    if (LL_TIM_IsActiveFlag_CC2(TIMAZIMUTH) ==1)
    {
      LL_TIM_ClearFlag_CC2(TIMAZIMUTH);
      fallAZIMUTHCaptured = 1;
    }
    
    TIM_IC_CaptureCallback(TIMAZIMUTH, riseAZIMUTHCaptured, fallAZIMUTHCaptured, 
                          riseDataAZIMUTH, fallDataAZIMUTH, &frequencyAZIMUTH, &widthAZIMUTH);
    riseAZIMUTHCaptured = 0;
    fallAZIMUTHCaptured = 0;
  }
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void TIMMEAN_IRQHandler(void)
{ 
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */
	// If the Interrupt is triggered by 1st Channel
  if (!UART_Input)
  {
    if (LL_TIM_IsActiveFlag_CC1(TIMMEAN)==1)
    {
      LL_TIM_ClearFlag_CC1(TIMMEAN);
      riseMEANCaptured = 1;
    }

    // If the Interrupt is triggered by 2nd Channel
    if (LL_TIM_IsActiveFlag_CC2(TIMMEAN) ==1)
    {
      LL_TIM_ClearFlag_CC2(TIMMEAN);
      fallMEANCaptured = 1;
    }

    TIM_IC_CaptureCallback(TIMMEAN, riseMEANCaptured, fallMEANCaptured, 
                          riseDataMEAN, fallDataMEAN, &frequencyMEAN, &widthMEAN);
    riseMEANCaptured = 0;
    fallMEANCaptured = 0;
  }
  /* USER CODE END TIM8_CC_IRQn 0 */
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**   
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
 
  /* USER CODE END EXTI9_5_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) != RESET && (EncRefM1.enc_I_counter <= NUMOFANGS))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
    /* USER CODE BEGIN LL_EXTI_LINE_8 */
    if (EncRefM1.enc_I_counter < NUMOFANGS)
    {
      //angArr[EncRefM1.enc_I_counter]=STO_CR_M1._Super.hElAngle;
      angArr[EncRefM1.enc_I_counter] = TIM4->CNT; //EncRefM1.hMechAngle;
      EncRefM1.enc_I_counter++;
    }
    else
    {
      EncRefM1.enc_I_angle = 0;
      int32_t var;
      for (int16_t i=NUMOFANGS/2;i<NUMOFANGS;i++)
        EncRefM1.enc_I_angle += angArr[i];//(EncRefM1.enc_I_angle*EncRefM1.enc_I_counter+Mci[M1].pFOCVars->hElAngle)/(EncRefM1.enc_I_counter+1);
      EncRefM1.enc_I_angle /= (NUMOFANGS-NUMOFANGS/2);
      for (int16_t i=NUMOFANGS/2;i<NUMOFANGS;i++)
        var += (angArr[i]-EncRefM1.enc_I_angle)*(angArr[i]-EncRefM1.enc_I_angle);//(EncRefM1.enc_I_angle*EncRefM1.enc_I_counter+Mci[M1].pFOCVars->hElAngle)/(EncRefM1.enc_I_counter+1);
      var /= (NUMOFANGS-NUMOFANGS/2);
      // float sd = sqrt(var);
      //NVIC_DisableIRQ(M1_ENCODER_I_EXTI_IRQn);
      EncRefM1.enc_I_counter++;
    }
    /* USER CODE END LL_EXTI_LINE_8 */
  }
  else if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
    //NVIC_DisableIRQ(M1_ENCODER_I_EXTI_IRQn);
  }
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/* USER CODE BEGIN 1 */
int isMeasured = 0;

void TIM_IC_CaptureCallback(TIM_TypeDef *tim, const int riseCaptured, const int fallCaptured, 
                            uint32_t riseData[], uint32_t fallData[], float* frequency, float *width)
{
	/* Rest of the calculations will be done,
	 * once both the DMAs have finished capturing enough data */
	if ((riseCaptured) && (fallCaptured))
	{

		// calculate the reference clock
		float refClock = TIMCLOCK/(PSCALAR+1);

		int indxr = 0;
		int indxf = 0;

		int countr = 0;
		int countrf = 0;

		float riseavg = 0;
		float rfavg = 0;

		/* In case of high Frequencies, the DMA sometimes captures 0's in the beginning.
		 * increment the index until some useful data shows up
		 */
		while (riseData[indxr] == 0) indxr++;

		/* Again at very high frequencies, sometimes the values don't change
		 * So we will wait for the update among the values
		 */
		while ((min((riseData[indxr+1]-riseData[indxr]), (riseData[indxr+2]-riseData[indxr+1]))) == 0) indxr++;

		/* riseavg is the difference in the 2 consecutive rise Time */

		/* Assign a start value to riseavg */
		riseavg += min((riseData[indxr+1]-riseData[indxr]), (riseData[indxr+2]-riseData[indxr+1]));
		indxr++;
		countr++;

		/* start adding the values to the riseavg */
		while (indxr < (PWMNUMVAL))
		{
			riseavg += min((riseData[indxr+1]-riseData[indxr]), riseavg/countr);
			countr++;
			indxr++;
		}

		/* Find the average riseavg, the average time between 2 RISE */
		riseavg = riseavg/countr;

		indxr = 0;

		/* The calculation for the Falling pulse on second channel */

		/* If the fall time is lower than rise time,
		 * Then there must be some error and we will increment
		 * both, until the error is gone
		 */
		if (fallData[indxf] < riseData[indxr])
		{
			indxf+=2; indxr+=2;
			while (fallData[indxf] < riseData[indxr]) indxf++;
		}

		else if (fallData[indxf] > riseData[indxr])
		{
			indxf+=2; indxr+=2;
			while (fallData[indxf] > riseData[indxr+1]) indxr++;
		}


		/* The method used for the calculation below is as follows:
		 * If Fall time < Rise Time, increment Fall counter
		 * If Fall time - Rise Time is in between 0 and (difference between 2 Rise times), then its a success
		 * If fall time > Rise time, but is also > (difference between 2 Rise times), then increment Rise Counter
		 */
		while ((indxf < (PWMNUMVAL)) && (indxr < (PWMNUMVAL)))
		{
			/* If the Fall time is lower than rise time, increment the fall indx */
			while ((int16_t)(fallData[indxf]-riseData[indxr]) < 0)
			{
				indxf++;
			}

			/* If the Difference in fall time and rise time is >0 and less than rise average,
			 * Then we will register it as a success and increment the countrf (the number of successes)
			 */
			if (((int16_t)(fallData[indxf]-riseData[indxr]) >= 0) && (((int16_t)(fallData[indxf]-riseData[indxr]) <= riseavg)))
			{
				rfavg += min((fallData[indxf]-riseData[indxr]), (fallData[indxf+1]-riseData[indxr+1]));
				indxf++;
				indxr++;
				countrf++;
			}

			else
			{
				indxr++;
			}
		}

		/* Calculate the Average time between 2 Rise */
		rfavg = rfavg/countrf;

		/* Calculate Frequency
		 * Freq = Clock/(time taken between 2 Rise)
		 */
		*frequency = (refClock/(float)riseavg);;

		/* Width of the pulse
		 *  = (Time between Rise and fall) / clock
		 */
		*width = ((rfavg)/((float)(refClock/1000000)))*1000;   // width in ns
	}
}
/* USER CODE END 1 */
