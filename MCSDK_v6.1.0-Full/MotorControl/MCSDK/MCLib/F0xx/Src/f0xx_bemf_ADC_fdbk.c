/**
  ******************************************************************************
  * @file    f0xx_bemf_ADC_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement Bemf sensing
  *          class to be stantiated when the six-step sensorless driving mode
  *          topology is used.
  * 
  *          It is specifically designed for STM32F0XX
  *          microcontrollers and implements the sensing using one ADC with
  *          DMA support.
  *           + MCU peripheral and handle initialization fucntion
  *           + ADC sampling function
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "f0xx_bemf_ADC_fdbk.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup SpeednPosFdbk_Bemf
  * @{
  */
  
/**
 * @defgroup F0XX_Bemf_ADC_SpeednPosFdbk Six-Step, F0xx Sensorless with ADC Position Feedback
 *
 * @brief STM32F0, Sensorless Bemf acquisition with ADC implementation
 *
 * This component is used in applications based on an STM32F0 MCU
 * and using a sensorless position feedback.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define MAX_PSEUDO_SPEED        ((int16_t)0x7FFF)

#define PHASE_U 0u
#define PHASE_V 1u
#define PHASE_W 2u

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  It initializes ADC1, DMA and NVIC for three bemf voltages reading
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval none
  */
__weak void BADC_Init( Bemf_ADC_Handle_t *pHandle )
{

  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {

    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC( ADC1 );
    LL_ADC_ClearFlag_EOC( ADC1 );
    LL_ADC_DisableIT_EOS( ADC1 );
    LL_ADC_ClearFlag_EOS( ADC1 );

    /* ADC Calibration */
    LL_ADC_StartCalibration( ADC1 );
    while ((LL_ADC_IsCalibrationOnGoing(ADC1) == SET) ||
           (LL_ADC_REG_IsConversionOngoing(ADC1) == SET) ||
           (LL_ADC_REG_IsStopConversionOngoing(ADC1) == SET) ||
           (LL_ADC_IsDisableOngoing(ADC1) == SET))
    {
      /* wait */
    }

    /* Enables the ADC peripheral */
    LL_ADC_Enable( ADC1 );
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_1CYCLE_5);  
    /* Wait ADC Ready */
    while ( LL_ADC_IsActiveFlag_ADRDY( ADC1 ) == RESET )
    {
      /* wait */
    }

    /* DMA1 Channel1 Config */
    LL_DMA_SetMemoryAddress( DMA1, LL_DMA_CHANNEL_1, ( uint32_t )pHandle->ADC1_DMA_converted );
    LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_1, ( uint32_t )&ADC1->DR );
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1u );

    /* We allow ADC usage for regular conversion on Systick*/
    pHandle->ADCRegularLocked=false; 

    pHandle->PseudoPeriodConv = ((pHandle->LfTimerClock * 10) / ((LL_TIM_GetPrescaler(pHandle->pParams_str->LfTim)+1) * (pHandle->_Super.bElToMecRatio)));
    pHandle->pSensing_Params = &(pHandle->Pwm_OFF);
    pHandle->TimerPeriodBufferIndex = 0;
    pHandle->SpeedErrorFlag = 0;
    pHandle->IsOnSensingEnabled = false;
    pHandle->ElPeriodSum = 0;
    pHandle->ZcEvents = 0;
    pHandle->DemagCounterThreshold  = pHandle->DemagParams.DemagMinimumThreshold;
  }
}

/**
  * @brief  Reset the ADC status
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval none
  */
__weak void BADC_Clear( Bemf_ADC_Handle_t *pHandle )
{
  uint8_t i;
  /* Disabling of DMA Interrupt Event configured */
  LL_DMA_DisableIT_TC( DMA1, LL_DMA_CHANNEL_1 );

  LL_ADC_REG_StopConversion( ADC1 );

  /* Disable ADC DMA request*/
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

  /* Clear Transmission Complete Flag  of DMA1 Channel1 */
  LL_DMA_ClearFlag_TC1( DMA1 );

  /* Clear EOC */
  LL_ADC_ClearFlag_EOC( ADC1 );
  
  LL_TIM_SetCounter(pHandle->pParams_str->LfTim, 0u);
 /* We allow ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=false; 
  pHandle->ZcEvents = 0;
  pHandle->ElPeriodSum = 0;
  for ( i = 0u; i < pHandle->SpeedBufferSize; i++ )
  {
    pHandle->TimerPeriodBuffer[i] = 0;
  }
  pHandle->TimerPeriodBufferIndex = 0;
  pHandle->BufferFilled = 0;
  pHandle->IsLoopClosed=false;
  pHandle->IsAlgorithmConverged = false;
}

/**
 * @brief  It enables the speed  loop (low frequency) timer
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 * @retval none
 */
__weak void BADC_SpeedMeasureOn(Bemf_ADC_Handle_t *pHandle)
{
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->LfTim);
  LL_TIM_EnableIT_UPDATE(pHandle->pParams_str->LfTim);
  LL_TIM_EnableCounter(pHandle->pParams_str->LfTim);
}

/**
 * @brief  It disables the speed  loop (low frequency) timer
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 * @retval none
 */
__weak void BADC_SpeedMeasureOff(Bemf_ADC_Handle_t *pHandle)
{
  LL_TIM_DisableIT_UPDATE(pHandle->pParams_str->LfTim);
  LL_TIM_DisableCounter(pHandle->pParams_str->LfTim);
}

/**
 * @brief  It starts bemf ADC conversion of the phase depending on current step
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 * @param  step: current step of the six-step sequence
 * @retval none
 */
__weak void BADC_Start(Bemf_ADC_Handle_t *pHandle, uint8_t step)
{

  pHandle->ADCRegularLocked=true;

  LL_ADC_REG_StopConversion( ADC1 );  
  /* Enable ADC DMA request*/
  LL_ADC_REG_SetDMATransfer( ADC1, LL_ADC_REG_DMA_TRANSFER_LIMITED );  
  /* Enables the DMA1 Channel1 peripheral */
  LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_1 );  
  /* Clear Pending Interrupt Bits */
  LL_DMA_ClearFlag_TC1( DMA1 );
  /* DMA Interrupt Event configuration */
  LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_1 );

/* enable ADC source trigger */
  LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  LL_ADC_REG_SetTriggerEdge (ADC1, LL_ADC_REG_TRIG_EXT_FALLING);
  LL_ADC_SetSamplingTimeCommonChannels (ADC1,  LL_ADC_SAMPLINGTIME_1CYCLE_5 );
  switch (step)
  {
    case STEP_1:
    case STEP_4:         
      BADC_SelectAdcChannel(pHandle, PHASE_W);
      break;
    case STEP_2:
    case STEP_5:         
      BADC_SelectAdcChannel(pHandle, PHASE_V);
      break;
    case STEP_3:
    case STEP_6:      
      BADC_SelectAdcChannel(pHandle, PHASE_U);
      break;
    default:
      break;
  }
  LL_ADC_REG_StartConversion( ADC1 );

}

/**
 * @brief  It stops bemf ADC conversion
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 * @retval none
 */
__weak void BADC_Stop(Bemf_ADC_Handle_t *pHandle)
{
  LL_ADC_REG_StopConversion( ADC1 );
  /* Clear Transmission Complete Flag  of DMA1 Channel1 */
  LL_DMA_ClearFlag_TC1( DMA1 );

  /* Clear EOC */
  LL_ADC_ClearFlag_EOC( ADC1 );
  
 /* We allow ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=false; 
}

/**
  * @brief  Configure the ADC for the current sampling.
  *         It means set the sampling point via TIM1_Ch4 value, the ADC sequence
  *         and channels.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pHandlePWMC: handler of the current instance of the PWMC component
  * @param  pHandleSTC: handler of the current instance of the Speed Control component
  * @retval none
  */
__weak void BADC_SetSamplingPoint(Bemf_ADC_Handle_t *pHandle, PWMC_Handle_t *pHandlePWMC, SpeednTorqCtrl_Handle_t *pHandleSTC )
{
  if ((pHandleSTC->ModeDefault == MCM_SPEED_MODE) && (pHandle->DriveMode == VM))
  {
    if (!(pHandle->IsOnSensingEnabled) && (pHandlePWMC->CntPh > pHandle->OnSensingEnThres))
    {
      pHandle->IsOnSensingEnabled=true;
      pHandle->pSensing_Params = &(pHandle->Pwm_ON);
    }
    else if ((pHandle->IsOnSensingEnabled) && (pHandlePWMC->CntPh < pHandle->OnSensingDisThres))
    {
      pHandle->IsOnSensingEnabled=false;
      pHandle->pSensing_Params = &(pHandle->Pwm_OFF);
    }
    else
    {
    }
  }
  else
  {
    pHandle->IsOnSensingEnabled=false;
    pHandle->pSensing_Params = &(pHandle->Pwm_OFF);
  }	
  if (true == pHandle->pParams_str->gpio_divider_available)  LL_GPIO_ResetOutputPin( pHandle->pParams_str->bemf_divider_port, pHandle->pParams_str->bemf_divider_pin );
  PWMC_SetADCTriggerChannel( pHandlePWMC, pHandle->pSensing_Params->SamplingPoint);
}

/**
 * @brief  It gets last bemf value and checks for zero crossing detection.
 *         It updates speed loop timer and electrical angle accordingly.
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 * @param  pHandlePWMC: handler of the current instance of the PWMC component
 * @retval none
 */
__weak bool BADC_IsZcDetected( Bemf_ADC_Handle_t *pHandle, PWMC_Handle_t *pHandlePWMC)
{
  uint16_t AdcValue = pHandle->ADC1_DMA_converted[0];
  bool ZcDetection = false;
  uint32_t LfTimPeriod;
  
  LL_ADC_REG_StopConversion(ADC1);
  
  if (pHandle->BemfValues.Index >= BEMF_BUFFER_LENGTH) pHandle->BemfValues.Index = 0;
  else pHandle->BemfValues.Index++;
  switch(pHandlePWMC->Step)
  {
    case STEP_1:
    case STEP_4:
      pHandle->BemfValues.LastValues[2] = AdcValue;
      break;
    case STEP_2:
    case STEP_5:
      pHandle->BemfValues.LastValues[1] = AdcValue;
      break;
    case STEP_3:
    case STEP_6:
      pHandle->BemfValues.LastValues[0] = AdcValue;
      break;
    default:
      break;
  }
  pHandle->BemfValues.Value[pHandle->BemfValues.Index]= AdcValue;
  pHandle->DemagCounter++;
  if ( pHandle->DemagCounter >= pHandle->DemagCounterThreshold)
  {
    if (pHandle->ZcDetected == false)
    {
      switch(pHandlePWMC->Step)
      {
        case STEP_1:
          if(pHandle->hForcedDirection == 1)
          {
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] < pHandle->pSensing_Params->AdcThresholdDown)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = - S16_60_PHASE_SHIFT / 2; 
            }
          }
          else
          {
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] > pHandle->pSensing_Params->AdcThresholdUp)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2;
            }
          }
        break;
        case STEP_2:
          if(pHandle->hForcedDirection == 1)
          {     
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] > pHandle->pSensing_Params->AdcThresholdUp)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = S16_60_PHASE_SHIFT / 2; 
            }
          }
          else
          {
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] < pHandle->pSensing_Params->AdcThresholdDown)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = - S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2;         
            }
          }
        break;      
        case STEP_3:
          if(pHandle->hForcedDirection == 1)
          {
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] < pHandle->pSensing_Params->AdcThresholdDown)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2;
            }
          }
          else
          {
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] > pHandle->pSensing_Params->AdcThresholdUp)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = - S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2;       
            }
          }
        break;
        case STEP_4:
          if(pHandle->hForcedDirection == 1)
          {     
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] > pHandle->pSensing_Params->AdcThresholdUp)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2;
            }
          }
          else
          {
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] < pHandle->pSensing_Params->AdcThresholdDown)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = - S16_60_PHASE_SHIFT / 2;          
            }            
          }
        break;
        case STEP_5:
          if(pHandle->hForcedDirection == 1)
          {
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] < pHandle->pSensing_Params->AdcThresholdDown)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = - S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2;
            }
          }
          else
          {
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] > pHandle->pSensing_Params->AdcThresholdUp)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = S16_60_PHASE_SHIFT / 2;        
            }
          }
        break;
        case STEP_6:
          if(pHandle->hForcedDirection == 1)
          {     
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] > pHandle->pSensing_Params->AdcThresholdUp)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = - S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2;
            }
          }
          else
          {
            if (pHandle->BemfValues.Value[pHandle->BemfValues.Index] < pHandle->pSensing_Params->AdcThresholdDown)
            {
              ZcDetection = true;
              pHandle->MeasuredElAngle = S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2;            
            }
          }
        break;
      }
    }
    if (ZcDetection)
    {
      if (pHandle->ZcEvents > pHandle->StartUpConsistThreshold) 
      {
		pHandle->IsAlgorithmConverged = true;
      }		
      pHandle->hElAngleBase = pHandle->MeasuredElAngle;
      pHandle->hElAngleAccu = 0;
      pHandle->ZcDetected = true;
      pHandle->ZcEvents++;
      pHandle->_Super.hElAngle = pHandle->MeasuredElAngle;
      pHandle->ElPeriodBase = LL_TIM_GetCounter(pHandle->pParams_str->LfTim);
      LfTimPeriod = pHandle->ElPeriodSum + pHandle->ElPeriodBase;
      pHandle->ElPeriodSum = 0;
      if (LfTimPeriod < pHandle->MinPeriod) LfTimPeriod=pHandle->MinPeriod;
      if (true == pHandle->IsAlgorithmConverged)
      {
       if (false == pHandle->IsLoopClosed)
       {   
         LfTimPeriod = ( uint32_t )((pHandle->PseudoPeriodConv * SPEED_UNIT) / (pHandle->hVirtualMecSpeedUnit * pHandle->hForcedDirection * U_RPM));  
       }   
         LL_TIM_SetAutoReload(pHandle->pParams_str->LfTim, pHandle->ElPeriodBase + (((uint32_t)((pHandle->Zc2CommDelay) * (pHandle->hAvrTimerPeriod))) >> 9));
      }
      pHandle->TimerPeriodBuffer[pHandle->TimerPeriodBufferIndex] = LfTimPeriod ;
      pHandle->TimerPeriodBufferIndex++;
      if ( pHandle->TimerPeriodBufferIndex >= pHandle->SpeedBufferSize )
      {
        pHandle->TimerPeriodBufferIndex = 0u;
        pHandle->BufferFilled = 1;      
      }
    }
  }
  return ZcDetection;
}

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter hMecSpeedUnit - the rotor average mechanical speed,
  *         expressed in Unit. Average is computed considering a FIFO depth
  *         equal to SpeedBufferSizeUnit. Moreover it also computes and returns
  *         the reliability state of the sensor.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         Reliability_hysteresys, VariancePercentage and SpeedBufferSize
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */
__weak bool BADC_CalcAvrgMecSpeedUnit( Bemf_ADC_Handle_t * pHandle, int16_t * pMecSpeedUnit )
{
  bool bIs_Speed_Reliable = false, bAux = false;
  uint32_t wAux, wAvrPeriod = ( int32_t )0;
  int32_t wError, wAvrSquarePeriod, wAvrQuadraticError = 0;
  uint8_t i, TimerPeriodBufferIndex = pHandle->TimerPeriodBufferIndex;

  if (pHandle->BufferFilled == 1) TimerPeriodBufferIndex = pHandle->SpeedBufferSize;
  
  for ( i = 0u; i < TimerPeriodBufferIndex; i++ )
  {
    wAvrPeriod += ( int32_t )( pHandle->TimerPeriodBuffer[i] );
  }

  if (wAvrPeriod == 0)
  {	  
    pHandle->IsSpeedReliable = false;
    *pMecSpeedUnit = ( int16_t ) 0;
  }	 
  else  
  {
    wAvrPeriod = wAvrPeriod / ( int16_t )TimerPeriodBufferIndex;
    pHandle->hAvrTimerPeriod = wAvrPeriod;
    for ( i = 0u; i < TimerPeriodBufferIndex; i++ )
    {
      wError = ( int32_t )( pHandle->TimerPeriodBuffer[i] ) - wAvrPeriod;
      wError = ( wError * wError );
      wAvrQuadraticError += wError;
    }

    /*It computes the measurement variance   */
    wAvrQuadraticError = wAvrQuadraticError / ( int16_t )TimerPeriodBufferIndex;

    /* The maximum variance acceptable is here calculated as a function of average
       speed                                                                    */
    wAvrSquarePeriod = wAvrPeriod * wAvrPeriod;
    wAvrSquarePeriod = ( wAvrSquarePeriod * ( int32_t )( pHandle->VariancePercentage )) / ( int16_t )128;
  
    if ( wAvrQuadraticError < wAvrSquarePeriod )
    {
      bIs_Speed_Reliable = true;
    }

    /*Computation of Mechanical speed Unit*/
    wAux = ( int16_t )((pHandle->PseudoPeriodConv *SPEED_UNIT )/ (wAvrPeriod * U_RPM));
    if (pHandle->hForcedDirection < 0)  
    {
      pHandle->_Super.hAvrMecSpeedUnit = ( int16_t ) (-wAux);
      *pMecSpeedUnit = ( int16_t ) (-wAux);
    }
    else 
    {
      pHandle->_Super.hAvrMecSpeedUnit = ( int16_t ) (wAux);
      *pMecSpeedUnit = ( int16_t ) (wAux);
    }

    pHandle->IsSpeedReliable = bIs_Speed_Reliable;
  }
  
  
  /*Decision making*/
  if ( pHandle->IsAlgorithmConverged == false )
  {
    bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );
    pHandle->_Super.hAvrMecSpeedUnit = 0;
  }
  else
  {
    if ( pHandle->IsSpeedReliable == false  )
    {
      pHandle->ReliabilityCounter++;
      if ( pHandle->ReliabilityCounter >= pHandle->Reliability_hysteresys )
      {
        pHandle->ReliabilityCounter = 0u;
        pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
        bAux = false;
      }
      else
      {
        bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );
      }
    }
    else
    {
      pHandle->ReliabilityCounter = 0u;
      bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );
    }
  }
  bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );
  return ( bAux );
}

/**
  * @brief  Method of the class to be called when speed loop timer update event occurs
  *         It updates the electrical angle accumulator and resets zero crossing detection flag
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval none
  */
__weak void * BADC_LfTim_UP_IRQHandler( Bemf_ADC_Handle_t *pHandle )
{
  int16_t ElAngleUpdate = 0;
  uint32_t LfTimPeriod;
  LfTimPeriod = LL_TIM_GetAutoReload(pHandle->pParams_str->LfTim);
  pHandle->ElPeriodSum = LfTimPeriod - pHandle->ElPeriodBase;
  if(pHandle->ZcDetected == false)
  {
    LL_TIM_DisableIT_UPDATE(pHandle->pParams_str->LfTim);
    LL_TIM_SetAutoReload(pHandle->pParams_str->LfTim, LfTimPeriod * 130 / 100);
  }
  pHandle->ElPeriodBase = 0;
  pHandle->BemfValues.Index = 0;
  pHandle->DemagCounter = 0;
  if (pHandle->IsAlgorithmConverged)
  {
    if(pHandle->hForcedDirection == -1)
    {
      ElAngleUpdate = -S16_60_PHASE_SHIFT ;
    }
    else
    {
      ElAngleUpdate = S16_60_PHASE_SHIFT ;
    }
  }
  pHandle->hElAngleBase += ElAngleUpdate;
  pHandle->ZcDetected = false;
  LL_TIM_EnableIT_UPDATE(pHandle->pParams_str->LfTim);
  return NULL;
}

/**
  * @brief  It returns false if calculated speed is out of reliability ranges
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval bool: reliability flag
  */
__weak bool BADC_IsSpeedReliable( Bemf_ADC_Handle_t *pHandle )
{
  bool SpeedError = false;

  if ( pHandle->_Super.hAvrMecSpeedUnit > pHandle->_Super.hMaxReliableMecSpeedUnit )
  {
    SpeedError = true;
  }

  if ( pHandle->_Super.hAvrMecSpeedUnit < pHandle->_Super.hMinReliableMecSpeedUnit )
  {
    SpeedError = true;
  }

  return ( SpeedError );
}

/**
  * @brief  It returns false if calculated speed loop timer is out of reliability ranges
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval bool: reliability flag
  */
bool BADC_IsPeriodReliable(Bemf_ADC_Handle_t *pHandle)
{
  return true;
}

/**
  * @brief  forces the rotation direction
  * @param  direction: imposed direction
  */
__weak void BADC_SetDirection( Bemf_ADC_Handle_t * pHandle, uint8_t direction )
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->hForcedDirection = direction;
  }
}

/**
  * @brief  It internally performs a checks necessary to state whether
  *         the bemf algorithm converged. To be periodically called
  *         during motor open-loop ramp-up (e.g. at the same frequency of
  *         SPD_CalcElAngle), it returns true if the estimated angle and speed
  *         can be considered reliable, false otherwise
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  hForcedMecSpeedUnit Mechanical speed in 0.1Hz unit as forced by VSS
  * @retval bool sensor reliability state
  */
__weak bool BADC_IsObserverConverged( Bemf_ADC_Handle_t * pHandle )
{
  return pHandle->IsAlgorithmConverged;
}

/**
  * @brief  It configures the proper ADC channel according to the current 
  *         step corresponding to the floating phase. To be periodically called
  *         at least at every step change. 
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  Phase: Floaing phase for bemf acquisition
  * @retval none
  */
void BADC_SelectAdcChannel(Bemf_ADC_Handle_t * pHandle, uint8_t Phase)
{
  while (LL_ADC_REG_IsConversionOngoing(ADC1))
  {
    LL_ADC_REG_StopConversion(ADC1);
    while(LL_ADC_REG_IsStopConversionOngoing(ADC1));
  }
	/* Sampling time configuration */

	/* Regular sequence configuration */
  LL_ADC_REG_SetSequencerChannels(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->AdcChannel[Phase]));
  LL_ADC_ClearFlag_EOC(ADC1);
}

/**
  * @brief  This method updates the estimated electrical angle.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
__weak int16_t BADC_CalcElAngle(Bemf_ADC_Handle_t * pHandle)
{
  int16_t hRetAngle;
  pHandle->hElAngleAccu += pHandle->_Super.hElSpeedDpp;
  pHandle->_Super.hElAngle = pHandle->hElAngleBase + pHandle->hElAngleAccu;
  hRetAngle = pHandle->_Super.hElAngle;
  return hRetAngle;
}

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and update component
  *         variable hElSpeedDpp that is estimated average electrical speed
  *         expressed in dpp used for instance in observer equations.
  *         Average is computed considering a FIFO depth equal to
  *         bSpeedBufferSizedpp.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval none
  */
__weak void BADC_CalcAvrgElSpeedDpp( Bemf_ADC_Handle_t * pHandle )
{

}

/**
  * @brief  This method is used during revup phase to synchronize estimated rotor
  *         position from Bemf acquisition with imposed virtual speed  
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  hAvrMecSpeedUnit: Mechanical speed imposed by virtual speed component
  * @retval none
  */
void BADC_SetLfTimerRevUp(Bemf_ADC_Handle_t * pHandle, int16_t hAvrMecSpeedUnit)
{
  uint32_t LfTimPeriod;
  if (hAvrMecSpeedUnit == 0)
  {
    LfTimPeriod = MAX_PSEUDO_SPEED;
  }
  else
  {
    LfTimPeriod = ( uint32_t )((pHandle->PseudoPeriodConv * SPEED_UNIT) / (hAvrMecSpeedUnit * pHandle->hForcedDirection * U_RPM));  
  }	
  LL_TIM_SetAutoReload(pHandle->pParams_str->LfTim, LfTimPeriod);
  LL_TIM_GenerateEvent_UPDATE(pHandle->pParams_str->LfTim);
  pHandle->hVirtualMecSpeedUnit = hAvrMecSpeedUnit;
}

/**
  * @brief  It calculates and stores in the corresponding variable the demagnetization 
  *         time in open loop operation  
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pHandleSTC: handler of the current instance of the Speed Control component
  * @retval none
  */
void BADC_CalcRevUpDemagTime(Bemf_ADC_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pHandleSTC )
{
  int16_t hSpeed;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pHandleSTC);
  hSpeed = SPD_GetAvrgMecSpeedUnit(speedHandle);
  if (hSpeed == 0)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;;
  }	  
  else
  {
	if (hSpeed < 0)
    {
	  hSpeed = - hSpeed;
    }
    pHandle->DemagCounterThreshold = (uint16_t) (pHandle->DemagParams.RevUpDemagSpeedConv / hSpeed);		
  }
  if (pHandle->DemagCounterThreshold < pHandle->DemagParams.DemagMinimumThreshold)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }  
}

/**
  * @brief  It calculates and stores in the corresponding variable the demagnetization 
  *         time in closed loop operation  
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pHandleSTC: handler of the current instance of the Speed Control component
  * @retval none
  */
void BADC_CalcRunDemagTime(Bemf_ADC_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pHandleSTC )
{
  int16_t hSpeed;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pHandleSTC);
  hSpeed = SPD_GetAvrgMecSpeedUnit(speedHandle);
  if (hSpeed < 0) hSpeed = - hSpeed; 
  if (hSpeed < pHandle->DemagParams.DemagMinimumSpeedUnit)
  {   
    pHandle->DemagCounterThreshold = (uint16_t) (pHandle->DemagParams.RunDemagSpeedConv / hSpeed);
    if (pHandle->DemagCounterThreshold < pHandle->DemagParams.DemagMinimumThreshold)
    {
      pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
    }   
  } 
  else
  {   
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }     
}

/**
  * @brief  This method must be called after switch-over procedure when
  *         virtual speed sensor transition is ended.  
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
void BADC_SetLoopClosed(Bemf_ADC_Handle_t *pHandle)
{
  pHandle->IsLoopClosed=true;
}

/**
  * @brief  It returns the last acquired bemf value
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  phase: motor phase under investigation
  * @retval uint16_t: Bemf value
  */
uint16_t BADC_GetLastBemfValue(Bemf_ADC_Handle_t *pHandle, uint8_t phase)
{   
  return ((MC_NULL == pHandle) ? 0U : pHandle->BemfValues.LastValues[phase]);
}

/**
  * @brief  It returns the zero crossing detection flag
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval bool: zero crossing detection flag
  */
bool BADC_GetBemfZcrFlag(Bemf_ADC_Handle_t *pHandle)
{   
  return ((MC_NULL == pHandle) ? 0U : pHandle->ZcDetected);
}  

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
