#ifndef __PI_FLOAT_REGULATOR_H
#define __PI_FLOAT_REGULATOR_H


#include "mc_type.h"


typedef struct
{
  
  float   fDefKpGain;           /**< Default @f$K_{pg}@f$ gain */
  float   fDefKiGain;           /**< Default @f$K_{ig}@f$ gain */
  
  float fKpGain;
  float fKiGain;
  float fIntegralTerm;

  
  float fUpperIntegralLimit;
  float fLowerIntegralLimit;
  
  float fLowerLimit;
  float fUpperLimit; 
  
  float fKs;
  float fAntiWindTerm;
  
  FunctionalState bAntiWindUpActivation;
  float fExecFrequencyHz;
  
} PI_Float_Handle_t;

//typedef struct
//{
//  float fKp;
//  float fKi;
//  float fKs;
//  FunctionalState AntiWindupEnable; 
//  float fLowerLimit;
//  float fUpperLimit;
//  float fExecFrequencyHz;
//}FloatPI_StructInit_t;


void PI_Float_HandleInit(PI_Float_Handle_t *pHandle);
float PI_Float_Calc(PI_Float_Handle_t *pHandle, float fProcessVarError);

#endif /* __PI_FLOAT_REGULATOR_H */
