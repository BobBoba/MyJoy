/////////////////////////////////////////////////////////////////////////
// File : d:/Lisp/Production/Ads111x/Ads1115/Windows/Ads1115.h
// 
//
// Generated on the 20/07/2012 22:02 by the 'super-cool' code generator 
//
// Generator written in Common Lisp, created by  Remi PRUD'HOMME 
// with the help of : 
//
/////////////////////////////////////////////////////////////////////////
//  Copyright  2012
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
//  Reference document : 
/////////////////////////////////////////////////////////////////////////

// This file is generated. Don't modify it 

#ifndef D__LISP_PRODUCTION_ADS111X_ADS1115_WINDOWS_ADS1115_H
#define D__LISP_PRODUCTION_ADS111X_ADS1115_WINDOWS_ADS1115_H

#include "Ads1115_types.h"
#include "stm32f1xx_hal.h"

uint16_t SetDataRegister( uint16_t Param );
uint16_t GetDataRegister();
uint16_t SetHighThresholdRegister( uint16_t Param );
uint16_t GetHighThresholdRegister();
uint16_t SetLowThresholdRegister( uint16_t Param );
uint16_t GetLowThresholdRegister();
uint16_t SetConfigurationRegister( uint16_t Param );
uint16_t GetConfigurationRegister();
uint16_t SetComparatorQueue( enum ComparatorQueue Param );
uint16_t GetComparatorQueue();
uint16_t SetLatchingComparator( enum LatchingComparator Param );
uint16_t GetLatchingComparator();
uint16_t SetComparatorPolarity( enum ComparatorPolarity Param );
uint16_t GetComparatorPolarity();
uint16_t SetComparatorMode( enum ComparatorMode Param );
uint16_t GetComparatorMode();
uint16_t SetDataRate( enum DataRate Param );
uint16_t GetDataRate();
uint16_t SetMode( enum Mode Param );
uint16_t GetMode();
uint16_t SetProgrammableGain( enum ProgrammableGain Param );
uint16_t GetProgrammableGain();
uint16_t SetInputMultiplexer( enum InputMultiplexer Param );
uint16_t GetInputMultiplexer();
uint16_t SetConversionStatus( enum ConversionStatus Param );
uint16_t GetConversionStatus();
uint16_t SetConversionStart( enum ConversionStart Param );
uint16_t GetConversionStart();
uint16_t SetPointerRegister( uint16_t Param );
uint16_t GetPointerRegister();


#ifdef __cplusplus
}
#endif

#endif
