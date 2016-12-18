#pragma once
#include "stm32f1xx_hal.h"

#define CONFIG_REGISTER 1
#define DATA_REGISTER   0

//#define ADS1115_ONBOARD_ADR 146 //(0x48 << 1) //(1001000) // original: 146
#define ADS1115_REMOTE_ADR  (0x48 << 1) // = 144

void ads111x_select(unsigned char);

unsigned short ads111x_read();
unsigned short ads111x_read_IT();

HAL_StatusTypeDef ads111x_write_pointer(  unsigned char RegisterAddress);
void ads111x_write_rr(unsigned short Data, unsigned char RegisterAddress);
void ads111x_write(unsigned short data);


#define ads111x_write(data) ads111x_write_rr(data,1)
