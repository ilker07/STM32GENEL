

#ifndef DS18B20__H
#define DS18B20__H

#include "stdint.h"
#include "stdbool.h"

bool DS18B20_Start (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);



#endif

