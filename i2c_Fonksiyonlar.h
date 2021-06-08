
#ifndef I2C_FONKSIYONLAR__H
#define I2C_FONKSIYONLAR__H

#include "stdint.h"




void  i2cBasla(void);
void i2cYaz(uint8_t adres,uint8_t veri,uint8_t yazmaAdresi);
int I2C1_Oku(int adres,uint8_t yazmaAdresi,uint8_t okumaAdresi);
void i2cislemler(void);
void i2cDongu(void);



#endif

