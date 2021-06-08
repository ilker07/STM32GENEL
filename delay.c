

#include "delay.h"
#include "stm32f4xx.h" 

extern volatile uint32_t sure;

void bekle(uint32_t us)
{

  sure=0;
	while(sure<us);
}
