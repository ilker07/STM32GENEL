


#include "stm32f4xx.h"  
#include "rcc.h"


void RCC_Config()
{
	 
	RCC->CR &=~(1<<0);//hsi off yap
	
	RCC->CR |= 0x00010000;	// HSEON
	while(!(RCC->CR & 0x00020000));	// HSEON Ready Flag wait
	
	RCC->CR |=(1<<19);//css 

	RCC->PLLCFGR = 0x00402A04; 
	RCC->CR |= 0x01000000;			// PLL On
	while(!(RCC->CR & (1<<25)));	// /pll bayrak kontolu.
	
	
	RCC->CFGR |= 0x00000000;		// AHB Prescaler 1   
	RCC->CFGR |= 0x00009400;		// APB1 Prescaler 4  APB2 Prescaler 2
	
	
	FLASH->ACR = 0x00000605;  //5 ONEMLI
	RCC->CFGR |= 0x00000002; //Sistem PLL icin
	while(!(RCC->CFGR & (1<<1)));	// /pll bayrak kontolu.
	
}



