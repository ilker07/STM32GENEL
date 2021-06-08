


#include "timer.h"
#include "stm32f4xx.h"

void tim1_Ayar()
{
	
	//Timer hatti 168 mhz de.
	
	
	
	RCC->APB2ENR |= 1<<0;  //TIM1   Aktif.

	NVIC->ISER[0] |=1<<25;//TIM1 kesmesi aktif.
	
	TIM1->PSC =6;  //0.5 sn icin psc:41999  ; 1us periyod icin psc=6 ;1 ms icin psc=69
	TIM1->ARR =23;  //0.5 sn icin arr:1999 ; 1us periyod icin  arr=23;1 ms icin psc=2399
	TIM1->CR1 |= 1<<2; //URS Aktif.
	TIM1->DIER |=1<<0;//UIE  Aktif.
	TIM1->CR1 |=1<<0;//CEN  Aktif.
	
	// 84 000  000  / (999+1) (83+1)  84 000 
	
	
	RCC->APB1ENR |= 1<<0;  //TIM2   Aktif.
	NVIC->ISER[0] |=1<<28;//TIM2 kesmesi aktif.
	
	TIM2->PSC =83;  //0.5 sn icin psc:41999  ; 1us periyod icin psc=6 ;1 ms icin psc=69
	TIM2->ARR =999;  //0.5 sn icin arr:1999 ; 1us periyod icin  arr=23;1 ms icin psc=2399
	TIM2->CR1 |= 1<<2; //URS Aktif.
	TIM2->DIER |=1<<0;//UIE  Aktif.
	TIM2->CR1 |=1<<0;//CEN  Aktif.
	
	   
}



