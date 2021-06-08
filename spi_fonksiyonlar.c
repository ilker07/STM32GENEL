
#include "spi_fonksiyonlar.h"
#include "stdbool.h"
#include "stm32f4xx.h"
#include "delay.h"

extern volatile uint8_t spi_RX_flag;


void SPI1_IRQHandler(void)
{

if(SPI1->SR & (1<<0))//Rx bayrak kontrolu
{
 GPIOD->BSRR |=1<<13;//led yak.
 spi_RX_flag=1;
	
}

NVIC->ICER[1] =(1<<3);//tx interrupt surekli tetiklediginden interrupt kapatiliyor.
__DSB();
__ISB();

}

void spi_init()
{
 
	RCC->AHB1ENR |= 0x00000081; //GPIOA  GPIOH aktif..

	
	GPIOA->AFR[0]= 0x05500000; //AF5  PA6  PA5 
	GPIOA->MODER |=0x00002804; //PA6  PA5  Alter.Function.a1 output
	GPIOA->OSPEEDR |=0x00003C0C;//Very High Speed.A6 A5 A1 icin.
  GPIOA->OTYPER =0x00000000; //Push-Pull.
	GPIOA->PUPDR =0x00000000;  //Pull Up
	

	GPIOA->BSRR |= 1<<1; //A1 HIGH
	
	RCC->APB2ENR |= 0x00005000; //SPI1  aktif.
	SPI1->CR1 = 0x0F2C;  //2 hat receive only,16 bit ,fclk/64,master,cpol=0 cpha=0,ssm=1 ssi=1,msb first.
	SPI1->CR2 |=1<<6;//RXNEIE 
  SPI1->CR2 &=~(1<<7);//TXNEIE 
  
	
}



void interruptile(void)
{
	
	
  NVIC->ISER[1] =1<<3;//SPI1 kesmesi aktif.35
  GPIOA->BSRR |= 1<<17; //A1 LOW
	SPI1->CR1 |=1<<6; //SPE ENABLE
  //while(spi_RX_flag==0);
	
	

}
void interruptsiz()
{
  GPIOA->BSRR |= 1<<17; //A1 LOW
	SPI1->CR1 |=1<<6; //SPE ENABLE
	while(!(SPI1->SR & 0x0001)); //RXNE =1 olana kadar bekle.
	
}


uint16_t veriAl()
{
	
  //interruptile();
	uint16_t veri=0;
	veri=(uint16_t)SPI1->DR;
	veri=veri>>3;
	veri=veri/4; 
	bekle(10000);
	SPI1->CR1 &=~(1<<6); //SPE DISABLE
	GPIOA->BSRR |= 1<<1; //A1 HIGH
	
	return veri;
}
