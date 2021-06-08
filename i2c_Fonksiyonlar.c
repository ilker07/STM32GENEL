
#include "stm32f4xx.h"
#include "i2c_Fonksiyonlar.h"
#include "stdbool.h"

/*
volatile bool yazOku=0;
volatile int adresDeger=0;
volatile uint8_t yazmaAdresDeger=0;
volatile uint8_t okumaAdresDeger=0;
volatile int temp=0;

volatile uint8_t i2c_flag=0;
volatile int sonuc=0;

volatile  bool girdiMi=0;

void I2C1_EV_IRQHandler(void)
{
    
	if(I2C1->SR1 & (1<<7)) //TXE
	{
		GPIOD->ODR = 0x00001000;
		I2C1->CR1 |= 0x0100; // START biti.
		return;
	}
	if(I2C1->SR1 & (1<<6)) //RXNE
	{
		GPIOD->ODR = 0x00002000;
		I2C1->CR1 |= 0x0200; // STOP biti.
		sonuc=I2C1->DR;
		i2c_flag=1;
		return;
		
	}

	if(I2C1->SR1 & (1<<1)) //ADDR
	{
		GPIOD->ODR = 0x00008000;
		
		
		temp = I2C1->SR2; // Bayrak temizle.
		if(girdiMi==0)
		{
      I2C1->DR = 0x6B; // Register adresi.
			girdiMi=1;
		}
		
		return;
	}
	if(I2C1->SR1 & (1<<0)) //SB
	{
		GPIOD->ODR = 0x00000020;
		if(yazOku==0)
		{
		  yazOku=1;
			I2C1->DR = 0xD0;
		}
		
		else if(yazOku==1)
		{
		  yazOku=0;
			I2C1->DR =0xD1;
		}
		return;
	}
	
	

}

*/


void  i2cBasla()
	
{

RCC->AHB1ENR |= 0x00000002; //GPIOB aktif..
RCC->APB1ENR |= 0x00200000; //I2C1  aktif.
GPIOB->AFR[0]= 0x44000000; //PB7 AF4 ayari ve PB6 AF4 ayari.
GPIOB->MODER |= 0x0000A000; //PB7 ve PB6 Alter.Function
GPIOB->OTYPER =0x000000C0; //PB7 ve PB6 Open-Drain.
	

I2C1->CR1 |=1<<15;
I2C1->CR1 &=~(1<<15);

/*	
adresDeger=0x6B;
yazmaAdresDeger=0xD0;
okumaAdresDeger=0xD1;
	

I2C1->CR2  |=1<<9;//ITEVTEN
I2C1->CR2  |=1<<10;	//ITBUFEN	
NVIC->ISER[0] = 1<<31;	//I2C1 Interrupt.
*/

I2C1->CR2 |= 0x0008 ; //8mhz 
I2C1->CCR = 0x0028;  //i2c 100khz.
I2C1->TRISE=0x09;// Maximum yukselme zamani(Datasheette yaziyor) 
I2C1->CR1=0x0001;// I2C aktif.



}

void i2cYaz(uint8_t adres,uint8_t veri,uint8_t yazmaAdresi)
{
 I2C1->CR1 |= 0x0100; // START biti
 while (!(I2C1->SR1 & 0x0001)); // SB=1 olmasini bekle.
 I2C1->DR = yazmaAdresi; // Slave adresi.
 while (!(I2C1->SR1 & 0x0002)); // ADDR=1 olmasini bekle.
 while (!(I2C1->SR2 & 0x0001));  //Master
 int Status2 = I2C1->SR2; // Bayrak temizle.
 I2C1->DR = adres; // Register adresi.
 while (!(I2C1->SR1 & 0x0080)); // TXE=1 olmasini bekle.
 I2C1->DR = veri; //Veriyi gonder.
 while (!(I2C1->SR1 & 0x0080)); // TXE=1 olmasini bekle.
 while (!(I2C1->SR1 & 0x0004)); // BTF=1 olmasini bekle.
 I2C1->CR1 |= 0x0200; // STOP biti.
 
	
}

int I2C1_Oku(int adres,uint8_t yazmaAdresi,uint8_t okumaAdresi) { 

	
 I2C1->CR1 |= 0x0100; // START biti.
 while (!(I2C1->SR1 & 0x0001)) {}; // SB=1 olmasini bekle.
 I2C1->DR = yazmaAdresi; // Slave Adresi.(Yazma)
 while (!(I2C1->SR1 & 0x0002)); // ADDR=1 olmasini bekle.
 int temp = I2C1->SR2; // Bayrak temizle.
 I2C1->DR = adres; // Register adresi.
 while (!(I2C1->SR1 & 0x0080)) {}; //TXE=1 olmasini bekle.
 while (!(I2C1->SR1 & 0x0004)) {}; // BTF=1 olmasini bekle.

 I2C1->CR1 |= 0x0100; // START biti.
 while (!(I2C1->SR1 & 0x0001)) {}; // SB=1 olmasini bekle.
 I2C1->DR = okumaAdresi; // Slave Adresi.(Okuma)
 while (!(I2C1->SR1 & 0x0002)) {}; // ADDR=1 olmasini bekle.
 temp= I2C1->SR2; // Bayrak temizle.
 	 
 while (!(I2C1->SR1 & 0x0040)) {}; // RxNE=1 olmasini bekle.
 I2C1->CR1 |= 0x0200; // STOP biti.

 return I2C1->DR; // gelen veriyi al. 
	 
}








