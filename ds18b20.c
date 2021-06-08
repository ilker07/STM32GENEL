




#include "stm32f4xx.h"  
#include "ds18b20.h"
#include "delay.h"
#include "stdbool.h"

#define port        GPIOC
#define pin         1
#define pinDeger   (port->IDR & (1<<pin))
extern volatile uint32_t sure;

void pinOutput()
{
  port->MODER |= 1<<(pin*2);
	port->MODER &=~(1<<(pin*2+1));
}
void pinInput()
{
  port->MODER &= ~(1<<(pin*2));
	port->MODER &=~(1<<(pin*2+1));
}


bool DS18B20_Start (void)
{
	bool cevap = 0;
  pinOutput();  
	port->BSRR |=1<<(pin+16); //B1 low 
	bekle(480); //480 us

  pinInput();  
	bekle(80); //80 us

	if (pinDeger==0) cevap = 1; 
	else cevap = 0;

	bekle(400);//400 us

	return cevap;
}

void DS18B20_Write (uint8_t data)
{
	
	pinOutput();

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  //Bit 1 ise
		{
			

			
			  pinOutput();
		   	port->BSRR |=1<<(pin+16); //B1 low 
			  bekle (1);  // 1 us
			  pinInput();
			  bekle (45);  // 45 us
		}

		else  //Bit 0 ise
		{
			

			  pinOutput();
			  port->BSRR |=1<<(pin+16); //B1 low 
			  bekle (60);  //60 us
			  pinInput();
			  bekle(5);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	pinInput();
	
	for (int i=0;i<8;i++)
	{
		
		pinOutput();
		port->BSRR |=1<<(16+pin); //B1 low 
		bekle(2);  // 1us den fazla .
    pinInput();
		bekle(5);
		if (pinDeger) 
		{
			value |= 1<<i;  
		}
		bekle(50);  //50 us
	}
	return value;
}


