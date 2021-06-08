
#include "stm32f4xx.h"                  // Device header
#include "stdbool.h"
#include  "math.h"
#include "rcc.h"
#include "i2c_Fonksiyonlar.h"
#include "spi_fonksiyonlar.h"
#include "ds18b20.h"
#include "sicaklik_dht11.h"
#include "timer.h"
#include "delay.h"
#include "degiskenler.h"


void sicaklikNemBMP180(void);
void sicaklikDS18B20(void);
void ivmeMPU6050(void);
void interruptMAX6675(void);
void DMA_Ayar(void);
void ADC_Config(void);
void DMA_I2CAyar(void);
void I2C_RX_DMA(uint8_t reg_adres,uint8_t slave_yazma_adresi,uint8_t slave_okuma_adresi);

volatile uint16_t msn=0;
volatile uint32_t ledSure=0;
volatile uint32_t sure=0;
volatile uint8_t sayi=0;
bool interrupCagrildi=0;
volatile bool sureDoldu=0;


uint8_t familyCode=0;
uint8_t seriNumber1=0;//Opsiyonel
uint8_t seriNumber2=0;//Opsiyonel
uint8_t seriNumber3=0;//Opsiyonel
uint8_t seriNumber4=0;//Opsiyonel
uint8_t seriNumber5=0;//Opsiyonel
uint8_t seriNumber6=0;//Opsiyonel
uint8_t crcNumber=0;//Opsiyonel
uint8_t thRegister=0;//Opsiyonel
uint8_t tlRegister=0;//Opsiyonel
uint8_t configRegister=0;//Opsiyonel
uint8_t reserved1=0;//Opsiyonel
uint8_t reserved2=0;//Opsiyonel
uint8_t reserved3=0;//Opsiyonel
uint8_t CRCBayt=0;//Opsiyonel
float floatsicaklikDS18B20=0;


uint8_t dmadizi[1];
uint8_t dmadeger =0;
uint32_t adc;
uint32_t adc1[8];

volatile uint32_t gir=0;
volatile bool bitti=0;

void DMA1_Stream0_IRQHandler()
{
	gir++;
  
  if((DMA1->LISR & (1<<5)))
	{
	  GPIOD ->BSRR |=1<<15;
		bitti=1;
	  DMA1->LIFCR = 1<<5; 
    DMA1_Stream0 ->CR &=~(1<<0); //DMA Pasif.
	  I2C1->CR1 |= 0x0200; // STOP biti.
	}


}







void I2C_RX_DMA(uint8_t reg_adres,uint8_t slave_yazma_adresi,uint8_t slave_okuma_adresi)
{
 I2C1->CR1 |= 0x0100; // START biti.
 while (!(I2C1->SR1 & 0x0001)) {}; // SB=1 olmasini bekle.
 I2C1->DR = slave_yazma_adresi; // Slave Adresi.(Yazma)
 while (!(I2C1->SR1 & 0x0002)); // ADDR=1 olmasini bekle.
 int temp = I2C1->SR2; // Bayrak temizle.
 I2C1->DR = reg_adres; // Register adresi.
 while (!(I2C1->SR1 & 0x0080)) {}; //TXE=1 olmasini bekle.
 while (!(I2C1->SR1 & 0x0004)) {}; // BTF=1 olmasini bekle.

 I2C1->CR1 |= 0x0100; // START biti.
 while (!(I2C1->SR1 & 0x0001)) {}; // SB=1 olmasini bekle.
 I2C1->DR = slave_okuma_adresi; // Slave Adresi.(Okuma)
 while (!(I2C1->SR1 & 0x0002)) {}; // ADDR=1 olmasini bekle.
 temp= I2C1->SR2; // Bayrak temizle.
 
 DMA1_Stream0 ->CR |=1<<0; //DMA Aktif.
	 
	
	 
	 /*
	//DMA Interrupt yokken asagidakiler . 
 while((DMA1->LISR & (1<<5))==0);//stream 0
 DMA1->LIFCR = 1<<5; 
 DMA1_Stream0 ->CR &=~(1<<0); //DMA Pasif.
 	 
 I2C1->CR1 |= 0x0200; // STOP biti.*/

}

void interruptMAX6675()
{
  	if(interrupCagrildi==0)
		{
			 interrupCagrildi=1;
			 interruptile();
		}
		if(spi_RX_flag==1)
		{
			sicaklikMAX6675=veriAl();
			spi_RX_flag=0;
			interrupCagrildi=0;
			
		}

}
void ivmeMPU6050()
{
    ivmeX=(int16_t)(I2C1_Oku(0x3C,MPU6050_WRITE,MPU6050_READ)+256*I2C1_Oku(0x3B,MPU6050_WRITE,MPU6050_READ));
		ivmeY=(int16_t)(I2C1_Oku(0x3E,MPU6050_WRITE,MPU6050_READ)+256*I2C1_Oku(0x3D,MPU6050_WRITE,MPU6050_READ));
		ivmeZ=(int16_t)(I2C1_Oku(0x40,MPU6050_WRITE,MPU6050_READ)+256*I2C1_Oku(0x3F,MPU6050_WRITE,MPU6050_READ));
		
		gyroX=(int16_t)(I2C1_Oku(0x44,MPU6050_WRITE,MPU6050_READ)+256*I2C1_Oku(0x43,MPU6050_WRITE,MPU6050_READ));
		gyroY=(int16_t)(I2C1_Oku(0x46,MPU6050_WRITE,MPU6050_READ)+256*I2C1_Oku(0x45,MPU6050_WRITE,MPU6050_READ));
		gyroZ=(int16_t)(I2C1_Oku(0x48,MPU6050_WRITE,MPU6050_READ)+256*I2C1_Oku(0x47,MPU6050_WRITE,MPU6050_READ));
		
		icSicaklik=(int16_t)(I2C1_Oku(0x42,MPU6050_WRITE,MPU6050_READ)+256*I2C1_Oku(0x41,MPU6050_WRITE,MPU6050_READ));
		icSicaklik=icSicaklik/340;
		icSicaklik +=36.53;
		
		aX=ivmeX/16384.0;
		aY=ivmeY/16384.0;
		aZ=ivmeZ/16384.0;
		
		gX=gyroX/131.0;
		gY=gyroY/131.0;
		gZ=gyroZ/131.0;

}


void sicaklikNemBMP180()
{
    i2cYaz(0xF4,0x2E,BMP180_WRITE);//Datasheet Sayfa 15
		bekle(5000);//4.5ms
		UT=(I2C1_Oku(0xF6,BMP180_WRITE,BMP180_READ)<<8)+I2C1_Oku(0xF7,BMP180_WRITE,BMP180_READ);
		
		
		i2cYaz(0xF4,(0x34+(1<<6)),BMP180_WRITE); //Oss 1 
		bekle(8000);//7.5 ms
		UP=(((I2C1_Oku(0xF6,BMP180_WRITE,BMP180_READ)<<16)+(I2C1_Oku(0xF7,BMP180_WRITE,BMP180_READ)<<8)+(I2C1_Oku(0xF8,BMP180_WRITE,BMP180_READ)))>>(8-1));
		X1=((UT-AC6)*AC5)/32768;
		X2=(MC*2048)/(X1+MD);
		B5=X1+X2;
		sicaklikBMP180=(B5+8)/16;
		sicaklikBMP180=sicaklikBMP180/10;//Gerçek sicaklik degeri.BMP180
		

}

void sicaklikDS18B20()
{
  if(!DS18B20_Start())
	return;
  DS18B20_Write (0x33);//Skip ROM komutu.(64 bit doner ilk 8 bit 28h)
	familyCode=DS18B20_Read();
	if(familyCode!=40)
		return;
	seriNumber1=DS18B20_Read();//Opsiyonel
	seriNumber2=DS18B20_Read();//Opsiyonel
	seriNumber3=DS18B20_Read();//Opsiyonel
	seriNumber4=DS18B20_Read();//Opsiyonel
	seriNumber5=DS18B20_Read();//Opsiyonel
	seriNumber6=DS18B20_Read();//Opsiyonel
	crcNumber=DS18B20_Read();//Opsiyonel
	
	if(!DS18B20_Start())
	  return;

	 DS18B20_Write (0xCC);  // skip ROM komutu
	 DS18B20_Write (0x44);  // Sicaklik donusum komutu.
	 bekle(1000000);//750 ms diyor.1 sn verdim.

	if(!DS18B20_Start())
		return;
		  
	DS18B20_Write (0xCC);  // skip ROM komutu.
	DS18B20_Write (0xBE);  // Read Scratch-pad komutu.
		 
	dusukSicaklikBayti = DS18B20_Read();
	yuksekSicaklikBayti = DS18B20_Read();
	thRegister=DS18B20_Read();//Opsiyonel
	tlRegister=DS18B20_Read();//Opsiyonel
	configRegister=DS18B20_Read();//Opsiyonel 127 okumaliyiz default deger
	reserved1=DS18B20_Read();//Opsiyonel 255 okumaliyiz default deger(0xFF)
	reserved2=DS18B20_Read();//Opsiyonel
	reserved3=DS18B20_Read();//Opsiyonel 16 okumaliyiz defaul deger(0x10)
	CRCBayt=DS18B20_Read();//Opsiyonel
	
	
	
	uint8_t dusukDortBit=dusukSicaklikBayti & 0x0F;
	floatsicaklikDS18B20=((yuksekSicaklikBayti<<8)|dusukSicaklikBayti)>>4;
	
	uint8_t sayac=1;
	uint8_t deger=0;
	float toplam=0;
	
	for (;sayac<5;sayac++)
	{
	   deger =dusukDortBit>>(4-sayac);
		 deger =deger & 0x01;
		 float us_alma=0;
		 us_alma=pow(2,(-1*sayac)) * deger;
		 toplam +=us_alma;
	
	}
	floatsicaklikDS18B20 +=toplam;
	
	
	
}





void TIM1_UP_TIM10_IRQHandler(void)  // 1 uslik timer
{  
	
		if (TIM1->SR & (1<<0))  //UIF Bayragi mi
  {
			
			sure++;
		
			if(sure>1000000)  //Flag bayragi 1 sn icin.
			{
				  sure=0;
				 
			}
		
      TIM1->SR &=~(1<<0); //UIF temizlendi.
  }
	
}

void TIM2_IRQHandler(void)
{
   	if (TIM2->SR & (1<<0))  //UIF Bayragi mi
  {
			
			msn++;
		 
		  ledSure++;
		
		  if(ledSure>=1000)
			{
			
			  GPIOD->BSRR |=1<<5;
				GPIOD->BSRR |=1<<12;
			}
			if(ledSure>=2000)
			{
			  GPIOD->BSRR |=1<<21;
			  GPIOD->BSRR |=1<<28;
				ledSure=0;
			}
		
		
			if(msn>=1000)  //Flag bayragi 1 sn icin.
			{
				 sureDoldu=1;
			}
			if(msn>=65000)  
			{
				 msn=0;
			}
			
		  
		
      TIM2->SR &=~(1<<0); //UIF temizlendi.
  }


}


void ADC_Config()
{
	
	/*
	RCC->APB2ENR |= 0x00000100;	// ADC1 Clock enable

	ADC->CCR 	|= 1 << 17;		// ADC Clock Divided By 8
	ADC->CCR 	|= 1 << 16;		// ADC Clock Divided By 8

	ADC1->CR1	|= 1 << 8;		// Scan conversion mode enable
	ADC1->CR2   |= 1 << 0;		// ADC enable
	ADC1->CR2   |= 1 << 1;		// Continuous conversion mode enable
	ADC1->CR2   |= 1 << 8;		// DMA Enable
	ADC1->CR2   |= 1 << 9;		// DDS
	ADC1->CR2   |= 1 << 10;		// EOCS
	
	*/
	GPIOA->MODER |= 0x00000003;	// Pin 0 Analog
	RCC->APB2ENR |= 0x00000100;	// ADC1 Clock enable

	ADC->CCR 	|= 1 << 16;		// ADC Clock Divided By 4
	//ADC1->SMPR2 |= 6 << 0;	// 144 Cycles for Channel 0
	ADC1->CR1 	|= 0 << 24;		// ADC Resolution 12 bit
	ADC1->CR1	|= 1 << 8;		// Scan conversion mode enable
	ADC1->CR2   |= 1 << 0;		// ADC enable
	ADC1->CR2   |= 1 << 1;		// Continuous conversion mode enable
	ADC1->CR2   |= 1 << 8;		// DMA Enable
	ADC1->CR2   |= 1 << 9;		// DDS
	ADC1->CR2   |= 1 << 10;		// EOCS
	//ADC1->CR2   |= 1 << 30;	//
	ADC1->SQR1 	|= 0 << 20;		// L = 1 conversion number
	ADC1->SQR3  |= 0 << 0;		// put channel number CH0


}


void DMA_I2CAyar()
{
  
	RCC->AHB1ENR |=1<<21;//DMA1 Clock.
	I2C1->CR2  |=1<<11;//dma
	//while((DMA1->LISR & (1<<5))==0);//stream 0
	//while((DMA1->HISR & (1<<21))==0);//stream 6
	//DMA1->LIFCR = 1<<5; 
	//DMA1->HIFCR = 1<<21; //TEMIZLEME
  //dma1 ch1 stream0 rx
 //dma1 ch1 stream6 tx
	
	DMA1_Stream0 ->CR &=~(1<<0); //EN
	while((DMA1_Stream0->CR & (1<<0))==1);
	DMA1_Stream0 ->PAR =(uint32_t)&I2C1->DR;
	DMA1_Stream0 ->M0AR =(uint32_t)&dmadizi;
	DMA1_Stream0 ->NDTR =1;//1 tane veri.
	DMA1_Stream0 ->CR |=1<<25; //Kanal 1.
	DMA1_Stream0 ->CR |=1<<16; //Priority very high.
	DMA1_Stream0 ->CR |=1<<17; //Priority very high.
	
	NVIC->ISER[0] |=1<<11;
	
	//DMA1_Stream0 ->CR |=1<<12; //32bit. 14 13 m--  12 11 p
	//DMA1_Stream0 ->CR |=1<<14;//32 BIT.
	DMA1_Stream0 ->CR |=1<<10; //MINC
	DMA1_Stream0 ->CR |=1<<8; //CIRC
	DMA1_Stream0 ->CR |=1<<4; //TCIE
	//DMA1_Stream0 ->CR |=1<<0; //EN
	

}

void DMA_Ayar()
{
	RCC->AHB1ENR |= 0x00400000;			// RCC->AHB1ENR |= (1<<22); // DMA2 clk Enable

	while((DMA2_Stream4->CR & 0x00000001) == 1);

	DMA2_Stream4->PAR|= (uint32_t) &ADC1->DR;
	DMA2_Stream4->M0AR |= (uint32_t) &adc1;
	DMA2_Stream4->NDTR = 1;
	DMA2_Stream4->CR |= 1 << 8;		  // Circular mode
	DMA2_Stream4->CR |= 1 << 10;		// memory incremented
	DMA2_Stream4->CR |= 2 << 11;		// peripheral data size 32 bit (word)
	DMA2_Stream4->CR |= 2 << 13;		// memory data size 32 bit (word)
	DMA2_Stream4->CR |= 2 << 16;		// priority level high
	DMA2_Stream4->CR |= 0 << 25;		// channel 0 selected
	DMA2_Stream4->CR |= 1 << 0;			// start stream 4
	
}




int main()
	
{
	
	RCC_Config();
	SystemCoreClockUpdate();
	
	//D12 Ayarlari
	RCC->AHB1ENR |=1<<3;  //D Clock Aktif.
	
	GPIOD->MODER  = 0x55000400; //Output modu
	GPIOD->OTYPER = 0x00000000; //Push-Pull
	GPIOD->OSPEEDR =0xAA000000; //Yuksek hiz
	GPIOD->BSRR =1<<28; //D12 low
	
	
	
	RCC->AHB1ENR |=1<<2;  //C Clock Aktif.
	
	
	GPIOC->BSRR =1<<1; //C1 high
	GPIOC->BSRR =1<<17; //C1 low
	
	GPIOC->MODER  |= 1<<30; // C15 Output modu
	GPIOC->OTYPER = 0x00000000; //Push-Pull
	
	GPIOC->BSRR =1<<15; //C15 high
	GPIOC->BSRR =1<<31; //C15 low
	RCC->AHB1ENR |= 0x00000081; //GPIOA  GPIOH aktif..
	
	
	
	i2cBasla();
	//spi_init();
	
	
	
	ADC_Config();
  DMA_Ayar();
	ADC1->CR2   |= 1 << 30;	//adc baslat.
	
	DMA_I2CAyar();
	tim1_Ayar();
	
	
	while(donenAdres!=0x55)  //Cihaz Adresi.
	{
	donenAdres=I2C1_Oku(0xD0,BMP180_WRITE,BMP180_READ);
	}
	
	//I2C DMA
  I2C_RX_DMA(0xD0,BMP180_WRITE,BMP180_READ);
	while(bitti==0);
	
	
	AC1=256*I2C1_Oku(0xAA,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xAB,BMP180_WRITE,BMP180_READ);
	AC2=256*I2C1_Oku(0xAC,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xAD,BMP180_WRITE,BMP180_READ);
	AC3=256*I2C1_Oku(0xAE,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xAF,BMP180_WRITE,BMP180_READ);
	AC4=256*I2C1_Oku(0xB0,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xB1,BMP180_WRITE,BMP180_READ);
	AC5=256*I2C1_Oku(0xB2,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xB3,BMP180_WRITE,BMP180_READ);
	AC6=256*I2C1_Oku(0xB4,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xB5,BMP180_WRITE,BMP180_READ);
	B1=256*I2C1_Oku(0xB6,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xB7,BMP180_WRITE,BMP180_READ);
	B2=256*I2C1_Oku(0xB8,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xB9,BMP180_WRITE,BMP180_READ);
	MB=256*I2C1_Oku(0xBA,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xBB,BMP180_WRITE,BMP180_READ);
	MC=256*I2C1_Oku(0xBC,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xBD,BMP180_WRITE,BMP180_READ);
	MD=256*I2C1_Oku(0xBE,BMP180_WRITE,BMP180_READ)+I2C1_Oku(0xBF,BMP180_WRITE,BMP180_READ);
  
	 
	
	
	
	//powerRegister=I2C1_Oku(0x6B,MPU6050_WRITE,MPU6050_READ);
	//whoIam=I2C1_Oku(0x75,MPU6050_WRITE,MPU6050_READ);
	
	


 
	
	
	
	
	//i2cYaz(0x6B,0x00,MPU6050_WRITE);//Uyku modundan cikarmak icin.
	//i2cYaz(0x19,0x07,MPU6050_WRITE);//1khz accelometre icin.
 
	



while(1)
{
  
	
	
	if(sureDoldu)
	{
		
		
	 
	 // interruptMAX6675();
		sicaklikNemBMP180();
		//ivmeMPU6050();
		//durum=sicaklikNemDHT11();//DHT11 durum 1 ise basarili.
		//sicaklikDS18B20();//DS18B20 Verisi
		
		//adc = adc1[0];
		dmadeger=dmadizi[0];
	  msn=0;
		sureDoldu=0;
	}

}

}



  



