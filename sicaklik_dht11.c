
#include "sicaklik_dht11.h"
#include "stm32f4xx.h" 


#define dht11Port   GPIOC
#define dht11Pin    15

#define pinDeger   (dht11Port->IDR & (1<<15))

extern uint8_t sicaklikDHT11;
extern uint8_t nem;

extern volatile uint32_t sure;



extern uint8_t geciciSicaklik;
extern uint8_t geciciNem;
extern bool dizi[40];
extern uint16_t olculenSure,zaman1,zaman2;
extern  bool bit;
extern uint8_t dusukSic,dusukNem,parity;


uint8_t sicaklikNemDHT11()
{
	
	
	 for(int a=0;a<40;a++) dizi[a]=0;
   olculenSure = 0, bit = 0; geciciSicaklik=0, geciciNem=0, dusukSic=0,dusukNem=0,parity=0;
	
	dht11Port->MODER  |= 1<<(dht11Pin*2); // C15 Output modu
	dht11Port->BSRR |=1<<(dht11Pin+16); //C15 low
  sure=0;
	while(sure<18000);
	dht11Port->BSRR |=1<<dht11Pin; //C15 high
	sure=0;
	while(sure<40);
	dht11Port->MODER  &= ~(1<<(dht11Pin*2)); //Input modu
	sure=0;
	while(pinDeger) if(sure>500) return 40;//500 us den fazla ise donguyu kirmak icin.
	
	
	sure=0;
	while(pinDeger==0) if(sure > 500) return 50;//500 us den fazla ise donguyu kirmak icin.
	zaman1 = sure;//80 us low olmali.

	sure=0;
	while(pinDeger) if(sure > 500) return 60;//500 us den fazla ise donguyu kirmak icin.
  zaman2 =sure;//80 us high olmali.


	if(!((zaman1 > 50 && zaman1 <85)) || !((zaman2 >75 && zaman2 < 100)))  //Sensor 57 ve 88 veriyor  ( 75 && mTime1 <85)
	return 70; //Degillerse fonksiyonu bitir.
	




	for(int j = 0; j < 40; j++)
	{
		
		sure=0;
		while(pinDeger==0) if(sure > 500) return 0;//50 us low olmali.
		sure=0;
		while(pinDeger) if(sure > 500) return 0;
		olculenSure =sure;

		
		if(olculenSure > 20 && olculenSure < 30)  //20 us ve 30 us arasi  ise bit 0
		{
			bit = 0;
		}
		else if(olculenSure > 60 && olculenSure < 80) //70 us ise bit 1
		{
			 bit = 1;
		}
		
	

		dizi[j] = bit;

	}
	
	nem=128 *dizi[0]+64 *dizi[1]+32 *dizi[2]+16 *dizi[3]+8 *dizi[4]+4 *dizi[5]+2 *dizi[6]+ dizi[7];
	sicaklikDHT11=128 *dizi[16]+64 *dizi[17]+32 *dizi[18]+16 *dizi[19]+8 *dizi[20]+4 *dizi[21]+2 *dizi[22]+ dizi[23];
  dusukNem=128 *dizi[8]+64 *dizi[9]+32 *dizi[10]+16 *dizi[11]+8 *dizi[12]+4 *dizi[13]+2 *dizi[14]+ dizi[15];
  dusukSic=128 *dizi[24]+64 *dizi[25]+32 *dizi[26]+16 *dizi[27]+8 *dizi[28]+4 *dizi[29]+2 *dizi[30]+ dizi[31];
  parity=128 *dizi[32]+64 *dizi[33]+32 *dizi[34]+16 *dizi[35]+8 *dizi[36]+4 *dizi[37]+2 *dizi[38]+ dizi[39];

	if(parity!=nem+sicaklikDHT11+dusukNem+dusukSic)
		return 100;
  return 1;

}

