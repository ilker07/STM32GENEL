
#include "stm32f4xx.h"
#include "stdint.h"
#include "stdbool.h"

#ifndef DEGISKENLER__H
#define DEGISKENLER__H




uint8_t DS1307_WRITE=0xD0;
uint8_t DS1307_READ =0xD1;

uint8_t BMP180_WRITE=0xEE;
uint8_t BMP180_READ=0xEF;

uint8_t MPU6050_WRITE =0xD0;
uint8_t MPU6050_READ  =0xD1;

uint32_t count=0;
int16_t AC1=0;
int16_t AC2=0;
int16_t AC3=0;
uint16_t AC4=0;
uint16_t AC5=0;
uint16_t AC6=0;
int16_t	B1=0;
int16_t	B2=0;
int16_t	MB=0;
int16_t	MC=0;
int16_t	MD=0;
long	UT=0;
long UP=0;
long X1=0;
long X2=0;
long B5=0;
int sicaklikBMP180=0;
long B6=0;
long X3=0;
long B3=0;
unsigned long B4=0;
unsigned long B7=0;
long p=0;


int powerRegister=0;
int whoIam=0;

int ivmeX=0,ivmeY=0,ivmeZ=0;
int gyroX=0,gyroY=0,gyroZ=0;
float icSicaklik=0;
float aX=0,aY=0,aZ=0;
float gX=0,gY=0,gZ=0;

int16_t sicaklikMAX6675=0;
volatile uint8_t spi_RX_flag=0;


int donenAdres=0;
uint8_t dusukSicaklikBayti=0, yuksekSicaklikBayti=0;


//DHT11
uint8_t durum=0;
uint8_t sicaklikDHT11=0;
uint8_t nem=0;

uint8_t geciciSicaklik=0;
uint8_t geciciNem=0;
bool dizi[40];
uint16_t olculenSure,zaman1=0,zaman2=0;
bool bit = 0;
uint8_t dusukSic=0,dusukNem=0,parity=0;



//I2 Interrupt
//volatile uint8_t adresDeger=0;
//volatile uint8_t yazmaAdresDeger=0;
//volatile uint8_t okumaAdresDeger=0;
//volatile uint8_t i2c_flag=0;
//volatile int  sonuc=0;

#endif



