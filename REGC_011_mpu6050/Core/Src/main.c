#include "stm32g0xx.h"
#include "stm32g031xx.h"
#include <stdio.h>
#define MPU6050_ADDRESS      0x68

#define MPU6050_WHO_AM_I     0x75
#define MPU6050_POWER_MGMT_1 0x6B

//ACCELEROMETER REGISTER ADDRESS
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C

#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E

#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

//GYRO REGISTER ADDRESS
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44

#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46

#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48


void GPIO_Init(void);
void I2C_Config(void);
uint8_t I2C_Read(uint8_t,uint8_t);

void GPIO_Init(void){
	/* Enable GPIOA */
	RCC->IOPENR |=(1U<<0);
	// Select AF from Moder for PA9,PA10 (10)
	GPIOA->MODER &= ~(3U<<2*9);
	GPIOA->MODER |= (2U<<2*9);
	GPIOA->OTYPER|=(1U<<9);
	GPIOA->OSPEEDR|=(3U<<18);//high speed
	GPIOA->PUPDR|=(1U<<18);
	GPIOA->AFR[1] |= (6U<< 4);//for PA9,PA10 AF6(I2C)

	GPIOA->MODER &= ~(3U<<2*10);
	GPIOA->MODER |= (2U<<2*10);
	GPIOA->OTYPER|=(1U<<10);
	GPIOA->OSPEEDR|=(3U<<20);//high speed
	GPIOA->PUPDR|=(1U<<20);
	GPIOA->AFR[1] |= (6U<<8);//for PA9,PA10 AF6(I2C)

}

void I2C_Config(void){
	//Enable the I2C Clock and GPIO Clock
	RCC->APBENR1|= (1U << 21);//enable IC2 clock
	// Enables GPIOA peripheral
	RCC->AHBENR |= 1;
	I2C1->CR1 |=0;


	I2C1->TIMINGR|=(3<<28);//PRESC
	I2C1->TIMINGR|=(0x13<<0);//SCLL
	I2C1->TIMINGR|=(0xF<<8);//SCLH
	I2C1->TIMINGR|=(0x2<<16);//SDADEL
	I2C1->TIMINGR|=(0x4<<20);//SCLDEL

	I2C1->CR1=(1U<<0);//PE

	NVIC_SetPriority(I2C1_IRQn,1);
	NVIC_EnableIRQ(I2C1_IRQn);

}

uint8_t I2C_Read(uint8_t devAddr,uint8_t regAddr){

	//WRITE OPERTAION(Send address and register to read)
	I2C1->CR2=0;
	I2C1->CR2|=((uint32_t)devAddr<<1);//slave address
	I2C1->CR2|=(1U<<16);//Number of bytes
    I2C1->CR2|=(1U<<13);//Generate Start
    while(!(I2C1->ISR&(1<<1)));//TXIS

    I2C1->TXDR=(uint32_t)regAddr;
    while(!(I2C1->ISR&(1<<6)));//TC

    //READ OPERATION(read data)
    I2C1->CR2=0;
    I2C1->CR2|=((uint32_t)devAddr<<1);//slave address
    I2C1->CR2|=(1U<<10);//READ mode
    I2C1->CR2|=(1U<<16);//Number of bytes
	I2C1->CR2|=(1U<<15);//NACK
	I2C1->CR2|=(1U<<25);//AUTOEND

	I2C1->CR2|=(1U<<13);//Generate Start
    while (!(I2C1->ISR & (1<<2)));//wait until RXNE=1

    uint8_t data=(uint8_t)I2C1->RXDR;
    return data;
}


void I2C_Write(uint8_t devAddr,uint8_t regAddr,uint8_t data){

	//WRITE OPERTAION(Send address and register to read)
	I2C1->CR2=0;
	I2C1->CR2|=((uint32_t)devAddr<<1);//slave address
	I2C1->CR2|=(2U<<16);//Number of bytes
	I2C1->CR2|=(1U<<25);//AUTOEND
    I2C1->CR2|=(1U<<13);//Generate Start

    while(!(I2C1->ISR&(1<<1)));//TXIS
    I2C1->TXDR=(uint32_t)regAddr;

    while(!(I2C1->ISR&(1<<1)));//TXIS
    I2C1->TXDR=(uint32_t)data;
}

void delay_ms(uint32_t ms)
{
	for (uint32_t i = 0; i < ms * 1000; i++)
	{// only wait
	}
}

uint16_t data;
uint16_t accelx;
uint16_t gyrox;
uint16_t accely;
uint16_t gyroy;
uint16_t accelz;
uint16_t gyroz;


int main(void){
	I2C_Config();
	GPIO_Init();

	data=I2C_Read(MPU6050_ADDRESS,MPU6050_WHO_AM_I);
	data=I2C_Read(MPU6050_ADDRESS,MPU6050_POWER_MGMT_1);
	I2C_Write(MPU6050_ADDRESS,MPU6050_POWER_MGMT_1,0x00);
	delay_ms(200);
	data=I2C_Read(MPU6050_ADDRESS,MPU6050_POWER_MGMT_1);

	for(;;){

			accelx=I2C_Read(MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_L);
			accelx=accelx|(I2C_Read(MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_H)<<8);
			accelx=accelx/16384;

			gyrox=I2C_Read(MPU6050_ADDRESS,MPU6050_GYRO_XOUT_L);
			gyrox=gyrox|(I2C_Read(MPU6050_ADDRESS,MPU6050_GYRO_XOUT_H)<<8);
			gyrox=gyrox/131;

			accely=I2C_Read(MPU6050_ADDRESS,MPU6050_ACCEL_YOUT_L);
			accely=accely|(I2C_Read(MPU6050_ADDRESS,MPU6050_ACCEL_YOUT_H)<<8);
			accely=accely/16384;

			gyroy=I2C_Read(MPU6050_ADDRESS,MPU6050_GYRO_YOUT_L);
			gyroy=gyroy|(I2C_Read(MPU6050_ADDRESS,MPU6050_GYRO_YOUT_H)<<8);
			gyroy=gyroy/131;

			accelz=I2C_Read(MPU6050_ADDRESS,MPU6050_ACCEL_ZOUT_L);
			accelz=accelz|(I2C_Read(MPU6050_ADDRESS,MPU6050_ACCEL_ZOUT_H)<<8);
			accelz=accelz/16384;

			gyroz=I2C_Read(MPU6050_ADDRESS,MPU6050_GYRO_ZOUT_L);
			gyroz=gyroz|(I2C_Read(MPU6050_ADDRESS,MPU6050_GYRO_ZOUT_H)<<8);
			gyroz=gyroz/131;
			delay_ms(80);

}
	while(1){

	}
}
