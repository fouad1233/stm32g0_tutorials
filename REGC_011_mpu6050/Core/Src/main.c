#include "stm32g0xx.h"
#include "stm32g031xx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define MPU6050_ADDRESS 0x68

#define MPU6050_WHO_AM_I 0x75
#define MPU6050_POWER_MGMT_1 0x6B

// ACCELEROMETER REGISTER ADDRESS
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C

#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E

#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

// GYRO REGISTER ADDRESS
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44

#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46

#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

#define SMOOTHING_FACTOR 0.001
#define KNOCK_THRESHOLD 20
#define TIMER3PSC 16000
#define TIMER3PERIYOD 100

typedef struct
{
	float accelx;
	float accely;
	float accelz;
	float gyrox;
	float gyroy;
	float gyroz;

	float accelx_smooth;
	float accely_smooth;
	float accelz_smooth;
	float gyrox_smooth;
	float gyroy_smooth;
	float gyroz_smooth;
} MPU6050;

void GPIO_Init(void);
void I2C_Config(void);
uint8_t I2C_Read(uint8_t, uint8_t);
void I2C_Write(uint8_t devAddr, uint8_t regAddr, uint8_t data);

uint8_t detect_knock(MPU6050 sensorData);

void TIM3_Clock_Init(void);
void TIM3_Interrupt_Config(void);
void TIM3_IRQHandler(void);
void Start_TIM3(void);
void Stop_TIM3(void);
void IMU_sensor_read(void);

int16_t data;
int16_t accelx;
int16_t gyrox;
int16_t accely;
int16_t gyroy;
int16_t accelz;
int16_t gyroz;

uint16_t knock_num;
uint8_t timerFlag;

MPU6050 sensorData;

int main(void)
{
	I2C_Config();
	GPIO_Init();
	TIM3_Clock_Init();
	TIM3_Interrupt_Config();
	Start_TIM3();

	data = I2C_Read(MPU6050_ADDRESS, MPU6050_WHO_AM_I);
	data = I2C_Read(MPU6050_ADDRESS, MPU6050_POWER_MGMT_1);
	I2C_Write(MPU6050_ADDRESS, MPU6050_POWER_MGMT_1, 0x00);

	data = I2C_Read(MPU6050_ADDRESS, MPU6050_POWER_MGMT_1);

	while (1)
	{
		IMU_sensor_read();

		if (timerFlag)
		{
			if (detect_knock(sensorData))
			{
				knock_num += detect_knock(sensorData);
				TIM3->CNT = 0;
				timerFlag = 0;
			}
		}
	}
}

void GPIO_Init(void)
{
	/* Enable GPIOA */
	RCC->IOPENR |= (1U << 0);
	// Select AF from Moder for PA9,PA10 (10)
	GPIOA->MODER &= ~(3U << 2 * 9);
	GPIOA->MODER |= (2U << 2 * 9);
	GPIOA->OTYPER |= (1U << 9);
	GPIOA->OSPEEDR |= (3U << 18); // high speed
	GPIOA->PUPDR |= (1U << 18);
	GPIOA->AFR[1] |= (6U << 4); // for PA9,PA10 AF6(I2C)

	GPIOA->MODER &= ~(3U << 2 * 10);
	GPIOA->MODER |= (2U << 2 * 10);
	GPIOA->OTYPER |= (1U << 10);
	GPIOA->OSPEEDR |= (3U << 20); // high speed
	GPIOA->PUPDR |= (1U << 20);
	GPIOA->AFR[1] |= (6U << 8); // for PA9,PA10 AF6(I2C)
}

void I2C_Config(void)
{
	// Enable the I2C Clock and GPIO Clock
	RCC->APBENR1 |= (1U << 21); // enable IC2 clock
	// Enables GPIOA peripheral
	RCC->AHBENR |= 1;
	I2C1->CR1 |= 0;

	I2C1->TIMINGR |= (3 << 28);	  // PRESC
	I2C1->TIMINGR |= (0x13 << 0); // SCLL
	I2C1->TIMINGR |= (0xF << 8);  // SCLH
	I2C1->TIMINGR |= (0x2 << 16); // SDADEL
	I2C1->TIMINGR |= (0x4 << 20); // SCLDEL

	I2C1->CR1 = (1U << 0); // PE

	NVIC_SetPriority(I2C1_IRQn, 1);
	NVIC_EnableIRQ(I2C1_IRQn);
}

uint8_t I2C_Read(uint8_t devAddr, uint8_t regAddr)
{

	// WRITE OPERTAION(Send address and register to read)
	I2C1->CR2 = 0;
	I2C1->CR2 |= ((uint32_t)devAddr << 1); // slave address
	I2C1->CR2 |= (1U << 16);			   // Number of bytes
	I2C1->CR2 |= (1U << 13);			   // Generate Start
	while (!(I2C1->ISR & (1 << 1)))
		; // TXIS

	I2C1->TXDR = (uint32_t)regAddr;
	while (!(I2C1->ISR & (1 << 6)))
		; // TC

	// READ OPERATION(read data)
	I2C1->CR2 = 0;
	I2C1->CR2 |= ((uint32_t)devAddr << 1); // slave address
	I2C1->CR2 |= (1U << 10);			   // READ mode
	I2C1->CR2 |= (1U << 16);			   // Number of bytes
	I2C1->CR2 |= (1U << 15);			   // NACK
	I2C1->CR2 |= (1U << 25);			   // AUTOEND

	I2C1->CR2 |= (1U << 13); // Generate Start
	while (!(I2C1->ISR & (1 << 2)))
		; // wait until RXNE=1

	uint8_t data = (uint8_t)I2C1->RXDR;
	return data;
}

void I2C_Write(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{

	// WRITE OPERTAION(Send address and register to read)
	I2C1->CR2 = 0;
	I2C1->CR2 |= ((uint32_t)devAddr << 1); // slave address
	I2C1->CR2 |= (2U << 16);			   // Number of bytes
	I2C1->CR2 |= (1U << 25);			   // AUTOEND
	I2C1->CR2 |= (1U << 13);			   // Generate Start

	while (!(I2C1->ISR & (1 << 1)))
		; // TXIS
	I2C1->TXDR = (uint32_t)regAddr;

	while (!(I2C1->ISR & (1 << 1)))
		; // TXIS
	I2C1->TXDR = (uint32_t)data;
}



uint8_t detect_knock(MPU6050 sensorData)
{
	if (abs(sensorData.gyrox - sensorData.gyrox_smooth) > KNOCK_THRESHOLD || abs(sensorData.gyrox - sensorData.gyroy_smooth) > KNOCK_THRESHOLD || abs(sensorData.gyroz - sensorData.gyroz_smooth) > KNOCK_THRESHOLD)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
void TIM3_Clock_Init(void)
{
	// Enable TIM3 clock
	RCC->APBENR1 |= RCC_APBENR1_TIM3EN;

	// Set TIM3 prescaler and period
	TIM3->PSC = TIMER3PSC - 1;
	TIM3->CNT = 0;
	TIM3->ARR = TIMER3PERIYOD - 1;
}
void TIM3_Interrupt_Config(void)
{
	NVIC_SetPriority(TIM3_IRQn, 15);
	NVIC_EnableIRQ(TIM3_IRQn);
	// Update interrupt enable
	TIM3->DIER |= TIM_DIER_UIE;
}

void Start_TIM3(void)
{
	// Start TIM3
	TIM3->CR1 |= TIM_CR1_CEN;
}
void Stop_TIM3(void)
{
	// Stop TIM3
	TIM3->CR1 &= ~TIM_CR1_CEN;
}
void TIM3_IRQHandler(void)
{
	if (TIM3->SR & TIM_SR_UIF)
	{
		// Code here
		timerFlag = 1;
		// Clear the interrupt flag
		TIM3->SR &= ~TIM_SR_UIF;
	}
}
void IMU_sensor_read(void){

	accelx = I2C_Read(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_L);
	accelx = accelx | (I2C_Read(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H) << 8);
	sensorData.accelx = (float)(accelx) / 16384;
	sensorData.accelx_smooth += SMOOTHING_FACTOR * (sensorData.accelx - sensorData.accelx_smooth);

	accely = I2C_Read(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_L);
	accely = accely | (I2C_Read(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_H) << 8);
	sensorData.accely = (float)(accely) / 16384;
	sensorData.accely_smooth += SMOOTHING_FACTOR * (sensorData.accely - sensorData.accely_smooth);

	accelz = I2C_Read(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_L);
	accelz = accelz | (I2C_Read(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_H) << 8);
	sensorData.accelz = (float)(accelz) / 16384;
	sensorData.accelz_smooth += SMOOTHING_FACTOR * (sensorData.accelz - sensorData.accelz_smooth);

	gyrox = I2C_Read(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_L);
	gyrox = gyrox | (I2C_Read(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H) << 8);
	sensorData.gyrox = (float)(gyrox) / 131;
	sensorData.gyrox_smooth += SMOOTHING_FACTOR * (sensorData.gyrox - sensorData.gyrox_smooth);

	gyroy = I2C_Read(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_L);
	gyroy = gyroy | (I2C_Read(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_H) << 8);
	sensorData.gyroy = (float)(gyroy) / 131;
	sensorData.gyroy_smooth += SMOOTHING_FACTOR * (sensorData.gyroy - sensorData.gyroy_smooth);

	gyroz = I2C_Read(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_L);
	gyroz = gyroz | (I2C_Read(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_H) << 8);
	sensorData.gyroz = (float)(gyroz) / 131;
	sensorData.gyroz_smooth += SMOOTHING_FACTOR * (sensorData.gyroz - sensorData.gyroz_smooth);

}
