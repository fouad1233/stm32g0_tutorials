#include "stm32g0xx.h"
#include <math.h>
#include "main.h"
#include <stdio.h>
#include "stm32g031xx.h"
/*
implemnt a code that read adc from pa0, convert it to voltage
and detect a knock because this is a sound sensor
*/
// defines
#define VDDA 3.3
#define ADC_MAX_VALUE 4095

// define prototypes
void RCC_init(void);
void GPIOA_init(void);
void ADC_init(void);
uint16_t poll_ADC(void);
void USART2_Init(void);
void printChar(uint8_t c);
int _print(int f, char *ptr, int len);
void print(char *s);
static inline float convert_to_voltage(uint16_t adc_value);
// variables

typedef struct
{
	uint16_t adc_value;
	float voltage;
} sound_sensor;

char txbuffer[6] = "/0";

sound_sensor soundSensor;
int main(void)
{
	// Initialize the system
	RCC_init();
	GPIOA_init();
	ADC_init();
	USART2_Init();
	// Infinite loop
	while (1)
	{
		// Read the ADC converted value
		soundSensor.adc_value = poll_ADC();
		// Convert adc value to voltage
		soundSensor.voltage = convert_to_voltage(soundSensor.adc_value);
		sprintf(txbuffer, "%d", soundSensor.adc_value);
		print(txbuffer);
		print("\n");


	}
	return 0;
}
void RCC_init(void)
{
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	// Enable ADC clock
	RCC->APBENR2 |= RCC_APBENR2_ADCEN;

	/************* Uart *************/
	// enable GPIOA and USART2 module clocks from RCC
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	RCC->APBENR1 |= RCC_APBENR1_USART2EN;
}

void GPIOA_init(void)
{
	// Pa12 pin as input
	GPIOA->MODER &= ~GPIO_MODER_MODE12;
	// Enable the adc mode
	GPIOA->MODER |= (GPIO_MODER_MODE12_1 | GPIO_MODER_MODE12_0);
	// Pa12 pin as push pull
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT12;
	// Pa12 pin as high speed
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED12;
	// Pa12 pin as no pull up, no pull down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD12;

	/**********UART************/
	// enable and setup pins PA2 and PA3 as alternate function for USART2 module
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL3_0);

}
void ADC_init(void)
{
	// Set ADC clock source to PCLK/2
	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;
	// Enable the ADC voltage regulator
	ADC1->CR |= ADC_CR_ADVREGEN;
	// Wait for the ADC voltage regulator startup time
	for (int i = 0; i < 1000; i++)
		;
	// Calibrate the ADC
	ADC1->CR |= ADC_CR_ADCAL;
	// Wait for the end of ADC calibration
	while (ADC1->CR & ADC_CR_ADCAL)
		;
	// Enable the ADC
	ADC1->ISR |= ADC_ISR_ADRDY;
	// Enable the ADC
	ADC1->CR |= ADC_CR_ADEN;
	// Wait until ADC ready
	while (!(ADC1->ISR & ADC_ISR_ADRDY))
		;
	// Select the ADC input channel
	ADC1->CHSELR |= ADC_CHSELR_CHSEL16;
}
void USART2_Init(void)
{
	// enable receive and transmit from USART2 module
	USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE);
	// set baud rate to 9600 bps assuming clock is running at 16Mhz
	// Baud Rate = APBxCLK / USARTx->BRR
	USART2->BRR = 0x683; // 1667
	// enable uart module
	USART2->CR1 |= USART_CR1_UE;
}

uint16_t poll_ADC(void)
{
	// Start the ADC conversion
	ADC1->CR |= ADC_CR_ADSTART;
	// Wait for the end of the conversion
	while (!(ADC1->ISR & ADC_ISR_EOC))
		;
	// Read the ADC converted value
	return ADC1->DR;
}
// Convert adc value to voltage
static inline float convert_to_voltage(uint16_t adc_value)
{
	return (float)adc_value * VDDA / ADC_MAX_VALUE;
}

//Uart functions

void printChar(uint8_t c)
{
	// write c to the transmit data register
	USART2->TDR = c;
	// wait until transmit is complete
	while (!(USART2->ISR & USART_ISR_TC))
		;
}
int _print(int f, char *ptr, int len)
{
	(void)f; // don't do anthing with f
	// call printChar within a for loop len times
	for (int i = 0; i < len; i++)
	{
		printChar(ptr[i]);
	}
	return len; // return length
}
void print(char *s)
{
	// count number of characters in s string until a null byte comes `\0`
	int length = 0;
	while (s[length] != '\0')
	{
		length++;
	}
	_print(0, s, length);
}
