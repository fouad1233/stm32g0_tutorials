#include "stm32g0xx.h"
#include <math.h>
#include "main.h"
#include <stdio.h>
#include "stm32g031xx.h"
/*
implement a code that read from adc the value of the ldr and turn on the led if the value is less than 1000
*/
// define prototypes
void RCC_init(void);
void GPIOB_init(void);
void ADC_init(void);
uint16_t poll_ADC(void);
uint16_t adc_value;
int main(void)
{
	// Initialize the system
	RCC_init();
	GPIOB_init();
	ADC_init();
	// Infinite loop
	while (1)
	{
		// Read the ADC converted value
		adc_value = poll_ADC();
	}
	return 0;
}
void RCC_init(void)
{
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	// Enable ADC clock
	RCC->APBENR2 |= RCC_APBENR2_ADCEN;
}

void GPIOB_init(void)
{
	// Pb1 pin as input
	GPIOB->MODER &= ~GPIO_MODER_MODE1;
	// Enable the adc mode
	GPIOB->MODER |= (GPIO_MODER_MODE1_1 |GPIO_MODER_MODE1_0);
	// Pb1 pin as push pull
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT1;
	// Pb1 pin as high speed
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1;
	// Pb1 pin as no pull up, no pull down
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD1;
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
	ADC1->CHSELR |= ADC_CHSELR_CHSEL9;
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
