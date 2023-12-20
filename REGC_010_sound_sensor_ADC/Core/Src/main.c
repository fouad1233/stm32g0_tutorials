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
static inline float convert_to_voltage(uint16_t adc_value);
// variables

typedef struct
{
	uint16_t adc_value;
	float voltage;
} sound_sensor;

sound_sensor soundSensor;
int main(void)
{
	// Initialize the system
	RCC_init();
	GPIOA_init();
	ADC_init();
	// Infinite loop
	while (1)
	{
		// Read the ADC converted value
		soundSensor.adc_value = poll_ADC();
		// Convert adc value to voltage
		soundSensor.voltage = convert_to_voltage(soundSensor.adc_value);
	}
	return 0;
}
void RCC_init(void)
{
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	// Enable ADC clock
	RCC->APBENR2 |= RCC_APBENR2_ADCEN;
}

void GPIOA_init(void)
{
	// Pa0 pin as input
	GPIOA->MODER &= ~GPIO_MODER_MODE0;
	// Enable the adc mode
	GPIOA->MODER |= (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE0_0);
	// Pa0 pin as push pull
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;
	// Pa0 pin as high speed
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0;
	// Pa0 pin as no pull up, no pull down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1;
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
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0;
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
