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
#define FULL_DUTY_CYCLE 1000

// define prototypes
void RCC_init(void);
void GPIOA_init(void);
void GPIOB_init(void);
void ADC_init(void);
uint16_t poll_ADC(void);
void USART2_Init(void);
void printChar(uint8_t c);
int _print(int f, char *ptr, int len);
void print(char *s);
float convert_to_voltage(uint16_t adc_value);
/******PWM******/
void PWM2_CH1_init(void);
void PWM2_CH2_init(void);
void PWM2_StartPWM(void);
void PWM2_StopPWM(void);
void PWM2_CH1_SetDutyCycle(uint16_t dutyCycle);
void PWM2_CH2_SetDutyCycle(uint16_t dutyCycle);
void TIM2_IRQHandler(void);
uint16_t PWM2_CH1_calculate_Duty_cycle(uint16_t adcValue);
uint16_t PWM2_CH2_calculate_Duty_cycle(uint16_t adcValue);

// Systick functions prototypes
void delay(volatile uint32_t);
void Init_Systick(uint32_t tick);
void Systick_Handler(void);
void increase_tick(void);
uint64_t getTick(void);
void delay_ms(uint64_t msvalue);
void SysTick_Handler(void);

// Data types
typedef struct
{
	uint16_t adc_value;
	float voltage;
} ADC_typedef;

// variables
char txbuffer[6] = "/0";
uint16_t tackCounter = 0;

ADC_typedef pot;
// systick variables
uint64_t tick;

/**********PWM Variables**********/
uint16_t PWM2CH1dutyCycle;
uint16_t PWM2CH2dutyCycle;

int main(void)
{
	// Initialize the system
	RCC_init();
	GPIOA_init();
	GPIOB_init();
	ADC_init();
	PWM2_CH1_init();
	PWM2_CH2_init();
	USART2_Init();
	Init_Systick(16000);
	PWM2_CH2_SetDutyCycle(0);
	PWM2_StartPWM();
	// Infinite loop
	while (1)
	{
		// Read the ADC converted value
		pot.adc_value = poll_ADC();
		// Convert adc value to voltage
		pot.voltage = convert_to_voltage(pot.adc_value);
		PWM2CH1dutyCycle = PWM2_CH1_calculate_Duty_cycle(pot.adc_value);
		PWM2CH2dutyCycle = PWM2_CH2_calculate_Duty_cycle(pot.adc_value);
		sprintf(txbuffer, "%d", pot.adc_value);
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

	/***************PWM*****************/
	// Enable GPIOB clock
	RCC->IOPENR |= (1U << 1);
	// Enable TIM2 clock
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN;
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

	/***********PWM************/
	// Set alternate function to 2
	// 15 comes from PA15
	GPIOA->AFR[1] |= (GPIO_AFRH_AFSEL15_1);
	// Set pwm for PA15
	GPIOA->MODER &= ~(GPIO_MODER_MODE15);

	// Select AF from Moder
	GPIOA->MODER |= (GPIO_MODER_MODE15_1);
}
void GPIOB_init(void)
{
	/***********PWM************/
	// Set alternate function to 2
	// 3 comes from PB3
	GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL3_1);
	// Select AF from Moder
	GPIOB->MODER &= ~(GPIO_MODER_MODE3);
	GPIOB->MODER |= (GPIO_MODER_MODE3_1);
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
float convert_to_voltage(uint16_t adc_value)
{
	return (float)adc_value * VDDA / ADC_MAX_VALUE;
}

/*********PWM***********/

void TIM2_IRQHandler(void)
{

	// update duty
	PWM2_CH1_SetDutyCycle(PWM2CH1dutyCycle);
	PWM2_CH2_SetDutyCycle(PWM2CH2dutyCycle);

	// Clear update status register
	TIM2->SR &= ~(1U << 0);
}

void PWM2_CH1_init(void)
{
	// zero out the control register just in case
	TIM2->CR1 = 0;

	// Select PWM Mode 1
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	// Preload Enable
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

	// Capture compare ch1 enable
	TIM2->CCER |= TIM_CCER_CC1E;

	// zero out counter
	TIM2->CNT = 0;
	// 1 ms interrupt
	// CLOCK/((PSC+1)* ARR)
	TIM2->PSC = 15;
	TIM2->ARR = 1000;

	// zero out duty
	TIM2->CCR1 = 0;

	// Update interrupt enable
	TIM2->DIER |= (1 << 0);

	// TIM1 Enable
	// TIM2->CR1 |= TIM_CR1_CEN;

	NVIC_SetPriority(TIM2_IRQn, 1);
	NVIC_EnableIRQ(TIM2_IRQn);
}

/*
 * Setup PWM output for
 * TIM2 CH2 on PB3
 */
void PWM2_CH2_init(void)
{
	// zero out the control register just in case
	TIM2->CR1 = 0;

	// Select PWM Mode 1
	TIM2->CCMR1 |= (6U << 12);
	// Preload Enable
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;

	// Capture compare ch2 enable
	TIM2->CCER |= TIM_CCER_CC2E;

	// zero out counter
	TIM2->CNT = 0;
	// 1 ms interrupt
	// CLOCK/((PSC+1)* ARR)
	TIM2->PSC = 15;
	TIM2->ARR = 1000;

	// zero out duty
	TIM2->CCR2 = 0;

	// Update interrupt enable
	TIM2->DIER |= (1 << 0);

	// TIM1 Enable
	// TIM2->CR1 |= TIM_CR1_CEN;

	NVIC_SetPriority(TIM2_IRQn, 1);
	NVIC_EnableIRQ(TIM2_IRQn);
}
void PWM2_StartPWM(void)
{
	// Enable the timer
	TIM2->CR1 |= TIM_CR1_CEN;
}

void PWM2_StopPWM(void)
{
	// Disable the timer
	TIM2->CR1 &= ~TIM_CR1_CEN;
}
void PWM2_CH1_SetDutyCycle(uint16_t dutyCycle)
{
	// Ensure duty cycle is within bounds
	// Set the duty cycle by updating the CCR1 register
	TIM2->CCR1 = dutyCycle;
}
void PWM2_CH2_SetDutyCycle(uint16_t dutyCycle)
{
	// Ensure duty cycle is within bounds
	// Set the duty cycle by updating the CCR2 register
	TIM2->CCR2 = dutyCycle;
}

uint16_t PWM2_CH1_calculate_Duty_cycle(uint16_t adcValue)
{
	uint16_t dutyCycle = 0;

	if (adcValue < (ADC_MAX_VALUE / 2))
	{
		dutyCycle = 0;
	}
	else
	{
		dutyCycle = (uint16_t)round((((float)(adcValue * 2 - ADC_MAX_VALUE)) / ADC_MAX_VALUE) * FULL_DUTY_CYCLE);
	}
	return dutyCycle;
}
// calculate duty cycle for the first led near the ground
uint16_t PWM2_CH2_calculate_Duty_cycle(uint16_t adcValue)
{
	uint16_t dutyCycle = 0;

	if (adcValue > (ADC_MAX_VALUE / 2))
	{
		dutyCycle = 0;
	}
	else
	{
		dutyCycle = FULL_DUTY_CYCLE - (uint16_t)round((((float)adcValue * 2) / ADC_MAX_VALUE) * FULL_DUTY_CYCLE);
	}
	return dutyCycle;
}

/*********Uart functions***********/

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

void increase_tick(void)
{
	tick++;
}
void Init_Systick(uint32_t tick)
{
	SysTick->LOAD = tick; // Count down from 999 to 0
	SysTick->VAL = 0;	  // Clear current value
	SysTick->CTRL = 0x7;  // Enable Systick, exception,and use processor clock
}
uint64_t getTick(void)
{
	return tick;
}
void delay_ms(uint64_t msvalue)
{
	uint64_t startTick = getTick();
	while ((getTick() - startTick) < msvalue)
	{
	}
}
/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	increase_tick();
}
