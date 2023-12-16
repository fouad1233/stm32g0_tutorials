#include "stm32g0xx.h"
#include <math.h>
#include "main.h"

void clearRowsKeypad(void);
void setRowsKeypad(void);
void storenumber(int sayı);
uint32_t number=0;// number of stored digits
uint32_t count;	 // duty cycle rate
uint32_t ust;
uint8_t b;
uint32_t numbers[3]; // store digit


void RCC_Init(void)
{
	RCC->IOPENR |= (1U << 0); /*Enable GPIOA clock*/
	RCC->IOPENR |= (1U << 1); /*Enable GPIOB clock*/
	// Enable clock for SYSCFG
	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
	// Enables GPIOA peripheral
	RCC->AHBENR |= 1;
}

void GPIOB_Init(void){
	/*Set up PB4,PB5,PB6,PB7 as output row(output=01)*/
		GPIOB->MODER &= ~(3U << 8);
		GPIOB->MODER |= (1U << 8); /*for PB4*/

		GPIOB->MODER &= ~(3U << 10);
		GPIOB->MODER |= (1U << 10); /*for PB5*/

		GPIOB->MODER &= ~(3U << 12);
		GPIOB->MODER |= (1U << 12); /*for PB6*/

		GPIOB->MODER &= ~(3U << 14);
		GPIOB->MODER |= (1U << 14); /*for PB7*/

}



void GPIOA_Init(void){
	/*Set up PA4,PA5,PA6,PA7 as input column(input=00)*/
		GPIOA->MODER &= ~(3U << 8); /*for PA4*/
		GPIOA->PUPDR |= (2U << 8);	/*Pull down mode*/

		GPIOA->MODER &= ~(3U << 10); /*for PA5*/
		GPIOA->PUPDR |= (2U << 10);	 /*Pull down mode*/

		GPIOA->MODER &= ~(3U << 12); /*for PA6*/
		GPIOA->PUPDR |= (2U << 12);	 /*Pull down mode*/

		GPIOA->MODER &= ~(3U << 14); /*for PA7*/
		GPIOA->PUPDR |= (2U << 14);	 /*Pull down mode*/

}


void EXTI_Init(void)
{
	/*Set up interrupt for input*/
		EXTI->EXTICR[2] &= ~EXTI_EXTICR2_EXTI4_Msk; /*PA4*/
		EXTI->EXTICR[2] &= ~EXTI_EXTICR2_EXTI5_Msk; /*PA5*/
		EXTI->EXTICR[2] &= ~EXTI_EXTICR2_EXTI6_Msk; /*PA6*/
		EXTI->EXTICR[2] &= ~EXTI_EXTICR2_EXTI7_Msk; /*PA7*/

		/*Rising edge*/
		EXTI->RTSR1 |= EXTI_RTSR1_RT4;
		EXTI->FTSR1 &= ~EXTI_FTSR1_FT4;
		EXTI->RTSR1 |= EXTI_RTSR1_RT5;
		EXTI->FTSR1 &= ~EXTI_FTSR1_FT5;
		EXTI->RTSR1 |= EXTI_RTSR1_RT6;
		EXTI->FTSR1 &= ~EXTI_FTSR1_FT6;
		EXTI->RTSR1 |= EXTI_RTSR1_RT7;
		EXTI->FTSR1 &= ~EXTI_FTSR1_FT7;

		/*MASK*/
		EXTI->IMR1 |= EXTI_IMR1_IM4;
		EXTI->IMR1 |= EXTI_IMR1_IM5;
		EXTI->IMR1 |= EXTI_IMR1_IM6;
		EXTI->IMR1 |= EXTI_IMR1_IM7;

		/*NVIC*/

		NVIC_SetPriority(EXTI4_15_IRQn,2);
		NVIC_EnableIRQ(EXTI4_15_IRQn);


}



void DelayMs(uint32_t ms) {
	for (uint32_t i = 0; i < ms * 1000; i++) {
	        // Hiçbir şey yapma - sadece beklet
	    }
}

void storenumber(int sayı)
{
	if (sayı >= 0 && sayı <=9 && number!=3)
	{
		numbers[number] = sayı;
		number = number + 1;
		DelayMs(500);
	}

	if (sayı == 10 && number != 0)
	{
		int i;

		for (i = 0; i < number; i++)
		{
			ust = pow(10, number - i - 1);
			count = count + numbers[i] * ust;
		}
		//count=0;
		number = 0;
		numbers[0] = 0;
		numbers[1] = 0;
		numbers[2] = 0;

	}



}



void EXTI4_15_IRQHandler(void)
{

	if ((EXTI->RPR1 >> 4) & 1)
	{ // Interrupt from PA4
	    clearRowsKeypad();

		GPIOB->ODR ^= (1U << 4); // PB4
		if ((GPIOA->IDR >> 4) & 1){
		   storenumber(1);
		   }
		GPIOB->ODR ^= (1U << 4); // PB4

		GPIOB->ODR ^= (1U << 5); // PB5
		if ((GPIOA->IDR >> 4) & 1){
		   storenumber(4);
		   }
		GPIOB->ODR ^= (1U << 5); // PB5

		GPIOB->ODR ^= (1U << 6); // PB6
		if ((GPIOA->IDR >> 4) & 1){
			storenumber(7);
		}
		GPIOB->ODR ^= (1U << 6); // PB6

		GPIOB->ODR ^= (1U << 7); // PB7
		if ((GPIOA->IDR >> 4) & 1){
			storenumber(11);
		}
		GPIOB->ODR ^= (1U << 7); // PB7

		EXTI->RPR1 |= (1U << 4); // clear interrupt flag
		setRowsKeypad();
	}

	if ((EXTI->RPR1 >> 5) & 1)
	{ // Interrupt from PA5

		clearRowsKeypad();
		GPIOB->ODR ^= (1U << 4); // PB4
		if ((GPIOA->IDR >> 5) & 1)
		{
			storenumber(2);
		}
		GPIOB->ODR ^= (1U << 4); // PB4

		GPIOB->ODR ^= (1U << 5); // PB5
		if ((GPIOA->IDR >> 5) & 1)
		{
			storenumber(5);
		}
		GPIOB->ODR ^= (1U << 5); // PB5

		GPIOB->ODR ^= (1U << 6); // PB6
		if ((GPIOA->IDR >> 5) & 1)
		{
			storenumber(8);
		}
		GPIOB->ODR ^= (1U << 6); // PB6

		GPIOB->ODR ^= (1U << 7); // PB7
		if ((GPIOA->IDR >> 5) & 1)
		{
			storenumber(0);
		}
		GPIOB->ODR ^= (1U << 7); // PB7
		EXTI->RPR1 |= (1U << 5); // clear interrupt flag
		setRowsKeypad();
	}

	if ((EXTI->RPR1 >> 6) & 1)
	{ // Interrupt from PA6
		clearRowsKeypad();
		GPIOB->ODR ^= (1U << 4); // PB4
		if ((GPIOA->IDR >> 6) & 1)
		{
			storenumber(3);
		}
		GPIOB->ODR ^= (1U << 4); // PB4

		GPIOB->ODR ^= (1U << 5); // PB5
		if ((GPIOA->IDR >> 6) & 1)
		{
			storenumber(6);
		}
		GPIOB->ODR ^= (1U << 5); // PB5

		GPIOB->ODR ^= (1U << 6); // PB6
		if ((GPIOA->IDR >> 6) & 1)
		{
			storenumber(9);
		}
		GPIOB->ODR ^= (1U << 6); // PB6

		GPIOB->ODR ^= (1U << 7); // PB7
		if ((GPIOA->IDR >> 6) & 1)
		{
			storenumber(10);
		}
		GPIOB->ODR ^= (1U << 7); // PB7

		EXTI->RPR1 |= (1U << 6); // clear interrupt flag
		setRowsKeypad();
	}
	if ((EXTI->RPR1 >> 7) & 1)
	{ // Interrupt from PA7
		clearRowsKeypad();
		GPIOB->ODR ^= (1U << 4); // PB4
		if ((GPIOA->IDR >> 7) & 1)
		{
			storenumber(11);
		}
		GPIOB->ODR ^= (1U << 4); // PB4

		GPIOB->ODR ^= (1U << 5); // PB5
		if ((GPIOA->IDR >> 7) & 1)
		{
			storenumber(11);
		}
		GPIOB->ODR ^= (1U << 5); // PB5

		GPIOB->ODR ^= (1U << 6); // PB6
		if ((GPIOA->IDR >> 7) & 1)
		{
			storenumber(11);
		}
		GPIOB->ODR ^= (1U << 6); // PB6

		GPIOB->ODR ^= (1U << 7); // PB7
		if ((GPIOA->IDR >> 7) & 1)
		{
			storenumber(11);
		}
		GPIOB->ODR ^= (1U << 7); // PB7

		EXTI->RPR1 |= (1U << 7); // clear interrupt flag
		setRowsKeypad();
	}
}

int main(void)
{
	RCC_Init();
    GPIOB_Init();/*Set up PB4,PB5,PB6,PB7 as output row(output=01)*/
	GPIOA_Init();/*Set up PA4,PA5,PA6,PA7 as input column(input=00)*/
	EXTI_Init();
	setRowsKeypad();/*Set all rows*/

	while (1)
	{

	}
	return 0;
}



void clearRowsKeypad(void)
{
	/*Clearing the rows*/
	GPIOB->ODR &= ~(1U << 4); /*PB4*/
	GPIOB->ODR &= ~(1U << 5); /*PB5*/
	GPIOB->ODR &= ~(1U << 6); /*PB6*/
	GPIOB->ODR &= ~(1U << 7); /*PB7*/
}

void setRowsKeypad(void)
{
	/*Setting rows*/
	GPIOB->ODR |= (1U << 4); /*PB4*/
	GPIOB->ODR |= (1U << 5); /*PB5*/
	GPIOB->ODR |= (1U << 6); /*PB6*/
	GPIOB->ODR |= (1U << 7); /*PB7*/
}
