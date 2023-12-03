#include "stm32g0xx.h"
#include "stm32g031xx.h"

// seven segment display defines
#define SEGMENT_A 1U << (1U - 1U)
#define SEGMENT_B 1U << (2U - 1U)
#define SEGMENT_C 1U << (3U - 1U)
#define SEGMENT_D 1U << (4U - 1U)
#define SEGMENT_E 1U << (5U - 1U)
#define SEGMENT_F 1U << (6U - 1U)
#define SEGMENT_G 1U << (7U - 1U)
#define SEGMENT_DP 1U << (8U - 1U)
#define SEGMENT_MASK ~(SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G | SEGMENT_DP)

#define DIGIT_1 1U << (4U - 1U)
#define DIGIT_2 1U << (5U - 1U)
#define DIGIT_3 1U << (6U - 1U)
#define DIGIT_4 1U << (7U - 1U)

#define DIGIT_MASK ~(DIGIT_1 | DIGIT_2 | DIGIT_3 | DIGIT_4)
// seven segment display numbers
uint8_t numbers[10] = {
	SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F,			   // 0
	SEGMENT_B | SEGMENT_C,															   // 1
	SEGMENT_A | SEGMENT_B | SEGMENT_D | SEGMENT_E | SEGMENT_G,						   // 2
	SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_G,						   // 3
	SEGMENT_B | SEGMENT_C | SEGMENT_F | SEGMENT_G,									   // 4
	SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,						   // 5
	SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G,			   // 6
	SEGMENT_A | SEGMENT_B | SEGMENT_C,												   // 7
	SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G, // 8
	SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,			   // 9
};
// defines
#define TIMERPSC 16000
#define TIMERPERIYOD 1000 / 1000
// variables
uint64_t tick;
uint64_t seconds;
uint8_t buttonCounter = 1;
uint8_t buttonPreviousState;
uint8_t buttonNewState;

// functions prototypes
void GPIO_RCC_Init(void);
void GPIOC_Init(void);
void GPIOA_Init(void);

void TIM2_Clock_Init(void);
void TIM2_Interrupt_Config(void);
void TIM2_IRQHandler(void);
void Start_TIM2(void);
void Stop_TIM2(void);

void toggle_led(void);
uint8_t readButton(void);

void EXTI_Init(void);
void EXTI0_1_IRQHandler(void);

void sevenSegmentShowNumber(uint16_t number);

int main(void)
{

	// init
	GPIO_RCC_Init();
	GPIOA_Init();
	GPIOC_Init();
	TIM2_Clock_Init();
	TIM2_Interrupt_Config();
	EXTI_Init();

	Start_TIM2();

	while (1)
	{
		sevenSegmentShowNumber(9999);
	}

	return 0;
}
void sevenSegmentShowNumber(uint16_t number)
{
	// define digits from the number and use GPIOB
	uint8_t digit1 = number / 1000;
	uint8_t digit2 = (number % 1000) / 100;
	uint8_t digit3 = (number % 100) / 10;
	uint8_t digit4 = number % 10;

	// select digit4 using GPIOA with mask
	GPIOA->ODR = (GPIOA->ODR & DIGIT_MASK) & ~DIGIT_4;
	// set GPIOB to output for digit4 using numbers with 8 bit mask
	GPIOB->ODR = (GPIOB->ODR & SEGMENT_MASK) | numbers[digit4];

	// select digit3 using GPIOA with mask
	GPIOA->ODR = (GPIOA->ODR & DIGIT_MASK) & ~DIGIT_3;
	// set GPIOB to output for digit3 using numbers
	GPIOB->ODR = (GPIOB->ODR & SEGMENT_MASK) | numbers[digit3];

	// select digit2 using GPIOA with mask
	GPIOA->ODR = (GPIOA->ODR & DIGIT_MASK) & ~DIGIT_2;
	// set GPIOB to output for digit2 using numbers
	GPIOB->ODR = (GPIOB->ODR & SEGMENT_MASK) | numbers[digit2];

	// select digit1 using GPIOA with mask
	GPIOA->ODR = (GPIOA->ODR & DIGIT_MASK) & ~DIGIT_1;
	// set GPIOB to output for digit1 using numbers
	GPIOB->ODR = (GPIOB->ODR & SEGMENT_MASK) | numbers[digit1];
}

void TIM2_IRQHandler(void)
{
	if (TIM2->SR & TIM_SR_UIF)
	{
		// Code here
		seconds++;
		toggle_led();

		// Clear the interrupt flag
		TIM2->SR &= ~TIM_SR_UIF;
	}
}
void EXTI0_1_IRQHandler(void)
{
	// Your code to handle the EXTI interrupt for PA0 goes here
	// This is where you can perform specific actions when the rising edge occurs
	if (EXTI->RPR1 & EXTI_FPR1_FPIF0) // Check if the interrupt is from line 0
	{
		buttonCounter++;
		if (buttonCounter > 10)
		{
			buttonCounter = 1;
		}
		TIM2_Clock_Init();
		TIM2_Interrupt_Config();
		// Clear EXTI0 pending flag
		EXTI->RPR1 |= EXTI_FPR1_FPIF0;
	}
}
uint8_t readButton(void)
{

	return GPIOA->IDR & 1;
}
void EXTI_Init(void)
{

	// Connect EXTI1 to GPIOA Pin 0
	EXTI->EXTICR[1] |= EXTI_EXTICR1_EXTI0_0;

	// Configure EXTI0 to rising edge trigger
	EXTI->RTSR1 |= EXTI_RTSR1_RT0;
	EXTI->FTSR1 &= ~EXTI_FTSR1_FT0;

	// Enable EXTI0 interrupt
	EXTI->IMR1 |= EXTI_IMR1_IM0;

	// Set the EXTI0 interrupt priority (adjust as needed)
	NVIC_SetPriority(EXTI0_1_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void GPIO_RCC_Init(void)
{
	/* Enable GPIOA ,B and C clock */
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;

	// Enable clock for SYSCFG
	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;

	// Enables GPIOA peripheral
	RCC->AHBENR |= 1;
}
void GPIOC_Init(void)
{
	// Configure GPIO for board on led
	GPIOC->MODER &= ~(3U << 2 * 6);
	GPIOC->MODER |= (1U << 2 * 6);
}
void GPIOA_Init(void)
{
	// Set GPIOA Pin 0 as input
	GPIOA->MODER &= ~GPIO_MODER_MODE0; // Clear mode bits for pin 0
	// clear GPIOA Pin 4,5,6,7
	GPIOA->MODER &= ~GPIO_MODER_MODE4; // Clear mode bits for pin 4
	GPIOA->MODER &= ~GPIO_MODER_MODE5; // Clear mode bits for pin 5
	GPIOA->MODER &= ~GPIO_MODER_MODE6; // Clear mode bits for pin 6
	GPIOA->MODER &= ~GPIO_MODER_MODE7; // Clear mode bits for pin 7
	//  Set GPIOA Pin 4,5,6,7 as output
	GPIOA->MODER |= (1U << 2 * 4); // Set mode bits for pin 4
	GPIOA->MODER |= (1U << 2 * 5); // Set mode bits for pin 5
	GPIOA->MODER |= (1U << 2 * 6); // Set mode bits for pin 6
	GPIOA->MODER |= (1U << 2 * 7); // Set mode bits for pin 7
	// Enable pull-down resistor for GPIOA Pin 0
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;	// Clear pull-up/pull-down bits for pin 0
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_1; // Set bit for pull-down
}

void toggle_led(void)
{
	GPIOC->ODR ^= (1U << 6);
}
void TIM2_Clock_Init(void)
{
	// Enable TIM2 clock
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN;

	// Set TIM2 prescaler and period
	TIM2->PSC = TIMERPSC - 1;
	TIM2->CNT = 0;
	TIM2->ARR = (TIMERPERIYOD)-1;
}
void TIM2_Interrupt_Config(void)
{
	NVIC_SetPriority(TIM2_IRQn, 15);
	NVIC_EnableIRQ(TIM2_IRQn);
	// Update interrupt enable
	TIM2->DIER |= TIM_DIER_UIE;
}

void Start_TIM2(void)
{
	// Start TIM2
	TIM2->CR1 |= TIM_CR1_CEN;
}
void Stop_TIM2(void)
{
	// Stop TIM2
	TIM2->CR1 &= ~TIM_CR1_CEN;
}
