#include "stm32g0xx.h"
#include "stm32g031xx.h"


//defines
#define TIMERPSC 16000
#define TIMERPERIYOD 1000
//variables
uint64_t tick;
uint64_t seconds;
uint8_t buttonCounter = 1;
uint8_t buttonPreviousState;
uint8_t buttonNewState;



//functions prototypes
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

int main(void) {

	//init
	GPIO_RCC_Init();
	GPIOA_Init();
	GPIOC_Init();
	TIM2_Clock_Init();
	TIM2_Interrupt_Config();


	Start_TIM2();


	while(1) {

		buttonNewState = readButton();
		if (buttonPreviousState == 0 && buttonNewState == 1){
			buttonCounter++;
			if (buttonCounter > 10) {
				buttonCounter = 1;
			}
			TIM2_Clock_Init();
		}
		buttonPreviousState = buttonNewState;
    }

    return 0;
}
uint8_t readButton(void){

	return GPIOA->IDR & 1;
}
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        //Code here
    	seconds++;
    	toggle_led();

        // Clear the interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;
    }
}


void GPIO_RCC_Init(void){
	 /* Enable GPIOC clock */
	RCC->IOPENR |= (1U << 2);

	// Enable clock for GPIOA
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

	// Enable clock for SYSCFG
	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
}
void GPIOC_Init(void){
	//Configure GPIO for board on led
	GPIOC->MODER &= ~(3U << 2*6);
	GPIOC->MODER |= (1U << 2*6);
}
void GPIOA_Init(void){
	// Set GPIOA Pin 0 as input
	GPIOA->MODER &= ~GPIO_MODER_MODE0; // Clear mode bits for pin 0

	// Enable pull-down resistor for GPIOA Pin 0
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0; // Clear pull-up/pull-down bits for pin 0
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_1; // Set bit for pull-down

}


void toggle_led(void){
	GPIOC->ODR ^= (1U << 6);
}
void TIM2_Clock_Init(void){
	// Enable TIM2 clock
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN;

	// Set TIM2 prescaler and period
	TIM2->PSC = TIMERPSC - 1;
	TIM2->ARR = (TIMERPERIYOD *buttonCounter) - 1;
}
void TIM2_Interrupt_Config(void){
	NVIC_SetPriority(TIM2_IRQn, 15);
	NVIC_EnableIRQ(TIM2_IRQn);
	//Update interrupt enable
	TIM2->DIER |= TIM_DIER_UIE;
}

void Start_TIM2(void){
	// Start TIM2
	TIM2->CR1 |= TIM_CR1_CEN;
}
void Stop_TIM2(void){
	// Stop TIM2
	TIM2->CR1 &= ~TIM_CR1_CEN;
}


