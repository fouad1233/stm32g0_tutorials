#include "stm32g0xx.h"
#include "stm32g031xx.h"



//variables
uint64_t tick;
uint8_t seconds;



//functions prototypes
void GPIOC_RCC_Init(void);
void GPIOC_Init(void);
void toggle_led(void);

void TIM2_Clock_Init(void);
void TIM2_Interrupt_Config(void);
void TIM2_IRQHandler(void);
void Start_TIM2(void);
void Stop_TIM2(void);


int main(void) {

	//init
	GPIOC_RCC_Init();
	GPIOC_Init();
	TIM2_Clock_Init();
	TIM2_Interrupt_Config();
	Start_TIM2();

	while(1) {

    }

    return 0;
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
void GPIOC_RCC_Init(void){
	 /* Enable GPIOC clock */
	RCC->IOPENR |= (1U << 2);
}
void GPIOC_Init(void){
	GPIOC->MODER &= ~(3U << 2*6);
	GPIOC->MODER |= (1U << 2*6);
}
void toggle_led(void){
	GPIOC->ODR ^= (1U << 6);
}
void TIM2_Clock_Init(void){
	// Enable TIM2 clock
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN;

	// Set TIM2 prescaler and period
	TIM2->PSC = 15999;
	TIM2->ARR = 999;
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


