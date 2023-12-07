#include "stm32g0xx.h"
#include "stm32g031xx.h"


//variables
uint64_t tick;
uint8_t seconds;
uint16_t counter = 0;
uint8_t pwmDirection = 1;//1 for increase, 0 for decrease

//defines
#define SYSTICK_FREQ 16000 //Khz


//functions prototypes
void delay(volatile uint32_t);
void Init_Systick(uint32_t tick);
void Systick_Handler(void);
void increase_tick(void);
uint64_t getTick(void);
void delay_ms(uint64_t msvalue);
void SysTick_Handler(void);
void RCC_Init(void);
void GPIOA_Init(void);
void TIM2_Init(void);
void SetDutyCycle(uint16_t dutyCycle);
void StartPWM(void);
void StopPWM(void);

int main(void) {

	//init
	RCC_Init();
	GPIOA_Init();
	TIM2_Init();
	Init_Systick(SYSTICK_FREQ);
	StartPWM();
	SetDutyCycle(7500);
	while(1) {

    }

    return 0;
}
void RCC_Init(void){
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;  // Enable GPIOA clock

	RCC->APBENR1 |= RCC_APBENR1_TIM2EN;  // Enable TIM2 clock
}

void GPIOA_Init(void){
	GPIOA->MODER &= ~GPIO_MODER_MODE1_Msk;
	GPIOA->MODER |= GPIO_MODER_MODE1_1;  // Alternate function mode
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL1_0;  // AF1 for TIM2
}

void TIM2_Init(void) {
    TIM2->CR1 &= ~TIM_CR1_DIR;  // Count up mode
    TIM2->PSC = 1599;  // Prescaler
    TIM2->ARR = 9999;  // Auto-reload value (adjust for desired PWM frequency)

    // Configure PWM mode for channel 2
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // PWM mode 1

    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;  // Output compare 2 preload enable

    //
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S_0;

    TIM2->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;
    TIM2->CCMR1 &= ~TIM_CCMR1_CC2S_0;


    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;


    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;  // PWM mode 1


    TIM2->CR1 &= ~TIM_CR1_DIR_Msk;
    TIM2->CR1 |=TIM_CR1_DIR;//counter used as up counter

    //Edge aligned mode enabled
    TIM2->CR1 &= ~TIM_CR1_CMS_Msk;
    TIM2->CR1 |=TIM_CR1_CMS_0;
    // Clear clock division
    TIM2->CR1 &= ~TIM_CR1_CKD_Msk;
    // Set clock division to divide by 1
    TIM2->CR1 |= TIM_CR1_CKD_0;


    TIM2->CR1  |= TIM_CR1_ARPE;

    TIM2->CCER |= TIM_CCER_CC2E;  // Enable capture/compare channel 2

    TIM2->CCER |= TIM_CCER_CC1E;


}
void StartPWM(void) {
    // Enable the timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

void StopPWM(void) {
    // Disable the timer
    TIM2->CR1 &= ~TIM_CR1_CEN;
}
void SetDutyCycle(uint16_t dutyCycle) {
    // Ensure duty cycle is within bounds


    // Set the duty cycle by updating the CCR2 register
    TIM2->CCR2 = dutyCycle;
    TIM2->CCR1 = dutyCycle;

}

void increase_tick(void){
	tick++;

}
void Init_Systick(uint32_t tick){
	SysTick->LOAD = tick; // Count down from 999 to 0
	SysTick->VAL  = 0;   // Clear current value
	SysTick->CTRL = 0x7; // Enable Systick, exception,and use processor clock
}
uint64_t getTick(void){
	return tick;
}
void delay_ms(uint64_t msvalue){
	uint64_t startTick =getTick() ;
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
	if(pwmDirection){
		counter++;
		if(counter>1000){
			counter =999;
			pwmDirection=0;
		}
	}else{
		counter--;
		if(counter<1){
			counter =0;
			pwmDirection=1;
		}
	}
}


