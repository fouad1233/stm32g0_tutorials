#include "stm32g0xx.h"

void SystemClock_Config(void);
void IWDG_Init(void);
void IWDG_Refresh(void);


void Init_Systick(uint32_t tick);

uint64_t getTick(void);
void delay_ms(uint64_t msvalue);

//variables
uint64_t tick;


#define LEDDELAYMS    300

void delay(volatile uint32_t);

int main(void) {
    SystemClock_Config();
    Init_Systick(16000);
    IWDG_Init();

    while (1) {
        // Your application code goes here
        //
    	  /* Enable GPIOC clock */
    	    RCC->IOPENR |= (1U << 2);

    	    /* Setup PC6 as output */
    	    GPIOC->MODER &= ~(3U << 2*6);
    	    GPIOC->MODER |= (1U << 2*6);

    	    /* Turn on LED */
    	    GPIOC->ODR |= (1U << 6);

    	    while(1) {
    	    	delay_ms(LEDDELAYMS);
    	        /* Toggle LED */
    	        GPIOC->ODR ^= (1U << 6);
    	        IWDG_Refresh();

    	    }

    	    return 0;


        // Reload the watchdog timer to prevent a reset

    }
}

void SystemClock_Config(void) {
    // System Clock Configuration
    // Bu fonksiyon STM32CubeMX tarafından otomatik olarak üretilen kodu içerebilir.
    // ...
}

void IWDG_Init(void) {
    // Enable write access to IWDG_PR and IWDG_RLR registers
    IWDG->KR = 0x5555;

    // Set prescaler and reload values
    IWDG->PR = 4;    // Prescaler: 4
    IWDG->RLR = 1000; // Reload value: 1000

    // Reload the watchdog counter
    IWDG->KR = 0xAAAA;

    // Enable the watchdog
    IWDG->KR = 0xCCCC;
}

void IWDG_Refresh(void) {
    // Reload the watchdog counter
    IWDG->KR = 0xAAAA;
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
