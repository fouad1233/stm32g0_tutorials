#include "stm32g0xx.h"

void SystemClock_Config(void);
void IWDG_Init(void);
void IWDG_Refresh(void);
#include "stm32g0xx.h"

#define LEDDELAY    160000

void delay(volatile uint32_t);

int main(void) {
    SystemClock_Config();
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
    	        delay(LEDDELAY);
    	        /* Toggle LED */
    	        GPIOC->ODR ^= (1U << 6);

    	    }

    	    return 0;
    	    IWDG_Refresh();

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
    IWDG->KR = 0xAAAA;

    // Set prescaler and reload values
    IWDG->PR = 4;    // Prescaler: 4
    IWDG->RLR = 100; // Reload value: 1000

    // Reload the watchdog counter
    IWDG->KR = 0x5555;

    // Enable the watchdog
    IWDG->KR = 0xCCCC;
}

void IWDG_Refresh(void) {
    // Reload the watchdog counter
    IWDG->KR = 0xAAAA;
}




void delay(volatile uint32_t s) {
    for(; s>0; s--);
}
