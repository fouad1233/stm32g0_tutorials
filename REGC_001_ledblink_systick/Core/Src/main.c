#include "stm32g0xx.h"

#define LEDDELAY    1600000

void delay(volatile uint32_t);

int main(void) {

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
}

void delay(volatile uint32_t s) {
    for(; s>0; s--);
}
