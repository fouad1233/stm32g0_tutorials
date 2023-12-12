/*
 * main.c
 */
#include "main.h"

void Init_Systick(uint32_t tick);
void increase_tick(void);
void TIM2_IRQHandler(void);
void SetDutyCycle(uint16_t dutyCycle);
void StartPWM(void);
void StopPWM(void);
void SysTick_Handler(void);
void init_pwm2(void);

// variables
uint64_t tick;
uint8_t seconds;
uint16_t counter = 0;
uint8_t pwmDirection = 1; // 1 for increase, 0 for decrease

// defines
#define SYSTICK_FREQ 16000 // Khz
int main(void)
{

    Init_Systick(SYSTICK_FREQ);
    init_pwm2();
    SetDutyCycle(0);
    StartPWM();

    while (1)
    {
    }

    return 0;
}

void TIM2_IRQHandler(void)
{
    // update duty
	SetDutyCycle(counter);
    // Clear update status register
    TIM2->SR &= ~(1U << 0);
}

/**
 * Setup PWM output for
 * TIM2 CH2 on PB3
 */
void init_pwm2(void)
{

    // Setup GPIO
    //

    // Enable GPIOB clock
    RCC->IOPENR |= (1U << 1);
    // Enable TIM2 clock
    RCC->APBENR1 |= RCC_APBENR1_TIM2EN;

    // Set alternate function to 2
    // 3 comes from PB3
    GPIOB->AFR[0] |= (2U << 4 * 3);
    // Select AF from Moder
    GPIOB->MODER &= ~(3U << 2 * 3);
    GPIOB->MODER |= (2U << 2 * 3);

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
void StartPWM(void)
{
    // Enable the timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

void StopPWM(void)
{
    // Disable the timer
    TIM2->CR1 &= ~TIM_CR1_CEN;
}
void SetDutyCycle(uint16_t dutyCycle)
{
    // Ensure duty cycle is within bounds

    // Set the duty cycle by updating the CCR2 register
    TIM2->CCR2 = dutyCycle;
    TIM2->CCR1 = dutyCycle;
}
void increase_tick(void)
{
    tick++;
}
void Init_Systick(uint32_t tick)
{
    SysTick->LOAD = tick; // Count down from 999 to 0
    SysTick->VAL = 0;     // Clear current value
    SysTick->CTRL = 0x7;  // Enable Systick, exception,and use processor clock
}
/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    increase_tick();
    if (pwmDirection)
    {
        counter++;
        if (counter > 1000)
        {
            counter = 999;
            pwmDirection = 0;
        }
    }
    else
    {
        counter--;
        if (counter < 1)
        {
            counter = 0;
            pwmDirection = 1;
        }
    }

}
