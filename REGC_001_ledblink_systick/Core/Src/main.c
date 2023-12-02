#include "stm32g0xx.h"
#include "stm32g031xx.h"


//variables
uint64_t tick;
uint8_t seconds;



//functions prototypes
void delay(volatile uint32_t);
void Init_Systick(uint32_t tick);
void Systick_Handler(void);
void increase_tick(void);
uint64_t getTick(void);
void delay_ms(uint64_t msvalue);

int main(void) {

	//init
	Init_Systick(16000);

	while(1) {
        delay_ms(1000);
        seconds ++;
    }

    return 0;
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

