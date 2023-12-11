#include "stm32g0xx.h"


void clearRowsKeypad(void);
void setRowsKeypad (void);

void EXTI4_15IRQHandler(void){
   clearRowsKeypad();
   GPIOA->ODR^=(1U<<4);
   if((EXTI->RPR1 >> 4) & 1){

      EXTI->RPR1|=(1U<<4);

    }
   GPIOA->ODR^=(1U<<4);

   GPIOA->ODR^=(1U<<5);
   if ((EXTI->RPR1 >> 5) & 1){

         EXTI->RPR1|=(1U<<5);

      }
   if ((EXTI->RPR1 >> 6) & 1){

         EXTI->RPR1|=(1U<<6);

      }
   if ((EXTI->RPR1 >> 7) & 1){

         EXTI->RPR1|=(1U<<7);

      }
   setRowsKeypad();
}


int main(void){

RCC->IOPENR |=(1U<<1);/*Enable GPIOB clock*/

/*Set up PB0,PB1,PB2,PB3 as output row(output=01)*/
GPIOB->MODER &=~(3U<<0);
GPIOB->MODER |=(1U<<0);/*for PB0*/

GPIOB->MODER &=~(3U<<2);
GPIOB->MODER |=(1U<<2);/*for PB1*/

GPIOB->MODER &=~(3U<<4);
GPIOB->MODER |=(1U<<4);/*for PB2*/

GPIOB->MODER &=~(3U<<6);
GPIOB->MODER |=(1U<<6);/*for PB3*/


/*Set up PB4,PB5,PB6,PB7 as input column(input=00)*/
GPIOB->MODER &=~(3U<<8);/*for PB4*/
GPIOB->PUPDR |=(2U<<8);/**/

GPIOB->MODER &=~(3U<<10);/*for PB5*/
GPIOB->PUPDR |=(2U<<10);

GPIOB->MODER &=~(3U<<12);/*for PB6*/
GPIOB->PUPDR |=(2U<<10);

GPIOB->MODER &=~(3U<<14);/*for PB7*/
GPIOB->PUPDR |=(2U<<10);


/*Set up interrupt for input*/
EXTI->EXTICR[1] |=(1U<<0);/*PB4*/
EXTI->EXTICR[1] |=(1U<<8);/*PB5*/
EXTI->EXTICR[1] |=(1U<<16);/*PB6*/
EXTI->EXTICR[1] |=(1U<<24);/*PB7*/


/*Rising edge*/
EXTI->RTSR1|=(1U<<4);
EXTI->RTSR1 |=(1U<<5);
EXTI->RTSR1|=(1U<<6);
EXTI->RTSR1|=(1U<<7);

/*MASK*/
EXTI->IMR1 |=(1U<<4);
EXTI->IMR1 |=(1U<<5);
EXTI->IMR1 |=(1U<<6);
EXTI->IMR1 |=(1U<<7);

/*NVIC*/

NVIC_SetPriority(EXTI4_15_IRQn,0);
NVIC_EnableIRQ(EXTI4_15_IRQn);


/*Set all rows*/
GPIOB->ODR |=(1U<<0);/*PB0*/
GPIOB->ODR |=(1U<<1);/*PB1*/
GPIOB->ODR |=(1U<<2);/*PB2*/
GPIOB->ODR |=(1U<<3);/*PB3*/


while(1){

}
return 0;

}

void clearRowsKeypad(void){
	/*Clearing the rows*/
	GPIOB->ODR &=(1U<<0);/*PB0*/
	GPIOB->ODR &=(1U<<1);/*PB1*/
	GPIOB->ODR &=(1U<<2);/*PB2*/
	GPIOB->ODR &=(1U<<3);/*PB3*/

}


void setRowsKeypad(void){
	/*Setting rows*/
	GPIOB->ODR |=(1U<<0);/*PB0*/
	GPIOB->ODR |=(1U<<1);/*PB1*/
	GPIOB->ODR |=(1U<<2);/*PB2*/
	GPIOB->ODR |=(1U<<3);/*PB3*/

}
