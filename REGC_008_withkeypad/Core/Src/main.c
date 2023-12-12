#include "stm32g0xx.h"


void clearRowsKeypad(void);
void setRowsKeypad (void);
void storenumber(int sayı);
uint32_t number;
uint32_t count;
uint32_t n;
uint32_t ust;
int numbers[]={};

void EXTI4_15IRQHandler(void){


   if((EXTI->RPR1 >> 4) & 1){//Interrupt from PA4
	   clearRowsKeypad();

	   GPIOA->ODR^=(1U<<0);//PA0
	   if ((GPIOA->IDR>>4)&1){
		   storenumber(1);

	   }
	   GPIOA->ODR^=(1U<<0);//PA0

	   GPIOA->ODR^=(1U<<1);//PA1
	   if ((GPIOA->IDR>>4)&1){
		   storenumber(4);

	   }
	   GPIOA->ODR^=(1U<<1);//PA1

	   GPIOA->ODR^=(1U<<2);//PA2
	   if ((GPIOA->IDR>>4)&1){
		   storenumber(7);

	   }
	   GPIOA->ODR^=(1U<<2);//PA2

	   GPIOA->ODR^=(1U<<3);//PA3
	   if ((GPIOA->IDR>>4)&1){
		   storenumber(12);

	   }
	   GPIOA->ODR^=(1U<<2);//PA2


       EXTI->RPR1|=(1U<<4);//clear interrupt flag
       setRowsKeypad();
    }

   if ((EXTI->RPR1 >> 5) & 1){//Interrupt from PA5

	   clearRowsKeypad();

	   	   GPIOA->ODR^=(1U<<0);//PA0
	   	   if ((GPIOA->IDR>>5)&1){
	   		   storenumber(2);

	   	   }
	   	   GPIOA->ODR^=(1U<<0);//PA0

	   	   GPIOA->ODR^=(1U<<1);//PA1
	   	   if ((GPIOA->IDR>>5)&1){
	   		   storenumber(5);

	   	   }
	   	   GPIOA->ODR^=(1U<<1);//PA1

	   	   GPIOA->ODR^=(1U<<2);//PA2
	   	   if ((GPIOA->IDR>>5)&1){
	   		   storenumber(8);

	   	   }
	   	   GPIOA->ODR^=(1U<<2);//PA2

	   	   GPIOA->ODR^=(1U<<3);//PA3
	   	   if ((GPIOA->IDR>>5)&1){
	   		   storenumber(0);

	   	   }
	   	   GPIOA->ODR^=(1U<<2);//PA2


       EXTI->RPR1|=(1U<<5);//clear interrupt flag
       setRowsKeypad();

      }
   if ((EXTI->RPR1 >> 6) & 1){//Interrupt from PA6

	   clearRowsKeypad();
       GPIOA->ODR^=(1U<<0);//PA0
	   if ((GPIOA->IDR>>6)&1){
		   storenumber(3);
         }
	   GPIOA->ODR^=(1U<<0);//PA0

	   GPIOA->ODR^=(1U<<1);//PA1
	   if ((GPIOA->IDR>>6)&1){
		   storenumber(6);

	   }
	   GPIOA->ODR^=(1U<<1);//PA1

	   GPIOA->ODR^=(1U<<2);//PA2
	   if ((GPIOA->IDR>>6)&1){
		   storenumber(9);

	   	}
	    GPIOA->ODR^=(1U<<2);//PA2

	   	GPIOA->ODR^=(1U<<3);//PA3
	   	if ((GPIOA->IDR>>6)&1){
	   		storenumber(10);//like #

	   	}
	   	GPIOA->ODR^=(1U<<2);//PA2

        EXTI->RPR1|=(1U<<6);//clear interrupt flag
        setRowsKeypad();

      }
   if ((EXTI->RPR1 >> 7) & 1){//Interrupt from PA7
        clearRowsKeypad();

	    GPIOA->ODR^=(1U<<0);//PA0
	   	if ((GPIOA->IDR>>7)&1){
	   		storenumber(11);

	    }
	   	GPIOA->ODR^=(1U<<0);//PA0

	   	GPIOA->ODR^=(1U<<1);//PA1
	    if ((GPIOA->IDR>>7)&1){
	    	storenumber(11);

	   	}
	   	GPIOA->ODR^=(1U<<1);//PA1

	   	GPIOA->ODR^=(1U<<2);//PA2
	   	if ((GPIOA->IDR>>7)&1){
	   		storenumber(11);

	   	}
	   	GPIOA->ODR^=(1U<<2);//PA2

	   	GPIOA->ODR^=(1U<<3);//PA3
	   	if ((GPIOA->IDR>>7)&1){
	   		storenumber(11);

	   	}
	   	GPIOA->ODR^=(1U<<2);//PA2

        EXTI->RPR1|=(1U<<7);//clear interrupt flag
        setRowsKeypad();

      }

}


int main(void){

RCC->IOPENR |=(1U<<0);/*Enable GPIOA clock*/

/*Set up PA0,PA1,PA2,PA3 as output row(output=01)*/
GPIOA->MODER &=~(3U<<0);
GPIOA->MODER |=(1U<<0);/*for PA0*/

GPIOA->MODER &=~(3U<<2);
GPIOA->MODER |=(1U<<2);/*for PA1*/

GPIOA->MODER &=~(3U<<4);
GPIOA->MODER |=(1U<<4);/*for PA2*/

GPIOA->MODER &=~(3U<<6);
GPIOA->MODER |=(1U<<6);/*for PA3*/


/*Set up PA4,PA5,PA6,PA7 as input column(input=00)*/
GPIOA->MODER &=~(3U<<8);/*for PA4*/
GPIOA->PUPDR |=(2U<<8);/*Pull down mode*/

GPIOA->MODER &=~(3U<<10);/*for PA5*/
GPIOA->PUPDR |=(2U<<10);/*Pull down mode*/

GPIOA->MODER &=~(3U<<12);/*for PA6*/
GPIOA->PUPDR |=(2U<<10);/*Pull down mode*/

GPIOA->MODER &=~(3U<<14);/*for PA7*/
GPIOA->PUPDR |=(2U<<10);/*Pull down mode*/


/*Set up interrupt for input*/
EXTI->EXTICR[1] |=(0U<<0);/*PA4*/
EXTI->EXTICR[1] |=(0U<<8);/*PA5*/
EXTI->EXTICR[1] |=(0U<<16);/*PA6*/
EXTI->EXTICR[1] |=(0U<<24);/*PA7*/


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


/*Set all rows*//*tuşların hepsi aktif 5v oldu*/
GPIOA->ODR |=(1U<<0);/*PA0*/
GPIOA->ODR |=(1U<<1);/*PA1*/
GPIOA->ODR |=(1U<<2);/*PA2*/
GPIOA->ODR |=(1U<<3);/*PA3*/


while(1){

}
return 0;

}



void storenumber(int sayı){
	if (sayı>=0 && sayı<=9){
		numbers[number]=sayı;
		number=number+1;
	}
	else if(sayı==10 && number!=0){
        int i;
        n=sizeof(number)-1;
		for(i=0;i<sizeof(number)-1;i++){
			ust=10^n;
			count=count+numbers[i]*ust;
			n=n-1;

		}
	    number=0;

	}

	else if(sayı==10 && number==0){
		number=0;
	}
	else{
		number=0;
	}


}

void clearRowsKeypad(void){
	/*Clearing the rows*/
	GPIOA->ODR &=~(1U<<0);/*PA0*/
	GPIOA->ODR &=~(1U<<1);/*PA1*/
	GPIOA->ODR &=~(1U<<2);/*PA2*/
	GPIOA->ODR &=~(1U<<3);/*PA3*/

}


void setRowsKeypad(void){
	/*Setting rows*/
	GPIOA->ODR |=(1U<<0);/*PA0*/
	GPIOA->ODR |=(1U<<1);/*PA1*/
	GPIOA->ODR |=(1U<<2);/*PA2*/
	GPIOA->ODR |=(1U<<3);/*PA3*/

}
