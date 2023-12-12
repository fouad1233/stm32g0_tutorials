#include "stm32g0xx.h"
#include <math.h>

void clearRowsKeypad(void);
void setRowsKeypad (void);
void storenumber(int sayı);
uint32_t number;//number of stored digits
uint32_t count;//duty cycle rate
uint32_t n;
uint32_t ust;
int numbers[]={0,0,0,0,0};//store digit

void EXTI4_15IRQHandler(void){


   if((EXTI->RPR1 >> 4) & 1){//Interrupt from PA4
	   clearRowsKeypad();

	   GPIOB->ODR^=(1U<<4);//PB4
	   if ((GPIOA->IDR>>4)&1){
		   storenumber(1);

	   }
	   GPIOB->ODR^=(1U<<4);//PB4

	   GPIOB->ODR^=(1U<<5);//PB5
	   if ((GPIOA->IDR>>4)&1){
		   storenumber(4);

	   }
	   GPIOB->ODR^=(1U<<5);//PB5

	   GPIOB->ODR^=(1U<<6);//PB6
	   if ((GPIOA->IDR>>4)&1){
		   storenumber(7);

	   }
	   GPIOB->ODR^=(1U<<6);//PB6

	   GPIOB->ODR^=(1U<<7);//PB7
	   if ((GPIOA->IDR>>4)&1){
		   storenumber(11);

	   }
	   GPIOB->ODR^=(1U<<7);//PB7


       EXTI->RPR1|=(1U<<4);//clear interrupt flag
       setRowsKeypad();
    }

   if ((EXTI->RPR1 >> 5) & 1){//Interrupt from PA5

	   clearRowsKeypad();
	   GPIOB->ODR^=(1U<<4);//PB4
	   if ((GPIOA->IDR>>5)&1){
	   	  storenumber(2);
       }
	   GPIOB->ODR^=(1U<<4);//PB4

	   GPIOB->ODR^=(1U<<5);//PB5
	   if ((GPIOA->IDR>>5)&1){
	      storenumber(5);
       }
	   GPIOB->ODR^=(1U<<5);//PB5

	   GPIOB->ODR^=(1U<<6);//PB6
	   if ((GPIOA->IDR>>5)&1){
	   	  storenumber(8);

	   }
	   GPIOB->ODR^=(1U<<6);//PB6

	   GPIOB->ODR^=(1U<<7);//PB7
	   if ((GPIOA->IDR>>5)&1){
	   	  storenumber(0);

	    }
	   	GPIOB->ODR^=(1U<<7);//PB7
	    EXTI->RPR1|=(1U<<5);//clear interrupt flag
	    setRowsKeypad();
}




   if ((EXTI->RPR1 >> 6) & 1){//Interrupt from PA6
       clearRowsKeypad();
       GPIOB->ODR^=(1U<<4);//PB4
	   if ((GPIOA->IDR>>6)&1){
		   storenumber(3);

	   }
	   GPIOB->ODR^=(1U<<4);//PB4

	   GPIOB->ODR^=(1U<<5);//PB5
	   if ((GPIOA->IDR>>6)&1){
		   storenumber(6);

	   }
	   GPIOB->ODR^=(1U<<5);//PB5

	   GPIOB->ODR^=(1U<<6);//PB6
	   if ((GPIOA->IDR>>6)&1){
		   storenumber(9);

	   }
	   GPIOB->ODR^=(1U<<6);//PB6

	   GPIOB->ODR^=(1U<<7);//PB7
	   if ((GPIOA->IDR>>6)&1){
		   storenumber(11);

	   }
	   GPIOB->ODR^=(1U<<7);//PB7


	   EXTI->RPR1|=(1U<<6);//clear interrupt flag
	   setRowsKeypad();



}
   if ((EXTI->RPR1 >> 7) & 1){//Interrupt from PA7
        clearRowsKeypad();
        GPIOB->ODR^=(1U<<4);//PB4
        	   if ((GPIOA->IDR>>7)&1){
        		   storenumber(11);

        	   }
        	   GPIOB->ODR^=(1U<<4);//PB4

        	   GPIOB->ODR^=(1U<<5);//PB5
        	   if ((GPIOA->IDR>>7)&1){
        		   storenumber(11);

        	   }
        	   GPIOB->ODR^=(1U<<5);//PB5

        	   GPIOB->ODR^=(1U<<6);//PB6
        	   if ((GPIOA->IDR>>7)&1){
        		   storenumber(11);

        	   }
        	   GPIOB->ODR^=(1U<<6);//PB6

        	   GPIOB->ODR^=(1U<<7);//PB7
        	   if ((GPIOA->IDR>>7)&1){
        		   storenumber(11);

        	   }
        	   GPIOB->ODR^=(1U<<7);//PB7


               EXTI->RPR1|=(1U<<7);//clear interrupt flag
               setRowsKeypad();


      }

}


int main(void){

RCC->IOPENR |=(1U<<0);/*Enable GPIOA clock*/
RCC->IOPENR |=(1U<<1);/*Enable GPIOB clock*/

/*Set up PB4,PB5,PB6,PB7 as output row(output=01)*/
GPIOB->MODER &=~(3U<<8);
GPIOB->MODER |=(1U<<8);/*for PB4*/

GPIOB->MODER &=~(3U<<10);
GPIOB->MODER |=(1U<<10);/*for PB5*/

GPIOB->MODER &=~(3U<<12);
GPIOB->MODER |=(1U<<12);/*for PB6*/

GPIOB->MODER &=~(3U<<14);
GPIOB->MODER |=(1U<<14);/*for PA7*/


/*Set up PA4,PA5,PA6,PA7 as input column(input=00)*/
GPIOA->MODER &=~(3U<<8);/*for PA4*/
GPIOA->PUPDR |=(2U<<8);/*Pull down mode*/

GPIOA->MODER &=~(3U<<10);/*for PA5*/
GPIOA->PUPDR |=(2U<<10);/*Pull down mode*/

GPIOA->MODER &=~(3U<<12);/*for PA6*/
GPIOA->PUPDR |=(2U<<12);/*Pull down mode*/

GPIOA->MODER &=~(3U<<14);/*for PA7*/
GPIOA->PUPDR |=(2U<<14);/*Pull down mode*/


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
GPIOA->ODR |=(1U<<4);/*PB4*/
GPIOA->ODR |=(1U<<5);/*PB5*/
GPIOA->ODR |=(1U<<6);/*PB6*/
GPIOA->ODR |=(1U<<7);/*PB7*/


while(1){

}
return 0;

}



void storenumber(int sayı){
	if (sayı>=0 && sayı<=9 && numbers[0]!=0){
		numbers[number]=sayı;
		number=number+1;
	}
	else if(sayı>=0 && sayı<=9 && numbers[0]==0){
		number=0;
	}
	else if(sayı==10 && number!=0){
        int i;

		for(i=0;i<number-1;i++){
			ust=pow(10,number-i-1);
			count=count+numbers[i]*ust;

		}
	    number=0;
	    numbers[0]=0;
	    numbers[1]=0;
	    numbers[2]=0;
	    numbers[3]=0;
	    numbers[4]=0;

	}

	else if(sayı==10 && number==0){
		number=0;
		numbers[0]=0;
		numbers[1]=0;
		numbers[2]=0;
		numbers[3]=0;
		numbers[4]=0;
	}
	else{
		number=0;
	    numbers[0]=0;
		numbers[1]=0;
		numbers[2]=0;
		numbers[3]=0;
		numbers[4]=0;
}


}

void clearRowsKeypad(void){
	/*Clearing the rows*/
	GPIOA->ODR &=~(1U<<4);/*PB4*/
	GPIOA->ODR &=~(1U<<5);/*PB5*/
	GPIOA->ODR &=~(1U<<6);/*PB6*/
	GPIOA->ODR &=~(1U<<7);/*PB7*/

}


void setRowsKeypad(void){
	/*Setting rows*/
	GPIOA->ODR |=(1U<<4);/*PB4*/
	GPIOA->ODR |=(1U<<5);/*PB5*/
	GPIOA->ODR |=(1U<<6);/*PB6*/
	GPIOA->ODR |=(1U<<7);/*PB7*/

}
