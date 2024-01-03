#include "stm32g0xx.h"
#include "stm32g031xx.h"
#include <stdint.h>

void hamming_weight(uint32_t input);

uint32_t counter=0;//to keep the amount of 1s counted
uint32_t number =0x01234567;//00000001001000110100010101100111(binary)

int main(void){
	hamming_weight(number);
	//The function is called according to the number specified in global
}

//Hamming function
void hamming_weight(uint32_t input) {
    while (input != 0) {// Check if input is zero
        counter += input & 0x01;//// Check the least significant bit
//With the and operation of R1 with 1, if the last bit is 1, the result will be 1 and
//if the last bit is 0, the result will be 1.Add the result to the counter
        input >>= 1;//// Right shift R1 to check the next bit
    }
}
//There are 12 1's in R1
//0xC=12(decimal)




