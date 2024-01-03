#include "stm32g0xx.h"
#include "stm32g031xx.h"
#include <stdint.h>

void hamming_distance(uint32_t a,uint32_t b);

uint32_t count=0;//to keep the amount of differ bit(counter)
uint32_t number1 =0xEEEEEEEE;//11101110111011101110111011101110(binary)
uint32_t number2 =0xFFFFFFFF;//11111111111111111111111111111111(binary)

//Main function
int main(void){
	hamming_distance(number1,number2);
	//The function is called according to the numbers specified in global
}
//Hamming distance Function
void hamming_distance(uint32_t a, uint32_t b) {
    uint32_t xor_result = a ^ b;//XOR operation converts different bits to 1

    while (xor_result != 0) {//In cases where the numbers are not 0, the numbers continue to be compared
        count+= xor_result & 0x1;//check if the last bit is 1
        //if the last bit is 1, it indicates that the numbers in that bit are different in both numbers
        xor_result >>= 1;
    }
}
//there are 8different bits
//0x8=8(decimal)




