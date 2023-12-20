#include "stm32g0xx.h"
#include "stm32g031xx.h"
/*
Setting up UART and creating a print() function
Pins PA2 and PA3 are connected to the ST/Link Debugge IC on the back which gets passed to the PC over USB as a Virtual COM port. This can be seen from the schematic of the board. First, we need to determine the USART modules PA2 and PA3 are connected to. From stm32g031k8 datasheet, we can see that we have two USART modules named USART1 and USART2.
Looking at Table 13 - Port A alternate function mapping, we can also see that PA2 and PA3 are connected to USART2 module using Alternate Function 1 mode (AF1). Now that we have the relevant info, we can start initializing steps. Check the RCC, GPIO and USART sections from rm0444 reference manual for exact registers and relevant bits.
1. enable GPIOA and USART2 module clocks from RCC
2. enable and setup pins PA2 and PA3 as alternate function for
	 USART2 module
3. enable receive and transmit from USART2 module
4. set baud rate to 9600 bps assuming clock is running at 16Mhz
5. enable uart module
At this point, if we open a serial port from PC, and send a character from MCU, we should be able to see. Let’s write some helpers for this purpose.
6. implement a printChar function given as below. Now you can just call printChar('a'); from your main loop and this should be displayed on your PC terminal
7.implement a _print function that will call the printChar function for the given string in a for loop given as below. Now you can call _print(0, "Hello\n", 6); and it should be displayed on your PC terminal
8. implement a print function that will calculate the number of characters in a string and call _print function given as below. Now you can just call print("Hello\n"); and it should be displayed on your PC terminal

********************************
 step 6 skeleton void
 printChar(uint8_t c) {
 write c to the transmit data register
 wait until transmit is complete
}
********************************
step 7 skeleton int _print(int f,
char *ptr, int len)
{
(void)f; // don't do anthing with f
 call printChar within a for loop len 􏰀mes return len; // return length
}
********************************
step 8 skeleton void
print(char *s) {
count number of characters in s string un􏰀l a null byte comes `\0` _print(0, s, length);
}

**************
In order to connect to your MC and see these, you can open a serial port from your PC. When you connect the device, it should create a COM port for Windows and /dev/tty* for Unix like systems (ACM / USB). You can use STM32CubeIDE.
1.From the Console window, press the window icon with plus symbol, then choose Command Shell Console
2. For connection type, choose Serial Port
3.Hit new and create a connection. Give a connection name,
choose correct uart setup, and choose serial port. If your device is connected a COM port should show up here. Choose that.
4. Choose encoding as UTF-8 and hit OK.
You should now have a serial connection to your board.
**************
Problem 1. In this problem, you will connect your board to the PC using UART protocol. For this you will need to create an initialization routine for UART, and create send and receive functions. You should not use interrupts for this assignment. List the possible transmit and receive bit rates for UART serial communication according to the EISA RS-232 communication Standard.
Basically, create two functions as given below
void uart_tx(uint8_t c); uint8_t uart_rx(void);
and in your main loop, call them like
while(1) { uart_tx(uart_rx()); }

*/
void RCC_Init(void);
void GPIOA_Init(void);
void USART2_Init(void);
void printChar(uint8_t c);
int _print(int f, char *ptr, int len);
void print(char *s);
uint8_t uart_rx(void);
void uart_tx(uint8_t c);

int main(void)
{
	RCC_Init();
	GPIOA_Init();
	USART2_Init();

	while (1)
	{
		uart_tx(uart_rx());
	}
}

void RCC_Init(void)
{
	// enable GPIOA and USART2 module clocks from RCC
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	RCC->APBENR1 |= RCC_APBENR1_USART2EN;
}
void GPIOA_Init(void)
{
	// enable and setup pins PA2 and PA3 as alternate function for USART2 module
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL3_0);
}
void USART2_Init(void)
{
	// enable receive and transmit from USART2 module
	USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE);
	// set baud rate to 9600 bps assuming clock is running at 16Mhz
	// Baud Rate = APBxCLK / USARTx->BRR
	USART2->BRR = 0x683; // 1667
	// enable uart module
	USART2->CR1 |= USART_CR1_UE;
}
void printChar(uint8_t c)
{
	// write c to the transmit data register
	USART2->TDR = c;
	// wait until transmit is complete
	while (!(USART2->ISR & USART_ISR_TC))
		;
}
int _print(int f, char *ptr, int len)
{
	(void)f; // don't do anthing with f
	// call printChar within a for loop len times
	for (int i = 0; i < len; i++)
	{
		printChar(ptr[i]);
	}
	return len; // return length
}
void print(char *s)
{
	// count number of characters in s string until a null byte comes `\0`
	int length = 0;
	while (s[length] != '\0')
	{
		length++;
	}
	_print(0, s, length);
}
uint8_t uart_rx(void)
{
	// wait until receive is complete
	while (!(USART2->ISR & USART_ISR_RXNE_RXFNE))
		;
	// return the received data
	return USART2->RDR;
}
void uart_tx(uint8_t c)
{
	// write c to the transmit data register
	USART2->TDR = c;
	// wait until transmit is complete
	while (!(USART2->ISR & USART_ISR_TC))
		;
}
