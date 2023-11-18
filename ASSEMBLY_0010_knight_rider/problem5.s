/*
 * asm.s
 *
 * author: Fouad Aladhami/Büşra Ülker/Ayşe Beri Çevik
 *
 * description: Added the necessary stuff for turning on the green LED on the 
 *   G031K8 Nucleo board. Mostly for teaching.
 */


.syntax unified
.cpu cortex-m0plus
.fpu softvfp
.thumb


/* make linker see this */
.global Reset_Handler

/* get these from linker script */
.word _sdata
.word _edata
.word _sbss
.word _ebss


/* define peripheral addresses from RM0444 page 57, Tables 3-4 */
.equ RCC_BASE,         (0x40021000)          // RCC base address
.equ RCC_IOPENR,       (RCC_BASE   + (0x34)) // RCC IOPENR register offset


.equ GPIOB_BASE,       (0x50000400)          // GPIOB base address
.equ GPIOB_MODER,      (GPIOB_BASE + (0x00)) // GPIOB MODER register offset
.equ GPIOB_ODR,        (GPIOB_BASE + (0x14)) // GPIOB ODR register offset

.equ GPIOA_BASE,       (0x50000000)          // GPIOA base address?
.equ GPIOA_MODER,      (GPIOA_BASE + (0x00)) // GPIOA MODER register offset
.equ GPIOA_ODR,        (GPIOA_BASE + (0x14)) // GPIOA ODR register offset
.equ GPIOA_IDR,        (GPIOA_BASE + (0x10)) // GPIOA IDR register offset

.equ DELAY_FREQ,		(16000000/30)


/* vector table, +1 thumb mode */
.section .vectors
vector_table:
	.word _estack             /*     Stack pointer */
	.word Reset_Handler +1    /*     Reset handler */
	.word Default_Handler +1  /*       NMI handler */
	.word Default_Handler +1  /* HardFault handler */
	/* add rest of them here if needed */


/* reset handler */
.section .text
Reset_Handler:
	/* set stack pointer */
	ldr r0, =_estack
	mov sp, r0

	/* initialize data and bss 
	 * not necessary for rom only code 
	 * */
	bl init_data
	/* call main */
	bl main
	/* trap if returned */
	b .


/* initialize data and bss sections */
.section .text
init_data:

	/* copy rom to ram */
	ldr r0, =_sdata
	ldr r1, =_edata
	ldr r2, =_sidata
	movs r3, #0
	b LoopCopyDataInit

	CopyDataInit:
		ldr r4, [r2, r3]
		str r4, [r0, r3]
		adds r3, r3, #4

	LoopCopyDataInit:
		adds r4, r0, r3
		cmp r4, r1
		bcc CopyDataInit

	/* zero bss */
	ldr r2, =_sbss
	ldr r4, =_ebss
	movs r3, #0
	b LoopFillZerobss

	FillZerobss:
		str  r3, [r2]
		adds r2, r2, #4

	LoopFillZerobss:
		cmp r2, r4
		bcc FillZerobss

	bx lr


/* default handler */
.section .text
Default_Handler:
	b Default_Handler

/* Function to initialize a GPIO pin */
init_gpio:
  // Parameters: r0 = GPIO base address, r1 = pin number, r2 = mode
  ldr r3, [r0]      // Load the current GPIO MODER register value
  lsls r1, r1, #1   //Multiply pin number by 2 for shifting moder mode value
  lsls r2, r2, r1      // Shift the mode bits to the position of the pin
  movs r4, #0x3 //define r4 mask
  lsls r4, r4, r1 //shift 0x11 mask to the pin register
  bics r3, r3, r4      // Clear the bits for this pin
  orrs r3, r3, r2      // Set the mode bits for this pin
  str r3, [r0]      // Store the updated value back to the MODER register
  bx lr
/* Function to turn on an LED */
turn_on_led:
  // Parameter: r0 = GPIO base address, r1 = pin number
  ldr r2, [r0,#0x14]   // Load the ODR (output data register)
  ldr r3, =#0x01
  lsls r3, r3, r1       // Create a bitmask for the pin
  orrs r2, r2, r3       // Set the pin to 1 (turn on the LED)
  str r2, [r0,#0x14]   // Store the updated value back to the ODR register
  bx lr

/* Function to turn off an LED */
turn_off_led:
  // Parameter: r0 = GPIO base address, r1 = pin number
  ldr r2, [r0, #0x14]   // Load the ODR (output data register)
  ldr r3, =#0x01
  lsls r3, r3, r1      // Create a bitmask for the pin
  bics r2, r2, r3       // Clear the pin to 0 (turn off the LED)
  str r2, [r0, #0x14]   // Store the updated value back to the ODR register
  bx lr

rcc_gpio_clock_enable:
	// Parameters: r0 = RCC_IOPENR base address, r1 = GPIO port
	ldr r3, [r0] //Load the RCC_IOPENR value
	//make a mask for the port
	movs r4, #0x1
	lsls r4, r4, r1
	//set the port bit
	orrs r3, r3, r4
	//store the updated value back to the RCC_IOPENR register
	str r3, [r0]
	bx lr

delay_loop:
	//load counter to r0 register
	subs r0, r0, #1 //This took 1 cycle
    bne delay_loop // This took 3 cycle
    bx lr

delay_loop1:
	//load counter to r0 register
	subs r0, r0, #1 //This took 1 cycle
    bne delay_loop1 // This took 3 cycle
    b left_shift

delay_loop2:
	//load counter to r0 register
	subs r0, r0, #1 //This took 1 cycle
    bne delay_loop2 // This took 3 cycle
    b right_shift


/* main function */
.section .text
main:
  	/*Enable GPIOA clock, bit 0 on IOPENR*/
  	ldr r0, =RCC_IOPENR
	movs r1, #0 //set r1 to 1 for GPIOA
	bl rcc_gpio_clock_enable //call the rcc_gpio_clock_enable function
	/*Enable GPIOB clock, bit 1 on IOPENR*/
  	ldr r0, =RCC_IOPENR
	movs r1, #1 //set r1 to 1 for GPIOB
	bl rcc_gpio_clock_enable //call the rcc_gpio_clock_enable function

  	/* Set up PB0- PB7 for the LED (bits 16-17 in MODER)*/
  	ldr r6, =GPIOB_MODER
  	ldr r5, [r6]
  	ldr r4, =0xFFFF
  	bics r5, r5, r4 //erase bits 16 and 17
  	ldr r4, =0x5555
  	orrs r5, r5, r4 // write 01 to the bits
  	str r5, [r6]

  	ldr r0, =GPIOA_MODER
  	movs r1, #0 //0 for active pin 0
  	movs r2, #0 //0 for input mode
  	bl init_gpio

	initstate:
		ldr r0, =GPIOB_BASE
		movs r1, #0 //0 for pin 0
		bl turn_on_led

		ldr r0, =DELAY_FREQ
		bl delay_loop

		ldr r0, =GPIOB_BASE
		movs r1, #1 //1 for pin 1
		bl turn_on_led

		ldr r0, =DELAY_FREQ
		bl delay_loop

		ldr r0, =GPIOB_BASE
		movs r1, #2 //2 for pin 2
		bl turn_on_led

		ldr r0, =DELAY_FREQ
		bl delay_loop

		ldr r5,=0x00000007 // assign led pattern to r5
		ldr r4,=0x1
		ldr r6,=0x000000E0 //left limit
		ldr r7,=0x00000007 //right limit
	while_loop:
	    cmp r5,r6
	    bne left_shift
	    beq right_shift


	left_shift:
	cmp r5,r6
	beq right_shift
	lsls r5,r5,r4
	ldr r1, =GPIOB_ODR
	str r5, [r1]
	ldr r0, =DELAY_FREQ
	b delay_loop1


	right_shift:
	cmp r5,r7
	beq left_shift
	lsrs r5,r5,r4
	ldr r1, =GPIOB_ODR
	str r5, [r1]
	ldr r0, =DELAY_FREQ
	b delay_loop2
