/*
 * asm.s
 *
 * author: Fouad Aladhami
 *
 *
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

.equ GPIOC_BASE,       (0x50000800)          // GPIOC base address
.equ GPIOC_MODER,      (GPIOC_BASE + (0x00)) // GPIOC MODER register offset
.equ GPIOC_ODR,        (GPIOC_BASE + (0x14)) // GPIOC ODR register offset


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

/* main function */
.section .text
main:
	ldr r0, =RCC_IOPENR
	movs r1, #2 //set r1 to 2 for GPIOC
	bl rcc_gpio_clock_enable //call the rcc_gpio_clock_enable function
	ldr r0, =GPIOC_BASE   // Load the GPIO base address into r0
	movs r1, #6            // Set r1 to the pin number (e.g., PC6)
	movs r2, #1            // Set r2 to the mode (0 for general purpose output)
	bl init_gpio           // Call the init_gpio function
	ldr r0, =GPIOC_BASE     // Load the GPIO base address into r0
	movs r1, #6            // Set r1 to the pin number (e.g., PC6)
	bl turn_on_led         // Call the turn_on_led function


	/* for(;;); */
	b .

	/* this should never get executed */
	nop
