/*
 * asm.s
 *
 * author: Fouad Aladhami
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


.equ GPIOA_BASE,       (0x50000000)          // GPIOA base address?
.equ GPIOA_MODER,      (GPIOA_BASE + (0x00)) // GPIOA MODER register offset
.equ GPIOA_ODR,        (GPIOA_BASE + (0x14)) // GPIOA ODR register offset

.equ DELAY_FREQ,		(16000000/3)

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


/* main function */
.section .text

main:
  /*Enable GPIOA clock, bit 0 on IOPENR*/
  ldr r6, =RCC_IOPENR
  ldr r5, [r6]
  movs r4, 0x1
  orrs r5, r5, r4
  str r5, [r6]

  /* Set up PA8 for the LED (bits 16-17 in MODER)*/
  ldr r6, =GPIOA_MODER
  ldr r5, [r6]
  ldr r4, =0x30000
  bics r5, r5, r4 //erase bits 16 and 17
  ldr r4, =0x10000
  orrs r5, r5, r4 // write 01 to the bits
  str r5, [r6]

  /* Blink the LED*/
  blink_loop:
  	ldr r6, =GPIOA_ODR
    /*Read the current state of the LED*/
    ldr r5, [r6]
    /*Set r4 to the value 0x100 (bit 8)*/
    ldr r4, =0x100
    //Toggle the LED state by XORing with 0x100
    eors r5, r5, r4
   	// Update the LED state
    str r5, [r6]

    // Delay between LED state changes (adjust for desired speed)
    ldr r0, =DELAY_FREQ
    delay_loop:
      subs r0, r0, #1 //This took 1 cycle
      bne delay_loop // This took 3 cycle

    // Repeat the blinking loop
    b blink_loop
