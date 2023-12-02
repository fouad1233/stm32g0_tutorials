/*
 * asm.s
 *
 * author: Fouad Aladhami/Büşra Ülker/Ayşe Beri Çevik
 *
 * description: Added the necessary stuff for turning on  LEDS with
 *   G031K8 Nucleo board.
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


/* main function */
.section .text
main:
	    ldr     R0, =0xE1EC       // Load R0 with 0xE1EC
	    LDR     R1, =0x0334       // Load R1 with 0x0334
	    MOVS    R2, #0            // Initialize result R2 to 0
	    MOVS    R4, #0            // Clear R4, will be used for left shifts of R0

	multiply:
	    TST     R1, #1            // Test LSB of R1

	    ADDNE   R2, R2, R0        // Add R0 to R2 if LSB of R1 is 1
	    LSLS    R0, R0, #1        // Logical shift left R0
	    ADC     R4, R4        // Add carry to R4 (for high part of R0)
	    LSRS    R1, R1, #1        // Logical shift right R1
	    BNE     multiply          // Continue if R1 is not zero
	// Now R2 contains the lower 32 bits and R4 contains the higher 32 bits of the result

	b .

