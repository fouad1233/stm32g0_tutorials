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

.section .data
    // Define the 128-bit numbers in little-endian format
    num1:
        .word 0x01020304
        .word 0x05060708
        .word 0x090A0B0C
        .word 0x0D0E0F10

    num2:
        .word 0x01000000
        .word 0x02000000
        .word 0x03000000
        .word 0x04000000
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
	movs r0,num2

	ldr r7, [r0]
    ldr r6, [r0, #4]
    ldr r5, [r0, #8]
    ldr r4, [r0, #12]

	ldr r11,r7
	movs r10,r6
	movs r9,r5
	movs r8,r4
    // Load the second operand into registers R11-R8
    ldr r11, [num2]
    ldr r10, [num2, #4]
    ldr r9, [num2, #8]
    ldr r8, [num2, #12]

    // Subtract the second operand from the first operand
    subs r3, r7, r11
    sbc r2, r6, r10
    sbc r1, r5, r9
    sbc r0, r4, r8
	b .

