/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_dcp.S
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2020-2024 Raspberry Pi (Trading) Ltd.
 *
 * Double-precision arithmetic on the RP2350's double-precision coprocessor
 * (DCP, coprocessor 4; RP2350 datasheet 3.6.2): the AEABI run-time helpers
 * (__aeabi_dadd ... __aeabi_d2f) and newlib's __ieee754_sqrt.
 *
 * The coprocessor instruction encodings and the canned sequences (the
 * dcp_*_m macros below) are from the Raspberry Pi Pico SDK
 * (src/rp2_common/hardware_dcp/include/hardware/dcp_instr.inc.S and
 * dcp_canned.inc.S, BSD-3-Clause):
 *
 *   Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the
 *      distribution.
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Interrupts and threads.  The coprocessor holds the state of one
 * operation (two unpacked operands and a status word) and each CPU core
 * has its own.  The Pico SDK tests the "engaged" flag at every entry and
 * saves the state on the stack when it interrupted a sequence, which is
 * enough for interrupt handlers but not for a preemptive SMP kernel: a
 * thread preempted in the middle of a sequence may resume on the other
 * core, or after another thread was itself preempted mid-sequence on this
 * one.  Here every sequence runs with PRIMASK set instead (a few cycles):
 * the coprocessor is never engaged where an interrupt, and so a context
 * switch, can happen, so its state never needs saving and threads may move
 * between cores freely.
 *
 * Exactness: see docs/hardware-accel.md.  Every result is bit for bit
 * the one libgcc's soft-float routines give (tested on the board by
 * dcptest).
 *
 ****************************************************************************/

#include <nuttx/config.h>

	.syntax	unified
	.thumb
	.file	"rp23xx_dcp.S"

/* Names: the run-time helpers, or (RP23XX_DCP_TEST, for apps/testing/
 * dcptest) a set of its own beside libgcc's.
 */

#ifdef RP23XX_DCP_TEST
#  ifndef DCPT
#    define DCPT(x) dcpt_##x
#  endif
#  define AE(x)   DCPT(x)
#  define GN(x)   DCPT(gnu_##x)
#  define LM(x)   DCPT(x)
#  define RP(x)   DCPT(x)
#  define SOFT(x) soft_##x
#else
#  define AE(x)   __aeabi_##x
#  define GN(x)   __##x
#  define LM(x)   x
#  define RP(x)   rp23xx_dcp_##x
#  define SOFT(x) __rp23xx_soft_##x
#endif

/* In the firmware: SRAM, if so configured.  In a loadable module (the
 * library MODULE_RTLIBS names, and dcptest): .text, beside the module's
 * other code and libgcc's, which call these with bl (+-16 MB), so not in
 * .time_critical, which goes to the SRAM, far from the PSRAM's .text.
 */

#if defined(CONFIG_RP23XX_DCP_SRAM) && !defined(RP23XX_DCP_MODULE) && \
    !defined(RP23XX_DCP_TEST)
	.section .time_critical.rp23xx_dcp, "ax", %progbits
#else
	.section .text.rp23xx_dcp, "ax", %progbits
#endif

/****************************************************************************
 * Coprocessor instructions (Pico SDK dcp_instr.inc.S, the ones used here)
 ****************************************************************************/

.macro ADD0
	cdp	p4,#0,c0,c0,c1,#0
.endm
.macro ADD1
	cdp	p4,#1,c0,c0,c1,#0
.endm
.macro SUB1
	cdp	p4,#1,c0,c0,c1,#1
.endm
.macro SQR0
	cdp	p4,#2,c0,c0,c1,#0
.endm
.macro NRDF
	cdp	p4,#8,c0,c0,c2,#1
.endm
.macro NRDD
	cdp	p4,#8,c0,c0,c0,#1
.endm
.macro NTDC
	cdp	p4,#8,c0,c0,c0,#2
.endm
.macro WXMS rl,rh
	mcrr	p4,#2,\rl,\rh,c0
.endm
.macro WXMO rl,rh
	mcrr	p4,#3,\rl,\rh,c0
.endm
.macro WXUP rl,rh
	mcrr	p4,#1,\rl,\rh,c0
.endm
.macro WYUP rl,rh
	mcrr	p4,#1,\rl,\rh,c1
.endm
.macro WXYU rl,rh
	mcrr	p4,#1,\rl,\rh,c2
.endm
.macro WXDD rl,rh
	mcrr	p4,#4,\rl,\rh,c0
.endm
.macro WXDQ rl,rh
	mcrr	p4,#5,\rl,\rh,c0
.endm
.macro WXUC rl,rh
	mcrr	p4,#6,\rl,\rh,c0
.endm
.macro WXIC rl,rh
	mcrr	p4,#7,\rl,\rh,c0
.endm
.macro WXDC rl,rh
	mcrr	p4,#8,\rl,\rh,c0
.endm
.macro RCMP rt
	mrc	p4,#0,\rt,c0,c0,#1
.endm
.macro RDFG rt
	mrc	p4,#0,\rt,c0,c2,#5
.endm
.macro RDIC rt
	mrc	p4,#0,\rt,c0,c3,#0
.endm
.macro RDUC rt
	mrc	p4,#0,\rt,c0,c3,#1
.endm
.macro RXMS rl,rh,s
	mrrc	p4,#\s,\rl,\rh,c4
.endm
.macro RYMS rl,rh,s
	mrrc	p4,#\s,\rl,\rh,c5
.endm
.macro RYMR rl,rh
	mrrc	p4,#2,\rl,\rh,c1
.endm
.macro RXMQ rl,rh
	mrrc	p4,#4,\rl,\rh,c1
.endm
.macro RDDA rl,rh
	mrrc	p4,#1,\rl,\rh,c0
.endm
.macro RDDS rl,rh
	mrrc	p4,#3,\rl,\rh,c0
.endm
.macro RDDM rl,rh
	mrrc	p4,#5,\rl,\rh,c0
.endm
.macro RDDD rl,rh
	mrrc	p4,#7,\rl,\rh,c0
.endm
.macro RDDQ rl,rh
	mrrc	p4,#9,\rl,\rh,c0
.endm
.macro RDDG rl,rh
	mrrc	p4,#11,\rl,\rh,c0
.endm

/****************************************************************************
 * Canned sequences (Pico SDK dcp_canned.inc.S)
 ****************************************************************************/

.macro dcp_dadd_m rzl,rzh,rxl,rxh,ryl,ryh
	WXUP	\rxl,\rxh
	WYUP	\ryl,\ryh
	ADD0
	ADD1
	NRDD
	RDDA	\rzl,\rzh
.endm

.macro dcp_dsub_m rzl,rzh,rxl,rxh,ryl,ryh
	WXUP	\rxl,\rxh
	WYUP	\ryl,\ryh
	ADD0
	SUB1
	NRDD
	RDDS	\rzl,\rzh
.endm

.macro dcp_dmul_m rzl,rzh,rxl,rxh,ryl,ryh,ra,rb,rc,rd,re,rf,rg
	WXUP	\rxl,\rxh
	WYUP	\ryl,\ryh
	RXMS	\ra,\rb,0
	RYMS	\rc,\rd,0
	umull	\re,\rf,\ra,\rc
	movs	\rg,#0
	umlal	\rf,\rg,\ra,\rd
	umlal	\rf,\rg,\rb,\rc
	WXMS	\re,\rf
	movs	\re,#0
	umlal	\rg,\re,\rb,\rd
	WXMO	\rg,\re
	NRDD
	RDDM	\rzl,\rzh
.endm

.macro dcp_ddiv_m rzl,rzh,rxl,rxh,ryl,ryh,ra,rb,rc,rd,re
	WXUP	\rxl,\rxh
	WYUP	\ryl,\ryh
	RYMR	\ra,\rb
	umull	\rb,\rc,\ra,\rb
	mvn	\rc,\rc,lsl #2
	smmlar	\ra,\rc,\ra,\ra
	smmulr	\rc,\rc,\rc
	smmlar	\ra,\rc,\ra,\ra
	sub	\re,\ra,\ra,lsr #31
	RXMS	\rc,\rd,0
	smmulr	\rb,\re,\rd
	RYMS	\rc,\rd,1
	umull	\rc,\ra,\rb,\rc
	mla	\ra,\rb,\rd,\ra
	RXMS	\rc,\rd,4
	sub	\ra,\rc,\ra
	smmulr	\rc,\ra,\re
	mov	\rd,\rb,lsr #4
	adds	\ra,\rc,\rb,lsl #28
	adc	\rb,\rd,\rc,asr #31
	orr	\ra,\ra,\rb,lsr #21
	bic	\ra,\ra,\rb,lsr #22
	bic	\ra,\ra,#7
	RYMS	\rc,\rd,7
	umull	\rd,\re,\ra,\rc
	RYMS	\rc,\rd,7
	mla	\re,\ra,\rd,\re
	mla	\re,\rb,\rc,\re
	RXMS	\rc,\rd,0
	sub	\re,\re,\rc,lsl #18
	orr	\ra,\ra,\re,lsr #29
	sub	\ra,\ra,#1
	WXDD	\ra,\rb
	NRDD
	RDDD	\rzl,\rzh
.endm

.macro dcp_dsqrt_m rzl,rzh,rxl,rxh,ra,rb,rc,rd,re
	WXUP	\rxl,\rxh
	SQR0
	RXMQ	\rc,\rd
	umull	\ra,\rb,\rc,\rc
	umull	\ra,\rb,\rb,\rd
	mov	\rb,\rb,lsl #3
	sub	\rb,\rb,#2147483648
	smmlsr	\rc,\rc,\rb,\rc
	umull	\ra,\rb,\rc,\rc
	umull	\ra,\rb,\rb,\rd
	movs	\ra,\ra,lsr #28
	adc	\rb,\ra,\rb,lsl #4
	smmulr	\ra,\rb,\rc
	sub	\rc,\rc,\ra,asr #1
	umull	\ra,\rb,\rc,\rd
	mov	\rb,\rb,lsr #1
	umull	\rd,\ra,\rb,\rb
	RXMS	\ra,\re,6
	sub	\rd,\ra,\rd
	smmulr	\rd,\rd,\rc
	add	\rb,\rb,\rd,asr #28
	mov	\rd,\rd,lsl #4
	bic	\rd,\rd,#255
	orr	\rd,\rd,#128
	umull	\re,\rc,\rd,\rd
	mul	\re,\rb,\rb
	umlal	\rc,\re,\rb,\rd
	umlal	\rc,\re,\rb,\rd
	sub	\re,\re,\ra
	orr	\rd,\rd,\re,lsr #24
	sub	\rd,\rd,#11
	WXDQ	\rd,\rb
	NRDD
	RDDQ	\rzl,\rzh
.endm

.macro dcp_dcmp_m rz,rxl,rxh,ryl,ryh
	WXUP	\rxl,\rxh
	WYUP	\ryl,\ryh
	ADD0
	RCMP	\rz
.endm

.macro dcp_float2double_m rzl,rzh,rx
	WXYU	\rx,\rx
	NRDD
	RDDG	\rzl,\rzh
.endm

.macro dcp_double2float_m rz,rxl,rxh
	WXUP	\rxl,\rxh
	NRDF
	RDFG	\rz
.endm

.macro dcp_int2double_m rzl,rzh,rx
	WXIC	\rx,\rx
	ADD0
	SUB1
	NRDD
	RDDS	\rzl,\rzh
.endm

.macro dcp_uint2double_m rzl,rzh,rx
	WXUC	\rx,\rx
	ADD0
	SUB1
	NRDD
	RDDS	\rzl,\rzh
.endm

.macro dcp_double2int_m rz,rxl,rxh
	WXDC	\rxl,\rxh
	ADD0
	ADD1
	NTDC
	RDIC	\rz
.endm

.macro dcp_double2uint_m rz,rxl,rxh
	WXDC	\rxl,\rxh
	ADD0
	ADD1
	NTDC
	RDUC	\rz
.endm

/****************************************************************************
 * Helpers
 ****************************************************************************/

/* No interrupt between the first and the last coprocessor instruction */

#ifdef RP23XX_DCP_NOMASK  /* dcptest only: to show what the masking is for */
.macro dcp_enter r
.endm
.macro dcp_leave r
.endm
#else
.macro dcp_enter r
	mrs	\r, primask
	cpsid	i
.endm
.macro dcp_leave r
	msr	primask, \r
.endm
#endif

.macro dcp_func name
	.p2align 2
	.global	\name
	.type	\name, %function
	.thumb_func
\name:
.endm

.macro dcp_alias alias, name
	.global	\alias
	.thumb_set \alias, \name
.endm

/* The same in software (libgcc's routine, renamed: rp23xx_dcpsoft.sh;
 * from a module through the firmware's symbol table, so not with bl)
 */

.macro soft_tail name
#ifdef RP23XX_DCP_TEST
	ldr	ip, =soft_\name
#else
	ldr	ip, =__rp23xx_soft_\name
#endif
	bx	ip
	.ltorg
.endm

/* To \slow unless the exponent field of the double whose high word is \hi
 * is in [64, 0x7fe] (not zero, denormal, tiny, infinite or NaN)
 */

.macro chk_add_in hi, slow
	lsls	ip, \hi, #1
	add	ip, ip, #0x00200000		/* (exponent + 1) << 21 */
	cmp	ip, #0x08200000
	blo	\slow
.endm

/* To \slow if the exponent fields of the doubles whose high words are
 * \h1 and \h2 differ by 33: the coprocessor's addition and subtraction
 * round wrongly there (by one unit in the last place) when the larger
 * operand is just above a power of two and the result just below it,
 * e.g. 0x4320000000000002 - 0x411fffffffffffff; at no other difference
 * (dcptest probe)
 */

.macro chk_add_d33 h1, h2, slow
	lsrs	ip, \h1, #20
	sub	ip, ip, \h2, lsr #20		/* exponents' difference... */
	lsls	ip, ip, #21			/* ...modulo 2048 (no signs) */
	cmp	ip, #(33 << 21)
	beq	\slow
	cmn	ip, #(33 << 21)
	beq	\slow
.endm

/* To \slow if the result's exponent field is 0 or 0x7ff */

.macro chk_res hi, slow
	lsls	ip, \hi, #1
	add	ip, ip, #0x00200000
	cmp	ip, #0x00400000
	blo	\slow
.endm

/* To \slow if the double \lo, \hi is a denormal */

.macro chk_cmp_in lo, hi, slow
	lsls	ip, \hi, #1
	cmp	ip, #0x00200000
	bhs	77f				/* exponent not 0 */
	orrs	ip, ip, \lo
	bne	\slow				/* not zero */
77:
.endm

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* void rp23xx_dcp_enable(void): coprocessor 4 for this core (CPACR CP4
 * full access), the engaged flag cleared.  Called on each core at start.
 */

#ifndef RP23XX_DCP_TEST
dcp_func rp23xx_dcp_enable
	ldr	r0, =0xe000ed88			/* CPACR */
	ldr	r1, [r0]
	orr	r1, r1, #(3 << 8)		/* CP4: privileged and user */
	str	r1, [r0]
	dsb
	isb
	RCMP	r0				/* clears the engaged flag */
	bx	lr
	.ltorg
#endif

/* Arithmetic ***************************************************************/

/* The coprocessor flushes denormal operands and results to zero, gives
 * a NaN of its own (0xfff8...) for an invalid operation and keeps NaN
 * payloads its own way: those cases go to libgcc's routines (SOFT(),
 * renamed copies), so that every result is libgcc's bit for bit.
 *
 * Addition and subtraction: both operands' exponent fields in
 * [64, 0x7fe] (then a result cannot be denormal: two such numbers differ
 * by 2^-1011 at least, or not at all), else software.
 */

dcp_func AE(dadd)
	chk_add_in r1, 9f
	chk_add_in r3, 9f
	chk_add_d33 r1, r3, 9f
	dcp_enter r12
	dcp_dadd_m r0,r1,r0,r1,r2,r3
	dcp_leave r12
	bx	lr
9:	soft_tail dadd

dcp_func AE(dsub)
	chk_add_in r1, 9f
	chk_add_in r3, 9f
	chk_add_d33 r1, r3, 9f
	dcp_enter r12
	dcp_dsub_m r0,r1,r0,r1,r2,r3
	dcp_leave r12
	bx	lr
9:	soft_tail dsub

dcp_func AE(drsub)
	chk_add_in r1, 9f
	chk_add_in r3, 9f
	chk_add_d33 r1, r3, 9f
	dcp_enter r12
	dcp_dsub_m r0,r1,r2,r3,r0,r1
	dcp_leave r12
	bx	lr
9:	soft_tail drsub

/* Multiplication and division: whatever the operands, a result that is
 * zero, infinite or NaN (which is what a denormal operand, an underflow,
 * an invalid operation or a NaN gives on the coprocessor) is done again
 * in software, from the operands kept on the stack.
 */

dcp_func AE(dmul)
	push	{r0-r5, lr}
	dcp_enter r5
	dcp_dmul_m r0,r1,r0,r1,r2,r3,r0,r1,r2,r3,r4,r12,r14
	dcp_leave r5
	chk_res	r1, 9f
	add	sp, #16
	pop	{r4, r5, pc}
9:	pop	{r0-r5, lr}
	soft_tail dmul

dcp_func AE(ddiv)
	push	{r0-r4, lr}
	dcp_enter r4
	dcp_ddiv_m r0,r1,r0,r1,r2,r3,r0,r1,r2,r3,r12
	dcp_leave r4
	chk_res	r1, 9f
	add	sp, #16
	pop	{r4, pc}
9:	pop	{r0-r4, lr}
	soft_tail ddiv

/* double rp23xx_dcp_sqrt_core(double) in r0:r1 (base PCS): correctly
 * rounded for positive normal numbers
 */

dcp_func RP(sqrt_core)
	push	{r4, lr}
	dcp_enter r4
	dcp_dsqrt_m r0,r1,r0,r1,r0,r1,r2,r3,r12
	dcp_leave r4
	pop	{r4, pc}

/* Conversions **************************************************************/

/* int32_t, uint32_t -> double: always exact */

dcp_func AE(i2d)
.Li2d:
	dcp_enter r12
	dcp_int2double_m r0,r1,r0
	dcp_leave r12
	bx	lr

dcp_func AE(ui2d)
.Lui2d:
	dcp_enter r12
	dcp_uint2double_m r0,r1,r0
	dcp_leave r12
	bx	lr

/* int64 -> double: (double)hi * 2^32 + (double)(uint32_t)lo, both terms
 * exact (normal, far above the denormals), so the one rounding is the
 * addition's.
 */

dcp_func AE(l2d)
	cmp	r1, r0, asr #31
	beq	.Li2d				/* fits in an int32_t */
	cmp	r1, #0
	beq	.Lui2d				/* fits in a uint32_t */
	dcp_enter r12
	WXIC	r1,r1
	ADD0
	SUB1
	NRDD
	RDDS	r2,r3
	b	1f

dcp_func AE(ul2d)
	cmp	r1, #0
	beq	.Lui2d
	dcp_enter r12
	WXUC	r1,r1
	ADD0
	SUB1
	NRDD
	RDDS	r2,r3
1:	add	r3, r3, #(32 << 20)		/* hi * 2^32: hi != 0 */
	WXUC	r0,r0
	ADD0
	SUB1
	NRDD
	RDDS	r0,r1
	dcp_dadd_m r0,r1,r0,r1,r2,r3
	dcp_leave r12
	bx	lr

/* float -> double: libgcc's is as fast (dcptest bench) */

dcp_func AE(f2d)
	soft_tail f2d

/* double -> float: infinities and NaNs in, zeros and denormals out, in
 * software
 */

dcp_func AE(d2f)
	lsls	r12, r1, #1
	cmn	r12, #0x00200000
	bcs	9f				/* exponent 0x7ff */
	dcp_enter r12
	dcp_double2float_m r2,r0,r1
	dcp_leave r12
	tst	r2, #0x7f800000
	beq	9f				/* zero or denormal */
	mov	r0, r2
	bx	lr
9:	soft_tail d2f

/* double -> int32_t, towards zero: NaNs (and infinities) in software
 * (libgcc gives 0 for a NaN, the coprocessor saturates)
 */

dcp_func AE(d2iz)
	lsls	r12, r1, #1
	cmn	r12, #0x00200000
	bcs	9f
	dcp_enter r12
	dcp_double2int_m r0,r0,r1
	dcp_leave r12
	bx	lr
9:	soft_tail d2iz

/* double -> uint32_t: libgcc's is faster */

dcp_func AE(d2uiz)
	soft_tail d2uiz

/* Comparisons **************************************************************/

/* The status RCMP gives: Z equal, C greater or equal (as cmp x, y), V
 * unordered (with C clear).  The coprocessor takes a denormal for zero, so
 * a comparison with a denormal operand is done in software.
 */

dcp_func AE(dcmpun)
	dcp_enter r12
	dcp_dcmp_m r0,r0,r1,r2,r3
	dcp_leave r12
	ubfx	r0, r0, #28, #1			/* V */
	bx	lr

dcp_func AE(dcmpeq)
	chk_cmp_in r0, r1, 9f
	chk_cmp_in r2, r3, 9f
	dcp_enter r12
	dcp_dcmp_m r0,r0,r1,r2,r3
	dcp_leave r12
	ubfx	r0, r0, #30, #1			/* Z */
	bx	lr
9:	soft_tail dcmpeq

dcp_func AE(dcmplt)
	chk_cmp_in r0, r1, 9f
	chk_cmp_in r2, r3, 9f
	dcp_enter r12
	dcp_dcmp_m apsr_nzcv,r2,r3,r0,r1	/* y > x */
	dcp_leave r12
	ite	hi
	movhi	r0, #1
	movls	r0, #0
	bx	lr
9:	soft_tail dcmplt

dcp_func AE(dcmple)
	chk_cmp_in r0, r1, 9f
	chk_cmp_in r2, r3, 9f
	dcp_enter r12
	dcp_dcmp_m apsr_nzcv,r2,r3,r0,r1	/* y >= x */
	dcp_leave r12
	ite	hs
	movhs	r0, #1
	movlo	r0, #0
	bx	lr
9:	soft_tail dcmple

dcp_func AE(dcmpge)
	chk_cmp_in r0, r1, 9f
	chk_cmp_in r2, r3, 9f
	dcp_enter r12
	dcp_dcmp_m apsr_nzcv,r0,r1,r2,r3
	dcp_leave r12
	ite	hs
	movhs	r0, #1
	movlo	r0, #0
	bx	lr
9:	soft_tail dcmpge

dcp_func AE(dcmpgt)
	chk_cmp_in r0, r1, 9f
	chk_cmp_in r2, r3, 9f
	dcp_enter r12
	dcp_dcmp_m apsr_nzcv,r0,r1,r2,r3
	dcp_leave r12
	ite	hi
	movhi	r0, #1
	movls	r0, #0
	bx	lr
9:	soft_tail dcmpgt

/* The flag-setting comparisons keep r0-r3 (only ip, lr and the flags may
 * change: libgcc's cdrcmple does not, but these do).  Unordered: C set, Z
 * clear.
 */

dcp_func AE(cdrcmple)
	chk_cmp_in r0, r1, 8f
	chk_cmp_in r2, r3, 8f
	dcp_enter r12
	dcp_dcmp_m apsr_nzcv,r2,r3,r0,r1
	dcp_leave r12
	bvs	2f
	bx	lr
8:	push	{r0-r3, lr}			/* software, operands swapped */
	mov	ip, r0
	mov	r0, r2
	mov	r2, ip
	mov	ip, r1
	mov	r1, r3
	mov	r3, ip
	b	7f

dcp_func AE(cdcmple)
	chk_cmp_in r0, r1, 9f
	chk_cmp_in r2, r3, 9f
	dcp_enter r12
	dcp_dcmp_m apsr_nzcv,r0,r1,r2,r3
	dcp_leave r12
	bvs	2f
	bx	lr
2:	movs	r12, #3
	lsrs	r12, #1				/* C set, Z clear */
	bx	lr
9:	push	{r0-r3, lr}			/* software */
7:	ldr	ip, =SOFT(cdcmple)
	blx	ip
	pop	{r0-r3}				/* (the flags stay) */
	pop	{pc}
	.ltorg

dcp_alias AE(cdcmpeq), AE(cdcmple)

/* libgcc's -1/0/1 comparisons (gcc calls the AEABI ones): libgcc's own,
 * as fast
 */

dcp_func GN(eqdf2)
	soft_tail gnu_eqdf2
dcp_func GN(nedf2)
	soft_tail gnu_nedf2
dcp_func GN(cmpdf2)
	soft_tail gnu_cmpdf2
dcp_func GN(ltdf2)
	soft_tail gnu_ltdf2
dcp_func GN(ledf2)
	soft_tail gnu_ledf2
dcp_func GN(gedf2)
	soft_tail gnu_gedf2
dcp_func GN(gtdf2)
	soft_tail gnu_gtdf2

/* GNU names of the helpers above (libgcc defines both) */

dcp_alias GN(adddf3), AE(dadd)
dcp_alias GN(subdf3), AE(dsub)
dcp_alias GN(muldf3), AE(dmul)
dcp_alias GN(divdf3), AE(ddiv)
dcp_alias GN(floatsidf), AE(i2d)
dcp_alias GN(floatunsidf), AE(ui2d)
dcp_alias GN(floatdidf), AE(l2d)
dcp_alias GN(floatundidf), AE(ul2d)
dcp_alias GN(extendsfdf2), AE(f2d)
dcp_alias GN(truncdfsf2), AE(d2f)
dcp_alias GN(fixdfsi), AE(d2iz)
dcp_alias GN(fixunsdfsi), AE(d2uiz)
dcp_alias GN(unorddf2), AE(dcmpun)

/* double __ieee754_sqrt(double): newlib's core of sqrt() (and of acos,
 * asin, acosh, asinh, hypot, pow), so all of them get the coprocessor.
 * Positive normal numbers on the coprocessor; the rest as newlib's fdlibm
 * e_sqrt.c has it: NaNs and infinities x * x + x, zeros themselves,
 * negative numbers (x - x) / (x - x).  A positive denormal is scaled into
 * the normal range by an even power of two, which commutes with the
 * square root and its rounding.
 */

dcp_func LM(__ieee754_sqrt)
#ifdef __ARM_PCS_VFP
	vmov	r0, r1, d0
#endif
	lsrs	r2, r1, #20			/* sign and exponent */
	subs	r2, #1
	movw	r3, #0x7fe
	cmp	r2, r3
	bhs	4f				/* not a positive normal number */
	push	{r4, lr}
	dcp_enter r4
	dcp_dsqrt_m r0,r1,r0,r1,r0,r1,r2,r3,r12
	dcp_leave r4
#ifdef __ARM_PCS_VFP
	vmov	d0, r0, r1
#endif
	pop	{r4, pc}

4:	push	{r4, r5, r6, lr}
	mov	r4, r0
	mov	r5, r1
	ubfx	r2, r1, #20, #11		/* the exponent */
	movw	r3, #0x7ff
	cmp	r2, r3
	beq	5f				/* NaN or infinity */
	orrs	r3, r0, r1, lsl #1
	beq	8f				/* +-0 */
	cmp	r1, #0
	blt	6f				/* negative */

	/* A positive denormal: sqrt(x * 2^108) * 2^-54 */

	movs	r2, #0
	movs	r3, #0
	movt	r3, #0x46b0			/* 2^108 */
	bl	AE(dmul)
	bl	RP(sqrt_core)
	movs	r2, #0
	movs	r3, #0
	movt	r3, #0x3c90			/* 2^-54 */
	bl	AE(dmul)
	b	7f

5:	mov	r2, r0				/* x * x + x */
	mov	r3, r1
	bl	AE(dmul)
	mov	r2, r4
	mov	r3, r5
	bl	AE(dadd)
	b	7f

6:	mov	r2, r0				/* (x - x) / (x - x) */
	mov	r3, r1
	bl	AE(dsub)
	mov	r2, r0
	mov	r3, r1
	bl	AE(ddiv)
	b	7f

8:	mov	r0, r4				/* x */
	mov	r1, r5
7:
#ifdef __ARM_PCS_VFP
	vmov	d0, r0, r1
#endif
	pop	{r4, r5, r6, pc}

	.end
