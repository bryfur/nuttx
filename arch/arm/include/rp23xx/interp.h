/****************************************************************************
 * arch/arm/include/rp23xx/interp.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* The SIO interpolators (RP2350 datasheet 3.1.10), for software renderers
 * and other inner loops (#include <arch/chip/interp.h>).
 *
 * Each core has two, INTERP0 and INTERP1, in its own SIO: the same
 * addresses reach the core's own pair.  Each has two lanes; a lane takes
 * its accumulator (or the other's, CROSS_INPUT), rotates it right by SHIFT
 * (the RP2350 rotates, the RP2040 shifted: a rotate by 32 - n is a left
 * shift by n once the mask has dropped the bits that came round), keeps
 * bits MASK_LSB..MASK_MSB (sign-extended from MASK_MSB if SIGNED), and adds
 * it to its BASE (or, ADD_RAW, adds the accumulator as it is instead).
 * PEEK0/1 are the two lanes' results, PEEK2 is BASE2 plus both lanes'
 * shifted and masked values (never the raw ones); reading a POP alias
 * returns the same and writes the lane results back into the accumulators
 * (CROSS_RESULT: each into the other's).  With ADD_RAW and BASEn as a step,
 * a POP is "result, then accumulator += step" for both lanes at once, in
 * one single-cycle load: the affine texture coordinate idiom.
 *
 * The state is the core's, not a thread's, and nothing saves it when
 * threads switch.  The rule, enforced by rp23xx_interp_claim(): a thread
 * uses them only while it holds its core's pair, which it can only claim
 * pinned to that one core (sched_setaffinity(), pthread_attr_setaffinity_np())
 * and while no other living thread holds it; another thread running on
 * the core in between leaves them alone.  No interrupt handler or kernel
 * code uses them.  A thread that cannot claim them keeps to plain C.
 */

#ifndef __ARCH_ARM_INCLUDE_RP23XX_INTERP_H
#define __ARCH_ARM_INCLUDE_RP23XX_INTERP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CTRL_LANEn */

#define INTERP_SHIFT(n)       ((uint32_t)(n) & 31)          /* Rotate right by */
#define INTERP_MASK(lsb, msb) (((uint32_t)(lsb) & 31) << 5 | \
                               ((uint32_t)(msb) & 31) << 10)
#define INTERP_SIGNED         (1u << 15)
#define INTERP_CROSS_INPUT    (1u << 16)  /* The other lane's accumulator */
#define INTERP_CROSS_RESULT   (1u << 17)  /* POP: into the other accumulator */
#define INTERP_ADD_RAW        (1u << 18)  /* Lane result: BASE + accumulator */
#define INTERP_FORCE_MSB(n)   (((uint32_t)(n) & 3) << 19)
#define INTERP_BLEND          (1u << 21)  /* INTERP0 lane 0 only */
#define INTERP_CLAMP          (1u << 22)  /* INTERP1 lane 0 only */

/* A lane giving (acc >> frac) << sh in bits sh..sh+bits-1: acc rotated
 * right by frac - sh (a left rotate when sh > frac), the bits that came
 * round from the bottom dropped by the mask.  Exact for 0 <= acc <
 * 2^(frac+bits) and frac - sh + sh + bits <= 32.
 */

#define INTERP_FIELD(frac, sh, bits) \
  (INTERP_SHIFT((frac) - (sh)) | INTERP_MASK((sh), (sh) + (bits) - 1))

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rp23xx_interp_s
{
  volatile uint32_t accum[2];             /* 0x00 */
  volatile uint32_t base[3];              /* 0x08 */
  volatile uint32_t pop[3];               /* 0x14: lane 0, lane 1, full */
  volatile uint32_t peek[3];              /* 0x20 */
  volatile uint32_t ctrl[2];              /* 0x2c */
  volatile uint32_t add_raw[2];           /* 0x34: write adds to accum;
                                           * read: shifted and masked */
  volatile uint32_t base01;               /* 0x3c: write only, 16 + 16 */
};

#define RP23XX_INTERP0 ((struct rp23xx_interp_s *)0xd0000080)
#define RP23XX_INTERP1 ((struct rp23xx_interp_s *)0xd00000c0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rp23xx_interp_claim
 *
 * Description:
 *   The calling thread takes both interpolators of the core it runs on,
 *   until rp23xx_interp_release() (or its end).  They are reset: all
 *   registers zero.
 *
 * Returned Value:
 *   The core's index (0 or 1) on success; -EPERM if the thread is not
 *   pinned to that one core; -EBUSY if another living thread holds them.
 *   Claiming again what the thread holds succeeds (and resets them).
 *
 ****************************************************************************/

int rp23xx_interp_claim(void);

/****************************************************************************
 * Name: rp23xx_interp_release
 *
 * Description:
 *   The calling thread gives back the interpolators it holds (of the core
 *   it runs on, the one it claimed); nothing if it holds none.
 *
 ****************************************************************************/

void rp23xx_interp_release(void);

/****************************************************************************
 * Name: rp23xx_interp_held
 *
 * Description:
 *   True if the calling thread holds the interpolators of the core it is
 *   running on.
 *
 ****************************************************************************/

int rp23xx_interp_held(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_RP23XX_INTERP_H */
