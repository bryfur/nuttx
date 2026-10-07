/****************************************************************************
 * arch/arm/include/rp23xx/atomic.h
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

/* The RP2350's atomic operations (CONFIG_LIBC_ATOMIC_CHIP, included by
 * <nuttx/atomic.h>).
 *
 * Exclusive loads and stores (LDREX/STREX) work on the SRAM only: its
 * global exclusive monitor sees both cores.  The XIP window (the flash and
 * the PSRAM, where the user heap and the modules live) has none: a STREX
 * there always fails, and a LDREX/STREX loop on a PSRAM word never ends.
 * So each operation looks at its address:
 *
 *   - in the SRAM (0x2xxxxxxx): the compiler's __atomic built-ins, inline
 *     (LDREX/STREX loops with barriers, as the pico-sdk's spin locks on the
 *     RP2350);
 *   - anywhere else: out of line, under SIO hardware spinlock 31 with
 *     interrupts masked (rp23xx_atomic.c, in the SRAM).
 *
 * A given word is always at the same address, so the two ways never mix on
 * one word.  A load is a plain aligned word load between barriers wherever
 * the word is (every writer of a PSRAM word writes it whole, under the
 * lock).  The memory order argument is ignored: every operation is
 * sequentially consistent, as the spinlock version was.  The 8-byte
 * operations stay in libs/libc/machine/arch_atomic64.c (a spinlock_t in the
 * SRAM).
 *
 * xcc (the board's C compiler: no inline assembly, no __atomic built-ins)
 * sees the out-of-line functions instead (nuttx/atomic.h).
 */

#ifndef __ARCH_ARM_INCLUDE_RP23XX_ATOMIC_H
#define __ARCH_ARM_INCLUDE_RP23XX_ATOMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* An address the SRAM's exclusive monitor covers */

#define RP23XX_ATOMIC_EXCLUSIVE(p)  (((uintptr_t)(p) >> 28) == 2)

/* rp23xx_atomic_locked_4()'s operations */

#define RP23XX_ATOMIC_STORE   0
#define RP23XX_ATOMIC_XCHG    1
#define RP23XX_ATOMIC_ADD     2
#define RP23XX_ATOMIC_SUB     3
#define RP23XX_ATOMIC_AND     4
#define RP23XX_ATOMIC_OR      5
#define RP23XX_ATOMIC_XOR     6

/* The names <nuttx/atomic.h> uses */

#define atomic_store_4             rp23xx_atomic_store_4
#define atomic_load_4              rp23xx_atomic_load_4
#define atomic_exchange_4          rp23xx_atomic_exchange_4
#define atomic_compare_exchange_4  rp23xx_atomic_compare_exchange_4
#define atomic_fetch_add_4         rp23xx_atomic_fetch_add_4
#define atomic_fetch_sub_4         rp23xx_atomic_fetch_sub_4
#define atomic_fetch_and_4         rp23xx_atomic_fetch_and_4
#define atomic_fetch_or_4          rp23xx_atomic_fetch_or_4
#define atomic_fetch_xor_4         rp23xx_atomic_fetch_xor_4

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Under the hardware spinlock (rp23xx_atomic.c): the previous value */

int32_t rp23xx_atomic_locked_4(FAR volatile void *ptr, int32_t value,
                               int op);
bool rp23xx_atomic_cmpxchg_locked_4(FAR volatile void *ptr,
                                    FAR volatile void *expect,
                                    int32_t desired);

/* The 8-byte ones (libs/libc/machine/arch_atomic64.c) */

void atomic_store_8(FAR volatile void *ptr, int64_t value, int memorder);
int64_t atomic_load_8(FAR const volatile void *ptr, int memorder);
int64_t atomic_exchange_8(FAR volatile void *ptr, int64_t value,
                          int memorder);
bool atomic_compare_exchange_8(FAR volatile void *ptr,
                               FAR volatile void *expect,
                               int64_t desired, bool weak,
                               int success, int failure);
int64_t atomic_fetch_add_8(FAR volatile void *ptr, int64_t value,
                           int memorder);
int64_t atomic_fetch_sub_8(FAR volatile void *ptr, int64_t value,
                           int memorder);
int64_t atomic_fetch_and_8(FAR volatile void *ptr, int64_t value,
                           int memorder);
int64_t atomic_fetch_or_8(FAR volatile void *ptr, int64_t value,
                          int memorder);
int64_t atomic_fetch_xor_8(FAR volatile void *ptr, int64_t value,
                           int memorder);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function void
rp23xx_atomic_store_4(FAR volatile void *ptr, int32_t value, int memorder)
{
  if (RP23XX_ATOMIC_EXCLUSIVE(ptr))
    {
      __atomic_store_n((FAR volatile int32_t *)ptr, value,
                       __ATOMIC_SEQ_CST);
    }
  else
    {
      rp23xx_atomic_locked_4(ptr, value, RP23XX_ATOMIC_STORE);
    }
}

static inline_function int32_t
rp23xx_atomic_load_4(FAR const volatile void *ptr, int memorder)
{
  int32_t value;

  __asm__ __volatile__ ("dmb" : : : "memory");
  value = *(FAR const volatile int32_t *)ptr;
  __asm__ __volatile__ ("dmb" : : : "memory");
  return value;
}

static inline_function int32_t
rp23xx_atomic_exchange_4(FAR volatile void *ptr, int32_t value,
                         int memorder)
{
  if (RP23XX_ATOMIC_EXCLUSIVE(ptr))
    {
      return __atomic_exchange_n((FAR volatile int32_t *)ptr, value,
                                 __ATOMIC_SEQ_CST);
    }

  return rp23xx_atomic_locked_4(ptr, value, RP23XX_ATOMIC_XCHG);
}

static inline_function bool
rp23xx_atomic_compare_exchange_4(FAR volatile void *ptr,
                                 FAR volatile void *expect,
                                 int32_t desired, bool weak,
                                 int success, int failure)
{
  if (RP23XX_ATOMIC_EXCLUSIVE(ptr))
    {
      return __atomic_compare_exchange_n((FAR volatile int32_t *)ptr,
                                         (FAR int32_t *)expect, desired,
                                         false, __ATOMIC_SEQ_CST,
                                         __ATOMIC_SEQ_CST);
    }

  return rp23xx_atomic_cmpxchg_locked_4(ptr, expect, desired);
}

#define RP23XX_ATOMIC_FETCH(name, builtin, op)                              \
  static inline_function int32_t                                            \
  rp23xx_atomic_fetch_##name##_4(FAR volatile void *ptr, int32_t value,     \
                                 int memorder)                              \
  {                                                                         \
    if (RP23XX_ATOMIC_EXCLUSIVE(ptr))                                       \
      {                                                                     \
        return builtin((FAR volatile int32_t *)ptr, value,                  \
                       __ATOMIC_SEQ_CST);                                   \
      }                                                                     \
                                                                            \
    return rp23xx_atomic_locked_4(ptr, value, op);                          \
  }

RP23XX_ATOMIC_FETCH(add, __atomic_fetch_add, RP23XX_ATOMIC_ADD)
RP23XX_ATOMIC_FETCH(sub, __atomic_fetch_sub, RP23XX_ATOMIC_SUB)
RP23XX_ATOMIC_FETCH(and, __atomic_fetch_and, RP23XX_ATOMIC_AND)
RP23XX_ATOMIC_FETCH(or,  __atomic_fetch_or,  RP23XX_ATOMIC_OR)
RP23XX_ATOMIC_FETCH(xor, __atomic_fetch_xor, RP23XX_ATOMIC_XOR)

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_INCLUDE_RP23XX_ATOMIC_H */
