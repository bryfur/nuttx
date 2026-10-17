/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_atomic.c
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

/* The out-of-line half of <arch/rp23xx/atomic.h>: the operations on words
 * outside the SRAM (the PSRAM), under SIO hardware spinlock 31, and the
 * <nuttx/atomic.h> functions themselves for callers that cannot inline
 * them (xcc's programs).  All in the SRAM.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/atomic.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/rp23xx_sio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Spinlock 31: not mirrored by any SIO register write (RP2350-E2) */

#define RP23XX_ATOMIC_SPINLOCK  RP23XX_SIO_SPINLOCK(31)

/* In the SRAM (the board copies .time_critical there at boot) */

#define ATOMIC_FAST  locate_code(".time_critical.atomic")

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function irqstate_t atomic_lock(void)
{
  irqstate_t flags = up_irq_save();

  while (getreg32(RP23XX_ATOMIC_SPINLOCK) == 0)
    {
    }

  UP_DMB();
  return flags;
}

static inline_function void atomic_unlock(irqstate_t flags)
{
  UP_DMB();
  putreg32(0, RP23XX_ATOMIC_SPINLOCK);
  up_irq_restore(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

ATOMIC_FAST int32_t rp23xx_atomic_locked_4(FAR volatile void *ptr,
                                           int32_t value, int op)
{
  FAR volatile int32_t *p = ptr;
  irqstate_t flags = atomic_lock();
  int32_t old = *p;

  switch (op)
    {
      case RP23XX_ATOMIC_STORE:
      case RP23XX_ATOMIC_XCHG:
        *p = value;
        break;

      case RP23XX_ATOMIC_ADD:
        *p = old + value;
        break;

      case RP23XX_ATOMIC_SUB:
        *p = old - value;
        break;

      case RP23XX_ATOMIC_AND:
        *p = old & value;
        break;

      case RP23XX_ATOMIC_OR:
        *p = old | value;
        break;

      default:
        *p = old ^ value;
        break;
    }

  atomic_unlock(flags);
  return old;
}

ATOMIC_FAST bool rp23xx_atomic_cmpxchg_locked_4(FAR volatile void *ptr,
                                                FAR volatile void *expect,
                                                int32_t desired)
{
  FAR volatile int32_t *p = ptr;
  FAR volatile int32_t *e = expect;
  irqstate_t flags = atomic_lock();
  int32_t old = *p;
  bool ok = old == *e;

  if (ok)
    {
      *p = desired;
    }

  atomic_unlock(flags);

  if (!ok)
    {
      *e = old;
    }

  return ok;
}

/* <nuttx/atomic.h>'s functions, for what does not inline them */

#undef atomic_store_4
#undef atomic_load_4
#undef atomic_exchange_4
#undef atomic_compare_exchange_4
#undef atomic_fetch_add_4
#undef atomic_fetch_sub_4
#undef atomic_fetch_and_4
#undef atomic_fetch_or_4
#undef atomic_fetch_xor_4

ATOMIC_FAST void atomic_store_4(FAR volatile void *ptr, int32_t value,
                                int memorder)
{
  rp23xx_atomic_store_4(ptr, value, memorder);
}

ATOMIC_FAST int32_t atomic_load_4(FAR const volatile void *ptr,
                                  int memorder)
{
  return rp23xx_atomic_load_4(ptr, memorder);
}

ATOMIC_FAST int32_t atomic_exchange_4(FAR volatile void *ptr, int32_t value,
                                      int memorder)
{
  return rp23xx_atomic_exchange_4(ptr, value, memorder);
}

ATOMIC_FAST bool atomic_compare_exchange_4(FAR volatile void *ptr,
                                           FAR volatile void *expect,
                                           int32_t desired, bool weak,
                                           int success, int failure)
{
  return rp23xx_atomic_compare_exchange_4(ptr, expect, desired, weak,
                                          success, failure);
}

ATOMIC_FAST int32_t atomic_fetch_add_4(FAR volatile void *ptr,
                                       int32_t value, int memorder)
{
  return rp23xx_atomic_fetch_add_4(ptr, value, memorder);
}

ATOMIC_FAST int32_t atomic_fetch_sub_4(FAR volatile void *ptr,
                                       int32_t value, int memorder)
{
  return rp23xx_atomic_fetch_sub_4(ptr, value, memorder);
}

ATOMIC_FAST int32_t atomic_fetch_and_4(FAR volatile void *ptr,
                                       int32_t value, int memorder)
{
  return rp23xx_atomic_fetch_and_4(ptr, value, memorder);
}

ATOMIC_FAST int32_t atomic_fetch_or_4(FAR volatile void *ptr,
                                      int32_t value, int memorder)
{
  return rp23xx_atomic_fetch_or_4(ptr, value, memorder);
}

ATOMIC_FAST int32_t atomic_fetch_xor_4(FAR volatile void *ptr,
                                       int32_t value, int memorder)
{
  return rp23xx_atomic_fetch_xor_4(ptr, value, memorder);
}
