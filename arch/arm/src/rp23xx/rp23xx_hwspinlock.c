/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_hwspinlock.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/hwspinlock/hwspinlock.h>

#include "arm_internal.h"
#include "hardware/rp23xx_sio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SIO spinlock that makes atomic operations atomic between the cores
 * (CONFIG_LIBC_ATOMIC_HWSPINLOCK).  Erratum RP2350-E2: writes to some SIO
 * registers also write spinlocks 0-4, 8, 9 and 12-17 (the doorbells alias
 * 0-3); 31 is one of those that do not alias (see rp23xx_start.c).
 */

#define RP23XX_ATOMIC_SPINLOCK  31

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool rp23xx_hwspinlock_trylock(struct hwspinlock_dev_s *dev);
static void rp23xx_hwspinlock_unlock(struct hwspinlock_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct hwspinlock_ops_s g_rp23xx_hwspinlock_ops =
{
  .trylock = rp23xx_hwspinlock_trylock,
  .unlock  = rp23xx_hwspinlock_unlock
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Atomic operations take this lock (libs/libc/machine/arch_atomic.c): with
 * it they are plain loads and stores, atomic between the cores on any
 * memory, the PSRAM included, where exclusive accesses do not work.
 */

struct hwspinlock_dev_s g_atomic_hwspinlock =
{
  .id       = RP23XX_ATOMIC_SPINLOCK,
  .priority = 0,
  .ops      = &g_rp23xx_hwspinlock_ops
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Reading a spinlock claims it (non-zero) if it was free; the barriers
 * order the protected accesses after the claim and before the release
 */

static bool rp23xx_hwspinlock_trylock(struct hwspinlock_dev_s *dev)
{
  if (getreg32(RP23XX_SIO_SPINLOCK(dev->id)) != 0)
    {
      __asm__ __volatile__ ("dmb" : : : "memory");
      return true;
    }

  return false;
}

static void rp23xx_hwspinlock_unlock(struct hwspinlock_dev_s *dev)
{
  __asm__ __volatile__ ("dmb" : : : "memory");
  putreg32(0, RP23XX_SIO_SPINLOCK(dev->id));
}
