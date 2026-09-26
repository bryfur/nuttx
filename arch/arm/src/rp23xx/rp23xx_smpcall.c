/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_smpcall.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <strings.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"
#include "arm_internal.h"
#include "hardware/rp23xx_sio.h"

#ifdef CONFIG_SMP

/* The cores signal each other with SIO doorbells: a bit set in the other
 * core's doorbell register raises its SIO_IRQ_BELL.  Unlike the inter-core
 * FIFO (which the boot ROM's core 1 launch uses), setting a bit never
 * waits, so two cores signalling each other with interrupts masked cannot
 * deadlock, and a request made twice before it is seen is seen once.
 *
 *   BELL_SCHED   process the tasks delivered to this CPU (up_send_smp_sched)
 *   BELL_CALL    run the SMP call queue (up_send_smp_call)
 *   BELL_IRQREQ  core 0: enable / disable the IRQs core 1 asked for
 *
 * External IRQs are all handled by core 0 (every driver's interrupt runs
 * there); up_enable_irq() and up_disable_irq() on core 1 leave the request
 * in g_irqreq and ring core 0.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BELL_SCHED    (1 << 0)
#define BELL_CALL     (1 << 1)
#define BELL_IRQREQ   (1 << 2)

#define SIO_BELL_OUT_SET (RP23XX_SIO_BASE + RP23XX_SIO_DOORBELL_OUT_SET_OFFSET)
#define SIO_BELL_IN_SET  (RP23XX_SIO_BASE + RP23XX_SIO_DOORBELL_IN_SET_OFFSET)
#define SIO_BELL_IN_CLR  (RP23XX_SIO_BASE + RP23XX_SIO_DOORBELL_IN_CLR_OFFSET)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* IRQ requests from core 1: for each external IRQ, whether a request is
 * pending and whether it is to enable (the last request wins)
 */

static struct
{
  spinlock_t lock;
  uint64_t   pending;
  uint64_t   enable;
} g_irqreq;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Ring the doorbell of a CPU: the other core's, or this one's own */

static void rp23xx_ring(int cpu, uint32_t bell)
{
  putreg32(bell, cpu == this_cpu() ? SIO_BELL_IN_SET : SIO_BELL_OUT_SET);
}

/* Core 0: apply what core 1 asked for */

static void rp23xx_handle_irqreqs(void)
{
  irqstate_t flags;
  uint64_t pending;
  uint64_t enable;
  int n;

  DEBUGASSERT(this_cpu() == 0);

  flags = spin_lock_irqsave(&g_irqreq.lock);
  pending = g_irqreq.pending;
  enable  = g_irqreq.enable;
  g_irqreq.pending = 0;
  spin_unlock_irqrestore(&g_irqreq.lock, flags);

  for (n = 0; pending != 0; n++, pending >>= 1, enable >>= 1)
    {
      if (pending & 1)
        {
          if (enable & 1)
            {
              up_enable_irq(RP23XX_IRQ_EXTINT + n);
            }
          else
            {
              up_disable_irq(RP23XX_IRQ_EXTINT + n);
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_smp_bell_handler
 *
 * Description:
 *   SIO_IRQ_BELL: what the other core (or this one) asked for.
 *
 ****************************************************************************/

int rp23xx_smp_bell_handler(int irq, void *c, void *arg)
{
  uint32_t bells = getreg32(SIO_BELL_IN_SET) & 0xff;

  /* Clear what is about to be handled: a bell rung from now on raises
   * the interrupt again
   */

  putreg32(bells, SIO_BELL_IN_CLR);

  if (bells & BELL_IRQREQ)
    {
      rp23xx_handle_irqreqs();
    }

  if (bells & BELL_CALL)
    {
      nxsched_smp_call_handler(irq, c, arg);
    }

  if (bells & BELL_SCHED)
    {
      nxsched_process_delivered(this_cpu());
    }

  return OK;
}

/****************************************************************************
 * Name: up_send_smp_sched
 *
 * Description:
 *   Have a CPU process the tasks delivered to it.
 *
 ****************************************************************************/

int up_send_smp_sched(int cpu)
{
  rp23xx_ring(cpu, BELL_SCHED);
  return OK;
}

/****************************************************************************
 * Name: up_send_smp_call
 *
 * Description:
 *   Have the CPUs in the set run their SMP call queues.
 *
 ****************************************************************************/

void up_send_smp_call(cpu_set_t cpuset)
{
  int cpu;

  for (; cpuset != 0; cpuset &= ~(1 << cpu))
    {
      cpu = ffs(cpuset) - 1;
      rp23xx_ring(cpu, BELL_CALL);
    }
}

/****************************************************************************
 * Name: rp23xx_send_irqreq
 *
 * Description:
 *   Core 1: ask core 0 to enable (irqreq > 0) or disable (irqreq < 0) an
 *   external IRQ.  Returns at once; core 0 does it when it takes the
 *   doorbell.
 *
 ****************************************************************************/

void rp23xx_send_irqreq(int irqreq)
{
  int n = (irqreq > 0 ? irqreq : -irqreq) - RP23XX_IRQ_EXTINT;
  irqstate_t flags;

  DEBUGASSERT(n >= 0 && n < 64);

  flags = spin_lock_irqsave(&g_irqreq.lock);
  g_irqreq.pending |= (uint64_t)1 << n;
  if (irqreq > 0)
    {
      g_irqreq.enable |= (uint64_t)1 << n;
    }
  else
    {
      g_irqreq.enable &= ~((uint64_t)1 << n);
    }

  spin_unlock_irqrestore(&g_irqreq.lock, flags);
  rp23xx_ring(0, BELL_IRQREQ);
}

#endif /* CONFIG_SMP */
