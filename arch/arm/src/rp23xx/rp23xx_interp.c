/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_interp.c
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

/* Who holds each core's SIO interpolators (arch/chip/interp.h).  Their
 * state is not saved across context switches: a thread pinned to one core
 * claims that core's pair, and no other thread may use it meanwhile, so
 * whatever runs on the core between the holder's turns leaves it as it
 * was.  That costs nothing per switch (saving and restoring both
 * interpolators, 14 registers each way, would be paid on every switch of
 * every thread, for the few that use them).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <sched.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <arch/chip/interp.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SMP
#  define NCPUS CONFIG_SMP_NCPUS
#else
#  define NCPUS 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pid_t g_interp_owner[NCPUS];       /* 0: free */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void interp_reset(struct rp23xx_interp_s *ip)
{
  ip->ctrl[0] = 0;
  ip->ctrl[1] = 0;
  ip->accum[0] = 0;
  ip->accum[1] = 0;
  ip->base[0] = 0;
  ip->base[1] = 0;
  ip->base[2] = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rp23xx_interp_claim(void)
{
  irqstate_t flags = enter_critical_section();
  FAR struct tcb_s *tcb = this_task();
  int cpu = this_cpu();
  pid_t owner = g_interp_owner[cpu];
  int ret = cpu;

#ifdef CONFIG_SMP
  /* Pinned to this core alone, or the scheduler could move it */

  if (tcb->affinity != (cpu_set_t)(1 << cpu))
    {
      ret = -EPERM;
    }
  else
#endif
  if (owner != 0 && owner != tcb->pid && nxsched_get_tcb(owner) != NULL)
    {
      ret = -EBUSY;
    }
  else
    {
      g_interp_owner[cpu] = tcb->pid;
      interp_reset(RP23XX_INTERP0);
      interp_reset(RP23XX_INTERP1);
    }

  leave_critical_section(flags);
  return ret;
}

void rp23xx_interp_release(void)
{
  irqstate_t flags = enter_critical_section();
  int cpu = this_cpu();

  if (g_interp_owner[cpu] == this_task()->pid)
    {
      g_interp_owner[cpu] = 0;
    }

  leave_critical_section(flags);
}

int rp23xx_interp_held(void)
{
  irqstate_t flags = enter_critical_section();
  int held = g_interp_owner[this_cpu()] == this_task()->pid;

  leave_critical_section(flags);
  return held;
}
