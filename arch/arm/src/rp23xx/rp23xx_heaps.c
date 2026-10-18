/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_heaps.c
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

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mm.h>

#include "arm_internal.h"
#include <arch/chip/psram.h>

#include "rp23xx_psram.h"
#include "hardware/rp23xx_xip.h"
#include "hardware/rp23xx_memorymap.h"

#if defined(CONFIG_RP23XX_PSRAM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The PSRAM base address is fixed (RP23XX_PSRAM_BASE); the size is
 * whatever rp23xx_psramconfig() detected at boot (0 if no PSRAM is fitted).
 * psramconfig() runs from rp23xx_boardinitialize(), before any of the heap
 * hooks below, so the detected size is always available here.
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A board's linker script may put static data at the start of the PSRAM
 * (sections that end at _epsram_bss): the heap is the rest of it.
 */

extern uint8_t _epsram_bss[] weak_data;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The part of the PSRAM the heap may have: after any static data */

static inline void *psram_heap(size_t *size)
{
  uintptr_t start = RP23XX_PSRAM_BASE;
  uintptr_t end   = RP23XX_PSRAM_BASE + rp23xx_psram_size();

  if ((uintptr_t)_epsram_bss > start && (uintptr_t)_epsram_bss < end)
    {
      start = ((uintptr_t)_epsram_bss + 15) & ~(uintptr_t)15;
    }

  *size = end > start ? end - start : 0;
  return (void *)start;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#if defined(CONFIG_RP23XX_PSRAM_HEAP_SEPARATE)
static struct mm_heap_s *g_psramheap;
#endif

#if defined(CONFIG_RP23XX_PSRAM_HEAP_SINGLE)

#if defined(CONFIG_MM_KERNEL_HEAP)
#error cannot use CONFIG_MM_KERNEL_HEAP with single heap
#endif

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
  size_t psram_size;
  void *psram_start = psram_heap(&psram_size);

  /* Add the detected PSRAM region to the main heap */

  if (psram_size > 0)
    {
      kumm_addregion(psram_start, psram_size);
    }
}
#endif

#elif defined (CONFIG_RP23XX_PSRAM_HEAP_USER)

#if !defined(CONFIG_MM_KERNEL_HEAP)
#error MM_KERNEL_HEAP is required for separate kernel heap
#endif

/* Use the internal SRAM as the kernel heap */

void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  *heap_start = (void *)g_idle_topstack;

#ifdef CONFIG_ARCH_PGPOOL_PBASE
  *heap_size  = CONFIG_ARCH_PGPOOL_PBASE - g_idle_topstack;
#else
  *heap_size  = CONFIG_RAM_END - g_idle_topstack;
#endif
}

/* Use the external PSRAM as the default user heap */

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  *heap_start = psram_heap(heap_size);
}

#if CONFIG_MM_REGIONS > 1
/* A board without flash runs from a second PSRAM on CS0, which its loader
 * made writable (docs/psram-boot.md in the fpc tree): what the image leaves
 * of it is a second region of the user heap, up to 4 KB short of its end
 * (the loader's record of the image is in its last 16 bytes: a warm boot
 * uses the image again).  Its size: the loader's mailbox says (psramboot.h:
 * "PBOT", CS0's KB at +32), else that of the same part on CS1.
 */

extern uint8_t __flash_binary_end[];

#define PSRAMBOOT_MBOX     0x20080000
#define PSRAMBOOT_MAGIC    0x544f4250
#define CS0_KEEP           4096

void arm_addregion(void)
{
  uintptr_t start;
  uintptr_t end;
  size_t size = rp23xx_psram_size();

  if ((getreg32(RP23XX_XIP_CTRL_BASE) & RP23XX_XIP_CTRL_WRITABLE_M0) == 0)
    {
      return;
    }

  if (getreg32(PSRAMBOOT_MBOX) == PSRAMBOOT_MAGIC &&
      getreg32(PSRAMBOOT_MBOX + 32) != 0 &&
      getreg32(PSRAMBOOT_MBOX + 32) <= RP23XX_PSRAM_WINDOW / 1024)
    {
      size = getreg32(PSRAMBOOT_MBOX + 32) * 1024;
    }

  start = ((uintptr_t)__flash_binary_end + 4095) & ~(uintptr_t)4095;
  end   = RP23XX_FLASH_BASE + size - CS0_KEEP;
  if (size > CS0_KEEP && start < end)
    {
      umm_addregion((void *)start, end - start);
    }
}
#endif

#elif defined (CONFIG_RP23XX_PSRAM_HEAP_SEPARATE)

#if !defined(CONFIG_ARCH_HAVE_EXTRA_HEAPS)
#error ARCH_HAVE_EXTRA_HEAPS is required for multiple heaps
#endif

void up_extraheaps_init(void)
{
  size_t psram_size;
  void *psram_start = psram_heap(&psram_size);

  if (psram_size > 0)
    {
      g_psramheap = mm_initialize("psram", psram_start, psram_size);
    }
}

/* The PSRAM heap, for large buffers that want it (NULL: no PSRAM) */

struct mm_heap_s *rp23xx_psram_heap(void)
{
  return g_psramheap;
}
#endif

#endif
