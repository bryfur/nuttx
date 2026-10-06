/****************************************************************************
 * graphics/nxbe/nxbe_swap.c
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

/* The swap (CONFIG_NX_COMPOSITOR_SWAP): memory for the cells of windows
 * that are not seen (nxbe_accel.c reads them back into it, and puts them
 * back when they are seen again).  A slot holds one cell as the hardware's
 * readbulk() leaves it: a word, the cell's pixels (rows packed), a word.
 * Slots come in chunks of NXSWAP_CHUNK from the heap, taken as needed
 * (while the heap keeps CONFIG_NX_COMPOSITOR_SWAPKEEP free in one block,
 * and up to CONFIG_NX_COMPOSITOR_SWAP in all) and given back as soon as
 * they are all free again.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <malloc.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nx/nxaccel.h>

#include "nxbe.h"

#ifdef CONFIG_NX_COMPOSITOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXSWAP_CHUNK     32                /* Slots a chunk */
#define NXSWAP_MAXCHUNKS 128               /* (16 bits of slot numbers) */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct
{
  size_t slotwords;                        /* A slot's size (words) */
  int maxchunks;                           /* The budget */
  int nchunks;                             /* Chunks taken */
  int nused;                               /* Slots in use */
  bool off;                                /* No slots (nxbe_swap_enable()) */
  FAR uint32_t *chunk[NXSWAP_MAXCHUNKS];
  uint32_t used[NXSWAP_MAXCHUNKS];         /* A bit a slot */
} g_swap;

struct nxbe_swapstats_s g_nxbe_swap;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* A chunk, out of the way of the large blocks a program (a game) may want
 * later: the heap gives the smallest free block that fits, from its start,
 * so the largest one (largest bytes, a node's size) is held meanwhile but
 * for exactly the chunk's node at its end; the chunk then goes into a
 * smaller free block if there is one, else at the largest one's end, and
 * nothing is left beside it for small allocations to pin.  Given back, it
 * joins the free block below it again.  (NXSWAP_NODE: the heap's node
 * header and alignment, mm_heap's with CONFIG_MM_BACKTRACE_SEQNO; were it
 * smaller, a few bytes more are the chunk's; larger, the chunk goes where
 * the heap puts it.)
 */

#define NXSWAP_NODE 16

static FAR uint32_t *swap_chunk(size_t size, size_t largest)
{
  FAR void *hold = NULL;
  FAR uint32_t *chunk;
  size_t node = (size + NXSWAP_NODE + 7) & ~(size_t)7;

  if (largest > 2 * node)
    {
      hold = kumm_malloc(largest - node - NXSWAP_NODE);
    }

  chunk = kumm_malloc(size);
  kumm_free(hold);
  return chunk != NULL ? chunk : kumm_malloc(size);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_swap_init
 *
 * Description:
 *   The cells are 1 << cellshift pixels square.
 *
 ****************************************************************************/

void nxbe_swap_init(int cellshift)
{
  size_t cell = (size_t)1 << cellshift;
  size_t slot = (cell * cell / 2 + 2) * sizeof(uint32_t);

  memset(&g_swap, 0, sizeof(g_swap));
  g_swap.slotwords = slot / sizeof(uint32_t);
  g_swap.maxchunks = (int)((size_t)CONFIG_NX_COMPOSITOR_SWAP * 1024 /
                           (slot * NXSWAP_CHUNK));
  if (g_swap.maxchunks > NXSWAP_MAXCHUNKS)
    {
      g_swap.maxchunks = NXSWAP_MAXCHUNKS;
    }
}

/****************************************************************************
 * Name: nxbe_swap_alloc, nxbe_swap_free, nxbe_swap_slot
 *
 * Description:
 *   A slot (its number), or -1 if there is no room within the budget;
 *   give one back; where a slot is.
 *
 ****************************************************************************/

int nxbe_swap_alloc(void)
{
  struct mallinfo mi;
  int c;
  int k;

  if (g_swap.off)
    {
      return -1;
    }

  for (c = 0; c < g_swap.nchunks; c++)
    {
      if (g_swap.chunk[c] != NULL && ~g_swap.used[c] != 0)
        {
          k = __builtin_ctz(~g_swap.used[c]);
          g_swap.used[c] |= 1u << k;
          g_swap.nused++;
          return c * NXSWAP_CHUNK + k;
        }
    }

  /* A chunk more, if the budget and the heap allow */

  for (c = 0; c < g_swap.maxchunks && g_swap.chunk[c] != NULL; c++)
    {
    }

  if (c >= g_swap.maxchunks)
    {
      return -1;
    }

  mi = kumm_mallinfo();
  if ((size_t)mi.mxordblk < (size_t)CONFIG_NX_COMPOSITOR_SWAPKEEP * 1024 +
      g_swap.slotwords * sizeof(uint32_t) * NXSWAP_CHUNK)
    {
      g_nxbe_swap.refused++;
      return -1;
    }

  g_swap.chunk[c] = swap_chunk(g_swap.slotwords * sizeof(uint32_t) *
                               NXSWAP_CHUNK, (size_t)mi.mxordblk);
  if (g_swap.chunk[c] == NULL)
    {
      g_nxbe_swap.refused++;
      return -1;
    }

  if (c >= g_swap.nchunks)
    {
      g_swap.nchunks = c + 1;
    }

  g_swap.used[c] = 1;
  g_swap.nused++;
  return c * NXSWAP_CHUNK;
}

void nxbe_swap_free(int slot)
{
  int c = slot / NXSWAP_CHUNK;
  uint32_t bit = 1u << (slot % NXSWAP_CHUNK);

  if (slot < 0 || c >= g_swap.nchunks || (g_swap.used[c] & bit) == 0)
    {
      return;
    }

  g_swap.used[c] &= ~bit;
  g_swap.nused--;
  if (g_swap.used[c] == 0)
    {
      kumm_free(g_swap.chunk[c]);
      g_swap.chunk[c] = NULL;
      while (g_swap.nchunks > 0 && g_swap.chunk[g_swap.nchunks - 1] == NULL)
        {
          g_swap.nchunks--;
        }
    }
}

FAR uint32_t *nxbe_swap_slot(int slot)
{
  return g_swap.chunk[slot / NXSWAP_CHUNK] +
         (size_t)(slot % NXSWAP_CHUNK) * g_swap.slotwords;
}

/****************************************************************************
 * Name: nxbe_swap_enable
 *
 * Description:
 *   Off: no slots are given (the CPU's memory is a game's: the compositor
 *   gives the slots it has back, nxbe_compose_pressure()); on again.
 *
 ****************************************************************************/

void nxbe_swap_enable(bool on)
{
  g_swap.off = !on;
}

/****************************************************************************
 * Name: nxbe_swap_usage
 *
 * Description:
 *   Slots in use, and the bytes the swap takes from the heap.
 *
 ****************************************************************************/

void nxbe_swap_usage(FAR uint32_t *slots, FAR uint32_t *bytes)
{
  int c;
  int n = 0;

  for (c = 0; c < g_swap.nchunks; c++)
    {
      n += g_swap.chunk[c] != NULL;
    }

  *slots = g_swap.nused;
  *bytes = (uint32_t)n * NXSWAP_CHUNK * g_swap.slotwords * sizeof(uint32_t);
}

#endif /* CONFIG_NX_COMPOSITOR */
