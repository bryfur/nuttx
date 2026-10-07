/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_sha256.c
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

/* The RP2350's SHA-256 accelerator (datasheet 12.13), shared by any number
 * of computations (rp23xx_sha256.h).
 *
 * The accelerator digests whole 64-byte blocks written to WDATA and can
 * neither be given a state to continue from nor serve two messages at
 * once; its state after each block can be read (SUM0-7).  So each
 * computation keeps its unfinished block and its length itself and only
 * whole blocks go to the accelerator, which belongs to one computation at
 * a time (g_owner).  The others go to crypto/sha2.c; an owner idle for
 * RP23XX_SHA256_IDLE_MS when another wants the accelerator continues in
 * software from the state read out of it.  A mutex covers each call.
 *
 * Throughput: the core takes 57 cycles per block after the 16 word writes
 * (4 cycles each over APB), about 121 cycles per block: the processor's
 * writes reach that, so DMA would not be faster (it would only leave the
 * processor free, for about 0.4 us per block).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/clock.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/rp23xx_resets.h"
#include "hardware/rp23xx_sha256.h"
#include "rp23xx_sha256.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLOCK         64

#define MODE_FRESH    0   /* Nothing digested yet */
#define MODE_HW       1   /* The accelerator holds the state */
#define MODE_SW       2   /* ctx->sw holds the state */

/* An owner not seen for this long gives the accelerator up to another */

#define IDLE_TICKS    MSEC2TICK(50)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_lock = NXMUTEX_INITIALIZER;
static FAR struct rp23xx_sha256_s *g_owner;
static bool g_ready;
static uint64_t g_hwbytes;
static uint64_t g_swbytes;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void put_be32(FAR uint8_t *p, uint32_t v)
{
  p[0] = v >> 24;
  p[1] = v >> 16;
  p[2] = v >> 8;
  p[3] = v;
}

/* Whole blocks to the accelerator */

static void hw_blocks(FAR const uint8_t *p, size_t n)
{
  while (n-- > 0)
    {
      while ((getreg32(RP23XX_SHA256_CSR) &
              RP23XX_SHA256_CSR_WDATA_RDY) == 0)
        {
        }

      if (((uintptr_t)p & 3) == 0)
        {
          FAR const uint32_t *w = (FAR const uint32_t *)p;
          int i;

          for (i = 0; i < BLOCK / 4; i += 4)
            {
              uint32_t a = w[i];
              uint32_t b = w[i + 1];
              uint32_t c = w[i + 2];
              uint32_t d = w[i + 3];

              putreg32(a, RP23XX_SHA256_WDATA);
              putreg32(b, RP23XX_SHA256_WDATA);
              putreg32(c, RP23XX_SHA256_WDATA);
              putreg32(d, RP23XX_SHA256_WDATA);
            }
        }
      else
        {
          int i;

          for (i = 0; i < BLOCK; i += 4)
            {
              uint32_t v;

              memcpy(&v, p + i, 4);
              putreg32(v, RP23XX_SHA256_WDATA);
            }
        }

      p += BLOCK;
    }
}

/* The state after the blocks written so far */

static void hw_state(FAR uint32_t *st)
{
  int i;

  while ((getreg32(RP23XX_SHA256_CSR) & RP23XX_SHA256_CSR_SUM_VLD) == 0)
    {
    }

  for (i = 0; i < 8; i++)
    {
      st[i] = getreg32(RP23XX_SHA256_SUM(i));
    }
}

/* The accelerator for ctx (FRESH), unless another is using it */

static bool hw_take(FAR struct rp23xx_sha256_s *ctx)
{
  uint32_t csr;

  if (g_owner != NULL)
    {
      if (clock_systime_ticks() - g_owner->last < IDLE_TICKS)
        {
          return false;
        }

      /* The owner continues in software from where the accelerator is */

      hw_state(g_owner->sw.state.st32);
      g_owner->mode = MODE_SW;
      g_owner = NULL;
    }

  if (!g_ready)
    {
      modifyreg32(RP23XX_RESETS_RESET, RP23XX_RESETS_RESET_SHA256, 0);
      while ((getreg32(RP23XX_RESETS_RESET_DONE) &
              RP23XX_RESETS_RESET_DONE_SHA256) == 0)
        {
        }

      g_ready = true;
    }

  /* START, the bytes of each word in message order (BSWAP), a stale
   * error cleared; not through the atomic aliases (write-one-to-clear and
   * self-clearing bits)
   */

  csr = getreg32(RP23XX_SHA256_CSR) & RP23XX_SHA256_CSR_DMA_SIZE_MASK;
  putreg32(csr | RP23XX_SHA256_CSR_BSWAP |
           RP23XX_SHA256_CSR_ERR_WDATA_NOT_RDY |
           RP23XX_SHA256_CSR_START, RP23XX_SHA256_CSR);

  g_owner = ctx;
  ctx->mode = MODE_HW;
  ctx->last = clock_systime_ticks();
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rp23xx_sha256_init(FAR struct rp23xx_sha256_s *ctx)
{
  sha256init(&ctx->sw);
  ctx->mode = MODE_FRESH;
  ctx->last = 0;
}

int rp23xx_sha256_update(FAR struct rp23xx_sha256_s *ctx,
                         FAR const void *data, size_t len)
{
  FAR const uint8_t *p = data;
  size_t used;
  size_t n;
  int ret;

  if (len == 0)
    {
      return OK;
    }

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  used = (ctx->sw.bitcount[0] >> 3) % BLOCK;
  if (ctx->mode == MODE_FRESH && used + len >= BLOCK && !hw_take(ctx))
    {
      ctx->mode = MODE_SW;
    }

  if (ctx->mode != MODE_HW)
    {
      /* In software, or less than a block so far: no other computation
       * touches this one now, so not under the lock
       */

      if (ctx->mode == MODE_SW)
        {
          g_swbytes += len;
        }

      nxmutex_unlock(&g_lock);
      sha256update(&ctx->sw, p, len);
      return OK;
    }

  g_hwbytes += len;
  ctx->sw.bitcount[0] += (uint64_t)len << 3;

  /* The unfinished block first */

  if (used > 0)
    {
      n = BLOCK - used < len ? BLOCK - used : len;
      memcpy(ctx->sw.buffer + used, p, n);
      p += n;
      len -= n;
      used += n;
      if (used < BLOCK)
        {
          goto out;
        }

      hw_blocks(ctx->sw.buffer, 1);
    }

  n = len / BLOCK;
  hw_blocks(p, n);
  p += n * BLOCK;
  len -= n * BLOCK;
  memcpy(ctx->sw.buffer, p, len);

out:
  ctx->last = clock_systime_ticks();
  nxmutex_unlock(&g_lock);
  return OK;
}

int rp23xx_sha256_final(FAR struct rp23xx_sha256_s *ctx,
                        FAR uint8_t digest[32])
{
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* A short message: the accelerator if it is free */

  if (ctx->mode == MODE_FRESH && !hw_take(ctx))
    {
      ctx->mode = MODE_SW;
    }

  if (ctx->mode == MODE_SW)
    {
      nxmutex_unlock(&g_lock);
      sha256final(digest, &ctx->sw);
      explicit_bzero(ctx, sizeof(*ctx));
      return OK;
    }
  else
    {
      uint32_t blk[2 * BLOCK / 4];
      FAR uint8_t *b = (FAR uint8_t *)blk;
      uint64_t bits = ctx->sw.bitcount[0];
      size_t used = (bits >> 3) % BLOCK;
      size_t n = used < BLOCK - 8 ? BLOCK : 2 * BLOCK;
      uint32_t st[8];
      int i;

      /* The padding: 1, zeros, the length in bits (big-endian) */

      memcpy(b, ctx->sw.buffer, used);
      b[used] = 0x80;
      memset(b + used + 1, 0, n - used - 1 - 8);
      put_be32(b + n - 8, bits >> 32);
      put_be32(b + n - 4, bits);
      hw_blocks(b, n / BLOCK);
      hw_state(st);
      for (i = 0; i < 8; i++)
        {
          put_be32(digest + 4 * i, st[i]);
        }

      g_owner = NULL;
    }

  explicit_bzero(ctx, sizeof(*ctx));
  nxmutex_unlock(&g_lock);
  return OK;
}

void rp23xx_sha256_abort(FAR struct rp23xx_sha256_s *ctx)
{
  nxmutex_lock(&g_lock);
  if (g_owner == ctx)
    {
      g_owner = NULL;
    }

  explicit_bzero(ctx, sizeof(*ctx));
  nxmutex_unlock(&g_lock);
}

int rp23xx_sha256(FAR const void *data, size_t len, FAR uint8_t digest[32])
{
  struct rp23xx_sha256_s ctx;
  int ret;

  rp23xx_sha256_init(&ctx);
  ret = rp23xx_sha256_update(&ctx, data, len);
  if (ret < 0)
    {
      rp23xx_sha256_abort(&ctx);
      return ret;
    }

  return rp23xx_sha256_final(&ctx, digest);
}

void rp23xx_sha256_stats(FAR uint64_t *hw, FAR uint64_t *sw)
{
  nxmutex_lock(&g_lock);
  *hw = g_hwbytes;
  *sw = g_swbytes;
  nxmutex_unlock(&g_lock);
}
