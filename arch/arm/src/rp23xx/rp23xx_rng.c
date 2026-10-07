/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_rng.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/drivers/drivers.h>

#ifdef CONFIG_CRYPTO
#  include <crypto/sha2.h>
#endif

#include "arm_internal.h"
#include "hardware/rp23xx_trng.h"

#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The TRNG delivers entropy 192 bits (six 32-bit EHR words) at a time. */

#define RP23XX_TRNG_EHR_WORDS  6
#define RP23XX_TRNG_EHR_BYTES  (RP23XX_TRNG_EHR_WORDS * sizeof(uint32_t))

/* RNG_ISR health-test error bits: a sample that failed the on-chip
 * von-Neumann, CRNGT or autocorrelation test must be discarded.
 */

#define RP23XX_TRNG_ISR_ERRORS (RP23XX_TRNG_RNG_ISR_VN_ERR |     \
                                RP23XX_TRNG_RNG_ISR_CRNGT_ERR |  \
                                RP23XX_TRNG_RNG_ISR_AUTOCORR_ERR)

/* Startup self-test: draw this many blocks and reject an obviously broken
 * source (stuck or all-constant) before the RNG is trusted.
 */

#define RP23XX_TRNG_SELFTEST_BLOCKS  4

/* A block that fails the health tests this many times in a row, or that
 * takes longer than this, is an error (-EIO), not an endless wait.  After
 * an autocorrelation failure the source stops until it is reset.
 */

#define RP23XX_TRNG_MAXFAILS    32
#define RP23XX_TRNG_TIMEOUT     MSEC2TICK(2000)

/* /dev/urandom: ChaCha20 keyed from the TRNG, a new key every
 * RP23XX_URANDOM_REKEY bytes (and the key it used erased: what was read
 * before cannot be worked out from the state after).
 */

#define RP23XX_URANDOM_REKEY    (64 * 1024)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t rp23xx_rng_read(struct file *filep, char *buffer,
                               size_t buflen);
#ifdef CONFIG_DEV_URANDOM_ARCH
static ssize_t rp23xx_urandom_read(struct file *filep, char *buffer,
                                   size_t buflen);
static ssize_t rp23xx_urandom_write(struct file *filep, const char *buffer,
                                    size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_rng_lock = NXMUTEX_INITIALIZER;
static bool    g_rng_healthy;   /* startup self-test passed */
static bool    g_rng_inited;

static const struct file_operations g_rngops =
{
  NULL,             /* open */
  NULL,             /* close */
  rp23xx_rng_read,  /* read */
  NULL,             /* write */
  NULL,             /* seek */
  NULL,             /* ioctl */
};

#ifdef CONFIG_DEV_URANDOM_ARCH
static const struct file_operations g_urandomops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  rp23xx_urandom_read,  /* read */
  rp23xx_urandom_write, /* write */
  NULL,                 /* seek */
  NULL,                 /* ioctl */
};

static struct
{
  mutex_t  lock;
  uint32_t state[16];           /* ChaCha20: constants, key, counter, nonce */
  uint32_t left;                /* Bytes until the next key */
  bool     keyed;
} g_urandom =
{
  NXMUTEX_INITIALIZER
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The source on, configured (again, after a reset) */

static void rp23xx_rng_start(void)
{
#if CONFIG_RP23XX_RNG_SAMPLE_CNT > 0
  putreg32(CONFIG_RP23XX_RNG_SAMPLE_CNT, RP23XX_TRNG_SAMPLE_CNT1);
#endif
  putreg32(0xffffffff, RP23XX_TRNG_RNG_ICR);
  putreg32(1, RP23XX_TRNG_RND_SOURCE_ENABLE);
}

/* After an autocorrelation failure: the TRNG reset, its configuration and
 * the source set up again
 */

static void rp23xx_rng_reset(void)
{
  int i;

  putreg32(0, RP23XX_TRNG_RND_SOURCE_ENABLE);
  putreg32(1, RP23XX_TRNG_TRNG_SW_RESET);
  for (i = 0; i < 1000 && (getreg32(RP23XX_TRNG_TRNG_BUSY) & 1) != 0; i++)
    {
    }

  rp23xx_rng_start();
}

/****************************************************************************
 * Name: rp23xx_rng_collect
 *
 * Description:
 *   Wait for one fresh 192-bit block into 'ehr', discarding any block the
 *   on-chip health tests rejected (re-arming the source each time).  The
 *   entropy source must already be enabled.  -EIO if the source keeps
 *   failing them, or delivers nothing.
 *
 ****************************************************************************/

static int rp23xx_rng_collect(uint32_t ehr[RP23XX_TRNG_EHR_WORDS])
{
  clock_t start = clock_systime_ticks();
  int fails = 0;
  int i;

  for (; ; )
    {
      uint32_t isr = getreg32(RP23XX_TRNG_RNG_ISR);

      if ((isr & RP23XX_TRNG_ISR_ERRORS) != 0)
        {
          /* Health test failed: drop the block and restart the source
           * (an autocorrelation failure stops it until a reset)
           */

          if (++fails > RP23XX_TRNG_MAXFAILS)
            {
              putreg32(0, RP23XX_TRNG_RND_SOURCE_ENABLE);
              return -EIO;
            }

          if ((isr & RP23XX_TRNG_RNG_ISR_AUTOCORR_ERR) != 0)
            {
              rp23xx_rng_reset();
            }
          else
            {
              putreg32(0xffffffff, RP23XX_TRNG_RNG_ICR);
              putreg32(0, RP23XX_TRNG_RND_SOURCE_ENABLE);
              putreg32(1, RP23XX_TRNG_RND_SOURCE_ENABLE);
            }

          start = clock_systime_ticks();
          continue;
        }

      if ((isr & RP23XX_TRNG_RNG_ISR_EHR_VALID) != 0)
        {
          break;
        }

      if (clock_systime_ticks() - start > RP23XX_TRNG_TIMEOUT)
        {
          putreg32(0, RP23XX_TRNG_RND_SOURCE_ENABLE);
          return -EIO;
        }
    }

  for (i = 0; i < RP23XX_TRNG_EHR_WORDS; i++)
    {
      ehr[i] = getreg32(RP23XX_TRNG_EHR_DATA(i));
    }

  putreg32(0xffffffff, RP23XX_TRNG_RNG_ICR);
  return OK;
}

/****************************************************************************
 * Name: rp23xx_rng_fill
 *
 * Description:
 *   buflen bytes of hardware entropy, with g_rng_lock held.  With
 *   CONFIG_CRYPTO, conditioned: each 32 bytes out are the SHA-256 of 64
 *   bytes from the TRNG (whose raw output is slightly biased: a chi-square
 *   test of its bytes fails), as the pico-sdk does with its TRNG samples.
 *
 ****************************************************************************/

static int rp23xx_rng_fill(FAR void *buffer, size_t buflen)
{
  FAR uint8_t *p = buffer;
  int ret = OK;

  rp23xx_rng_start();

  while (buflen > 0)
    {
#ifdef CONFIG_CRYPTO
      uint32_t ehr[RP23XX_TRNG_EHR_WORDS * 3];  /* 72 bytes: 64 used */
      uint8_t digest[SHA256_DIGEST_LENGTH];
      SHA2_CTX ctx;
      int i;
#else
      uint32_t ehr[RP23XX_TRNG_EHR_WORDS];
#endif
      size_t chunk;

#ifdef CONFIG_CRYPTO
      for (i = 0; i < 3 && ret == OK; i++)
        {
          ret = rp23xx_rng_collect(&ehr[i * RP23XX_TRNG_EHR_WORDS]);
        }

      if (ret < 0)
        {
          break;
        }

      sha256init(&ctx);
      sha256update(&ctx, ehr, 64);
      sha256final(digest, &ctx);
      chunk = buflen < sizeof(digest) ? buflen : sizeof(digest);
      memcpy(p, digest, chunk);
      explicit_bzero(digest, sizeof(digest));
      explicit_bzero(&ctx, sizeof(ctx));
#else
      ret = rp23xx_rng_collect(ehr);
      if (ret < 0)
        {
          break;
        }

      chunk = buflen < RP23XX_TRNG_EHR_BYTES ? buflen :
              RP23XX_TRNG_EHR_BYTES;
      memcpy(p, ehr, chunk);
#endif
      explicit_bzero(ehr, sizeof(ehr));
      p      += chunk;
      buflen -= chunk;
    }

  /* Leave the source disabled to save power. */

  putreg32(0, RP23XX_TRNG_RND_SOURCE_ENABLE);
  return ret;
}

/****************************************************************************
 * Name: rp23xx_rng_selftest
 *
 * Description:
 *   Draw a few conditioned blocks at startup and reject an obviously broken
 *   source (a stuck source emitting identical blocks).  The on-chip tests
 *   cover statistical quality; this only catches a dead source before any
 *   key is derived from it.  Returns true if the source looks healthy.
 *
 ****************************************************************************/

static bool rp23xx_rng_selftest(void)
{
  uint32_t block[RP23XX_TRNG_SELFTEST_BLOCKS][RP23XX_TRNG_EHR_WORDS];
  bool ok = true;
  int i;

  rp23xx_rng_start();

  for (i = 0; i < RP23XX_TRNG_SELFTEST_BLOCKS; i++)
    {
      if (rp23xx_rng_collect(block[i]) < 0 ||
          (i > 0 && memcmp(block[i], block[i - 1],
                           RP23XX_TRNG_EHR_BYTES) == 0))
        {
          ok = false;
          break;
        }
    }

  putreg32(0, RP23XX_TRNG_RND_SOURCE_ENABLE);
  return ok;
}

/****************************************************************************
 * Name: rp23xx_rng_read
 *
 * Description:
 *   Fill 'buffer' with 'buflen' bytes of hardware entropy, discarding any
 *   block the on-chip health tests reject.  Fails with -EIO if the startup
 *   self-test found the source broken, or it fails now.
 *
 ****************************************************************************/

static ssize_t rp23xx_rng_read(struct file *filep, char *buffer,
                               size_t buflen)
{
  int ret;

  ret = nxmutex_lock(&g_rng_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = g_rng_healthy ? rp23xx_rng_fill(buffer, buflen) : -EIO;
  nxmutex_unlock(&g_rng_lock);
  return ret < 0 ? ret : (ssize_t)buflen;
}

#ifdef CONFIG_DEV_URANDOM_ARCH
/****************************************************************************
 * Name: chacha20_block
 *
 * Description:
 *   One 64-byte ChaCha20 block of the state (RFC 8439), the 32-bit block
 *   counter (word 12) advanced.
 *
 ****************************************************************************/

#define ROTL(v, n) (((v) << (n)) | ((v) >> (32 - (n))))
#define QR(a, b, c, d) \
  do \
    { \
      a += b; d ^= a; d = ROTL(d, 16); \
      c += d; b ^= c; b = ROTL(b, 12); \
      a += b; d ^= a; d = ROTL(d, 8); \
      c += d; b ^= c; b = ROTL(b, 7); \
    } \
  while (0)

static void chacha20_block(uint32_t state[16], uint32_t out[16])
{
  uint32_t x[16];
  int i;

  memcpy(x, state, sizeof(x));
  for (i = 0; i < 10; i++)
    {
      QR(x[0], x[4], x[8],  x[12]);
      QR(x[1], x[5], x[9],  x[13]);
      QR(x[2], x[6], x[10], x[14]);
      QR(x[3], x[7], x[11], x[15]);
      QR(x[0], x[5], x[10], x[15]);
      QR(x[1], x[6], x[11], x[12]);
      QR(x[2], x[7], x[8],  x[13]);
      QR(x[3], x[4], x[9],  x[14]);
    }

  for (i = 0; i < 16; i++)
    {
      out[i] = x[i] + state[i];
    }

  state[12]++;
}

/* A new key and nonce from the TRNG (with g_urandom.lock held) */

static int rp23xx_urandom_rekey(void)
{
  uint32_t seed[12];
  int ret;

  ret = nxmutex_lock(&g_rng_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = g_rng_healthy ? rp23xx_rng_fill(seed, sizeof(seed)) : -EIO;
  nxmutex_unlock(&g_rng_lock);
  if (ret < 0)
    {
      return ret;
    }

  g_urandom.state[0] = 0x61707865;      /* "expand 32-byte k" */
  g_urandom.state[1] = 0x3320646e;
  g_urandom.state[2] = 0x79622d32;
  g_urandom.state[3] = 0x6b206574;
  memcpy(&g_urandom.state[4], seed, sizeof(seed));  /* key, counter, nonce */
  explicit_bzero(seed, sizeof(seed));
  g_urandom.left  = RP23XX_URANDOM_REKEY;
  g_urandom.keyed = true;
  return OK;
}

/****************************************************************************
 * Name: rp23xx_urandom_read
 *
 * Description:
 *   ChaCha20's key stream, keyed from the TRNG.  After each read the key is
 *   replaced by the next block's first 32 bytes, which are never handed out
 *   (fast key erasure): the state after a read says nothing of the bytes it
 *   gave.
 *
 ****************************************************************************/

static ssize_t rp23xx_urandom_read(struct file *filep, char *buffer,
                                   size_t buflen)
{
  uint32_t block[16];
  size_t done = 0;
  int ret;

  ret = nxmutex_lock(&g_urandom.lock);
  if (ret < 0)
    {
      return ret;
    }

  while (done < buflen)
    {
      size_t chunk;

      if (!g_urandom.keyed || g_urandom.left == 0)
        {
          ret = rp23xx_urandom_rekey();
          if (ret < 0)
            {
              break;
            }
        }

      chacha20_block(g_urandom.state, block);
      chunk = buflen - done < sizeof(block) ? buflen - done : sizeof(block);
      memcpy(buffer + done, block, chunk);
      done += chunk;
      g_urandom.left = g_urandom.left > sizeof(block) ?
                       g_urandom.left - sizeof(block) : 0;
    }

  if (g_urandom.keyed)
    {
      chacha20_block(g_urandom.state, block);
      memcpy(&g_urandom.state[4], block, 32);
    }

  explicit_bzero(block, sizeof(block));
  nxmutex_unlock(&g_urandom.lock);
  return done > 0 ? (ssize_t)done : ret;
}

/* Writing to /dev/urandom mixes the bytes into the key (as Linux does);
 * the next read uses a fresh key anyway once this one has served
 * RP23XX_URANDOM_REKEY bytes
 */

static ssize_t rp23xx_urandom_write(struct file *filep, const char *buffer,
                                    size_t buflen)
{
  size_t i;
  int ret;

  ret = nxmutex_lock(&g_urandom.lock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_urandom.keyed)
    {
      for (i = 0; i < buflen; i++)
        {
          ((FAR uint8_t *)&g_urandom.state[4])[i % 32] ^= buffer[i];
        }
    }

  nxmutex_unlock(&g_urandom.lock);
  return buflen;
}
#endif

/****************************************************************************
 * Name: rp23xx_rng_init
 *
 * Description:
 *   Run the startup self-test once and latch the result.  Idempotent.
 *
 ****************************************************************************/

static void rp23xx_rng_init(void)
{
  if (!g_rng_inited)
    {
      g_rng_healthy = rp23xx_rng_selftest();
      g_rng_inited  = true;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Register /dev/random, backed by the RP2350 hardware TRNG.  Called
 *   automatically from drivers_initialize() when CONFIG_DEV_RANDOM is set.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void)
{
  rp23xx_rng_init();
  register_driver("/dev/random", &g_rngops, 0444, NULL);
}
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom: ChaCha20 keyed from the hardware TRNG (the TRNG
 *   itself gives some tens of KB a second).  Called from
 *   drivers_initialize() when CONFIG_DEV_URANDOM selects the arch source.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
  rp23xx_rng_init();
  register_driver("/dev/urandom", &g_urandomops, 0666, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
