/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_crypto.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership.  The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
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

#ifdef CONFIG_CRYPTO_CRYPTODEV_HARDWARE

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/queue.h>

#include <crypto/cryptodev.h>
#include <crypto/xform.h>
#include <nuttx/crypto/crypto.h>
#include <nuttx/kmalloc.h>

#include "rp23xx_sha256.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SHA-256 on the accelerator (rp23xx_sha256.c), which also serves any
 * number of sessions at once (the ones it cannot take are computed in
 * software)
 */

struct rp23xx_crypto_data
{
  int alg;
  union
  {
    struct
    {
      FAR struct rp23xx_sha256_s *ictx;
      FAR const struct auth_hash *axf;
    } auth;
  } u;

#define hw_ictx u.auth.ictx
#define hw_axf  u.auth.axf

  SLIST_ENTRY(rp23xx_crypto_data) next;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sha256_init(FAR void *ctx);
static int sha256_update(FAR void *ctx, FAR const uint8_t *in, size_t len);
static void sha256_final(FAR uint8_t *out, FAR void *ctx);
static int rp23xx_freesession(uint64_t tid);

/****************************************************************************
 * Private Data
 ****************************************************************************/

SLIST_HEAD(rp23xx_crypto_list, rp23xx_crypto_data);
static FAR struct rp23xx_crypto_list *g_rp23xx_sessions;
static uint32_t g_rp23xx_sesnum;

static const struct auth_hash g_auth_hash_sha256_rp23xx =
{
  CRYPTO_SHA2_256, "SHA256",
  0, 32, 12, sizeof(struct rp23xx_sha256_s),
  0,
  sha256_init, NULL, NULL,
  sha256_update,
  sha256_final
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sha256_init(FAR void *ctx)
{
  rp23xx_sha256_init(ctx);
}

static int sha256_update(FAR void *ctx, FAR const uint8_t *in, size_t len)
{
  return rp23xx_sha256_update(ctx, in, len);
}

static void sha256_final(FAR uint8_t *out, FAR void *ctx)
{
  uint8_t digest[32];

  rp23xx_sha256_final(ctx, digest);
  if (out != NULL)
    {
      memcpy(out, digest, sizeof(digest));
    }

  /* The session may hash another message */

  rp23xx_sha256_init(ctx);
}

static int rp23xx_hash(FAR struct cryptop *crp,
                       FAR struct cryptodesc *crd,
                       FAR struct rp23xx_crypto_data *data,
                       caddr_t buf)
{
  FAR const struct auth_hash *axf = data->hw_axf;

  if (data->hw_ictx == NULL)
    {
      return -EINVAL;
    }

  if (crd->crd_flags & CRD_F_UPDATE)
    {
      return axf->update(data->hw_ictx,
                         (FAR uint8_t *)buf + crd->crd_skip,
                         crd->crd_len);
    }

  axf->final((FAR uint8_t *)crp->crp_mac, data->hw_ictx);
  return OK;
}

static int rp23xx_newsession(uint32_t *sid, FAR struct cryptoini *cri)
{
  FAR struct rp23xx_crypto_list *session;
  FAR struct rp23xx_crypto_data *data;
  FAR const struct auth_hash *axf;
  int i;

  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < g_rp23xx_sesnum; i++)
    {
      if (SLIST_EMPTY(&g_rp23xx_sessions[i]))
        {
          break;
        }
    }

  if (i >= g_rp23xx_sesnum)
    {
      FAR struct rp23xx_crypto_list *new_sessions;
      uint32_t new_count =
        g_rp23xx_sessions == NULL ? 1 : g_rp23xx_sesnum * 2;

      new_sessions = kmm_calloc(new_count, sizeof(*new_sessions));
      if (new_sessions == NULL)
        {
          return -ENOBUFS;
        }

      if (g_rp23xx_sessions != NULL)
        {
          memcpy(new_sessions, g_rp23xx_sessions,
                 g_rp23xx_sesnum * sizeof(*new_sessions));
          kmm_free(g_rp23xx_sessions);
        }

      g_rp23xx_sessions = new_sessions;
      g_rp23xx_sesnum = new_count;
    }

  session = &g_rp23xx_sessions[i];
  *sid = i;

  while (cri != NULL)
    {
      data = kmm_zalloc(sizeof(*data));
      if (data == NULL)
        {
          rp23xx_freesession(i);
          return -ENOBUFS;
        }

      switch (cri->cri_alg)
        {
          case CRYPTO_SHA2_256:
            axf = &g_auth_hash_sha256_rp23xx;
            data->hw_ictx = kmm_zalloc(axf->ctxsize);
            if (data->hw_ictx == NULL)
              {
                kmm_free(data);
                rp23xx_freesession(i);
                return -ENOBUFS;
              }

            axf->init(data->hw_ictx);
            data->hw_axf = axf;
            data->alg = cri->cri_alg;
            SLIST_INSERT_HEAD(session, data, next);
            break;

          default:
            kmm_free(data);
            rp23xx_freesession(i);
            return -EINVAL;
        }

      cri = cri->cri_next;
    }

  return OK;
}

static int rp23xx_freesession(uint64_t tid)
{
  uint32_t sid = tid & 0xffffffff;
  FAR struct rp23xx_crypto_data *data;
  FAR struct rp23xx_crypto_data *next;

  if (sid >= g_rp23xx_sesnum)
    {
      return -EINVAL;
    }

  data = SLIST_FIRST(&g_rp23xx_sessions[sid]);
  while (data != NULL)
    {
      next = SLIST_NEXT(data, next);

      if (data->hw_ictx != NULL)
        {
          rp23xx_sha256_abort(data->hw_ictx);
          kmm_free(data->hw_ictx);
        }

      kmm_free(data);
      data = next;
    }

  SLIST_INIT(&g_rp23xx_sessions[sid]);
  return OK;
}

static int rp23xx_process(FAR struct cryptop *crp)
{
  uint32_t lid;
  FAR struct cryptodesc *crd;
  FAR struct rp23xx_crypto_list *session;
  FAR struct rp23xx_crypto_data *data;

  lid = crp->crp_sid & 0xffffffff;
  if (lid >= g_rp23xx_sesnum)
    {
      return -EINVAL;
    }

  session = &g_rp23xx_sessions[lid];

  for (crd = crp->crp_desc; crd != NULL; crd = crd->crd_next)
    {
      SLIST_FOREACH(data, session, next)
        {
          if (data->alg == crd->crd_alg)
            {
              break;
            }
        }

      if (data == NULL)
        {
          crp->crp_etype = EINVAL;
          return -EINVAL;
        }

      switch (data->alg)
        {
          case CRYPTO_SHA2_256:
            crp->crp_etype = rp23xx_hash(crp, crd, data, crp->crp_buf);
            if (crp->crp_etype != 0)
              {
                return crp->crp_etype;
              }
            break;

          default:
            return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void hwcr_init(void)
{
  int hwcr_id;
  int algs[CRYPTO_ALGORITHM_MAX + 1];

  hwcr_id = crypto_get_driverid(0);
  DEBUGASSERT(hwcr_id >= 0);

  memset(algs, 0, sizeof(algs));
  algs[CRYPTO_SHA2_256] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_register(hwcr_id, algs, rp23xx_newsession,
                  rp23xx_freesession, rp23xx_process);
}

#endif /* CONFIG_CRYPTO_CRYPTODEV_HARDWARE */
