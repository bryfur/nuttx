/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_sha256.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_SHA256_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_SHA256_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

#include <crypto/sha2.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One SHA-256 computation.  Any number may be in progress at once, from
 * any task on either core: the one accelerator serves one of them at a
 * time (the first to need it, while it keeps using it), the others are
 * computed in software (crypto/sha2.c) with the same result.  An owner
 * left idle for a while is moved to software when another wants the
 * accelerator: its state so far is read out of the accelerator.
 */

struct rp23xx_sha256_s
{
  SHA2_CTX sw;          /* Software state; the bytes of an unfinished
                         * block and the length, whoever computes */
  uint32_t last;        /* Tick of the last use of the accelerator */
  uint8_t mode;         /* RP23XX_SHA256_FRESH, _HW, _SW */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Kernel use only (a mutex is taken): not from interrupt handlers */

void rp23xx_sha256_init(FAR struct rp23xx_sha256_s *ctx);
int rp23xx_sha256_update(FAR struct rp23xx_sha256_s *ctx,
                         FAR const void *data, size_t len);
int rp23xx_sha256_final(FAR struct rp23xx_sha256_s *ctx,
                        FAR uint8_t digest[32]);

/* Gives up a computation without a result (frees the accelerator) */

void rp23xx_sha256_abort(FAR struct rp23xx_sha256_s *ctx);

/* The SHA-256 of a buffer */

int rp23xx_sha256(FAR const void *data, size_t len, FAR uint8_t digest[32]);

/* Statistics: bytes hashed by the accelerator and in software */

void rp23xx_sha256_stats(FAR uint64_t *hw, FAR uint64_t *sw);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_SHA256_H */
