/****************************************************************************
 * arch/arm/include/rp23xx/psram.h
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

/* What the PSRAM driver (arch/arm/src/rp23xx/rp23xx_psram.c) found at
 * boot, for the board's boot messages and for tests (apps/testing/mm/
 * psramtest).
 */

#ifndef __ARCH_ARM_INCLUDE_RP23XX_PSRAM_H
#define __ARCH_ARM_INCLUDE_RP23XX_PSRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The PSRAM through the XIP cache, and the same without it (no allocation
 * in the cache, RP2350 datasheet 4.4.1)
 */

#define RP23XX_PSRAM_CACHED      0x11000000
#define RP23XX_PSRAM_UNCACHED    0x15000000
#define RP23XX_PSRAM_WINDOW      (16 * 1024 * 1024)

/* The parts recognised (rp23xx_psram_info_s.chip) */

#define RP23XX_PSRAM_NONE        0   /* Nothing answered (KGD not 5Dh) */
#define RP23XX_PSRAM_APMEMORY    1   /* APS6404L / ESP-PSRAM64H, MF ID 0Dh */
#define RP23XX_PSRAM_ISSI        2   /* IS66WVS16M8, MF ID 9Dh */
#define RP23XX_PSRAM_UNKNOWN     3   /* KGD 5Dh, another maker: APS timing */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rp23xx_psram_info_s
{
  uint8_t     id[8];       /* Read ID (9Fh) after the address: MF ID, KGD,
                            * EID (density in the top bits of id[2]) */
  uint8_t     chip;        /* RP23XX_PSRAM_* */
  const char *name;        /* "APS6404", "IS66WVS16M8", ... */
  uint32_t    id_size;     /* Bytes, from the ID (0: not known) */
  uint32_t    probe_size;  /* Bytes, from the aliasing probe at boot */
  uint32_t    size;        /* Bytes in use: the smaller of the two */
  uint32_t    timing;      /* QMI M1_TIMING in use */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RP23XX_PSRAM

/* What rp23xx_psramconfig() found (all zero before it ran) */

void rp23xx_psram_info(struct rp23xx_psram_info_s *info);

/* Change QMI M1_TIMING (tests: other dividers, sample delays).  It stays
 * in use, also after flash programming (rp23xx_psram_restore()).
 */

void rp23xx_psram_set_timing(uint32_t timing);

#endif

#endif /* __ARCH_ARM_INCLUDE_RP23XX_PSRAM_H */
