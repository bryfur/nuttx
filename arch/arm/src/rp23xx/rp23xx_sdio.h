/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_sdio.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_SDIO_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_SDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/sdio.h>

#ifdef CONFIG_RP23XX_SDIO

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rp23xx_sdio_initialize
 *
 * Description:
 *   Take the PIO block, its DMA channels and the slot's six pins
 *   (CONFIG_RP23XX_SDIO_*) and return the SD host, its clock running at
 *   the identification rate.  For mmcsd_slotinitialize().
 *
 ****************************************************************************/

struct sdio_dev_s *rp23xx_sdio_initialize(int slotno);

/****************************************************************************
 * Name: rp23xx_sdio_probe
 *
 * Description:
 *   Whether a card in the slot answers in SD mode (CMD0, then CMD8 or
 *   CMD55 at the identification rate).  A card that a program put into SPI
 *   mode stays there until it loses power, and answers nothing here: the
 *   board can then take the slot back for SPI (rp23xx_sdio_release()).
 *
 * Returned Value:
 *   OK if a card answered, -ENODEV if none did.
 *
 ****************************************************************************/

int rp23xx_sdio_probe(struct sdio_dev_s *dev);

/****************************************************************************
 * Name: rp23xx_sdio_release
 *
 * Description:
 *   Stop the host: its state machines halted, the pins back to plain
 *   inputs with their pull-ups, the clock low.  Only before
 *   mmcsd_slotinitialize() has been given the host.
 *
 ****************************************************************************/

void rp23xx_sdio_release(struct sdio_dev_s *dev);

/****************************************************************************
 * Name: rp23xx_sdio_readonly
 *
 * Description:
 *   Refuse writes (CMD24, CMD25 fail with -EROFS without reaching the
 *   card) while the file system above still mounts read-write: for trying
 *   the host on a card with data on it.
 *
 ****************************************************************************/

void rp23xx_sdio_readonly(struct sdio_dev_s *dev, bool readonly);

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic when a card is inserted or removed.
 *   The slot has no card-detect switch: the board says a card is present
 *   once rp23xx_sdio_probe() found one.
 *
 ****************************************************************************/

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot);

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report a write-protected card.
 *
 ****************************************************************************/

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_RP23XX_SDIO */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_SDIO_H */
