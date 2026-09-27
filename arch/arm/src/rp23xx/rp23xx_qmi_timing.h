/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_qmi_timing.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_QMI_TIMING_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_QMI_TIMING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_qmi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QMI M0/M1_TIMING for a QSPI memory, from clk_sys and the memory's AC
 * limits (RP2350 datasheet 12.14.3).
 *
 * Reads: the QMI launches a falling edge of SCK, the memory puts the next
 * data out on it (tCO later), the data comes back through the pads and is
 * sampled at the next rising edge plus RXDELAY half cycles of clk_sys.
 * With RXDELAY = CLKDIV the sample lands one whole SCK period after the
 * falling edge, the moment the QMI launches the next one.  Then
 *
 *   setup: SCK period >= pad round trip + tCO (the maximum delays)
 *   hold:  the next data can only come back later than that sample (by
 *          the pads' minimum delays plus the memory's output hold): > 0
 *
 * whatever the clock, so the same setting is safe at the 12 MHz of early
 * boot and at full speed, and the fastest SCK is the lower of the memory's
 * own limit and 1 / (round trip + tCO).  The round trip (datasheet Table
 * 1289, worst case, VDDIO 3.3 V) is clk_sys to QSPI output 2.5 ns plus
 * QSPI input to clk_sys 1.5 ns.
 *
 *   f_sys  f_max   tCO    SCK max            CLKDIV  SCK      RXDELAY
 *   200    133    6.0    1/(4+6) = 100      2       100 MHz  2 (flash)
 *   200    133    5.5    1/9.5 = 105        2       100 MHz  2 (PSRAM)
 *   200     50    6.0    50                 4        50 MHz  4 (03h)
 *   150    133    6.0    100                2        75 MHz  2
 */

#define RP23XX_QMI_ROUNDTRIP_PS 4000

#define RP23XX_QMI_SCK_MAX(fmax, tco_ps) \
  ((1000000000000ull / (RP23XX_QMI_ROUNDTRIP_PS + (tco_ps))) < (fmax) ? \
   (1000000000000ull / (RP23XX_QMI_ROUNDTRIP_PS + (tco_ps))) : (fmax))

#define RP23XX_QMI_CLKDIV(fsys, fmax, tco_ps) \
  (((fsys) + RP23XX_QMI_SCK_MAX(fmax, tco_ps) - 1) / \
   RP23XX_QMI_SCK_MAX(fmax, tco_ps))

/* Whole clk_sys cycles covering ns, rounded up */

#define RP23XX_QMI_CYCLES(fsys, ns) \
  (((fsys) * 1ull * (ns) + 999999999ull) / 1000000000ull)

/* Chip select high between transfers: the QMI gives half an SCK (rounded
 * up to whole clk_sys cycles) plus MIN_DESELECT cycles.
 */

#define RP23XX_QMI_MIN_DESELECT(fsys, clkdiv, ns) \
  (RP23XX_QMI_CYCLES(fsys, ns) > ((clkdiv) + 1) / 2 ? \
   RP23XX_QMI_CYCLES(fsys, ns) - ((clkdiv) + 1) / 2 : 0)

/* Chip select low for at most ns (a PSRAM's tCEM), in units of 64 clk_sys
 * cycles: one unit less than fits, for the transfer under way when the
 * limit is reached (a whole new burst at SCK = clk_sys / 2 is some 60
 * cycles: 8-bit command, 24-bit address, 6 wait cycles, 8 bytes).
 */

#define RP23XX_QMI_MAX_SELECT(fsys, ns) \
  ((fsys) * 1ull * (ns) / 1000000000ull / 64 - 1)

#define RP23XX_QMI_TIMING(cooldown, pagebreak, setup, hold, maxsel, \
                          mindesel, rxdelay, clkdiv) \
  ((uint32_t)(cooldown) << RP23XX_QMI_TIMING_COOLDOWN_SHIFT | \
   (uint32_t)(pagebreak) << RP23XX_QMI_TIMING_PAGEBREAK_SHIFT | \
   ((setup) ? RP23XX_QMI_TIMING_SELECT_SETUP : 0) | \
   (uint32_t)(hold) << RP23XX_QMI_TIMING_SELECT_HOLD_SHIFT | \
   (uint32_t)(maxsel) << RP23XX_QMI_TIMING_MAX_SELECT_SHIFT | \
   (uint32_t)(mindesel) << RP23XX_QMI_TIMING_MIN_DESELECT_SHIFT | \
   (uint32_t)(rxdelay) << RP23XX_QMI_TIMING_RXDELAY_SHIFT | \
   (uint32_t)(clkdiv))

/* PAGEBREAK values */

#define RP23XX_QMI_PAGEBREAK_NONE 0
#define RP23XX_QMI_PAGEBREAK_256  1
#define RP23XX_QMI_PAGEBREAK_1024 2
#define RP23XX_QMI_PAGEBREAK_4096 3

#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_QMI_TIMING_H */
