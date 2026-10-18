/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_psram.c
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
 *
 * Driver for the external QSPI PSRAM on QMI chip select 1: an AP Memory
 * APS6404-family part (2-8 MB; the Pimoroni Pico Plus 2, the Waveshare
 * RP2350-PiZero) or an ISSI IS66WVS16M8FALL/FBLL (16 MB, two 8 MB dies).
 * Once configured it is directly addressable and writable at
 * RP23XX_PSRAM_BASE (0x11000000).  The part is told by its Read ID, its
 * size by the ID and by an aliasing probe (the smaller wins), and the QMI
 * timing is the part's own (docs/psram.md in the fpc tree).
 *
 * The QMI is the same serial interface that serves execute-in-place from the
 * flash on chip select 0.  Reprogramming it uses direct (non-XIP) mode,
 * which stalls all XIP fetches, so the detection and configuration code must
 * run from RAM with interrupts disabled -- hence the .time_critical section,
 * which the board linker scripts copy to RAM.  The sequence follows the
 * Raspberry Pi Pico SDK's setup_psram().
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#include <nuttx/irq.h>

#include <arch/board/board.h>

#include <arch/chip/psram.h>

#include "arm_internal.h"
#include "rp23xx_gpio.h"
#include "rp23xx_psram.h"
#include "rp23xx_qmi_timing.h"
#include "hardware/rp23xx_qmi.h"
#include "hardware/rp23xx_xip.h"
#include "hardware/rp23xx_memorymap.h"

#ifdef CONFIG_RP23XX_PSRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Detection and (re)configuration touch the QMI in direct mode, which stalls
 * execute-in-place from the flash.  The code that does so must therefore run
 * from RAM: the board linker scripts copy the .time_critical section there.
 */

#define RP23XX_PSRAM_RAMFUNC \
  locate_code(".time_critical.rp23xx_psram") noinline_function

/* GPIO function select for the XIP chip-select-1 line (PSRAM). */

#define RP23XX_GPIO_FUNC_XIP_CS1  9

/* PSRAM interface width for direct-mode transfers (quad). */

#define QMI_DIRECT_TX_IWIDTH_Q    (2 << RP23XX_QMI_DIRECT_TX_IWIDTH_SHIFT)

/* The part's limits (APS6404L; a board may give its own in board.h) */

#ifndef BOARD_PSRAM_MAX_FREQ
#  define BOARD_PSRAM_MAX_FREQ    133000000
#endif
#ifndef BOARD_PSRAM_TCO_PS
#  define BOARD_PSRAM_TCO_PS      5500      /* tACLK */
#endif
#ifndef BOARD_PSRAM_TCPH_NS
#  define BOARD_PSRAM_TCPH_NS     18        /* CS high between bursts */
#endif
#ifndef BOARD_PSRAM_TCEM_NS
#  define BOARD_PSRAM_TCEM_NS     8000      /* CS low at most (refresh) */
#endif

/* M1_TIMING from clk_sys and those (rp23xx_qmi_timing.h):
 *
 *   CLKDIV       SCK within 133 MHz and one round trip: 2 at 150 and
 *                200 MHz (75 and 100 MHz)
 *   RXDELAY      = CLKDIV: sampled one SCK after the launching edge
 *   MIN_DESELECT tCPH: 1 + 7 cycles at 150 MHz, 1 + 9 at 200 MHz (with
 *                the ESP-PSRAM64H's 50 ns)
 *   MAX_SELECT   tCEM less one burst: 17 x 64 cycles (7.3 us) at 150 MHz,
 *                24 x 64 (7.7 us) at 200 MHz
 *   SELECT_SETUP one more cycle between CS and SCK: CS1 is a bank 0 GPIO,
 *                up to 2.1 ns later than the QSPI pins (datasheet Table
 *                1288), which would leave some 3 ns of the half SCK at
 *                100 MHz
 *   SELECT_HOLD  3 cycles; COOLDOWN 64 cycles; PAGEBREAK at the part's
 *                1 KB pages (a burst across one is limited to 84 MHz)
 */

#define PSRAM_CLKDIV \
  RP23XX_QMI_CLKDIV(BOARD_SYS_FREQ, BOARD_PSRAM_MAX_FREQ, BOARD_PSRAM_TCO_PS)

#define RP23XX_PSRAM_M1_TIMING \
  RP23XX_QMI_TIMING(1, RP23XX_QMI_PAGEBREAK_1024, 1, 3, \
                    RP23XX_QMI_MAX_SELECT(BOARD_SYS_FREQ, \
                                          BOARD_PSRAM_TCEM_NS), \
                    RP23XX_QMI_MIN_DESELECT(BOARD_SYS_FREQ, PSRAM_CLKDIV, \
                                            BOARD_PSRAM_TCPH_NS), \
                    PSRAM_CLKDIV, PSRAM_CLKDIV)

static_assert(PSRAM_CLKDIV >= 1 && PSRAM_CLKDIV <= 7,
              "PSRAM: CLKDIV (and RXDELAY) out of range");
static_assert(RP23XX_QMI_MAX_SELECT(BOARD_SYS_FREQ, BOARD_PSRAM_TCEM_NS)
              >= 1 &&
              RP23XX_QMI_MAX_SELECT(BOARD_SYS_FREQ, BOARD_PSRAM_TCEM_NS)
              <= 63, "PSRAM: MAX_SELECT out of range");
static_assert(RP23XX_QMI_MIN_DESELECT(BOARD_SYS_FREQ, PSRAM_CLKDIV,
                                      BOARD_PSRAM_TCPH_NS) <= 31,
              "PSRAM: MIN_DESELECT out of range");

/* The ISSI IS66WVS16M8FALL/FBLL (data sheet Rev. A, 01/16/2025; a board
 * may give its own in board.h):
 *
 *   MAX_FREQ  104 MHz: QPI Quad IO Read EBh and Write 38h (Table 4.1)
 *   TCO_PS    tACLK, CLK falling to output valid, 7 ns max (7.6)
 *   TCPH_NS   tCPH, CE# high between operations, 1 tCLK min (7.6): 10 ns
 *             at its 100 MHz
 *   TCEM_NS   tCEM, CE# low at most, 4 us to 85 C (1 us to 105 C) (7.6),
 *             half the APS6404's 8 us
 *
 * Its pages are 1 KB and reads and writes always wrap within one (4.2),
 * and no burst may cross from one die to the other (the 8 MB boundary,
 * a page boundary: 2, note 1): the QMI's 1 KB page break covers both.
 *
 * With tCO 7 ns the round trip (rp23xx_qmi_timing.h) is 11 ns, more than
 * the 10 ns SCK at clk_sys / 3, so instead of a slower divider the sample
 * moves later: RXDELAY half clk_sys cycles after SCK's rising edge, the
 * fewest that reach 11 ns after the launching falling edge, but no later
 * than half a clk_sys cycle past the next falling edge (the data hold
 * there for tOH 1.5 ns (7.6) plus the pads' own delay).  At 300 MHz:
 * CLKDIV 3 (100 MHz), RXDELAY 4 (11.7 ns); 200: 2, 3; 150: 2, 2.
 * Measured on the first board at 300 MHz (room temperature): RXDELAY 3
 * to 7 pass over all 16 MB, 2 fails everywhere.
 */

#ifndef BOARD_PSRAM_ISSI_MAX_FREQ
#  define BOARD_PSRAM_ISSI_MAX_FREQ  104000000
#endif
#ifndef BOARD_PSRAM_ISSI_TCO_PS
#  define BOARD_PSRAM_ISSI_TCO_PS    7000
#endif
#ifndef BOARD_PSRAM_ISSI_TCPH_NS
#  define BOARD_PSRAM_ISSI_TCPH_NS   10
#endif
#ifndef BOARD_PSRAM_ISSI_TCEM_NS
#  define BOARD_PSRAM_ISSI_TCEM_NS   4000
#endif

#ifndef BOARD_PSRAM_ISSI_CLKDIV
#  define BOARD_PSRAM_ISSI_CLKDIV \
  ((BOARD_SYS_FREQ + BOARD_PSRAM_ISSI_MAX_FREQ - 1) / \
   BOARD_PSRAM_ISSI_MAX_FREQ)
#endif

/* Half clk_sys cycles from the launching falling edge to valid data */

#define PSRAM_ISSI_VALID_HALVES \
  ((2ull * BOARD_SYS_FREQ * (RP23XX_QMI_ROUNDTRIP_PS + \
                             BOARD_PSRAM_ISSI_TCO_PS) + \
    999999999999ull) / 1000000000000ull)

#ifndef BOARD_PSRAM_ISSI_RXDELAY
#  define BOARD_PSRAM_ISSI_RXDELAY \
  (PSRAM_ISSI_VALID_HALVES > 2 * BOARD_PSRAM_ISSI_CLKDIV ? \
   PSRAM_ISSI_VALID_HALVES - BOARD_PSRAM_ISSI_CLKDIV : \
   BOARD_PSRAM_ISSI_CLKDIV)
#endif

#define RP23XX_PSRAM_ISSI_M1_TIMING \
  RP23XX_QMI_TIMING(1, RP23XX_QMI_PAGEBREAK_1024, 1, 3, \
                    RP23XX_QMI_MAX_SELECT(BOARD_SYS_FREQ, \
                                          BOARD_PSRAM_ISSI_TCEM_NS), \
                    RP23XX_QMI_MIN_DESELECT(BOARD_SYS_FREQ, \
                                            BOARD_PSRAM_ISSI_CLKDIV, \
                                            BOARD_PSRAM_ISSI_TCPH_NS), \
                    BOARD_PSRAM_ISSI_RXDELAY, BOARD_PSRAM_ISSI_CLKDIV)

static_assert(BOARD_PSRAM_ISSI_CLKDIV >= 1 && BOARD_PSRAM_ISSI_CLKDIV <= 7,
              "PSRAM (ISSI): CLKDIV out of range");
static_assert(BOARD_PSRAM_ISSI_RXDELAY <= 7 &&
              BOARD_PSRAM_ISSI_RXDELAY <= BOARD_PSRAM_ISSI_CLKDIV + 1,
              "PSRAM (ISSI): no RXDELAY within the hold time");
static_assert(RP23XX_QMI_MAX_SELECT(BOARD_SYS_FREQ,
                                    BOARD_PSRAM_ISSI_TCEM_NS) >= 1 &&
              RP23XX_QMI_MAX_SELECT(BOARD_SYS_FREQ,
                                    BOARD_PSRAM_ISSI_TCEM_NS) <= 63,
              "PSRAM (ISSI): MAX_SELECT out of range");

/* The rest of the QMI M1 registers for the APS6404 in quad (QPI) mode:
 *
 * RFMT:   quad prefix/addr/suffix/dummy/data, 8-bit prefix, 24 dummy bits
 *         (the part's 6 wait cycles for EBh, at any clock up to 133 MHz).
 * RCMD:   read prefix 0xeb.
 * WFMT:   quad prefix/addr/suffix/dummy/data, 8-bit prefix, no dummy.
 * WCMD:   write prefix 0x38.
 */

#define RP23XX_PSRAM_M1_RFMT      0x000612aa
#define RP23XX_PSRAM_M1_RCMD      0x000000eb
#define RP23XX_PSRAM_M1_WFMT      0x000012aa
#define RP23XX_PSRAM_M1_WCMD      0x00000038

/* The ISSI part takes the same: in QPI mode EBh with 6 wait cycles to
 * 104 MHz, 38h (or 02h) with none (Table 4.1, 5.4, 5.5).
 */

/* APS6404 SPI opcodes used during detection and mode switching. */

#define PSRAM_CMD_READ_ID         0x9f
#define PSRAM_CMD_RSTEN           0x66
#define PSRAM_CMD_RST             0x99
#define PSRAM_CMD_QUAD_ENABLE     0x35

#define PSRAM_KGD_PASS            0x5d

/* Read ID (9Fh) in SPI mode: command, 24 address bits (don't care), then
 * the ID with no wait cycles: MF ID, KGD (5Dh: passed), EID.  AP Memory:
 * MF ID 0Dh, EID[47:45] the density (APS6404L data sheet).  ISSI: MF ID
 * 9Dh, KGD 5Dh (55h: not passed), ID bits 47-45 011 64 Mb, 100 128 Mb
 * (IS66WVS16M8 5.7, Table 6.2).
 */

#define PSRAM_MFID_APMEMORY       0x0d
#define PSRAM_MFID_ISSI           0x9d
#define PSRAM_ID_BYTES            8

/* The aliasing probe: a word at each 4 MB, through the uncached window */

#define PSRAM_PROBE_STEP          (4 * 1024 * 1024)
#define PSRAM_PROBE_MARK(i) \
  (0x5a17c0deu ^ ((uint32_t)(i) * 0x01010101u))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static size_t g_psram_size;
static uint32_t g_psram_timing;
static struct rp23xx_psram_info_s g_psram_info;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_psram_apply_format
 *
 * Description:
 *   Program the QMI M1 timing/format/command registers and mark the region
 *   writable.  Shared by the initial configuration and the post-flash
 *   restore.  Runs from RAM.
 *
 ****************************************************************************/

static void RP23XX_PSRAM_RAMFUNC
rp23xx_psram_apply_format(void)
{
  putreg32(g_psram_timing,         RP23XX_QMI_M1_TIMING);
  putreg32(RP23XX_PSRAM_M1_RFMT,   RP23XX_QMI_M1_RFMT);
  putreg32(RP23XX_PSRAM_M1_RCMD,   RP23XX_QMI_M1_RCMD);
  putreg32(RP23XX_PSRAM_M1_WFMT,   RP23XX_QMI_M1_WFMT);
  putreg32(RP23XX_PSRAM_M1_WCMD,   RP23XX_QMI_M1_WCMD);

  /* Allow the memory-mapped window to accept writes. */

  putreg32(getreg32(RP23XX_XIP_CTRL_BASE) | RP23XX_XIP_CTRL_WRITABLE_M1,
           RP23XX_XIP_CTRL_BASE);
}

/****************************************************************************
 * Name: rp23xx_psram_detect
 *
 * Description:
 *   Read the PSRAM device ID over the QMI direct interface, reset the part
 *   and put it in quad (QPI) mode, and note what it is (g_psram_info,
 *   g_psram_timing).  1 if a part answered with a good KGD, else 0.
 *   Runs from RAM with the caller holding interrupts disabled.
 *
 ****************************************************************************/

static int RP23XX_PSRAM_RAMFUNC
rp23xx_psram_detect(void)
{
  uint8_t mfid = 0;
  uint8_t kgd = 0;
  uint8_t eid = 0;
  size_t size;
  size_t i;

  /* Enable direct mode at a conservative clock divisor.  No BUSY wait is
   * needed for the previous XIP transfer's cooldown: it drains before the
   * first chip-select assertion below (verified on hardware).
   */

  putreg32((30 << RP23XX_QMI_DIRECT_CSR_CLKDIV_SHIFT) |
           RP23XX_QMI_DIRECT_CSR_EN, RP23XX_QMI_DIRECT_CSR);

  /* Nudge the part out of any quad-continuation mode left by a prior init by
   * clocking one quad byte with CS asserted.
   */

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) |
           RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);
  putreg32(RP23XX_QMI_DIRECT_TX_OE | QMI_DIRECT_TX_IWIDTH_Q | 0xf5,
           RP23XX_QMI_DIRECT_TX);

  /* This BUSY wait is required: the frame must finish shifting before the
   * chip select is deasserted, else the nudge is truncated, the part stays
   * in quad-continuation mode, and the ID read below returns garbage so
   * detection fails (verified on hardware).
   */

  while ((getreg32(RP23XX_QMI_DIRECT_CSR) & RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
    {
    }

  (void)getreg32(RP23XX_QMI_DIRECT_RX);
  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
           ~RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);

  /* Read the identification: command 0x9f, three address bytes, then the
   * ID: MF ID (byte 4), Known Good Die (byte 5), EID (6 on).  All eight ID
   * bytes are kept for rp23xx_psram_info().
   */

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) |
           RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);
  for (i = 0; i < 4 + PSRAM_ID_BYTES; i++)
    {
      uint8_t rx;

      putreg32(i == 0 ? PSRAM_CMD_READ_ID : 0xff, RP23XX_QMI_DIRECT_TX);

      /* This BUSY wait is required: the byte must finish shifting in before
       * DIRECT_RX is read, else the KGD/EID bytes are sampled early as
       * garbage and detection fails (verified on hardware).  A TXEMPTY wait
       * ahead of it is redundant -- BUSY already covers the whole transfer.
       */

      while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
              RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
        {
        }

      rx = (uint8_t)getreg32(RP23XX_QMI_DIRECT_RX);
      if (i >= 4)
        {
          g_psram_info.id[i - 4] = rx;
        }

      if (i == 4)
        {
          mfid = rx;
        }
      else if (i == 5)
        {
          kgd = rx;
        }
      else if (i == 6)
        {
          eid = rx;
        }
    }

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
           ~(RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N | RP23XX_QMI_DIRECT_CSR_EN),
           RP23XX_QMI_DIRECT_CSR);

  if (kgd != PSRAM_KGD_PASS)
    {
      return 0;
    }

  /* Reset the device and place it in quad mode: RSTEN, RST, quad-enable. */

  putreg32((30 << RP23XX_QMI_DIRECT_CSR_CLKDIV_SHIFT) |
           RP23XX_QMI_DIRECT_CSR_EN, RP23XX_QMI_DIRECT_CSR);

  for (i = 0; i < 3; i++)
    {
      /* The command byte is selected with a ternary rather than a lookup
       * table on purpose: a "static const" array is emitted to .rodata in
       * flash, and reading it here -- while the QMI is in direct mode and
       * flash is inaccessible -- would fault.  An immediate keeps it in RAM.
       */

      uint8_t cmd = (i == 0) ? PSRAM_CMD_RSTEN :
                    (i == 1) ? PSRAM_CMD_RST   :
                               PSRAM_CMD_QUAD_ENABLE;

      putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) |
               RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);
      putreg32(cmd, RP23XX_QMI_DIRECT_TX);

      /* This BUSY wait is required: the command must finish shifting before
       * the chip select is deasserted.  Cutting it short truncates the
       * RSTEN/RST/quad-enable commands, so the part is not reset into quad
       * mode and later accesses fail or wedge the shared QMI bus (verified
       * on hardware).  No extra settling delay is needed after CS deassert:
       * the QMI's own minimum-deselect timing covers it.
       */

      while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
              RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
        {
        }

      putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
               ~RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);

      (void)getreg32(RP23XX_QMI_DIRECT_RX);
    }

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
           ~(RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N | RP23XX_QMI_DIRECT_CSR_EN),
           RP23XX_QMI_DIRECT_CSR);

  /* The part, its timing, and its size from the ID (0: not known; the
   * probe decides).  The names are string literals: only their addresses
   * are stored here, nothing is read from the flash.
   */

  size = 0;
  if (mfid == PSRAM_MFID_ISSI)
    {
      g_psram_info.chip = RP23XX_PSRAM_ISSI;
      g_psram_info.name = "IS66WVS16M8";
      g_psram_timing = RP23XX_PSRAM_ISSI_M1_TIMING;
      if ((eid >> 5) == 3)
        {
          size = 8 * 1024 * 1024;
        }
      else if ((eid >> 5) == 4)
        {
          size = 16 * 1024 * 1024;
        }
    }
  else
    {
      g_psram_info.chip = mfid == PSRAM_MFID_APMEMORY ?
                          RP23XX_PSRAM_APMEMORY : RP23XX_PSRAM_UNKNOWN;
      g_psram_info.name = mfid == PSRAM_MFID_APMEMORY ?
                          "APS6404" : "unknown (KGD 5Dh)";
      g_psram_timing = RP23XX_PSRAM_M1_TIMING;

      /* The APS6404 family's EID (2, 4, 8 MB) */

      if (eid == 0x26 || (eid >> 5) == 2)
        {
          size = 8 * 1024 * 1024;
        }
      else if ((eid >> 5) == 0)
        {
          size = 2 * 1024 * 1024;
        }
      else if ((eid >> 5) == 1)
        {
          size = 4 * 1024 * 1024;
        }
    }

  g_psram_info.id_size = size;
  return 1;
}

/****************************************************************************
 * Name: rp23xx_psram_probe
 *
 * Description:
 *   The size by aliasing: a word at 0, then one at each 4 MB above it,
 *   through the uncached window; the first that changes the word at 0 is
 *   where the address wraps.  Then each word written below that must read
 *   back as written (a die that does not answer ends the PSRAM there).
 *   Destroys those words: at boot, before anything is in the PSRAM.
 *
 ****************************************************************************/

static size_t RP23XX_PSRAM_RAMFUNC
rp23xx_psram_probe(void)
{
  volatile uint32_t *base = (volatile uint32_t *)RP23XX_PSRAM_UNCACHED;
  size_t size = RP23XX_PSRAM_WINDOW;
  size_t off;

  base[0] = PSRAM_PROBE_MARK(0);
  if (base[0] != PSRAM_PROBE_MARK(0))
    {
      return 0;
    }

  for (off = PSRAM_PROBE_STEP; off < RP23XX_PSRAM_WINDOW;
       off += PSRAM_PROBE_STEP)
    {
      base[off / 4] = PSRAM_PROBE_MARK(off / PSRAM_PROBE_STEP);
      if (base[0] != PSRAM_PROBE_MARK(0))
        {
          size = off;
          break;
        }
    }

  for (off = 0; off < size; off += PSRAM_PROBE_STEP)
    {
      if (base[off / 4] != PSRAM_PROBE_MARK(off / PSRAM_PROBE_STEP))
        {
          size = off;
          break;
        }
    }

  return size;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_psramconfig
 ****************************************************************************/

size_t RP23XX_PSRAM_RAMFUNC
rp23xx_psramconfig(void)
{
  irqstate_t flags;
  size_t size;

  rp23xx_gpio_set_function(CONFIG_RP23XX_PSRAM_CS1_GPIO,
                           RP23XX_GPIO_FUNC_XIP_CS1);

  flags = up_irq_save();

  size = 0;
  if (rp23xx_psram_detect())
    {
      rp23xx_psram_apply_format();

      /* The smaller of the ID's size and the probe's */

      g_psram_info.probe_size = rp23xx_psram_probe();
      size = g_psram_info.probe_size;
      if (g_psram_info.id_size != 0 && g_psram_info.id_size < size)
        {
          size = g_psram_info.id_size;
        }

      /* A board without flash runs from a second PSRAM on CS0 (the same
       * part: docs/psram-boot.md in the fpc tree), left by its loader at a
       * timing safe at any clock: now the clock is up, the part's own.
       * Its contents are the image: no commands, only the timing.
       */

      if ((getreg32(RP23XX_XIP_CTRL_BASE) & RP23XX_XIP_CTRL_WRITABLE_M0)
          != 0)
        {
          putreg32(g_psram_timing, RP23XX_QMI_M0_TIMING);
          (void)getreg32(RP23XX_QMI_M0_TIMING);
        }
    }

  up_irq_restore(flags);

  g_psram_info.size = size;
  g_psram_info.timing = g_psram_timing;
  g_psram_size = size;
  return size;
}

/****************************************************************************
 * Name: rp23xx_psram_size
 ****************************************************************************/

size_t rp23xx_psram_size(void)
{
  return g_psram_size;
}

/****************************************************************************
 * Name: rp23xx_psram_info
 ****************************************************************************/

void rp23xx_psram_info(struct rp23xx_psram_info_s *info)
{
  *info = g_psram_info;
  info->timing = g_psram_timing;
}

/****************************************************************************
 * Name: rp23xx_psram_set_timing
 ****************************************************************************/

void rp23xx_psram_set_timing(uint32_t timing)
{
  irqstate_t flags = up_irq_save();

  g_psram_timing = timing;
  putreg32(timing, RP23XX_QMI_M1_TIMING);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: rp23xx_psram_restore
 ****************************************************************************/

void RP23XX_PSRAM_RAMFUNC
rp23xx_psram_restore(void)
{
  if (g_psram_size != 0)
    {
      rp23xx_psram_apply_format();
    }
}

#endif /* CONFIG_RP23XX_PSRAM */
