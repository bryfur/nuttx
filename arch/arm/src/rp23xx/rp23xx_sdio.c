/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_sdio.c
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

/* A host for the 4-bit SD bus made of one PIO block and three DMA
 * channels, the lower half (include/nuttx/sdio.h) under which
 * drivers/mmcsd/mmcsd_sdio.c runs a microSD card in 4-bit mode.
 *
 * Three state machines, all clocked alike (the divider "div": one SD clock
 * is six of their ticks, so 300 MHz / 6 = 50 MHz at div 1):
 *
 *   SM0  the clock and the command line.  It makes the clock all the time
 *        (side-set), idle or not; it sends a 48-bit command when the CPU
 *        has put two words in its FIFO and samples the response, the CMD
 *        input inverted (GPIO INOVER) so that "jmp pin" finds the start
 *        bit.  Commands change on the falling edge; the response is
 *        sampled on the falling or the rising edge (its "variant"), the
 *        input synchronizer bypassed or not: four sampling points.
 *   SM1  data in.  It finds each rising edge of the clock (wait gpio),
 *        waits rxk ticks and samples DAT0, until the start bit; then it
 *        samples DAT0-3 for the block and its CRCs every six ticks: being
 *        clocked like SM0 it stays in step with the clock without looking
 *        at it again.
 *   SM2  data out.  Per block: the start bit, the block, the CRCs and the
 *        end bit from its FIFO, then 16 samples of DAT0 (the CRC status
 *        token, in the second or third clock after the end bit) pushed to
 *        the CPU, then waits until the card stops holding DAT0 low (busy).
 *
 * The 13 + 7 + 12 instructions fill the block's 32; they are
 * assembled here, with the sampling points and delays as parameters.
 *
 * Data goes by DMA: one channel moves the FIFO's words, a second loads its
 * control blocks (per block: the data to or from the caller's buffer, the
 * CRCs to or from a table), a third collects the write status words.  DMA
 * byte-swaps, so that the first nibble on the bus is the high nibble of the
 * first byte in memory.
 *
 * The four lines' CRC16s are computed together, 32 bits (8 clocks) at a
 * time: interleaving four CRCs by nibble is one CRC whose polynomial is
 * the original with every power of x times four,
 * x^64 + x^48 + x^20 + 1, and the familiar table-free byte step of
 * CRC-CCITT carries over with every shift times four (sdio_crc16x4()).
 *
 * The sampling points are tuned whenever a 4-bit transfer clock is set: a
 * few CMD13s for each command sampling point and a few CMD6 queries (a
 * 64-byte status block, CRC-checked, no side effects) for each data delay,
 * and the middle of each passing range is kept.  A read whose CRCs fail,
 * or a write the card rejects, is done again (CMD12, the command again);
 * a second failure lowers the clock first.
 *
 * Prior art, read and learnt from, not copied: ZuluSCSI/BlueSCSI's RP2040
 * SDIO (rp2040_sdio.pio and rp2040_sdio.cpp, GPL-3.0) and carlk3's
 * no-OS-FatFS-SD-SDIO-SPI-RPi-Pico, whose SDIO back end derives from it
 * (Apache-2.0 wrapper, GPL-3.0 core): there too one state machine owns the
 * clock and the command line and the data lines' CRCs are computed in
 * software.  The PIO programs, the 64-bit CRC step and the rest here are
 * written afresh.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <inttypes.h>
#include <syslog.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "rp23xx_gpio.h"
#include "rp23xx_dmac.h"
#include "rp23xx_sdio.h"
#include "hardware/rp23xx_memorymap.h"
#include "hardware/rp23xx_pio.h"
#include "hardware/rp23xx_io_bank0.h"
#include "hardware/rp23xx_pads_bank0.h"
#include "hardware/rp23xx_timer.h"
#include "hardware/rp23xx_dma.h"
#include "hardware/rp23xx_dreq.h"

#ifdef CONFIG_RP23XX_SDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SDIO_BLOCKSETUP
#  error "CONFIG_SDIO_BLOCKSETUP is required"
#endif

#ifndef CONFIG_SDIO_DMA
#  error "CONFIG_SDIO_DMA is required"
#endif

#define SDIO_PIO         CONFIG_RP23XX_SDIO_PIO
#define PIO_BASE         RP23XX_PIO_BASE(SDIO_PIO)
#define CLK_GPIO         CONFIG_RP23XX_SDIO_CLK_GPIO
#define CMD_GPIO         CONFIG_RP23XX_SDIO_CMD_GPIO
#define D0_GPIO          CONFIG_RP23XX_SDIO_D0_GPIO
#define MAXBLOCKS        CONFIG_RP23XX_SDIO_MAXBLOCKS

#if CONFIG_MMCSD_MULTIBLOCK_LIMIT == 0 || \
    CONFIG_MMCSD_MULTIBLOCK_LIMIT > CONFIG_RP23XX_SDIO_MAXBLOCKS
#  error "CONFIG_MMCSD_MULTIBLOCK_LIMIT must be 1..CONFIG_RP23XX_SDIO_MAXBLOCKS"
#endif

/* A PIO block reaches 32 GPIOs: 0-31, or 16-47 (GPIOBASE 16) */

#if CLK_GPIO >= 32 || CMD_GPIO >= 32 || D0_GPIO + 3 >= 32
#  define SDIO_GPIOBASE  16
#else
#  define SDIO_GPIOBASE  0
#endif

#if CLK_GPIO < SDIO_GPIOBASE || CMD_GPIO < SDIO_GPIOBASE || \
    D0_GPIO < SDIO_GPIOBASE || D0_GPIO + 3 > SDIO_GPIOBASE + 31
#  error "SDIO pins must fit in GPIO 0-31 or GPIO 16-47"
#endif

#define VCLK             (CLK_GPIO - SDIO_GPIOBASE)   /* PIO's pin numbers */
#define VCMD             (CMD_GPIO - SDIO_GPIOBASE)
#define VD0              (D0_GPIO - SDIO_GPIOBASE)

#define SM_CMD           0
#define SM_RX            1
#define SM_TX            2
#define SM_ALL           7

/* Instruction memory */

#define CMD_OFS          0
#define CMD_LEN          13
#define CMD_IDLE         (CMD_OFS + 11)                /* wrap target */
#define CMD_RW0          (CMD_OFS + 6)                 /* waiting for the */
#define CMD_RW1          (CMD_OFS + 7)                 /* response */
#define RX_OFS           (CMD_OFS + CMD_LEN)
#define RX_LEN           7
#define TX_OFS           (RX_OFS + RX_LEN)
#define TX_LEN           12

#if TX_OFS + TX_LEN > 32
#  error "PIO programs do not fit"
#endif

/* PIO registers */

#define PIO_REG(o)       (PIO_BASE + (o))
#define PIO_CTRL         PIO_REG(RP23XX_PIO_CTRL_OFFSET)
#define PIO_FSTAT        PIO_REG(RP23XX_PIO_FSTAT_OFFSET)
#define PIO_FDEBUG       PIO_REG(RP23XX_PIO_FDEBUG_OFFSET)
#define PIO_TXF(sm)      PIO_REG(RP23XX_PIO_TXF_OFFSET(sm))
#define PIO_RXF(sm)      PIO_REG(RP23XX_PIO_RXF_OFFSET(sm))
#define PIO_SYNCBYPASS   PIO_REG(RP23XX_PIO_INPUT_SYNC_BYPASS_OFFSET)
#define PIO_INSTR_MEM(i) PIO_REG(RP23XX_PIO_INSTR_MEM_OFFSET(i))
#define PIO_CLKDIV(sm)   PIO_REG(RP23XX_PIO_SM_CLKDIV_OFFSET(sm))
#define PIO_EXECCTRL(sm) PIO_REG(RP23XX_PIO_SM_EXECCTRL_OFFSET(sm))
#define PIO_SHIFTCTRL(sm) PIO_REG(RP23XX_PIO_SM_SHIFTCTRL_OFFSET(sm))
#define PIO_ADDR(sm)     PIO_REG(RP23XX_PIO_SM_ADDR_OFFSET(sm))
#define PIO_INSTR(sm)    PIO_REG(RP23XX_PIO_SM_INSTR_OFFSET(sm))
#define PIO_PINCTRL(sm)  PIO_REG(RP23XX_PIO_SM_PINCTRL_OFFSET(sm))
#define PIO_GPIOBASE     PIO_REG(RP23XX_PIO_GPIOBASE_OFFSET)

#define REG_XOR(a)       ((a) + 0x1000)               /* atomic aliases */
#define REG_SET(a)       ((a) + 0x2000)
#define REG_CLR(a)       ((a) + 0x3000)

#define FSTAT_RXEMPTY(sm) (1 << (RP23XX_PIO_FSTAT_RXEMPTY_SHIFT + (sm)))
#define FSTAT_TXEMPTY(sm) (1 << (RP23XX_PIO_FSTAT_TXEMPTY_SHIFT + (sm)))

#define DREQ_TX(sm)      (RP23XX_DMA_DREQ_PIO0_TX0 + 8 * SDIO_PIO + (sm))
#define DREQ_RX(sm)      (RP23XX_DMA_DREQ_PIO0_RX0 + 8 * SDIO_PIO + (sm))

/* PIO instructions */

#define I_JMP(c, a)      (0x0000 | ((c) << 5) | (a))
#define I_WAIT(p, s, i)  (0x2000 | ((p) << 7) | ((s) << 5) | (i))
#define I_IN(s, n)       (0x4000 | ((s) << 5) | ((n) & 31))
#define I_OUT(d, n)      (0x6000 | ((d) << 5) | ((n) & 31))
#define I_PUSH           0x8020                       /* push block */
#define I_PULL           0x80a0                       /* pull block */
#define I_MOV(d, s)      (0xa000 | ((d) << 5) | (s))
#define I_SET(d, v)      (0xe000 | ((d) << 5) | (v))

#define C_ALWAYS         0                            /* jmp conditions */
#define C_NOTX           1
#define C_XDEC           2
#define C_YDEC           4
#define C_PIN            6

#define S_GPIO           0                            /* wait sources */
#define S_PIN            1

#define R_PINS           0                            /* in/out/mov/set */
#define R_X              1
#define R_Y              2
#define R_NULL           3
#define R_PINDIRS        4                            /* out, set */
#define R_STATUS         5                            /* mov source */
#define R_ISR            6
#define R_OSR            7

#define SIDE(v)          ((v) << 12)                  /* SM0: 1 side-set bit */
#define DLY(d)           ((d) << 8)

/* Clocks */

#define SYS_MHZ          (BOARD_SYS_FREQ / 1000000)
#define DIV_FOR(hz)      ((BOARD_SYS_FREQ + 6 * (hz) - 1) / (6 * (hz)))
#define DIV_ID           DIV_FOR(400000)
#define DIV_NARROW       DIV_FOR(6250000)             /* the 1-bit SCR read */
#define DIV_DS           DIV_FOR(25000000)
#define DIV_HS           DIV_FOR(50000000)
#define DIV_SLOWEST      8                            /* lowest after errors */
#define KHZ(div)         ((unsigned int)(BOARD_SYS_FREQ / 6000 / (div)))

/* Command sampling points */

#define VAR_RISE         1                            /* bit 0: rising edge */
#define VAR_BYPASS       2                            /* bit 1: no synchronizer */

/* Data transfers */

#define DIR_NONE         0
#define DIR_RX           1
#define DIR_TX           2

#define XFER_IDLE        0
#define XFER_BUSY        1
#define XFER_DONE        2
#define XFER_ERROR       3

#define BOUNCE_BYTES     (512 + 8)
#define RETRIES          3                            /* attempts in all */

#define START_WORD       0xfffffff0u                  /* 7 idle nibbles, start */
#define END_WORD         0xffffffffu                  /* end nibble (and idle) */

#define BUSY_TIMEOUT_US  1000000                      /* card busy at most */

#define SDIO_WAITALL     (SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | \
                          SDIOWAIT_ERROR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_sdio_s
{
  struct sdio_dev_s dev;          /* Must be first */

  /* The bus */

  uint16_t div;                   /* Divider of the three machines */
  uint16_t floor;                 /* Smallest divider allowed, after errors */
  uint16_t nominal;               /* Divider mmcsd asked for */
  bool     widebus;
  bool     tuned;                 /* Sampling points tuned for div */
  uint8_t  cmdvar;                /* Command sampling point (VAR_*) */
  uint8_t  rxk;                   /* Data in: ticks after the rising edge */
  uint8_t  txfall;                /* Data out: launch after the falling edge */
  uint8_t  txk;                   /* ... and this many ticks */
  uint8_t  stk;                   /* Status token sampling delay */

  /* The last command */

  uint32_t cmd;
  uint32_t arg;
  uint32_t resp[5];
  int      respret;               /* waitresponse()'s result */
  uint8_t  rxbits;                /* response bits after the start bit */
  bool     app;                   /* the last command was CMD55 */
  bool     rcavalid;
  uint16_t rca;
  uint32_t cmd23;                 /* CMD23's count before a data command */
  bool     nowrite;               /* writes refused (a trial) */
  bool     refused;               /* the last command was not sent */

  /* The data transfer */

  uint16_t blocklen;
  uint16_t nblocks;
  uint8_t  dir;                   /* DIR_* */
  bool     bounce;                /* through g_bounce */
  uint8_t  attempt;
  uint8_t *rxbuf;
  const uint8_t *txbuf;
  size_t   nbytes;
  uint32_t datacmd;               /* the command that started it ... */
  uint32_t dataarg;               /* ... and its argument, for retries */
  uint32_t datacmd23;             /* its CMD23 count, or 0 */
  volatile uint8_t xfer;          /* XFER_* */

  /* Events */

  sem_t    waitsem;
  sdio_eventset_t waitevents;
  volatile sdio_eventset_t wkupevent;
  uint32_t waitms;
  struct wdog_s waitwdog;
  spinlock_t lock;

  /* Card detection (no switch: the board reports it) */

  sdio_statset_t cdstatus;
  sdio_eventset_t cbevents;
  worker_t callback;
  void    *cbarg;
  struct work_s cbwork;

  /* DMA */

  DMA_HANDLE dmactrl;             /* loads the control blocks */
  DMA_HANDLE dmadata;             /* moves the data */
  DMA_HANDLE dmastat;             /* collects write status words */

  /* Counts */

  uint32_t crcerrors;
  uint32_t timeouts;
  uint32_t retries;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDIO interface */

static void rp23xx_reset(struct sdio_dev_s *dev);
static sdio_capset_t rp23xx_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t rp23xx_status(struct sdio_dev_s *dev);
static void rp23xx_widebus(struct sdio_dev_s *dev, bool enable);
static void rp23xx_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int  rp23xx_attach(struct sdio_dev_s *dev);
static int  rp23xx_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t arg);
static void rp23xx_blocksetup(struct sdio_dev_s *dev,
                              unsigned int blocklen, unsigned int nblocks);
static int  rp23xx_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                             size_t nbytes);
static int  rp23xx_sendsetup(struct sdio_dev_s *dev, const uint8_t *buffer,
                             size_t nbytes);
static int  rp23xx_cancel(struct sdio_dev_s *dev);
static int  rp23xx_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  rp23xx_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                uint32_t *rshort);
static int  rp23xx_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t rlong[4]);
static int  rp23xx_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *rshort);
static void rp23xx_waitenable(struct sdio_dev_s *dev,
                              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t rp23xx_eventwait(struct sdio_dev_s *dev);
static void rp23xx_callbackenable(struct sdio_dev_s *dev,
                                  sdio_eventset_t eventset);
static int  rp23xx_registercallback(struct sdio_dev_s *dev,
                                    worker_t callback, void *arg);
static int  rp23xx_dmapreflight(struct sdio_dev_s *dev,
                                const uint8_t *buffer, size_t buflen);
static int  rp23xx_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                                size_t buflen);
static int  rp23xx_dmasendsetup(struct sdio_dev_s *dev,
                                const uint8_t *buffer, size_t buflen);

static void sdio_restart_wd(struct rp23xx_sdio_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rp23xx_sdio_s g_sdio =
{
  .dev =
  {
    .reset            = rp23xx_reset,
    .capabilities     = rp23xx_capabilities,
    .status           = rp23xx_status,
    .widebus          = rp23xx_widebus,
    .clock            = rp23xx_clock,
    .attach           = rp23xx_attach,
    .sendcmd          = rp23xx_sendcmd,
    .blocksetup       = rp23xx_blocksetup,
    .recvsetup        = rp23xx_recvsetup,
    .sendsetup        = rp23xx_sendsetup,
    .cancel           = rp23xx_cancel,
    .waitresponse     = rp23xx_waitresponse,
    .recv_r1          = rp23xx_recvshortcrc,
    .recv_r2          = rp23xx_recvlong,
    .recv_r3          = rp23xx_recvshort,
    .recv_r4          = rp23xx_recvshort,
    .recv_r5          = rp23xx_recvshortcrc,
    .recv_r6          = rp23xx_recvshortcrc,
    .recv_r7          = rp23xx_recvshortcrc,
    .waitenable       = rp23xx_waitenable,
    .eventwait        = rp23xx_eventwait,
    .callbackenable   = rp23xx_callbackenable,
    .registercallback = rp23xx_registercallback,
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = rp23xx_dmapreflight,
#endif
    .dmarecvsetup     = rp23xx_dmarecvsetup,
    .dmasendsetup     = rp23xx_dmasendsetup,
  },
  .waitsem = SEM_INITIALIZER(0),
};

/* DMA control blocks (data and CRCs of each block, and the end), the
 * CRCs received, the words sent after each block, the write status words
 * and a buffer for transfers the DMA cannot make into the caller's.
 */

static dma_control_block_t g_cb[2 * MAXBLOCKS + 1] aligned_data(16);
static uint32_t g_rxcrc[2 * MAXBLOCKS];
static uint32_t g_txtail[4 * MAXBLOCKS];
static uint32_t g_txstat[MAXBLOCKS];
static uint32_t g_bounce[BOUNCE_BYTES / 4];

/****************************************************************************
 * Private Functions: helpers
 ****************************************************************************/

static inline uint32_t sdio_now(void)
{
  return getreg32(RP23XX_TIMER_BASE(0) + RP23XX_TIMER_TIMERAWL_OFFSET);
}

static inline bool sdio_d0(void)
{
  return rp23xx_gpio_get(D0_GPIO);
}

static inline void sm_exec(int sm, uint32_t instr)
{
  putreg32(instr, PIO_INSTR(sm));
}

static inline void sm_enable(uint32_t mask)
{
  putreg32(mask, REG_SET(PIO_CTRL));
}

static inline void sm_disable(uint32_t mask)
{
  putreg32(mask, REG_CLR(PIO_CTRL));
}

static void sm_clear_fifos(int sm)
{
  /* Changing a FIFO join flushes both FIFOs: twice leaves it as it was */

  putreg32(RP23XX_PIO_SM_SHIFTCTRL_FJOIN_RX, REG_XOR(PIO_SHIFTCTRL(sm)));
  putreg32(RP23XX_PIO_SM_SHIFTCTRL_FJOIN_RX, REG_XOR(PIO_SHIFTCTRL(sm)));
}

/* A stopped machine back to its program's start, FIFOs and shift
 * counters empty.
 */

static void sm_reset(int sm, int pc)
{
  sm_disable(1 << sm);
  putreg32(1 << (RP23XX_PIO_CTRL_SM_RESTART_SHIFT + sm), REG_SET(PIO_CTRL));
  sm_clear_fifos(sm);
  putreg32(1 << sm, PIO_FDEBUG);
  putreg32(1 << (sm + 8), PIO_FDEBUG);
  putreg32(1 << (sm + 16), PIO_FDEBUG);
  putreg32(1 << (sm + 24), PIO_FDEBUG);
  sm_exec(sm, I_JMP(C_ALWAYS, pc));
}

/* Y of a stopped machine, 15 bits, by way of its ISR */

static void sm_set_y(int sm, uint32_t v)
{
  sm_exec(sm, I_MOV(R_ISR, R_NULL));
  sm_exec(sm, I_SET(R_X, (v >> 10) & 31));
  sm_exec(sm, I_IN(R_X, 5));
  sm_exec(sm, I_SET(R_X, (v >> 5) & 31));
  sm_exec(sm, I_IN(R_X, 5));
  sm_exec(sm, I_SET(R_X, v & 31));
  sm_exec(sm, I_IN(R_X, 5));
  sm_exec(sm, I_MOV(R_Y, R_ISR));
  sm_exec(sm, I_MOV(R_ISR, R_NULL));
}

/* The three dividers, restarted together so that their ticks coincide */

static void sdio_setdiv(struct rp23xx_sdio_s *priv, uint32_t div)
{
  int sm;

  priv->div = div;
  for (sm = 0; sm < 3; sm++)
    {
      putreg32(div << RP23XX_PIO_SM_CLKDIV_INT_SHIFT, PIO_CLKDIV(sm));
    }

  putreg32(SM_ALL << RP23XX_PIO_CTRL_CLKDIV_RESTART_SHIFT,
           REG_SET(PIO_CTRL));
}

/****************************************************************************
 * Private Functions: CRCs
 ****************************************************************************/

/* CRC7 of a command or response (x^7 + x^3 + 1) */

static uint8_t sdio_crc7(const uint8_t *p, int n)
{
  uint8_t crc = 0;
  int i;
  int b;

  for (i = 0; i < n; i++)
    {
      for (b = 7; b >= 0; b--)
        {
          uint8_t in = ((p[i] >> b) & 1) ^ (crc >> 6);

          crc = (crc << 1) & 0x7f;
          if (in)
            {
              crc ^= 0x09;
            }
        }
    }

  return crc;
}

/* CRC16-CCITT of a byte string, MSB first (one data line) */

static uint16_t sdio_crc16(const uint8_t *p, size_t n)
{
  uint16_t crc = 0;

  while (n-- > 0)
    {
      uint16_t x = (crc >> 8) ^ *p++;

      x ^= x >> 4;
      crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }

  return crc;
}

/* The four data lines' CRC16s at once, over words in memory order (the
 * bus's nibble order once byte-swapped).  The result is the 16 CRC
 * nibbles as they go on the bus: hi first, most significant nibble first.
 */

static void sdio_crc16x4(const uint32_t *w, size_t nwords,
                         uint32_t *hi, uint32_t *lo)
{
  uint32_t h = 0;
  uint32_t l = 0;

  while (nwords-- > 0)
    {
      uint32_t x = h ^ __builtin_bswap32(*w++);

      x ^= x >> 16;
      h  = l ^ (x << 16) ^ (x >> 12);
      l  = x ^ (x << 20);
    }

  *hi = h;
  *lo = l;
}

/****************************************************************************
 * Private Functions: the PIO programs
 ****************************************************************************/

static void sdio_load_cmd(int var)
{
  uint16_t s = (var & VAR_RISE) ? SIDE(1) : SIDE(0);
  uint16_t ns = (var & VAR_RISE) ? SIDE(0) : SIDE(1);
  uint16_t prog[CMD_LEN];
  int i;

  /* Every instruction takes three ticks (one SD clock is two of them).
   * c6-c10 sample on side s: the falling or the rising edge.
   */

  prog[0]  = I_OUT(R_X, 8)              | SIDE(1) | DLY(2);  /* 47 */
  prog[1]  = I_SET(R_PINDIRS, 1)        | SIDE(0) | DLY(2);
  prog[2]  = I_OUT(R_Y, 8)              | SIDE(1) | DLY(2);  /* rx bits */
  prog[3]  = I_OUT(R_PINS, 1)           | SIDE(0) | DLY(2);  /* tx: */
  prog[4]  = I_JMP(C_XDEC, CMD_OFS + 3) | SIDE(1) | DLY(2);
  prog[5]  = I_SET(R_PINDIRS, 0)        | SIDE(0) | DLY(2);
  prog[6]  = I_JMP(C_PIN, CMD_OFS + 9)  | s       | DLY(2);  /* start? */
  prog[7]  = I_JMP(C_ALWAYS, CMD_OFS + 6) | ns    | DLY(2);
  prog[8]  = I_IN(R_PINS, 1)            | s       | DLY(2);
  prog[9]  = I_JMP(C_YDEC, CMD_OFS + 8) | ns      | DLY(2);
  prog[10] = I_PUSH                     | s       | DLY(2);
  prog[11] = I_MOV(R_X, R_STATUS)       | ns      | DLY(2);  /* idle: */
  prog[12] = I_JMP(C_NOTX, CMD_OFS)     | s       | DLY(2);

  for (i = 0; i < CMD_LEN; i++)
    {
      putreg32(prog[i], PIO_INSTR_MEM(CMD_OFS + i));
    }

  if (var & VAR_BYPASS)
    {
      putreg32(1 << VCMD, REG_SET(PIO_SYNCBYPASS));
    }
  else
    {
      putreg32(1 << VCMD, REG_CLR(PIO_SYNCBYPASS));
    }
}

static void sdio_load_rx(int k)
{
  uint16_t prog[RX_LEN];
  int i;

  /* Until the start bit, every clock is found again (wait gpio): a
   * command sent meanwhile shifts SM0's clock by half a period.  From the
   * start bit on, the block is counted out in step, six ticks a nibble.
   */

  prog[0] = I_MOV(R_X, R_Y);
  prog[1] = I_WAIT(0, S_GPIO, VCLK);
  prog[2] = I_WAIT(1, S_GPIO, VCLK) | DLY(k);
  prog[3] = I_JMP(C_PIN, RX_OFS + 1);                      /* start bit? */
  prog[4] = I_MOV(R_Y, R_Y) | DLY(4);                      /* (nop) */
  prog[5] = I_IN(R_PINS, 4) | DLY(4);
  prog[6] = I_JMP(C_XDEC, RX_OFS + 5);

  for (i = 0; i < RX_LEN; i++)
    {
      putreg32(prog[i], PIO_INSTR_MEM(RX_OFS + i));
    }
}

static void sdio_load_tx(int fall, int k, int stk)
{
  uint16_t prog[TX_LEN];
  int i;

  prog[0]  = I_PULL;                                       /* start word */
  prog[1]  = I_SET(R_PINDIRS, 15);
  prog[2]  = I_MOV(R_X, R_Y);
  prog[3]  = I_WAIT(fall ? 1 : 0, S_GPIO, VCLK);
  prog[4]  = I_WAIT(fall ? 0 : 1, S_GPIO, VCLK) | DLY(k);
  prog[5]  = I_OUT(R_PINS, 4) | DLY(2);
  prog[6]  = I_JMP(C_XDEC, TX_OFS + 5) | DLY(2);
  prog[7]  = I_SET(R_PINDIRS, 0);
  prog[8]  = I_SET(R_X, 15) | DLY(stk);
  prog[9]  = I_IN(R_PINS, 1) | DLY(4);                     /* status */
  prog[10] = I_JMP(C_XDEC, TX_OFS + 9);
  prog[11] = I_WAIT(1, S_PIN, 0);                          /* busy */

  for (i = 0; i < TX_LEN; i++)
    {
      putreg32(prog[i], PIO_INSTR_MEM(TX_OFS + i));
    }
}

/* The status token's sample delay: the same point in the clock as the
 * data in samples at
 */

static int sdio_stk(int rxk, int fall, int txk)
{
  return ((rxk - 3 * fall - txk - 2) % 6 + 12) % 6;
}

/* SM0 to idle (the clock running), whatever it was doing */

static void sdio_cmd_restart(struct rp23xx_sdio_s *priv)
{
  sm_reset(SM_CMD, CMD_IDLE);
  sm_exec(SM_CMD, I_SET(R_PINDIRS, 0) | SIDE(0));
  sm_exec(SM_CMD, I_SET(R_PINS, 1) | SIDE(0));
  sm_exec(SM_CMD, I_JMP(C_ALWAYS, CMD_IDLE) | SIDE(0));
  putreg32(SM_ALL << RP23XX_PIO_CTRL_CLKDIV_RESTART_SHIFT,
           REG_SET(PIO_CTRL));
  sm_enable(1 << SM_CMD);
}

/* Data lines released and both data machines stopped */

static void sdio_data_stop(struct rp23xx_sdio_s *priv)
{
  rp23xx_dmastop(priv->dmactrl);
  rp23xx_dmastop(priv->dmadata);
  rp23xx_dmastop(priv->dmastat);
  sm_disable((1 << SM_RX) | (1 << SM_TX));
  sm_exec(SM_TX, I_SET(R_PINDIRS, 0));
  sm_exec(SM_TX, I_SET(R_PINS, 15));
  sm_reset(SM_RX, RX_OFS);
  sm_reset(SM_TX, TX_OFS);
}

/* The sampling points for a divider, before tuning */

static void sdio_defaults(struct rp23xx_sdio_s *priv)
{
  if (priv->div == 1)
    {
      priv->cmdvar = VAR_RISE | VAR_BYPASS;
      priv->rxk    = 3;
      priv->txfall = 1;
      priv->txk    = 1;
    }
  else
    {
      priv->cmdvar = 0;
      priv->rxk    = 0;
      priv->txfall = 0;
      priv->txk    = 0;
    }

  priv->stk   = sdio_stk(priv->rxk, priv->txfall, priv->txk);
  priv->tuned = false;
}

/* Programs and divider in place; SM0 restarted (the clock stops briefly) */

static void sdio_apply(struct rp23xx_sdio_s *priv)
{
  sm_disable(SM_ALL);
  sdio_load_cmd(priv->cmdvar);
  sdio_load_rx(priv->rxk);
  sdio_load_tx(priv->txfall, priv->txk, priv->stk);
  sdio_setdiv(priv, priv->div);
  sm_reset(SM_RX, RX_OFS);
  sm_reset(SM_TX, TX_OFS);
  sdio_cmd_restart(priv);
}

/****************************************************************************
 * Private Functions: commands
 ****************************************************************************/

static bool sdio_is_datacmd(uint32_t cmd, bool app)
{
  uint32_t idx = cmd & MMCSD_CMDIDX_MASK;

  return !app && (idx == 17 || idx == 18 || idx == 24 || idx == 25);
}

static void sdio_cmd_send(struct rp23xx_sdio_s *priv, uint32_t cmd,
                          uint32_t arg)
{
  uint32_t idx = cmd & MMCSD_CMDIDX_MASK;
  uint8_t frame[5];
  uint32_t pc;
  int rxbits;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_NO_RESPONSE:
        rxbits = 0;
        break;

      case MMCSD_R2_RESPONSE:
        rxbits = 135;
        break;

      default:
        rxbits = 47;
        break;
    }

  /* SM0 should be idle; a command whose answer never came may still be
   * waiting.
   */

  pc = getreg32(PIO_ADDR(SM_CMD));
  if (pc < CMD_IDLE - 1 || pc > CMD_IDLE + 1 ||
      (getreg32(PIO_FSTAT) & FSTAT_TXEMPTY(SM_CMD)) == 0)
    {
      sdio_cmd_restart(priv);
    }

  while ((getreg32(PIO_FSTAT) & FSTAT_RXEMPTY(SM_CMD)) == 0)
    {
      getreg32(PIO_RXF(SM_CMD));
    }

  frame[0] = 0x40 | idx;
  frame[1] = arg >> 24;
  frame[2] = arg >> 16;
  frame[3] = arg >> 8;
  frame[4] = arg;

  priv->cmd    = cmd;
  priv->arg    = arg;
  priv->rxbits = rxbits;

  /* SM0 starts once both words are in (mov x, status: TX level < 2) */

  putreg32((47u << 24) | ((uint32_t)rxbits << 16) |
           ((uint32_t)frame[0] << 8) | frame[1], PIO_TXF(SM_CMD));
  putreg32(((uint32_t)frame[2] << 24) | ((uint32_t)frame[3] << 16) |
           ((uint32_t)frame[4] << 8) | (sdio_crc7(frame, 5) << 1) | 1,
           PIO_TXF(SM_CMD));
}

/* The response, or the command sent if it has none */

static int sdio_cmd_wait(struct rp23xx_sdio_s *priv)
{
  int nwords = priv->rxbits == 0 ? 0 : priv->rxbits == 47 ? 2 : 5;
  uint32_t clocks = 48 + 80 + priv->rxbits + 16;
  uint32_t limit = clocks * 6 * priv->div / SYS_MHZ + 500;
  uint32_t start = sdio_now();
  uint32_t pc;
  int i;

  /* (Each limit is looked at before the last check, so that a thread
   * preempted for longer than the limit does not miss what arrived.)
   */

  if (nwords == 0)
    {
      bool late;

      do
        {
          late = sdio_now() - start > limit;
          pc = getreg32(PIO_ADDR(SM_CMD));
          if (pc == CMD_RW0 || pc == CMD_RW1)
            {
              sdio_cmd_restart(priv);
              return OK;
            }
        }
      while (!late);

      sdio_cmd_restart(priv);
      return -ETIMEDOUT;
    }

  for (i = 0; i < nwords; i++)
    {
      for (; ; )
        {
          bool late = sdio_now() - start > limit;

          if ((getreg32(PIO_FSTAT) & FSTAT_RXEMPTY(SM_CMD)) == 0)
            {
              break;
            }

          if (late)
            {
              sdio_cmd_restart(priv);
              return -ETIMEDOUT;
            }
        }

      /* The CMD input is inverted */

      priv->resp[i] = ~getreg32(PIO_RXF(SM_CMD));
    }

  if (nwords == 2)
    {
      priv->resp[1] &= 0x7fff;
    }
  else
    {
      priv->resp[4] &= 0x7f;
    }

  /* The push done, SM0 goes back to idle within a few ticks */

  while (getreg32(PIO_ADDR(SM_CMD)) < CMD_IDLE &&
         sdio_now() - start <= limit)
    {
    }

  return OK;
}

/* A 48-bit response: its 32-bit argument, its index and CRC checked */

static int sdio_decode_short(struct rp23xx_sdio_s *priv, bool check,
                             uint32_t *value)
{
  uint64_t v = ((uint64_t)priv->resp[0] << 15) | priv->resp[1];
  uint8_t frame[5];

  *value = (uint32_t)(v >> 8);

  if ((v >> 46) & 1)
    {
      return -EIO;                                /* transmission bit */
    }

  if (!check)
    {
      return OK;
    }

  frame[0] = (v >> 40) & 0x7f;
  frame[1] = v >> 32;
  frame[2] = v >> 24;
  frame[3] = v >> 16;
  frame[4] = v >> 8;

  if ((v & 1) == 0 || sdio_crc7(frame, 5) != ((v >> 1) & 0x7f) ||
      (frame[0] & 0x3f) != (priv->cmd & MMCSD_CMDIDX_MASK))
    {
      return -EIO;
    }

  return OK;
}

/* Send a command and get its R1 (internal use: tuning and retries) */

static int sdio_cmd_r1(struct rp23xx_sdio_s *priv, uint32_t cmd,
                       uint32_t arg, uint32_t *r1)
{
  uint32_t dummy;
  int ret;

  sdio_cmd_send(priv, cmd, arg);
  ret = sdio_cmd_wait(priv);
  if (ret == OK && (cmd & MMCSD_RESPONSE_MASK) != MMCSD_NO_RESPONSE)
    {
      ret = sdio_decode_short(priv, true, r1 ? r1 : &dummy);
    }

  priv->app = false;
  return ret;
}

/* Wait until the card no longer holds DAT0 low */

static int sdio_wait_busy(uint32_t limit_us)
{
  uint32_t start = sdio_now();

  while (!sdio_d0())
    {
      if (sdio_now() - start > limit_us)
        {
          return -ETIMEDOUT;
        }

      if (sdio_now() - start > 200)
        {
          nxsched_usleep(1000);
        }
    }

  return OK;
}

/****************************************************************************
 * Private Functions: data transfers
 ****************************************************************************/

static void sdio_dma_done(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct rp23xx_sdio_s *priv = arg;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->xfer = status == 0 ? XFER_DONE : XFER_ERROR;
  if ((priv->waitevents & SDIOWAIT_TRANSFERDONE) != 0)
    {
      priv->wkupevent |= SDIOWAIT_TRANSFERDONE;
      spin_unlock_irqrestore(&priv->lock, flags);
      nxsem_post(&priv->waitsem);
      return;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

/* Arm the data-in machine and its DMA for priv->nblocks blocks of
 * priv->blocklen bytes into buf (or g_bounce)
 */

static int sdio_rx_start(struct rp23xx_sdio_s *priv)
{
  uint32_t bl = priv->blocklen;
  uint32_t n = priv->nblocks;
  uint8_t *dst = priv->bounce ? (uint8_t *)g_bounce : priv->rxbuf;
  uint32_t ctrl;
  uint32_t i;

  if (sdio_wait_busy(BUSY_TIMEOUT_US) < 0)
    {
      return -EBUSY;
    }

  sm_reset(SM_RX, RX_OFS);

  ctrl = rp23xx_dma_ctrl_blk_ctrl(priv->dmactrl, RP23XX_DMA_SIZE_WORD,
                                  DREQ_RX(SM_RX),
                                  RP23XX_DMA_CTRL_TRIG_INCR_WRITE |
                                  RP23XX_DMA_CTRL_TRIG_BSWAP |
                                  RP23XX_DMA_CTRL_TRIG_HIGH_PRIORITY);

  if (priv->widebus)
    {
      /* Per block: the data, then its four CRC16s */

      sm_set_y(SM_RX, bl * 2 + 16 - 1);
      for (i = 0; i < n; i++)
        {
          g_cb[2 * i].ctrl_trig      = ctrl;
          g_cb[2 * i].read_addr      = PIO_RXF(SM_RX);
          g_cb[2 * i].write_addr     = (uintptr_t)(dst + i * bl);
          g_cb[2 * i].xfer_count     = bl / 4;
          g_cb[2 * i + 1].ctrl_trig  = ctrl;
          g_cb[2 * i + 1].read_addr  = PIO_RXF(SM_RX);
          g_cb[2 * i + 1].write_addr = (uintptr_t)&g_rxcrc[2 * i];
          g_cb[2 * i + 1].xfer_count = 2;
        }
    }
  else
    {
      /* 1-bit: a nibble per bit (DAT0 in bit 0), data and CRC together */

      sm_set_y(SM_RX, bl * 8 + 16 - 1);
      g_cb[0].ctrl_trig  = ctrl;
      g_cb[0].read_addr  = PIO_RXF(SM_RX);
      g_cb[0].write_addr = (uintptr_t)g_bounce;
      g_cb[0].xfer_count = bl + 2;
      n = 1;
    }

  g_cb[2 * n].ctrl_trig  = RP23XX_DMA_CTRL_TRIG_IRQ_QUIET;
  g_cb[2 * n].read_addr  = 0;
  g_cb[2 * n].write_addr = 0;
  g_cb[2 * n].xfer_count = 0;
  if (!priv->widebus)
    {
      g_cb[1] = g_cb[2];
    }

  priv->xfer = XFER_BUSY;
  rp23xx_ctrl_dmasetup(priv->dmactrl, priv->dmadata, g_cb,
                       sdio_dma_done, priv);
  rp23xx_dmastart(priv->dmactrl, NULL, NULL);
  sm_enable(1 << SM_RX);
  return OK;
}

/* Received blocks against their CRCs; bounced data to the caller */

static int sdio_rx_check(struct rp23xx_sdio_s *priv)
{
  uint32_t bl = priv->blocklen;
  uint32_t i;

  if (priv->widebus)
    {
      const uint8_t *src = priv->bounce ? (const uint8_t *)g_bounce :
                                          priv->rxbuf;

      for (i = 0; i < priv->nblocks; i++)
        {
          uint32_t hi;
          uint32_t lo;

          sdio_crc16x4((const uint32_t *)(src + i * bl), bl / 4, &hi, &lo);
          if (hi != __builtin_bswap32(g_rxcrc[2 * i]) ||
              lo != __builtin_bswap32(g_rxcrc[2 * i + 1]))
            {
              return -EIO;
            }
        }

      if (priv->bounce)
        {
          memcpy(priv->rxbuf, g_bounce, priv->nbytes);
        }
    }
  else
    {
      /* Byte k of the stream holds nibbles 2k (high) and 2k + 1 (low) */

      const uint8_t *s = (const uint8_t *)g_bounce;
      uint8_t tmp[64];
      uint16_t crc = 0;
      uint32_t bit;

      if (bl > sizeof(tmp))
        {
          return -EIO;
        }

      memset(tmp, 0, bl);
      for (bit = 0; bit < bl * 8 + 16; bit++)
        {
          uint8_t nib = (s[bit >> 1] >> ((bit & 1) ? 0 : 4)) & 1;

          if (bit < bl * 8)
            {
              tmp[bit >> 3] |= nib << (7 - (bit & 7));
            }
          else
            {
              crc = (crc << 1) | nib;
            }
        }

      if (crc != sdio_crc16(tmp, bl))
        {
          return -EIO;
        }

      memcpy(priv->rxbuf, tmp, priv->nbytes);
    }

  return OK;
}

/* Arm the data-out machine and its DMA: the CRCs computed first */

static int sdio_tx_start(struct rp23xx_sdio_s *priv)
{
  uint32_t bl = priv->blocklen;
  uint32_t n = priv->nblocks;
  const uint8_t *src = priv->bounce ? (const uint8_t *)g_bounce :
                                      priv->txbuf;
  dma_config_t cfg;
  uint32_t ctrl;
  uint32_t i;

  for (i = 0; i < n; i++)
    {
      uint32_t hi;
      uint32_t lo;

      sdio_crc16x4((const uint32_t *)(src + i * bl), bl / 4, &hi, &lo);
      g_txtail[4 * i]     = __builtin_bswap32(hi);
      g_txtail[4 * i + 1] = __builtin_bswap32(lo);
      g_txtail[4 * i + 2] = END_WORD;
      g_txtail[4 * i + 3] = __builtin_bswap32(START_WORD);
    }

  ctrl = rp23xx_dma_ctrl_blk_ctrl(priv->dmactrl, RP23XX_DMA_SIZE_WORD,
                                  DREQ_TX(SM_TX),
                                  RP23XX_DMA_CTRL_TRIG_INCR_READ |
                                  RP23XX_DMA_CTRL_TRIG_BSWAP |
                                  RP23XX_DMA_CTRL_TRIG_HIGH_PRIORITY);

  for (i = 0; i < n; i++)
    {
      g_cb[2 * i].ctrl_trig      = ctrl;
      g_cb[2 * i].read_addr      = (uintptr_t)(src + i * bl);
      g_cb[2 * i].write_addr     = PIO_TXF(SM_TX);
      g_cb[2 * i].xfer_count     = bl / 4;
      g_cb[2 * i + 1].ctrl_trig  = ctrl;
      g_cb[2 * i + 1].read_addr  = (uintptr_t)&g_txtail[4 * i];
      g_cb[2 * i + 1].write_addr = PIO_TXF(SM_TX);
      g_cb[2 * i + 1].xfer_count = i == n - 1 ? 3 : 4;
    }

  g_cb[2 * n].ctrl_trig  = RP23XX_DMA_CTRL_TRIG_IRQ_QUIET;
  g_cb[2 * n].read_addr  = 0;
  g_cb[2 * n].write_addr = 0;
  g_cb[2 * n].xfer_count = 0;

  sm_reset(SM_TX, TX_OFS);
  sm_set_y(SM_TX, 8 + bl * 2 + 16 + 1 - 1);

  /* The status words: one per block, its end the transfer's */

  memset(g_txstat, 0xff, n * sizeof(uint32_t));
  cfg.dreq   = DREQ_RX(SM_TX);
  cfg.size   = RP23XX_DMA_SIZE_WORD;
  cfg.noincr = 0;
  rp23xx_rxdmasetup(priv->dmastat, PIO_RXF(SM_TX), (uintptr_t)g_txstat,
                    n * sizeof(uint32_t), cfg);

  priv->xfer = XFER_BUSY;
  rp23xx_dmastart(priv->dmastat, sdio_dma_done, priv);

  /* The first start word from here, the rest by DMA; the FIFO filled
   * before the machine starts.
   */

  putreg32(START_WORD, PIO_TXF(SM_TX));
  rp23xx_ctrl_dmasetup(priv->dmactrl, priv->dmadata, g_cb, NULL, NULL);
  rp23xx_dmastart(priv->dmactrl, NULL, NULL);
  sm_enable(1 << SM_TX);
  return OK;
}

/* Each block's CRC status token: 0 010 1 is accepted */

static int sdio_tx_check(struct rp23xx_sdio_s *priv)
{
  uint32_t i;

  for (i = 0; i < priv->nblocks; i++)
    {
      uint32_t s = g_txstat[i] & 0xffff;
      int p;

      for (p = 15; p >= 4 && (s & (1 << p)) != 0; p--)
        {
        }

      if (p < 4 || ((s >> (p - 3)) & 7) != 2)
        {
          syslog(LOG_WARNING, "sdio: write block %" PRIu32 "/%u status "
                 "%04" PRIx32 "\n", i, priv->nblocks, s);
          return -EIO;
        }
    }

  return OK;
}

/* Start the transfer set up by recvsetup/sendsetup again from the start:
 * the card stopped, the command sent again.
 */

static int sdio_restart_xfer(struct rp23xx_sdio_s *priv)
{
  uint32_t idx = priv->datacmd & MMCSD_CMDIDX_MASK;
  uint32_t r1;
  int ret;

  sdio_data_stop(priv);

  if (idx == 18 || idx == 25)
    {
      sdio_cmd_r1(priv, MMCSD_CMD12, 0, NULL);
    }

  if (sdio_wait_busy(BUSY_TIMEOUT_US) < 0)
    {
      return -ETIMEDOUT;
    }

  if (priv->dir == DIR_RX)
    {
      ret = sdio_rx_start(priv);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (priv->datacmd23 != 0)
    {
      ret = sdio_cmd_r1(priv, MMCSD_CMD23, priv->datacmd23, &r1);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = sdio_cmd_r1(priv, priv->datacmd, priv->dataarg, &r1);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->dir == DIR_TX)
    {
      ret = sdio_tx_start(priv);
    }

  return ret;
}

/****************************************************************************
 * Private Functions: tuning
 ****************************************************************************/

/* How many of n CMD13s come back right */

static int sdio_test_cmd(struct rp23xx_sdio_s *priv, int n)
{
  int good = 0;
  uint32_t r1;
  int i;

  for (i = 0; i < n; i++)
    {
      if (sdio_cmd_r1(priv, MMCSD_CMD13, (uint32_t)priv->rca << 16,
                      &r1) == OK &&
          ((r1 >> 9) & 15) == 4)                     /* tran */
        {
          good++;
        }
    }

  return good;
}

/* How many of n CMD6 queries (64 bytes of switch status) come in right */

static int sdio_test_rx(struct rp23xx_sdio_s *priv, int n)
{
  uint16_t savebl = priv->blocklen;
  uint16_t saven = priv->nblocks;
  bool savebounce = priv->bounce;
  uint8_t *saverx = priv->rxbuf;
  size_t savebytes = priv->nbytes;
  uint8_t savedir = priv->dir;
  uint8_t buf[64];
  int good = 0;
  int i;

  priv->blocklen = 64;
  priv->nblocks  = 1;
  priv->bounce   = true;
  priv->rxbuf    = buf;
  priv->nbytes   = 64;
  priv->dir      = DIR_RX;

  for (i = 0; i < n; i++)
    {
      uint32_t start;
      uint32_t r1;

      if (sdio_rx_start(priv) < 0)
        {
          break;
        }

      if (sdio_cmd_r1(priv, SD_CMD6, 0x00fffff1, &r1) == OK)
        {
          start = sdio_now();
          while (priv->xfer == XFER_BUSY && sdio_now() - start < 20000)
            {
            }

          if (priv->xfer == XFER_DONE && sdio_rx_check(priv) == OK)
            {
              good++;
            }
        }

      sdio_data_stop(priv);
    }

  priv->blocklen = savebl;
  priv->nblocks  = saven;
  priv->bounce   = savebounce;
  priv->rxbuf    = saverx;
  priv->nbytes   = savebytes;
  priv->dir      = savedir;
  priv->xfer     = XFER_IDLE;
  nxsem_reset(&priv->waitsem, 0);
  return good;
}

/* The middle of the longest run of passes, going round; dflt if all pass,
 * -1 if none does
 */

static int sdio_middle(const int *pass, int n, int dflt)
{
  int best = -1;
  int bestlen = 0;
  int s;

  for (s = 0; s < n && pass[s] != 0; s++)
    {
    }

  if (s == n)
    {
      return dflt;
    }

  for (s = 0; s < n; s++)
    {
      int len = 0;

      if (pass[s] == 0 || pass[(s + n - 1) % n] != 0)
        {
          continue;                                  /* not a run's start */
        }

      while (len < n && pass[(s + len) % n] != 0)
        {
          len++;
        }

      if (len > bestlen)
        {
          bestlen = len;
          best = (s + len / 2) % n;
        }
    }

  return best;
}

/* Tune the sampling points at priv->div.  Needs the card selected. */

static int sdio_tune(struct rp23xx_sdio_s *priv)
{
  static const uint8_t vars[4] =
  {
    0, VAR_BYPASS, VAR_RISE, VAR_RISE | VAR_BYPASS
  };

  uint16_t savebl = priv->blocklen;
  uint16_t saven = priv->nblocks;
  int phase[4];
  int order[4];
  int pass[6];
  char cl[5];
  char rl[7];
  int i;
  int j;
  int m;

  if (!priv->rcavalid || !priv->widebus)
    {
      return -EINVAL;
    }

  /* Command sampling: the four points, ordered by when they sample */

  for (i = 0; i < 4; i++)
    {
      int p = (vars[i] & VAR_RISE) ? 3 * priv->div : 6 * priv->div;

      phase[i] = (p - ((vars[i] & VAR_BYPASS) ? 0 : 2) +
                  6 * priv->div) % (6 * priv->div);
      order[i] = i;
    }

  for (i = 0; i < 4; i++)
    {
      for (j = i + 1; j < 4; j++)
        {
          if (phase[order[j]] < phase[order[i]])
            {
              int t = order[i];
              order[i] = order[j];
              order[j] = t;
            }
        }
    }

  for (i = 0; i < 4; i++)
    {
      sm_disable(1 << SM_CMD);
      sdio_load_cmd(vars[order[i]]);
      sdio_cmd_restart(priv);
      pass[i] = sdio_test_cmd(priv, 8) == 8;
      cl[i] = pass[i] ? '+' : '.';
    }

  cl[4] = '\0';
  sdio_defaults(priv);
  for (i = 0; i < 4 && vars[order[i]] != priv->cmdvar; i++)
    {
    }

  m = sdio_middle(pass, 4, i);
  if (m < 0)
    {
      syslog(LOG_WARNING, "sdio: %u kHz: no command sampling point "
             "works (%s)\n", KHZ(priv->div), cl);
      sdio_apply(priv);
      return -EIO;
    }

  priv->cmdvar = vars[order[m]];
  sm_disable(1 << SM_CMD);
  sdio_load_cmd(priv->cmdvar);
  sdio_cmd_restart(priv);

  /* Data in: the six delays of a clock */

  for (i = 0; i < 6; i++)
    {
      sdio_load_rx(i);
      pass[i] = sdio_test_rx(priv, 4) == 4;
      rl[i] = pass[i] ? '+' : '.';
    }

  rl[6] = '\0';
  priv->blocklen = savebl;
  priv->nblocks  = saven;
  m = sdio_middle(pass, 6, priv->rxk);
  if (m < 0)
    {
      syslog(LOG_WARNING, "sdio: %u kHz: no data sampling point works "
             "(cmd %s data %s)\n", KHZ(priv->div),
             cl, rl);
      sdio_load_rx(priv->rxk);
      return -EIO;
    }

  priv->rxk = m;
  priv->stk = sdio_stk(priv->rxk, priv->txfall, priv->txk);
  sdio_load_rx(priv->rxk);
  sdio_load_tx(priv->txfall, priv->txk, priv->stk);
  priv->tuned = true;

  syslog(LOG_INFO, "sdio: %u kHz: cmd %s -> %d, data %s -> %d, "
         "out %s+%d, status +%d\n", KHZ(priv->div),
         cl, priv->cmdvar, rl, priv->rxk, priv->txfall ? "fall" : "rise",
         priv->txk, priv->stk);
  return OK;
}

/* Set a divider (no faster than the floor) and tune there; slower until
 * it works.
 */

static void sdio_setspeed(struct rp23xx_sdio_s *priv, uint32_t div,
                          bool tune)
{
  if (div < priv->floor)
    {
      div = priv->floor;
    }

  for (; ; )
    {
      priv->div = div;
      sdio_defaults(priv);
      sdio_apply(priv);

      if (!tune || sdio_tune(priv) == OK || div >= DIV_SLOWEST)
        {
          break;
        }

      div++;
      priv->floor = div;
      syslog(LOG_WARNING, "sdio: down to %u kHz\n",
             KHZ(div));
    }
}

/****************************************************************************
 * Private Functions: events
 ****************************************************************************/

static void sdio_endwait(struct rp23xx_sdio_s *priv, sdio_eventset_t ev)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->wkupevent |= ev;
  spin_unlock_irqrestore(&priv->lock, flags);
  nxsem_post(&priv->waitsem);
}

static void sdio_timeout(wdparm_t arg)
{
  sdio_endwait((struct rp23xx_sdio_s *)arg, SDIOWAIT_TIMEOUT);
}

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
static int sdio_d0_interrupt(int irq, void *context, void *arg)
{
  struct rp23xx_sdio_s *priv = arg;

  rp23xx_gpio_disable_irq(D0_GPIO);
  if ((priv->waitevents & SDIOWAIT_WRCOMPLETE) != 0)
    {
      sdio_endwait(priv, SDIOWAIT_WRCOMPLETE);
    }

  return OK;
}
#endif

static void sdio_callback(struct rp23xx_sdio_s *priv)
{
  if (priv->callback == NULL)
    {
      return;
    }

  if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
    {
      if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
        {
          return;
        }
    }
  else if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
    {
      return;
    }

  priv->cbevents = 0;
  if (up_interrupt_context())
    {
      work_queue(HPWORK, &priv->cbwork, priv->callback, priv->cbarg, 0);
    }
  else
    {
      priv->callback(priv->cbarg);
    }
}

/****************************************************************************
 * Private Functions: the SDIO interface
 ****************************************************************************/

static void rp23xx_reset(struct sdio_dev_s *dev)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;

  sdio_data_stop(priv);
  priv->widebus  = false;
  priv->rcavalid = false;
  priv->app      = false;
  priv->cmd23    = 0;
  priv->dir      = DIR_NONE;
  priv->xfer     = XFER_IDLE;
  priv->nominal  = DIV_ID;
  priv->div      = DIV_ID;
  sdio_defaults(priv);
  sdio_apply(priv);
}

static sdio_capset_t rp23xx_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = SDIO_CAPS_4BIT | SDIO_CAPS_DMASUPPORTED;

#ifdef CONFIG_RP23XX_SDIO_HIGHSPEED
  caps |= SDIO_CAPS_SD_HS_MODE;
#endif

  return caps;
}

static sdio_statset_t rp23xx_status(struct sdio_dev_s *dev)
{
  return ((struct rp23xx_sdio_s *)dev)->cdstatus;
}

static void rp23xx_widebus(struct sdio_dev_s *dev, bool enable)
{
  ((struct rp23xx_sdio_s *)dev)->widebus = enable;
}

static void rp23xx_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  uint32_t div;
  bool tune = false;

  switch (rate)
    {
      case CLOCK_SDIO_DISABLED:
        sm_disable(SM_ALL);
        return;

      case CLOCK_IDMODE:
        div = DIV_ID;
        break;

      case CLOCK_MMC_TRANSFER:
      case CLOCK_SD_TRANSFER_1BIT:
        div = DIV_NARROW;
        break;

      case CLOCK_SD_TRANSFER_4BIT_HS:
        div = DIV_HS;
        tune = true;
        break;

      case CLOCK_SD_TRANSFER_4BIT:
      case CLOCK_MMC_TRANSFER_4BIT:
      default:
        div = DIV_DS;
        tune = true;
        break;
    }

  priv->nominal = div;
  sdio_setspeed(priv, div, tune && priv->widebus);
}

static int rp23xx_attach(struct sdio_dev_s *dev)
{
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  rp23xx_gpio_irq_attach(D0_GPIO, RP23XX_GPIO_INTR_LEVEL_HIGH,
                         sdio_d0_interrupt, dev);
#endif
  return OK;
}

static int rp23xx_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t arg)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  uint32_t idx = cmd & MMCSD_CMDIDX_MASK;
  bool app = priv->app;

  /* What a retry needs: the data command, its argument and its CMD23 */

  if (sdio_is_datacmd(cmd, app))
    {
      priv->datacmd   = cmd;
      priv->dataarg   = arg;
      priv->datacmd23 = priv->cmd23;
      priv->attempt   = 1;
    }
  else if ((cmd & MMCSD_DATAXFR_MASK) != 0)
    {
      priv->datacmd = 0;                         /* ACMD51, CMD6: no retry */
    }

  priv->cmd23 = (!app && idx == 23) ? arg : 0;
  priv->app = (idx == 55);

  if (idx == 7 && (arg >> 16) != 0)
    {
      priv->rca = arg >> 16;
      priv->rcavalid = true;
    }

  /* Read only: a write command is not sent, and fails at its response */

  priv->refused = priv->nowrite && !app && (idx == 24 || idx == 25);
  if (priv->refused)
    {
      return OK;
    }

  sdio_cmd_send(priv, cmd, arg);
  return OK;
}

static void rp23xx_blocksetup(struct sdio_dev_s *dev,
                              unsigned int blocklen, unsigned int nblocks)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;

  priv->blocklen = blocklen;
  priv->nblocks  = nblocks;
}

/* A read into buffer: straight there if the DMA can, else bounced */

static int sdio_recv(struct rp23xx_sdio_s *priv, uint8_t *buffer,
                     size_t nbytes)
{
  uint32_t bl = priv->blocklen;

  if (bl == 0 || (bl & 3) != 0 || priv->nblocks == 0 ||
      priv->nblocks > MAXBLOCKS || nbytes != bl * priv->nblocks)
    {
      return -EINVAL;
    }

  priv->bounce = !priv->widebus || ((uintptr_t)buffer & 3) != 0;
  if (priv->bounce &&
      (priv->widebus ? nbytes > BOUNCE_BYTES - 8 :
                       priv->nblocks != 1 || bl > 64))
    {
      return -EFAULT;
    }

  priv->rxbuf  = buffer;
  priv->nbytes = nbytes;
  priv->dir    = DIR_RX;
  if (priv->waitevents != 0)
    {
      sdio_restart_wd(priv);
    }

  return sdio_rx_start(priv);
}

static int rp23xx_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                            size_t nbytes)
{
  return sdio_recv((struct rp23xx_sdio_s *)dev, buffer, nbytes);
}

static int rp23xx_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                               size_t buflen)
{
  return sdio_recv((struct rp23xx_sdio_s *)dev, buffer, buflen);
}

static int sdio_send(struct rp23xx_sdio_s *priv, const uint8_t *buffer,
                     size_t nbytes)
{
  uint32_t bl = priv->blocklen;

  if (!priv->widebus || bl == 0 || (bl & 3) != 0 || priv->nblocks == 0 ||
      priv->nblocks > MAXBLOCKS || nbytes != bl * priv->nblocks)
    {
      return -EINVAL;
    }

  priv->bounce = ((uintptr_t)buffer & 3) != 0;
  if (priv->bounce)
    {
      if (nbytes > BOUNCE_BYTES - 8)
        {
          return -EFAULT;
        }

      memcpy(g_bounce, buffer, nbytes);
    }

  priv->txbuf  = buffer;
  priv->nbytes = nbytes;
  priv->dir    = DIR_TX;
  return sdio_tx_start(priv);
}

static int rp23xx_sendsetup(struct sdio_dev_s *dev, const uint8_t *buffer,
                            size_t nbytes)
{
  return sdio_send((struct rp23xx_sdio_s *)dev, buffer, nbytes);
}

static int rp23xx_dmasendsetup(struct sdio_dev_s *dev,
                               const uint8_t *buffer, size_t buflen)
{
  return sdio_send((struct rp23xx_sdio_s *)dev, buffer, buflen);
}

static int rp23xx_dmapreflight(struct sdio_dev_s *dev,
                               const uint8_t *buffer, size_t buflen)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;

  /* Word-aligned buffers go straight to the DMA; for others the FAT file
   * system tries again through its own sector buffer
   * (CONFIG_FAT_DIRECT_RETRY), which this driver bounces if it must.
   */

  if (((uintptr_t)buffer & 3) != 0 && buflen > BOUNCE_BYTES - 8)
    {
      return -EFAULT;
    }

  return priv->widebus || buflen <= 64 ? OK : -EFAULT;
}

static int rp23xx_cancel(struct sdio_dev_s *dev)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  irqstate_t flags;

  wd_cancel(&priv->waitwdog);
  sdio_data_stop(priv);

  flags = spin_lock_irqsave(&priv->lock);
  priv->dir        = DIR_NONE;
  priv->xfer       = XFER_IDLE;
  priv->waitevents = 0;
  priv->wkupevent  = 0;
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

static int rp23xx_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;

  if (priv->refused)
    {
      priv->refused = false;
      priv->respret = -EROFS;
      return -EROFS;
    }

  priv->respret = sdio_cmd_wait(priv);
  if (priv->respret == -ETIMEDOUT)
    {
      priv->timeouts++;
    }

  return priv->respret;
}

static int rp23xx_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                               uint32_t *rshort)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  uint32_t value = 0;
  int ret = priv->respret;

  if (ret == OK)
    {
      ret = sdio_decode_short(priv, true, &value);
      if (ret == OK && (cmd & MMCSD_RESPONSE_MASK) == MMCSD_R6_RESPONSE)
        {
          priv->rca = value >> 16;
          priv->rcavalid = true;
        }
    }

  if (rshort)
    {
      *rshort = value;
    }

  return ret;
}

static int rp23xx_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t *rshort)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  uint32_t value = 0;
  int ret = priv->respret;

  /* R3 and R4 carry no index and no CRC */

  if (ret == OK)
    {
      ret = sdio_decode_short(priv, false, &value);
    }

  if (rshort)
    {
      *rshort = value;
    }

  return ret;
}

static int rp23xx_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t rlong[4])
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  uint32_t *w = priv->resp;
  uint32_t r[4];
  uint8_t bytes[15];
  int ret = priv->respret;
  int i;

  if (ret != OK)
    {
      return ret;
    }

  /* 135 bits after the start bit: transmission bit, six reserved bits and
   * the 128 bits of CID or CSD (CRC7 and end bit included)
   */

  r[0] = ((w[0] & 0x1ffffff) << 7) | (w[1] >> 25);
  r[1] = (w[1] << 7) | (w[2] >> 25);
  r[2] = (w[2] << 7) | (w[3] >> 25);
  r[3] = (w[3] << 7) | (w[4] & 0x7f);

  for (i = 0; i < 15; i++)
    {
      bytes[i] = r[i / 4] >> (24 - 8 * (i % 4));
    }

  if ((w[0] >> 31) != 0 || (r[3] & 1) == 0 ||
      sdio_crc7(bytes, 15) != ((r[3] >> 1) & 0x7f))
    {
      ret = -EIO;
    }

  if (rlong)
    {
      memcpy(rlong, r, sizeof(r));
    }

  return ret;
}

static void rp23xx_waitenable(struct sdio_dev_s *dev,
                              sdio_eventset_t eventset, uint32_t timeout)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  irqstate_t flags;

  wd_cancel(&priv->waitwdog);
  nxsem_reset(&priv->waitsem, 0);

  flags = spin_lock_irqsave(&priv->lock);
  priv->waitevents = eventset;
  priv->wkupevent  = 0;
  priv->waitms     = timeout;

  /* A transfer that finished before it was waited for */

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0 &&
      priv->xfer != XFER_IDLE && priv->xfer != XFER_BUSY)
    {
      priv->wkupevent = SDIOWAIT_TRANSFERDONE;
      nxsem_post(&priv->waitsem);
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if ((eventset & SDIOWAIT_TIMEOUT) != 0)
    {
      if (timeout == 0)
        {
          sdio_endwait(priv, SDIOWAIT_TIMEOUT);
        }
      else
        {
          wd_start(&priv->waitwdog, MSEC2TICK(timeout) + 1, sdio_timeout,
                   (wdparm_t)priv);
        }
    }
}

/* The data timeout: mmcsd's, but for a read no more than the card may
 * take (100 ms to the first block) and the blocks themselves, so that a
 * missed start bit costs a retry, not seconds.
 */

static void sdio_restart_wd(struct rp23xx_sdio_s *priv)
{
  uint32_t ms = priv->waitms;

  wd_cancel(&priv->waitwdog);
  if (priv->dir == DIR_RX && ms > 250 + 2 * priv->nblocks)
    {
      ms = 250 + 2 * priv->nblocks;
    }

  if (ms != 0 && (priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      wd_start(&priv->waitwdog, MSEC2TICK(ms) + 1, sdio_timeout,
               (wdparm_t)priv);
    }
}

/* A finished transfer checked; a failed one started again (-EAGAIN) */

static int sdio_finish(struct rp23xx_sdio_s *priv, sdio_eventset_t ev)
{
  int ret;

  if ((ev & SDIOWAIT_TRANSFERDONE) != 0 && priv->xfer == XFER_DONE)
    {
      if (priv->dir == DIR_RX)
        {
          sm_disable(1 << SM_RX);
          ret = sdio_rx_check(priv);
          if (ret < 0)
            {
              priv->crcerrors++;
            }
        }
      else
        {
          ret = sdio_tx_check(priv);
          if (ret < 0)
            {
              priv->crcerrors++;
            }
        }

      if (ret == OK)
        {
          return OK;
        }
    }
  else
    {
      priv->timeouts++;
      ret = -ETIMEDOUT;
    }

  /* Again; the third attempt one clock step slower */

  if (priv->datacmd == 0 || priv->attempt >= RETRIES)
    {
      syslog(LOG_ERR, "sdio: CMD%" PRIu32 " %08" PRIx32 " x%u failed: "
             "%d\n", priv->datacmd & MMCSD_CMDIDX_MASK, priv->dataarg,
             priv->nblocks, ret);
      return ret;
    }

  priv->attempt++;
  priv->retries++;
  syslog(LOG_WARNING, "sdio: CMD%" PRIu32 " %08" PRIx32 " x%u: %s, "
         "again\n", priv->datacmd & MMCSD_CMDIDX_MASK, priv->dataarg,
         priv->nblocks, ret == -ETIMEDOUT ? "timeout" : "CRC");

  if (priv->attempt == RETRIES && priv->div < DIV_SLOWEST)
    {
      uint8_t dir = priv->dir;

      sdio_data_stop(priv);
      if ((priv->datacmd & MMCSD_CMDIDX_MASK) == 18 ||
          (priv->datacmd & MMCSD_CMDIDX_MASK) == 25)
        {
          sdio_cmd_r1(priv, MMCSD_CMD12, 0, NULL);
        }

      sdio_wait_busy(BUSY_TIMEOUT_US);
      priv->floor = priv->div + 1;
      syslog(LOG_WARNING, "sdio: down to %u kHz\n",
             KHZ(priv->floor));
      sdio_setspeed(priv, priv->floor, true);
      priv->dir = dir;
      sdio_restart_wd(priv);
    }

  ret = sdio_restart_xfer(priv);
  return ret < 0 ? ret : -EAGAIN;
}

static sdio_eventset_t rp23xx_eventwait(struct sdio_dev_s *dev)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  sdio_eventset_t ev = 0;
  irqstate_t flags;
  int ret;

  for (; ; )
    {
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
      if ((priv->waitevents & SDIOWAIT_WRCOMPLETE) != 0)
        {
          if (sdio_d0())
            {
              ev = SDIOWAIT_WRCOMPLETE;
              break;
            }

          rp23xx_gpio_enable_irq(D0_GPIO);
        }
#endif

      if (priv->waitevents == 0)
        {
          ev = SDIOWAIT_ERROR;
          break;
        }

      ret = nxsem_wait_uninterruptible(&priv->waitsem);
      if (ret < 0)
        {
          ev = SDIOWAIT_ERROR;
          break;
        }

      flags = spin_lock_irqsave(&priv->lock);
      ev = priv->wkupevent;
      spin_unlock_irqrestore(&priv->lock, flags);

      if (ev == 0)
        {
          continue;
        }

      if (priv->dir != DIR_NONE &&
          (ev & (SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT)) != 0)
        {
          ret = sdio_finish(priv, ev);
          if (ret == -EAGAIN)
            {
              /* Started again: wait for it as for the first */

              flags = spin_lock_irqsave(&priv->lock);
              priv->wkupevent = 0;
              spin_unlock_irqrestore(&priv->lock, flags);
              sdio_restart_wd(priv);
              continue;
            }

          if (ret < 0)
            {
              sdio_data_stop(priv);
              ev = (ret == -ETIMEDOUT) ? SDIOWAIT_TIMEOUT : SDIOWAIT_ERROR;
            }
          else
            {
              ev = SDIOWAIT_TRANSFERDONE;
              if (priv->dir == DIR_TX)
                {
                  sm_disable(1 << SM_TX);
                }
            }

          priv->dir  = DIR_NONE;
          priv->xfer = XFER_IDLE;
        }

      break;
    }

  wd_cancel(&priv->waitwdog);
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  rp23xx_gpio_disable_irq(D0_GPIO);
#endif

  flags = spin_lock_irqsave(&priv->lock);
  priv->waitevents = 0;
  priv->wkupevent  = 0;
  spin_unlock_irqrestore(&priv->lock, flags);
  return ev;
}

static void rp23xx_callbackenable(struct sdio_dev_s *dev,
                                  sdio_eventset_t eventset)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;

  priv->cbevents = eventset;
  sdio_callback(priv);
}

static int rp23xx_registercallback(struct sdio_dev_s *dev,
                                   worker_t callback, void *arg)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct sdio_dev_s *rp23xx_sdio_initialize(int slotno)
{
  struct rp23xx_sdio_s *priv = &g_sdio;
  static const int pins[6] =
  {
    CLK_GPIO, CMD_GPIO, D0_GPIO, D0_GPIO + 1, D0_GPIO + 2, D0_GPIO + 3
  };

  int i;

  if (priv->dmadata != NULL)
    {
      return &priv->dev;
    }

  nxmutex_init(&priv->dev.mutex);
  spin_lock_init(&priv->lock);
  priv->floor = 1;

  /* DMA: the control channel's own interrupts are not wanted (it finishes
   * a control block every few words)
   */

  priv->dmactrl = rp23xx_dmachannel();
  priv->dmadata = rp23xx_dmachannel();
  priv->dmastat = rp23xx_dmachannel();
  putreg32(1 << ((rp23xx_dma_register(priv->dmactrl, 0) -
                  RP23XX_DMA_BASE) / 0x40),
           REG_CLR(RP23XX_DMA_INTE0));

  /* The PIO block: all its state machines but SM3 */

  putreg32(SDIO_GPIOBASE, PIO_GPIOBASE);
  sm_disable(SM_ALL);

  putreg32(RP23XX_PIO_SM_SHIFTCTRL_AUTOPULL |
           RP23XX_PIO_SM_SHIFTCTRL_AUTOPUSH, PIO_SHIFTCTRL(SM_CMD));
  putreg32((VCMD << RP23XX_PIO_SM_EXECCTRL_JMP_PIN_SHIFT) |
           ((CMD_OFS + 12) << RP23XX_PIO_SM_EXECCTRL_WRAP_TOP_SHIFT) |
           (CMD_IDLE << RP23XX_PIO_SM_EXECCTRL_WRAP_BOTTOM_SHIFT) |
           (0 << RP23XX_PIO_SM_EXECCTRL_STATUS_SEL_SHIFT) | 2,
           PIO_EXECCTRL(SM_CMD));
  putreg32((1 << RP23XX_PIO_SM_PINCTRL_SIDESET_COUNT_SHIFT) |
           (1 << RP23XX_PIO_SM_PINCTRL_SET_COUNT_SHIFT) |
           (1 << RP23XX_PIO_SM_PINCTRL_OUT_COUNT_SHIFT) |
           (VCMD << RP23XX_PIO_SM_PINCTRL_IN_BASE_SHIFT) |
           (VCLK << RP23XX_PIO_SM_PINCTRL_SIDESET_BASE_SHIFT) |
           (VCMD << RP23XX_PIO_SM_PINCTRL_SET_BASE_SHIFT) |
           (VCMD << RP23XX_PIO_SM_PINCTRL_OUT_BASE_SHIFT),
           PIO_PINCTRL(SM_CMD));

  putreg32(RP23XX_PIO_SM_SHIFTCTRL_AUTOPUSH |
           RP23XX_PIO_SM_SHIFTCTRL_FJOIN_RX, PIO_SHIFTCTRL(SM_RX));
  putreg32((VD0 << RP23XX_PIO_SM_EXECCTRL_JMP_PIN_SHIFT) |
           ((RX_OFS + RX_LEN - 1) << RP23XX_PIO_SM_EXECCTRL_WRAP_TOP_SHIFT) |
           (RX_OFS << RP23XX_PIO_SM_EXECCTRL_WRAP_BOTTOM_SHIFT),
           PIO_EXECCTRL(SM_RX));
  putreg32(VD0 << RP23XX_PIO_SM_PINCTRL_IN_BASE_SHIFT, PIO_PINCTRL(SM_RX));

  putreg32(RP23XX_PIO_SM_SHIFTCTRL_AUTOPULL |
           RP23XX_PIO_SM_SHIFTCTRL_AUTOPUSH |
           (16 << RP23XX_PIO_SM_SHIFTCTRL_PUSH_THRESH_SHIFT),
           PIO_SHIFTCTRL(SM_TX));
  putreg32((VD0 << RP23XX_PIO_SM_EXECCTRL_JMP_PIN_SHIFT) |
           ((TX_OFS + TX_LEN - 1) << RP23XX_PIO_SM_EXECCTRL_WRAP_TOP_SHIFT) |
           (TX_OFS << RP23XX_PIO_SM_EXECCTRL_WRAP_BOTTOM_SHIFT),
           PIO_EXECCTRL(SM_TX));
  putreg32((4 << RP23XX_PIO_SM_PINCTRL_SET_COUNT_SHIFT) |
           (4 << RP23XX_PIO_SM_PINCTRL_OUT_COUNT_SHIFT) |
           (VD0 << RP23XX_PIO_SM_PINCTRL_IN_BASE_SHIFT) |
           (VD0 << RP23XX_PIO_SM_PINCTRL_SET_BASE_SHIFT) |
           (VD0 << RP23XX_PIO_SM_PINCTRL_OUT_BASE_SHIFT),
           PIO_PINCTRL(SM_TX));

  /* Pins: fast edges, pull-ups on the card's lines (DAT1 and DAT2 have
   * none on the board), the CMD input inverted for SM0
   */

  for (i = 0; i < 6; i++)
    {
      rp23xx_gpio_set_function(pins[i], RP23XX_GPIO_FUNC_PIO0 + SDIO_PIO);
      rp23xx_gpio_set_pulls(pins[i], i != 0, false);
      rp23xx_gpio_set_slew_fast(pins[i], true);
      rp23xx_gpio_set_drive_strength(pins[i],
                                     RP23XX_PADS_BANK0_GPIO_DRIVE_8MA);
    }

  modifyreg32(RP23XX_IO_BANK0_GPIO_CTRL(CMD_GPIO), 3 << 16, 1 << 16);

  /* The clock pin an output (driven low), CMD and DAT0-3 inputs with
   * their output latches high
   */

  putreg32((1 << RP23XX_PIO_SM_PINCTRL_SIDESET_COUNT_SHIFT) |
           (1 << RP23XX_PIO_SM_PINCTRL_SET_COUNT_SHIFT) |
           (VCLK << RP23XX_PIO_SM_PINCTRL_SET_BASE_SHIFT) |
           (VCLK << RP23XX_PIO_SM_PINCTRL_SIDESET_BASE_SHIFT),
           PIO_PINCTRL(SM_CMD) + 0);
  sm_exec(SM_CMD, I_SET(R_PINS, 0) | SIDE(0));
  sm_exec(SM_CMD, I_SET(R_PINDIRS, 1) | SIDE(0));
  putreg32((1 << RP23XX_PIO_SM_PINCTRL_SIDESET_COUNT_SHIFT) |
           (1 << RP23XX_PIO_SM_PINCTRL_SET_COUNT_SHIFT) |
           (1 << RP23XX_PIO_SM_PINCTRL_OUT_COUNT_SHIFT) |
           (VCMD << RP23XX_PIO_SM_PINCTRL_IN_BASE_SHIFT) |
           (VCLK << RP23XX_PIO_SM_PINCTRL_SIDESET_BASE_SHIFT) |
           (VCMD << RP23XX_PIO_SM_PINCTRL_SET_BASE_SHIFT) |
           (VCMD << RP23XX_PIO_SM_PINCTRL_OUT_BASE_SHIFT),
           PIO_PINCTRL(SM_CMD));
  sm_exec(SM_CMD, I_SET(R_PINS, 1) | SIDE(0));
  sm_exec(SM_CMD, I_SET(R_PINDIRS, 0) | SIDE(0));
  sm_exec(SM_TX, I_SET(R_PINS, 15));
  sm_exec(SM_TX, I_SET(R_PINDIRS, 0));

  /* Programs loaded, the clock running at the identification rate */

  rp23xx_reset(&priv->dev);

  syslog(LOG_INFO, "sdio: PIO%d, CLK %d CMD %d DAT0-3 %d-%d, DMA %d %d "
         "%d\n", SDIO_PIO, CLK_GPIO, CMD_GPIO, D0_GPIO, D0_GPIO + 3,
         (int)((rp23xx_dma_register(priv->dmactrl, 0) -
                RP23XX_DMA_BASE) / 0x40),
         (int)((rp23xx_dma_register(priv->dmadata, 0) -
                RP23XX_DMA_BASE) / 0x40),
         (int)((rp23xx_dma_register(priv->dmastat, 0) -
                RP23XX_DMA_BASE) / 0x40));
  return &priv->dev;
}

int rp23xx_sdio_probe(struct sdio_dev_s *dev)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  uint32_t r;
  int ret;
  int i;

  rp23xx_reset(dev);
  up_mdelay(1);                         /* 74 clocks and more at 400 kHz */

  for (i = 0; i < 3; i++)
    {
      sdio_cmd_r1(priv, MMCSD_CMD0, 0, NULL);
      up_mdelay(1);

      /* CMD8 (R7 echoes the pattern), or for older cards CMD55 */

      ret = sdio_cmd_r1(priv, SD_CMD8, 0x1aa, &r);
      if (ret == OK && (r & 0xfff) == 0x1aa)
        {
          return OK;
        }

      ret = sdio_cmd_r1(priv, SD_CMD55, 0, &r);
      if (ret == OK)
        {
          return OK;
        }
    }

  return -ENODEV;
}

void rp23xx_sdio_release(struct sdio_dev_s *dev)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  static const int pins[6] =
  {
    CLK_GPIO, CMD_GPIO, D0_GPIO, D0_GPIO + 1, D0_GPIO + 2, D0_GPIO + 3
  };

  int i;

  sdio_data_stop(priv);
  sm_disable(SM_ALL);
  modifyreg32(RP23XX_IO_BANK0_GPIO_CTRL(CMD_GPIO), 3 << 16, 0);
  for (i = 0; i < 6; i++)
    {
      rp23xx_gpio_init(pins[i]);
      rp23xx_gpio_set_pulls(pins[i], true, false);
    }
}

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  sdio_statset_t cdstatus;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  cdstatus = priv->cdstatus;
  if (cardinslot)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if (cdstatus != priv->cdstatus)
    {
      sdio_callback(priv);
    }
}

void rp23xx_sdio_readonly(struct sdio_dev_s *dev, bool readonly)
{
  ((struct rp23xx_sdio_s *)dev)->nowrite = readonly;
}

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
{
  struct rp23xx_sdio_s *priv = (struct rp23xx_sdio_s *)dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (wrprotect)
    {
      priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

#endif /* CONFIG_RP23XX_SDIO */
