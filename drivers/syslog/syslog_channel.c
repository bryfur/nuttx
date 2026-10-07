/****************************************************************************
 * drivers/syslog/syslog_channel.c
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
#include <assert.h>
#include <errno.h>
#include <semaphore.h>

#include <nuttx/syslog/syslog.h>
#include <nuttx/compiler.h>
#include <nuttx/init.h>

#ifdef CONFIG_RAMLOG_SYSLOG
#  include <nuttx/syslog/ramlog.h>
#endif

#ifdef CONFIG_SYSLOG_RPMSG
#  include <nuttx/syslog/syslog_rpmsg.h>
#endif

#ifdef CONFIG_SYSLOG_RTT
#  include <nuttx/segger/rtt.h>
#endif

#ifdef CONFIG_SYSLOG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_ARCH_LOWPUTC
#  include <nuttx/arch.h>
#endif

#ifdef CONFIG_SYSLOG_DEFAULT_CONSOLE
#  include <fcntl.h>
#  include <nuttx/irq.h>
#  include <nuttx/sched.h>
#  include <nuttx/signal.h>
#  include <nuttx/fs/fs.h>
#  include <nuttx/serial/serial.h>
#endif

#include "syslog.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_DEFAULT
static int syslog_default_putc(FAR syslog_channel_t *channel,
                               int ch);
static ssize_t syslog_default_write(FAR syslog_channel_t *channel,
                                    FAR const char *buffer, size_t buflen);
#  ifdef CONFIG_SYSLOG_DEFAULT_CONSOLE
static int syslog_default_force(FAR syslog_channel_t *channel, int ch);
static ssize_t syslog_default_write_force(FAR syslog_channel_t *channel,
                                          FAR const char *buffer,
                                          size_t buflen);
static int syslog_default_flush(FAR syslog_channel_t *channel);
#  endif
#endif

#ifdef CONFIG_SYSLOG_CDCACM
static int syslog_cdcacm_putc(FAR syslog_channel_t *channel,
                              int ch);
static ssize_t syslog_cdcacm_write(FAR syslog_channel_t *channel,
                                   FAR const char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
static const struct syslog_channel_ops_s g_ramlog_channel_ops =
{
  ramlog_putc,
  ramlog_putc,
  NULL,
  ramlog_write,
  ramlog_write
};

static syslog_channel_t g_ramlog_channel =
{
  &g_ramlog_channel_ops
#  ifdef CONFIG_SYSLOG_IOCTL
  , "ram"
#  endif
#  ifdef CONFIG_SYSLOG_CRLF
  , SYSLOG_CHANNEL_DISABLE_CRLF
#  endif
};
#endif

#ifdef CONFIG_SYSLOG_RPMSG
static const struct syslog_channel_ops_s g_rpmsg_channel_ops =
{
  syslog_rpmsg_putc,
  syslog_rpmsg_putc,
  syslog_rpmsg_flush,
  syslog_rpmsg_write,
  syslog_rpmsg_write
};

static syslog_channel_t g_rpmsg_channel =
{
  &g_rpmsg_channel_ops
#  ifdef CONFIG_SYSLOG_IOCTL
  , "rpmsg"
#  endif
#  ifdef CONFIG_SYSLOG_CRLF
  , SYSLOG_CHANNEL_DISABLE_CRLF
#  endif
};
#endif

#ifdef CONFIG_SYSLOG_RTT
static const struct syslog_channel_ops_s g_rtt_channel_ops =
{
  syslog_rtt_putc,
  syslog_rtt_putc,
  NULL,
  syslog_rtt_write,
  syslog_rtt_write
};

static syslog_channel_t g_rtt_channel =
{
  &g_rtt_channel_ops
#  ifdef CONFIG_SYSLOG_IOCTL
  , "rtt"
#  endif
#  ifdef CONFIG_SYSLOG_CRLF
  , SYSLOG_CHANNEL_DISABLE_CRLF
#  endif
};
#endif

#ifdef CONFIG_SYSLOG_CDCACM
static const struct syslog_channel_ops_s g_cdcacm_channel_ops =
{
  syslog_cdcacm_putc,
  syslog_cdcacm_putc,
  NULL,
  syslog_cdcacm_write,
  syslog_cdcacm_write
};

static struct syslog_channel_s g_cdcacm_channel =
{
  &g_cdcacm_channel_ops
#  ifdef CONFIG_SYSLOG_IOCTL
  , "cdcacm"
#  endif
};
#endif

#ifdef CONFIG_SYSLOG_DEFAULT
static const struct syslog_channel_ops_s g_default_channel_ops =
{
  syslog_default_putc,
#  ifdef CONFIG_SYSLOG_DEFAULT_CONSOLE
  syslog_default_force,
  syslog_default_flush,
  syslog_default_write,
  syslog_default_write_force
#  else
  syslog_default_putc,
  NULL,
  syslog_default_write
#  endif
};

#  ifdef CONFIG_SYSLOG_DEFAULT_CONSOLE
/* /dev/console, opened by syslog_initialize() */

static struct file g_default_console;
static bool g_default_console_open;
#  endif

static syslog_channel_t g_default_channel =
{
  &g_default_channel_ops
#  ifdef CONFIG_SYSLOG_IOCTL
  , "default"
#  endif
};
#endif

/* This is a simply sanity check to avoid we have more elements than the
 * `g_syslog_channel` array can hold
 */

#ifdef CONFIG_SYSLOG_DEFAULT
#  define SYSLOG_DEFAULT_AVAILABLE 1
#else
#  define SYSLOG_DEFAULT_AVAILABLE 0
#endif

#ifdef CONFIG_RAMLOG_SYSLOG
#  define RAMLOG_SYSLOG_AVAILABLE 1
#else
#  define RAMLOG_SYSLOG_AVAILABLE 0
#endif

#ifdef CONFIG_SYSLOG_RPMSG
#  define SYSLOG_RPMSG_AVAILABLE 1
#else
#  define SYSLOG_RPMSG_AVAILABLE 0
#endif

#ifdef CONFIG_SYSLOG_RTT
#  define SYSLOG_RTT_AVAILABLE 1
#else
#  define SYSLOG_RTT_AVAILABLE 0
#endif

#define SYSLOG_NCHANNELS (SYSLOG_DEFAULT_AVAILABLE + \
                          RAMLOG_SYSLOG_AVAILABLE + \
                          SYSLOG_RPMSG_AVAILABLE + \
                          SYSLOG_RTT_AVAILABLE)

#if SYSLOG_NCHANNELS > CONFIG_SYSLOG_MAX_CHANNELS
#  error "Maximum channel number exceeds."
#endif

/* This is the current syslog channel in use */

FAR syslog_channel_t *
#ifndef CONFIG_SYSLOG_REGISTER
const
#endif
g_syslog_channel[CONFIG_SYSLOG_MAX_CHANNELS] =
{
#ifdef CONFIG_SYSLOG_DEFAULT
  &g_default_channel,
#endif
#ifdef CONFIG_RAMLOG_SYSLOG
  &g_ramlog_channel,
#endif
#ifdef CONFIG_SYSLOG_RPMSG
  &g_rpmsg_channel,
#endif
#ifdef CONFIG_SYSLOG_RTT
  &g_rtt_channel,
#endif
#ifdef CONFIG_SYSLOG_CDCACM
  &g_cdcacm_channel
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_SYSLOG_DEFAULT) && defined(CONFIG_ARCH_LOWPUTC)

/****************************************************************************
 * Name: csection_available
 *
 * Description:
 *   Return true if the critical section is available.
 *
 ****************************************************************************/

static bool csection_available(void)
{
  /* Degrade the critical section in a few cases:
   *
   * a) early in the boot, where tasks are not available
   *
   * b) after a panic, where taking a lock can make the situation worse
   */

  return OSINIT_TASK_READY() && g_nx_initstate != OSINIT_PANIC;
}

#endif /* defined(CONFIG_SYSLOG_DEFAULT) && defined(CONFIG_ARCH_LOWPUTC) */

/****************************************************************************
 * Name: syslog_default_putc
 *
 * Description:
 *   If the arch supports a low-level putc function, output will be
 *   redirected there. Else acts as a dummy, no-nothing channel.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_DEFAULT
#  ifdef CONFIG_SYSLOG_DEFAULT_CONSOLE
/* (FPC) Can this context write through the console's serial driver?  Not
 * before /dev/console is open, not from an interrupt handler or the IDLE
 * loop (they may not sleep), not with interrupts masked (a spinlock or
 * critical section holder may not sleep either; up_putc() stays the way
 * there) and not after a panic.
 */

static bool syslog_default_console_ok(void)
{
  return g_default_console_open && !OSINIT_IS_PANIC() &&
         !up_interrupt_context() && !sched_idletask() &&
         getbasepri() == 0 && getprimask() == 0;
}

static int syslog_default_putc(FAR syslog_channel_t *channel, int ch)
{
  char c = ch;

  if (syslog_default_console_ok() &&
      file_write(&g_default_console, &c, 1) == 1)
    {
      return ch;
    }

  return syslog_default_force(channel, ch);
}

static ssize_t syslog_default_write(FAR syslog_channel_t *channel,
                                    FAR const char *buffer, size_t buflen)
{
  if (syslog_default_console_ok())
    {
      ssize_t ret = file_write(&g_default_console, buffer, buflen);

      if (ret >= 0)
        {
          return ret;
        }
    }

  return syslog_default_write_force(channel, buffer, buflen);
}

/* syslog_flush(): after a panic (the other CPUs paused, interrupts off)
 * send what the serial driver still holds, by polling, so that it comes
 * out before the panic's own output; from a task, wait until the driver
 * has sent it (fpc_mark() does this before its direct output).
 */

static int syslog_default_flush(FAR syslog_channel_t *channel)
{
  FAR uart_dev_t *dev;
  int ms;

  UNUSED(channel);
  if (!g_default_console_open)
    {
      return OK;
    }

  dev = g_default_console.f_inode->i_private;
  if (OSINIT_IS_PANIC())
    {
      while (dev->xmit.tail != dev->xmit.head)
        {
          char c = dev->xmit.buffer[dev->xmit.tail];

          if (c != '\r') /* up_putc() adds one before each '\n' */
            {
              up_putc(c);
            }

          dev->xmit.tail = dev->xmit.tail + 1 >= dev->xmit.size ?
                           0 : dev->xmit.tail + 1;
        }
    }
  else if (syslog_default_console_ok())
    {
      for (ms = 0; ms < 500 && dev->xmit.tail != dev->xmit.head; ms++)
        {
          nxsig_usleep(1000);
        }
    }

  return OK;
}

/* The polled output (interrupts, IDLE, interrupts masked, panic, and
 * before /dev/console is open)
 */

static int syslog_default_force(FAR syslog_channel_t *channel, int ch)
#  else
static int syslog_default_putc(FAR syslog_channel_t *channel, int ch)
#  endif
{
#  ifdef CONFIG_ARCH_LOWPUTC
  if (csection_available())
    {
      /* See https://github.com/apache/nuttx/issues/14662
       * about what this critical section is for.
       */

      irqstate_t flags = enter_critical_section();
      up_putc(ch);
      leave_critical_section(flags);
    }
  else
    {
      up_putc(ch);
    }
#  endif

  UNUSED(channel);
  return ch;
}

#  ifdef CONFIG_SYSLOG_DEFAULT_CONSOLE
static ssize_t syslog_default_write_force(FAR syslog_channel_t *channel,
                                          FAR const char *buffer,
                                          size_t buflen)
#  else
static ssize_t syslog_default_write(FAR syslog_channel_t *channel,
                                    FAR const char *buffer, size_t buflen)
#  endif
{
#  ifdef CONFIG_ARCH_LOWPUTC
  if (csection_available())
    {
      /* See https://github.com/apache/nuttx/issues/14662
       * about what this critical section is for.
       */

      irqstate_t flags = enter_critical_section();
      up_nputs(buffer, buflen);
      leave_critical_section(flags);
    }
  else
    {
      up_nputs(buffer, buflen);
    }
#  endif

  UNUSED(channel);
  return buflen;
}
#endif

#ifdef CONFIG_SYSLOG_CDCACM
static int syslog_cdcacm_putc(FAR syslog_channel_t *channel, int ch)
{
  char tmp;

  tmp = ch;
  cdcacm_write(&tmp, 1);

  UNUSED(channel);
  return ch;
}

static ssize_t syslog_cdcacm_write(FAR syslog_channel_t *channel,
                                   FAR const char *buffer, size_t buflen)
{
  UNUSED(channel);

  return cdcacm_write(buffer, buflen);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_default_console_open
 *
 * Description:
 *   (FPC, CONFIG_SYSLOG_DEFAULT_CONSOLE) Open /dev/console for the default
 *   channel: from now on a task's output goes through the console's
 *   buffered serial driver.  Called by syslog_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_DEFAULT_CONSOLE
int syslog_default_console_open(void)
{
  int ret = file_open(&g_default_console, "/dev/console",
                      O_WRONLY | O_CLOEXEC);

  if (ret >= 0)
    {
      g_default_console_open = true;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: syslog_channel
 *
 * Description:
 *   Configure the SYSLOGging function to use the provided channel to
 *   generate SYSLOG output.
 *
 * Input Parameters:
 *   channel - Provides the interface to the channel to be used.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_REGISTER
int syslog_channel_register(FAR syslog_channel_t *channel)
{
  DEBUGASSERT(channel != NULL);

  if (channel != NULL)
    {
#if CONFIG_SYSLOG_MAX_CHANNELS == 1
      g_syslog_channel[0] = channel;
      return OK;
#else
      int i;

      for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
        {
          if (g_syslog_channel[i] == NULL)
            {
#  ifdef CONFIG_SYSLOG_IOCTL
              if (channel->sc_name[0] == '\0')
                {
                  snprintf(channel->sc_name, sizeof(channel->sc_name),
                           "channel-%p", channel->sc_ops);
                }
#  endif

              g_syslog_channel[i] = channel;
              return OK;
            }
          else if (g_syslog_channel[i] == channel)
            {
              return OK;
            }
        }
#endif
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: syslog_channel_unregister
 *
 * Description:
 *   Removes an already configured SYSLOG channel from the list of used
 *   channels.
 *
 * Input Parameters:
 *   channel - Provides the interface to the channel to be removed.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int syslog_channel_unregister(FAR syslog_channel_t *channel)
{
  int i;

  DEBUGASSERT(channel != NULL);

  if (channel != NULL)
    {
      for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
        {
          if (g_syslog_channel[i] == channel)
            {
              /* Get the rest of the channels one position back
               * to ensure that there are no holes in the list.
               */

              while (i < (CONFIG_SYSLOG_MAX_CHANNELS - 1) &&
                     g_syslog_channel[i + 1] != NULL)
                {
                  g_syslog_channel[i] = g_syslog_channel[i + 1];
                  i++;
                }

              g_syslog_channel[i] = NULL;

              /* The channel is now removed from the list and its driver
               * can be safely uninitialized.
               */

              if (channel->sc_ops->sc_close)
                {
                  channel->sc_ops->sc_close(channel);
                }

              return OK;
            }
        }
    }

  return -EINVAL;
}
#endif
