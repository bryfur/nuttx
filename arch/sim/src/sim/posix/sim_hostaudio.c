/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostaudio.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * A board's audio out to the host's speakers: raw 16-bit little-endian
 * PCM into a pipe to a player, `aplay` unless $SIM_AUDIO_PLAYER names
 * another command line (empty: no sound).  The pipe does not block: what
 * does not fit is dropped, so the simulation never waits for the host's
 * sound card.  The board writes at its own (host) clock; the player plays
 * at the sound card's.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#define _GNU_SOURCE 1

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sim_internal.h"

extern char **environ;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_fd = -1;                   /* The pipe to the player */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_audio_open
 *
 * Description:
 *   Start the player for a stream of rate Hz, channels channels.  0, or -1
 *   (no player).  The pipe's end is kept away from the low descriptors,
 *   which the simulation's console may take over.
 *
 ****************************************************************************/

static int audio_open(unsigned int rate, unsigned int channels)
{
  const char *player = getenv("SIM_AUDIO_PLAYER");
  struct sigaction sa;
  char cmd[320];
  char *argv[4];
  int p[2];
  int rd;
  pid_t pid;

  /* (Only the host's functions that NuttX's do not shadow: see
   * nuttx-names.in; pipe2(), fcntl(), close(), posix_spawn() and
   * sigaction() are the host's here, dup2() and the spawn file actions
   * would be NuttX's)
   */

  if (g_fd >= 0)
    {
      return 0;
    }

  if (player != NULL && player[0] == '\0')
    {
      return -1;
    }

  if (pipe2(p, O_CLOEXEC) < 0)
    {
      return -1;
    }

  rd   = fcntl(p[0], F_DUPFD, 64);           /* The player's: inherited */
  g_fd = fcntl(p[1], F_DUPFD_CLOEXEC, 64);   /* Ours */
  close(p[0]);
  close(p[1]);
  if (rd < 0 || g_fd < 0)
    {
      goto errout;
    }

  /* The shell makes the pipe the player's standard input (by its path:
   * sh takes only single-digit descriptors)
   */

  if (player != NULL)
    {
      snprintf(cmd, sizeof(cmd), "exec </dev/fd/%d; %s", rd, player);
    }
  else
    {
      snprintf(cmd, sizeof(cmd), "exec </dev/fd/%d; exec aplay -q -t raw "
               "-f S16_LE -r %u -c %u -B 100000 2>/dev/null", rd, rate,
               channels);
    }

  argv[0] = "sh";
  argv[1] = "-c";
  argv[2] = cmd;
  argv[3] = NULL;
  if (posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ) != 0)
    {
      goto errout;
    }

  close(rd);

  /* A player that goes away must not take the simulation with it */

  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = SIG_IGN;
  sigaction(SIGPIPE, &sa, NULL);
  fcntl(g_fd, F_SETFL, O_NONBLOCK);
  return 0;

errout:
  if (rd >= 0)
    {
      close(rd);
    }

  if (g_fd >= 0)
    {
      close(g_fd);
      g_fd = -1;
    }

  return -1;
}

/* All of it with the simulation's signals held off: one arriving while
 * the player is spawned would switch NuttX tasks in the middle of it
 */

int host_audio_open(unsigned int rate, unsigned int channels)
{
  return host_uninterruptible(audio_open, rate, channels);
}

/****************************************************************************
 * Name: host_audio_write
 *
 * Description:
 *   Hand the player nbytes (whole frames).  Pieces of up to PIPE_BUF go
 *   into the pipe whole or not at all, so frames are never split.  Returns
 *   the bytes taken.
 *
 ****************************************************************************/

int host_audio_write(const void *buf, int nbytes, int framebytes)
{
  const char *p = buf;
  int chunk = PIPE_BUF - PIPE_BUF % framebytes;
  int done = 0;

  if (g_fd < 0)
    {
      return 0;
    }

  while (done < nbytes)
    {
      int n = nbytes - done > chunk ? chunk : nbytes - done;
      ssize_t ret = host_uninterruptible(write, g_fd, p + done, n);

      if (ret <= 0)
        {
          if (ret < 0 && errno == EPIPE)
            {
              host_uninterruptible(close, g_fd);   /* The player has gone */
              g_fd = -1;
            }

          break;
        }

      done += ret;
    }

  return done;
}

/****************************************************************************
 * Name: host_audio_close
 ****************************************************************************/

void host_audio_close(void)
{
  if (g_fd >= 0)
    {
      host_uninterruptible(close, g_fd);
      g_fd = -1;
    }
}
