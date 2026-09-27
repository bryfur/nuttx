/****************************************************************************
 * include/nuttx/audio/fpc_mixer.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The FPC's software mixer (fpc/drivers/audio/fpc_audio.c): the programs
 * playing (each on a /dev/audio/pcmN of its own), their levels, the
 * master volume and mute, and what the GPU (HDMI) is doing with them.
 *
 * /dev/mixer, a character device:
 *
 *   read()     the state as text (what `cat /dev/mixer` shows)
 *   poll()     POLLIN when anything has changed since this open file last
 *              read the state (MIXIOC_GETINFO or read()): a volume key,
 *              another program's change, a stream starting or ending
 *   ioctl()    below
 *
 * Every /dev/audio/pcmN is a channel of the mixer, a NuttX audio device
 * (nxplayer and the audio ioctls work on it).  Besides the audio buffers
 * of the upper half, a channel takes write(): PCM in the format last
 * configured (48 kHz stereo 16-bit if none), blocking while the channel's
 * FIFO is full; closing it lets what was written play out.  Each channel
 * also answers MIXIOC_SETNAME.
 *
 * Volumes are 0..1000 like AUDIO_FU_VOLUME.  The master volume and the
 * application levels are slider positions: the gain is (v / 1000)^3 (50 %
 * is -18 dB); a stream's own volume (AUDIO_FU_VOLUME) is linear.
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_AUDIO_FPC_MIXER_H
#define __INCLUDE_NUTTX_AUDIO_FPC_MIXER_H

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FPCMIX_DEVPATH       "/dev/mixer"
#define FPCMIX_PCMPATH       "/dev/audio/pcm%d"

#define FPCMIX_NCHAN_MAX     8       /* Channels (pcm devices) at most */
#define FPCMIX_NAME_MAX      16      /* A client's name, with its NUL */
#define FPCMIX_LEVELS_MAX    12      /* Application levels remembered */
#define FPCMIX_VOLUME_MAX    1000

/* A channel's state */

#define FPCMIX_IDLE          0       /* Free */
#define FPCMIX_OPEN          1       /* Reserved, not playing */
#define FPCMIX_PLAYING       2
#define FPCMIX_PAUSED        3
#define FPCMIX_DRAINING      4       /* Closed, what was written plays out */

/* ioctls on /dev/mixer */

#define MIXIOC_GETINFO       _AUDIOIOC(0x60)  /* struct fpcmix_info_s * */
#define MIXIOC_SETMASTER     _AUDIOIOC(0x61)  /* int: 0..1000 */
#define MIXIOC_STEPMASTER    _AUDIOIOC(0x62)  /* int: +- change, unmutes */
#define MIXIOC_SETMUTE       _AUDIOIOC(0x63)  /* int: 0 off, 1 on, 2 toggle */
#define MIXIOC_SETLEVEL      _AUDIOIOC(0x64)  /* struct fpcmix_level_s * */
#define MIXIOC_GETLEVELS     _AUDIOIOC(0x65)  /* struct fpcmix_levels_s * */

/* ioctl on a /dev/audio/pcmN: the name it is listed under, and whose
 * application level applies (the task's name until set)
 */

#define MIXIOC_SETNAME       _AUDIOIOC(0x68)  /* const char * */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct fpcmix_chan_s
{
  uint8_t  state;                    /* FPCMIX_* */
  uint8_t  channels;                 /* 1 or 2 */
  uint8_t  bits;                     /* 8, 16, 24, 32 */
  uint8_t  writing;                  /* Fed by write(), not buffers */
  uint32_t rate;                     /* Its sample rate */
  int32_t  pid;                      /* Who reserved it, -1 */
  uint16_t volume;                   /* Its own volume, 0..1000 */
  uint16_t level;                    /* Its application's level */
  uint16_t peak;                     /* Recent peak after the gains, 0..32767 */
  uint16_t pad;
  uint32_t frames;                   /* Frames played (at its rate) */
  uint32_t starved;                  /* Output frames it had nothing for */
  char     name[FPCMIX_NAME_MAX];
};

struct fpcmix_info_s
{
  uint32_t serial;                   /* Changes with anything below but the
                                      * counters */
  uint16_t master;                   /* 0..1000 */
  uint8_t  mute;
  uint8_t  nchan;                    /* Channels: /dev/audio/pcm0.. */
  uint8_t  gpu;                      /* 1: the GPU plays; 0: no audio in the
                                      * GPU (discarded at the rate) */
  uint8_t  pad[3];
  uint16_t latency;                  /* Queued ahead in the GPU, ms (target) */
  uint16_t peak_l;                   /* Recent output peaks, 0..32767 */
  uint16_t peak_r;
  uint16_t ringms;                   /* The GPU's ring, ms */
  uint32_t rate;                     /* The output rate, 48000 */

  /* The GPU's counters (AUDIO_STATUS) as last read */

  uint32_t played;
  uint32_t queued;
  uint32_t underruns;
  uint32_t overflows;

  uint32_t xruns;                    /* Times the ring ran dry mid-stream */
  uint32_t sent;                     /* Frames sent */
  uint32_t mixus;                    /* CPU time mixing in the last second */
  struct fpcmix_chan_s chan[FPCMIX_NCHAN_MAX];
};

struct fpcmix_level_s
{
  char     name[FPCMIX_NAME_MAX];
  uint16_t level;                    /* 0..1000 */
};

struct fpcmix_levels_s
{
  int      n;
  struct fpcmix_level_s l[FPCMIX_LEVELS_MAX];
};

#endif /* __INCLUDE_NUTTX_AUDIO_FPC_MIXER_H */
