/****************************************************************************
 * libs/libnx/nxmu/nx_keys.c
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

#include <stdio.h>
#include <string.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>
#include <nuttx/nx/nxkeys.h>

#ifdef CONFIG_NX_KBD

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A US keyboard's shifted characters and the keys that type them */

static const char g_shifted[]   = "!@#$%^&*()_+{}|:\"<>?~";
static const char g_unshifted[] = "1234567890-=[]\\;',./`";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The xterm modifier parameter: 1 + Shift 1, Alt 2, Ctrl 4 (0: none) */

static int key_modparam(uint8_t flags)
{
  int m = ((flags & NX_KEY_SHIFT) ? 1 : 0) |
          ((flags & NX_KEY_ALT) ? 2 : 0) |
          ((flags & NX_KEY_CTRL) ? 4 : 0);

  return m ? m + 1 : 0;
}

/* A special key's VT100 / xterm sequence: CSI final (arrows, Home, End),
 * CSI n ~ (the editing keys, F5-F12), SS3 final (F1-F4); with modifiers
 * as xterm sends them (CSI 1 ; m final, CSI n ; m ~)
 */

static int key_sequence(uint8_t keycode, uint8_t flags, FAR uint8_t *buf)
{
  char seq[16];
  char final = 0;
  int len;
  int n = 0;
  int m = key_modparam(flags);

  switch (keycode)
    {
      case KEYCODE_UP:
        final = 'A';
        break;

      case KEYCODE_DOWN:
        final = 'B';
        break;

      case KEYCODE_RIGHT:
        final = 'C';
        break;

      case KEYCODE_LEFT:
        final = 'D';
        break;

      case KEYCODE_HOME:
        n = 1;
        break;

      case KEYCODE_INSERT:
        n = 2;
        break;

      case KEYCODE_FWDDEL:
        n = 3;
        break;

      case KEYCODE_END:
        n = 4;
        break;

      case KEYCODE_PAGEUP:
        n = 5;
        break;

      case KEYCODE_PAGEDOWN:
        n = 6;
        break;

      case KEYCODE_F5:
        n = 15;
        break;

      case KEYCODE_F6:
        n = 17;
        break;

      case KEYCODE_F7:
        n = 18;
        break;

      case KEYCODE_F8:
        n = 19;
        break;

      case KEYCODE_F9:
        n = 20;
        break;

      case KEYCODE_F10:
        n = 21;
        break;

      case KEYCODE_F11:
        n = 23;
        break;

      case KEYCODE_F12:
        n = 24;
        break;

      case KEYCODE_F1:
      case KEYCODE_F2:
      case KEYCODE_F3:
      case KEYCODE_F4:
        if (m == 0)
          {
            buf[0] = 0x1b;
            buf[1] = 'O';
            buf[2] = 'P' + (keycode - KEYCODE_F1);
            return 3;
          }

        final = 'P' + (keycode - KEYCODE_F1);
        break;

      case KEYCODE_ENTER:
        buf[0] = '\n';
        return 1;

      case KEYCODE_BACKDEL:
        buf[0] = 0x7f;
        return 1;

      default:
        return 0;
    }

  if (final != 0)
    {
      len = m ? snprintf(seq, sizeof(seq), "\033[1;%d%c", m, final) :
                snprintf(seq, sizeof(seq), "\033[%c", final);
    }
  else
    {
      len = m ? snprintf(seq, sizeof(seq), "\033[%d;%d~", n, m) :
                snprintf(seq, sizeof(seq), "\033[%d~", n);
    }

  len = len > NX_KEY_MAXBYTES ? NX_KEY_MAXBYTES : len;
  memcpy(buf, seq, len);
  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_kbdmode
 ****************************************************************************/

int nx_kbdmode(NXHANDLE handle, int mode)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  struct nxsvrmsg_kbdmode_s outmsg;

  outmsg.msgid = NX_SVRMSG_KBDMODE;
  outmsg.conn  = conn;
  outmsg.mode  = (uint8_t)mode;

  return nxmu_sendserver(conn, &outmsg, sizeof(outmsg));
}

/****************************************************************************
 * Name: nx_key_encode
 ****************************************************************************/

void nx_key_encode(FAR uint8_t *buf, FAR const struct nx_key_s *key)
{
  buf[0] = NX_KEY_PREFIX;
  buf[1] = key->flags;
  buf[2] = key->code;
  buf[3] = key->ch;
}

/****************************************************************************
 * Name: nx_key_next
 ****************************************************************************/

int nx_key_next(FAR const uint8_t **pch, FAR int *pnch,
                FAR struct nx_key_s *key, FAR uint8_t *byte)
{
  FAR const uint8_t *ch = *pch;
  int nch = *pnch;

  while (nch > 0)
    {
      if (ch[0] != NX_KEY_PREFIX)
        {
          *byte  = ch[0];
          *pch   = ch + 1;
          *pnch  = nch - 1;
          return NX_KEYITEM_BYTE;
        }

      if (nch < NX_KEY_SIZE)
        {
          break;                        /* Cut short: dropped */
        }

      key->flags = ch[1];
      key->code  = ch[2];
      key->ch    = ch[3];
      *pch  = ch + NX_KEY_SIZE;
      *pnch = nch - NX_KEY_SIZE;
      return NX_KEYITEM_KEY;
    }

  *pch  = ch + nch;
  *pnch = 0;
  return NX_KEYITEM_NONE;
}

/****************************************************************************
 * Name: nx_key_bytes
 ****************************************************************************/

int nx_key_bytes(FAR const struct nx_key_s *key, FAR uint8_t *buf)
{
  if ((key->flags & NX_KEY_RELEASE) != 0)
    {
      return 0;
    }

  if (NX_KEY_ISSPECIAL(key->code))
    {
      return key_sequence(NX_KEY_KEYCODE(key->code), key->flags, buf);
    }

  /* Shift-Tab: back tab, as xterm */

  if (key->code == NX_KEY_TAB && (key->flags & NX_KEY_SHIFT) != 0)
    {
      buf[0] = 0x1b;
      buf[1] = '[';
      buf[2] = 'Z';
      return 3;
    }

  if (key->ch != 0)
    {
      buf[0] = key->ch;
      return 1;
    }

  switch (key->code)
    {
      case NX_KEY_ENTER:
        buf[0] = '\n';
        return 1;

      case NX_KEY_TAB:
        buf[0] = '\t';
        return 1;

      case NX_KEY_BACKSPACE:
        buf[0] = 0x7f;
        return 1;

      case NX_KEY_ESCAPE:
        buf[0] = 0x1b;
        return 1;

      default:
        return 0;
    }
}

/****************************************************************************
 * Name: nx_key_split
 ****************************************************************************/

int nx_key_split(FAR const uint8_t *ch, int nch, int max)
{
  int i;

  if (nch <= max)
    {
      return nch;
    }

  /* A record starting in the last NX_KEY_SIZE - 1 bytes goes whole in the
   * next message (records are never cut, unless one is longer than max)
   */

  for (i = max - 1; i > max - NX_KEY_SIZE && i > 0; i--)
    {
      if (ch[i] == NX_KEY_PREFIX)
        {
          return i;
        }
    }

  return max;
}

/****************************************************************************
 * Name: nx_key_code
 ****************************************************************************/

uint8_t nx_key_code(uint8_t ch, bool ctrl)
{
  FAR const char *p;

  if (ch >= 'A' && ch <= 'Z')
    {
      return ch - 'A' + 'a';
    }

  if (ch >= 0x01 && ch <= 0x1a &&
      (ctrl || (ch != '\b' && ch != '\t' && ch != '\n' && ch != '\r')))
    {
      return ch - 0x01 + 'a';                  /* Ctrl-a to Ctrl-z */
    }

  switch (ch)
    {
      case '\b':
      case 0x7f:
        return NX_KEY_BACKSPACE;

      case '\t':
        return NX_KEY_TAB;

      case '\n':
      case '\r':
        return NX_KEY_ENTER;

      case 0x1b:
        return NX_KEY_ESCAPE;

      case 0x00:
        return '2';                            /* Ctrl-@ */

      case 0x1c:
        return '\\';

      case 0x1d:
        return ']';

      case 0x1e:
        return '6';

      case 0x1f:
        return '-';

      default:
        break;
    }

  if (ch != 0 && (p = strchr(g_shifted, ch)) != NULL)
    {
      return (uint8_t)g_unshifted[p - g_shifted];
    }

  return ch;
}

#endif /* CONFIG_NX_KBD */
