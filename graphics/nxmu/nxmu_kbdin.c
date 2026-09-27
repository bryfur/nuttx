/****************************************************************************
 * graphics/nxmu/nxmu_kbdin.c
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

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxkeys.h>

#include "nxmu.h"

#ifdef CONFIG_NX_KBD

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Send nch bytes to the window's client, in as many messages as fit its
 * queue, never cutting a key event in two
 */

static void nxmu_kbdsend(FAR struct nxbe_window_s *wnd, int nch,
                         FAR const uint8_t *ch)
{
  union
  {
    struct nxclimsg_kbdin_s msg;
    uint8_t buf[NX_MXCLIMSGLEN];
  } out;

  int n;

  while (nch > 0)
    {
      n = nx_key_split(ch, nch, NX_KBDIN_MAXCH);
      out.msg.msgid = NX_CLIMSG_KBDIN;
      out.msg.wnd   = wnd;
      out.msg.nch   = n;
      memcpy(out.msg.ch, ch, n);

      nxmu_sendclientwindow(wnd, &out.msg,
                            offsetof(struct nxclimsg_kbdin_s, ch) + n);
      ch  += n;
      nch -= n;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_kbdin
 *
 * Description:
 *   New keyboard data has been received from the thread or interrupt
 *   handler that manages some kind of keyboard/keypad hardware.  Route that
 *   positional data to the appropriate window client.
 *
 *   The data may hold key events (nxkeys.h).  A client that asked for them
 *   (nx_kbdmode()) gets the data as it is; any other gets bytes: what each
 *   press types, nothing for the releases.
 *
 ****************************************************************************/

void nxmu_kbdin(FAR struct nxmu_state_s *nxmu, uint8_t nch, FAR uint8_t *ch)
{
  FAR struct nxbe_window_s *wnd = nxmu->be.topwnd;
  FAR const uint8_t *in = ch;
  uint8_t out[2 * NX_KBDIN_MAXCH];
  struct nx_key_s key;
  uint8_t byte;
  int left = nch;
  int n = 0;
  int item;

  if (wnd == NULL)
    {
      return;
    }

  /* Only to the top window */

  if (wnd->conn != NULL && wnd->conn->kbdmode == NX_KBDMODE_KEYS)
    {
      nxmu_kbdsend(wnd, nch, ch);
      return;
    }

  while ((item = nx_key_next(&in, &left, &key, &byte)) != NX_KEYITEM_NONE)
    {
      if (n + NX_KEY_MAXBYTES > (int)sizeof(out))
        {
          nxmu_kbdsend(wnd, n, out);
          n = 0;
        }

      if (item == NX_KEYITEM_KEY)
        {
          n += nx_key_bytes(&key, &out[n]);
        }
      else
        {
          out[n++] = byte;
        }
    }

  if (n > 0)
    {
      nxmu_kbdsend(wnd, n, out);
    }
}

#endif /* CONFIG_NX_KBD */
