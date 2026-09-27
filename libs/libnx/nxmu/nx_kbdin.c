/****************************************************************************
 * libs/libnx/nxmu/nx_kbdin.c
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
#include <errno.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxkeys.h>
#include <nuttx/nx/nxmu.h>

#include "nxcontext.h"

#ifdef CONFIG_NX_KBD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_kbdin
 *
 * Description:
 *   Used by a thread or interrupt handler that manages some kind of keypad
 *   hardware to report text information to the NX server.  That text
 *   data will be routed by the NX server to the appropriate window client.
 *
 ****************************************************************************/

int nx_kbdin(NXHANDLE handle, uint8_t nch, FAR const uint8_t *ch)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  union
  {
    struct nxsvrmsg_kbdin_s msg;
    uint8_t buf[NX_MXSVRMSGLEN];
  } out;

  int ret = OK;
  int n;

  /* As many messages as it takes: each must fit the server's queue, and
   * the one the server sends on must fit the client's
   */

  while (nch > 0 && ret >= 0)
    {
      n = nx_key_split(ch, nch, NX_KBDIN_MAXCH);
      out.msg.msgid = NX_SVRMSG_KBDIN;
      out.msg.nch   = n;
      memcpy(out.msg.ch, ch, n);

      ret  = nxmu_sendserver(conn, &out.msg,
                             offsetof(struct nxsvrmsg_kbdin_s, ch) + n);
      ch  += n;
      nch -= n;
    }

  return ret;
}

#endif /* CONFIG_NX_KBD */
