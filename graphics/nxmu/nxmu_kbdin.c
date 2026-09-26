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

#include "nxmu.h"

#ifdef CONFIG_NX_KBD

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
 ****************************************************************************/

void nxmu_kbdin(FAR struct nxmu_state_s *nxmu, uint8_t nch, FAR uint8_t *ch)
{
  union
  {
    struct nxclimsg_kbdin_s msg;
    uint8_t buf[NX_MXCLIMSGLEN];
  } out;

  int n;

  /* Only to the top window; in as many messages as fit its client's
   * queue
   */

  while (nch > 0)
    {
      n = nch < NX_KBDIN_MAXCH ? nch : NX_KBDIN_MAXCH;
      out.msg.msgid = NX_CLIMSG_KBDIN;
      out.msg.wnd   = nxmu->be.topwnd;
      out.msg.nch   = n;
      memcpy(out.msg.ch, ch, n);

      nxmu_sendclientwindow(nxmu->be.topwnd, &out.msg,
                            offsetof(struct nxclimsg_kbdin_s, ch) + n);
      ch  += n;
      nch -= n;
    }
}

#endif /* CONFIG_NX_KBD */
