/****************************************************************************
 * libs/libnx/nxmu/nx_wheelin.c
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

#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>

#ifdef CONFIG_NX_XYINPUT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_wheelin
 *
 * Description:
 *   Used by the thread that manages a pointing device to report that its
 *   wheel turned (or a touchpad scrolled), in 1/NX_WHEEL_STEP of a step.
 *   The NX server hands it to the window under the pointer.
 *
 ****************************************************************************/

int nx_wheelin(NXHANDLE handle, int dx, int dy)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)handle;
  struct nxsvrmsg_wheelin_s outmsg;

  if (dx == 0 && dy == 0)
    {
      return OK;
    }

  outmsg.msgid = NX_SVRMSG_WHEELIN;
  outmsg.dx    = (int16_t)(dx < INT16_MIN ? INT16_MIN :
                           dx > INT16_MAX ? INT16_MAX : dx);
  outmsg.dy    = (int16_t)(dy < INT16_MIN ? INT16_MIN :
                           dy > INT16_MAX ? INT16_MAX : dy);
  return nxmu_sendserver(conn, &outmsg, sizeof(struct nxsvrmsg_wheelin_s));
}

#endif /* CONFIG_NX_XYINPUT */
