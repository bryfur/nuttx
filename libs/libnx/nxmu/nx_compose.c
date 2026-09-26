/****************************************************************************
 * libs/libnx/nxmu/nx_compose.c
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
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxmu.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Send a message that carries the caller's memory and wait until the
 * server is done with it.  wnd NULL: a message to the server.
 */

static int nx_compose_wait(FAR struct nxmu_conn_s *conn,
                           FAR struct nxbe_window_s *wnd, FAR void *msg,
                           size_t len, FAR sem_t **sem_done)
{
  sem_t sem;
  int ret;

  ret = nxsem_init(&sem, 0, 0);
  if (ret < 0)
    {
      return ret;
    }

  *sem_done = &sem;
  ret = wnd != NULL ? nxmu_sendwindow(wnd, msg, len) :
                      nxmu_sendserver(conn, msg, len);
  if (ret == OK)
    {
      ret = nxsem_wait_uninterruptible(&sem);
    }

  nxsem_destroy(&sem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int nx_compose_query(NXHANDLE handle, FAR struct nxgl_size_s *size)
{
  struct nxsvrmsg_compositor_s msg;
  int caps = 0;
  int ret;

  msg.msgid = NX_SVRMSG_COMPOSITOR;
  msg.caps  = &caps;
  msg.size  = size;
  ret = nx_compose_wait((FAR struct nxmu_conn_s *)handle, NULL, &msg,
                        sizeof(msg), &msg.sem_done);
  return ret < 0 ? ret : caps;
}

int nx_compositor(NXHANDLE handle)
{
  struct nxgl_size_s size;
  int caps = nx_compose_query(handle, &size);

  return caps < 0 ? 0 : caps;
}

int nx_displaysize(NXHANDLE handle, FAR struct nxgl_size_s *size)
{
  int ret = nx_compose_query(handle, size);

  return ret < 0 ? ret : OK;
}

static int nx_displaymode(NXHANDLE handle, int mode, bool set,
                          FAR struct nxgl_size_s *size)
{
  struct nxsvrmsg_displaymode_s msg;
  struct nxgl_size_s dummy;
  int result = -ENOSYS;
  int ret;

  msg.msgid  = NX_SVRMSG_DISPLAYMODE;
  msg.mode   = mode;
  msg.set    = set;
  msg.result = &result;
  msg.size   = size != NULL ? size : &dummy;
  ret = nx_compose_wait((FAR struct nxmu_conn_s *)handle, NULL, &msg,
                        sizeof(msg), &msg.sem_done);
  return ret < 0 ? ret : result;
}

int nx_getdisplaymode(NXHANDLE handle, int mode,
                      FAR struct nxgl_size_s *size)
{
  return nx_displaymode(handle, mode, false, size);
}

int nx_setdisplaymode(NXHANDLE handle, int mode)
{
  return nx_displaymode(handle, mode, true, NULL);
}

int nx_seteffects(NXWINDOW hwnd, FAR const struct nx_effects_s *fx)
{
  struct nxsvrmsg_seteffects_s msg;

  if (hwnd == NULL || fx == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  msg.msgid = NX_SVRMSG_SETEFFECTS;
  msg.wnd   = (FAR struct nxbe_window_s *)hwnd;
  msg.fx    = *fx;
  return nxmu_sendwindow(msg.wnd, &msg, sizeof(msg));
}

static int nx_sendops(NXWINDOW hwnd, uint32_t msgid,
                      FAR const struct nx_gpuop_s *ops, int nops)
{
  struct nxsvrmsg_gpuops_s msg;

  if (hwnd == NULL || nops < 0 || nops > NX_GPUOPS_MAX ||
      (nops > 0 && ops == NULL))
    {
      set_errno(EINVAL);
      return ERROR;
    }

  msg.msgid = msgid;
  msg.wnd   = (FAR struct nxbe_window_s *)hwnd;
  msg.ops   = ops;
  msg.nops  = nops;
  return nx_compose_wait(NULL, msg.wnd, &msg, sizeof(msg), &msg.sem_done);
}

int nx_setdisplaylist(NXWINDOW hwnd, FAR const struct nx_gpuop_s *ops,
                      int nops)
{
  return nx_sendops(hwnd, NX_SVRMSG_SETDLIST, ops, nops);
}

int nx_gpuops(NXWINDOW hwnd, FAR const struct nx_gpuop_s *ops, int nops)
{
  return nx_sendops(hwnd, NX_SVRMSG_GPUOPS, ops, nops);
}

int nx_setlut(NXHANDLE handle, int slot,
              FAR const uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N])
{
  struct nxsvrmsg_setlut_s msg;

  if (handle == NULL || lut == NULL || slot < NX_LUT_CLIENT0 ||
      slot >= NX_LUT_RING)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  msg.msgid = NX_SVRMSG_SETLUT;
  msg.slot  = slot;
  msg.lut   = &lut[0][0];
  return nx_compose_wait((FAR struct nxmu_conn_s *)handle, NULL, &msg,
                         sizeof(msg), &msg.sem_done);
}
