/****************************************************************************
 * graphics/nxmu/nxmu_server.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <mqueue.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <time.h>
#include <nuttx/debug.h>

#include <nuttx/clock.h>
#include <nuttx/mqueue.h>
#include <nuttx/signal.h>
#include <nuttx/nx/nx.h>

#include "nxmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* While messages are held for clients (nxmu_sendclient.c), the longest the
 * server waits for a message of its own before trying them again.
 */

#define NXMU_FLUSH_MS 10

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_accel_idle
 *
 * Description:
 *   If no message is waiting, let the display hardware have what it has
 *   batched: a burst of messages goes out together, and nothing is left
 *   behind once the burst ends.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_ACCEL
static void nxmu_accel_idle(mqd_t mqd)
{
  struct mq_attr attr;
  bool idle = mq_getattr(mqd, &attr) < 0 || attr.mq_curmsgs == 0;

#ifdef CONFIG_NX_COMPOSITOR
  /* Composite what has changed: when nothing more is coming, or when it
   * has waited long enough in a stream of messages; and not before the
   * frame is due (nxbe_compose_wait())
   */

  if ((idle || nxbe_compose_due()) && nxbe_compose_wait() == 0)
    {
      nxbe_compose_run();
      idle = true;
    }
#endif

  if (idle)
    {
      nxbe_accel_flush();
    }
}
#endif

/****************************************************************************
 * Name: nxmu_disconnect
 ****************************************************************************/

static inline void nxmu_disconnect(FAR struct nxmu_conn_s *conn)
{
  struct nxclimsg_disconnected_s outmsg;
  int retries;
  int ret;

  /* Nothing held for it goes out now: the client frees the connection
   * once it has the handshake below.
   */

  nxmu_dropclient(conn);

  /* Send the handshake message back to the client.  It is waiting for it,
   * reading its MQ, so room comes; give up after a second (a client that
   * has gone without reading).
   */

  outmsg.msgid = NX_CLIMSG_DISCONNECTED;

  for (retries = 0; retries < 1000; retries++)
    {
      ret = nxmq_send(conn->swrmq, (FAR const char *)&outmsg,
                      sizeof(struct nxclimsg_disconnected_s),
                      NX_CLIMSG_PRIO);
      if (ret != -EAGAIN)
        {
          break;
        }

      nxsig_usleep(1000);
    }

  if (ret < 0)
    {
      gerr("ERROR: nxmq_send failed: %d\n", ret);
    }

  /* Close the outgoing client message queue */

  nxmq_close(conn->swrmq);
}

/****************************************************************************
 * Name: nxmu_connect
 ****************************************************************************/

static inline void nxmu_connect(FAR struct nxmu_conn_s *conn)
{
  char mqname[NX_CLIENT_MXNAMELEN];
  struct nxclimsg_connected_s outmsg;
  int ret;

  /* Create the client MQ name */

  snprintf(mqname, sizeof(mqname), NX_CLIENT_MQNAMEFMT, conn->cid);

  /* Open the client MQ -- this should have already been created by the
   * client
   */

  conn->swrmq = nxmq_open(mqname, O_WRONLY | O_NONBLOCK);
  if (conn->swrmq < 0)
    {
      gerr("ERROR: nxmq_open(%s) failed: %d\n", mqname, conn->swrmq);
      outmsg.msgid = NX_CLIMSG_DISCONNECTED;
    }

  /* Send the handshake message back to the client */

  outmsg.msgid = NX_CLIMSG_CONNECTED;
  ret = nxmu_sendclient(conn, &outmsg, sizeof(struct nxclimsg_connected_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendclient failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: nxmu_shutdown
 ****************************************************************************/

static inline void nxmu_shutdown(FAR struct nxmu_state_s *nxmu)
{
  FAR struct nxbe_window_s *wnd;

  /* Inform all of the clients in the window list that the server is
   * exit-ting.  Notes: (1) that the following loop will probably attempt to
   * disconnect clients multiple times because one client may have multiple
   * windows:  The first disconnect will fail; subsequent will return errors
   * that are ignored.  (2) The final window to be disconnected will be the
   * background window, thus close all of the servers message queues.
   */

  for (wnd = nxmu->be.topwnd; wnd; wnd = wnd->below)
    {
       nxmu_disconnect(wnd->conn);
    }
}

/****************************************************************************
 * Name: nxmu_event
 ****************************************************************************/

static void nxmu_event(FAR struct nxbe_window_s *wnd, enum nx_event_e event,
                       FAR void *arg)
{
  struct nxclimsg_event_s outmsg;
  int ret;

  outmsg.msgid = NX_CLIMSG_EVENT;
  outmsg.wnd   = wnd;
  outmsg.arg   = arg;
  outmsg.event = event;

  ret = nxmu_sendclient(wnd->conn, &outmsg,
                        sizeof(struct nxclimsg_event_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendclient failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: nxmu_displaymode
 *
 * Description:
 *   Get or change the display mode (nx_setdisplaymode()).  A change: the
 *   compositor gives up its second buffer, the hardware changes mode, the
 *   surfaces the display now covers move elsewhere, the display's size
 *   (vinfo, the background, the pointer's range) follows, all of it is
 *   composited again, and the clients are told: every window's new
 *   position report carries the display's bounds, and one window of each
 *   client gets NXEVENT_DISPLAYMODE.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_COMPOSITOR
static int nxmu_displaymode(FAR struct nxmu_state_s *nxmu, int mode,
                            bool set, FAR struct nxgl_size_s *size)
{
  FAR struct nxbe_state_s *be = &nxmu->be;
  FAR struct nxmu_conn_s *told[8];
  FAR struct nxbe_window_s *wnd;
  struct nxgl_size_s cursize;
  int ntold = 0;
  int cur;
  int ret;
  int i;

  if (g_nxaccel == NULL || g_nxaccel->getmode == NULL)
    {
      return -ENOSYS;
    }

  ret = g_nxaccel->getmode(mode, size);
  if (ret < 0 || !set)
    {
      return ret;
    }

  mode = ret;
  cur  = g_nxaccel->getmode(-1, &cursize);
  if (mode == cur || g_nxaccel->setmode == NULL)
    {
      return mode == cur ? mode : -ENOSYS;
    }

  nxbe_compose_setmode(true);
  ret = g_nxaccel->setmode(mode);
  if (ret < 0)
    {
      nxbe_compose_setmode(false);
      return ret;
    }

  be->vinfo.xres        = size->w;
  be->vinfo.yres        = size->h;
  be->bkgd.bounds.pt2.x = size->w - 1;
  be->bkgd.bounds.pt2.y = size->h - 1;
#ifdef CONFIG_NX_RAMBACKED
  nxbe_surface_evacuate(&be->bkgd.bounds);
#endif
#ifdef CONFIG_NX_XYINPUT
  nxmu_mouserange(size->w, size->h);
#endif
  nxbe_compose_setmode(false);

  for (wnd = be->topwnd; wnd != NULL; wnd = wnd->below)
    {
      if (wnd == &be->bkgd)
        {
          continue;
        }

      nxmu_reportposition(wnd);
      for (i = 0; i < ntold && told[i] != wnd->conn; i++)
        {
        }

      if (i == ntold)
        {
          nxmu_event(wnd, NXEVENT_DISPLAYMODE, (FAR void *)(uintptr_t)mode);
          if (ntold < (int)(sizeof(told) / sizeof(told[0])))
            {
              told[ntold++] = wnd->conn;
            }
        }
    }

  return mode;
}
#endif

/****************************************************************************
 * Name: nxmu_setup
 ****************************************************************************/

static inline int nxmu_setup(FAR const char *mqname, FAR NX_DRIVERTYPE *dev,
                             FAR struct nxmu_state_s *nxmu)
{
  struct mq_attr attr;
  int            ret;

  memset(nxmu, 0, sizeof(struct nxmu_state_s));

  /* Configure the framebuffer/LCD device */

  ret = nxbe_configure(dev, &nxmu->be);
  if (ret < 0)
    {
      gerr("ERROR: nxbe_configure failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_FB_CMAP
  ret = nxbe_colormap(dev);
  if (ret < 0)
    {
      gerr("ERROR: nxbe_colormap failed: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_FB_CMAP */

  /* Initialize the non-NULL elements of the server connection structure.
   * Oddly, this structure represents the connection between the server and
   * itself.
   *
   * Open the incoming server MQ.  The server receives messages on the
   * background window's incoming message queue.
   */

  attr.mq_maxmsg  = CONFIG_NX_MXSERVERMSGS;
  attr.mq_msgsize = NX_MXSVRMSGLEN;
  attr.mq_flags   = 0;

  nxmu->conn.crdmq = nxmq_open(mqname, O_RDONLY | O_CREAT, 0666, &attr);
  if (nxmu->conn.crdmq < 0)
    {
      gerr("ERROR: nxmq_open(%s) failed: %d\n", mqname, nxmu->conn.crdmq);
      return nxmu->conn.crdmq;
    }

  /* NOTE that the outgoing client MQ (cwrmq) is not initialized.  The
   * background window never initiates messages.
   */

  /* Open the outgoing server MQ.  This is used to send messages to the
   * background window which will, of course, be received and handled by
   * the server message loop.
   */

  nxmu->conn.swrmq = nxmq_open(mqname, O_WRONLY | O_NONBLOCK);
  if (nxmu->conn.swrmq < 0)
    {
      gerr("ERROR: nxmq_open(%s) failed: %d\n", mqname, nxmu->conn.swrmq);
      nxmq_close(nxmu->conn.crdmq);
      return nxmu->conn.swrmq;
    }

  /* The server is now "connected" to itself via the background window */

  nxmu->conn.state = NX_CLISTATE_CONNECTED;

  /* Initialize the non-NULL elements of the background window */

  nxmu->be.bkgd.conn = &nxmu->conn;
  nxmu->be.bkgd.be   = (FAR struct nxbe_state_s *)nxmu;

  nxmu->be.bkgd.bounds.pt2.x = nxmu->be.vinfo.xres - 1;
  nxmu->be.bkgd.bounds.pt2.y = nxmu->be.vinfo.yres - 1;

  /* Complete initialization of the server state structure.  The
   * window list contains only one element:  The background window
   * with nothing else above or below it
   */

  nxmu->be.topwnd = &nxmu->be.bkgd;

  /* Initialize the mouse position */

#ifdef CONFIG_NX_XYINPUT
  nxmu_mouseinit(nxmu->be.vinfo.xres, nxmu->be.vinfo.yres);
#endif
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_runinstance
 *
 * Description:
 *   This is the server entry point.  It does not return; the calling thread
 *   is dedicated to supporting NX server.
 *
 *   NOTE that multiple instances of the NX server may run at the same time,
 *   each with different callback and message queue names.
 *
 * Input Parameters:
 *   mqname - The name for the server incoming message queue
 *   dev    - Vtable "object" of the framebuffer/LCD "driver" to use
 *
 * Returned Value:
 *   This function usually does not return.  If it does return, it will
 *   return a negated errno value indicating the cause of the failure.
 *
 ****************************************************************************/

int nx_runinstance(FAR const char *mqname, FAR NX_DRIVERTYPE *dev)
{
  struct nxmu_state_s    nxmu;
  FAR struct nxsvrmsg_s *msg;
  char                   buffer[NX_MXSVRMSGLEN];
  int                    nbytes;
  int                    waitms;
#ifdef CONFIG_NX_COMPOSITOR
  int                    ticks;
#endif
  int                    ret;

  /* Initialization *********************************************************/

  DEBUGASSERT(mqname != NULL || dev != NULL);

  /* Initialize and configure the server */

  ret = nxmu_setup(mqname, dev, &nxmu);
  if (ret < 0)
    {
      return ret;
    }

  /* Produce the initial, background display */

  nxbe_redraw(&nxmu.be, &nxmu.be.bkgd, &nxmu.be.bkgd.bounds);

  /* Message Loop ***********************************************************/

  /* Then loop forever processing incoming messages */

  for (; ; )
    {
#ifdef CONFIG_NX_ACCEL
      nxmu_accel_idle(nxmu.conn.crdmq);
#endif

      /* Send what clients had no room for, then receive the next server
       * message; while anything is still held, wait no more than
       * NXMU_FLUSH_MS for one, so it goes out as room comes (nor longer
       * than a frame of the compositor's has to wait).
       */

      waitms = nxmu_flushclients() ? NXMU_FLUSH_MS : -1;

#ifdef CONFIG_NX_COMPOSITOR
      /* A frame waiting for its time: no longer than that */

      ticks = nxbe_compose_wait();
      if (ticks > 0 && (waitms < 0 || TICK2MSEC(ticks) < waitms))
        {
          waitms = TICK2MSEC(ticks);
        }
#endif

      if (waitms >= 0)
        {
          struct timespec abstime;

          clock_gettime(CLOCK_REALTIME, &abstime);
          abstime.tv_nsec += waitms * 1000000;
          if (abstime.tv_nsec >= 1000000000)
            {
              abstime.tv_sec++;
              abstime.tv_nsec -= 1000000000;
            }

          nbytes = nxmq_timedreceive(nxmu.conn.crdmq, buffer,
                                     NX_MXSVRMSGLEN, 0, &abstime);
          if (nbytes == -ETIMEDOUT)
            {
              continue;
            }
        }
      else
        {
          nbytes = nxmq_receive(nxmu.conn.crdmq, buffer, NX_MXSVRMSGLEN,
                                0);
        }

      if (nbytes < 0)
        {
          if (nbytes != -EINTR)
            {
              gerr("ERROR: nxmq_receive() failed: %d\n", nbytes);
              ret = nbytes;
              goto errout;
            }

          continue;
        }

      /* Dispatch the message appropriately */

      DEBUGASSERT(nbytes >= sizeof(struct nxsvrmsg_releasebkgd_s));
      msg = (FAR struct nxsvrmsg_s *)buffer;

      ginfo("Received opcode=%" PRId32 " nbytes=%d\n", msg->msgid, nbytes);
      switch (msg->msgid)
        {
          /* Messages sent from clients to the NX server ********************/

          case NX_SVRMSG_CONNECT: /* Establish connection with new NX server client */
            {
              FAR struct nxsvrmsg_s *connmsg =
                (FAR struct nxsvrmsg_s *)buffer;
              nxmu_connect(connmsg->conn);
            }
            break;

          case NX_SVRMSG_DISCONNECT: /* Tear down connection with terminating client */
            {
              FAR struct nxsvrmsg_s *disconnmsg =
                (FAR struct nxsvrmsg_s *)buffer;
              nxmu_disconnect(disconnmsg->conn);
            }
            break;

          case NX_SVRMSG_OPENWINDOW: /* Create a new window */
            {
              FAR struct nxsvrmsg_openwindow_s *openmsg =
                (FAR struct nxsvrmsg_openwindow_s *)buffer;
              nxmu_openwindow(&nxmu.be, openmsg->wnd);
            }
            break;

          case NX_SVRMSG_CLOSEWINDOW: /* Close an existing window */
            {
              FAR struct nxsvrmsg_closewindow_s *closemsg =
                (FAR struct nxsvrmsg_closewindow_s *)buffer;
              nxbe_closewindow(closemsg->wnd);
            }
            break;

          case NX_SVRMSG_BLOCKED: /* Block messages to a window */
            {
              FAR struct nxsvrmsg_blocked_s *blocked =
                (FAR struct nxsvrmsg_blocked_s *)buffer;
              nxmu_event(blocked->wnd, NXEVENT_BLOCKED, blocked->arg);
            }
            break;

          case NX_SVRMSG_SYNCH: /* Synchronization request */
            {
              FAR struct nxsvrmsg_synch_s *synch =
                (FAR struct nxsvrmsg_synch_s *)buffer;
              nxmu_event(synch->wnd, NXEVENT_SYNCHED, synch->arg);
            }
            break;

#if defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)
          case NX_SVRMSG_CURSOR_ENABLE: /* Enable/disable cursor */
            {
              FAR struct nxsvrmsg_curenable_s *enabmsg =
                (FAR struct nxsvrmsg_curenable_s *)buffer;
              nxbe_cursor_enable(&nxmu.be, enabmsg->enable);
            }
            break;

#if defined(CONFIG_NX_HWCURSORIMAGE) || defined(CONFIG_NX_SWCURSOR)
          case NX_SVRMSG_CURSOR_IMAGE: /* Set cursor image */
            {
              FAR struct nxsvrmsg_curimage_s *imgmsg =
                (FAR struct nxsvrmsg_curimage_s *)buffer;
              nxbe_cursor_setimage(&nxmu.be, &imgmsg->image);
            }
            break;
#endif
          case NX_SVRMSG_CURSOR_SETPOS: /* Set cursor position */
            {
              FAR struct nxsvrmsg_curpos_s *posmsg =
                (FAR struct nxsvrmsg_curpos_s *)buffer;
              nxbe_cursor_setposition(&nxmu.be, &posmsg->pos);
            }
            break;
#endif

          case NX_SVRMSG_REQUESTBKGD: /* Give access to the background window */
            {
              FAR struct nxsvrmsg_requestbkgd_s *rqbgmsg =
                (FAR struct nxsvrmsg_requestbkgd_s *)buffer;
              nxmu_requestbkgd(rqbgmsg->conn, &nxmu.be, rqbgmsg->cb,
                               rqbgmsg->arg);
            }
            break;

          case NX_SVRMSG_RELEASEBKGD: /* End access to the background window */
            {
              nxmu_releasebkgd(&nxmu);
            }
            break;

          case NX_SVRMSG_SETPOSITION: /* Change window position */
            {
              FAR struct nxsvrmsg_setposition_s *setposmsg =
                (FAR struct nxsvrmsg_setposition_s *)buffer;
              nxbe_setposition(setposmsg->wnd, &setposmsg->pos);
            }
            break;

          case NX_SVRMSG_SETSIZE: /* Change window size */
            {
              FAR struct nxsvrmsg_setsize_s *setsizemsg =
                (FAR struct nxsvrmsg_setsize_s *)buffer;
              nxbe_setsize(setsizemsg->wnd, &setsizemsg->size);
            }
            break;

          case NX_SVRMSG_GETPOSITION: /* Get the window size/position */
            {
              FAR struct nxsvrmsg_getposition_s *getposmsg =
                (FAR struct nxsvrmsg_getposition_s *)buffer;
              nxmu_reportposition(getposmsg->wnd);
            }
            break;

          case NX_SVRMSG_RAISE: /* Move the window to the top of the display */
            {
              FAR struct nxsvrmsg_raise_s *raisemsg =
                (FAR struct nxsvrmsg_raise_s *)buffer;
              nxbe_raise(raisemsg->wnd);
            }
            break;

          case NX_SVRMSG_LOWER: /* Lower the window to the bottom of the display */
            {
              FAR struct nxsvrmsg_lower_s *lowermsg =
                (FAR struct nxsvrmsg_lower_s *)buffer;
              nxbe_lower(lowermsg->wnd);
            }
            break;

          case NX_SVRMSG_MODAL: /* Select/De-select window modal state */
            {
              FAR struct nxsvrmsg_modal_s *modalmsg =
                (FAR struct nxsvrmsg_modal_s *)buffer;
              nxbe_modal(modalmsg->wnd, modalmsg->modal);
            }
            break;

          case NX_SVRMSG_SETVISIBILITY: /* Show or hide a window */
            {
              FAR struct nxsvrmsg_setvisibility_s *vismsg =
               (FAR struct nxsvrmsg_setvisibility_s *)buffer;
              nxbe_setvisibility(vismsg->wnd, vismsg->hide);
            }
            break;

          case NX_SVRMSG_SETPIXEL: /* Set a single pixel in the window with a color */
            {
              FAR struct nxsvrmsg_setpixel_s *setmsg =
                (FAR struct nxsvrmsg_setpixel_s *)buffer;
              nxbe_setpixel(setmsg->wnd, &setmsg->pos, setmsg->color);
            }
            break;

          case NX_SVRMSG_FILL: /* Fill a rectangular region in the window with a color */
            {
              FAR struct nxsvrmsg_fill_s *fillmsg =
                (FAR struct nxsvrmsg_fill_s *)buffer;
              nxbe_fill(fillmsg->wnd, &fillmsg->rect, fillmsg->color);
            }
            break;

          case NX_SVRMSG_GETRECTANGLE: /* Get a rectangular region from the window */
            {
              FAR struct nxsvrmsg_getrectangle_s *getmsg =
                (FAR struct nxsvrmsg_getrectangle_s *)buffer;
              nxbe_getrectangle(getmsg->wnd, &getmsg->rect, getmsg->plane,
                               getmsg->dest, getmsg->deststride);

              if (getmsg->sem_done)
                {
                  nxsem_post(getmsg->sem_done);
                }
            }
            break;

          case NX_SVRMSG_FILLTRAP: /* Fill a trapezoidal region in the window with a color */
            {
              FAR struct nxsvrmsg_filltrapezoid_s *trapmsg =
                (FAR struct nxsvrmsg_filltrapezoid_s *)buffer;
              nxbe_filltrapezoid(trapmsg->wnd, &trapmsg->clip,
                                 &trapmsg->trap, trapmsg->color);
            }
            break;

          case NX_SVRMSG_MOVE: /* Move a rectangular region within the window */
            {
              FAR struct nxsvrmsg_move_s *movemsg =
                (FAR struct nxsvrmsg_move_s *)buffer;
              nxbe_move(movemsg->wnd, &movemsg->rect, &movemsg->offset);
            }
            break;

          case NX_SVRMSG_BITMAP: /* Copy a rectangular bitmap into the window */
            {
              FAR struct nxsvrmsg_bitmap_s *bmpmsg =
                (FAR struct nxsvrmsg_bitmap_s *)buffer;
              nxbe_bitmap(bmpmsg->wnd, &bmpmsg->dest, bmpmsg->src,
                          &bmpmsg->origin, bmpmsg->stride);

              if (bmpmsg->sem_done)
                {
                  nxsem_post(bmpmsg->sem_done);
                }
            }
            break;

          case NX_SVRMSG_SETBGCOLOR: /* Set the color of the background */
            {
              FAR struct nxsvrmsg_setbgcolor_s *bgcolormsg =
                (FAR struct nxsvrmsg_setbgcolor_s *)buffer;

              /* Has the background color changed? */

              if (!nxgl_colorcmp(nxmu.be.bgcolor, bgcolormsg->color))
                {
                  /* Yes.. fill the background */

                  nxgl_colorcopy(nxmu.be.bgcolor, bgcolormsg->color);
                  nxbe_fill(&nxmu.be.bkgd, &nxmu.be.bkgd.bounds,
                            bgcolormsg->color);
                }
            }
            break;

#ifdef CONFIG_NX_XYINPUT
          case NX_SVRMSG_MOUSEIN: /* New mouse report from mouse client */
            {
              FAR struct nxsvrmsg_mousein_s *mousemsg =
                (FAR struct nxsvrmsg_mousein_s *)buffer;
              nxmu_mousein(&nxmu, &mousemsg->pt, mousemsg->buttons);
            }
            break;
#endif
#ifdef CONFIG_NX_KBD
          case NX_SVRMSG_KBDIN: /* New keyboard report from keyboard client */
            {
              FAR struct nxsvrmsg_kbdin_s *kbdmsg =
                (FAR struct nxsvrmsg_kbdin_s *)buffer;
              nxmu_kbdin(&nxmu, kbdmsg->nch, kbdmsg->ch);
            }
            break;

          case NX_SVRMSG_KBDMODE: /* Bytes or key events for a client */
            {
              FAR struct nxsvrmsg_kbdmode_s *modemsg =
                (FAR struct nxsvrmsg_kbdmode_s *)buffer;
              modemsg->conn->kbdmode = modemsg->mode;
            }
            break;
#endif

          case NX_SVRMSG_REDRAWREQ: /* Request re-drawing of rectangular region */
            {
              FAR struct nxsvrmsg_redrawreq_s *redrawmsg =
                (FAR struct nxsvrmsg_redrawreq_s *)buffer;
              nxmu_redraw(redrawmsg->wnd, &redrawmsg->rect);
            }
            break;

#ifdef CONFIG_NX_COMPOSITOR
          case NX_SVRMSG_COMPOSITOR: /* Compositor capabilities */
            {
              FAR struct nxsvrmsg_compositor_s *compmsg =
                (FAR struct nxsvrmsg_compositor_s *)buffer;
              *compmsg->caps = nxbe_compose_on() ? NX_CAP_COMPOSITOR : 0;
              compmsg->size->w = nxmu.be.vinfo.xres;
              compmsg->size->h = nxmu.be.vinfo.yres;
              nxsem_post(compmsg->sem_done);
            }
            break;

          case NX_SVRMSG_SETEFFECTS: /* Window effects and layer */
            {
              FAR struct nxsvrmsg_seteffects_s *fxmsg =
                (FAR struct nxsvrmsg_seteffects_s *)buffer;
              nxbe_compose_seteffects(fxmsg->wnd, &fxmsg->fx);
            }
            break;

          case NX_SVRMSG_SETDLIST: /* A layer window's operations */
          case NX_SVRMSG_GPUOPS:   /* Operations into a window */
            {
              FAR struct nxsvrmsg_gpuops_s *opsmsg =
                (FAR struct nxsvrmsg_gpuops_s *)buffer;
              if (msg->msgid == NX_SVRMSG_SETDLIST)
                {
                  nxbe_compose_setdlist(opsmsg->wnd, opsmsg->ops,
                                        opsmsg->nops);
                }
              else
                {
                  nxbe_compose_gpuops(opsmsg->wnd, opsmsg->ops,
                                      opsmsg->nops);
                }

              nxsem_post(opsmsg->sem_done);
            }
            break;

          case NX_SVRMSG_DISPLAYMODE: /* Get or set the display mode */
            {
              FAR struct nxsvrmsg_displaymode_s *modemsg =
                (FAR struct nxsvrmsg_displaymode_s *)buffer;
              *modemsg->result = nxmu_displaymode(&nxmu, modemsg->mode,
                                                  modemsg->set,
                                                  modemsg->size);
              nxsem_post(modemsg->sem_done);
            }
            break;

          case NX_SVRMSG_SURFACE: /* Where a window's pixels are */
            {
              FAR struct nxsvrmsg_surface_s *surfmsg =
                (FAR struct nxsvrmsg_surface_s *)buffer;
#if defined(CONFIG_NX_RAMBACKED) && defined(CONFIG_NX_ACCEL)
              *surfmsg->result = surfmsg->wnd->surface ? 1 : 0;
#else
              *surfmsg->result = 0;
#endif
              nxsem_post(surfmsg->sem_done);
            }
            break;

          case NX_SVRMSG_SETLUT: /* Load a shape table */
            {
              FAR struct nxsvrmsg_setlut_s *lutmsg =
                (FAR struct nxsvrmsg_setlut_s *)buffer;
              if (nxbe_compose_on())
                {
                  nxbe_compose_setlut(lutmsg->slot,
                                      (FAR const uint8_t (*)[NX_BLEND_LUT_N])
                                      lutmsg->lut);
                }

              nxsem_post(lutmsg->sem_done);
            }
            break;
#endif

          /* Messages sent to the background window *************************/

          case NX_CLIMSG_REDRAW: /* Re-draw the background window */
            {
              FAR struct nxclimsg_redraw_s *redraw =
                (FAR struct nxclimsg_redraw_s *)buffer;
              DEBUGASSERT(redraw->wnd == &nxmu.be.bkgd);
              ginfo("Re-draw background rect={(%d,%d),(%d,%d)}\n",
                    redraw->rect.pt1.x, redraw->rect.pt1.y,
                    redraw->rect.pt2.x, redraw->rect.pt2.y);
              nxbe_fill(&nxmu.be.bkgd, &redraw->rect, nxmu.be.bgcolor);
            }
            break;

          case NX_CLIMSG_MOUSEIN:      /* Ignored */
          case NX_CLIMSG_KBDIN:
            break;

          case NX_CLIMSG_CONNECTED:    /* Shouldn't happen */
          case NX_CLIMSG_DISCONNECTED:
          default:
            gerr("ERROR: Unrecognized command: %" PRId32 "\n", msg->msgid);
            break;
        }
    }

  nxmu_shutdown(&nxmu);
  return OK;

errout:
  nxmu_shutdown(&nxmu);
  return ret;
}
