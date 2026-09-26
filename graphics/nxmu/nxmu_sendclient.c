/****************************************************************************
 * graphics/nxmu/nxmu_sendclient.c
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

#include <mqueue.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include <nuttx/mqueue.h>

#include "nxmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A client that stops reading altogether (hung, or gone without
 * disconnecting) must not take all memory: past this many held messages,
 * new ones for it are dropped.
 */

#define NXMU_MXPENDING 1024

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A message held for a client whose MQ was full */

struct nxmu_pending_s
{
  sq_entry_t link;
  uint8_t    len;
  uint8_t    msg[NX_MXCLIMSGLEN];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The connections that have held messages.  Only server threads touch
 * them; the lock is for more than one server (display).
 */

static mutex_t g_pendlock = NXMUTEX_INITIALIZER;
static FAR struct nxmu_conn_s *g_pendconns;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Send the connection's held messages, oldest first, until its MQ is full.
 * Called with g_pendlock held; returns true if some are still held.
 */

static bool nxmu_flushconn(FAR struct nxmu_conn_s *conn)
{
  FAR struct nxmu_pending_s *p;
  int ret;

  while ((p = (FAR struct nxmu_pending_s *)sq_peek(&conn->pending)) != NULL)
    {
      ret = nxmq_send(conn->swrmq, (FAR const char *)p->msg, p->len,
                      NX_CLIMSG_PRIO);
      if (ret == -EAGAIN)
        {
          return true;
        }

      if (ret < 0)
        {
          gerr("ERROR: nxmq_send failed: %d\n", ret);
        }

      sq_remfirst(&conn->pending);
      conn->npending--;
      kmm_free(p);
    }

  return false;
}

/* Take the connection off the list of those with held messages */

static void nxmu_unlist(FAR struct nxmu_conn_s *conn)
{
  FAR struct nxmu_conn_s **pp;

  for (pp = &g_pendconns; *pp != NULL; pp = &(*pp)->pnext)
    {
      if (*pp == conn)
        {
          *pp = conn->pnext;
          break;
        }
    }

  conn->pnext  = NULL;
  conn->listed = false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_sendclient
 *
 * Description:
 *  Send a message to the client at NX_CLIMSG_PRIO priority, without
 *  waiting: if the client's MQ is full (or messages for it are already
 *  held, which must go first), the message is held and sent later by
 *  nxmu_flushclients().  A server that waited here would deadlock with a
 *  client that is itself waiting for room in the server's MQ (a client
 *  that draws from its event callbacks, while the server floods it with
 *  redraws and mouse events during a window drag).
 *
 * Input Parameters:
 *   conn   - A pointer to the server connection structure
 *   msg    - A pointer to the message to send
 *   msglen - The length of the message in bytes.
 *
 * Returned Value:
 *   OK on success (sent or held); a negated errno value on failure.
 *
 ****************************************************************************/

int nxmu_sendclient(FAR struct nxmu_conn_s *conn, FAR const void *msg,
                    size_t msglen)
{
  FAR struct nxmu_pending_s *p;
  int ret;

  DEBUGASSERT(conn != NULL && msg != NULL && msglen <= NX_MXCLIMSGLEN);

  nxmutex_lock(&g_pendlock);
  if (conn->npending == 0 || !nxmu_flushconn(conn))
    {
      ret = nxmq_send(conn->swrmq, msg, msglen, NX_CLIMSG_PRIO);
      if (ret != -EAGAIN)
        {
          nxmutex_unlock(&g_pendlock);
          if (ret < 0)
            {
              gerr("ERROR: nxmq_send failed: %d\n", ret);
            }

          return ret;
        }
    }

  /* No room: hold it */

  if (conn->npending >= NXMU_MXPENDING)
    {
      nxmutex_unlock(&g_pendlock);
      gerr("ERROR: client %d is not reading, message dropped\n",
           conn->cid);
      return -EAGAIN;
    }

  p = kmm_malloc(sizeof(struct nxmu_pending_s));
  if (p == NULL)
    {
      nxmutex_unlock(&g_pendlock);
      return -ENOMEM;
    }

  p->len = msglen;
  memcpy(p->msg, msg, msglen);
  sq_addlast(&p->link, &conn->pending);
  conn->npending++;

  if (!conn->listed)
    {
      conn->pnext  = g_pendconns;
      conn->listed = true;
      g_pendconns  = conn;
    }

  nxmutex_unlock(&g_pendlock);
  return OK;
}

/****************************************************************************
 * Name: nxmu_flushclients
 *
 * Description:
 *   Send what is held for every client, as far as their MQs have room.
 *   Called by the server loop before it waits for the next message.
 *
 * Returned Value:
 *   true if messages are still held (the server should not wait long).
 *
 ****************************************************************************/

bool nxmu_flushclients(void)
{
  FAR struct nxmu_conn_s *conn;
  FAR struct nxmu_conn_s *next;
  bool held = false;

  nxmutex_lock(&g_pendlock);
  for (conn = g_pendconns; conn != NULL; conn = next)
    {
      next = conn->pnext;
      if (nxmu_flushconn(conn))
        {
          held = true;
        }
      else
        {
          nxmu_unlist(conn);
        }
    }

  nxmutex_unlock(&g_pendlock);
  return held;
}

/****************************************************************************
 * Name: nxmu_dropclient
 *
 * Description:
 *   Forget what is held for a client that is disconnecting (its connection
 *   structure is about to be freed).
 *
 ****************************************************************************/

void nxmu_dropclient(FAR struct nxmu_conn_s *conn)
{
  FAR sq_entry_t *p;

  nxmutex_lock(&g_pendlock);
  while ((p = sq_remfirst(&conn->pending)) != NULL)
    {
      kmm_free(p);
    }

  conn->npending = 0;
  if (conn->listed)
    {
      nxmu_unlist(conn);
    }

  nxmutex_unlock(&g_pendlock);
}
