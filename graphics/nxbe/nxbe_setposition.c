/****************************************************************************
 * graphics/nxbe/nxbe_setposition.c
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

#include <assert.h>

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"
#include "nxmu.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_NX_SWCURSOR
/****************************************************************************
 * Name: nxbe_redrawminus
 *
 * Description:
 *   Redraw, from wnd down, the part of rect1 that is not in rect2 (up to
 *   four rectangles).
 *
 ****************************************************************************/

static void nxbe_redrawminus(FAR struct nxbe_window_s *wnd,
                             FAR const struct nxgl_rect_s *rect1,
                             FAR const struct nxgl_rect_s *rect2)
{
  struct nxgl_rect_s part[4];
  int i;

  if (nxgl_nullrect(rect1))
    {
      return;
    }

  if (nxgl_nullrect(rect2) ||
      !nxgl_rectoverlap((FAR struct nxgl_rect_s *)rect1,
                        (FAR struct nxgl_rect_s *)rect2))
    {
      nxbe_redrawbelow(wnd->be, wnd, rect1);
      return;
    }

  nxgl_nonintersecting(part, rect1, rect2);
  for (i = 0; i < 4; i++)
    {
      if (!nxgl_nullrect(&part[i]))
        {
          nxbe_redrawbelow(wnd->be, wnd, &part[i]);
        }
    }
}

/****************************************************************************
 * Name: nxbe_movetop
 *
 * Description:
 *   Move a window that nothing covers: its pixels are on the display
 *   already, so move them there (one moverectangle, a single command for
 *   display hardware, CONFIG_NX_ACCEL) and redraw only what that leaves:
 *   the window where it was off the display, and what it uncovered.
 *   Otherwise a move redraws the union of the old and new areas, the
 *   window and everything below it, at every step of a drag.
 *
 ****************************************************************************/

static void nxbe_movetop(FAR struct nxbe_window_s *wnd,
                         FAR const struct nxgl_rect_s *before)
{
  FAR struct nxbe_state_s *be = wnd->be;
  struct nxgl_point_s offset;
  struct nxgl_rect_s oldvis;
  struct nxgl_rect_s newvis;
  struct nxgl_rect_s src;
  struct nxgl_rect_s dst;
  int i;

  offset.x = wnd->bounds.pt1.x - before->pt1.x;
  offset.y = wnd->bounds.pt1.y - before->pt1.y;

  nxgl_rectintersect(&oldvis, before, &be->bkgd.bounds);
  nxgl_rectintersect(&newvis, &wnd->bounds, &be->bkgd.bounds);

  /* What was on the display before and still is after, where it was */

  nxgl_rectoffset(&src, &newvis, -offset.x, -offset.y);
  nxgl_rectintersect(&src, &src, &oldvis);

  if (nxgl_nullrect(&src))
    {
      dst = src;
    }
  else
    {
      nxgl_rectoffset(&dst, &src, offset.x, offset.y);
      for (i = 0; i < be->vinfo.nplanes; i++)
        {
          FAR struct nxbe_plane_s *plane = &be->plane[i];

          plane->dev.moverectangle(&plane->pinfo, &src, &offset);
#ifdef CONFIG_NX_UPDATE
          nxbe_notify_rectangle(plane->driver, &dst);
#endif
        }
    }

  /* The window where the move did not bring its pixels, then what the
   * window uncovered (from the windows below and the background).
   */

  nxbe_redrawminus(wnd, &newvis, &dst);
  nxbe_redrawminus(wnd, &oldvis, &newvis);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_setposition
 *
 * Description:
 *   This function checks for intersections and redraws the display after
 *   a change in the position of a window.
 *
 ****************************************************************************/

void nxbe_setposition(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_point_s *pos)
{
  struct nxgl_rect_s before;
  struct nxgl_rect_s rect;

  DEBUGASSERT(wnd != NULL && pos != NULL);

  /* Back out the old window origin position from the bounding box */

  nxgl_rectoffset(&rect,
                  &wnd->bounds,
                  -wnd->bounds.pt1.x,
                  -wnd->bounds.pt1.y);

  /* Add the new window origin into the bounding box */

  nxgl_rectcopy(&before, &wnd->bounds);
  nxgl_rectoffset(&wnd->bounds, &rect, pos->x, pos->y);

  /* Report the new size/position */

  nxmu_reportposition(wnd);

#ifndef CONFIG_NX_SWCURSOR
  /* A window on top of all others (the one being dragged, usually) moves
   * what is already on the display.  (A software cursor would be moved
   * with it, so not then.)
   */

  if (wnd->above == NULL && !NXBE_ISHIDDEN(wnd))
    {
      nxbe_movetop(wnd, &before);
      return;
    }
#endif

  /* Get the union of the 'before' bounding box and the 'after' bounding
   * this union is the region of the display that must be updated.
   */

  nxgl_rectunion(&rect, &before, &wnd->bounds);
  nxgl_rectintersect(&rect, &rect, &wnd->be->bkgd.bounds);

  /* Then redraw this window AND all windows below it. Having moved the
   * window, we may have exposed previously obscured portions of windows
   * below this one.
   */

  nxbe_redrawbelow(wnd->be, wnd, &rect);
}
