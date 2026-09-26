/****************************************************************************
 * graphics/nxbe/nxbe_accel.c
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

#include <string.h>
#include <sys/param.h>
#include <fixedmath.h>

#include <nuttx/nx/nxaccel.h>

#include "nxbe.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The board's operations, once installed (one display) */

FAR const struct nx_accel_s *g_nxaccel;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NX_RAMBACKED
/* The per-window rasterizers for windows in memory, which the dispatchers
 * below fall back to for windows without a surface.
 */

static struct nxbe_pwfb_vtable_s g_pwfb_ram;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The device vtable entries (nxbe_dev_vtable_s), in terms of the board's */

static void accel_setpixel(FAR NX_PLANEINFOTYPE *pinfo,
                           FAR const struct nxgl_point_s *pos,
                           nxgl_mxpixel_t color)
{
  struct nxgl_rect_s rect;

  rect.pt1 = *pos;
  rect.pt2 = *pos;
  g_nxaccel->fill(&rect, color);
}

static void accel_fillrectangle(FAR NX_PLANEINFOTYPE *pinfo,
                                FAR const struct nxgl_rect_s *rect,
                                nxgl_mxpixel_t color)
{
  g_nxaccel->fill(rect, color);
}

static void accel_getrectangle(FAR NX_PLANEINFOTYPE *pinfo,
                               FAR const struct nxgl_rect_s *rect,
                               FAR void *dest, unsigned int deststride)
{
  FAR uint8_t *line = dest;
  unsigned int len;
  int y;

  if (g_nxaccel->get != NULL)
    {
      g_nxaccel->get(rect, dest, deststride);
      return;
    }

  len = ((rect->pt2.x - rect->pt1.x + 1) * pinfo->bpp + 7) >> 3;
  for (y = rect->pt1.y; y <= rect->pt2.y; y++, line += deststride)
    {
      memset(line, 0, len);
    }
}

/* A trapezoid as one fill per row: the same arithmetic as
 * nxglib_filltrapezoid.c, so the same pixels.  Rows with the same run are
 * merged into one fill (a rectangle is a trapezoid too).
 */

static void accel_filltrapezoid(FAR NX_PLANEINFOTYPE *pinfo,
                                FAR const struct nxgl_trapezoid_s *trap,
                                FAR const struct nxgl_rect_s *bounds,
                                nxgl_mxpixel_t color)
{
  struct nxgl_rect_s run;
  bool pending = false;
  b16_t x1;
  b16_t x2;
  b16_t dx1dy;
  b16_t dx2dy;
  nxgl_coord_t y1;
  nxgl_coord_t y2;
  nxgl_coord_t y;
  int nrows;

  x1    = trap->top.x1;
  x2    = trap->top.x2;
  y1    = trap->top.y;
  y2    = trap->bot.y;
  nrows = y2 - y1 + 1;

  if (nrows > 1)
    {
      dx1dy = b16divi((trap->bot.x1 - x1), nrows - 1);
      dx2dy = b16divi((trap->bot.x2 - x2), nrows - 1);
    }
  else
    {
      x1    = (x1 + trap->bot.x1) >> 1;
      x2    = (x2 + trap->bot.x2) >> 1;
      dx1dy = 0;
      dx2dy = 0;
    }

  if (y1 < bounds->pt1.y)
    {
      if (y2 < bounds->pt1.y)
        {
          return;
        }

      x1 += (bounds->pt1.y - y1) * dx1dy;
      x2 += (bounds->pt1.y - y1) * dx2dy;
      y1  = bounds->pt1.y;
    }

  if (y2 > bounds->pt2.y)
    {
      if (y1 > bounds->pt2.y)
        {
          return;
        }

      y2 = bounds->pt2.y;
    }

  for (y = y1; y <= y2; y++, x1 += dx1dy, x2 += dx2dy)
    {
      nxgl_coord_t ix1;
      nxgl_coord_t ix2;

      if (x1 > x2)
        {
          b16_t tmp;

          tmp = x1;
          x1 = x2;
          x2 = tmp;
          tmp = dx1dy;
          dx1dy = dx2dy;
          dx2dy = tmp;
        }

      ix1 = b16toi(x1);
      ix2 = b16toi(x2);
      if (ix2 < bounds->pt1.x || ix1 > bounds->pt2.x)
        {
          continue;
        }

      if (ix1 < bounds->pt1.x)
        {
          ix1 = bounds->pt1.x;
        }

      if (ix2 > bounds->pt2.x)
        {
          ix2 = bounds->pt2.x;
        }

      if (pending && ix1 == run.pt1.x && ix2 == run.pt2.x &&
          y == run.pt2.y + 1)
        {
          run.pt2.y = y;
          continue;
        }

      if (pending)
        {
          g_nxaccel->fill(&run, color);
        }

      run.pt1.x = ix1;
      run.pt2.x = ix2;
      run.pt1.y = y;
      run.pt2.y = y;
      pending   = true;
    }

  if (pending)
    {
      g_nxaccel->fill(&run, color);
    }
}

static void accel_moverectangle(FAR NX_PLANEINFOTYPE *pinfo,
                                FAR const struct nxgl_rect_s *rect,
                                FAR struct nxgl_point_s *offset)
{
  g_nxaccel->move(rect, offset);
}

static void accel_copyrectangle(FAR NX_PLANEINFOTYPE *pinfo,
                                FAR const struct nxgl_rect_s *dest,
                                FAR const void *src,
                                FAR const struct nxgl_point_s *origin,
                                unsigned int srcstride)
{
  g_nxaccel->copy(dest, src, origin, srcstride);
}

#ifdef CONFIG_NX_RAMBACKED
/* The per-window rasterizers (nxbe_pwfb_vtable_s): for a window with a
 * surface, the board's operations on the surface (window coordinates plus
 * the surface's position); otherwise the memory rasterizers.
 */

static void surf_rect(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rel,
                      FAR struct nxgl_rect_s *abs)
{
  nxgl_rectoffset(abs, rel, wnd->surfpos.x, wnd->surfpos.y);
}

static void pwfb_setpixel(FAR struct nxbe_window_s *wnd,
                          FAR const struct nxgl_point_s *pos,
                          nxgl_mxpixel_t color)
{
  struct nxgl_rect_s rect;

  if (!wnd->surface)
    {
      g_pwfb_ram.setpixel(wnd, pos, color);
      return;
    }

  rect.pt1.x = pos->x + wnd->surfpos.x;
  rect.pt1.y = pos->y + wnd->surfpos.y;
  rect.pt2   = rect.pt1;
  g_nxaccel->fill(&rect, color);
}

static void pwfb_fillrectangle(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *rect,
                               nxgl_mxpixel_t color)
{
  struct nxgl_rect_s abs;

  if (!wnd->surface)
    {
      g_pwfb_ram.fillrectangle(wnd, rect, color);
      return;
    }

  surf_rect(wnd, rect, &abs);
  g_nxaccel->fill(&abs, color);
}

static void pwfb_getrectangle(FAR struct nxbe_window_s *wnd,
                              FAR const struct nxgl_rect_s *rect,
                              FAR void *dest, unsigned int deststride)
{
  struct nxgl_rect_s abs;

  if (!wnd->surface)
    {
      g_pwfb_ram.getrectangle(wnd, rect, dest, deststride);
      return;
    }

  surf_rect(wnd, rect, &abs);
  accel_getrectangle(NULL, &abs, dest, deststride);
}

static void pwfb_filltrapezoid(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_trapezoid_s *trap,
                               FAR const struct nxgl_rect_s *bounds,
                               nxgl_mxpixel_t color)
{
  struct nxgl_trapezoid_s t;
  struct nxgl_rect_s b;

  if (!wnd->surface)
    {
      g_pwfb_ram.filltrapezoid(wnd, trap, bounds, color);
      return;
    }

  t = *trap;
  t.top.x1 += itob16(wnd->surfpos.x);
  t.top.x2 += itob16(wnd->surfpos.x);
  t.bot.x1 += itob16(wnd->surfpos.x);
  t.bot.x2 += itob16(wnd->surfpos.x);
  t.top.y  += wnd->surfpos.y;
  t.bot.y  += wnd->surfpos.y;
  surf_rect(wnd, bounds, &b);
  accel_filltrapezoid(NULL, &t, &b, color);
}

/* NB: the third argument is where the rectangle goes, not an offset */

static void pwfb_moverectangle(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *rect,
                               FAR struct nxgl_point_s *destpos)
{
  struct nxgl_point_s offset;
  struct nxgl_rect_s abs;

  if (!wnd->surface)
    {
      g_pwfb_ram.moverectangle(wnd, rect, destpos);
      return;
    }

  offset.x = destpos->x - rect->pt1.x;
  offset.y = destpos->y - rect->pt1.y;
  surf_rect(wnd, rect, &abs);
  g_nxaccel->move(&abs, &offset);
}

static void pwfb_copyrectangle(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *dest,
                               FAR const void *src,
                               FAR const struct nxgl_point_s *origin,
                               unsigned int srcstride)
{
  struct nxgl_point_s org;
  struct nxgl_rect_s abs;

  if (!wnd->surface)
    {
      g_pwfb_ram.copyrectangle(wnd, dest, src, origin, srcstride);
      return;
    }

  surf_rect(wnd, dest, &abs);
  org.x = origin->x + wnd->surfpos.x;
  org.y = origin->y + wnd->surfpos.y;
  g_nxaccel->copy(&abs, src, &org, srcstride);
}

/* Showing a surface: a move from it to each visible part of the display */

struct surf_present_s
{
  struct nxbe_clipops_s cops;
  FAR struct nxbe_window_s *wnd;
};

static void surf_present_visible(FAR struct nxbe_clipops_s *cops,
                                 FAR struct nxbe_plane_s *plane,
                                 FAR const struct nxgl_rect_s *rect)
{
  FAR struct nxbe_window_s *wnd = ((FAR struct surf_present_s *)cops)->wnd;
  struct nxgl_point_s offset;
  struct nxgl_rect_s src;

  /* rect is on the display; the same pixels in the surface */

  nxgl_rectoffset(&src, rect, wnd->surfpos.x - wnd->bounds.pt1.x,
                  wnd->surfpos.y - wnd->bounds.pt1.y);
  offset.x = rect->pt1.x - src.pt1.x;
  offset.y = rect->pt1.y - src.pt1.y;
  g_nxaccel->move(&src, &offset);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_accel_install
 *
 * Description:
 *   Replace the plane's framebuffer rasterizers with the board's
 *   accelerated operations, if the board has them for this plane.
 *
 ****************************************************************************/

void nxbe_accel_install(FAR struct nxbe_plane_s *plane, int display,
                        int planeno)
{
  FAR const struct nx_accel_s *accel = board_nx_accel(display, planeno);

  if (accel == NULL)
    {
      return;
    }

  DEBUGASSERT(accel->fill != NULL && accel->move != NULL &&
              accel->copy != NULL);

  g_nxaccel = accel;

#ifdef CONFIG_NX_RAMBACKED
  if (accel->surface_alloc != NULL && accel->surface_free != NULL)
    {
      g_pwfb_ram = plane->pwfb;
      plane->pwfb.setpixel      = pwfb_setpixel;
      plane->pwfb.fillrectangle = pwfb_fillrectangle;
      plane->pwfb.getrectangle  = pwfb_getrectangle;
      plane->pwfb.filltrapezoid = pwfb_filltrapezoid;
      plane->pwfb.moverectangle = pwfb_moverectangle;
      plane->pwfb.copyrectangle = pwfb_copyrectangle;
    }
#endif

  plane->dev.setpixel      = accel_setpixel;
  plane->dev.fillrectangle = accel_fillrectangle;
  plane->dev.getrectangle  = accel_getrectangle;
  plane->dev.filltrapezoid = accel_filltrapezoid;
  plane->dev.moverectangle = accel_moverectangle;
  plane->dev.copyrectangle = accel_copyrectangle;
}

/****************************************************************************
 * Name: nxbe_accel_flush
 *
 * Description:
 *   Let the board send what it has batched.
 *
 ****************************************************************************/

void nxbe_accel_flush(void)
{
  if (g_nxaccel != NULL && g_nxaccel->flush != NULL)
    {
      g_nxaccel->flush();
    }
}

#ifdef CONFIG_NX_RAMBACKED
/****************************************************************************
 * Name: nxbe_surface_open
 *
 * Description:
 *   Give a RAM-backed window a surface of the display hardware instead of
 *   memory, if the board has surfaces and room for one this size.
 *
 * Returned Value:
 *   true if the window now has a surface.
 *
 ****************************************************************************/

bool nxbe_surface_open(FAR struct nxbe_window_s *wnd)
{
  struct nxgl_point_s pos;
  nxgl_coord_t w = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
  nxgl_coord_t h = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;

  if (g_nxaccel == NULL || g_nxaccel->surface_alloc == NULL ||
      w <= 0 || h <= 0 || g_nxaccel->surface_alloc(w, h, &pos) < 0)
    {
      return false;
    }

  wnd->surface    = true;
  wnd->surfpos    = pos;
  wnd->surfsize.w = w;
  wnd->surfsize.h = h;
  return true;
}

/****************************************************************************
 * Name: nxbe_surface_close
 ****************************************************************************/

void nxbe_surface_close(FAR struct nxbe_window_s *wnd)
{
  if (wnd->surface)
    {
      g_nxaccel->surface_free(&wnd->surfpos, wnd->surfsize.w,
                              wnd->surfsize.h);
      wnd->surface = false;
    }
}

/****************************************************************************
 * Name: nxbe_surface_resize
 *
 * Description:
 *   The window's size has changed: a surface of the new size with what
 *   the old one had where both overlap (zero elsewhere).
 *
 * Returned Value:
 *   false if there is no room: the window has no surface any more.
 *
 ****************************************************************************/

bool nxbe_surface_resize(FAR struct nxbe_window_s *wnd)
{
  struct nxgl_point_s oldpos = wnd->surfpos;
  struct nxgl_size_s oldsize = wnd->surfsize;
  struct nxgl_point_s offset;
  struct nxgl_rect_s rect;

  wnd->surface = false;
  if (!nxbe_surface_open(wnd))
    {
      g_nxaccel->surface_free(&oldpos, oldsize.w, oldsize.h);
      return false;
    }

  /* The old content, as much as fits */

  rect.pt1   = oldpos;
  rect.pt2.x = oldpos.x + MIN(oldsize.w, wnd->surfsize.w) - 1;
  rect.pt2.y = oldpos.y + MIN(oldsize.h, wnd->surfsize.h) - 1;
  offset.x   = wnd->surfpos.x - oldpos.x;
  offset.y   = wnd->surfpos.y - oldpos.y;
  g_nxaccel->move(&rect, &offset);

  /* Zero the new territory, right and bottom */

  if (wnd->surfsize.w > oldsize.w)
    {
      rect.pt1.x = wnd->surfpos.x + oldsize.w;
      rect.pt1.y = wnd->surfpos.y;
      rect.pt2.x = wnd->surfpos.x + wnd->surfsize.w - 1;
      rect.pt2.y = wnd->surfpos.y + wnd->surfsize.h - 1;
      g_nxaccel->fill(&rect, 0);
    }

  if (wnd->surfsize.h > oldsize.h)
    {
      rect.pt1.x = wnd->surfpos.x;
      rect.pt1.y = wnd->surfpos.y + oldsize.h;
      rect.pt2.x = wnd->surfpos.x + wnd->surfsize.w - 1;
      rect.pt2.y = wnd->surfpos.y + wnd->surfsize.h - 1;
      g_nxaccel->fill(&rect, 0);
    }

  g_nxaccel->surface_free(&oldpos, oldsize.w, oldsize.h);
  return true;
}

/****************************************************************************
 * Name: nxbe_surface_present
 *
 * Description:
 *   Show part of a window's surface (dest, window coordinates) where the
 *   window is visible: moves from the surface to the display, clipped by
 *   the windows above.
 *
 ****************************************************************************/

void nxbe_surface_present(FAR struct nxbe_window_s *wnd,
                          FAR const struct nxgl_rect_s *dest)
{
  struct surf_present_s info;
  struct nxgl_rect_s bounds;

  if (NXBE_ISHIDDEN(wnd))
    {
      return;
    }

  nxgl_rectoffset(&bounds, dest, wnd->bounds.pt1.x, wnd->bounds.pt1.y);
  nxgl_rectintersect(&bounds, &bounds, &wnd->bounds);
  nxgl_rectintersect(&bounds, &bounds, &wnd->be->bkgd.bounds);
  if (nxgl_nullrect(&bounds))
    {
      return;
    }

  info.cops.visible  = surf_present_visible;
  info.cops.obscured = nxbe_clipnull;
  info.wnd           = wnd;
  nxbe_clipper(wnd->above, &bounds, NX_CLIPORDER_DEFAULT, &info.cops,
               &wnd->be->plane[0]);
}
#endif /* CONFIG_NX_RAMBACKED */
