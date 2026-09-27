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

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nx/nxaccel.h>
#include <nuttx/nx/nxcompose.h>

#include "nxbe.h"
#include "nxmu.h"

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

/* The RAM-backed windows (surfnext): in surfaces, in memory or parked,
 * hidden ones too
 */

static FAR struct nxbe_window_s *g_ramwnds;
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
 * the surface's position); otherwise the memory rasterizers.  A parked
 * window's drawing goes nowhere: its client draws all of it again when it
 * is seen (nxbe_surface_touch()).
 */

static void surf_rect(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rel,
                      FAR struct nxgl_rect_s *abs)
{
  nxgl_rectoffset(abs, rel, wnd->surfpos.x, wnd->surfpos.y);
}

static inline bool pwfb_parked(FAR struct nxbe_window_s *wnd)
{
#ifdef CONFIG_NX_COMPOSITOR
  if (wnd->parked != NXBE_PARK_NONE)
    {
      nxbe_surface_touch(wnd);
      return true;
    }
#endif

  return false;
}

/* Its surface is drawn into (window coordinates): copies of it read back
 * are out of date there
 */

static inline void pwfb_drawn(FAR struct nxbe_window_s *wnd,
                              FAR const struct nxgl_rect_s *rel)
{
#ifdef CONFIG_NX_COMPOSITOR
  if (wnd->srccache != NULL)
    {
      nxbe_srccache_drop(wnd, rel);
    }
#endif
}

static void pwfb_setpixel(FAR struct nxbe_window_s *wnd,
                          FAR const struct nxgl_point_s *pos,
                          nxgl_mxpixel_t color)
{
  struct nxgl_rect_s rect;

  if (pwfb_parked(wnd))
    {
      return;
    }

  if (!wnd->surface)
    {
      g_pwfb_ram.setpixel(wnd, pos, color);
      return;
    }

  rect.pt1 = *pos;
  rect.pt2 = *pos;
  pwfb_drawn(wnd, &rect);
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

  if (pwfb_parked(wnd))
    {
      return;
    }

  if (!wnd->surface)
    {
      g_pwfb_ram.fillrectangle(wnd, rect, color);
      return;
    }

  pwfb_drawn(wnd, rect);
  surf_rect(wnd, rect, &abs);
  g_nxaccel->fill(&abs, color);
}

static void pwfb_getrectangle(FAR struct nxbe_window_s *wnd,
                              FAR const struct nxgl_rect_s *rect,
                              FAR void *dest, unsigned int deststride)
{
  struct nxgl_rect_s abs;

  if (!wnd->surface && wnd->fbmem != NULL)
    {
      g_pwfb_ram.getrectangle(wnd, rect, dest, deststride);
      return;
    }

  if (!wnd->surface)
    {
      /* Parked: nothing to read */

      FAR uint8_t *line = dest;
      unsigned int len = ((rect->pt2.x - rect->pt1.x + 1) *
                          wnd->be->plane[0].pinfo.bpp + 7) >> 3;
      int y;

      for (y = rect->pt1.y; y <= rect->pt2.y; y++, line += deststride)
        {
          memset(line, 0, len);
        }

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

  if (pwfb_parked(wnd))
    {
      return;
    }

  if (!wnd->surface)
    {
      g_pwfb_ram.filltrapezoid(wnd, trap, bounds, color);
      return;
    }

  pwfb_drawn(wnd, bounds);
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

  if (pwfb_parked(wnd))
    {
      return;
    }

  if (!wnd->surface)
    {
      g_pwfb_ram.moverectangle(wnd, rect, destpos);
      return;
    }

  offset.x = destpos->x - rect->pt1.x;
  offset.y = destpos->y - rect->pt1.y;
  nxgl_rectoffset(&abs, rect, offset.x, offset.y);
  pwfb_drawn(wnd, &abs);
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

  if (pwfb_parked(wnd))
    {
      return;
    }

  if (!wnd->surface)
    {
      g_pwfb_ram.copyrectangle(wnd, dest, src, origin, srcstride);
      return;
    }

  pwfb_drawn(wnd, dest);
  surf_rect(wnd, dest, &abs);
  org.x = origin->x + wnd->surfpos.x;
  org.y = origin->y + wnd->surfpos.y;
  g_nxaccel->copy(&abs, src, &org, srcstride);
}

/* The list of RAM-backed windows: in surfaces, in memory, or parked */

static void surf_link(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_window_s *w;

  for (w = g_ramwnds; w != NULL; w = w->surfnext)
    {
      if (w == wnd)
        {
          return;
        }
    }

  wnd->surfnext = g_ramwnds;
  g_ramwnds     = wnd;
}

static void surf_unlink(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_window_s **pp;

  for (pp = &g_ramwnds; *pp != NULL; pp = &(*pp)->surfnext)
    {
      if (*pp == wnd)
        {
          *pp = wnd->surfnext;
          return;
        }
    }
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
 *   memory, if the board has surfaces and room for one this size.  When
 *   there is no room: windows that are not seen are parked (the least
 *   recently seen first), the compositor gives up its second display
 *   buffer, and the surfaces are packed together; only then does the
 *   window stay in memory.
 *
 * Returned Value:
 *   true if the window now has a surface.
 *
 ****************************************************************************/

/* Every surface as far up, then left, as it will go, from the top down:
 * the room between them comes together below them.  Each is one copy,
 * which may overlap where it was.
 */

static void surf_compact(void)
{
  FAR struct nxbe_window_s **order;
  FAR struct nxbe_window_s *wnd;
  int n = 0;
  int i;
  int j;

  for (wnd = g_ramwnds; wnd != NULL; wnd = wnd->surfnext)
    {
      n += wnd->surface;
    }

  if (n == 0)
    {
      return;
    }

  order = kmm_malloc(n * sizeof(*order));
  if (order == NULL)
    {
      return;
    }

  for (i = 0, wnd = g_ramwnds; wnd != NULL; wnd = wnd->surfnext)
    {
      if (!wnd->surface)
        {
          continue;
        }

      /* In order of place: lowest y, then lowest x */

      for (j = i++; j > 0 &&
           (order[j - 1]->surfpos.y > wnd->surfpos.y ||
            (order[j - 1]->surfpos.y == wnd->surfpos.y &&
             order[j - 1]->surfpos.x > wnd->surfpos.x)); j--)
        {
          order[j] = order[j - 1];
        }

      order[j] = wnd;
    }

  for (i = 0; i < n; i++)
    {
      if (order[i]->surface && !nxbe_surface_relocate(order[i]))
        {
          /* Lost (it was just freed, so there was room): as for a
           * display mode, in memory
           */

          nxbe_surface_tomemory(order[i]);
        }
    }

  kmm_free(order);
}

#ifdef CONFIG_NX_COMPOSITOR
/* Park the window that has not been seen for longest (other than except),
 * if any may be parked
 */

static bool surf_park_lru(FAR struct nxbe_window_s *except)
{
  FAR struct nxbe_window_s *best = NULL;
  FAR struct nxbe_window_s *wnd;
  uint32_t now = (uint32_t)clock_systime_ticks();
  uint32_t bestage = 0;

  for (wnd = g_ramwnds; wnd != NULL; wnd = wnd->surfnext)
    {
      uint32_t age;

      if (wnd == except || !wnd->surface ||
          (wnd->fx.flags & NX_FX_PARK) == 0 || nxbe_compose_seen(wnd))
        {
          continue;
        }

      age = wnd->unseen != 0 ? now - wnd->unseen : 0;
      if (best == NULL || age > bestage)
        {
          best    = wnd;
          bestage = age;
        }
    }

  return best != NULL && nxbe_surface_park(best);
}
#endif

/* A surface of w x h for the window: only if there is room (make false),
 * or making room
 */

static bool surf_alloc(FAR struct nxbe_window_s *wnd, nxgl_coord_t w,
                       nxgl_coord_t h, bool make,
                       FAR struct nxgl_point_s *pos)
{
  bool compacted = false;

  while (g_nxaccel->surface_alloc(w, h, wnd->surfkind, pos) < 0)
    {
      if (!make)
        {
          return false;
        }

#ifdef CONFIG_NX_COMPOSITOR
      /* Windows not seen give up theirs first, then the compositor its
       * second display buffer
       */

      if (surf_park_lru(wnd) || nxbe_compose_reclaim())
        {
          continue;
        }
#endif

      /* Then the room there is, together */

      if (!compacted && g_ramwnds != NULL)
        {
          surf_compact();
          compacted = true;
          continue;
        }

      return false;
    }

  return true;
}

static bool surf_open(FAR struct nxbe_window_s *wnd, bool make)
{
  struct nxgl_point_s pos;
  nxgl_coord_t w = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
  nxgl_coord_t h = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;

  if (g_nxaccel == NULL || g_nxaccel->surface_alloc == NULL ||
      w <= 0 || h <= 0 || !surf_alloc(wnd, w, h, make, &pos))
    {
      return false;
    }

  wnd->surface    = true;
  wnd->surfpos    = pos;
  wnd->surfsize.w = w;
  wnd->surfsize.h = h;
#ifdef CONFIG_NX_COMPOSITOR
  nxbe_compose_changed();
#endif
  return true;
}

/* The surface goes back to the hardware */

static void surf_free(FAR struct nxbe_window_s *wnd)
{
  g_nxaccel->surface_free(&wnd->surfpos, wnd->surfsize.w, wnd->surfsize.h);
  wnd->surface = false;
#ifdef CONFIG_NX_COMPOSITOR
  nxbe_compose_freed();
#endif
}

bool nxbe_surface_open(FAR struct nxbe_window_s *wnd)
{
  if (g_nxaccel == NULL || g_nxaccel->surface_alloc == NULL)
    {
      return false;
    }

  surf_link(wnd);
  return surf_open(wnd, true);
}

/****************************************************************************
 * Name: nxbe_surface_list
 *
 * Description:
 *   The RAM-backed windows (linked by surfnext): in surfaces, in memory or
 *   parked, hidden ones too.
 *
 ****************************************************************************/

FAR struct nxbe_window_s *nxbe_surface_list(void)
{
  return g_ramwnds;
}

/****************************************************************************
 * Name: nxbe_surface_close
 ****************************************************************************/

void nxbe_surface_close(FAR struct nxbe_window_s *wnd)
{
  if (g_nxaccel == NULL || g_nxaccel->surface_alloc == NULL)
    {
      return;
    }

  if (wnd->surface)
    {
      surf_free(wnd);
    }

#ifdef CONFIG_NX_COMPOSITOR
  if (wnd->parked == NXBE_PARK_STREAM && g_nxaccel->park_free != NULL)
    {
      g_nxaccel->park_free(wnd->parkhandle);
    }

  wnd->parked = NXBE_PARK_NONE;
  nxbe_srccache_free(wnd);
  nxbe_compose_changed();
#endif
  surf_unlink(wnd);
}

/****************************************************************************
 * Name: nxbe_surface_resize
 *
 * Description:
 *   The window's size has changed: a surface of the new size with what
 *   the old one had where both overlap (zero elsewhere).  If there is no
 *   room for both, the old one goes first, and its content with it (all
 *   zero, *lost is set); the compositor's second display buffer is given
 *   up only if that is not enough.
 *
 * Returned Value:
 *   false if there is no room: the window has no surface any more.
 *
 ****************************************************************************/

bool nxbe_surface_resize(FAR struct nxbe_window_s *wnd, FAR bool *lost)
{
  struct nxgl_point_s oldpos = wnd->surfpos;
  struct nxgl_size_s oldsize = wnd->surfsize;
  struct nxgl_point_s offset;
  struct nxgl_rect_s rect;

#ifdef CONFIG_NX_COMPOSITOR
  nxbe_srccache_free(wnd);
#endif

  /* A new one beside the old, else in the old one's place too, else
   * making room
   */

  *lost = false;
  wnd->surface = false;
  if (!surf_open(wnd, false))
    {
      g_nxaccel->surface_free(&oldpos, oldsize.w, oldsize.h);
#ifdef CONFIG_NX_COMPOSITOR
      nxbe_compose_freed();
#endif
      if (!surf_open(wnd, false) && !surf_open(wnd, true))
        {
          return false;
        }

      rect.pt1   = wnd->surfpos;
      rect.pt2.x = wnd->surfpos.x + wnd->surfsize.w - 1;
      rect.pt2.y = wnd->surfpos.y + wnd->surfsize.h - 1;
      g_nxaccel->fill(&rect, 0);
      *lost = true;
      return true;
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
#ifdef CONFIG_NX_COMPOSITOR
  nxbe_compose_freed();
#endif
  return true;
}

/****************************************************************************
 * Name: nxbe_surface_relocate
 *
 * Description:
 *   Move a window's surface out of the way of rectangle keep (the display,
 *   grown by a display mode): a new surface elsewhere and one copy.
 *
 * Returned Value:
 *   false if there is no room elsewhere: the window has no surface any
 *   more (nor its content).
 *
 ****************************************************************************/

bool nxbe_surface_relocate(FAR struct nxbe_window_s *wnd)
{
  struct nxgl_point_s oldpos = wnd->surfpos;
  struct nxgl_size_s oldsize = wnd->surfsize;
  struct nxgl_point_s offset;
  struct nxgl_point_s pos;
  struct nxgl_rect_s rect;

  /* The new place may overlap the old one (the copy works as if through
   * a buffer), not any other
   */

  g_nxaccel->surface_free(&oldpos, oldsize.w, oldsize.h);
  if (g_nxaccel->surface_alloc(oldsize.w, oldsize.h, wnd->surfkind,
                               &pos) < 0)
    {
      wnd->surface = false;
#ifdef CONFIG_NX_COMPOSITOR
      nxbe_srccache_free(wnd);
      nxbe_compose_changed();
#endif
      return false;
    }

  rect.pt1   = oldpos;
  rect.pt2.x = oldpos.x + oldsize.w - 1;
  rect.pt2.y = oldpos.y + oldsize.h - 1;
  offset.x   = pos.x - oldpos.x;
  offset.y   = pos.y - oldpos.y;
  g_nxaccel->move(&rect, &offset);
  wnd->surfpos = pos;
  return true;
}

/****************************************************************************
 * Name: nxbe_surface_evacuate
 *
 * Description:
 *   The display has grown over keep (a display mode): move every surface
 *   out of it, the largest first.  A window whose surface finds no room
 *   elsewhere goes to memory, and its client is asked to draw it again.
 *
 ****************************************************************************/

void nxbe_surface_evacuate(FAR const struct nxgl_rect_s *keep)
{
  FAR struct nxbe_window_s *wnd;
  FAR struct nxbe_window_s *big;
  struct nxgl_rect_s r;
  int32_t area;
  int32_t a;

  for (; ; )
    {
      big  = NULL;
      area = 0;
      for (wnd = g_ramwnds; wnd != NULL; wnd = wnd->surfnext)
        {
          if (!wnd->surface)
            {
              continue;
            }

          r.pt1   = wnd->surfpos;
          r.pt2.x = wnd->surfpos.x + wnd->surfsize.w - 1;
          r.pt2.y = wnd->surfpos.y + wnd->surfsize.h - 1;
          a       = (int32_t)wnd->surfsize.w * wnd->surfsize.h;
          if (nxgl_rectoverlap(&r, (FAR struct nxgl_rect_s *)keep) &&
              a > area)
            {
              big  = wnd;
              area = a;
            }
        }

      if (big == NULL)
        {
          return;
        }

      if (nxbe_surface_relocate(big))
        {
          continue;
        }

#ifdef CONFIG_NX_COMPOSITOR
      /* Rather than in memory, parked if it may be */

      if ((big->fx.flags & NX_FX_PARK) != 0 && !nxbe_compose_seen(big))
        {
          big->parked = NXBE_PARK_LOST;
          g_nxbe_counts.parks++;
          continue;
        }
#endif

      nxbe_surface_tomemory(big);
    }
}

/****************************************************************************
 * Name: nxbe_surface_tomemory
 *
 * Description:
 *   A window whose surface is gone (no room for it where it had to move):
 *   in memory, all zero, and its client is asked to draw it again.
 *
 ****************************************************************************/

void nxbe_surface_tomemory(FAR struct nxbe_window_s *wnd)
{
  nxgl_coord_t w = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
  nxgl_coord_t h = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;

  gwarn("WARNING: no room for a %dx%d surface: in memory\n", w, h);
#ifndef CONFIG_BUILD_KERNEL
  wnd->stride = (wnd->be->plane[0].pinfo.bpp * w + 7) >> 3;
  wnd->fbmem  = kumm_zalloc((size_t)wnd->stride * h);
#endif
  if (wnd->fbmem == NULL)
    {
      wnd->stride = 0;
      NXBE_CLRRAMBACKED(wnd);
    }

#ifdef CONFIG_NX_COMPOSITOR
  nxbe_compose_changed();
#endif
  nxmu_redrawreq(wnd, &wnd->bounds);
}

#ifdef CONFIG_NX_COMPOSITOR
/****************************************************************************
 * Name: nxbe_surface_park
 *
 * Description:
 *   A window that is not seen gives up its surface: its pixels go into a
 *   compressed block of the hardware's (nx_accel_s park) if it has them
 *   and room for one, else they are dropped, and its client will be asked
 *   to draw it again when it is seen.
 *
 ****************************************************************************/

bool nxbe_surface_park(FAR struct nxbe_window_s *wnd)
{
  int handle = -ENOSYS;

  if (wnd->parked != NXBE_PARK_NONE)
    {
      return false;
    }

  if (!wnd->surface)
    {
      /* In memory: the memory goes */

      if (wnd->fbmem == NULL)
        {
          return false;
        }

      kumm_free(wnd->fbmem);
      wnd->fbmem  = NULL;
      wnd->parked = NXBE_PARK_LOST;
      g_nxbe_counts.parks++;
      nxbe_compose_changed();
      return true;
    }

  if (g_nxaccel->park != NULL)
    {
      handle = g_nxaccel->park(&wnd->surfpos, wnd->surfsize.w,
                               wnd->surfsize.h);
    }

  nxbe_srccache_free(wnd);
  surf_free(wnd);
  wnd->parked     = handle > 0 ? NXBE_PARK_STREAM : NXBE_PARK_LOST;
  wnd->parkhandle = handle > 0 ? handle : 0;
  g_nxbe_counts.parks++;
  nxbe_compose_changed();
  ginfo("Parked %p (%dx%d): %s\n", wnd, wnd->surfsize.w, wnd->surfsize.h,
        handle > 0 ? "compressed" : "to be drawn again");
  return true;
}

/****************************************************************************
 * Name: nxbe_surface_touch
 *
 * Description:
 *   A parked window is drawn into: its compressed pixels are out of date,
 *   and go.  It will be drawn again in full when it is seen.
 *
 ****************************************************************************/

void nxbe_surface_touch(FAR struct nxbe_window_s *wnd)
{
  if (wnd->parked == NXBE_PARK_STREAM)
    {
      if (g_nxaccel->park_free != NULL)
        {
          g_nxaccel->park_free(wnd->parkhandle);
        }

      wnd->parked     = NXBE_PARK_LOST;
      wnd->parkhandle = 0;
      nxbe_compose_changed();
    }
}

/****************************************************************************
 * Name: nxbe_surface_unpark
 *
 * Description:
 *   A parked window is seen again: a surface for it (making room, as for
 *   a new window), its pixels put back, or its client asked to draw it
 *   again.  With no room at all, it goes to memory.
 *
 ****************************************************************************/

void nxbe_surface_unpark(FAR struct nxbe_window_s *wnd)
{
  struct nxgl_rect_s rect;
  uint8_t parked = wnd->parked;
  bool sized;

  if (parked == NXBE_PARK_NONE)
    {
      return;
    }

  /* Other windows may be parked to make room, not this one */

  sized = wnd->surfsize.w == wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1 &&
          wnd->surfsize.h == wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;
  wnd->parked = NXBE_PARK_NONE;
  wnd->unseen = 0;
  g_nxbe_counts.unparks++;
  if (surf_open(wnd, true))
    {
      if (parked == NXBE_PARK_STREAM && sized && g_nxaccel->unpark != NULL)
        {
          int ret = g_nxaccel->unpark(wnd->parkhandle, &wnd->surfpos);

          wnd->parkhandle = 0;
          if (ret >= 0)
            {
              nxbe_compose_changed();
              nxbe_compose_damage_window(wnd);
              return;
            }

          parked = NXBE_PARK_LOST;          /* The block is gone */
        }

      rect.pt1   = wnd->surfpos;
      rect.pt2.x = wnd->surfpos.x + wnd->surfsize.w - 1;
      rect.pt2.y = wnd->surfpos.y + wnd->surfsize.h - 1;
      g_nxaccel->fill(&rect, 0);
    }
  else
    {
      wnd->stride = (wnd->be->plane[0].pinfo.bpp *
                     (wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1) + 7) >> 3;
      wnd->fbmem  = kumm_zalloc((size_t)wnd->stride *
                                (wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1));
      if (wnd->fbmem == NULL)
        {
          wnd->stride = 0;
          NXBE_CLRRAMBACKED(wnd);
        }
    }

  if (parked == NXBE_PARK_STREAM && g_nxaccel->park_free != NULL)
    {
      g_nxaccel->park_free(wnd->parkhandle);
    }

  wnd->parkhandle = 0;
  g_nxbe_counts.redraws++;
  nxbe_compose_changed();
  nxbe_compose_redrawing(wnd);
  nxmu_redrawreq(wnd, &wnd->bounds);
}

/****************************************************************************
 * Name: nxbe_surface_frommemory
 *
 * Description:
 *   Windows in memory move into surfaces if there is room for them now:
 *   their pixels are put there, the memory freed.  Returns how many.
 *
 ****************************************************************************/

int nxbe_surface_frommemory(void)
{
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s rect;
  bool compacted = false;
  int left;
  int pass;
  int n = 0;

  for (; ; )
    {
      /* Windows that are seen first */

      left = 0;
      for (pass = 0; pass < 2; pass++)
        {
          for (wnd = g_ramwnds; wnd != NULL; wnd = wnd->surfnext)
            {
              if (wnd->surface || wnd->fbmem == NULL ||
                  !NXBE_ISRAMBACKED(wnd) || wnd->parked != NXBE_PARK_NONE ||
                  wnd->be->plane[0].pinfo.bpp != 16 ||
                  nxbe_compose_seen(wnd) != (pass == 0))
                {
                  continue;
                }

              if (!surf_open(wnd, false))
                {
                  left += pass == 0;
                  continue;
                }

              rect.pt1   = wnd->surfpos;
              rect.pt2.x = wnd->surfpos.x + wnd->surfsize.w - 1;
              rect.pt2.y = wnd->surfpos.y + wnd->surfsize.h - 1;
              g_nxaccel->copy(&rect, wnd->fbmem, &wnd->surfpos,
                              wnd->stride);
              kumm_free(wnd->fbmem);
              wnd->fbmem = NULL;
              g_nxbe_counts.tosurface++;
              nxbe_compose_damage_window(wnd);
              n++;
            }
        }

      /* Seen windows that did not fit: the room there is, together, once
       * (the room may be there, in pieces)
       */

      if (left == 0 || compacted)
        {
          return n;
        }

      surf_compact();
      compacted = true;
    }
}
#endif

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

  if (NXBE_ISHIDDEN(wnd) || !wnd->surface)
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
