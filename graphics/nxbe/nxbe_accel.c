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
/* Surfaces in pieces ********************************************************/

/* Make a window's pieces again from its cells: the runs of cells along a
 * row that are together in memory, each grown down as long as the rows
 * below have the same run below it in memory.  Built in cells, then made
 * pixels (clipped to the surface's size).
 */

static bool cells_build(FAR struct nxbe_window_s *wnd,
                        FAR struct nxbe_cells_s *c)
{
  FAR struct nxbe_piece_s *p;
  FAR uint16_t *open;
  FAR uint16_t *prev;
  FAR uint16_t *cur;
  FAR uint16_t *tmp;
  int sh = g_nxaccel->cellshift;
  int nprev = 0;
  int n = 0;
  int i;
  int j;
  int k;

  c->dirty = false;
  if (c->cw == 0)
    {
      if (c->maxpieces < 1)
        {
          p = kmm_malloc(sizeof(*p));
          if (p == NULL)
            {
              c->npieces = 0;
              return false;
            }

          c->pieces    = p;
          c->maxpieces = 1;
        }

      c->pieces[0].r.pt1.x = 0;
      c->pieces[0].r.pt1.y = 0;
      c->pieces[0].r.pt2.x = c->w - 1;
      c->pieces[0].r.pt2.y = c->h - 1;
      c->pieces[0].pos = wnd->surfpos;
      c->npieces = 1;
      return true;
    }

  open = kmm_malloc(c->cw * sizeof(uint16_t) * 2);
  if (open == NULL)
    {
      c->npieces = 0;
      c->dirty   = true;
      return false;
    }

  /* prev: the pieces that reached the row above; cur: this row's */

  prev = open;
  cur  = open + c->cw;
  for (j = 0; j < c->ch; j++)
    {
      int ncur = 0;

      for (i = 0; i < c->cw; )
        {
          uint16_t m = c->map[j * c->cw + i];
          int i0 = i;

          if (m == NX_CELL_NONE)
            {
              i++;
              continue;
            }

          while (i + 1 < c->cw &&
                 c->map[j * c->cw + i + 1] == m + (i + 1 - i0))
            {
              i++;
            }

          for (k = 0; k < nprev; k++)
            {
              p = &c->pieces[prev[k]];
              if (p->r.pt1.x == i0 && p->r.pt2.x == i &&
                  p->pos.x == NX_CELL_X(m) &&
                  p->pos.y + (j - p->r.pt1.y) == NX_CELL_Y(m))
                {
                  break;
                }
            }

          if (k < nprev)
            {
              p->r.pt2.y = j;
              cur[ncur++] = prev[k];
            }
          else
            {
              if (n >= c->maxpieces)
                {
                  int max = MAX(8, c->maxpieces * 2);

                  p = kmm_realloc(c->pieces, max * sizeof(*p));
                  if (p == NULL)
                    {
                      kmm_free(open);
                      c->npieces = 0;
                      c->dirty   = true;
                      return false;
                    }

                  c->pieces    = p;
                  c->maxpieces = max;
                }

              p = &c->pieces[n];
              p->r.pt1.x = i0;
              p->r.pt1.y = j;
              p->r.pt2.x = i;
              p->r.pt2.y = j;
              p->pos.x   = NX_CELL_X(m);
              p->pos.y   = NX_CELL_Y(m);
              cur[ncur++] = n++;
            }

          i++;
        }

      tmp   = prev;
      prev  = cur;
      cur   = tmp;
      nprev = ncur;
    }

  kmm_free(open);

  for (k = 0; k < n; k++)
    {
      p = &c->pieces[k];
      p->r.pt1.x <<= sh;
      p->r.pt1.y <<= sh;
      p->r.pt2.x = MIN(((p->r.pt2.x + 1) << sh) - 1, c->w - 1);
      p->r.pt2.y = MIN(((p->r.pt2.y + 1) << sh) - 1, c->h - 1);
      p->pos.x <<= sh;
      p->pos.y <<= sh;
    }

  c->npieces = n;

  /* Where its first pixel is (for those who must know: GPU Bench) */

  if (n > 0 && c->pieces[0].r.pt1.x == 0 && c->pieces[0].r.pt1.y == 0)
    {
      wnd->surfpos = c->pieces[0].pos;
    }

  return true;
}

int nxbe_surface_pieces(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxbe_piece_s **pieces)
{
  FAR struct nxbe_cells_s *c = wnd->cells;

  if (!wnd->surface || c == NULL)
    {
      *pieces = NULL;
      return 0;
    }

  if (c->dirty)
    {
      cells_build(wnd, c);
    }

  *pieces = c->pieces;
  return c->npieces;
}

/* A window's cells have moved (or been added): its pieces are made again;
 * shown in perspective, it is drawn again (its pieces' rows, where the
 * texels fall, are not the same)
 */

static void cells_moved(FAR struct nxbe_window_s *wnd)
{
  if (wnd->cells != NULL)
    {
      wnd->cells->dirty = true;
    }

#ifdef CONFIG_NX_COMPOSITOR
  if ((wnd->fx.flags & NX_FX_QUAD) != 0)
    {
      nxbe_compose_damage_window(wnd);
    }
#endif
}

#define piece_part nxbe_piece_part

void nxbe_surface_fill(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *rel,
                       nxgl_mxpixel_t color)
{
  FAR const struct nxbe_piece_s *p;
  struct nxgl_rect_s x;
  struct nxgl_rect_s abs;
  int n = nxbe_surface_pieces(wnd, &p);
  int i;

  for (i = 0; i < n; i++)
    {
      if (piece_part(&p[i], rel, &x, &abs))
        {
          g_nxaccel->fill(&abs, color);
        }
    }
}

/* Pixels from memory: the one for rel->pt1 at src, stride bytes a row */

void nxbe_surface_put(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rel,
                      FAR const void *src, unsigned int stride)
{
  FAR const struct nxbe_piece_s *p;
  struct nxgl_rect_s x;
  struct nxgl_rect_s abs;
  struct nxgl_point_s org;
  int n = nxbe_surface_pieces(wnd, &p);
  int i;

  for (i = 0; i < n; i++)
    {
      if (piece_part(&p[i], rel, &x, &abs))
        {
          org.x = rel->pt1.x + p[i].pos.x - p[i].r.pt1.x;
          org.y = rel->pt1.y + p[i].pos.y - p[i].r.pt1.y;
          g_nxaccel->copy(&abs, src, &org, stride);
        }
    }
}

/* Pixels into memory: rel->pt1's at dest */

void nxbe_surface_get(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rel,
                      FAR void *dest, unsigned int stride)
{
  FAR const struct nxbe_piece_s *p;
  struct nxgl_rect_s x;
  struct nxgl_rect_s abs;
  unsigned int bpp = wnd->be->plane[0].pinfo.bpp;
  int n = nxbe_surface_pieces(wnd, &p);
  int i;

  for (i = 0; i < n; i++)
    {
      if (piece_part(&p[i], rel, &x, &abs))
        {
          accel_getrectangle(&wnd->be->plane[0].pinfo, &abs,
                             (FAR uint8_t *)dest +
                             (size_t)(x.pt1.y - rel->pt1.y) * stride +
                             (((x.pt1.x - rel->pt1.x) * bpp) >> 3), stride);
        }
    }
}

/* The window's pixels in rel to the hardware's memory at dest (rel->pt1
 * there): a copy of each piece
 */

void nxbe_surface_copyto(FAR struct nxbe_window_s *wnd,
                         FAR const struct nxgl_rect_s *rel,
                         FAR const struct nxgl_point_s *dest)
{
  FAR const struct nxbe_piece_s *p;
  struct nxgl_rect_s x;
  struct nxgl_rect_s abs;
  struct nxgl_point_s off;
  int n = nxbe_surface_pieces(wnd, &p);
  int i;

  for (i = 0; i < n; i++)
    {
      if (piece_part(&p[i], rel, &x, &abs))
        {
          off.x = dest->x + x.pt1.x - rel->pt1.x - abs.pt1.x;
          off.y = dest->y + x.pt1.y - rel->pt1.y - abs.pt1.y;
          if (off.x != 0 || off.y != 0)
            {
              g_nxaccel->move(&abs, &off);
            }
        }
    }
}

/* Move rel of a window's pixels by offset, within it, as if through a
 * buffer.  Within one piece that is one copy (the hardware's copies
 * work so); across pieces, a copy for each part of the destination in
 * one piece whose source is in one piece, in an order where none
 * overwrites what another has still to read (each before those that
 * write where it reads).  If there is no such order, the pixels go
 * through memory.
 */

struct move_part_s
{
  struct nxgl_rect_s src;            /* Of the window */
  struct nxgl_rect_s dst;
  struct nxgl_rect_s abs;            /* src in memory */
  struct nxgl_point_s off;           /* abs to where dst is in memory */
  bool done;
};

void nxbe_surface_move(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *rel,
                       FAR const struct nxgl_point_s *offset)
{
  FAR const struct nxbe_piece_s *p;
  FAR struct move_part_s *part;
  struct nxgl_rect_s dst;
  struct nxgl_rect_s d;
  struct nxgl_rect_s s;
  struct nxgl_rect_s x;
  struct nxgl_rect_s abs;
  int n = nxbe_surface_pieces(wnd, &p);
  int np = 0;
  int left;
  int i;
  int j;
  int k;

  if (n == 1)
    {
      if (piece_part(&p[0], rel, &x, &abs))
        {
          g_nxaccel->move(&abs, offset);
        }

      return;
    }

  part = kmm_malloc(n * n * sizeof(*part));
  if (part == NULL)
    {
      return;
    }

  nxgl_rectoffset(&dst, rel, offset->x, offset->y);
  for (i = 0; i < n; i++)
    {
      nxgl_rectintersect(&d, &dst, &p[i].r);
      if (d.pt1.x > d.pt2.x || d.pt1.y > d.pt2.y)
        {
          continue;
        }

      nxgl_rectoffset(&s, &d, -offset->x, -offset->y);
      for (j = 0; j < n; j++)
        {
          if (!piece_part(&p[j], &s, &x, &abs))
            {
              continue;
            }

          part[np].src  = x;
          nxgl_rectoffset(&part[np].dst, &x, offset->x, offset->y);
          part[np].abs  = abs;
          part[np].off.x = part[np].dst.pt1.x + p[i].pos.x - p[i].r.pt1.x -
                           abs.pt1.x;
          part[np].off.y = part[np].dst.pt1.y + p[i].pos.y - p[i].r.pt1.y -
                           abs.pt1.y;
          part[np].done = false;
          np++;
        }
    }

  /* Each part when no part not yet done reads where it writes */

  for (left = np; left > 0; )
    {
      for (k = 0; k < np; k++)
        {
          if (part[k].done)
            {
              continue;
            }

          for (j = 0; j < np; j++)
            {
              if (j != k && !part[j].done &&
                  nxgl_rectoverlap(&part[k].dst, &part[j].src))
                {
                  break;
                }
            }

          if (j == np)
            {
              break;
            }
        }

      if (k == np)
        {
          break;                         /* No such order */
        }

      if (part[k].off.x != 0 || part[k].off.y != 0)
        {
          g_nxaccel->move(&part[k].abs, &part[k].off);
        }

      part[k].done = true;
      left--;
    }

  kmm_free(part);
  if (left > 0)
    {
      /* Through memory: what is left to read, read, then written */

      unsigned int bpp = wnd->be->plane[0].pinfo.bpp;
      unsigned int stride = (((rel->pt2.x - rel->pt1.x + 1) * bpp) + 7) >> 3;
      FAR uint8_t *buf = kumm_malloc((size_t)stride *
                                     (rel->pt2.y - rel->pt1.y + 1));

      gwarn("WARNING: a move across pieces through memory\n");
      if (buf != NULL)
        {
          nxbe_surface_get(wnd, rel, buf, stride);
          nxbe_surface_put(wnd, &dst, buf, stride);
          kumm_free(buf);
        }
    }
}

/* The per-window rasterizers (nxbe_pwfb_vtable_s): for a window with a
 * surface, the board's operations on each piece of the surface that the
 * drawing meets; otherwise the memory rasterizers.  A parked window's
 * drawing goes nowhere: its client draws all of it again when it is seen
 * (nxbe_surface_unpark()).
 */

static inline bool pwfb_parked(FAR struct nxbe_window_s *wnd)
{
#ifdef CONFIG_NX_COMPOSITOR
  return wnd->parked;
#else
  return false;
#endif
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
  nxbe_surface_fill(wnd, &rect, color);
}

static void pwfb_fillrectangle(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *rect,
                               nxgl_mxpixel_t color)
{
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
  nxbe_surface_fill(wnd, rect, color);
}

static void pwfb_getrectangle(FAR struct nxbe_window_s *wnd,
                              FAR const struct nxgl_rect_s *rect,
                              FAR void *dest, unsigned int deststride)
{
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

  nxbe_surface_get(wnd, rect, dest, deststride);
}

static void pwfb_filltrapezoid(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_trapezoid_s *trap,
                               FAR const struct nxgl_rect_s *bounds,
                               nxgl_mxpixel_t color)
{
  FAR const struct nxbe_piece_s *p;
  struct nxgl_trapezoid_s t;
  struct nxgl_rect_s x;
  struct nxgl_rect_s b;
  int n;
  int i;

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
  n = nxbe_surface_pieces(wnd, &p);
  for (i = 0; i < n; i++)
    {
      nxgl_coord_t dx = p[i].pos.x - p[i].r.pt1.x;
      nxgl_coord_t dy = p[i].pos.y - p[i].r.pt1.y;

      if (!piece_part(&p[i], bounds, &x, &b))
        {
          continue;
        }

      t = *trap;
      t.top.x1 += itob16(dx);
      t.top.x2 += itob16(dx);
      t.bot.x1 += itob16(dx);
      t.bot.x2 += itob16(dx);
      t.top.y  += dy;
      t.bot.y  += dy;
      accel_filltrapezoid(NULL, &t, &b, color);
    }
}

/* NB: the third argument is where the rectangle goes, not an offset */

static void pwfb_moverectangle(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *rect,
                               FAR struct nxgl_point_s *destpos)
{
  struct nxgl_point_s offset;
  struct nxgl_rect_s dst;

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
  nxgl_rectoffset(&dst, rect, offset.x, offset.y);
  pwfb_drawn(wnd, &dst);
  nxbe_surface_move(wnd, rect, &offset);
}

static void pwfb_copyrectangle(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *dest,
                               FAR const void *src,
                               FAR const struct nxgl_point_s *origin,
                               unsigned int srcstride)
{
  FAR const uint8_t *pix;

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
  pix = (FAR const uint8_t *)src +
        (size_t)(dest->pt1.y - origin->y) * srcstride +
        (((dest->pt1.x - origin->x) * wnd->be->plane[0].pinfo.bpp) >> 3);
  nxbe_surface_put(wnd, dest, pix, srcstride);
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
  struct nxgl_rect_s rel;

  /* rect is on the display; the same pixels in the window */

  nxgl_rectoffset(&rel, rect, -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);
  nxbe_surface_copyto(wnd, &rel, &rect->pt1);
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
#ifdef CONFIG_NX_RAMBACKED
  if (g_nxaccel != NULL && g_nxaccel->lost != NULL && g_nxaccel->lost())
    {
      nxbe_surface_lost();
    }
#endif

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
 *   memory, if the board has surfaces and room for one this size: its
 *   cells, wherever they are free (an image window, one surface).  When
 *   there is no room: windows that are not seen are parked (the least
 *   recently seen first) and the compositor gives up its second display
 *   buffer; only then does the window stay in memory.
 *
 * Returned Value:
 *   true if the window now has a surface.
 *
 ****************************************************************************/

static FAR struct nxbe_cells_s *cells_new(int cw, int ch, nxgl_coord_t w,
                                          nxgl_coord_t h)
{
  FAR struct nxbe_cells_s *c;
  int k;

  c = kmm_zalloc(sizeof(*c) + (cw * ch > 1 ? cw * ch - 1 : 0) *
                 sizeof(uint16_t));
  if (c != NULL)
    {
      c->w     = w;
      c->h     = h;
      c->cw    = cw;
      c->ch    = ch;
      c->dirty = true;
      for (k = 0; k < cw * ch; k++)
        {
          c->map[k] = NX_CELL_NONE;
        }
    }

  return c;
}

static void cells_delete(FAR struct nxbe_cells_s *c)
{
  if (c != NULL)
    {
      kmm_free(c->pieces);
      kmm_free(c);
    }
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

/* Room was made: windows not seen give up theirs first, then the
 * compositor its second display buffer
 */

static inline bool surf_room(FAR struct nxbe_window_s *wnd)
{
#ifdef CONFIG_NX_COMPOSITOR
  return surf_park_lru(wnd) || nxbe_compose_reclaim();
#else
  return false;
#endif
}

/* Cells for every entry of the window's map that has none: only if there
 * are (make false), or making room
 */

static bool surf_cells(FAR struct nxbe_window_s *wnd,
                       FAR struct nxbe_cells_s *c, bool make)
{
  while (g_nxaccel->cells_alloc(c->cw, c->ch, c->map) < 0)
    {
      if (!make || !surf_room(wnd))
        {
          return false;
        }
    }

  c->dirty = true;
  return true;
}

/* Windows' cells inside rectangle keep (of the hardware's memory) moved
 * elsewhere, one copy each; the old cells given back (those a displacing
 * surface took are its already: no matter).  A window whose cells find
 * no room goes to memory (parked, if it may be and is not seen), drawn
 * again by its client.
 */

static void surf_cells_out(FAR const struct nxgl_rect_s *keep)
{
  FAR struct nxbe_window_s *wnd;
  int sh = g_nxaccel->cellshift;

  for (wnd = g_ramwnds; wnd != NULL; wnd = wnd->surfnext)
    {
      FAR struct nxbe_cells_s *c = wnd->cells;
      FAR uint16_t *old;
      struct nxgl_rect_s r;
      struct nxgl_point_s off;
      int n = 0;
      int k;

      if (!wnd->surface || c == NULL || c->cw == 0)
        {
          continue;
        }

      for (k = 0; k < c->cw * c->ch; k++)
        {
          if (c->map[k] != NX_CELL_NONE)
            {
              r.pt1.x = NX_CELL_X(c->map[k]) << sh;
              r.pt1.y = NX_CELL_Y(c->map[k]) << sh;
              r.pt2.x = r.pt1.x + (1 << sh) - 1;
              r.pt2.y = r.pt1.y + (1 << sh) - 1;
              n += nxgl_rectoverlap(&r, (FAR struct nxgl_rect_s *)keep);
            }
        }

      if (n == 0)
        {
          continue;
        }

      old = kmm_malloc(c->cw * c->ch * sizeof(uint16_t));
      if (old == NULL)
        {
          continue;
        }

      memcpy(old, c->map, c->cw * c->ch * sizeof(uint16_t));
      for (k = 0; k < c->cw * c->ch; k++)
        {
          if (c->map[k] != NX_CELL_NONE)
            {
              r.pt1.x = NX_CELL_X(c->map[k]) << sh;
              r.pt1.y = NX_CELL_Y(c->map[k]) << sh;
              r.pt2.x = r.pt1.x + (1 << sh) - 1;
              r.pt2.y = r.pt1.y + (1 << sh) - 1;
              if (nxgl_rectoverlap(&r, (FAR struct nxgl_rect_s *)keep))
                {
                  c->map[k] = NX_CELL_NONE;
                }
              else
                {
                  old[k] = NX_CELL_NONE;      /* (stays) */
                }
            }
          else
            {
              old[k] = NX_CELL_NONE;
            }
        }

      if (g_nxaccel->cells_alloc(c->cw, c->ch, c->map) >= 0)
        {
          for (k = 0; k < c->cw * c->ch; k++)
            {
              if (old[k] == NX_CELL_NONE)
                {
                  continue;
                }

              r.pt1.x = NX_CELL_X(old[k]) << sh;
              r.pt1.y = NX_CELL_Y(old[k]) << sh;
              r.pt2.x = r.pt1.x + (1 << sh) - 1;
              r.pt2.y = r.pt1.y + (1 << sh) - 1;
              off.x   = (NX_CELL_X(c->map[k]) << sh) - r.pt1.x;
              off.y   = (NX_CELL_Y(c->map[k]) << sh) - r.pt1.y;
              g_nxaccel->move(&r, &off);
            }

          g_nxaccel->cells_free(c->cw * c->ch, old);
          kmm_free(old);
          cells_moved(wnd);
          continue;
        }

      /* No room for them: the rest of the cells go too */

      g_nxaccel->cells_free(c->cw * c->ch, old);
      kmm_free(old);
      g_nxaccel->cells_free(c->cw * c->ch, c->map);
      cells_delete(c);
      wnd->cells   = NULL;
      wnd->surface = false;
#ifdef CONFIG_NX_COMPOSITOR
      nxbe_srccache_free(wnd);
      nxbe_compose_changed();
      if ((wnd->fx.flags & NX_FX_PARK) != 0 && !nxbe_compose_seen(wnd))
        {
          wnd->parked = true;
          g_nxbe_counts.parks++;
          continue;
        }
#endif

      nxbe_surface_tomemory(wnd);
    }
}

/****************************************************************************
 * Name: nxbe_surface_take
 *
 * Description:
 *   A surface of its own (a display buffer, staging, an image window's):
 *   where there is room, else over windows' cells, which then move
 *   elsewhere.
 *
 ****************************************************************************/

bool nxbe_surface_take(nxgl_coord_t w, nxgl_coord_t h, int kind,
                       FAR struct nxgl_point_s *pos)
{
  struct nxgl_rect_s r;

  if (g_nxaccel->surface_alloc(w, h, kind, pos) >= 0)
    {
      return true;
    }

  if (g_nxaccel->cells_alloc == NULL ||
      g_nxaccel->surface_alloc(w, h, kind | NX_SURFACE_DISPLACE, pos) < 0)
    {
      return false;
    }

  r.pt1   = *pos;
  r.pt2.x = pos->x + w - 1;
  r.pt2.y = pos->y + h - 1;
  surf_cells_out(&r);
  return true;
}

/* A surface of its own for the window (an image window's), making room
 * if make is set
 */

static bool surf_contig(FAR struct nxbe_window_s *wnd, nxgl_coord_t w,
                        nxgl_coord_t h, bool make,
                        FAR struct nxgl_point_s *pos)
{
  while (g_nxaccel->surface_alloc(w, h, wnd->surfkind, pos) < 0)
    {
      if (!make)
        {
          return false;
        }

      if (!surf_room(wnd))
        {
          return nxbe_surface_take(w, h, wnd->surfkind, pos);
        }
    }

  return true;
}

static bool surf_open(FAR struct nxbe_window_s *wnd, bool make)
{
  FAR struct nxbe_cells_s *c;
  struct nxgl_point_s pos;
  nxgl_coord_t w = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
  nxgl_coord_t h = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;
  int sh;
  int cw;
  int ch;

  if (g_nxaccel == NULL || g_nxaccel->surface_alloc == NULL ||
      w <= 0 || h <= 0)
    {
      return false;
    }

  if (g_nxaccel->cells_alloc == NULL || wnd->surfkind == NX_SURFACE_IMAGE)
    {
      if (!surf_contig(wnd, w, h, make, &pos))
        {
          return false;
        }

      c = cells_new(0, 0, w, h);
      if (c == NULL)
        {
          g_nxaccel->surface_free(&pos, w, h);
          return false;
        }

      wnd->surfpos = pos;
    }
  else
    {
      sh = g_nxaccel->cellshift;
      cw = (w + (1 << sh) - 1) >> sh;
      ch = (h + (1 << sh) - 1) >> sh;
      if (cw > 255 || ch > 255 || (c = cells_new(cw, ch, w, h)) == NULL)
        {
          return false;
        }

      if (!surf_cells(wnd, c, make))
        {
          cells_delete(c);
          return false;
        }
    }

  wnd->cells      = c;
  wnd->surface    = true;
  wnd->surfsize.w = w;
  wnd->surfsize.h = h;
  cells_build(wnd, c);
#ifdef CONFIG_NX_COMPOSITOR
  nxbe_compose_changed();
#endif
  return true;
}

/* The surface goes back to the hardware */

static void surf_free(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_cells_s *c = wnd->cells;

  if (c != NULL && c->cw > 0)
    {
      g_nxaccel->cells_free(c->cw * c->ch, c->map);
    }
  else
    {
      g_nxaccel->surface_free(&wnd->surfpos, wnd->surfsize.w,
                              wnd->surfsize.h);
    }

  cells_delete(c);
  wnd->cells   = NULL;
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
  wnd->parked = false;
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
 *   the old one had where both overlap (zero elsewhere).  Made of cells,
 *   it keeps those both sizes have and takes or gives back the rest.  One
 *   surface (an image window's): if there is no room for both, the old
 *   one goes first, and its content with it (all zero, *lost is set).
 *
 * Returned Value:
 *   false if there is no room: the window has no surface any more.
 *
 ****************************************************************************/

static bool surf_resize_contig(FAR struct nxbe_window_s *wnd,
                               FAR bool *lost)
{
  FAR struct nxbe_cells_s *c = wnd->cells;
  struct nxgl_point_s oldpos = wnd->surfpos;
  struct nxgl_size_s oldsize = wnd->surfsize;
  struct nxgl_point_s offset;
  struct nxgl_rect_s rect;

  /* A new one beside the old, else in the old one's place too, else
   * making room
   */

  wnd->surface = false;
  wnd->cells   = NULL;
  cells_delete(c);
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

bool nxbe_surface_resize(FAR struct nxbe_window_s *wnd, FAR bool *lost)
{
  FAR struct nxbe_cells_s *c = wnd->cells;
  FAR struct nxbe_cells_s *nc;
  struct nxgl_rect_s rect;
  nxgl_coord_t w = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
  nxgl_coord_t h = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;
  nxgl_coord_t oldw = wnd->surfsize.w;
  nxgl_coord_t oldh = wnd->surfsize.h;
  int sh;
  int cw;
  int ch;
  int i;
  int j;

#ifdef CONFIG_NX_COMPOSITOR
  nxbe_srccache_free(wnd);
#endif

  *lost = false;
  if (c == NULL || c->cw == 0)
    {
      return surf_resize_contig(wnd, lost);
    }

  sh = g_nxaccel->cellshift;
  cw = (w + (1 << sh) - 1) >> sh;
  ch = (h + (1 << sh) - 1) >> sh;
  nc = cw > 255 || ch > 255 ? NULL : cells_new(cw, ch, w, h);
  if (nc == NULL)
    {
      surf_free(wnd);
      return false;
    }

  /* All new cells, if there are without making room (placed together,
   * not round the old ones): what it had copied there, the old cells
   * given back
   */

  if (g_nxaccel->cells_alloc(cw, ch, nc->map) >= 0)
    {
      FAR const struct nxbe_piece_s *op;
      struct nxgl_rect_s x;
      struct nxgl_rect_s abs;
      struct nxgl_rect_s nabs;
      struct nxgl_rect_s y;
      struct nxgl_point_s off;
      int nop;
      int k;
      int m;

      cells_build(wnd, nc);
      rect.pt1.x = 0;
      rect.pt1.y = 0;
      rect.pt2.x = MIN(w, oldw) - 1;
      rect.pt2.y = MIN(h, oldh) - 1;
      nop = nxbe_surface_pieces(wnd, &op);
      for (k = 0; k < nop; k++)
        {
          if (!piece_part(&op[k], &rect, &x, &abs))
            {
              continue;
            }

          for (m = 0; m < nc->npieces; m++)
            {
              if (piece_part(&nc->pieces[m], &x, &y, &nabs))
                {
                  struct nxgl_rect_s from;

                  nxgl_rectoffset(&from, &y, abs.pt1.x - x.pt1.x,
                                  abs.pt1.y - x.pt1.y);
                  off.x = nabs.pt1.x - from.pt1.x;
                  off.y = nabs.pt1.y - from.pt1.y;
                  g_nxaccel->move(&from, &off);
                }
            }
        }

      g_nxaccel->cells_free(c->cw * c->ch, c->map);
      cells_delete(c);
      wnd->cells      = nc;
      wnd->surfsize.w = w;
      wnd->surfsize.h = h;
      cells_moved(wnd);
#ifdef CONFIG_NX_COMPOSITOR
      nxbe_compose_freed();
#endif
      goto zero;
    }

  /* The cells both sizes have stay; the others go back */

  for (j = 0; j < MIN(ch, c->ch); j++)
    {
      for (i = 0; i < MIN(cw, c->cw); i++)
        {
          nc->map[j * cw + i]    = c->map[j * c->cw + i];
          c->map[j * c->cw + i]  = NX_CELL_NONE;
        }
    }

  g_nxaccel->cells_free(c->cw * c->ch, c->map);
  cells_delete(c);
  wnd->cells      = nc;
  wnd->surfsize.w = w;
  wnd->surfsize.h = h;
#ifdef CONFIG_NX_COMPOSITOR
  nxbe_compose_freed();
#endif

  if (!surf_cells(wnd, nc, true))
    {
      surf_free(wnd);
      return false;
    }

  cells_build(wnd, nc);
  cells_moved(wnd);

zero:

  /* Zero the new territory, right and bottom */

  if (w > oldw)
    {
      rect.pt1.x = oldw;
      rect.pt1.y = 0;
      rect.pt2.x = w - 1;
      rect.pt2.y = h - 1;
      nxbe_surface_fill(wnd, &rect, 0);
    }

  if (h > oldh)
    {
      rect.pt1.x = 0;
      rect.pt1.y = oldh;
      rect.pt2.x = w - 1;
      rect.pt2.y = h - 1;
      nxbe_surface_fill(wnd, &rect, 0);
    }

#ifdef CONFIG_NX_COMPOSITOR
  nxbe_compose_changed();
#endif
  return true;
}

/****************************************************************************
 * Name: nxbe_surface_evacuate
 *
 * Description:
 *   The display has grown over keep (a display mode): move every surface
 *   out of it, the largest first, and every window's cell in it.  A
 *   window whose surface finds no room elsewhere goes to memory, and its
 *   client is asked to draw it again.
 *
 ****************************************************************************/

/* A window's own surface (an image window's) elsewhere: false if there is
 * no room (it has none then, nor its content)
 */

static bool surf_relocate(FAR struct nxbe_window_s *wnd)
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
      cells_delete(wnd->cells);
      wnd->cells   = NULL;
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
  wnd->cells->dirty = true;
  return true;
}

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
          if (!wnd->surface || wnd->cells == NULL || wnd->cells->cw > 0)
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
          break;
        }

      if (surf_relocate(big))
        {
          continue;
        }

#ifdef CONFIG_NX_COMPOSITOR
      /* Rather than in memory, parked if it may be */

      if ((big->fx.flags & NX_FX_PARK) != 0 && !nxbe_compose_seen(big))
        {
          big->parked = true;
          g_nxbe_counts.parks++;
          continue;
        }
#endif

      nxbe_surface_tomemory(big);
    }

  if (g_nxaccel->cells_alloc != NULL)
    {
      surf_cells_out(keep);
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
 *   A window that is not seen gives up its surface (or its memory): its
 *   pixels are dropped, and its client will be asked to draw it again when
 *   it is seen.
 *
 ****************************************************************************/

bool nxbe_surface_park(FAR struct nxbe_window_s *wnd)
{
  if (wnd->parked)
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
      wnd->fbmem = NULL;
    }
  else
    {
      nxbe_srccache_free(wnd);
      surf_free(wnd);
    }

  wnd->parked = true;
  g_nxbe_counts.parks++;
  nxbe_compose_changed();
  ginfo("Parked %p (%dx%d)\n", wnd, wnd->surfsize.w, wnd->surfsize.h);
  return true;
}

/****************************************************************************
 * Name: nxbe_surface_unpark
 *
 * Description:
 *   A parked window is seen again: a surface for it (making room, as for
 *   a new window), and its client asked to draw it again.  With no room
 *   at all, it goes to memory.
 *
 ****************************************************************************/

void nxbe_surface_unpark(FAR struct nxbe_window_s *wnd)
{
  struct nxgl_rect_s rect;

  if (!wnd->parked)
    {
      return;
    }

  /* Other windows may be parked to make room, not this one */

  wnd->parked = false;
  wnd->unseen = 0;
  g_nxbe_counts.unparks++;
  if (surf_open(wnd, true))
    {
      rect.pt1.x = 0;
      rect.pt1.y = 0;
      rect.pt2.x = wnd->surfsize.w - 1;
      rect.pt2.y = wnd->surfsize.h - 1;
      nxbe_surface_fill(wnd, &rect, 0);
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

  nxbe_compose_changed();
  nxbe_compose_redrawing(wnd);
  nxmu_redrawreq(wnd, &wnd->bounds);
}

/****************************************************************************
 * Name: nxbe_surface_lost
 *
 * Description:
 *   The hardware lost what its memory held (nx_accel_s lost: another GPU
 *   ran for a while): every window with a surface is asked to draw itself
 *   again (its client's redraw: hidden windows holding pictures too, whose
 *   clients take that as their pixels lost), the sources copied from them
 *   are forgotten, and the display is composited afresh.  Windows in
 *   memory and parked ones need nothing.
 *
 ****************************************************************************/

void nxbe_surface_lost(void)
{
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s rect;

  for (wnd = g_ramwnds; wnd != NULL; wnd = wnd->surfnext)
    {
      if (!wnd->surface)
        {
          continue;
        }

      nxbe_srccache_free(wnd);
      rect.pt1.x = 0;
      rect.pt1.y = 0;
      rect.pt2.x = wnd->surfsize.w - 1;
      rect.pt2.y = wnd->surfsize.h - 1;
      nxbe_surface_fill(wnd, &rect, 0);
      nxbe_compose_redrawing(wnd);
      nxmu_redrawreq(wnd, &wnd->bounds);
    }

  nxbe_compose_setmode(true);
  nxbe_compose_setmode(false);
}

/****************************************************************************
 * Name: nxbe_surface_frommemory
 *
 * Description:
 *   Windows in memory move into surfaces if there is room for them now
 *   (those that are seen first): their pixels are put there, the memory
 *   freed.  Returns how many.
 *
 ****************************************************************************/

int nxbe_surface_frommemory(void)
{
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s rect;
  int pass;
  int n = 0;

  for (pass = 0; pass < 2; pass++)
    {
      for (wnd = g_ramwnds; wnd != NULL; wnd = wnd->surfnext)
        {
          if (wnd->surface || wnd->fbmem == NULL ||
              !NXBE_ISRAMBACKED(wnd) || wnd->parked ||
              wnd->be->plane[0].pinfo.bpp != 16 ||
              nxbe_compose_seen(wnd) != (pass == 0))
            {
              continue;
            }

          if (!surf_open(wnd, false))
            {
              continue;
            }

          rect.pt1.x = 0;
          rect.pt1.y = 0;
          rect.pt2.x = wnd->surfsize.w - 1;
          rect.pt2.y = wnd->surfsize.h - 1;
          nxbe_surface_put(wnd, &rect, wnd->fbmem, wnd->stride);
          kumm_free(wnd->fbmem);
          wnd->fbmem = NULL;
          g_nxbe_counts.tosurface++;
          nxbe_compose_damage_window(wnd);
          n++;
        }
    }

  return n;
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
