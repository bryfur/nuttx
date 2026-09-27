/****************************************************************************
 * graphics/nxbe/nxbe_compose.c
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

/* The compositor (CONFIG_NX_COMPOSITOR).
 *
 * Nothing is drawn on the display directly.  Whatever changes it (a window
 * drawn into, moved, raised, opened, closed, given effects) only records
 * the rectangle of the display that is now out of date: the damage.  When
 * the server has no more messages waiting (or damage has waited
 * NXCOMP_LATENCY), and at most once a frame (NXCOMP_FRAME), a frame is
 * composited into the back buffer and flipped to:
 *
 *   - the back buffer is brought up to date first: it is the frame before
 *     last, so what the last frame changed (its buffer age) is copied over
 *     from the front buffer, except where this frame redraws anyway;
 *   - a window that was moved and is opaque is copied as it was
 *     composited, from where it was in the front buffer to where it is in
 *     the back: one copy instead of drawing it and all that is below it
 *     again; only its round corners, what it uncovered and its shadow's
 *     margins become damage;
 *   - each damaged rectangle is composited from the lowest window that
 *     matters to the top one:
 *       - the background: its colour;
 *       - a window's shadow: black, blended with the shape table
 *         NX_LUT_SHADOW around the window's rounded rectangle (only where
 *         the window does not cover it);
 *       - its backdrop: a tint blended behind it (translucent panels);
 *       - its pixels: copied from its surface, the round corners (and the
 *         whole window, if it has an opacity or a colour key) blended with
 *         the shape table NX_LUT_CORNER;
 *       - or, for a layer window, its display list, replayed;
 *     the lowest window that matters is the top one that covers the
 *     rectangle with opaque pixels: typing in a terminal on top costs one
 *     copy from its surface (and one more of the last frame's change);
 *   - the back buffer is flipped to (nx_accel_s flip): the hardware shows
 *     it from the next frame, so no partial state is ever seen.
 *
 * The two display buffers are the display at (0, 0) and a surface of the
 * current mode's size.  When a window's surface does not fit, the second
 * buffer is given up (nxbe_compose_reclaim()) and the display composited
 * in place, until there is room again.  Without flip() it is always so.
 *
 * Everything is the display hardware's operations (nx_accel_s): the
 * server only computes rectangles.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nx/nxaccel.h>
#include <nuttx/nx/nxcompose.h>

#include "nxbe.h"

#ifdef CONFIG_NX_COMPOSITOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXCOMP_R          NX_COMPOSITOR_RADIUS
#define NXCOMP_LUTMAX     (NX_BLEND_LUT_N - 1)
#define NXCOMP_SIGMA      6.0f    /* The shadow's blur */
#define NXCOMP_LATENCY    MSEC2TICK(20)
#define NXCOMP_FRAME      MSEC2TICK(CONFIG_NX_COMPOSITOR_FRAMEMS)
#define NXCOMP_MAXDAMAGE  CONFIG_NX_COMPOSITOR_MAXDAMAGE
#define NXCOMP_MAXMOVES   4
#define NXCOMP_MAXPREV    (NXCOMP_MAXDAMAGE + NXCOMP_MAXMOVES)
#define NXCOMP_MAXPIECES  32      /* Buffer age: pieces of one rectangle */
#define NXCOMP_MAXADD     64      /* Damage from moves, in one frame */
#define NXCOMP_STAGEH     16      /* Staging: lines of a window in memory */
#define NXCOMP_MAXCOVER   24      /* Parking: opaque windows over one */
#define NXCOMP_IDLEPOLL   MSEC2TICK(50)    /* Parking: looked at so often */
#define NXCOMP_QUIET      MSEC2TICK(400)   /* ... and not while windows move */
#define NXCOMP_PARKHIDDEN MSEC2TICK(500)   /* ... hidden so long */
#define NXCOMP_PARKCOVER  MSEC2TICK(1000)  /* ... covered so long */
#define NXCOMP_REDRAWHOLD MSEC2TICK(50)    /* A window being drawn again:
                                            * the frame waits for it */
#define NXCOMP_PACKAGAIN  MSEC2TICK(5000)  /* Surfaces packed for a second
                                            * buffer at most this often */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A window moved since the last frame, and where it was then */

struct nxcomp_move_s
{
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s from;
};

/* A copy of composited pixels from the front buffer to the back one
 * (display coordinates)
 */

struct nxcomp_copy_s
{
  struct nxgl_rect_s src;
  struct nxgl_rect_s dest;
};

struct nxcomp_s
{
  FAR struct nxbe_state_s *be;
  bool on;                                /* Compositing */
  bool dbuf;                              /* Two display buffers */
  bool retry;                             /* Surfaces freed: try for two */
  uint8_t front;                          /* buf[front] is shown */
  struct nxgl_point_s buf[2];             /* buf[0]: the display, (0, 0);
                                           * buf[1]: a surface */
  struct nxgl_size_s bufsize;             /* buf[1]'s size */
  bool stage;                             /* stagepos: staging surface */
  struct nxgl_point_s stagepos;
  struct nxgl_size_s stagesize;
  clock_t since;                          /* When the oldest damage came */
  clock_t lastflip;
  clock_t lastmove;                       /* When a window last moved */
  clock_t lastidle;                       /* Parking: looked at then */
  clock_t holduntil;                      /* A frame waits until then */
  bool hold;                              /* ... for windows being drawn */
  bool changed;                           /* The statistics are old */
  bool packed;                            /* Packed for a second buffer */
  bool packlater;                         /* ... to be tried again */
  clock_t lastpack;                       /* ... then */
  uint16_t nparked;                       /* Windows parked */
  int ndamage;
  int nprev;
  int nmoves;
  struct nxgl_rect_s damage[NXCOMP_MAXDAMAGE];
  struct nxgl_rect_s prev[NXCOMP_MAXPREV];  /* The last frame's changes */
  struct nxcomp_move_s moves[NXCOMP_MAXMOVES];
  uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nxcomp_s g_comp;

/* The state of the windows, for nxbe_compose_stats() (another thread):
 * made by the server when it has changed
 */

static struct nx_compose_stats_s g_snap;

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct nxbe_counts_s g_nxbe_counts;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool rect_empty(FAR const struct nxgl_rect_s *r)
{
  return r->pt1.x > r->pt2.x || r->pt1.y > r->pt2.y;
}

static inline int32_t rect_area(FAR const struct nxgl_rect_s *r)
{
  return rect_empty(r) ? 0 : (int32_t)(r->pt2.x - r->pt1.x + 1) *
                             (r->pt2.y - r->pt1.y + 1);
}

static inline bool rect_contains(FAR const struct nxgl_rect_s *outer,
                                 FAR const struct nxgl_rect_s *inner)
{
  return inner->pt1.x >= outer->pt1.x && inner->pt2.x <= outer->pt2.x &&
         inner->pt1.y >= outer->pt1.y && inner->pt2.y <= outer->pt2.y;
}

static inline void rect_set(FAR struct nxgl_rect_s *r, nxgl_coord_t x1,
                            nxgl_coord_t y1, nxgl_coord_t x2,
                            nxgl_coord_t y2)
{
  r->pt1.x = x1;
  r->pt1.y = y1;
  r->pt2.x = x2;
  r->pt2.y = y2;
}

/* Enough arithmetic for 8-bit tables, without a maths library */

static float lut_sqrt(float x)
{
  float r = x > 1.0f ? x : 1.0f;
  int i;

  if (x <= 0.0f)
    {
      return 0.0f;
    }

  for (i = 0; i < 24; i++)
    {
      r = 0.5f * (r + x / r);
    }

  return r;
}

static float lut_exp_neg(float a)
{
  /* e^-a, a >= 0: (e^(-a / 64))^64, the small one by its series */

  float b = a / 64.0f;
  float y = 1.0f - b + b * b / 2.0f - b * b * b / 6.0f +
            b * b * b * b / 24.0f;
  int i;

  for (i = 0; i < 6; i++)
    {
      y *= y;
    }

  return y;
}

/* The shape tables.  Distances are from the inner rectangle to the pixel:
 * a pixel ddx to the left of it spans [-ddx, -ddx + 1) from its edge.
 */

static float lut_outside(float u)
{
  return u < 0.0f ? -u : 0.0f;
}

/* Coverage of a disc of radius NXCOMP_R (8 x 8 samples a pixel) */

static void lut_corner(uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N])
{
  const float r2 = (float)NXCOMP_R * NXCOMP_R;
  int dx;
  int dy;
  int i;
  int j;

  for (dy = 0; dy < NX_BLEND_LUT_N; dy++)
    {
      for (dx = 0; dx < NX_BLEND_LUT_N; dx++)
        {
          int n = 0;

          for (j = 0; j < 8; j++)
            {
              for (i = 0; i < 8; i++)
                {
                  float u = lut_outside(-dx + (i + 0.5f) / 8.0f);
                  float v = lut_outside(-dy + (j + 0.5f) / 8.0f);

                  n += u * u + v * v <= r2;
                }
            }

          lut[dy][dx] = (uint8_t)((n * 255 + 32) / 64);
        }
    }
}

/* The edge of the same disc, one pixel wide */

static void lut_ring(uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N])
{
  const float r2 = (float)NXCOMP_R * NXCOMP_R;
  const float q2 = (float)(NXCOMP_R - 1) * (NXCOMP_R - 1);
  int dx;
  int dy;
  int i;
  int j;

  for (dy = 0; dy < NX_BLEND_LUT_N; dy++)
    {
      for (dx = 0; dx < NX_BLEND_LUT_N; dx++)
        {
          int n = 0;

          for (j = 0; j < 8; j++)
            {
              for (i = 0; i < 8; i++)
                {
                  float u = lut_outside(-dx + (i + 0.5f) / 8.0f);
                  float v = lut_outside(-dy + (j + 0.5f) / 8.0f);
                  float d2 = u * u + v * v;

                  n += d2 <= r2 && d2 > q2;
                }
            }

          lut[dy][dx] = (uint8_t)((n * 255 + 32) / 64);
        }
    }
}

/* A soft shadow: the rounded rectangle blurred by a Gaussian (the signed
 * distance to its edge through the error function)
 */

static float lut_erfc(float x)
{
  /* Abramowitz and Stegun 7.1.26 */

  float ax = x < 0.0f ? -x : x;
  float t = 1.0f / (1.0f + 0.3275911f * ax);
  float y = t * (0.254829592f + t * (-0.284496736f + t * (1.421413741f +
            t * (-1.453152027f + t * 1.061405429f)))) * lut_exp_neg(ax * ax);

  return x >= 0.0f ? y : 2.0f - y;
}

static void lut_shadow(uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N])
{
  int dx;
  int dy;

  for (dy = 0; dy < NX_BLEND_LUT_N; dy++)
    {
      for (dx = 0; dx < NX_BLEND_LUT_N; dx++)
        {
          float u = lut_outside(-dx + 0.5f);
          float v = lut_outside(-dy + 0.5f);
          float d = lut_sqrt(u * u + v * v) - NXCOMP_R;
          float a = 0.5f * lut_erfc(d / (NXCOMP_SIGMA * 1.41421356f));

          if (dx == NXCOMP_LUTMAX || dy == NXCOMP_LUTMAX)
            {
              a = 0.0f;                   /* Beyond: nothing */
            }

          lut[dy][dx] = (uint8_t)(a * 255.0f + 0.5f);
        }
    }
}

/* Windows ******************************************************************/

static inline bool wnd_round(FAR struct nxbe_window_s *wnd)
{
  return (wnd->fx.flags & NX_FX_ROUND) != 0 &&
         wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1 >= 2 * NXCOMP_R &&
         wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1 >= 2 * NXCOMP_R;
}

static inline bool wnd_pixels(FAR struct nxbe_window_s *wnd)
{
  return !NXBE_ISLAYER(wnd) && !NXBE_ISIMAGE(wnd) &&
         (wnd->surface || wnd->fbmem != NULL);
}

/* The shadow's inner rectangle (the window's rounded rectangle, inset by
 * the radius, moved down) and how far it reaches, with the window at b
 */

static void wnd_shadow_at(FAR struct nxbe_window_s *wnd,
                          FAR const struct nxgl_rect_s *b,
                          FAR struct nxgl_rect_s *inner,
                          FAR struct nxgl_rect_s *reach)
{
  rect_set(inner, b->pt1.x + NXCOMP_R,
           b->pt1.y + NXCOMP_R + wnd->fx.shadowdy,
           b->pt2.x - NXCOMP_R,
           b->pt2.y - NXCOMP_R + wnd->fx.shadowdy);
  rect_set(reach, inner->pt1.x - NXCOMP_LUTMAX + 1,
           inner->pt1.y - NXCOMP_LUTMAX + 1,
           inner->pt2.x + NXCOMP_LUTMAX - 1,
           inner->pt2.y + NXCOMP_LUTMAX - 1);
}

static inline void wnd_shadow(FAR struct nxbe_window_s *wnd,
                              FAR struct nxgl_rect_s *inner,
                              FAR struct nxgl_rect_s *reach)
{
  wnd_shadow_at(wnd, &wnd->bounds, inner, reach);
}

/* All of the display a window touches (its bounds and its shadow), with
 * the window at b
 */

static void wnd_extent_at(FAR struct nxbe_window_s *wnd,
                          FAR const struct nxgl_rect_s *b,
                          FAR struct nxgl_rect_s *ext)
{
  struct nxgl_rect_s inner;
  struct nxgl_rect_s reach;

  *ext = *b;
  if (wnd->fx.shadow > 0)
    {
      wnd_shadow_at(wnd, b, &inner, &reach);
      nxgl_rectunion(ext, ext, &reach);
    }
}

static inline void wnd_extent(FAR struct nxbe_window_s *wnd,
                              FAR struct nxgl_rect_s *ext)
{
  wnd_extent_at(wnd, &wnd->bounds, ext);
}

/* Does the window hide everything below it in r? */

static bool wnd_covers(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *r)
{
  FAR const struct nxgl_rect_s *b = &wnd->bounds;

  if (wnd == &g_comp.be->bkgd)
    {
      return true;
    }

  if (!wnd_pixels(wnd) || wnd->fx.opacity != 255 ||
      (wnd->fx.flags & NX_FX_KEY) != 0 || !rect_contains(b, r))
    {
      return false;
    }

  if (wnd_round(wnd))
    {
      /* Not if r reaches into a corner square */

      bool left   = r->pt1.x < b->pt1.x + NXCOMP_R;
      bool right  = r->pt2.x > b->pt2.x - NXCOMP_R;
      bool top    = r->pt1.y < b->pt1.y + NXCOMP_R;
      bool bottom = r->pt2.y > b->pt2.y - NXCOMP_R;

      if ((left || right) && (top || bottom))
        {
          return false;
        }
    }

  return true;
}

/* Stacking: a window goes to the top (or the bottom) of its layer */

static void wnd_unlink(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_state_s *be = wnd->be;

  if (wnd->above != NULL)
    {
      wnd->above->below = wnd->below;
    }
  else
    {
      be->topwnd = wnd->below;
    }

  wnd->below->above = wnd->above;
}

static void wnd_insert_above(FAR struct nxbe_window_s *wnd,
                             FAR struct nxbe_window_s *below)
{
  FAR struct nxbe_state_s *be = wnd->be;

  wnd->below = below;
  wnd->above = below->above;
  if (below->above != NULL)
    {
      below->above->below = wnd;
    }
  else
    {
      be->topwnd = wnd;
    }

  below->above = wnd;
}

/* Operations ***************************************************************/

/* Composited rectangles are the display's moved by an offset: to the back
 * buffer's position
 */

static void op_fill(FAR const struct nxgl_rect_s *dest, nxgl_mxpixel_t c)
{
  g_nxaccel->fill(dest, c);
}

static void op_copy(FAR const struct nxgl_point_s *src,
                    FAR const struct nxgl_rect_s *dest)
{
  struct nxgl_rect_s srect;
  struct nxgl_point_s offset;

  rect_set(&srect, src->x, src->y,
           src->x + dest->pt2.x - dest->pt1.x,
           src->y + dest->pt2.y - dest->pt1.y);
  offset.x = dest->pt1.x - src->x;
  offset.y = dest->pt1.y - src->y;
  if (offset.x != 0 || offset.y != 0)
    {
      g_nxaccel->move(&srect, &offset);
    }
}

/* A window's pixels at display rectangle r (inside its bounds) into dest
 * (the same size): from its surface, or from its memory
 */

static void op_window(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *r,
                      FAR const struct nxgl_rect_s *dest)
{
  struct nxgl_point_s src;

  if (wnd->surface)
    {
      src.x = wnd->surfpos.x + r->pt1.x - wnd->bounds.pt1.x;
      src.y = wnd->surfpos.y + r->pt1.y - wnd->bounds.pt1.y;
      op_copy(&src, dest);
    }
  else if (wnd->fbmem != NULL)
    {
      struct nxgl_point_s origin;
      unsigned int bpp = wnd->be->plane[0].pinfo.bpp;
      FAR const uint8_t *pix = (FAR const uint8_t *)wnd->fbmem +
                               (r->pt1.y - wnd->bounds.pt1.y) * wnd->stride +
                               (((r->pt1.x - wnd->bounds.pt1.x) * bpp) >> 3);

      origin = dest->pt1;
      g_nxaccel->copy(dest, pix, &origin, wnd->stride);
    }
}

static void op_blend_init(FAR struct nx_blend_s *b,
                          FAR const struct nxgl_rect_s *dest)
{
  memset(b, 0, sizeof(*b));
  b->dest  = *dest;
  b->alpha = 255;
}

/* LUT shape parameters: an inner rectangle moved by off */

static void op_inner(FAR struct nx_blend_s *b,
                     FAR const struct nxgl_rect_s *inner,
                     FAR const struct nxgl_point_s *off)
{
  b->p[0] = inner->pt1.x + off->x;
  b->p[1] = inner->pt1.y + off->y;
  b->p[2] = inner->pt2.x + off->x;
  b->p[3] = inner->pt2.y + off->y;
}

/* Draw a window's shadow in display rectangle r */

static void comp_shadow(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *r,
                        FAR const struct nxgl_point_s *off)
{
  FAR const struct nxgl_rect_s *w = &wnd->bounds;
  struct nxgl_rect_s inner;
  struct nxgl_rect_s reach;
  struct nxgl_rect_s e;
  struct nxgl_rect_s part[8];
  struct nx_blend_s b;
  int n = 0;
  int i;

  wnd_shadow(wnd, &inner, &reach);
  nxgl_rectintersect(&e, &reach, r);
  if (rect_empty(&e))
    {
      return;
    }

  /* e minus the window: above, below, left and right of it ... */

  rect_set(&part[n++], e.pt1.x, e.pt1.y, e.pt2.x, MIN(e.pt2.y, w->pt1.y - 1));
  rect_set(&part[n++], e.pt1.x, MAX(e.pt1.y, w->pt2.y + 1), e.pt2.x, e.pt2.y);
  rect_set(&part[n++], e.pt1.x, MAX(e.pt1.y, w->pt1.y),
           MIN(e.pt2.x, w->pt1.x - 1), MIN(e.pt2.y, w->pt2.y));
  rect_set(&part[n++], MAX(e.pt1.x, w->pt2.x + 1), MAX(e.pt1.y, w->pt1.y),
           e.pt2.x, MIN(e.pt2.y, w->pt2.y));

  /* ... and its round corners, which show what is below them (only when
   * the window is opaque: through a translucent one the shadow would show
   * in the corner squares and not beside them)
   */

  if (wnd_round(wnd) && wnd_pixels(wnd) && wnd->fx.opacity == 255 &&
      wnd->fx.backdrop == 0 && (wnd->fx.flags & NX_FX_KEY) == 0)
    {
      rect_set(&part[n++], w->pt1.x, w->pt1.y,
               w->pt1.x + NXCOMP_R - 1, w->pt1.y + NXCOMP_R - 1);
      rect_set(&part[n++], w->pt2.x - NXCOMP_R + 1, w->pt1.y,
               w->pt2.x, w->pt1.y + NXCOMP_R - 1);
      rect_set(&part[n++], w->pt1.x, w->pt2.y - NXCOMP_R + 1,
               w->pt1.x + NXCOMP_R - 1, w->pt2.y);
      rect_set(&part[n++], w->pt2.x - NXCOMP_R + 1, w->pt2.y - NXCOMP_R + 1,
               w->pt2.x, w->pt2.y);
      for (i = 4; i < n; i++)
        {
          nxgl_rectintersect(&part[i], &part[i], &e);
        }
    }

  for (i = 0; i < n; i++)
    {
      if (rect_empty(&part[i]))
        {
          continue;
        }

      nxgl_rectoffset(&part[i], &part[i], off->x, off->y);
      op_blend_init(&b, &part[i]);
      b.colour = 0;
      b.alpha  = wnd->fx.shadow;
      b.mode   = NX_BLEND_SRC_COLOUR | NX_BLEND_MOD_LUT |
                 NX_BLEND_DITHER;
      b.slot   = NX_LUT_SHADOW;
      op_inner(&b, &inner, off);
      g_nxaccel->blend(&b);
    }
}

/* A small surface to blend the pixels of windows in memory from: their
 * round corners, opacity and colour key.  Made when first needed, as wide
 * as the widest display mode.
 */

static bool comp_stage(void)
{
  struct nxgl_size_s size;
  nxgl_coord_t w = g_comp.be->vinfo.xres;
  int i;

  if (!g_comp.stage)
    {
      for (i = 0; g_nxaccel->getmode != NULL &&
                  g_nxaccel->getmode(i, &size) >= 0; i++)
        {
          w = MAX(w, size.w);
        }

      g_comp.stage = g_nxaccel->surface_alloc(w, NXCOMP_STAGEH,
                                              NX_SURFACE_STAGE,
                                              &g_comp.stagepos) >= 0;
      g_comp.stagesize.w = w;
      g_comp.stagesize.h = NXCOMP_STAGEH;
    }

  return g_comp.stage;
}

static void comp_unstage(void)
{
  if (g_comp.stage)
    {
      g_nxaccel->surface_free(&g_comp.stagepos, g_comp.stagesize.w,
                              g_comp.stagesize.h);
      g_comp.stage = false;
    }
}

/* Display rectangle r of a window in memory, blended as from a surface:
 * a staging band at a time, its pixels put there first (without a
 * staging surface, just the pixels)
 */

static void comp_memblend(FAR struct nxbe_window_s *wnd,
                          FAR const struct nxgl_rect_s *r,
                          FAR const struct nxgl_point_s *off, bool corner,
                          FAR const struct nxgl_rect_s *inner)
{
  unsigned int bpp = wnd->be->plane[0].pinfo.bpp;
  struct nxgl_rect_s chunk;
  struct nxgl_rect_s stage;
  struct nxgl_rect_s dest;
  struct nx_blend_s b;
  nxgl_coord_t x;
  nxgl_coord_t y;

  if (!comp_stage())
    {
      nxgl_rectoffset(&dest, r, off->x, off->y);
      op_window(wnd, r, &dest);
      return;
    }

  for (y = r->pt1.y; y <= r->pt2.y; y += g_comp.stagesize.h)
    {
      for (x = r->pt1.x; x <= r->pt2.x; x += g_comp.stagesize.w)
        {
          FAR const uint8_t *pix;

          rect_set(&chunk, x, y,
                   MIN(r->pt2.x, x + g_comp.stagesize.w - 1),
                   MIN(r->pt2.y, y + g_comp.stagesize.h - 1));
          pix = (FAR const uint8_t *)wnd->fbmem +
                (chunk.pt1.y - wnd->bounds.pt1.y) * wnd->stride +
                (((chunk.pt1.x - wnd->bounds.pt1.x) * bpp) >> 3);
          rect_set(&stage, g_comp.stagepos.x, g_comp.stagepos.y,
                   g_comp.stagepos.x + chunk.pt2.x - chunk.pt1.x,
                   g_comp.stagepos.y + chunk.pt2.y - chunk.pt1.y);
          g_nxaccel->copy(&stage, pix, &stage.pt1, wnd->stride);

          nxgl_rectoffset(&dest, &chunk, off->x, off->y);
          op_blend_init(&b, &dest);
          b.src   = g_comp.stagepos;
          b.alpha = wnd->fx.opacity;
          b.mode  = NX_BLEND_SRC_RGB;
          if ((wnd->fx.flags & NX_FX_KEY) != 0)
            {
              b.mode |= NX_BLEND_KEY;
              b.key   = (uint16_t)wnd->fx.key;
            }

          if (corner)
            {
              b.mode |= NX_BLEND_MOD_LUT;
              b.slot  = NX_LUT_CORNER;
              op_inner(&b, inner, off);
            }

          g_nxaccel->blend(&b);
        }
    }
}

/* A window's pixels in display rectangle c (inside its bounds) */

static void comp_pixels(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *c,
                        FAR const struct nxgl_point_s *off)
{
  FAR const struct nxgl_rect_s *w = &wnd->bounds;
  bool plain = wnd->fx.opacity == 255 && (wnd->fx.flags & NX_FX_KEY) == 0;
  bool round = wnd_round(wnd);
  struct nxgl_rect_s inner;
  struct nxgl_rect_s part[7];
  struct nxgl_rect_s dest;
  struct nx_blend_s b;
  int ncorner = 0;
  int n = 0;
  int i;

  rect_set(&inner, w->pt1.x + NXCOMP_R, w->pt1.y + NXCOMP_R,
           w->pt2.x - NXCOMP_R, w->pt2.y - NXCOMP_R);

  if (round)
    {
      rect_set(&part[n++], w->pt1.x, w->pt1.y,
               w->pt1.x + NXCOMP_R - 1, w->pt1.y + NXCOMP_R - 1);
      rect_set(&part[n++], w->pt2.x - NXCOMP_R + 1, w->pt1.y,
               w->pt2.x, w->pt1.y + NXCOMP_R - 1);
      rect_set(&part[n++], w->pt1.x, w->pt2.y - NXCOMP_R + 1,
               w->pt1.x + NXCOMP_R - 1, w->pt2.y);
      rect_set(&part[n++], w->pt2.x - NXCOMP_R + 1, w->pt2.y - NXCOMP_R + 1,
               w->pt2.x, w->pt2.y);
      ncorner = n;
      rect_set(&part[n++], w->pt1.x + NXCOMP_R, w->pt1.y,
               w->pt2.x - NXCOMP_R, w->pt1.y + NXCOMP_R - 1);
      rect_set(&part[n++], w->pt1.x, w->pt1.y + NXCOMP_R,
               w->pt2.x, w->pt2.y - NXCOMP_R);
      rect_set(&part[n++], w->pt1.x + NXCOMP_R, w->pt2.y - NXCOMP_R + 1,
               w->pt2.x - NXCOMP_R, w->pt2.y);
    }
  else
    {
      part[n++] = *w;
    }

  for (i = 0; i < n; i++)
    {
      nxgl_rectintersect(&part[i], &part[i], c);
      if (rect_empty(&part[i]))
        {
          continue;
        }

      nxgl_rectoffset(&dest, &part[i], off->x, off->y);
      if (i >= ncorner && plain)
        {
          op_window(wnd, &part[i], &dest);
          continue;
        }

      if (!wnd->surface)
        {
          comp_memblend(wnd, &part[i], off, i < ncorner, &inner);
          continue;
        }

      op_blend_init(&b, &dest);
      b.src.x = wnd->surfpos.x + part[i].pt1.x - w->pt1.x;
      b.src.y = wnd->surfpos.y + part[i].pt1.y - w->pt1.y;
      b.alpha = wnd->fx.opacity;
      b.mode  = NX_BLEND_SRC_RGB;
      if ((wnd->fx.flags & NX_FX_KEY) != 0)
        {
          b.mode |= NX_BLEND_KEY;
          b.key   = (uint16_t)wnd->fx.key;
        }

      if (i < ncorner)
        {
          b.mode |= NX_BLEND_MOD_LUT;
          b.slot  = NX_LUT_CORNER;
          op_inner(&b, &inner, off);
        }

      g_nxaccel->blend(&b);
    }
}

/* A window's 1-pixel light edge (dark with NX_FX_DARKEDGE) where it meets
 * display rectangle c: its outer rows and columns and its corner squares,
 * blended with the ring
 */

static void comp_border(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *c,
                        FAR const struct nxgl_point_s *off)
{
  FAR const struct nxgl_rect_s *w = &wnd->bounds;
  struct nxgl_rect_s inner;
  struct nxgl_rect_s part[8];
  struct nx_blend_s b;
  int i;

  rect_set(&inner, w->pt1.x + NXCOMP_R, w->pt1.y + NXCOMP_R,
           w->pt2.x - NXCOMP_R, w->pt2.y - NXCOMP_R);
  rect_set(&part[0], w->pt1.x, w->pt1.y, w->pt2.x, w->pt1.y);
  rect_set(&part[1], w->pt1.x, w->pt2.y, w->pt2.x, w->pt2.y);
  rect_set(&part[2], w->pt1.x, w->pt1.y + 1, w->pt1.x, w->pt2.y - 1);
  rect_set(&part[3], w->pt2.x, w->pt1.y + 1, w->pt2.x, w->pt2.y - 1);
  rect_set(&part[4], w->pt1.x + 1, w->pt1.y + 1,
           w->pt1.x + NXCOMP_R - 1, w->pt1.y + NXCOMP_R - 1);
  rect_set(&part[5], w->pt2.x - NXCOMP_R + 1, w->pt1.y + 1,
           w->pt2.x - 1, w->pt1.y + NXCOMP_R - 1);
  rect_set(&part[6], w->pt1.x + 1, w->pt2.y - NXCOMP_R + 1,
           w->pt1.x + NXCOMP_R - 1, w->pt2.y - 1);
  rect_set(&part[7], w->pt2.x - NXCOMP_R + 1, w->pt2.y - NXCOMP_R + 1,
           w->pt2.x - 1, w->pt2.y - 1);

  for (i = 0; i < 8; i++)
    {
      nxgl_rectintersect(&part[i], &part[i], c);
      if (rect_empty(&part[i]))
        {
          continue;
        }

      nxgl_rectoffset(&part[i], &part[i], off->x, off->y);
      op_blend_init(&b, &part[i]);
      b.colour = (wnd->fx.flags & NX_FX_DARKEDGE) != 0 ? 0x0000 : 0xffff;
      b.alpha  = wnd->fx.border;
      b.mode   = NX_BLEND_SRC_COLOUR | NX_BLEND_MOD_LUT;
      b.slot   = NX_LUT_RING;
      op_inner(&b, &inner, off);
      g_nxaccel->blend(&b);
    }
}

/* One operation of a window's into dest (device coordinates), its image
 * source (if any) at spos; shape parameters are the window's moved to org
 */

static void comp_gpuop_do(FAR const struct nx_gpuop_s *op,
                          FAR const struct nxgl_point_s *org,
                          FAR const struct nxgl_rect_s *dest,
                          FAR const struct nxgl_point_s *spos)
{
  struct nx_blend_s b;
  uint8_t smode = op->mode & NX_BLEND_SRC_MASK;
  bool image = op->op == NX_GPUOP_COPY ||
               (op->op == NX_GPUOP_BLEND &&
                (smode != NX_BLEND_SRC_COLOUR ||
                 (op->mode & NX_BLEND_EXT) != 0));

  switch (op->op)
    {
      case NX_GPUOP_FILL:
        op_fill(dest, op->colour);
        break;

      case NX_GPUOP_COPY:
        op_copy(spos, dest);
        break;

      case NX_GPUOP_BLEND:
        op_blend_init(&b, dest);
        if (image)
          {
            b.src = *spos;
          }

        b.colour = op->colour;
        b.alpha  = op->alpha;
        b.mode   = op->mode;
        b.shift  = op->shift;
        b.slot   = op->slot;
        b.key    = op->key;
        switch (op->mode & NX_BLEND_MOD_MASK)
          {
            case NX_BLEND_MOD_LUT:
              b.p[0] = op->p[0] + org->x;
              b.p[1] = op->p[1] + org->y;
              b.p[2] = op->p[2] + org->x;
              b.p[3] = op->p[3] + org->y;
              break;

            case NX_BLEND_MOD_HRAMP:
              b.p[0] = op->p[0] + org->x;
              b.p[1] = op->p[1];
              b.p[2] = op->p[2];
              break;

            case NX_BLEND_MOD_VRAMP:
              b.p[0] = op->p[0] + org->y;
              b.p[1] = op->p[1];
              b.p[2] = op->p[2];
              break;

            default:
              break;
          }

        g_nxaccel->blend(&b);
        break;

      default:
        break;
    }
}

/* An image source in memory (a window with no room for a surface): a
 * staging band at a time, its pixels put there first
 */

static void comp_gpuop_mem(FAR struct nxbe_window_s *src,
                           FAR const struct nx_gpuop_s *op,
                           FAR const struct nxgl_point_s *org,
                           FAR const struct nxgl_rect_s *whole,
                           FAR const struct nxgl_rect_s *dest)
{
  unsigned int bpp = src->be->plane[0].pinfo.bpp;
  struct nxgl_rect_s chunk;
  struct nxgl_rect_s srect;
  struct nxgl_rect_s sbounds;
  struct nxgl_rect_s stage;
  nxgl_coord_t x;
  nxgl_coord_t y;

  if (src->fbmem == NULL || !comp_stage())
    {
      return;
    }

  rect_set(&sbounds, 0, 0, src->bounds.pt2.x - src->bounds.pt1.x,
           src->bounds.pt2.y - src->bounds.pt1.y);
  for (y = dest->pt1.y; y <= dest->pt2.y; y += g_comp.stagesize.h)
    {
      for (x = dest->pt1.x; x <= dest->pt2.x; x += g_comp.stagesize.w)
        {
          FAR const uint8_t *pix;

          /* The source of this piece, in the source window, inside it */

          rect_set(&srect, op->sx + x - whole->pt1.x,
                   op->sy + y - whole->pt1.y,
                   op->sx + MIN(dest->pt2.x, x + g_comp.stagesize.w - 1) -
                   whole->pt1.x,
                   op->sy + MIN(dest->pt2.y, y + g_comp.stagesize.h - 1) -
                   whole->pt1.y);
          nxgl_rectintersect(&srect, &srect, &sbounds);
          if (rect_empty(&srect))
            {
              continue;
            }

          rect_set(&chunk, whole->pt1.x + srect.pt1.x - op->sx,
                   whole->pt1.y + srect.pt1.y - op->sy,
                   whole->pt1.x + srect.pt2.x - op->sx,
                   whole->pt1.y + srect.pt2.y - op->sy);
          pix = (FAR const uint8_t *)src->fbmem +
                srect.pt1.y * src->stride + ((srect.pt1.x * bpp) >> 3);
          rect_set(&stage, g_comp.stagepos.x, g_comp.stagepos.y,
                   g_comp.stagepos.x + chunk.pt2.x - chunk.pt1.x,
                   g_comp.stagepos.y + chunk.pt2.y - chunk.pt1.y);
          g_nxaccel->copy(&stage, pix, &stage.pt1, src->stride);
          comp_gpuop_do(op, org, &chunk, &g_comp.stagepos);
        }
    }
}

/* One operation of a window's (window coordinates) at org on the device,
 * clipped to clip (device coordinates)
 */

static void comp_gpuop(FAR struct nxbe_window_s *wnd,
                       FAR const struct nx_gpuop_s *op,
                       FAR const struct nxgl_point_s *org,
                       FAR const struct nxgl_rect_s *clip)
{
  FAR struct nxbe_window_s *src;
  struct nxgl_rect_s whole;
  struct nxgl_rect_s dest;
  struct nxgl_point_s spos;
  uint8_t smode = op->mode & NX_BLEND_SRC_MASK;
  bool image = op->op == NX_GPUOP_COPY ||
               (op->op == NX_GPUOP_BLEND &&
                (smode != NX_BLEND_SRC_COLOUR ||
                 (op->mode & NX_BLEND_EXT) != 0));

  if (op->w <= 0 || op->h <= 0 ||
      (op->op == NX_GPUOP_BLEND && (op->mode & NX_BLEND_EXT) != 0 &&
       (g_nxaccel->flags & NX_ACCEL_BC1) == 0))
    {
      return;
    }

  rect_set(&whole, org->x + op->x, org->y + op->y,
           org->x + op->x + op->w - 1, org->y + op->y + op->h - 1);
  nxgl_rectintersect(&dest, &whole, clip);
  if (rect_empty(&dest))
    {
      return;
    }

  if (image)
    {
      src = op->src != NULL ? (FAR struct nxbe_window_s *)op->src : wnd;
      if (!src->surface)
        {
          /* In memory (or parked): a BC1 image is not drawn from there */

          if ((op->mode & NX_BLEND_EXT) == 0)
            {
              comp_gpuop_mem(src, op, org, &whole, &dest);
            }

          return;
        }

      /* A BC1 image's pixels are in the hardware's memory seen four times
       * as tall
       */

      spos.x = src->surfpos.x + op->sx + dest.pt1.x - whole.pt1.x;
      spos.y = ((op->mode & NX_BLEND_EXT) != 0 ? 4 * src->surfpos.y :
                src->surfpos.y) + op->sy + dest.pt1.y - whole.pt1.y;
    }

  comp_gpuop_do(op, org, &dest, &spos);
}

/* Draw one window where it meets display rectangle r */

static void comp_window(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *r,
                        FAR const struct nxgl_point_s *off)
{
  struct nxgl_rect_s c;
  struct nxgl_rect_s dest;
  struct nxgl_point_s org;
  struct nx_blend_s b;
  int i;

  if (wnd == &g_comp.be->bkgd)
    {
      nxgl_rectoffset(&dest, r, off->x, off->y);
      op_fill(&dest, wnd->be->bgcolor[0]);
      return;
    }

  if (wnd->fx.shadow > 0)
    {
      comp_shadow(wnd, r, off);
    }

  nxgl_rectintersect(&c, r, &wnd->bounds);
  if (rect_empty(&c))
    {
      return;
    }

  if (wnd->fx.backdrop > 0)
    {
      nxgl_rectoffset(&dest, &c, off->x, off->y);
      op_blend_init(&b, &dest);
      b.colour = wnd->fx.tint;
      b.alpha  = wnd->fx.backdrop;
      b.mode   = NX_BLEND_SRC_COLOUR | NX_BLEND_DITHER;
      if (wnd_round(wnd))
        {
          struct nxgl_rect_s inner;

          rect_set(&inner, wnd->bounds.pt1.x + NXCOMP_R,
                   wnd->bounds.pt1.y + NXCOMP_R,
                   wnd->bounds.pt2.x - NXCOMP_R,
                   wnd->bounds.pt2.y - NXCOMP_R);
          b.mode |= NX_BLEND_MOD_LUT;
          b.slot  = NX_LUT_CORNER;
          op_inner(&b, &inner, off);
        }

      g_nxaccel->blend(&b);
    }

  if (wnd_pixels(wnd))
    {
      comp_pixels(wnd, &c, off);
    }

  if (wnd->fx.border > 0 && wnd_round(wnd))
    {
      comp_border(wnd, &c, off);
    }

  if (NXBE_ISLAYER(wnd) && wnd->ndlist > 0)
    {
      org.x = wnd->bounds.pt1.x + off->x;
      org.y = wnd->bounds.pt1.y + off->y;
      nxgl_rectoffset(&dest, &c, off->x, off->y);
      for (i = 0; i < wnd->ndlist; i++)
        {
          comp_gpuop(wnd, &wnd->dlist[i], &org, &dest);
        }
    }
}

/* Composite display rectangle r, at off: from the top window that hides
 * everything below it there, up
 */

static void comp_rect(FAR const struct nxgl_rect_s *r,
                      FAR const struct nxgl_point_s *off)
{
  FAR struct nxbe_state_s *be = g_comp.be;
  FAR struct nxbe_window_s *start;
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s ext;

  for (start = be->topwnd; start != NULL; start = start->below)
    {
      if (wnd_covers(start, r))
        {
          break;
        }
    }

  if (start == NULL)
    {
      start = &be->bkgd;
    }

  comp_window(start, r, off);
  for (wnd = start->above; wnd != NULL; wnd = wnd->above)
    {
      wnd_extent(wnd, &ext);
      if (nxgl_rectoverlap(&ext, (FAR struct nxgl_rect_s *)r))
        {
          comp_window(wnd, r, off);
        }
    }
}

/* Rectangle lists **********************************************************/

static bool list_add(FAR struct nxgl_rect_s *list, FAR int *n, int max,
                     FAR const struct nxgl_rect_s *r)
{
  if (rect_empty(r))
    {
      return true;
    }

  if (*n >= max)
    {
      return false;
    }

  list[(*n)++] = *r;
  return true;
}

/* p minus c: up to four rectangles, above, below, left and right of c */

static int rect_minus(FAR const struct nxgl_rect_s *p,
                      FAR const struct nxgl_rect_s *c,
                      FAR struct nxgl_rect_s part[4])
{
  struct nxgl_rect_s x;
  int n = 0;

  nxgl_rectintersect(&x, p, c);
  if (rect_empty(&x))
    {
      part[0] = *p;
      return 1;
    }

  rect_set(&part[n], p->pt1.x, p->pt1.y, p->pt2.x, x.pt1.y - 1);
  n += !rect_empty(&part[n]);
  rect_set(&part[n], p->pt1.x, x.pt2.y + 1, p->pt2.x, p->pt2.y);
  n += !rect_empty(&part[n]);
  rect_set(&part[n], p->pt1.x, x.pt1.y, x.pt1.x - 1, x.pt2.y);
  n += !rect_empty(&part[n]);
  rect_set(&part[n], x.pt2.x + 1, x.pt1.y, p->pt2.x, x.pt2.y);
  n += !rect_empty(&part[n]);
  return n;
}

/* p minus all of cut[]: the pieces, or -1 if there are too many */

static int rect_cut(FAR const struct nxgl_rect_s *p,
                    FAR const struct nxgl_rect_s *cut, int ncut,
                    FAR struct nxgl_rect_s *piece)
{
  struct nxgl_rect_s part[4];
  int npiece = 1;
  int i;
  int j;
  int k;
  int m;

  piece[0] = *p;
  for (i = 0; i < ncut && npiece > 0; i++)
    {
      for (j = 0; j < npiece; j++)
        {
          if (!nxgl_rectoverlap(&piece[j],
                                (FAR struct nxgl_rect_s *)&cut[i]))
            {
              continue;
            }

          m = rect_minus(&piece[j], &cut[i], part);
          if (npiece - 1 + m > NXCOMP_MAXPIECES)
            {
              return -1;
            }

          /* The first part takes the piece's place, the rest go last (and
           * are cut by this rectangle no more: they miss it)
           */

          piece[j] = part[0];
          for (k = 1; k < m; k++)
            {
              piece[npiece++] = part[k];
            }

          if (m == 0)
            {
              piece[j--] = piece[--npiece];
            }
        }
    }

  return npiece;
}

/* Display buffers **********************************************************/

static inline int comp_back(void)
{
  return g_comp.dbuf ? g_comp.front ^ 1 : g_comp.front;
}

/* Copy display rectangle r from the front buffer to the back one */

static void comp_bufcopy(FAR const struct nxgl_rect_s *src,
                         FAR const struct nxgl_rect_s *dest)
{
  struct nxgl_point_s spos;
  struct nxgl_rect_s d;

  spos.x = src->pt1.x + g_comp.buf[g_comp.front].x;
  spos.y = src->pt1.y + g_comp.buf[g_comp.front].y;
  nxgl_rectoffset(&d, dest, g_comp.buf[comp_back()].x,
                  g_comp.buf[comp_back()].y);
  op_copy(&spos, &d);
}

/* The back buffer lacks the last frame's changes: copy them over from the
 * front, except where cut[] (what this frame redraws) covers them
 */

static void comp_age(FAR const struct nxgl_rect_s *cut, int ncut)
{
  struct nxgl_rect_s piece[NXCOMP_MAXPIECES];
  int n;
  int i;
  int j;

  for (i = 0; i < g_comp.nprev; i++)
    {
      n = rect_cut(&g_comp.prev[i], cut, ncut, piece);
      if (n < 0)
        {
          comp_bufcopy(&g_comp.prev[i], &g_comp.prev[i]);
          continue;
        }

      for (j = 0; j < n; j++)
        {
          comp_bufcopy(&piece[j], &piece[j]);
        }
    }

  g_comp.nprev = 0;
}

static void comp_flip(int buf)
{
  g_nxaccel->flip(&g_comp.buf[buf]);
  g_comp.front    = buf;
  g_comp.lastflip = clock_systime_ticks();
}

/* A second display buffer: a surface of the display's size, if there is
 * room (the surfaces packed together for it if they have not been lately,
 * or pack is set: a display mode moved them).  It holds nothing yet: all
 * of the display is the last frame's change to it.
 */

static void comp_getback(bool pack)
{
  FAR struct nxbe_state_s *be = g_comp.be;
  struct nxgl_point_s pos;

  g_comp.retry = false;
  if (g_comp.dbuf || g_nxaccel->flip == NULL)
    {
      return;
    }

  if (g_nxaccel->surface_alloc(be->vinfo.xres, be->vinfo.yres,
                               NX_SURFACE_BUFFER, &pos) < 0)
    {
      if (!pack && g_comp.packed &&
          clock_systime_ticks() - g_comp.lastpack < NXCOMP_PACKAGAIN)
        {
          g_comp.packlater = true;         /* nxbe_compose_idle() */
          return;
        }

      nxbe_surface_compact();
      g_comp.packed   = true;
      g_comp.lastpack = clock_systime_ticks();
      if (g_nxaccel->surface_alloc(be->vinfo.xres, be->vinfo.yres,
                                   NX_SURFACE_BUFFER, &pos) < 0)
        {
          return;
        }
    }

  g_comp.buf[1]    = pos;
  g_comp.bufsize.w = be->vinfo.xres;
  g_comp.bufsize.h = be->vinfo.yres;
  g_comp.dbuf      = true;
  g_comp.nprev     = 1;
  g_comp.prev[0]   = be->bkgd.bounds;
  ginfo("Double buffered: (0,0) and (%d,%d)\n", pos.x, pos.y);
}

/* Down to one buffer, the display at (0, 0), shown and up to date */

static void comp_dropback(void)
{
  if (!g_comp.dbuf)
    {
      return;
    }

  if (g_comp.front != 0)
    {
      comp_age(NULL, 0);                 /* buf[0] is the back buffer */
      comp_flip(0);
    }

  g_nxaccel->surface_free(&g_comp.buf[1], g_comp.bufsize.w,
                          g_comp.bufsize.h);
  g_comp.dbuf  = false;
  g_comp.nprev = 0;
  ginfo("Single buffered\n");
}

/* Moves ********************************************************************/

static FAR struct nxcomp_move_s *move_find(FAR struct nxbe_window_s *wnd)
{
  int i;

  for (i = 0; i < g_comp.nmoves; i++)
    {
      if (g_comp.moves[i].wnd == wnd)
        {
          return &g_comp.moves[i];
        }
    }

  return NULL;
}

/* The window will not be copied from where it was: that is out of date */

static void move_drop(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxcomp_move_s *m = move_find(wnd);
  struct nxgl_rect_s ext;

  if (m != NULL)
    {
      wnd_extent_at(wnd, &m->from, &ext);
      *m = g_comp.moves[--g_comp.nmoves];
      nxbe_compose_damage(&ext);
    }
}

/* Can the window's composited pixels be copied to where it is now? */

static bool move_copyable(FAR struct nxcomp_move_s *m, int ncopy)
{
  FAR struct nxbe_window_s *wnd = m->wnd;

  return !NXBE_ISHIDDEN(wnd) && wnd->surface && !NXBE_ISLAYER(wnd) &&
         wnd->fx.opacity == 255 && (wnd->fx.flags & NX_FX_KEY) == 0 &&
         wnd->fx.backdrop == 0 &&
         m->from.pt2.x - m->from.pt1.x == wnd->bounds.pt2.x -
                                          wnd->bounds.pt1.x &&
         m->from.pt2.y - m->from.pt1.y == wnd->bounds.pt2.y -
                                          wnd->bounds.pt1.y &&
         (g_comp.dbuf || ncopy == 0);  /* One buffer: one copy in place */
}

/* A moved window, copied: what to copy, and the damage around it (add[]).
 * False if add[] has no room.
 */

static bool move_plan(FAR struct nxcomp_move_s *m,
                      FAR struct nxcomp_copy_s *copy,
                      FAR struct nxgl_rect_s *add, FAR int *nadd, int max)
{
  FAR struct nxbe_window_s *wnd = m->wnd;
  FAR const struct nxgl_rect_s *from = &m->from;
  FAR const struct nxgl_rect_s *to = &wnd->bounds;
  FAR const struct nxgl_rect_s *disp = &g_comp.be->bkgd.bounds;
  FAR struct nxbe_window_s *above;
  FAR struct nxcomp_move_s *am;
  struct nxgl_rect_s part[4];
  struct nxgl_rect_s u;
  struct nxgl_rect_s e;
  struct nxgl_rect_s x;
  nxgl_coord_t dx = to->pt1.x - from->pt1.x;
  nxgl_coord_t dy = to->pt1.y - from->pt1.y;
  bool ok = true;
  int n;
  int i;

  /* What is copied: the old pixels that were on the display, to where
   * they land on it
   */

  nxgl_rectintersect(&copy->src, from, disp);
  nxgl_rectoffset(&copy->dest, &copy->src, dx, dy);
  nxgl_rectintersect(&copy->dest, &copy->dest, disp);
  nxgl_rectoffset(&copy->src, &copy->dest, -dx, -dy);

  /* What it uncovered and its shadow, old and new: all it touches, then
   * and now, but for where it is
   */

  wnd_extent_at(wnd, from, &u);
  wnd_extent_at(wnd, to, &e);
  nxgl_rectunion(&u, &u, &e);
  n = rect_minus(&u, to, part);
  for (i = 0; i < n; i++)
    {
      ok &= list_add(add, nadd, max, &part[i]);
    }

  /* Where it is, but what the copy did not bring (off the display) */

  if (rect_empty(&copy->dest))
    {
      ok &= list_add(add, nadd, max, to);
    }
  else
    {
      n = rect_minus(to, &copy->dest, part);
      for (i = 0; i < n && !rect_contains(&copy->dest, to); i++)
        {
          ok &= list_add(add, nadd, max, &part[i]);
        }
    }

  /* Its round corners show what is below */

  if (wnd_round(wnd))
    {
      rect_set(&e, to->pt1.x, to->pt1.y,
               to->pt1.x + NXCOMP_R - 1, to->pt1.y + NXCOMP_R - 1);
      ok &= list_add(add, nadd, max, &e);
      rect_set(&e, to->pt2.x - NXCOMP_R + 1, to->pt1.y,
               to->pt2.x, to->pt1.y + NXCOMP_R - 1);
      ok &= list_add(add, nadd, max, &e);
      rect_set(&e, to->pt1.x, to->pt2.y - NXCOMP_R + 1,
               to->pt1.x + NXCOMP_R - 1, to->pt2.y);
      ok &= list_add(add, nadd, max, &e);
      rect_set(&e, to->pt2.x - NXCOMP_R + 1, to->pt2.y - NXCOMP_R + 1,
               to->pt2.x, to->pt2.y);
      ok &= list_add(add, nadd, max, &e);
    }

  /* What was out of date where it was (its own drawing, what changed
   * over it) is out of date where the copy takes it
   */

  for (i = 0; i < g_comp.ndamage; i++)
    {
      nxgl_rectintersect(&x, &g_comp.damage[i], from);
      nxgl_rectoffset(&x, &x, dx, dy);
      ok &= list_add(add, nadd, max, &x);
    }

  /* Windows above it: they were over the pixels copied (where they were
   * then, if they moved too), and are over where it is now
   */

  for (above = wnd->above; above != NULL; above = above->above)
    {
      wnd_extent(above, &e);
      nxgl_rectintersect(&x, &e, to);
      ok &= list_add(add, nadd, max, &x);
      nxgl_rectintersect(&x, &e, from);
      nxgl_rectoffset(&x, &x, dx, dy);
      ok &= list_add(add, nadd, max, &x);

      am = move_find(above);
      if (am != NULL)
        {
          wnd_extent_at(above, &am->from, &e);
          nxgl_rectintersect(&x, &e, from);
          nxgl_rectoffset(&x, &x, dx, dy);
          ok &= list_add(add, nadd, max, &x);
        }
    }

  return ok;
}

/* The moves of this frame: copies where the windows can be copied, and
 * damage for the rest
 */

static int comp_moves(FAR struct nxcomp_copy_s *copy)
{
  struct nxgl_rect_s add[NXCOMP_MAXADD];
  struct nxgl_rect_s e;
  int ncopy = 0;
  int nadd = 0;
  int start;
  int i;

  for (i = 0; i < g_comp.nmoves; i++)
    {
      FAR struct nxcomp_move_s *m = &g_comp.moves[i];

      start = nadd;
      if (move_copyable(m, ncopy) &&
          move_plan(m, &copy[ncopy], add, &nadd, NXCOMP_MAXADD - 2))
        {
          if (!rect_empty(&copy[ncopy].dest))
            {
              ncopy++;
            }

          continue;
        }

      /* Composited again, where it was and where it is */

      nadd = start;
      wnd_extent_at(m->wnd, &m->from, &e);
      list_add(add, &nadd, NXCOMP_MAXADD, &e);
      if (!NXBE_ISHIDDEN(m->wnd))
        {
          wnd_extent(m->wnd, &e);
          list_add(add, &nadd, NXCOMP_MAXADD, &e);
        }
    }

  g_comp.nmoves = 0;
  for (i = 0; i < nadd; i++)
    {
      nxbe_compose_damage(&add[i]);
    }

  return ncopy;
}

/* Parking ******************************************************************/

/* Whether any of a window is seen: it is not hidden, some of it is on the
 * display, and not all of that is covered by the opaque windows above it
 * (their round corners do not cover)
 */

static bool comp_seen(FAR struct nxbe_window_s *wnd)
{
  struct nxgl_rect_s cover[NXCOMP_MAXCOVER];
  struct nxgl_rect_s piece[NXCOMP_MAXPIECES];
  FAR struct nxbe_window_s *a;
  FAR const struct nxgl_rect_s *b;
  struct nxgl_rect_s r;
  int n = 0;

  if (NXBE_ISHIDDEN(wnd))
    {
      return false;
    }

  nxgl_rectintersect(&r, &wnd->bounds, &g_comp.be->bkgd.bounds);
  if (rect_empty(&r))
    {
      return false;
    }

  for (a = wnd->above; a != NULL; a = a->above)
    {
      b = &a->bounds;
      if (!wnd_pixels(a) || a->fx.opacity != 255 ||
          (a->fx.flags & NX_FX_KEY) != 0 ||
          !nxgl_rectoverlap(&r, (FAR struct nxgl_rect_s *)b))
        {
          continue;
        }

      if (n + 2 > NXCOMP_MAXCOVER)
        {
          return true;                  /* Too many to tell: seen */
        }

      if (wnd_round(a))
        {
          rect_set(&cover[n++], b->pt1.x + NXCOMP_R, b->pt1.y,
                   b->pt2.x - NXCOMP_R, b->pt2.y);
          rect_set(&cover[n++], b->pt1.x, b->pt1.y + NXCOMP_R,
                   b->pt2.x, b->pt2.y - NXCOMP_R);
        }
      else
        {
          cover[n++] = *b;
        }
    }

  return rect_cut(&r, cover, n, piece) != 0;
}

/* The state of the windows, for the statistics */

static void comp_snapshot(void)
{
  FAR struct nxbe_window_s *wnd;
  struct nx_compose_stats_s st;
  irqstate_t flags;

  memset(&st, 0, sizeof(st));
  st.on   = g_comp.on;
  st.dbuf = g_comp.dbuf;
  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
    {
      uint32_t px = (uint32_t)(wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1) *
                    (uint32_t)(wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1);

      if (wnd->surface)
        {
          px = (uint32_t)wnd->surfsize.w * wnd->surfsize.h;
          st.nsurf++;
          st.surfpx += px;
          if (wnd->surfkind == NX_SURFACE_IMAGE)
            {
              st.nimage++;
              st.imagepx += px;
            }
        }
      else if (wnd->parked == NXBE_PARK_STREAM)
        {
          st.nparked++;
          st.parkedpx += (uint32_t)wnd->surfsize.w * wnd->surfsize.h;
          if (g_nxaccel->park_size != NULL)
            {
              st.parkedsize += g_nxaccel->park_size(wnd->parkhandle);
            }
        }
      else if (wnd->parked == NXBE_PARK_LOST)
        {
          st.nlost++;
          st.lostpx += px;
        }
      else if (wnd->fbmem != NULL)
        {
          st.nmem++;
          st.membytes += (uint32_t)wnd->stride *
                         (wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1);
        }
    }

  g_comp.nparked = st.nparked + st.nlost;
  flags  = enter_critical_section();
  g_snap = st;
  leave_critical_section(flags);
  g_comp.changed = false;
}

/* Before a frame: windows parked that are seen again get their pixels
 * back; windows in memory go into surfaces if there is room now.  False
 * if the frame should wait for windows being drawn again.
 */

static bool comp_unpark(void)
{
  FAR struct nxbe_window_s *wnd;
  clock_t now = clock_systime_ticks();

  if (g_comp.nparked > 0 || g_comp.changed)
    {
      for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
        {
          if (wnd->parked != NXBE_PARK_NONE && comp_seen(wnd))
            {
              nxbe_surface_unpark(wnd);
            }
        }

      if (g_comp.changed)
        {
          comp_snapshot();
        }
    }

  if (g_comp.retry && nxbe_surface_frommemory() > 0)
    {
      nxbe_srccache_trim();
      comp_snapshot();
    }

  if (g_comp.hold && (int32_t)(g_comp.holduntil - now) <= 0)
    {
      g_comp.hold = false;
    }

  return !g_comp.hold;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_compose_install
 *
 * Description:
 *   Composite if the hardware can: blending, shape tables and surfaces;
 *   double buffered if it can flip and there is room for a second display
 *   buffer.  Called once the board's operations are in place.
 *
 ****************************************************************************/

void nxbe_compose_install(FAR struct nxbe_state_s *be)
{
  memset(&g_comp, 0, sizeof(g_comp));
  g_comp.be = be;

  if (g_nxaccel == NULL || g_nxaccel->blend == NULL ||
      g_nxaccel->set_lut == NULL || g_nxaccel->surface_alloc == NULL)
    {
      ginfo("No blending: not compositing\n");
      return;
    }

  lut_corner(g_comp.lut);
  nxbe_compose_setlut(NX_LUT_CORNER, g_comp.lut);
  lut_shadow(g_comp.lut);
  nxbe_compose_setlut(NX_LUT_SHADOW, g_comp.lut);
  lut_ring(g_comp.lut);
  nxbe_compose_setlut(NX_LUT_RING, g_comp.lut);

  /* The display at (0, 0) is shown (and becomes the front buffer) */

  if (g_nxaccel->flip != NULL)
    {
      g_nxaccel->flip(&g_comp.buf[0]);
    }

  g_comp.on = true;
  comp_getback(false);
}

bool nxbe_compose_on(void)
{
  return g_comp.on;
}

/****************************************************************************
 * Name: nxbe_compose_reclaim, nxbe_compose_freed
 *
 * Description:
 *   A surface did not fit: give up the second display buffer, if there is
 *   one (true: try again).  Surfaces have been freed: try for a second
 *   display buffer again at the next frame.
 *
 ****************************************************************************/

bool nxbe_compose_reclaim(void)
{
  if (!g_comp.on || !g_comp.dbuf)
    {
      return false;
    }

  comp_dropback();
  return true;
}

void nxbe_compose_freed(void)
{
  g_comp.retry = true;
}

/****************************************************************************
 * Name: nxbe_compose_setmode
 *
 * Description:
 *   The display is changing size (a display mode): before the change
 *   (begin), give up the second buffer; after it (with be->vinfo and the
 *   background's bounds updated, the surfaces moved out of the display),
 *   take a new one and composite all of it.
 *
 ****************************************************************************/

void nxbe_compose_setmode(bool begin)
{
  if (!g_comp.on)
    {
      return;
    }

  if (begin)
    {
      if (g_comp.dbuf)
        {
          if (g_comp.front != 0)
            {
              comp_flip(0);
            }

          g_nxaccel->surface_free(&g_comp.buf[1], g_comp.bufsize.w,
                                  g_comp.bufsize.h);
          g_comp.dbuf = false;
        }

      comp_unstage();                    /* It may be in the way */
      g_comp.nprev   = 0;
      g_comp.nmoves  = 0;
      g_comp.ndamage = 0;
      return;
    }

  comp_getback(true);
  g_comp.nprev = 0;                      /* All of it is drawn anyway */
  nxbe_compose_damage(&g_comp.be->bkgd.bounds);
}

/****************************************************************************
 * Name: nxbe_compose_damage
 *
 * Description:
 *   Display rectangle rect is out of date.  Kept as at most
 *   NXCOMP_MAXDAMAGE rectangles: one inside another is dropped, and when
 *   there is no room the two whose union wastes least are merged.
 *
 ****************************************************************************/

void nxbe_compose_damage(FAR const struct nxgl_rect_s *rect)
{
  struct nxgl_rect_s r;
  struct nxgl_rect_s u;
  int32_t best = INT32_MAX;
  int bi = 0;
  int bj = 0;
  int i;
  int j;

  if (!g_comp.on)
    {
      return;
    }

  nxgl_rectintersect(&r, rect, &g_comp.be->bkgd.bounds);
  if (rect_empty(&r))
    {
      return;
    }

  for (i = 0; i < g_comp.ndamage; i++)
    {
      if (rect_contains(&g_comp.damage[i], &r))
        {
          return;
        }

      if (rect_contains(&r, &g_comp.damage[i]))
        {
          g_comp.damage[i--] = g_comp.damage[--g_comp.ndamage];
        }
    }

  if (g_comp.ndamage == 0 && g_comp.nmoves == 0)
    {
      g_comp.since = clock_systime_ticks();
    }

  if (g_comp.ndamage < NXCOMP_MAXDAMAGE)
    {
      g_comp.damage[g_comp.ndamage++] = r;
      return;
    }

  /* Full: merge the new one or two old ones, whichever wastes least */

  for (i = 0; i <= g_comp.ndamage; i++)
    {
      FAR struct nxgl_rect_s *a = i < g_comp.ndamage ?
                                  &g_comp.damage[i] : &r;

      for (j = i + 1; j <= g_comp.ndamage; j++)
        {
          FAR struct nxgl_rect_s *b = j < g_comp.ndamage ?
                                      &g_comp.damage[j] : &r;
          int32_t waste;

          nxgl_rectunion(&u, a, b);
          waste = rect_area(&u) - rect_area(a) - rect_area(b);
          if (waste < best)
            {
              best = waste;
              bi = i;
              bj = j;
            }
        }
    }

  if (bj == g_comp.ndamage)
    {
      nxgl_rectunion(&g_comp.damage[bi], &g_comp.damage[bi], &r);
    }
  else
    {
      nxgl_rectunion(&g_comp.damage[bi], &g_comp.damage[bi],
                     &g_comp.damage[bj]);
      g_comp.damage[bj] = r;
    }
}

/* A window's own rectangle (window coordinates) changed */

void nxbe_compose_damage_wrect(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *rel)
{
  struct nxgl_rect_s r;

  if (NXBE_ISHIDDEN(wnd))
    {
      return;
    }

  nxgl_rectoffset(&r, rel, wnd->bounds.pt1.x, wnd->bounds.pt1.y);
  nxgl_rectintersect(&r, &r, &wnd->bounds);
  nxbe_compose_damage(&r);
}

/* All that the window touches (bounds and shadow) is out of date, and
 * where it was at the last frame if it has moved since
 */

void nxbe_compose_damage_window(FAR struct nxbe_window_s *wnd)
{
  struct nxgl_rect_s ext;

  if (!g_comp.on || wnd == &g_comp.be->bkgd)
    {
      return;
    }

  move_drop(wnd);
  if (!NXBE_ISHIDDEN(wnd))
    {
      wnd_extent(wnd, &ext);
      nxbe_compose_damage(&ext);
    }
}

/****************************************************************************
 * Name: nxbe_compose_setposition
 *
 * Description:
 *   Move a window.  One the compositor can copy (it has a surface) is
 *   only noted, with where it was at the last frame; the frame works out
 *   what to copy and what is damaged.  Others damage where they were and
 *   where they are.
 *
 ****************************************************************************/

void nxbe_compose_setposition(FAR struct nxbe_window_s *wnd,
                              FAR const struct nxgl_point_s *pos)
{
  nxgl_coord_t dx = pos->x - wnd->bounds.pt1.x;
  nxgl_coord_t dy = pos->y - wnd->bounds.pt1.y;

  if (dx == 0 && dy == 0)
    {
      return;
    }

  g_comp.lastmove = clock_systime_ticks();
  if (!NXBE_ISHIDDEN(wnd) && wnd->surface && !NXBE_ISLAYER(wnd) &&
      (move_find(wnd) != NULL || g_comp.nmoves < NXCOMP_MAXMOVES))
    {
      if (move_find(wnd) == NULL)
        {
          if (g_comp.ndamage == 0 && g_comp.nmoves == 0)
            {
              g_comp.since = clock_systime_ticks();
            }

          g_comp.moves[g_comp.nmoves].wnd  = wnd;
          g_comp.moves[g_comp.nmoves].from = wnd->bounds;
          g_comp.nmoves++;
        }

      nxgl_rectoffset(&wnd->bounds, &wnd->bounds, dx, dy);
      return;
    }

  nxbe_compose_damage_window(wnd);
  nxgl_rectoffset(&wnd->bounds, &wnd->bounds, dx, dy);
  nxbe_compose_damage_window(wnd);
}

/****************************************************************************
 * Name: nxbe_compose_restack
 *
 * Description:
 *   Put a window (in the hierarchy) at the top of its layer, or at the
 *   bottom of it.  Returns whether the stacking has changed.
 *
 ****************************************************************************/

bool nxbe_compose_restack(FAR struct nxbe_window_s *wnd, bool top)
{
  FAR struct nxbe_state_s *be = wnd->be;
  FAR struct nxbe_window_s *oldbelow = wnd->below;
  FAR struct nxbe_window_s *below;

  if (wnd == &be->bkgd || NXBE_ISHIDDEN(wnd) || wnd->below == NULL ||
      NXBE_STATE_ISMODAL(be))
    {
      return false;
    }

  wnd_unlink(wnd);

  if (top)
    {
      /* Below the lowest window of a higher layer */

      for (below = be->topwnd;
           below != &be->bkgd && below->fx.layer > wnd->fx.layer;
           below = below->below)
        {
        }
    }
  else
    {
      /* Above the highest window of a lower layer */

      for (below = &be->bkgd;
           below->above != NULL && below->above->fx.layer < wnd->fx.layer;
           below = below->above)
        {
        }
    }

  wnd_insert_above(wnd, below);
  return below != oldbelow;
}

/****************************************************************************
 * Name: nxbe_compose_open, nxbe_compose_close
 *
 * Description:
 *   A window has been opened (and put in the hierarchy): its effects start
 *   as none, in the normal layer.  A window is being closed.
 *
 ****************************************************************************/

void nxbe_compose_open(FAR struct nxbe_window_s *wnd)
{
  memset(&wnd->fx, 0, sizeof(wnd->fx));
  wnd->fx.layer   = NX_LAYER_NORMAL;
  wnd->fx.opacity = 255;
  wnd->dlist      = NULL;
  wnd->ndlist     = 0;

  if (g_comp.on)
    {
      nxbe_compose_restack(wnd, true);
      nxbe_compose_damage_window(wnd);
    }
}

void nxbe_compose_close(FAR struct nxbe_window_s *wnd)
{
  nxbe_compose_damage_window(wnd);
  if (wnd->dlist != NULL)
    {
      kmm_free(wnd->dlist);
      wnd->dlist  = NULL;
      wnd->ndlist = 0;
    }
}

/****************************************************************************
 * Name: nxbe_compose_seteffects
 ****************************************************************************/

void nxbe_compose_seteffects(FAR struct nxbe_window_s *wnd,
                             FAR const struct nx_effects_s *fx)
{
  bool relayer = fx->layer != wnd->fx.layer;

  nxbe_compose_damage_window(wnd);
  wnd->fx = *fx;
  if (wnd->fx.layer > NX_LAYER_OVERLAY)
    {
      wnd->fx.layer = NX_LAYER_OVERLAY;
    }

  if (relayer)
    {
      nxbe_compose_restack(wnd, true);
    }

  nxbe_compose_damage_window(wnd);
}

/****************************************************************************
 * Name: nxbe_compose_setdlist
 *
 * Description:
 *   A layer window's operations (copied).
 *
 ****************************************************************************/

void nxbe_compose_setdlist(FAR struct nxbe_window_s *wnd,
                           FAR const struct nx_gpuop_s *ops, int nops)
{
  FAR struct nx_gpuop_s *copy = NULL;

  if (nops > 0)
    {
      copy = kmm_malloc(nops * sizeof(struct nx_gpuop_s));
      if (copy == NULL)
        {
          gerr("ERROR: no memory for %d operations\n", nops);
          return;
        }

      memcpy(copy, ops, nops * sizeof(struct nx_gpuop_s));
    }

  if (wnd->dlist != NULL)
    {
      kmm_free(wnd->dlist);
    }

  wnd->dlist  = copy;
  wnd->ndlist = nops;
  if (!NXBE_ISHIDDEN(wnd))
    {
      nxbe_compose_damage(&wnd->bounds);
    }
}

/****************************************************************************
 * Name: nxbe_compose_gpuops
 *
 * Description:
 *   Operations into a window's surface; what they cover is then out of
 *   date on the display.
 *
 ****************************************************************************/

void nxbe_compose_gpuops(FAR struct nxbe_window_s *wnd,
                         FAR const struct nx_gpuop_s *ops, int nops)
{
  struct nxgl_rect_s clip;
  struct nxgl_rect_s dirty;
  struct nxgl_rect_s r;
  int i;

  if (!g_comp.on)
    {
      return;
    }

  if (wnd->parked != NXBE_PARK_NONE)
    {
      /* Parked: drawn again in full when it is seen */

      nxbe_surface_touch(wnd);
      return;
    }

  if (!wnd->surface)
    {
      /* In memory (no room for a surface): in software */

      nxbe_compose_memops(wnd, ops, nops, &dirty);
      if (!rect_empty(&dirty))
        {
          nxbe_compose_damage_wrect(wnd, &dirty);
        }

      return;
    }

  rect_set(&clip, wnd->surfpos.x, wnd->surfpos.y,
           wnd->surfpos.x + wnd->bounds.pt2.x - wnd->bounds.pt1.x,
           wnd->surfpos.y + wnd->bounds.pt2.y - wnd->bounds.pt1.y);
  rect_set(&dirty, INT16_MAX, INT16_MAX, INT16_MIN, INT16_MIN);

  for (i = 0; i < nops; i++)
    {
      comp_gpuop(wnd, &ops[i], &wnd->surfpos, &clip);
      rect_set(&r, ops[i].x, ops[i].y, ops[i].x + ops[i].w - 1,
               ops[i].y + ops[i].h - 1);
      if (!rect_empty(&r))
        {
          nxgl_rectunion(&dirty, &dirty, &r);
        }
    }

  if (!rect_empty(&dirty))
    {
      if (wnd->srccache != NULL)
        {
          nxbe_srccache_drop(wnd, &dirty);
        }

      nxbe_compose_damage_wrect(wnd, &dirty);
    }
}

/****************************************************************************
 * Name: nxbe_compose_due, nxbe_compose_wait, nxbe_compose_run
 *
 * Description:
 *   Whether damage has waited long enough to be composited even though
 *   messages are still coming.  How long until a frame may be composited
 *   (-1: nothing to composite; 0: now): with two buffers, at most one a
 *   frame, so that the hardware, which holds everything after a flip
 *   until the frame starts, is not kept waiting by the next one.
 *   Composite a frame.
 *
 ****************************************************************************/

bool nxbe_compose_due(void)
{
  return g_comp.on && (g_comp.ndamage > 0 || g_comp.nmoves > 0) &&
         clock_systime_ticks() - g_comp.since >= NXCOMP_LATENCY &&
         (!g_comp.hold ||
          (int32_t)(g_comp.holduntil - clock_systime_ticks()) <= 0);
}

int nxbe_compose_wait(void)
{
  clock_t elapsed;

  if (!g_comp.on || (g_comp.ndamage == 0 && g_comp.nmoves == 0 &&
                     !(g_comp.retry && !g_comp.dbuf)))
    {
      return -1;
    }

  if (g_comp.hold)
    {
      elapsed = g_comp.holduntil - clock_systime_ticks();
      if ((int32_t)elapsed > 0)
        {
          return (int)elapsed;
        }
    }

  if (!g_comp.dbuf)
    {
      return 0;
    }

  elapsed = clock_systime_ticks() - g_comp.lastflip;
  return elapsed >= NXCOMP_FRAME ? 0 : (int)(NXCOMP_FRAME - elapsed);
}

void nxbe_compose_run(void)
{
  struct nxcomp_copy_s copy[NXCOMP_MAXMOVES];
  struct nxgl_rect_s redraw[NXCOMP_MAXPREV];
  struct nxgl_point_s off;
  int nredraw = 0;
  int ncopy;
  int i;

  if (!g_comp.on)
    {
      return;
    }

  /* Windows seen again get their pixels back first */

  if (!comp_unpark())
    {
      return;
    }

  if (g_comp.retry && !g_comp.dbuf)
    {
      comp_getback(false);
    }

  if (g_comp.ndamage == 0 && g_comp.nmoves == 0)
    {
      return;
    }

  /* The moves: what is copied, and what they damage */

  ncopy = comp_moves(copy);

  /* What this frame redraws */

  for (i = 0; i < g_comp.ndamage; i++)
    {
      redraw[nredraw++] = g_comp.damage[i];
    }

  for (i = 0; i < ncopy; i++)
    {
      redraw[nredraw++] = copy[i].dest;
    }

  /* The back buffer: the last frame's changes it lacks, then the moved
   * windows from where they were, then the damage
   */

  if (g_comp.dbuf)
    {
      comp_age(redraw, nredraw);
    }

  for (i = 0; i < ncopy; i++)
    {
      comp_bufcopy(&copy[i].src, &copy[i].dest);
    }

  off = g_comp.buf[comp_back()];
  for (i = 0; i < g_comp.ndamage; i++)
    {
      comp_rect(&g_comp.damage[i], &off);
    }

  /* Show it; the other buffer lacks what it changed */

  if (g_comp.dbuf)
    {
      comp_flip(comp_back());
      memcpy(g_comp.prev, redraw, nredraw * sizeof(redraw[0]));
      g_comp.nprev = nredraw;
    }

  g_comp.ndamage = 0;
  g_nxbe_counts.frames++;
}

/****************************************************************************
 * Name: nxbe_compose_seen, nxbe_compose_changed, nxbe_compose_redrawing
 ****************************************************************************/

bool nxbe_compose_seen(FAR struct nxbe_window_s *wnd)
{
  return g_comp.on && comp_seen(wnd);
}

void nxbe_compose_changed(void)
{
  g_comp.changed = true;
}

void nxbe_compose_redrawing(FAR struct nxbe_window_s *wnd)
{
  g_comp.hold      = true;
  g_comp.holduntil = clock_systime_ticks() + NXCOMP_REDRAWHOLD;
}

/****************************************************************************
 * Name: nxbe_compose_idle
 *
 * Description:
 *   The server is idle: windows that have not been seen for a while (half
 *   a second hidden, a second covered) and may be parked (NX_FX_PARK)
 *   are parked, unless windows are being moved (a drag).  Returns how many
 *   ticks until it should be called again, -1 if only after something
 *   changes.
 *
 ****************************************************************************/

int nxbe_compose_idle(void)
{
  FAR struct nxbe_window_s *wnd;
  clock_t now = clock_systime_ticks();
  clock_t elapsed = now - g_comp.lastidle;
  int32_t next = -1;
  int32_t wait;
  bool quiet;

  if (!g_comp.on)
    {
      return -1;
    }

  if (g_comp.changed)
    {
      nxbe_srccache_trim();
      comp_snapshot();
    }

  /* A second display buffer that did not fit: packing again when it may */

  if (g_comp.packlater && !g_comp.dbuf)
    {
      clock_t since = now - g_comp.lastpack;

      if (since >= NXCOMP_PACKAGAIN)
        {
          g_comp.packlater = false;
          g_comp.retry     = true;
        }
      else if (elapsed >= NXCOMP_IDLEPOLL)
        {
          next = (int32_t)(NXCOMP_PACKAGAIN - since);
        }
    }

  if (elapsed < NXCOMP_IDLEPOLL)
    {
      return (int)(NXCOMP_IDLEPOLL - elapsed);
    }

  g_comp.lastidle = now;
  quiet = now - g_comp.lastmove >= NXCOMP_QUIET;
  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
    {
      uint32_t delay;
      uint32_t age;

      if ((!wnd->surface && wnd->fbmem == NULL) ||
          wnd->parked != NXBE_PARK_NONE || NXBE_ISLAYER(wnd))
        {
          continue;
        }

      if (comp_seen(wnd))
        {
          wnd->unseen = 0;
          continue;
        }

      if (wnd->unseen == 0)
        {
          wnd->unseen = (uint32_t)now | 1;
        }

      if ((wnd->fx.flags & NX_FX_PARK) == 0)
        {
          continue;
        }

      delay = NXBE_ISHIDDEN(wnd) ? NXCOMP_PARKHIDDEN : NXCOMP_PARKCOVER;
      age   = (uint32_t)now - wnd->unseen;
      if (age >= delay && quiet)
        {
          nxbe_surface_park(wnd);
          continue;
        }

      wait = age < delay ? (int32_t)(delay - age) : 0;
      if (!quiet)
        {
          wait = MAX(wait, (int32_t)(NXCOMP_QUIET - (now -
                                                     g_comp.lastmove)));
        }

      next = next < 0 ? wait : MIN(next, wait);
    }

  if (g_comp.changed)
    {
      comp_snapshot();
    }

  return next < 0 ? -1 : MAX(next, (int32_t)NXCOMP_IDLEPOLL);
}

/****************************************************************************
 * Name: nxbe_compose_stats
 ****************************************************************************/

void nxbe_compose_stats(FAR struct nx_compose_stats_s *st)
{
  irqstate_t flags = enter_critical_section();

  *st = g_snap;
  leave_critical_section(flags);
  st->frames    = g_nxbe_counts.frames;
  st->parks     = g_nxbe_counts.parks;
  st->unparks   = g_nxbe_counts.unparks;
  st->redraws   = g_nxbe_counts.redraws;
  st->srcread   = g_nxbe_counts.srcread;
  st->srccached = g_nxbe_counts.srccached;
  st->tosurface = g_nxbe_counts.tosurface;
}

#endif /* CONFIG_NX_COMPOSITOR */
