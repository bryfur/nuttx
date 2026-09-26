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
 * NXCOMP_LATENCY), each damaged rectangle is composited: in bands of up to
 * CONFIG_NX_COMPOSITOR_BANDH lines, in an off-screen surface of that
 * height (the band), from the lowest window that matters to the top one:
 *
 *   - the background: its colour;
 *   - a window's shadow: black, blended with the shape table NX_LUT_SHADOW
 *     around the window's rounded rectangle (only where the window does
 *     not cover it);
 *   - its backdrop: a tint blended behind it (translucent panels);
 *   - its pixels: copied from its surface, the round corners (and the
 *     whole window, if it has an opacity or a colour key) blended with the
 *     shape table NX_LUT_CORNER;
 *   - or, for a layer window, its display list, replayed;
 *
 * and the band is then moved to the display in one piece, so no partial
 * state is ever seen.  The lowest window that matters is the top one that
 * covers the band with opaque pixels; if nothing above it touches the
 * band, its pixels go straight to the display with no band at all (typing
 * in a terminal on top costs one copy, as without the compositor).
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
#define NXCOMP_MAXDAMAGE  CONFIG_NX_COMPOSITOR_MAXDAMAGE
#define NXCOMP_BANDH      CONFIG_NX_COMPOSITOR_BANDH

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxcomp_s
{
  FAR struct nxbe_state_s *be;
  bool on;                                /* Compositing */
  nxgl_coord_t bandy;                     /* The band: lines bandy.. */
  nxgl_coord_t bandh;
  int ndamage;
  clock_t since;                          /* When the oldest damage came */
  struct nxgl_rect_s damage[NXCOMP_MAXDAMAGE];
  uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nxcomp_s g_comp;

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
  return !NXBE_ISLAYER(wnd) && (wnd->surface || wnd->fbmem != NULL);
}

/* The shadow's inner rectangle (the window's rounded rectangle, inset by
 * the radius, moved down) and how far it reaches
 */

static void wnd_shadow(FAR struct nxbe_window_s *wnd,
                       FAR struct nxgl_rect_s *inner,
                       FAR struct nxgl_rect_s *reach)
{
  rect_set(inner, wnd->bounds.pt1.x + NXCOMP_R,
           wnd->bounds.pt1.y + NXCOMP_R + wnd->fx.shadowdy,
           wnd->bounds.pt2.x - NXCOMP_R,
           wnd->bounds.pt2.y - NXCOMP_R + wnd->fx.shadowdy);
  rect_set(reach, inner->pt1.x - NXCOMP_LUTMAX + 1,
           inner->pt1.y - NXCOMP_LUTMAX + 1,
           inner->pt2.x + NXCOMP_LUTMAX - 1,
           inner->pt2.y + NXCOMP_LUTMAX - 1);
}

/* All of the display a window touches: its bounds and its shadow */

static void wnd_extent(FAR struct nxbe_window_s *wnd,
                       FAR struct nxgl_rect_s *ext)
{
  struct nxgl_rect_s inner;
  struct nxgl_rect_s reach;

  *ext = wnd->bounds;
  if (wnd->fx.shadow > 0)
    {
      wnd_shadow(wnd, &inner, &reach);
      nxgl_rectunion(ext, ext, &reach);
    }
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

/* The band: display rectangle r is composited at r moved down by dy */

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

/* LUT shape parameters: an inner rectangle moved by (0, dy) */

static void op_inner(FAR struct nx_blend_s *b,
                     FAR const struct nxgl_rect_s *inner, nxgl_coord_t dy)
{
  b->p[0] = inner->pt1.x;
  b->p[1] = inner->pt1.y + dy;
  b->p[2] = inner->pt2.x;
  b->p[3] = inner->pt2.y + dy;
}

/* Draw a window's shadow in display rectangle r, into the band */

static void comp_shadow(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *r, nxgl_coord_t dy)
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

      nxgl_rectoffset(&part[i], &part[i], 0, dy);
      op_blend_init(&b, &part[i]);
      b.colour = 0;
      b.alpha  = wnd->fx.shadow;
      b.mode   = NX_BLEND_SRC_COLOUR | NX_BLEND_MOD_LUT |
                 NX_BLEND_DITHER;
      b.slot   = NX_LUT_SHADOW;
      op_inner(&b, &inner, dy);
      g_nxaccel->blend(&b);
    }
}

/* A window's pixels in display rectangle c (inside its bounds) */

static void comp_pixels(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *c, nxgl_coord_t dy)
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

  if (!wnd->surface)
    {
      /* In memory: no blending from there, just the pixels */

      nxgl_rectoffset(&dest, c, 0, dy);
      op_window(wnd, c, &dest);
      return;
    }

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

      nxgl_rectoffset(&dest, &part[i], 0, dy);
      if (i >= ncorner && plain)
        {
          op_window(wnd, &part[i], &dest);
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
          op_inner(&b, &inner, dy);
        }

      g_nxaccel->blend(&b);
    }
}

/* A window's 1-pixel light edge where it meets display rectangle c: its
 * outer rows and columns and its corner squares, blended with the ring
 */

static void comp_border(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *c, nxgl_coord_t dy)
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

      nxgl_rectoffset(&part[i], &part[i], 0, dy);
      op_blend_init(&b, &part[i]);
      b.colour = 0xffff;
      b.alpha  = wnd->fx.border;
      b.mode   = NX_BLEND_SRC_COLOUR | NX_BLEND_MOD_LUT;
      b.slot   = NX_LUT_RING;
      op_inner(&b, &inner, dy);
      g_nxaccel->blend(&b);
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
  struct nx_blend_s b;
  uint8_t smode = op->mode & NX_BLEND_SRC_MASK;
  bool image = op->op == NX_GPUOP_COPY ||
               (op->op == NX_GPUOP_BLEND && smode != NX_BLEND_SRC_COLOUR);

  if (op->w <= 0 || op->h <= 0)
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
          return;                         /* Images come from surfaces */
        }

      spos.x = src->surfpos.x + op->sx + dest.pt1.x - whole.pt1.x;
      spos.y = src->surfpos.y + op->sy + dest.pt1.y - whole.pt1.y;
    }

  switch (op->op)
    {
      case NX_GPUOP_FILL:
        op_fill(&dest, op->colour);
        break;

      case NX_GPUOP_COPY:
        op_copy(&spos, &dest);
        break;

      case NX_GPUOP_BLEND:
        op_blend_init(&b, &dest);
        if (image)
          {
            b.src = spos;
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

/* Draw one window into the band, where it meets display rectangle r */

static void comp_window(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *r, nxgl_coord_t dy)
{
  struct nxgl_rect_s c;
  struct nxgl_rect_s dest;
  struct nxgl_point_s org;
  struct nx_blend_s b;
  int i;

  if (wnd == &g_comp.be->bkgd)
    {
      nxgl_rectoffset(&dest, r, 0, dy);
      op_fill(&dest, wnd->be->bgcolor[0]);
      return;
    }

  if (wnd->fx.shadow > 0)
    {
      comp_shadow(wnd, r, dy);
    }

  nxgl_rectintersect(&c, r, &wnd->bounds);
  if (rect_empty(&c))
    {
      return;
    }

  if (wnd->fx.backdrop > 0)
    {
      nxgl_rectoffset(&dest, &c, 0, dy);
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
          op_inner(&b, &inner, dy);
        }

      g_nxaccel->blend(&b);
    }

  if (wnd_pixels(wnd))
    {
      comp_pixels(wnd, &c, dy);
    }

  if (wnd->fx.border > 0 && wnd_round(wnd))
    {
      comp_border(wnd, &c, dy);
    }

  if (NXBE_ISLAYER(wnd) && wnd->ndlist > 0)
    {
      org.x = wnd->bounds.pt1.x;
      org.y = wnd->bounds.pt1.y + dy;
      nxgl_rectoffset(&dest, &c, 0, dy);
      for (i = 0; i < wnd->ndlist; i++)
        {
          comp_gpuop(wnd, &wnd->dlist[i], &org, &dest);
        }
    }
}

/* Composite display rectangle r (at most a band high) */

static void comp_band(FAR const struct nxgl_rect_s *r)
{
  FAR struct nxbe_state_s *be = g_comp.be;
  FAR struct nxbe_window_s *start;
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s ext;
  struct nxgl_rect_s band;
  struct nxgl_point_s src;
  nxgl_coord_t dy;
  bool alone = true;

  /* The top window that hides everything below it here */

  for (start = be->topwnd; start != NULL; start = start->below)
    {
      if (wnd_covers(start, r))
        {
          break;
        }

      wnd_extent(start, &ext);
      if (nxgl_rectoverlap(&ext, (FAR struct nxgl_rect_s *)r))
        {
          alone = false;
        }
    }

  if (start == NULL)
    {
      start = &be->bkgd;
    }

  /* Nothing above it here: straight to the display */

  if (alone && (start == &be->bkgd || start->surface))
    {
      comp_window(start, r, 0);
      return;
    }

  /* Else bottom to top in the band, then the band to the display */

  dy = g_comp.bandy - r->pt1.y;
  for (wnd = start; wnd != NULL; wnd = wnd->above)
    {
      wnd_extent(wnd, &ext);
      if (wnd == &be->bkgd ||
          nxgl_rectoverlap(&ext, (FAR struct nxgl_rect_s *)r))
        {
          comp_window(wnd, r, dy);
        }
    }

  nxgl_rectoffset(&band, r, 0, dy);
  src = band.pt1;
  op_copy(&src, r);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_compose_install
 *
 * Description:
 *   Composite if the hardware can: blending, shape tables, and room for
 *   the band.  Called once the board's operations are in place.
 *
 ****************************************************************************/

void nxbe_compose_install(FAR struct nxbe_state_s *be)
{
  struct nxgl_point_s pos;
  nxgl_coord_t w = be->vinfo.xres;

  g_comp.be = be;
  g_comp.on = false;

  if (g_nxaccel == NULL || g_nxaccel->blend == NULL ||
      g_nxaccel->set_lut == NULL || g_nxaccel->surface_alloc == NULL)
    {
      ginfo("No blending: not compositing\n");
      return;
    }

  g_comp.bandh = MIN(NXCOMP_BANDH, be->vinfo.yres);
  if (g_nxaccel->surface_alloc(w, g_comp.bandh, &pos) < 0 || pos.x != 0)
    {
      gerr("ERROR: no room for the composition band\n");
      return;
    }

  g_comp.bandy = pos.y;

  lut_corner(g_comp.lut);
  g_nxaccel->set_lut(NX_LUT_CORNER, g_comp.lut);
  lut_shadow(g_comp.lut);
  g_nxaccel->set_lut(NX_LUT_SHADOW, g_comp.lut);
  lut_ring(g_comp.lut);
  g_nxaccel->set_lut(NX_LUT_RING, g_comp.lut);

  g_comp.on = true;
  ginfo("Compositing, band at line %d\n", g_comp.bandy);
}

bool nxbe_compose_on(void)
{
  return g_comp.on;
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

  if (g_comp.ndamage == 0)
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

/* All that the window touches (bounds and shadow) is out of date */

void nxbe_compose_damage_window(FAR struct nxbe_window_s *wnd)
{
  struct nxgl_rect_s ext;

  if (g_comp.on && wnd != &g_comp.be->bkgd && !NXBE_ISHIDDEN(wnd))
    {
      wnd_extent(wnd, &ext);
      nxbe_compose_damage(&ext);
    }
}

/****************************************************************************
 * Name: nxbe_compose_restack
 *
 * Description:
 *   Put a window (in the hierarchy) at the top of its layer, or at the
 *   bottom of it.
 *
 ****************************************************************************/

void nxbe_compose_restack(FAR struct nxbe_window_s *wnd, bool top)
{
  FAR struct nxbe_state_s *be = wnd->be;
  FAR struct nxbe_window_s *below;

  if (wnd == &be->bkgd || NXBE_ISHIDDEN(wnd) || wnd->below == NULL ||
      NXBE_STATE_ISMODAL(be))
    {
      return;
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

  if (!g_comp.on || !wnd->surface)
    {
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
      nxbe_compose_damage_wrect(wnd, &dirty);
    }
}

/****************************************************************************
 * Name: nxbe_compose_due, nxbe_compose_run
 *
 * Description:
 *   Whether damage has waited long enough to be composited even though
 *   messages are still coming; composite all the damage.
 *
 ****************************************************************************/

bool nxbe_compose_due(void)
{
  return g_comp.on && g_comp.ndamage > 0 &&
         clock_systime_ticks() - g_comp.since >= NXCOMP_LATENCY;
}

void nxbe_compose_run(void)
{
  struct nxgl_rect_s band;
  int i;

  if (!g_comp.on)
    {
      return;
    }

  for (i = 0; i < g_comp.ndamage; i++)
    {
      FAR const struct nxgl_rect_s *d = &g_comp.damage[i];
      nxgl_coord_t y;

      for (y = d->pt1.y; y <= d->pt2.y; y += g_comp.bandh)
        {
          rect_set(&band, d->pt1.x, y, d->pt2.x,
                   MIN(d->pt2.y, y + g_comp.bandh - 1));
          comp_band(&band);
        }
    }

  g_comp.ndamage = 0;
}

#endif /* CONFIG_NX_COMPOSITOR */
