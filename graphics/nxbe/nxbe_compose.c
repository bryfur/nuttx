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
 * Changes to the display only record damage.  When the server is idle (or
 * damage has waited NXCOMP_LATENCY), at most once per NXCOMP_FRAME, a frame
 * is composited into the back buffer and flipped to:
 *
 *   - the back buffer (the frame before last) is first brought up to date
 *     with what the last frame changed, copied from the front buffer;
 *   - an opaque window that moved is copied from its old place in the front
 *     buffer; only its corners, what it uncovered and its shadow are damage;
 *   - each damaged rectangle is drawn from the top window that covers it
 *     opaquely upwards: background colour, shadows (NX_LUT_SHADOW),
 *     backdrops, window pixels with round corners (NX_LUT_CORNER), layer
 *     windows' display lists;
 *   - the flip takes effect at the next frame, so no partial state shows.
 *
 * The buffers are the display at (0, 0) and a surface of the mode's size.
 * When a window's surface does not fit, the second buffer is given up
 * (nxbe_compose_reclaim()) and the display is composited in place.
 * All drawing is done by the hardware (nx_accel_s).
 *
 * Composited in place (one buffer), a frame is drawn behind the beam when
 * the hardware can wait for it (nx_accel_s waitline): the damage, top to
 * bottom in bands of NXCOMP_RACEBAND lines, each band once the scanout
 * has read its last line (and not gone half a frame past its first), so
 * no band is drawn while it is being shown; windows that moved are drawn
 * again from their surfaces (a COPY of the display onto itself cannot be
 * cut into bands).  A large change spans frames band by band.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/param.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nx/nxaccel.h>
#include <nuttx/nx/nxcompose.h>

#include "nxbe.h"
#include "nxmu.h"

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
#define NXCOMP_QUADSPLIT  3       /* Windows in perspective a damaged
                                   * rectangle is split by, at most */
#define NXCOMP_IDLEPOLL   MSEC2TICK(50)    /* Parking: looked at so often */
#define NXCOMP_QUIET      MSEC2TICK(400)   /* ... and not while windows move */
#define NXCOMP_PARKHIDDEN MSEC2TICK(500)   /* ... hidden so long */
#define NXCOMP_PARKCOVER  MSEC2TICK(1000)  /* ... covered so long */
#define NXCOMP_REDRAWHOLD MSEC2TICK(50)    /* A window being drawn again:
                                            * the frame waits for it */
#define NXCOMP_MAXHELD    8                /* ... windows told apart */
#define NXCOMP_REDRAWGRACE MSEC2TICK(16)   /* ... after they start */
#define NXCOMP_PACKAGAIN  MSEC2TICK(5000)  /* Surfaces packed for a second
                                            * buffer at most this often */
#define NXCOMP_BUFAGAIN   MSEC2TICK(2000)  /* ... taken back so long after
                                            * it was given up */
#define NXCOMP_SWAPFREE   64               /* Swap: cells kept free */
#define NXCOMP_MAXPOLY    16               /* ... windows in perspective
                                            * over one, told apart */
#define NXCOMP_MIPMAG     1.5f             /* A window's smaller picture's
                                            * pixel: at most this large */
#define NXCOMP_SWAPSTEP   64               /* ... read back when idle, at
                                            * most, a time */
#define NXCOMP_RACEBAND   64               /* Drawn behind the beam: a
                                            * band's lines */

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
  clock_t holdsince;
  uint8_t nheld;                          /* ... or until these have drawn */
  FAR struct nxbe_window_s *held[NXCOMP_MAXHELD];
  bool hold;                              /* ... for windows being drawn */
  bool changed;                           /* The statistics are old */
  bool packed;                            /* Packed for a second buffer */
  bool packlater;                         /* ... to be tried again */
  bool needs;                             /* Windows' needs are old */
  clock_t lastdrop;                       /* The back buffer given up */
  FAR struct nxbe_window_s *target;       /* nx_gpuops() into it */
  bool game;                              /* A game has the display */
  uint32_t gameseq;                       /* ... its last ask for room met */
  clock_t lastpack;                       /* ... then */
  uint16_t nparked;                       /* Windows parked */
  int ndamage;
  int nprev;
  int nmoves;
  int race;                               /* Behind the beam: a band's
                                           * lines (0: not) */
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

/* Shown as a quadrilateral (NX_FX_QUAD), which needs its surface */

static inline bool wnd_quad(FAR struct nxbe_window_s *wnd)
{
  return (wnd->fx.flags & NX_FX_QUAD) != 0 && wnd->surface &&
         !NXBE_ISLAYER(wnd) && !NXBE_ISIMAGE(wnd) &&
         g_nxaccel->quad != NULL;
}

/* ... its corners (12.4), with the window at b */

static void wnd_corners_at(FAR struct nxbe_window_s *wnd,
                           FAR const struct nxgl_rect_s *b,
                           FAR int32_t *x, FAR int32_t *y)
{
  FAR const int16_t *q = wnd->fx.quad;

  x[0] = (b->pt1.x + q[0]) * 16;
  y[0] = (b->pt1.y + q[1]) * 16;
  x[1] = (b->pt2.x + 1 + q[2]) * 16;
  y[1] = (b->pt1.y + q[3]) * 16;
  x[2] = (b->pt2.x + 1 + q[4]) * 16;
  y[2] = (b->pt2.y + 1 + q[5]) * 16;
  x[3] = (b->pt1.x + q[6]) * 16;
  y[3] = (b->pt2.y + 1 + q[7]) * 16;
}

/* ... the pixels their bounding box touches */

static void wnd_quadbox_at(FAR struct nxbe_window_s *wnd,
                           FAR const struct nxgl_rect_s *b,
                           FAR struct nxgl_rect_s *box)
{
  int32_t x[4];
  int32_t y[4];
  int32_t x0;
  int32_t y0;
  int32_t x1;
  int32_t y1;
  int k;

  wnd_corners_at(wnd, b, x, y);
  x0 = x1 = x[0];
  y0 = y1 = y[0];
  for (k = 1; k < 4; k++)
    {
      x0 = x[k] < x0 ? x[k] : x0;
      x1 = x[k] > x1 ? x[k] : x1;
      y0 = y[k] < y0 ? y[k] : y0;
      y1 = y[k] > y1 ? y[k] : y1;
    }

  rect_set(box, x0 >> 4, y0 >> 4, (x1 + 15) >> 4, (y1 + 15) >> 4);
}

/* ... the rectangle that is their average (where its shadow falls), and
 * one inside them (that the quadrilateral covers, near enough: its
 * shadow need not be drawn there), with the window at b
 */

static void wnd_quadrects_at(FAR struct nxbe_window_s *wnd,
                             FAR const struct nxgl_rect_s *b,
                             FAR struct nxgl_rect_s *avg,
                             FAR struct nxgl_rect_s *in)
{
  int32_t x[4];
  int32_t y[4];

  wnd_corners_at(wnd, b, x, y);
  rect_set(avg, (x[0] + x[3]) >> 5, (y[0] + y[1]) >> 5,
           ((x[1] + x[2]) >> 5) - 1, ((y[2] + y[3]) >> 5) - 1);
  if (in != NULL)
    {
      rect_set(in, (MAX(x[0], x[3]) + 15) >> 4, (MAX(y[0], y[1]) + 15) >> 4,
               (MIN(x[1], x[2]) >> 4) - 1, (MIN(y[2], y[3]) >> 4) - 1);
    }
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

  if (wnd_quad(wnd))
    {
      struct nxgl_rect_s avg;

      /* The quadrilateral's bounding box, and its shadow's reach */

      wnd_quadbox_at(wnd, b, ext);
      if (wnd->fx.shadow > 0)
        {
          wnd_quadrects_at(wnd, b, &avg, NULL);
          wnd_shadow_at(wnd, &avg, &inner, &reach);
          nxgl_rectunion(ext, ext, &reach);
        }

      return;
    }

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

/* Shown in perspective: the rectangle its quadrilateral surely hides
 * (inside it, clear of the round corners), false if none
 */

static bool wnd_quadcover(FAR struct nxbe_window_s *wnd,
                          FAR struct nxgl_rect_s *c)
{
  struct nxgl_rect_s avg;
  struct nxgl_rect_s in;

  if (!wnd_quad(wnd) || NXBE_ISHIDDEN(wnd))
    {
      return false;
    }

  wnd_quadrects_at(wnd, &wnd->bounds, &avg, &in);
  rect_set(c, in.pt1.x + NXCOMP_R, in.pt1.y + NXCOMP_R,
           in.pt2.x - NXCOMP_R, in.pt2.y - NXCOMP_R);
  return !rect_empty(c);
}

/* Does the window hide everything below it in r? */

static bool wnd_covers(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *r)
{
  FAR const struct nxgl_rect_s *b = &wnd->bounds;
  struct nxgl_rect_s c;

  if (wnd == &g_comp.be->bkgd)
    {
      return true;
    }

  if (wnd_quad(wnd))
    {
      return wnd_quadcover(wnd, &c) && rect_contains(&c, r);
    }

  if (!wnd_pixels(wnd) || wnd->fx.opacity != 255 || wnd_quad(wnd) ||
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
  struct nxgl_rect_s rel;

  if (wnd->surface)
    {
      nxgl_rectoffset(&rel, r, -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);
      nxbe_surface_copyto(wnd, &rel, &dest->pt1);
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

/* Draw a window's shadow in display rectangle r: the shadow of rectangle
 * at, except where w (the window) covers it
 */

static void comp_shadow_at(FAR struct nxbe_window_s *wnd,
                           FAR const struct nxgl_rect_s *at,
                           FAR const struct nxgl_rect_s *w,
                           FAR const struct nxgl_rect_s *r,
                           FAR const struct nxgl_point_s *off)
{
  struct nxgl_rect_s inner;
  struct nxgl_rect_s reach;
  struct nxgl_rect_s e;
  struct nxgl_rect_s part[8];
  struct nx_blend_s b;
  int n = 0;
  int i;

  wnd_shadow_at(wnd, at, &inner, &reach);
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
      wnd->fx.backdrop == 0 && (wnd->fx.flags & NX_FX_KEY) == 0 &&
      w->pt2.x - w->pt1.x >= 2 * NXCOMP_R &&
      w->pt2.y - w->pt1.y >= 2 * NXCOMP_R)
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

static inline void comp_shadow(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *r,
                               FAR const struct nxgl_point_s *off)
{
  comp_shadow_at(wnd, &wnd->bounds, &wnd->bounds, r, off);
}

/* A staging surface for blending windows held in memory, as wide as the
 * widest display mode.  Made when first needed.
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

      g_comp.stage = nxbe_surface_take(w, NXCOMP_STAGEH, NX_SURFACE_STAGE,
                                       &g_comp.stagepos);
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

/* Blend display rectangle r of a window in memory, one staging band at a
 * time (without a staging surface, just copy the pixels)
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
  FAR const struct nxbe_piece_s *p;
  struct nxgl_rect_s inner;
  struct nxgl_rect_s part[7];
  struct nxgl_rect_s dest;
  struct nxgl_rect_s rel;
  struct nxgl_rect_s x;
  struct nxgl_rect_s abs;
  struct nx_blend_s b;
  int ncorner = 0;
  int n = 0;
  int np;
  int i;
  int k;

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

      /* From each piece of its surface */

      nxgl_rectoffset(&rel, &part[i], -w->pt1.x, -w->pt1.y);
      np = nxbe_surface_pieces(wnd, &p);
      for (k = 0; k < np; k++)
        {
          if (!nxbe_piece_part(&p[k], &rel, &x, &abs))
            {
              continue;
            }

          nxgl_rectoffset(&dest, &x, w->pt1.x + off->x, w->pt1.y + off->y);
          op_blend_init(&b, &dest);
          b.src   = abs.pt1;
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
}

/* A window's 1-pixel light edge (dark with NX_FX_DARKEDGE), blended with
 * the ring, where it meets display rectangle c
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

/* One display-list operation into dest (device coordinates); the image
 * source, if any, is at spos; shape parameters are relative to org
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

/* An image source held in memory, one staging band at a time */

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

/* One display-list operation (window coordinates) at org, clipped to clip
 * (device coordinates)
 */

static void comp_gpuop(FAR struct nxbe_window_s *wnd,
                       FAR const struct nx_gpuop_s *op,
                       FAR const struct nxgl_point_s *org,
                       FAR const struct nxgl_rect_s *clip)
{
  FAR struct nxbe_window_s *src;
  FAR const struct nxbe_piece_s *p;
  struct nxgl_rect_s whole;
  struct nxgl_rect_s dest;
  struct nxgl_rect_s srel;
  struct nxgl_rect_s x;
  struct nxgl_rect_s abs;
  struct nxgl_rect_s d;
  struct nxgl_point_s spos;
  uint8_t smode = op->mode & NX_BLEND_SRC_MASK;
  bool image = op->op == NX_GPUOP_COPY ||
               (op->op == NX_GPUOP_BLEND &&
                (smode != NX_BLEND_SRC_COLOUR ||
                 (op->mode & NX_BLEND_EXT) != 0));
  int np;
  int k;

  if (op->w <= 0 || op->h <= 0 ||
      (op->op == NX_GPUOP_BLEND && (op->mode & NX_BLEND_EXT) != 0 &&
       (g_nxaccel->flags & NX_ACCEL_BC1) == 0))
    {
      return;
    }

  spos.x = 0;                             /* (a colour: no source) */
  spos.y = 0;
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
          /* In memory or parked: BC1 images can't be drawn from there */

          if ((op->mode & NX_BLEND_EXT) == 0)
            {
              comp_gpuop_mem(src, op, org, &whole, &dest);
            }

          return;
        }

      /* BC1 pixels are addressed at four times the height */

      if ((op->mode & NX_BLEND_EXT) != 0)
        {
          /* (An image window: one surface of its own, at surfpos) */

          spos.x = src->surfpos.x + op->sx + dest.pt1.x - whole.pt1.x;
          spos.y = 4 * src->surfpos.y + op->sy + dest.pt1.y - whole.pt1.y;
          comp_gpuop_do(op, org, &dest, &spos);
          return;
        }

      /* The source's pieces: the part of dest each has the source of */

      rect_set(&srel, op->sx + dest.pt1.x - whole.pt1.x,
               op->sy + dest.pt1.y - whole.pt1.y,
               op->sx + dest.pt2.x - whole.pt1.x,
               op->sy + dest.pt2.y - whole.pt1.y);
      np = nxbe_surface_pieces(src, &p);
      for (k = 0; k < np; k++)
        {
          if (nxbe_piece_part(&p[k], &srel, &x, &abs))
            {
              nxgl_rectoffset(&d, &x, dest.pt1.x - srel.pt1.x,
                              dest.pt1.y - srel.pt1.y);
              comp_gpuop_do(op, org, &d, &abs.pt1);
            }
        }

      /* Some of the source not resident (not seen): into a window, what
       * it made there is not what it should be
       */

      if (g_comp.target != NULL && !nxbe_surface_resident(src, &srel))
        {
          nxgl_rectoffset(&d, &dest, -org->x, -org->y);
          nxbe_surface_lose(g_comp.target, &d);
        }

      return;
    }

  comp_gpuop_do(op, org, &dest, &spos);
}

/* A window in perspective: each piece of its surface drawn as the
 * quadrilateral that the window's projective map makes of its rectangle,
 * so that they meet without a seam (corners that are the window's are
 * its own, exactly); the round corners are the window's.  The arithmetic
 * is relative to the pixel at the corners' least x and y, so that the
 * window looks the same wherever it is drawn (either display buffer).
 */

static int32_t quad_round(float v)
{
  return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static void comp_quadimg(FAR const struct nxbe_piece_s *p, int n,
                         nxgl_coord_t w, nxgl_coord_t h, int radius,
                         FAR const int32_t *x, FAR const int32_t *y,
                         FAR const struct nxgl_rect_s *dest)
{
  struct nx_quadpart_s part;
  int32_t bx16 = MIN(MIN(x[0], x[1]), MIN(x[2], x[3])) & ~15;
  int32_t by16 = MIN(MIN(y[0], y[1]), MIN(y[2], y[3])) & ~15;
  float x0 = (x[0] - bx16) / 16.0f;
  float y0 = (y[0] - by16) / 16.0f;
  float x1 = (x[1] - bx16) / 16.0f;
  float y1 = (y[1] - by16) / 16.0f;
  float x2 = (x[2] - bx16) / 16.0f;
  float y2 = (y[2] - by16) / 16.0f;
  float x3 = (x[3] - bx16) / 16.0f;
  float y3 = (y[3] - by16) / 16.0f;
  float dx1 = x1 - x2;
  float dx2 = x3 - x2;
  float dx3 = x0 - x1 + x2 - x3;
  float dy1 = y1 - y2;
  float dy2 = y3 - y2;
  float dy3 = y0 - y1 + y2 - y3;
  float den = dx1 * dy2 - dx2 * dy1;
  float a;
  float b;
  float c;
  float d;
  float e;
  float f;
  float g;
  float hh;
  int k;
  int q;

  /* Every piece drawn relative to the pixel at the corners' least x and
   * y: the same pixels wherever the window is drawn
   */

  part.fw = w;
  part.fh = h;
  part.bx = bx16 >> 4;
  part.by = by16 >> 4;
  if (n == 1 && p[0].r.pt1.x == 0 && p[0].r.pt1.y == 0 &&
      p[0].r.pt2.x == w - 1 && p[0].r.pt2.y == h - 1)
    {
      part.ox = 0;
      part.oy = 0;
      g_nxaccel->quad(&p[0].pos, w, h, x, y, radius, &part, dest);
      return;
    }

  if (den > -1e-6f && den < 1e-6f)
    {
      return;                             /* Three corners in a line */
    }

  /* The map from the window's (u, v) in [0, 1] to the display */

  g  = (dx3 * dy2 - dx2 * dy3) / den;
  hh = (dx1 * dy3 - dx3 * dy1) / den;
  a  = x1 - x0 + g * x1;
  b  = x3 - x0 + hh * x3;
  c  = x0;
  d  = y1 - y0 + g * y1;
  e  = y3 - y0 + hh * y3;
  f  = y0;

  for (k = 0; k < n; k++)
    {
      int32_t qx[4];
      int32_t qy[4];
      nxgl_coord_t pu[4];
      nxgl_coord_t pv[4];

      pu[0] = pu[3] = p[k].r.pt1.x;
      pu[1] = pu[2] = p[k].r.pt2.x + 1;
      pv[0] = pv[1] = p[k].r.pt1.y;
      pv[2] = pv[3] = p[k].r.pt2.y + 1;
      for (q = 0; q < 4; q++)
        {
          float u = (float)pu[q] / w;
          float v = (float)pv[q] / h;
          float wn = g * u + hh * v + 1.0f;

          if ((pu[q] == 0 || pu[q] == w) && (pv[q] == 0 || pv[q] == h))
            {
              int corner = pv[q] == 0 ? (pu[q] == 0 ? 0 : 1) :
                                        (pu[q] == 0 ? 3 : 2);

              qx[q] = x[corner];
              qy[q] = y[corner];
            }
          else
            {
              qx[q] = bx16 + quad_round((a * u + b * v + c) / wn * 16.0f);
              qy[q] = by16 + quad_round((d * u + e * v + f) / wn * 16.0f);
            }
        }

      /* (Its round corners only for a piece that reaches into one: the
       * same rows, without looking for them)
       */

      part.ox = p[k].r.pt1.x;
      part.oy = p[k].r.pt1.y;
      g_nxaccel->quad(&p[k].pos, pu[1] - pu[0], pv[2] - pv[0], qx, qy,
                      (pu[0] < radius || pu[1] > w - radius) &&
                      (pv[0] < radius || pv[2] > h - radius) ? radius : 0,
                      &part, dest);
    }
}

static void comp_quad(FAR struct nxbe_window_s *wnd, FAR const int32_t *x,
                      FAR const int32_t *y,
                      FAR const struct nxgl_rect_s *dest)
{
  FAR const struct nxbe_piece_s *p;
  int n = nxbe_surface_pieces(wnd, &p);

  comp_quadimg(p, n, wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1,
               wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1,
               wnd_round(wnd) ? NXCOMP_R : 0, x, y, dest);
}

/* Mipmaps *******************************************************************/

/* A window drawn small in perspective (the switcher's cards) is drawn from
 * a smaller picture of it: 1 << lx of its pixels a pixel across and
 * 1 << ly down (each the one at their middle, as the hardware samples it
 * anyway when it draws the window that small), in a surface of its own,
 * made from its cells (resident: drawn by the hardware, scaled; in the
 * swap: read there) and made again where it is drawn into.  Its cells are
 * then not needed: the picture takes an eighth to a half of the room.  A
 * picture's pixel is never shown much larger than a pixel of the display
 * (NXCOMP_MIPMAG).  Only when the windows in perspective would not have
 * room as they are (ten in the switcher).
 */

#define MIP_LX(l)         ((l) & 15)
#define MIP_LY(l)         ((l) >> 4)
#define MIP_MIN           96               /* (smaller windows: none) */

struct nxbe_mip_s
{
  uint8_t level;                           /* lx | ly << 4 */
  nxgl_coord_t w;                          /* Its size */
  nxgl_coord_t h;
  nxgl_coord_t ww;                         /* The window's, when made */
  nxgl_coord_t wh;
  FAR struct nxbe_cells_s *cells;          /* Its cells (all resident) */
  bool dirty;                              /* drect of the window to be */
  struct nxgl_rect_s drect;                /* made again */
  bool fresh;                              /* Not made yet */
};

/* The projective map of a quadrilateral (12.4 corners) from the image's
 * (u, v) in [0, 1] to the display, relative to (bx16, by16)
 */

struct quad_fwd_s
{
  int32_t bx16;
  int32_t by16;
  float a;
  float b;
  float c;
  float d;
  float e;
  float f;
  float g;
  float h;
};

static bool quad_fwd(FAR const int32_t *x, FAR const int32_t *y,
                     FAR struct quad_fwd_s *m)
{
  float x0;
  float y0;
  float x1;
  float y1;
  float x2;
  float y2;
  float x3;
  float y3;
  float dx1;
  float dx2;
  float dx3;
  float dy1;
  float dy2;
  float dy3;
  float den;

  m->bx16 = MIN(MIN(x[0], x[1]), MIN(x[2], x[3])) & ~15;
  m->by16 = MIN(MIN(y[0], y[1]), MIN(y[2], y[3])) & ~15;
  x0  = (x[0] - m->bx16) / 16.0f;
  y0  = (y[0] - m->by16) / 16.0f;
  x1  = (x[1] - m->bx16) / 16.0f;
  y1  = (y[1] - m->by16) / 16.0f;
  x2  = (x[2] - m->bx16) / 16.0f;
  y2  = (y[2] - m->by16) / 16.0f;
  x3  = (x[3] - m->bx16) / 16.0f;
  y3  = (y[3] - m->by16) / 16.0f;
  dx1 = x1 - x2;
  dx2 = x3 - x2;
  dx3 = x0 - x1 + x2 - x3;
  dy1 = y1 - y2;
  dy2 = y3 - y2;
  dy3 = y0 - y1 + y2 - y3;
  den = dx1 * dy2 - dx2 * dy1;
  if (den > -1e-6f && den < 1e-6f)
    {
      return false;
    }

  m->g = (dx3 * dy2 - dx2 * dy3) / den;
  m->h = (dx1 * dy3 - dx3 * dy1) / den;
  m->a = x1 - x0 + m->g * x1;
  m->b = x3 - x0 + m->h * x3;
  m->c = x0;
  m->d = y1 - y0 + m->g * y1;
  m->e = y3 - y0 + m->h * y3;
  m->f = y0;
  return true;
}

static void quad_at(FAR const struct quad_fwd_s *m, float u, float v,
                    FAR float *px, FAR float *py)
{
  float wn = m->g * u + m->h * v + 1.0f;

  if (wn < 1e-6f)
    {
      wn = 1e-6f;
    }

  *px = (m->a * u + m->b * v + m->c) / wn;
  *py = (m->d * u + m->e * v + m->f) / wn;
}

/* How much smaller a window in perspective may be pictured: the largest
 * scale of its image on the display, across and down (near its corners,
 * an eighth of it), and the levels whose pixels that shows at most
 * NXCOMP_MIPMAG display pixels large
 */

static int mip_axis(float s2)
{
  /* s2: the scale, squared: the level whose pixels it shows at most
   * NXCOMP_MIPMAG display pixels large
   */

  return s2 * 16.0f <= NXCOMP_MIPMAG * NXCOMP_MIPMAG ? 2 :
         s2 * 4.0f <= NXCOMP_MIPMAG * NXCOMP_MIPMAG ? 1 : 0;
}

static uint8_t comp_miplevel(FAR struct nxbe_window_s *wnd, int bias)
{
  struct quad_fwd_s m;
  int32_t x[4];
  int32_t y[4];
  float sx = 0.0f;
  float sy = 0.0f;
  float ax;
  float ay;
  float bx;
  float by;
  nxgl_coord_t w = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
  nxgl_coord_t h = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;
  int k;

  if (w < MIP_MIN || h < MIP_MIN || wnd->cells == NULL ||
      wnd->cells->cw == 0)
    {
      return 0;
    }

  wnd_corners_at(wnd, &wnd->bounds, x, y);
  if (!quad_fwd(x, y, &m))
    {
      return 0;
    }

  for (k = 0; k < 4; k++)
    {
      float e = (k & 1) ? 1.0f : 0.0f;      /* The edge: v (or u) 0, 1 */
      float o = (k & 2) ? 0.875f : 0.0f;    /* ... its end */
      float d2;

      quad_at(&m, o, e, &ax, &ay);
      quad_at(&m, o + 0.125f, e, &bx, &by);
      d2 = ((bx - ax) * (bx - ax) + (by - ay) * (by - ay)) /
           (w * 0.125f * w * 0.125f);
      sx = d2 > sx ? d2 : sx;

      quad_at(&m, e, o, &ax, &ay);
      quad_at(&m, e, o + 0.125f, &bx, &by);
      d2 = ((bx - ax) * (bx - ax) + (by - ay) * (by - ay)) /
           (h * 0.125f * h * 0.125f);
      sy = d2 > sy ? d2 : sy;
    }

  return (uint8_t)(MAX(mip_axis(sx), bias) | MAX(mip_axis(sy), bias) << 4);
}

static void comp_mipfree(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_mip_s *mip = wnd->mip;

  if (mip != NULL)
    {
      nxbe_cells_drop(mip->cells);
      kmm_free(mip);
      wnd->mip = NULL;
    }
}

/* A picture at the window's level (allocated: its surface, wherever the
 * room is made; to be made): false if there is no room
 */

static bool comp_mipensure(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_mip_s *mip = wnd->mip;
  uint8_t level = wnd->miplevel;
  nxgl_coord_t w = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
  nxgl_coord_t h = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;

  if (mip != NULL &&
      (mip->level != level || mip->ww != w || mip->wh != h))
    {
      comp_mipfree(wnd);
      mip = NULL;
    }

  if (level == 0 || mip != NULL)
    {
      return level != 0;
    }

  mip = kmm_zalloc(sizeof(*mip));
  if (mip == NULL)
    {
      return false;
    }

  mip->level = level;
  mip->ww    = w;
  mip->wh    = h;
  mip->w     = (w + (1 << MIP_LX(level)) - 1) >> MIP_LX(level);
  mip->h     = (h + (1 << MIP_LY(level)) - 1) >> MIP_LY(level);
  mip->cells = nxbe_cells_make(mip->w, mip->h);
  if (mip->cells == NULL)
    {
      kmm_free(mip);
      return false;
    }

  mip->dirty = true;
  mip->fresh = true;
  rect_set(&mip->drect, 0, 0, w - 1, h - 1);
  wnd->mip = mip;
  return true;
}

/****************************************************************************
 * Name: nxbe_compose_mipdrop
 *
 * Description:
 *   Room wanted: the pictures not in use given up.  Returns how many
 *   cells that freed.
 *
 ****************************************************************************/

int nxbe_compose_mipdrop(void)
{
  FAR struct nxbe_window_s *wnd;
  int n = 0;

  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
    {
      if (wnd->mip != NULL && wnd->miplevel == 0)
        {
          n += wnd->mip->cells->cw * wnd->mip->cells->ch;
          comp_mipfree(wnd);
        }
    }

  return n;
}

/****************************************************************************
 * Name: nxbe_compose_mipcells, nxbe_compose_mipfree
 *
 * Description:
 *   A window's smaller picture's cells; the picture gone (made again if
 *   it is wanted).
 *
 ****************************************************************************/

FAR struct nxbe_cells_s *nxbe_compose_mipcells(FAR struct nxbe_window_s *wnd)
{
  return wnd->mip != NULL ? wnd->mip->cells : NULL;
}

void nxbe_compose_mipfree(FAR struct nxbe_window_s *wnd)
{
  comp_mipfree(wnd);
  wnd->miplevel = 0;
  g_comp.needs  = true;
  nxbe_compose_damage_window(wnd);
}

/****************************************************************************
 * Name: nxbe_compose_mipdirty
 *
 * Description:
 *   A window with a picture of it (rel, window coordinates) drawn into.
 *
 ****************************************************************************/

void nxbe_compose_mipdirty(FAR struct nxbe_window_s *wnd,
                           FAR const struct nxgl_rect_s *rel)
{
  FAR struct nxbe_mip_s *mip = wnd->mip;

  if (mip == NULL || rect_empty(rel))
    {
      return;
    }

  if (!mip->dirty)
    {
      mip->drect = *rel;
      mip->dirty = true;
    }
  else
    {
      nxgl_rectunion(&mip->drect, &mip->drect, rel);
    }
}

/* The picture made again where it is out of date: from each piece of the
 * window (its cells resident), drawn scaled by the hardware; from the
 * swap for cells there (the pixels at the middle of each picture
 * pixel's, as the hardware takes them); lost cells black
 */

/* A rectangle of a picture (its own coordinates): a colour, pixels */

static void mip_fill(FAR struct nxbe_mip_s *mip,
                     FAR const struct nxgl_rect_s *r, nxgl_mxpixel_t col)
{
  FAR const struct nxbe_piece_s *m;
  struct nxgl_rect_s x;
  struct nxgl_rect_s abs;
  int nm = nxbe_cells_pieces(mip->cells, &m);
  int q;

  for (q = 0; q < nm; q++)
    {
      if (nxbe_piece_part(&m[q], r, &x, &abs))
        {
          g_nxaccel->fill(&abs, col);
        }
    }
}

static void mip_put(FAR struct nxbe_mip_s *mip,
                    FAR const struct nxgl_rect_s *r, FAR const void *src,
                    unsigned int stride)
{
  FAR const struct nxbe_piece_s *m;
  struct nxgl_rect_s x;
  struct nxgl_rect_s abs;
  struct nxgl_point_s org;
  int nm = nxbe_cells_pieces(mip->cells, &m);
  int q;

  for (q = 0; q < nm; q++)
    {
      if (nxbe_piece_part(&m[q], r, &x, &abs))
        {
          org.x = r->pt1.x + m[q].pos.x - m[q].r.pt1.x;
          org.y = r->pt1.y + m[q].pos.y - m[q].r.pt1.y;
          g_nxaccel->copy(&abs, src, &org, stride);
        }
    }
}

static void comp_mipmake(FAR struct nxbe_window_s *wnd)
{
  FAR struct nxbe_mip_s *mip = wnd->mip;
  FAR struct nxbe_cells_s *c = wnd->cells;
  FAR const struct nxbe_piece_s *p;
  FAR const struct nxbe_piece_s *m;
  struct nxgl_rect_s d;
  struct nxgl_rect_s mclip;
  int lx;
  int ly;
  int sh = g_nxaccel->cellshift;
  int np;
  int nm;
  int k;

  if (mip == NULL || !mip->dirty || c == NULL)
    {
      return;
    }

  mip->dirty = false;
  lx = MIP_LX(mip->level);
  ly = MIP_LY(mip->level);
  rect_set(&d, 0, 0, mip->ww - 1, mip->wh - 1);
  nxgl_rectintersect(&d, &d, &mip->drect);
  if (rect_empty(&d))
    {
      return;
    }

  /* The picture's pixels that out-of-date part is (its pixels the ones
   * whose middles fall there, and a little more: the edges of its pixels)
   */

  rect_set(&mclip, d.pt1.x >> lx, d.pt1.y >> ly, d.pt2.x >> lx,
           d.pt2.y >> ly);

  /* Each piece of the window: its rectangle in the picture, the
   * hardware's scaled quadrilateral of it into each piece of the picture
   */

  np = nxbe_surface_pieces(wnd, &p);
  nm = nxbe_cells_pieces(mip->cells, &m);
  for (k = 0; k < np; k++)
    {
      int32_t x0 = (p[k].r.pt1.x * 16) >> lx;
      int32_t x1 = ((p[k].r.pt2.x + 1) * 16) >> lx;
      int32_t y0 = (p[k].r.pt1.y * 16) >> ly;
      int32_t y1 = ((p[k].r.pt2.y + 1) * 16) >> ly;
      int q;

      for (q = 0; q < nm; q++)
        {
          struct nxgl_rect_s x;
          struct nxgl_rect_s abs;
          int32_t dx = (m[q].pos.x - m[q].r.pt1.x) * 16;
          int32_t dy = (m[q].pos.y - m[q].r.pt1.y) * 16;
          int32_t qx[4];
          int32_t qy[4];

          if (!nxbe_piece_part(&m[q], &mclip, &x, &abs))
            {
              continue;
            }

          qx[0] = qx[3] = x0 + dx;
          qx[1] = qx[2] = x1 + dx;
          qy[0] = qy[1] = y0 + dy;
          qy[2] = qy[3] = y1 + dy;
          g_nxaccel->quad(&p[k].pos, p[k].r.pt2.x - p[k].r.pt1.x + 1,
                          p[k].r.pt2.y - p[k].r.pt1.y + 1, qx, qy, 0, NULL,
                          &abs);
        }
    }

  /* Cells not resident */

  if (c->cw > 0 && c->nres < c->cw * c->ch)
    {
      int i0 = d.pt1.x >> sh;
      int j0 = d.pt1.y >> sh;
      int i1 = MIN(d.pt2.x >> sh, c->cw - 1);
      int j1 = MIN(d.pt2.y >> sh, c->ch - 1);
      int i;
      int j;

      for (j = j0; j <= j1; j++)
        {
          for (i = i0; i <= i1; i++)
            {
              struct nxgl_rect_s cr;
              struct nxgl_rect_s r;
              int kk = j * c->cw + i;
              int xa;
              int xb;
              int ya;
              int yb;

              if (c->map[kk] != NX_CELL_NONE)
                {
                  continue;
                }

              /* The picture's pixels whose middles are in this cell (and
               * the part out of date)
               */

              rect_set(&cr, i << sh, j << sh,
                       MIN(((i + 1) << sh) - 1, mip->ww - 1),
                       MIN(((j + 1) << sh) - 1, mip->wh - 1));
              nxgl_rectintersect(&cr, &cr, &d);
              xa = (cr.pt1.x + (1 << lx) - 1 - ((1 << lx) >> 1)) >> lx;
              xb = (cr.pt2.x - ((1 << lx) >> 1)) >> lx;
              ya = (cr.pt1.y + (1 << ly) - 1 - ((1 << ly) >> 1)) >> ly;
              yb = (cr.pt2.y - ((1 << ly) >> 1)) >> ly;
              xb = MIN(xb, mip->w - 1);
              yb = MIN(yb, mip->h - 1);
              if (xa > xb || ya > yb)
                {
                  continue;
                }

              rect_set(&r, xa, ya, xb, yb);
              if ((c->flags[kk] & NXBE_CELL_SWAP) != 0 &&
                  NXBE_SWAP_ISCOLOUR(c->swap[kk]))
                {
                  mip_fill(mip, &r, NXBE_SWAP_THECOLOUR(c->swap[kk]));
                }
              else if ((c->flags[kk] & NXBE_CELL_SWAP) != 0 && sh <= 5)
                {
                  static uint16_t px[32 * 32];          /* (the server's) */
                  FAR const uint16_t *slot =
                    (FAR const uint16_t *)(nxbe_swap_slot(c->swap[kk]) + 1);
                  int n = xb - xa + 1;
                  int u;
                  int v;

                  for (v = ya; v <= yb; v++)
                    {
                      int sy = (v << ly) + ((1 << ly) >> 1) - (j << sh);

                      for (u = xa; u <= xb; u++)
                        {
                          int sx = (u << lx) + ((1 << lx) >> 1) - (i << sh);

                          px[(v - ya) * n + (u - xa)] = slot[(sy << sh) + sx];
                        }
                    }

                  mip_put(mip, &r, px, n * 2);
                }
              else if (mip->fresh)
                {
                  mip_fill(mip, &r, 0);         /* (lost: as it was) */
                }
            }
        }
    }

  mip->fresh = false;
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

  if (wnd_quad(wnd))
    {
      int32_t x[4];
      int32_t y[4];

      /* Its shadow, under the rectangle that is the quadrilateral's
       * average; its surface in perspective
       */

      if (wnd->fx.shadow > 0)
        {
          struct nxgl_rect_s avg;
          struct nxgl_rect_s in;

          wnd_quadrects_at(wnd, &wnd->bounds, &avg, &in);
          if (avg.pt2.x - avg.pt1.x >= 2 * NXCOMP_R &&
              avg.pt2.y - avg.pt1.y >= 2 * NXCOMP_R)
            {
              comp_shadow_at(wnd, &avg, &in, r, off);
            }
        }

      wnd_corners_at(wnd, &wnd->bounds, x, y);
      for (i = 0; i < 4; i++)
        {
          x[i] += off->x * 16;
          y[i] += off->y * 16;
        }

      nxgl_rectoffset(&dest, r, off->x, off->y);
      if (wnd->mip != NULL && wnd->miplevel != 0 &&
          wnd->mip->level == wnd->miplevel && !wnd->mip->dirty)
        {
          FAR struct nxbe_mip_s *mip = wnd->mip;
          FAR const struct nxbe_piece_s *m;
          int lmax = MAX(MIP_LX(mip->level), MIP_LY(mip->level));
          int nm = nxbe_cells_pieces(mip->cells, &m);

          /* Its smaller picture, in its pieces */

          comp_quadimg(m, nm, mip->w, mip->h,
                       wnd_round(wnd) ? NXCOMP_R >> lmax : 0, x, y, &dest);
        }
      else
        {
          comp_quad(wnd, x, y, &dest);
        }

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

static int rect_minus(FAR const struct nxgl_rect_s *p,
                      FAR const struct nxgl_rect_s *c,
                      FAR struct nxgl_rect_s part[4]);

/* Composite display rectangle r at off, from the top window that hides
 * everything below it up to top (inclusive).  A window in perspective
 * that hides part of r splits what is below it (split: how many more
 * times that may happen), so that what its quadrilateral hides is not
 * drawn for nothing; it, and what is above it, are drawn over all of r
 * (its rows whole, as when nothing splits; and not if a window in
 * perspective below would have its rows cut).
 */

/* Is there a window in perspective below wnd in r whose rows splitting r
 * around c would cut (one beside c, left or right, is not cut)?
 */

static bool quad_below(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *r,
                       FAR const struct nxgl_rect_s *c)
{
  struct nxgl_rect_s box;

  for (wnd = wnd->below; wnd != NULL; wnd = wnd->below)
    {
      if (wnd_quad(wnd) && !NXBE_ISHIDDEN(wnd))
        {
          wnd_quadbox_at(wnd, &wnd->bounds, &box);
          if (nxgl_rectoverlap(&box, (FAR struct nxgl_rect_s *)r) &&
              box.pt2.x >= c->pt1.x && box.pt1.x <= c->pt2.x)
            {
              return true;
            }
        }
    }

  return false;
}

static void comp_rect(FAR const struct nxgl_rect_s *r,
                      FAR const struct nxgl_point_s *off,
                      FAR struct nxbe_window_s *top, int split)
{
  FAR struct nxbe_state_s *be = g_comp.be;
  FAR struct nxbe_window_s *start;
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s ext;
  struct nxgl_rect_s c;

  for (start = top; start != NULL; start = start->below)
    {
      if (wnd_covers(start, r))
        {
          break;
        }

      if (split > 0 && wnd_quadcover(start, &c) &&
          nxgl_rectoverlap(&c, (FAR struct nxgl_rect_s *)r) &&
          !quad_below(start, r, &c))
        {
          struct nxgl_rect_s part[4];
          int n = rect_minus(r, &c, part);
          int k;

          /* Below it: only around what it hides */

          for (k = 0; k < n; k++)
            {
              if (start->below != NULL)
                {
                  comp_rect(&part[k], off, start->below, split - 1);
                }
              else
                {
                  comp_window(&be->bkgd, &part[k], off);
                }
            }

          break;
        }
    }

  if (start == NULL)
    {
      start = &be->bkgd;
    }

  comp_window(start, r, off);
  for (wnd = start->above; wnd != NULL && wnd != top->above;
       wnd = wnd->above)
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

          /* The first part replaces the piece, the rest are appended (they
           * miss this cut rectangle)
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

/* Copy display rectangle src of the front buffer to dest in the back */

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

/* Whether a second buffer of the mode's size can lie in the space at all,
 * beside or below the display at (0, 0): not at 1920 x 1080 in 2048 x
 * 2048, where the display is composited in place (single buffered: a
 * frame being composited may show partly drawn; windows are not evicted
 * for a buffer that cannot be placed)
 */

static bool comp_backfits(void)
{
  FAR struct nxbe_state_s *be = g_comp.be;

  return g_nxaccel->spacew == 0 ||
         g_nxaccel->spacew - be->vinfo.xres >= be->vinfo.xres ||
         g_nxaccel->spaceh - be->vinfo.yres >= be->vinfo.yres;
}

/* Get a second display buffer if there is room, compacting the surfaces
 * for it unless done recently (pack forces it).  The new buffer is empty:
 * all of the display counts as the last frame's change.
 */

static void comp_getback(bool pack)
{
  FAR struct nxbe_state_s *be = g_comp.be;
  struct nxgl_point_s pos;

  g_comp.retry = false;
  if (g_comp.dbuf || g_nxaccel->flip == NULL || !comp_backfits())
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

      g_comp.packed   = true;
      g_comp.lastpack = clock_systime_ticks();
      if (!nxbe_surface_take(be->vinfo.xres, be->vinfo.yres,
                             NX_SURFACE_BUFFER, &pos))
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
  g_comp.dbuf     = false;
  g_comp.nprev    = 0;
  g_comp.lastdrop = clock_systime_ticks();
  g_comp.needs    = true;
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

/* Forget a window's move: it is redrawn instead of copied */

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
         !wnd_quad(wnd) &&
         wnd->fx.backdrop == 0 &&
         m->from.pt2.x - m->from.pt1.x == wnd->bounds.pt2.x -
                                          wnd->bounds.pt1.x &&
         m->from.pt2.y - m->from.pt1.y == wnd->bounds.pt2.y -
                                          wnd->bounds.pt1.y &&
         (g_comp.dbuf || ncopy == 0);  /* One buffer: one copy in place */
}

/* Plan copying a moved window: the copy, and the damage around it in
 * add[].  False if add[] is full.
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

  /* The old pixels that were on the display, to where they land on it */

  nxgl_rectintersect(&copy->src, from, disp);
  nxgl_rectoffset(&copy->dest, &copy->src, dx, dy);
  nxgl_rectintersect(&copy->dest, &copy->dest, disp);
  nxgl_rectoffset(&copy->src, &copy->dest, -dx, -dy);

  /* What it uncovered and its shadows: both extents minus where it is */

  wnd_extent_at(wnd, from, &u);
  wnd_extent_at(wnd, to, &e);
  nxgl_rectunion(&u, &u, &e);
  n = rect_minus(&u, to, part);
  for (i = 0; i < n; i++)
    {
      ok &= list_add(add, nadd, max, &part[i]);
    }

  /* Where it is but the copy didn't reach (it was off the display) */

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

  /* Damage on its old place moves with it */

  for (i = 0; i < g_comp.ndamage; i++)
    {
      nxgl_rectintersect(&x, &g_comp.damage[i], from);
      nxgl_rectoffset(&x, &x, dx, dy);
      ok &= list_add(add, nadd, max, &x);
    }

  /* Windows above it: over the copied pixels (at their old place if they
   * moved too) and over where it is now
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

/* This frame's moves: copies where possible, damage for the rest */

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

/* Whether any of a window is visible: not hidden, partly on the display
 * and not covered by opaque windows above (round corners don't cover)
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

  /* (Shown in perspective, or to be once it has its surface again:
   * where the quadrilateral is; a window shown so covers only the
   * rectangle inside it)
   */

  if ((wnd->fx.flags & NX_FX_QUAD) != 0 && g_nxaccel->quad != NULL)
    {
      wnd_quadbox_at(wnd, &wnd->bounds, &r);
      nxgl_rectintersect(&r, &r, &g_comp.be->bkgd.bounds);
    }
  else
    {
      nxgl_rectintersect(&r, &wnd->bounds, &g_comp.be->bkgd.bounds);
    }

  if (rect_empty(&r))
    {
      return false;
    }

  for (a = wnd->above; a != NULL; a = a->above)
    {
      struct nxgl_rect_s avg;
      struct nxgl_rect_s in;

      b = &a->bounds;
      if (wnd_quad(a))
        {
          wnd_quadrects_at(a, &a->bounds, &avg, &in);
          if (in.pt2.x - in.pt1.x < 2 * NXCOMP_R ||
              in.pt2.y - in.pt1.y < 2 * NXCOMP_R)
            {
              continue;
            }

          b = &in;
        }

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
          FAR const struct nxbe_piece_s *p;
          int np = nxbe_surface_pieces(wnd, &p);

          px = (uint32_t)wnd->surfsize.w * wnd->surfsize.h;
          st.nsurf++;
          st.surfpx += px;
          st.npieces += np;
          st.nsplit  += np > 1;
          if (wnd->cells != NULL && wnd->cells->cw > 0)
            {
              st.ncells += wnd->cells->cw * wnd->cells->ch;
              st.nres   += wnd->cells->nres;
            }
          if (wnd->surfkind == NX_SURFACE_IMAGE)
            {
              st.nimage++;
              st.imagepx += px;
            }
        }
      else if (wnd->parked)
        {
          st.nparked++;
          st.parkedpx += px;
        }
      else if (wnd->fbmem != NULL)
        {
          st.nmem++;
          st.membytes += (uint32_t)wnd->stride *
                         (wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1);
        }
    }

  g_comp.nparked = st.nparked;
  flags  = enter_critical_section();
  g_snap = st;
  leave_critical_section(flags);
  g_comp.changed = false;
}

/* Residency ****************************************************************/

/* The opaque windows above wnd that meet r, as rectangles they surely
 * cover (as comp_seen() has them); false if there are too many to tell
 */

static bool comp_covers(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *r,
                        FAR struct nxgl_rect_s *cover, FAR int *ncover)
{
  FAR struct nxbe_window_s *a;
  FAR const struct nxgl_rect_s *b;
  struct nxgl_rect_s avg;
  struct nxgl_rect_s in;
  int n = 0;

  for (a = wnd->above; a != NULL; a = a->above)
    {
      b = &a->bounds;
      if (wnd_quad(a))
        {
          wnd_quadrects_at(a, &a->bounds, &avg, &in);
          if (in.pt2.x - in.pt1.x < 2 * NXCOMP_R ||
              in.pt2.y - in.pt1.y < 2 * NXCOMP_R)
            {
              continue;
            }

          b = &in;
        }

      if (NXBE_ISHIDDEN(a) || !wnd_pixels(a) || a->fx.opacity != 255 ||
          (a->fx.flags & NX_FX_KEY) != 0 ||
          !nxgl_rectoverlap((FAR struct nxgl_rect_s *)r,
                            (FAR struct nxgl_rect_s *)b))
        {
          continue;
        }

      if (n + 2 > NXCOMP_MAXCOVER)
        {
          return false;
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

  *ncover = n;
  return true;
}

/* The windows in perspective above wnd that meet r and are opaque, as
 * their quadrilaterals (pixels); how many (at most max)
 */

struct nxcomp_poly_s
{
  float x[4];
  float y[4];
  float area;                         /* (twice it, signed: the winding) */
};

static int comp_polys(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *r,
                      FAR struct nxcomp_poly_s *poly, int max)
{
  FAR struct nxbe_window_s *a;
  struct nxgl_rect_s box;
  int32_t x[4];
  int32_t y[4];
  int n = 0;
  int k;

  for (a = wnd->above; a != NULL && n < max; a = a->above)
    {
      if (!wnd_quad(a) || NXBE_ISHIDDEN(a) || a->fx.opacity != 255 ||
          (a->fx.flags & NX_FX_KEY) != 0)
        {
          continue;
        }

      wnd_quadbox_at(a, &a->bounds, &box);
      if (!nxgl_rectoverlap(&box, (FAR struct nxgl_rect_s *)r))
        {
          continue;
        }

      wnd_corners_at(a, &a->bounds, x, y);
      poly[n].area = 0.0f;
      for (k = 0; k < 4; k++)
        {
          poly[n].x[k] = x[k] / 16.0f;
          poly[n].y[k] = y[k] / 16.0f;
        }

      for (k = 0; k < 4; k++)
        {
          poly[n].area += poly[n].x[k] * poly[n].y[(k + 1) & 3] -
                          poly[n].x[(k + 1) & 3] * poly[n].y[k];
        }

      if (poly[n].area < -1.0f || poly[n].area > 1.0f)
        {
          n++;
        }
    }

  return n;
}

/* Does the quadrilateral surely hide rectangle r (its round corners, of
 * radius NXCOMP_R at most on the display, kept clear of)?
 */

static bool poly_hides(FAR const struct nxcomp_poly_s *p,
                       FAR const struct nxgl_rect_s *r)
{
  float rx[4];
  float ry[4];
  int k;
  int q;

  rx[0] = rx[3] = r->pt1.x;
  rx[1] = rx[2] = r->pt2.x + 1;
  ry[0] = ry[1] = r->pt1.y;
  ry[2] = ry[3] = r->pt2.y + 1;
  for (k = 0; k < 4; k++)
    {
      float ex = p->x[(k + 1) & 3] - p->x[k];
      float ey = p->y[(k + 1) & 3] - p->y[k];

      /* Its corner's round cut: no part of r near the corner */

      if (p->x[k] > r->pt1.x - 2 * NXCOMP_R &&
          p->x[k] < r->pt2.x + 1 + 2 * NXCOMP_R &&
          p->y[k] > r->pt1.y - 2 * NXCOMP_R &&
          p->y[k] < r->pt2.y + 1 + 2 * NXCOMP_R)
        {
          return false;
        }

      for (q = 0; q < 4; q++)
        {
          float c = ex * (ry[q] - p->y[k]) - ey * (rx[q] - p->x[k]);

          if ((p->area > 0.0f && c < 0.0f) || (p->area < 0.0f && c > 0.0f))
            {
              return false;
            }
        }
    }

  return true;
}

/* Which cells a window needs: those seen (not covered by opaque windows
 * above, on the display), all of a hidden window's that may not be parked
 * (images and glyphs that others draw from), none of a hidden one's that
 * may; in perspective, those whose quadrilaterals are seen
 */

static void comp_need(FAR struct nxbe_window_s *wnd)
{
  static struct nxgl_rect_s cover[NXCOMP_MAXCOVER];   /* (the server's) */
  static struct nxgl_rect_s piece[NXCOMP_MAXPIECES];
  static struct nxcomp_poly_s poly[NXCOMP_MAXPOLY];
  FAR struct nxbe_cells_s *c = wnd->cells;
  struct nxgl_rect_s r;
  int sh = g_nxaccel->cellshift;
  int npoly;
  int ncover;
  int npiece;
  int n = c->cw * c->ch;
  int i;
  int j;
  int k;

  for (k = 0; k < n; k++)
    {
      c->flags[k] &= ~NXBE_CELL_NEED;
    }

  /* A game has the display: nothing of any window is seen (all of them
   * may go to the swap; hidden ones too, for the game's room)
   */

  if (g_comp.game)
    {
      return;
    }

  if (NXBE_ISHIDDEN(wnd))
    {
      if ((wnd->fx.flags & NX_FX_PARK) == 0)
        {
          for (k = 0; k < n; k++)
            {
              c->flags[k] |= NXBE_CELL_NEED;
            }
        }

      return;
    }

  /* In perspective, drawn from a smaller picture of it: none */

  if ((wnd->fx.flags & NX_FX_QUAD) != 0 && g_nxaccel->quad != NULL &&
      wnd->mip != NULL && wnd->miplevel != 0 &&
      wnd->mip->level == wnd->miplevel)
    {
      return;
    }

  if ((wnd->fx.flags & NX_FX_QUAD) != 0 && g_nxaccel->quad != NULL)
    {
      int32_t x[4];
      int32_t y[4];
      int32_t bx16;
      int32_t by16;
      float x0;
      float y0;
      float x1;
      float y1;
      float x2;
      float y2;
      float x3;
      float y3;
      float dx1;
      float dx2;
      float dx3;
      float dy1;
      float dy2;
      float dy3;
      float den;
      float a;
      float b;
      float cc;
      float d;
      float e;
      float f;
      float g;
      float hh;
      nxgl_coord_t w = wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1;
      nxgl_coord_t h = wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1;

      /* Each cell's quadrilateral's box, seen unless an opaque window
       * above surely covers it (or off the display)
       */

      wnd_quadbox_at(wnd, &wnd->bounds, &r);
      if (!comp_covers(wnd, &r, cover, &ncover))
        {
          ncover = 0;
        }

      npoly = comp_polys(wnd, &r, poly, NXCOMP_MAXPOLY);

      wnd_corners_at(wnd, &wnd->bounds, x, y);
      bx16 = MIN(MIN(x[0], x[1]), MIN(x[2], x[3])) & ~15;
      by16 = MIN(MIN(y[0], y[1]), MIN(y[2], y[3])) & ~15;
      x0 = (x[0] - bx16) / 16.0f;
      y0 = (y[0] - by16) / 16.0f;
      x1 = (x[1] - bx16) / 16.0f;
      y1 = (y[1] - by16) / 16.0f;
      x2 = (x[2] - bx16) / 16.0f;
      y2 = (y[2] - by16) / 16.0f;
      x3 = (x[3] - bx16) / 16.0f;
      y3 = (y[3] - by16) / 16.0f;
      dx1 = x1 - x2;
      dx2 = x3 - x2;
      dx3 = x0 - x1 + x2 - x3;
      dy1 = y1 - y2;
      dy2 = y3 - y2;
      dy3 = y0 - y1 + y2 - y3;
      den = dx1 * dy2 - dx2 * dy1;
      if (den > -1e-6f && den < 1e-6f)
        {
          return;
        }

      g  = (dx3 * dy2 - dx2 * dy3) / den;
      hh = (dx1 * dy3 - dx3 * dy1) / den;
      a  = x1 - x0 + g * x1;
      b  = x3 - x0 + hh * x3;
      cc = x0;
      d  = y1 - y0 + g * y1;
      e  = y3 - y0 + hh * y3;
      f  = y0;

      for (k = 0; k < n; k++)
        {
          float xmin = 1e9f;
          float ymin = 1e9f;
          float xmax = -1e9f;
          float ymax = -1e9f;
          int q;

          for (q = 0; q < 4; q++)
            {
              float u = (float)((k % c->cw + (q == 1 || q == 2)) << sh) / w;
              float v = (float)((k / c->cw + (q >= 2)) << sh) / h;
              float wn = g * u + hh * v + 1.0f;
              float px;
              float py;

              u  = u > 1.0f ? 1.0f : u;
              v  = v > 1.0f ? 1.0f : v;
              wn = g * u + hh * v + 1.0f;
              if (wn < 1e-6f)
                {
                  xmin = -1e9f;               /* (past the horizon: all) */
                  ymin = -1e9f;
                  xmax = 1e9f;
                  ymax = 1e9f;
                  break;
                }

              px = (a * u + b * v + cc) / wn + bx16 / 16.0f;
              py = (d * u + e * v + f) / wn + by16 / 16.0f;
              xmin = px < xmin ? px : xmin;
              xmax = px > xmax ? px : xmax;
              ymin = py < ymin ? py : ymin;
              ymax = py > ymax ? py : ymax;
            }

          rect_set(&r, xmin < -30000.0f ? -30000 : (nxgl_coord_t)xmin - 1,
                   ymin < -30000.0f ? -30000 : (nxgl_coord_t)ymin - 1,
                   xmax > 30000.0f ? 30000 : (nxgl_coord_t)xmax + 1,
                   ymax > 30000.0f ? 30000 : (nxgl_coord_t)ymax + 1);
          nxgl_rectintersect(&r, &r, &g_comp.be->bkgd.bounds);
          if (rect_empty(&r))
            {
              continue;
            }

          for (i = 0; i < ncover && !rect_contains(&cover[i], &r); i++)
            {
            }

          for (j = 0; i == ncover && j < npoly && !poly_hides(&poly[j], &r);
               j++)
            {
            }

          if (i == ncover && j == npoly)
            {
              c->flags[k] |= NXBE_CELL_NEED;
            }
        }

      return;
    }

  /* What of it is on the display, but for what opaque windows cover */

  nxgl_rectintersect(&r, &wnd->bounds, &g_comp.be->bkgd.bounds);
  if (rect_empty(&r))
    {
      return;
    }

  if (!comp_covers(wnd, &r, cover, &ncover) ||
      (npiece = rect_cut(&r, cover, ncover, piece)) < 0)
    {
      npiece = 1;
      piece[0] = r;
    }

  for (k = 0; k < npiece; k++)
    {
      int i0 = (piece[k].pt1.x - wnd->bounds.pt1.x) >> sh;
      int j0 = (piece[k].pt1.y - wnd->bounds.pt1.y) >> sh;
      int i1 = (piece[k].pt2.x - wnd->bounds.pt1.x) >> sh;
      int j1 = (piece[k].pt2.y - wnd->bounds.pt1.y) >> sh;

      i1 = MIN(i1, c->cw - 1);
      j1 = MIN(j1, c->ch - 1);
      for (j = MAX(j0, 0); j <= j1; j++)
        {
          for (i = MAX(i0, 0); i <= i1; i++)
            {
              c->flags[j * c->cw + i] |= NXBE_CELL_NEED;
            }
        }
    }
}

/****************************************************************************
 * Name: nxbe_compose_needs
 *
 * Description:
 *   Which cells the windows need (NXBE_CELL_NEED), worked out again if
 *   windows have moved, been restacked, shown, hidden, parked..., or if all
 *   is set.
 *
 ****************************************************************************/

void nxbe_compose_needs(bool all)
{
  FAR struct nxbe_window_s *wnd;

  if (!g_comp.on || g_nxaccel->cells_alloc == NULL ||
      (!g_comp.needs && !all))
    {
      return;
    }

  g_comp.needs = false;
  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
    {
      if (wnd->surface && wnd->cells != NULL && wnd->cells->cw > 0)
        {
          comp_need(wnd);
        }
    }
}

/* Before a frame: every window has the cells it needs (put back from the
 * swap, or drawn again by its client); one that cannot have them goes to
 * memory (in perspective: what it lacks is not drawn)
 */

static void comp_resident(void)
{
  FAR struct nxbe_window_s *wnd;
  FAR struct nxbe_window_s *next;
  int bias;

  if (g_nxaccel->cells_alloc == NULL)
    {
      return;
    }

  /* Windows drawn small in perspective: a smaller picture of each (its
   * cells then not needed); the others none.  If the windows in
   * perspective would not fit as they are (as the switcher opens, when
   * they are still large), smaller pictures of them all (bias): for a
   * moment a little coarse rather than the second display buffer given
   * up.
   */

  {
    int room = g_nxaccel->cells_free_count() - NXCOMP_SWAPFREE;
    int need[3];
    int full = 0;
    int other = 0;
    int b;

    need[0] = need[1] = need[2] = 0;
    for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
      {
        FAR struct nxbe_cells_s *c = wnd->cells;
        int k;

        if (!wnd->surface || c == NULL || c->cw == 0)
          {
            continue;
          }

        room += c->nres;
        if (wnd->mip != NULL)
          {
            /* (Its picture's room is room for its next one) */

            room += wnd->mip->cells->cw * wnd->mip->cells->ch;
          }

        if (!NXBE_ISHIDDEN(wnd) && wnd_quad(wnd))
          {
            /* What it takes at each bias: its picture's cells, or all;
             * and without one
             */

            full += c->cw * c->ch;
            for (b = 0; b < 3; b++)
              {
                uint8_t l = comp_miplevel(wnd, b);

                need[b] += (c->cw * c->ch) >> (MIP_LX(l) + MIP_LY(l));
              }
          }
        else
          {
            for (k = 0; k < c->cw * c->ch; k++)
              {
                other += (c->flags[k] & NXBE_CELL_NEED) != 0;
              }
          }
      }

    for (bias = 0; bias < 2 && other + need[bias] > room; bias++)
      {
      }

    /* (Room for them all as they are: no pictures) */

    if (other + full <= room)
      {
        bias = -1;
      }
  }

  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
    {
      FAR struct nxbe_mip_s *mip = wnd->mip;
      uint8_t level = 0;

      if (bias >= 0 && wnd->surface && !NXBE_ISHIDDEN(wnd) &&
          wnd_quad(wnd))
        {
          level = comp_miplevel(wnd, bias);
        }

      /* A picture it has that is as fine or finer will do (made again
       * only when a finer one is wanted: not at every step a card takes);
       * one it does not use now is kept until the room is wanted
       */

      if (level != 0 && mip != NULL &&
          MIP_LX(mip->level) <= MIP_LX(level) &&
          MIP_LY(mip->level) <= MIP_LY(level) &&
          mip->ww == wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1 &&
          mip->wh == wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1)
        {
          level = mip->level;
        }

      if (level == wnd->miplevel &&
          (level == 0 || (mip != NULL && mip->level == level)))
        {
          continue;
        }

      wnd->miplevel = level;
      if (level != 0 && !comp_mipensure(wnd))
        {
          wnd->miplevel = 0;
          comp_mipfree(wnd);
        }

      g_comp.needs = true;
      nxbe_compose_damage_window(wnd);
    }

  nxbe_compose_needs(false);
  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = next)
    {
      FAR struct nxbe_cells_s *c = wnd->cells;

      next = wnd->surfnext;
      if (!wnd->surface || c == NULL || c->cw == 0)
        {
          continue;
        }

      /* (In perspective, as the switcher's cards are, the frame does
       * not wait for what is drawn again: the cards move on)
       */

      if (c->nres == c->cw * c->ch ||
          nxbe_surface_ensure(wnd, NXBE_ENSURE_ROOM |
                                   (wnd_quad(wnd) ? 0 : NXBE_ENSURE_HOLD)))
        {
          /* In perspective: in one piece if it can be (it then looks the
           * same whatever its cells are)
           */

          if ((wnd->fx.flags & NX_FX_QUAD) != 0 && g_nxaccel->quad != NULL)
            {
              nxbe_surface_defrag(wnd);
            }

          continue;
        }

      if ((wnd->fx.flags & NX_FX_QUAD) != 0 && g_nxaccel->quad != NULL)
        {
          /* In perspective: a smaller picture of it instead (or what it
           * lacks is not drawn)
           */

          uint8_t level = comp_miplevel(wnd, 1);

          if (level != wnd->miplevel)
            {
              wnd->miplevel = level;
              if (!comp_mipensure(wnd))
                {
                  wnd->miplevel = 0;
                  comp_mipfree(wnd);
                }

              g_comp.needs = true;
              nxbe_compose_damage_window(wnd);
            }

          continue;
        }

      gwarn("WARNING: no room for what a window shows: in memory\n");
      nxbe_srccache_free(wnd);
      nxbe_surface_close(wnd);
      nxbe_surface_open(wnd);
      if (!wnd->surface)
        {
          nxbe_surface_tomemory(wnd);
        }
      else
        {
          nxbe_compose_redrawing(wnd);
          nxmu_redrawreq(wnd, &wnd->bounds);
        }

      nxbe_compose_damage_window(wnd);
    }

  /* The smaller pictures made where they are out of date (where cells
   * are lost, not drawn again for it: as they were, a picture black there
   * at first)
   */

  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
    {
      if (wnd->mip != NULL && wnd->miplevel != 0)
        {
          comp_mipmake(wnd);
        }
    }
}

/* The server is idle and windows are still: cells that windows do not
 * need read back into the swap, so that some are free (NXCOMP_SWAPFREE)
 * for what will be seen next, and enough for a second display buffer if
 * the windows' needs leave room for one; then the buffer taken back
 */

static void comp_swapidle(FAR int32_t *next)
{
  FAR struct nxbe_window_s *wnd;
  int sh = g_nxaccel->cellshift;
  int bufcells = 0;
  int spare = 0;
  int nfree;
  int want;
  int k;

  nxbe_compose_needs(false);
  nfree = g_nxaccel->cells_free_count();
  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
    {
      FAR struct nxbe_cells_s *c = wnd->cells;

      if (!wnd->surface || c == NULL || c->cw == 0)
        {
          continue;
        }

      for (k = 0; k < c->cw * c->ch; k++)
        {
          spare += c->map[k] != NX_CELL_NONE &&
                   (c->flags[k] & NXBE_CELL_NEED) == 0;
        }
    }

  if (!g_comp.dbuf && g_nxaccel->flip != NULL && !g_comp.game &&
      comp_backfits() &&
      clock_systime_ticks() - g_comp.lastdrop >= NXCOMP_BUFAGAIN)
    {
      bufcells = (((g_comp.be->vinfo.xres + (1 << sh) - 1) >> sh) *
                  ((g_comp.be->vinfo.yres + (1 << sh) - 1) >> sh));
      if (nfree + spare < bufcells + NXCOMP_SWAPFREE)
        {
          bufcells = 0;                   /* (No room for it) */
        }
    }

  want = bufcells + NXCOMP_SWAPFREE - nfree;
  if (want > 0 && spare > 0)
    {
      nxbe_surface_evict(MIN(want, NXCOMP_SWAPSTEP), true);
      if (want > NXCOMP_SWAPSTEP)
        {
          *next = *next < 0 ? NXCOMP_IDLEPOLL : MIN(*next, NXCOMP_IDLEPOLL);
          return;
        }
    }

  if (bufcells > 0 &&
      g_nxaccel->cells_free_count() >= bufcells + NXCOMP_SWAPFREE / 2)
    {
      comp_getback(true);
      if (g_comp.dbuf)
        {
          nxbe_compose_damage(&g_comp.be->bkgd.bounds);
        }
    }

  /* Then copies of the cells not needed (they give their cells up for
   * nothing when room is wanted while windows move)
   */

  else if (nxbe_surface_clean(NXCOMP_SWAPSTEP) >= NXCOMP_SWAPSTEP)
    {
      *next = *next < 0 ? NXCOMP_IDLEPOLL : MIN(*next, NXCOMP_IDLEPOLL);
    }
}

/* Before a frame: unpark windows that are visible again and move windows
 * in memory into surfaces if there is room.  False if the frame should wait
 * for windows being redrawn.
 */

static bool comp_unpark(void)
{
  FAR struct nxbe_window_s *wnd;
  clock_t now = clock_systime_ticks();

  if (g_comp.nparked > 0 || g_comp.changed)
    {
      for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
        {
          if (wnd->parked && comp_seen(wnd))
            {
              nxbe_surface_unpark(wnd);
            }
        }

      if (g_comp.changed)
        {
          comp_snapshot();
        }
    }

  comp_resident();
  if (g_comp.retry && nxbe_surface_frommemory() > 0)
    {
      nxbe_srccache_trim();
      comp_snapshot();
    }

  if (g_comp.hold && (int32_t)(g_comp.holduntil - now) <= 0)
    {
      g_comp.hold  = false;
      g_comp.nheld = 0;
      g_nxbe_counts.holdticks += now - g_comp.holdsince;
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

  g_comp.on    = true;
  g_comp.needs = true;
  g_comp.race  = NXCOMP_RACEBAND;
  if (g_nxaccel->cells_alloc != NULL)
    {
      nxbe_swap_init(g_nxaccel->cellshift);
    }

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
      {
        FAR struct nxbe_window_s *wnd;

        for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
          {
            comp_mipfree(wnd);
            wnd->miplevel = 0;
          }
      }

      g_comp.nprev   = 0;
      g_comp.nmoves  = 0;
      g_comp.ndamage = 0;
      return;
    }

  comp_getback(true);
  g_comp.needs = true;
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

  /* Reaching into a window in perspective: all of it, so that its rows
   * are always drawn whole (where a row starts decides which texels it
   * shows: a part drawn again must look the same)
   */

  {
    FAR struct nxbe_window_s *wnd;
    struct nxgl_rect_s box;
    bool grown;

    do
      {
        grown = false;
        for (wnd = g_comp.be->topwnd; wnd != NULL; wnd = wnd->below)
          {
            if (!wnd_quad(wnd) || NXBE_ISHIDDEN(wnd))
              {
                continue;
              }

            wnd_quadbox_at(wnd, &wnd->bounds, &box);
            nxgl_rectintersect(&box, &box, &g_comp.be->bkgd.bounds);
            if (nxgl_rectoverlap(&box, &r) && !rect_contains(&r, &box))
              {
                nxgl_rectunion(&r, &r, &box);
                grown = true;             /* (It may reach more now) */
              }
          }
      }
    while (grown);
  }

  /* One that overlaps it much becomes one with it, so that what moves a
   * little (a window in perspective: its quadrilateral then and now) is
   * drawn once, not again where the two overlap
   */

  for (i = 0; i < g_comp.ndamage; i++)
    {
      FAR struct nxgl_rect_s *a = &g_comp.damage[i];
      struct nxgl_rect_s x;

      nxgl_rectintersect(&x, a, &r);
      if (rect_empty(&x))
        {
          continue;
        }

      nxgl_rectunion(&u, a, &r);
      if (4 * (rect_area(&u) - rect_area(a) - rect_area(&r) +
               rect_area(&x)) <= rect_area(&u))
        {
          r = u;
          g_comp.damage[i] = g_comp.damage[--g_comp.ndamage];
          i = -1;                         /* (Again, with the union) */
        }
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

static void comp_drew(FAR struct nxbe_window_s *wnd);

void nxbe_compose_damage_wrect(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *rel)
{
  struct nxgl_rect_s r;

  if (g_comp.nheld > 0)
    {
      comp_drew(wnd);
    }

  if (NXBE_ISHIDDEN(wnd))
    {
      return;
    }

  if (wnd_quad(wnd))
    {
      wnd_quadbox_at(wnd, &wnd->bounds, &r);  /* (it shows anywhere in it) */
      nxbe_compose_damage(&r);
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

  g_comp.needs = true;
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
  g_comp.needs    = true;
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
  g_comp.needs = true;
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
  wnd->mip        = NULL;
  wnd->miplevel   = 0;

  if (g_comp.on)
    {
      nxbe_compose_restack(wnd, true);
      nxbe_compose_damage_window(wnd);
    }
}

void nxbe_compose_close(FAR struct nxbe_window_s *wnd)
{
  comp_drew(wnd);
  nxbe_compose_damage_window(wnd);
  comp_mipfree(wnd);
  wnd->miplevel = 0;
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

/* A copy within a window whose surface is in pieces (window coordinates,
 * clipped to it)
 */

static void comp_selfcopy(FAR struct nxbe_window_s *wnd,
                          FAR const struct nx_gpuop_s *op)
{
  struct nxgl_rect_s win;
  struct nxgl_rect_s d;
  struct nxgl_rect_s s;
  struct nxgl_point_s off;

  rect_set(&win, 0, 0, wnd->surfsize.w - 1, wnd->surfsize.h - 1);
  rect_set(&d, op->x, op->y, op->x + op->w - 1, op->y + op->h - 1);
  nxgl_rectintersect(&d, &d, &win);
  off.x = op->x - op->sx;
  off.y = op->y - op->sy;
  nxgl_rectoffset(&s, &d, -off.x, -off.y);
  nxgl_rectintersect(&s, &s, &win);
  if (!rect_empty(&s))
    {
      nxbe_surface_move(wnd, &s, &off);
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
  FAR const struct nxbe_piece_s *p;
  struct nxgl_rect_s clip;
  struct nxgl_rect_s dirty;
  struct nxgl_rect_s r;
  struct nxgl_point_s org;
  int np;
  int i;
  int k;

  if (!g_comp.on)
    {
      return;
    }

  if (wnd->parked)
    {
      /* Parked: drawn again in full when it is seen */

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

  rect_set(&dirty, INT16_MAX, INT16_MAX, INT16_MIN, INT16_MIN);
  if (wnd->mip != NULL)
    {
      /* (What it will draw over put back first: nxbe_surface_drawing()) */

      for (i = 0; i < nops; i++)
        {
          rect_set(&r, ops[i].x, ops[i].y, ops[i].x + ops[i].w - 1,
                   ops[i].y + ops[i].h - 1);
          if (!rect_empty(&r))
            {
              nxgl_rectunion(&dirty, &dirty, &r);
            }
        }

      if (!rect_empty(&dirty))
        {
          nxbe_surface_drawing(wnd, &dirty);
        }

      rect_set(&dirty, INT16_MAX, INT16_MAX, INT16_MIN, INT16_MIN);
    }

  np = nxbe_surface_pieces(wnd, &p);
  g_comp.target = wnd;
  for (i = 0; i < nops; i++)
    {
      FAR const struct nx_gpuop_s *op = &ops[i];

      if (np > 1 && op->op == NX_GPUOP_COPY &&
          (op->src == NULL || op->src == (FAR void *)wnd))
        {
          /* A copy within the window, across pieces */

          comp_selfcopy(wnd, op);
        }
      else
        {
          /* Into each piece, what of the operation lies there */

          for (k = 0; k < np; k++)
            {
              org.x = p[k].pos.x - p[k].r.pt1.x;
              org.y = p[k].pos.y - p[k].r.pt1.y;
              nxgl_rectoffset(&clip, &p[k].r, org.x, org.y);
              comp_gpuop(wnd, op, &org, &clip);
            }
        }

      rect_set(&r, ops[i].x, ops[i].y, ops[i].x + ops[i].w - 1,
               ops[i].y + ops[i].h - 1);
      if (!rect_empty(&r))
        {
          nxgl_rectunion(&dirty, &dirty, &r);
        }
    }

  g_comp.target = NULL;
  if (!rect_empty(&dirty))
    {
      if (wnd->srccache != NULL)
        {
          nxbe_srccache_drop(wnd, &dirty);
        }

      nxbe_surface_drawn(wnd, &dirty);
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
  return g_comp.on && !g_comp.game &&
         (g_comp.ndamage > 0 || g_comp.nmoves > 0) &&
         clock_systime_ticks() - g_comp.since >= NXCOMP_LATENCY &&
         (!g_comp.hold ||
          (int32_t)(g_comp.holduntil - clock_systime_ticks()) <= 0);
}

int nxbe_compose_wait(void)
{
  clock_t elapsed;

  if (!g_comp.on || g_comp.game ||
      (g_comp.ndamage == 0 && g_comp.nmoves == 0 &&
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

/* A frame composited in place behind the beam (nx_accel_s waitline): the
 * damage and the moved windows' new places, top to bottom in bands of
 * g_comp.race lines; before a band's first rectangle the wait for the
 * scanout to have read the band's last line and not gone half a frame past
 * its first (a band then has at least about half a frame to be drawn
 * before the scanout reads it again).  A rectangle is cut only between
 * rows (a window in perspective keeps its rows whole: their texels are
 * the same).  Each piece is composited from the windows' surfaces, as a
 * whole frame would be, so the picture is the same as without bands.
 */

static void comp_race(FAR const struct nxcomp_copy_s *copy, int ncopy,
                      FAR const struct nxgl_point_s *off)
{
  FAR struct nxbe_state_s *be = g_comp.be;
  struct nxgl_rect_s all[NXCOMP_MAXPREV];
  struct nxgl_rect_s piece;
  nxgl_coord_t lines = be->vinfo.yres;
  nxgl_coord_t band = g_comp.race;
  nxgl_coord_t top = lines;
  nxgl_coord_t bot = -1;
  nxgl_coord_t t;
  int n = 0;
  int i;

  for (i = 0; i < g_comp.ndamage; i++)
    {
      all[n++] = g_comp.damage[i];
    }

  for (i = 0; i < ncopy; i++)
    {
      all[n++] = copy[i].dest;
    }

  for (i = 0; i < n; i++)
    {
      top = MIN(top, all[i].pt1.y);
      bot = MAX(bot, all[i].pt2.y);
    }

  top = MAX(top, 0);
  bot = MIN(bot, lines - 1);
  for (t = top - top % band; t <= bot; t += band)
    {
      nxgl_coord_t b = MIN(t + band - 1, lines - 1);
      int m = t + lines / 2;
      bool waited = false;

      for (i = 0; i < n; i++)
        {
          if (all[i].pt2.y < t || all[i].pt1.y > b)
            {
              continue;
            }

          piece = all[i];
          piece.pt1.y = MAX(piece.pt1.y, t);
          piece.pt2.y = MIN(piece.pt2.y, b);
          if (!waited)
            {
              g_nxaccel->waitline(b, m < lines ? m : -1);
              g_nxbe_counts.bands++;
              waited = true;
            }

          comp_rect(&piece, off, be->topwnd, NXCOMP_QUADSPLIT);
        }
    }

  g_nxbe_counts.raced++;
}

static uint32_t comp_us(clock_t t0)
{
  struct timespec ts;

  perf_convert(perf_gettime() - t0, &ts);
  return (uint32_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

void nxbe_compose_run(void)
{
  static struct nxcomp_copy_s copy[NXCOMP_MAXMOVES];   /* (the server's) */
  static struct nxgl_rect_s redraw[NXCOMP_MAXPREV];
  struct nxgl_point_s off;
  clock_t t0;
  int nredraw = 0;
  int ncopy;
  int i;

  if (!g_comp.on || g_comp.game)
    {
      return;
    }

  /* Windows seen again get their pixels back first */

  t0 = perf_gettime();
  if (!comp_unpark())
    {
      g_nxbe_counts.prepus += comp_us(t0);
      return;
    }

  g_nxbe_counts.prepus += comp_us(t0);

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
   * windows from where they were, then the damage.  In place (one
   * buffer), behind the beam if the hardware can wait for it.
   */

  off = g_comp.buf[comp_back()];
  if (!g_comp.dbuf && g_comp.race > 0 && g_nxaccel->waitline != NULL &&
      g_nxaccel->waitline(-1, 0) == 0)
    {
      t0 = perf_gettime();
      comp_race(copy, ncopy, &off);
      g_nxbe_counts.drawus += comp_us(t0);
      g_comp.ndamage = 0;
      g_nxbe_counts.frames++;
      return;
    }

  if (g_comp.dbuf)
    {
      comp_age(redraw, nredraw);
    }

  for (i = 0; i < ncopy; i++)
    {
      comp_bufcopy(&copy[i].src, &copy[i].dest);
    }

  t0  = perf_gettime();
  for (i = 0; i < g_comp.ndamage; i++)
    {
      comp_rect(&g_comp.damage[i], &off, g_comp.be->topwnd,
                NXCOMP_QUADSPLIT);
    }

  g_nxbe_counts.drawus += comp_us(t0);

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
 * Name: nxbe_compose_pressure
 *
 * Description:
 *   Between messages: whether a game is starting or has taken the display
 *   (nx_accel_s game).  Starting (1: it wants the CPU's memory first), the
 *   swap is given back and kept empty: windows whose pixels are partly
 *   there are parked (drawn again when seen), the copies of cells kept are
 *   dropped.  With the display taken (2), nothing is composited and the
 *   windows hold no memory at all, in the GPU or the CPU's: every window
 *   is parked (its surface or cells, or its memory, given up; opened while
 *   the game runs, it starts parked), the second display buffer, the
 *   staging band and the smaller pictures go too.  Either way the driver
 *   is told once it is done (gameready()), so that the game's first
 *   allocations find the room.  Given back (0), the swap is used again
 *   and the display composited: the windows that may not be parked
 *   (NX_FX_PARK) are unparked at once, the others when they are seen, and
 *   their clients draw them again.
 *
 ****************************************************************************/

void nxbe_compose_pressure(void)
{
  FAR struct nxbe_window_s *wnd;
  uint32_t seq = 0;
  bool wasgame = g_comp.game;
  int state;

  if (!g_comp.on || g_nxaccel->game == NULL)
    {
      return;
    }

  state = g_nxaccel->game(&seq);
  if (seq == g_comp.gameseq)
    {
      return;
    }

  g_comp.gameseq = seq;
  g_comp.game    = state >= 2;
  g_comp.needs   = true;
  g_comp.changed = true;
  nxbe_swap_enable(state == 0);
  if (state >= 2)
    {
      comp_dropback();
      comp_unstage();
      for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
        {
          comp_mipfree(wnd);
          wnd->miplevel = 0;
          if (!NXBE_ISLAYER(wnd))
            {
              nxbe_srccache_free(wnd);
              nxbe_surface_park(wnd);
            }
        }

      g_comp.hold  = false;
      g_comp.nheld = 0;
      nxbe_srccache_trim();
      nxbe_accel_flush();
    }
  else if (state == 1)
    {
      /* The swap emptied (and windows in memory out of it); the display
       * goes on being composited.  A window that may not be parked gets a
       * surface again at once, its client asked to draw it (hidden ones
       * too: an atlas is drawn from)
       */

      for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
        {
          if (NXBE_ISLAYER(wnd) || wnd->parked ||
              (wnd->surface && nxbe_surface_unswap(wnd)))
            {
              continue;
            }

          nxbe_srccache_free(wnd);
          if (nxbe_surface_park(wnd) && (wnd->fx.flags & NX_FX_PARK) == 0)
            {
              nxbe_surface_unpark(wnd);
            }
        }
    }

  if (wasgame && !g_comp.game)
    {
      /* Windows that may not be parked get their surfaces back now, seen
       * or not (hidden ones may be sources, an atlas: their clients draw
       * them again on request); the others when they are seen
       */

      for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
        {
          if (wnd->parked && !NXBE_ISLAYER(wnd) &&
              (wnd->fx.flags & NX_FX_PARK) == 0)
            {
              nxbe_surface_unpark(wnd);
            }
        }

      g_comp.lastdrop = clock_systime_ticks() - NXCOMP_BUFAGAIN;
      g_comp.retry    = true;
      g_comp.nprev    = 0;
      nxbe_compose_damage(&g_comp.be->bkgd.bounds);
    }

  if (g_nxaccel->gameready != NULL)
    {
      g_nxaccel->gameready(seq);
    }
}

/****************************************************************************
 * Name: nxbe_compose_gaming
 *
 * Description:
 *   Whether a game has the display (nxbe_compose_pressure()): windows are
 *   then opened parked.
 *
 ****************************************************************************/

bool nxbe_compose_gaming(void)
{
  return g_comp.game;
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
  g_comp.needs   = true;
}

void nxbe_compose_redrawing(FAR struct nxbe_window_s *wnd)
{
  int i;

  /* The frame waits until each window asked has drawn (or a while) */

  for (i = 0; i < g_comp.nheld && g_comp.held[i] != wnd; i++)
    {
    }

  if (i == g_comp.nheld && i < NXCOMP_MAXHELD)
    {
      g_comp.held[g_comp.nheld++] = wnd;
    }

  if (!g_comp.hold)
    {
      g_comp.hold      = true;
      g_comp.holdsince = clock_systime_ticks();
      g_comp.holduntil = g_comp.holdsince + NXCOMP_REDRAWHOLD;
      g_nxbe_counts.holds++;
    }
}

/* A window waited for has started drawing: the frame waits only a little
 * more for it
 */

static void comp_drew(FAR struct nxbe_window_s *wnd)
{
  int i;

  for (i = 0; i < g_comp.nheld; i++)
    {
      if (g_comp.held[i] == wnd)
        {
          g_comp.held[i] = g_comp.held[--g_comp.nheld];
          if (g_comp.nheld == 0 &&
              (int32_t)(g_comp.holduntil - clock_systime_ticks() -
                        NXCOMP_REDRAWGRACE) > 0)
            {
              /* (a little longer: the rest of what it draws) */

              g_comp.holduntil = clock_systime_ticks() + NXCOMP_REDRAWGRACE;
            }

          return;
        }
    }
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
          wnd->parked || NXBE_ISLAYER(wnd))
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

  /* Cells not needed read back while all is still */

  if (quiet && g_nxaccel->cells_alloc != NULL)
    {
      comp_swapidle(&next);
    }

  if (g_comp.changed)
    {
      comp_snapshot();
    }

  return next < 0 ? -1 : MAX(next, (int32_t)NXCOMP_IDLEPOLL);
}

/****************************************************************************
 * Name: nxbe_compose_windows
 *
 * Description:
 *   The RAM-backed windows as text, a line each (/proc/gpumap): size,
 *   where on the display, where its pixels are (pieces of the surface,
 *   cells, memory, parked), hidden, seen, in perspective.  Not locked:
 *   for looking at, not for programs.
 *
 ****************************************************************************/

size_t nxbe_compose_windows(FAR char *buf, size_t size)
{
  FAR struct nxbe_window_s *wnd;
  size_t n = 0;
  int r;

  for (wnd = nxbe_surface_list(); wnd != NULL && n < size; wnd = wnd->surfnext)
    {
      FAR const struct nxbe_piece_s *p;
      FAR struct nxbe_cells_s *c = wnd->cells;
      int np = wnd->surface ? nxbe_surface_pieces(wnd, &p) : 0;
      int need = 0;
      int k;

      if (wnd->surface && c != NULL && c->cw > 0)
        {
          for (k = 0; k < c->cw * c->ch; k++)
            {
              need += (c->flags[k] & NXBE_CELL_NEED) != 0;
            }
        }

      r = snprintf(buf + n, size - n, "%4dx%-4d at %5d,%-5d %s%s%s%s%s %d"
                   " pieces, cells %d need %d resident %d swap %d lost %d"
                   " mip %02x\n",
                   wnd->bounds.pt2.x - wnd->bounds.pt1.x + 1,
                   wnd->bounds.pt2.y - wnd->bounds.pt1.y + 1,
                   wnd->bounds.pt1.x, wnd->bounds.pt1.y,
                   wnd->surface ? "surface" : wnd->parked ? "parked" :
                   wnd->fbmem != NULL ? "memory" : "none",
                   NXBE_ISHIDDEN(wnd) ? " hidden" : "",
                   g_comp.on && comp_seen(wnd) ? " seen" : "",
                   wnd_quad(wnd) ? " quad" : "",
                   wnd->surface && wnd->surfkind == NX_SURFACE_IMAGE ?
                   " image" : "", np,
                   c != NULL && c->cw > 0 ? c->cw * c->ch : 0, need,
                   c != NULL && c->cw > 0 ? c->nres : 0,
                   c != NULL ? c->nswap : 0, c != NULL ? c->nlost : 0,
                   wnd->mip != NULL ? wnd->mip->level : 0);
      n += r > 0 ? (size_t)r : 0;
    }

  return n < size ? n : size;
}

/****************************************************************************
 * Name: nxbe_compose_race
 *
 * Description:
 *   Single buffered frames behind the beam in bands of so many lines (0:
 *   not; < 0: NXCOMP_RACEBAND).  deskctl race.
 *
 ****************************************************************************/

void nxbe_compose_race(int lines)
{
  g_comp.race = lines < 0 ? NXCOMP_RACEBAND : MIN(lines, 1024);
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
  st->srcread   = g_nxbe_counts.srcread;
  st->srccached = g_nxbe_counts.srccached;
  st->tosurface = g_nxbe_counts.tosurface;
  st->swapouts  = g_nxbe_swap.outs;
  st->swapins   = g_nxbe_swap.ins;
  st->swaplost  = g_nxbe_swap.lost;
  st->redrawn   = g_nxbe_swap.redrawn;
  st->swapoutus = g_nxbe_swap.outus;
  st->swapinus  = g_nxbe_swap.inus;
  st->swapuniform = g_nxbe_swap.uniform;
  st->swapcleaned = g_nxbe_swap.cleaned;
  st->holds       = g_nxbe_counts.holds;
  st->prepms      = (uint32_t)(g_nxbe_counts.prepus / 1000);
  st->drawms      = (uint32_t)(g_nxbe_counts.drawus / 1000);
  st->holdms      = TICK2MSEC(g_nxbe_counts.holdticks);
  st->raced       = g_nxbe_counts.raced;
  st->bands       = g_nxbe_counts.bands;
  st->race        = g_comp.race;
  st->dbuf        = g_comp.dbuf;          /* (now, not as last snapped) */
  nxbe_swap_usage(&st->swapcells, &st->swapbytes);
}

#endif /* CONFIG_NX_COMPOSITOR */
