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
#include <fixedmath.h>

#include <nuttx/nx/nxaccel.h>

#include "nxbe.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The board's operations, once installed (one display) */

FAR const struct nx_accel_s *g_nxaccel;

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
