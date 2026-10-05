/****************************************************************************
 * graphics/nxbe/nxbe_compose_mem.c
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

/* Windows in memory, composited (CONFIG_NX_COMPOSITOR).
 *
 * A RAM-backed window that found no room for a surface in the display
 * hardware keeps its pixels in memory.  The operations clients draw with
 * (nx_gpuops()) are then done here, in software, with the arithmetic of
 * the hardware's BLEND (nxaccel.h): a source image in a surface is read
 * back (nx_accel_s get), the shape tables are the copies kept here of what
 * was loaded (nxbe_compose_setlut()).  Slow, but the window looks as it
 * would in a surface (RGB565 only).  The compositor shows it from memory.
 *
 * Reading back is slow too (about 1 MB/s on the FPC), and text is a BLEND
 * from the glyph atlas for every character: what is read of a source is
 * kept (struct nxbe_srccache_s, bands of MEM_BAND rows of all of its
 * width, read when first needed) until the source is drawn into there.
 * The copies go when no window is in memory any more.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nx/nxaccel.h>
#include <nuttx/nx/nxcompose.h>

#include "nxbe.h"

#ifdef CONFIG_NX_COMPOSITOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MEM_LUTSIZE   (NX_BLEND_LUT_SLOTS * NX_BLEND_LUT_N * NX_BLEND_LUT_N)
#define MEM_MAXSRC    (64 * 1024)          /* Source pixels read at once */
#define MEM_BAND      16                   /* Rows of a source kept */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* What has been read back of a source window: its pixels (as big as the
 * window was when made), a band of MEM_BAND rows at a time
 */

struct nxbe_srccache_s
{
  nxgl_coord_t w;
  nxgl_coord_t h;
  FAR uint16_t *pix;
  uint8_t valid[1];                        /* One per band: read */
};

/* A source image: pix, stride pixels between rows; owned: to be freed;
 * clear: a pixel's f is 0 (BC1's transparent pixels), NULL if none is
 */

struct mem_img_s
{
  FAR const uint16_t *pix;
  int stride;
  FAR uint16_t *owned;
  FAR uint8_t *clear;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The shape tables as loaded ([slot][ddy][ddx]); NULL: no memory (every
 * entry is then taken as 255, as the hardware has them until loaded)
 */

static FAR uint8_t *g_luts;

static const uint8_t g_bayer[4][4] =
{
  { 0, 8, 2, 10 }, { 12, 4, 14, 6 }, { 3, 11, 1, 9 }, { 15, 7, 13, 5 }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t mul8(uint32_t p, uint32_t q)
{
  return (p * q + 255) >> 8;
}

static inline uint32_t dist(int32_t v, int32_t lo, int32_t hi, int shift)
{
  int32_t d = v < lo ? lo - v : v > hi ? v - hi : 0;

  d >>= shift;
  return d > NX_BLEND_LUT_N - 1 ? NX_BLEND_LUT_N - 1 : d;
}

static uint16_t blend_px(uint16_t s, uint16_t d, uint32_t a, uint32_t bias)
{
  uint32_t ap = a + (a >> 7);
  uint32_t r = ((s >> 11) * ap + (d >> 11) * (256 - ap) + bias) >> 8;
  uint32_t g = (((s >> 5) & 63) * ap + ((d >> 5) & 63) * (256 - ap) +
                bias) >> 8;
  uint32_t b = ((s & 31) * ap + (d & 31) * (256 - ap) + bias) >> 8;

  return (uint16_t)(r << 11 | g << 5 | b);
}

static inline FAR uint16_t *mem_pixel(FAR struct nxbe_window_s *wnd,
                                      int x, int y)
{
  return (FAR uint16_t *)((FAR uint8_t *)wnd->fbmem + y * wnd->stride) + x;
}

/* The copy of what was read back of a source window, as big as it is now
 * (NULL: no memory for it)
 */

static FAR struct nxbe_srccache_s *cache_get(FAR struct nxbe_window_s *src)
{
  FAR struct nxbe_srccache_s *c = src->srccache;
  nxgl_coord_t w = src->bounds.pt2.x - src->bounds.pt1.x + 1;
  nxgl_coord_t h = src->bounds.pt2.y - src->bounds.pt1.y + 1;
  int nbands = (h + MEM_BAND - 1) / MEM_BAND;

  if (c != NULL && (c->w != w || c->h != h))
    {
      nxbe_srccache_free(src);
      c = NULL;
    }

  if (c == NULL && w > 0 && h > 0)
    {
      c = kumm_zalloc(sizeof(*c) + nbands);
      if (c == NULL)
        {
          return NULL;
        }

      c->pix = kumm_malloc((size_t)w * h * 2);
      if (c->pix == NULL)
        {
          kumm_free(c);
          return NULL;
        }

      c->w = w;
      c->h = h;
      src->srccache = c;
    }

  return c;
}

/* Rows y0 .. y1 of the copy, read back where they are not yet; width w of
 * them is wanted (for the counters)
 */

static void cache_fill(FAR struct nxbe_window_s *src,
                       FAR struct nxbe_srccache_s *c, int y0, int y1, int w)
{
  struct nxgl_rect_s r;
  int b;

  for (b = y0 / MEM_BAND; b <= y1 / MEM_BAND; b++)
    {
      int top = b * MEM_BAND;
      int rows = MIN(MEM_BAND, c->h - top);

      if (c->valid[b])
        {
          g_nxbe_counts.srccached += (uint32_t)w *
                                     (MIN(y1, top + rows - 1) -
                                      MAX(y0, top) + 1);
          continue;
        }

      r.pt1.x = 0;
      r.pt1.y = top;
      r.pt2.x = c->w - 1;
      r.pt2.y = top + rows - 1;
      nxbe_surface_get(src, &r, c->pix + (size_t)top * c->w, c->w * 2);
      c->valid[b] = 1;
      g_nxbe_counts.srcread += (uint32_t)c->w * rows;
    }
}

/* A w x h source image at (sx, sy) of window src (its own coordinates):
 * from its surface (read back, or what was read of it before) or its
 * memory; outside it, zero.
 */

static bool mem_source(FAR struct nxbe_window_s *src, int sx, int sy,
                       int w, int h, FAR struct mem_img_s *img)
{
  FAR struct nxbe_srccache_s *c = NULL;
  FAR uint16_t *buf;
  struct nxgl_rect_s r;
  struct nxgl_rect_s b;
  int y;

  r.pt1.x = sx;
  r.pt1.y = sy;
  r.pt2.x = sx + w - 1;
  r.pt2.y = sy + h - 1;
  b.pt1.x = 0;
  b.pt1.y = 0;
  b.pt2.x = src->bounds.pt2.x - src->bounds.pt1.x;
  b.pt2.y = src->bounds.pt2.y - src->bounds.pt1.y;
  nxgl_rectintersect(&r, &r, &b);
  img->owned = NULL;

  if (src->surface && g_nxaccel->get != NULL &&
      r.pt1.x <= r.pt2.x && r.pt1.y <= r.pt2.y)
    {
      c = cache_get(src);
      if (c != NULL)
        {
          cache_fill(src, c, r.pt1.y, r.pt2.y, r.pt2.x - r.pt1.x + 1);
          if (r.pt1.x == sx && r.pt1.y == sy && r.pt2.x == sx + w - 1 &&
              r.pt2.y == sy + h - 1)
            {
              /* All of it inside: straight from the copy */

              img->pix    = c->pix + (size_t)sy * c->w + sx;
              img->stride = c->w;
              return true;
            }
        }
    }

  buf = kumm_zalloc((size_t)w * h * 2);
  if (buf == NULL)
    {
      return false;
    }

  img->owned  = buf;
  img->pix    = buf;
  img->stride = w;
  if (r.pt1.x > r.pt2.x || r.pt1.y > r.pt2.y)
    {
      return true;
    }

  if (c != NULL)
    {
      for (y = r.pt1.y; y <= r.pt2.y; y++)
        {
          memcpy(buf + (y - sy) * w + (r.pt1.x - sx),
                 c->pix + (size_t)y * c->w + r.pt1.x,
                 (r.pt2.x - r.pt1.x + 1) * 2);
        }
    }
  else if (src->surface)
    {
      FAR uint16_t *dest = buf + (r.pt1.y - sy) * w + (r.pt1.x - sx);

      if (g_nxaccel->get != NULL)
        {
          nxbe_surface_get(src, &r, dest, w * 2);
          g_nxbe_counts.srcread += (uint32_t)(r.pt2.x - r.pt1.x + 1) *
                                   (r.pt2.y - r.pt1.y + 1);
        }
    }
  else if (src->fbmem != NULL)
    {
      for (y = r.pt1.y; y <= r.pt2.y; y++)
        {
          memcpy(buf + (y - sy) * w + (r.pt1.x - sx),
                 mem_pixel(src, r.pt1.x, y), (r.pt2.x - r.pt1.x + 1) * 2);
        }
    }

  return true;
}

/* A w x h source image at (sx, sy) of a BC1 image window (pixels of the
 * image, four times as tall as the window): its blocks read as for any
 * source, decoded as the hardware does (docs/gpu.md "BC1 images")
 */

static uint16_t bc1_mix(uint16_t c0, uint16_t c1, int k0, int k1, int d)
{
  uint32_t r = (((c0 >> 11) * k0 + (c1 >> 11) * k1) + d / 2) / d;
  uint32_t g = ((((c0 >> 5) & 63) * k0 + ((c1 >> 5) & 63) * k1) + d / 2) /
               d;
  uint32_t b = (((c0 & 31) * k0 + (c1 & 31) * k1) + d / 2) / d;

  return (uint16_t)(r << 11 | g << 5 | b);
}

static bool mem_source_bc1(FAR struct nxbe_window_s *src, int sx, int sy,
                           int w, int h, FAR struct mem_img_s *img)
{
  struct mem_img_s blk;
  FAR uint16_t *buf;
  FAR uint8_t *clear;
  int bx0 = sx & ~3;
  int by0 = sy >> 2;
  int bw = ((sx + w + 3) & ~3) - bx0;
  int bh = ((sy + h + 3) >> 2) - by0;
  int x;
  int y;

  if (sx < 0 || sy < 0)
    {
      return false;
    }

  buf   = kumm_malloc((size_t)w * h * 3);
  if (buf == NULL)
    {
      return false;
    }

  clear = (FAR uint8_t *)(buf + (size_t)w * h);
  if (!mem_source(src, bx0, by0, bw, bh, &blk))
    {
      kumm_free(buf);
      return false;
    }

  for (y = 0; y < h; y++)
    {
      int v = sy + y;

      for (x = 0; x < w; x++)
        {
          int u = sx + x;
          FAR const uint16_t *b = blk.pix + (v / 4 - by0) * blk.stride +
                                  ((u & ~3) - bx0);
          uint16_t c0 = b[0];
          uint16_t c1 = b[1];
          uint32_t idx = b[2] | (uint32_t)b[3] << 16;
          int i = (v & 3) * 4 + (u & 3);
          uint16_t c;

          clear[y * w + x] = 0;
          switch ((idx >> (2 * i)) & 3)
            {
              case 0:
                c = c0;
                break;

              case 1:
                c = c1;
                break;

              case 2:
                c = c0 > c1 ? bc1_mix(c0, c1, 2, 1, 3) :
                              bc1_mix(c0, c1, 1, 1, 2);
                break;

              default:
                if (c0 > c1)
                  {
                    c = bc1_mix(c0, c1, 1, 2, 3);
                  }
                else
                  {
                    c = 0;
                    clear[y * w + x] = 1;
                  }
                break;
            }

          buf[y * w + x] = c;
        }
    }

  kumm_free(blk.owned);
  img->pix    = buf;
  img->stride = w;
  img->owned  = buf;
  img->clear  = clear;
  return true;
}

/* One operation, window coordinates, into the window's memory; the part
 * of the window it drew in *dirty (window coordinates)
 */

static void mem_gpuop(FAR struct nxbe_window_s *wnd,
                      FAR const struct nx_gpuop_s *op,
                      FAR struct nxgl_rect_s *dirty)
{
  FAR struct nxbe_window_s *src;
  struct mem_img_s img;
  struct nxgl_rect_s whole;
  struct nxgl_rect_s bounds;
  struct nxgl_rect_s d;
  uint8_t smode = op->mode & NX_BLEND_SRC_MASK;
  uint8_t mmode = op->mode & NX_BLEND_MOD_MASK;
  bool bc1 = op->op == NX_GPUOP_BLEND && (op->mode & NX_BLEND_EXT) != 0;
  bool image = op->op == NX_GPUOP_COPY || bc1 ||
               (op->op == NX_GPUOP_BLEND && smode != NX_BLEND_SRC_COLOUR);
  int w;
  int h;
  int x;
  int y;

  if (op->w <= 0 || op->h <= 0)
    {
      return;
    }

  whole.pt1.x  = op->x;
  whole.pt1.y  = op->y;
  whole.pt2.x  = op->x + op->w - 1;
  whole.pt2.y  = op->y + op->h - 1;
  bounds.pt1.x = 0;
  bounds.pt1.y = 0;
  bounds.pt2.x = wnd->bounds.pt2.x - wnd->bounds.pt1.x;
  bounds.pt2.y = wnd->bounds.pt2.y - wnd->bounds.pt1.y;
  nxgl_rectintersect(&d, &whole, &bounds);
  if (d.pt1.x > d.pt2.x || d.pt1.y > d.pt2.y)
    {
      return;
    }

  w = d.pt2.x - d.pt1.x + 1;
  h = d.pt2.y - d.pt1.y + 1;
  nxgl_rectunion(dirty, dirty, &d);

  if (op->op == NX_GPUOP_FILL)
    {
      for (y = d.pt1.y; y <= d.pt2.y; y++)
        {
          FAR uint16_t *p = mem_pixel(wnd, d.pt1.x, y);

          for (x = 0; x < w; x++)
            {
              p[x] = (uint16_t)op->colour;
            }
        }

      return;
    }

  /* All of the source is read before anything is written (the source may
   * be the window itself), in bands of rows if it is large
   */

  if (image)
    {
      int rows = MAX(1, MIN(h, MEM_MAXSRC / w));
      struct nx_gpuop_s band = *op;

      if (rows < h)
        {
          for (y = 0; y < h; y += rows)
            {
              band.x  = d.pt1.x;
              band.y  = d.pt1.y + y;
              band.w  = w;
              band.h  = MIN(rows, h - y);
              band.sx = op->sx + d.pt1.x - whole.pt1.x;
              band.sy = op->sy + d.pt1.y + y - whole.pt1.y;
              mem_gpuop(wnd, &band, dirty);
            }

          return;
        }

      src = op->src != NULL ? (FAR struct nxbe_window_s *)op->src : wnd;
      img.clear = NULL;
      if (bc1 ? (smode != 0 ||
                 !mem_source_bc1(src, op->sx + d.pt1.x - whole.pt1.x,
                                 op->sy + d.pt1.y - whole.pt1.y, w, h,
                                 &img)) :
                !mem_source(src, op->sx + d.pt1.x - whole.pt1.x,
                            op->sy + d.pt1.y - whole.pt1.y, w, h, &img))
        {
          return;
        }
    }
  else
    {
      img.pix   = NULL;
      img.owned = NULL;
      img.clear = NULL;
    }

  for (y = d.pt1.y; y <= d.pt2.y; y++)
    {
      FAR uint16_t *p = mem_pixel(wnd, d.pt1.x, y);
      FAR const uint16_t *s = img.pix != NULL ?
                              img.pix + (y - d.pt1.y) * img.stride : NULL;

      for (x = d.pt1.x; x <= d.pt2.x; x++, p++)
        {
          uint16_t sp = s != NULL ? s[x - d.pt1.x] : 0;
          uint16_t spix;
          uint32_t f;
          uint32_t m;
          uint32_t bias;
          int32_t v;

          if (op->op == NX_GPUOP_COPY)
            {
              *p = sp;
              continue;
            }

          switch (bc1 ? NX_BLEND_SRC_RGB : smode)
            {
              case NX_BLEND_SRC_COLOUR:
                spix = (uint16_t)op->colour;
                f    = 255;
                break;

              case NX_BLEND_SRC_RGB:
                spix = sp;
                f    = 255;
                break;

              case NX_BLEND_SRC_ARGB4444:
                {
                  uint32_t r = (sp >> 8) & 15;
                  uint32_t g = (sp >> 4) & 15;
                  uint32_t b = sp & 15;

                  spix = (uint16_t)((r << 1 | r >> 3) << 11 |
                                    (g << 2 | g >> 2) << 5 |
                                    (b << 1 | b >> 3));
                  f    = (sp >> 12) * 17u;
                }
                break;

              default:
                spix = (uint16_t)op->colour;
                f    = sp & 0xff;
                break;
            }

          if (((op->mode & NX_BLEND_KEY) != 0 && image && sp == op->key) ||
              (img.clear != NULL &&
               img.clear[(y - d.pt1.y) * img.stride + x - d.pt1.x]))
            {
              f = 0;
            }

          switch (mmode)
            {
              case NX_BLEND_MOD_LUT:
                m = g_luts == NULL ? 255 :
                    g_luts[((op->slot & 7) * NX_BLEND_LUT_N +
                            dist(y, op->p[1], op->p[3], op->shift & 3)) *
                           NX_BLEND_LUT_N +
                           dist(x, op->p[0], op->p[2], op->shift & 3)];
                break;

              case NX_BLEND_MOD_HRAMP:
              case NX_BLEND_MOD_VRAMP:
                v = (int32_t)(uint16_t)op->p[2] +
                    ((mmode == NX_BLEND_MOD_HRAMP ? x : y) - op->p[0]) *
                    op->p[1];
                v >>= 8;
                m = v < 0 ? 0 : v > 255 ? 255 : (uint32_t)v;
                break;

              default:
                m = 255;
                break;
            }

          bias = (op->mode & NX_BLEND_DITHER) != 0 ?
                 g_bayer[y & 3][x & 3] * 16u + 8 : 128;
          *p = blend_px(spix, *p, mul8(mul8(op->alpha, f), m), bias);
        }
    }

  kumm_free(img.owned);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_compose_setlut
 *
 * Description:
 *   Load a shape table into the hardware, and keep a copy for windows in
 *   memory.
 *
 ****************************************************************************/

void nxbe_compose_setlut(int slot,
                         FAR const uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N])
{
  if (g_luts == NULL)
    {
      g_luts = kumm_malloc(MEM_LUTSIZE);
      if (g_luts != NULL)
        {
          memset(g_luts, 255, MEM_LUTSIZE);
        }
    }

  if (g_luts != NULL && slot >= 0 && slot < NX_BLEND_LUT_SLOTS)
    {
      memcpy(g_luts + slot * NX_BLEND_LUT_N * NX_BLEND_LUT_N, lut,
             NX_BLEND_LUT_N * NX_BLEND_LUT_N);
    }

  g_nxaccel->set_lut(slot, lut);
}

/****************************************************************************
 * Name: nxbe_compose_memops
 *
 * Description:
 *   Operations into a window in memory (window coordinates); what they
 *   drew is returned in *dirty (window coordinates, empty if nothing).
 *
 ****************************************************************************/

void nxbe_compose_memops(FAR struct nxbe_window_s *wnd,
                         FAR const struct nx_gpuop_s *ops, int nops,
                         FAR struct nxgl_rect_s *dirty)
{
  int i;

  dirty->pt1.x = INT16_MAX;
  dirty->pt1.y = INT16_MAX;
  dirty->pt2.x = INT16_MIN;
  dirty->pt2.y = INT16_MIN;

  if (wnd->fbmem == NULL || wnd->be->plane[0].pinfo.bpp != 16)
    {
      return;
    }

  for (i = 0; i < nops; i++)
    {
      mem_gpuop(wnd, &ops[i], dirty);
    }
}

/****************************************************************************
 * Name: nxbe_srccache_drop, nxbe_srccache_free, nxbe_srccache_trim
 *
 * Description:
 *   A source window is drawn into at rel (window coordinates): the bands
 *   of the copy of it there are out of date.  Its copy goes.  All copies
 *   go if no window is in memory any more.
 *
 ****************************************************************************/

void nxbe_srccache_drop(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *rel)
{
  FAR struct nxbe_srccache_s *c = wnd->srccache;
  int b0;
  int b1;
  int b;

  if (c == NULL)
    {
      return;
    }

  b0 = MAX(rel->pt1.y, 0) / MEM_BAND;
  b1 = MIN(rel->pt2.y, c->h - 1) / MEM_BAND;
  for (b = b0; b <= b1; b++)
    {
      c->valid[b] = 0;
    }
}

void nxbe_srccache_free(FAR struct nxbe_window_s *wnd)
{
  if (wnd->srccache != NULL)
    {
      kumm_free(wnd->srccache->pix);
      kumm_free(wnd->srccache);
      wnd->srccache = NULL;
    }
}

void nxbe_srccache_trim(void)
{
  FAR struct nxbe_window_s *wnd;

  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
    {
      if (!wnd->surface && wnd->fbmem != NULL)
        {
          return;
        }
    }

  for (wnd = nxbe_surface_list(); wnd != NULL; wnd = wnd->surfnext)
    {
      nxbe_srccache_free(wnd);
    }
}

#endif /* CONFIG_NX_COMPOSITOR */
