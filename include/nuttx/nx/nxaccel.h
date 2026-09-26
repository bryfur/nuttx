/****************************************************************************
 * include/nuttx/nx/nxaccel.h
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

#ifndef __INCLUDE_NUTTX_NX_NXACCEL_H
#define __INCLUDE_NUTTX_NX_NXACCEL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/nx/nxglib.h>

#ifdef CONFIG_NX_ACCEL

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* nx_blend_s mode: the source (bits 1:0), the shape (bits 3:2), a colour
 * key and dithering; slot: the shape table.  For the destination pixel D at
 * (x, y) and the source pixel s:
 *
 *   a   = alpha x f(source) x m(shape)            (each 0..255)
 *   out = S * a + D * (1 - a), per channel
 *
 * LUT shape: m = lut[slot][min(ddy >> shift, 31)][min(ddx >> shift, 31)],
 * ddx the distance left of p[0] or right of p[2] (else 0), ddy above p[1]
 * or below p[3]: a rectangle whose inner rectangle p[] is inset by r from
 * it, with a table of a disc's coverage, has round corners of radius r;
 * with a falloff, it is a soft shadow.  Ramps: m = clamp((p[2] + (x - p[0])
 * * p[1]) / 256, 0, 255) (x, or y).  Shape coordinates are the display's,
 * like dest.
 */

#define NX_BLEND_SRC_COLOUR   0x00     /* S = colour, f = 255 */
#define NX_BLEND_SRC_RGB      0x01     /* S = s, f = 255 */
#define NX_BLEND_SRC_ARGB4444 0x02     /* S = s's RGB, f = s's A */
#define NX_BLEND_SRC_A8       0x03     /* S = colour, f = s & 0xff */
#define NX_BLEND_SRC_MASK     0x03
#define NX_BLEND_MOD_NONE     0x00
#define NX_BLEND_MOD_LUT      0x04
#define NX_BLEND_MOD_HRAMP    0x08
#define NX_BLEND_MOD_VRAMP    0x0c
#define NX_BLEND_MOD_MASK     0x0c
#define NX_BLEND_KEY          0x40     /* s == key: f = 0 */
#define NX_BLEND_DITHER       0x80

#define NX_BLEND_LUT_SLOTS    8
#define NX_BLEND_LUT_N        32

struct nx_blend_s
{
  struct nxgl_rect_s dest;             /* Where (clipped, on the device) */
  struct nxgl_point_s src;             /* Image sources: dest.pt1's source */
  nxgl_mxpixel_t colour;
  uint8_t alpha;
  uint8_t mode;                        /* NX_BLEND_* */
  uint8_t shift;                       /* LUT distances >> shift */
  uint8_t slot;                        /* LUT: which table */
  uint16_t key;
  int16_t p[4];                        /* LUT: inner x0, y0, x1, y1;
                                        * ramp: origin, step, start (8.8) */
};

/* Display operations done by the display hardware (a GPU) instead of by
 * the NX server writing the framebuffer (CONFIG_NX_ACCEL).  With these the
 * server does not touch the framebuffer memory at all, and the update
 * callout (CONFIG_NX_UPDATE) is not used: the hardware already has the
 * pixels.  Rectangles are in device coordinates, already clipped.
 */

struct nx_accel_s
{
  /* Fill the rectangle with the color */

  CODE void (*fill)(FAR const struct nxgl_rect_s *rect,
                    nxgl_mxpixel_t color);

  /* Move the rectangle by offset; source and destination may overlap */

  CODE void (*move)(FAR const struct nxgl_rect_s *rect,
                    FAR const struct nxgl_point_s *offset);

  /* Copy an image from memory into the rectangle: the pixel for device
   * (x, y) is at src + (y - origin->y) * srcstride + (x - origin->x) *
   * bytes per pixel.
   */

  CODE void (*copy)(FAR const struct nxgl_rect_s *dest,
                    FAR const void *src,
                    FAR const struct nxgl_point_s *origin,
                    unsigned int srcstride);

  /* Read the rectangle back into memory; NULL if the hardware cannot (the
   * result is then zero).  RAM-backed windows are read from their own
   * memory and do not come here.
   */

  CODE void (*get)(FAR const struct nxgl_rect_s *rect, FAR void *dest,
                   unsigned int deststride);

  /* Send what is batched; called when the server has nothing more to do
   * for now.  May be NULL.
   */

  CODE void (*flush)(void);

  /* Off-screen surfaces, optional (NULL): memory of the display hardware,
   * outside the display, addressed like it (a rectangle at *pos in the same
   * coordinates as the operations above).  A RAM-backed window gets one
   * instead of memory when it can: its drawing becomes the operations
   * above aimed at its surface, and showing it a move from the surface to
   * the display.  surface_alloc() returns a negated errno value if there
   * is no room (the window then uses memory as usual).
   */

  CODE int  (*surface_alloc)(nxgl_coord_t w, nxgl_coord_t h,
                             FAR struct nxgl_point_s *pos);
  CODE void (*surface_free)(FAR const struct nxgl_point_s *pos,
                            nxgl_coord_t w, nxgl_coord_t h);

  /* Per-pixel blending, optional (NULL): a colour or an image blended into
   * a rectangle with an alpha made of a global value, the image's own
   * alpha and a shape (a lookup table of distances from an inner
   * rectangle, or a ramp), struct nx_blend_s.  With it and surfaces, the
   * server composites (CONFIG_NX_COMPOSITOR).
   */

  CODE void (*blend)(FAR const struct nx_blend_s *op);

  /* Load one of the shape tables (NX_BLEND_LUT_SLOTS of them, each
   * NX_BLEND_LUT_N x NX_BLEND_LUT_N alpha values indexed [ddy][ddx])
   */

  CODE void (*set_lut)(int slot,
                       FAR const uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N]);

  /* Display modes, optional (NULL).  getmode() gives the size of mode n
   * (0, 1, ...) and returns 0, or -EINVAL past the last; for n < 0 it
   * gives the current mode's size and returns its number.  setmode()
   * changes the mode from the next frame; what the display shows is
   * undefined until it is redrawn.  The display is the rectangle of the
   * current mode's size at (0, 0), which surface_alloc() never hands out
   * (at the new size once setmode() has returned: it may then overlap
   * surfaces, which the server moves).
   */

  CODE int  (*getmode)(int mode, FAR struct nxgl_size_s *size);
  CODE int  (*setmode)(int mode);

  /* Double buffering, optional (NULL): from the next frame, scan the
   * display out from the rectangle of its size at origin (the display at
   * (0, 0), or a surface; origin->x even).  Later operations may draw into
   * what was shown before: the hardware holds them until that frame has
   * started, so flip once a frame, at its end.
   */

  CODE void (*flip)(FAR const struct nxgl_point_s *origin);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: board_nx_accel
 *
 * Description:
 *   Provided by the board (CONFIG_NX_ACCEL): the accelerated operations
 *   for a display and color plane, or NULL to draw into the framebuffer as
 *   usual.  Called by the NX server when it configures the display.
 *
 ****************************************************************************/

FAR const struct nx_accel_s *board_nx_accel(int display, int plane);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NX_ACCEL */
#endif /* __INCLUDE_NUTTX_NX_NXACCEL_H */
