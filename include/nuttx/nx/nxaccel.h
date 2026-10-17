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
 *
 * A BC1 source (NX_ACCEL_BC1) reads 4 x 4 blocks of 64 bits (DXT1: two
 * RGB565 colours, sixteen 2-bit indices) from the hardware's memory seen
 * as four times as tall: a surface at (X, Y) holding a row of blocks per
 * row of the surface (8 bytes, 4 pixels, a block) is the image at (X, 4Y);
 * src is the image pixel at dest.pt1 there.
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
#define NX_BLEND_EXT          0x10     /* Bits 1:0 select an extended source: */
#define NX_BLEND_SRC_BC1      0x10     /* BC1 blocks (below), f = 0 where
                                        * transparent */
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

/* A part of a larger image drawn in perspective (nx_accel_s quad) */

struct nx_quadpart_s
{
  nxgl_coord_t ox;                     /* This part's place in the image */
  nxgl_coord_t oy;
  nxgl_coord_t fw;                     /* The image's size */
  nxgl_coord_t fh;
  nxgl_coord_t bx;                     /* The display pixel every part is */
  nxgl_coord_t by;                     /* drawn relative to */
  int32_t fx[4];                       /* The whole image's corners (as */
  int32_t fy[4];                       /* quad()'s x, y): its own edges */
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
   * is no room (the window then uses memory as usual).  kind says what
   * it is for (NX_SURFACE_*: for the board's statistics only).
   */

  CODE int  (*surface_alloc)(nxgl_coord_t w, nxgl_coord_t h, int kind,
                             FAR struct nxgl_point_s *pos);
  CODE void (*surface_free)(FAR const struct nxgl_point_s *pos,
                            nxgl_coord_t w, nxgl_coord_t h);

  /* Cells, optional (NULL): the memory as a grid of squares of
   * 1 << cellshift pixels, which a window's surface may be made of, in as
   * many pieces as there must be (no room is ever lost to fragments).  A
   * cell is named by its place: (x | y << 8) for the cell at (x << shift,
   * y << shift); NX_CELL_NONE is none.  cells_alloc() gives a cell to
   * every entry of map (cw x ch, a row of the window's cells after
   * another) that is NX_CELL_NONE, the window's neighbours together in
   * memory where there is room for them so, and returns 0; or none and
   * -ENOMEM if there are not enough.  cells_free() gives n back;
   * cells_claim() takes those n back if they are all free (0, else
   * -EBUSY and none); cells_free_count(): how many are free.
   *
   * surface_alloc() with NX_SURFACE_DISPLACE in kind may put the surface
   * over cells that windows have: those are then the surface's, no longer
   * the windows' (not to be freed), and NX moves what they hold elsewhere
   * before it draws into the surface.  It fails only if the cells free
   * outside it would not take them.
   */

  CODE int  (*cells_alloc)(int cw, int ch, FAR uint16_t *map);
  CODE int  (*cells_claim)(int n, FAR const uint16_t *cells);
  CODE int  (*cells_free_count)(void);
  CODE void (*cells_free)(int n, FAR const uint16_t *cells);
  uint8_t cellshift;

  /* The space surfaces and the display share (0: not said): a second
   * display buffer is only tried where one of the mode's size can fit
   * beside or below the display at (0, 0)
   */

  nxgl_coord_t spacew;
  nxgl_coord_t spaceh;

  /* Read a rectangle straight into memory over a fast path, optional
   * (NULL): x and w even; buf receives a word before the pixels (two a
   * word, rows packed) and one after them.  0 or a negated errno value.
   */

  CODE int  (*readbulk)(FAR const struct nxgl_rect_s *rect,
                        FAR uint32_t *buf);

  /* A game, optional (NULL): game() returns 2 while it has taken the
   * display, 1 while it is starting (it wants the CPU's memory first), 0
   * if there is none, and in *seq how often that has changed.  Starting,
   * NX gives back and uses no memory for copies of windows' pixels (its
   * swap).  With the display taken, NX composites nothing and its windows
   * hold no memory at all: all of them are parked (no surfaces or cells,
   * no copies in the swap, no second display buffer), and their clients
   * draw them again when the display is given back.  NX calls
   * gameready(seq) once it has done so for the change seq (the game may
   * then have the memory).  notify() gives the driver the function that
   * wakes NX's server to look (from any task).
   */

  CODE int  (*game)(FAR uint32_t *seq);
  CODE void (*gameready)(uint32_t seq);
  CODE void (*notify)(CODE void (*wake)(void));

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

  /* Drawing behind the beam, optional (NULL): the operations after it wait
   * until the scanout has read line n of the display (its rows from the
   * scanout origin) in the frame being shown, and not yet line m (m > n;
   * m < 0: or the whole frame has been read, vertical blanking); a frame
   * read past m already waits for the next one's line n.  A single
   * buffered display redrawn in bands, each after its wait, does not
   * tear.  Returns 0, or -ENOSYS when the hardware cannot (then nothing
   * waits); n < 0 only asks.
   */

  CODE int  (*waitline)(int n, int m);

  /* An image in perspective, optional (NULL): the w x h rectangle at src
   * (a surface) drawn as any convex quadrilateral, its corners (top-left,
   * top-right, bottom-right, bottom-left) at (x[k], y[k]) in 1/16 pixels,
   * only inside clip; radius > 0: the image's corners cut round by
   * circles of that radius (image pixels), as a window's.  part (NULL:
   * none) says the image is a part of a larger one drawn in parts (a
   * window's surface in pieces): its corners are the ones cut, and all
   * parts are drawn relative to the same point, so that they meet
   * without a seam and look the same wherever they are drawn; with the
   * whole image's corners (fx, fy), a part's pixels on the image's border
   * and its texels are the ones the whole image drawn at once would have,
   * however it is cut into parts.  For NX_FX_QUAD (nxcompose.h)
   */

  CODE void (*quad)(FAR const struct nxgl_point_s *src, nxgl_coord_t w,
                    nxgl_coord_t h, FAR const int32_t *x,
                    FAR const int32_t *y, int radius,
                    FAR const struct nx_quadpart_s *part,
                    FAR const struct nxgl_rect_s *clip);

  /* Whether the hardware has lost what its memory held (every surface's
   * pixels, the display buffers) since it last said so, optional (NULL):
   * true once, then the server has everything drawn again (each window
   * with a surface asked to redraw, the display composited afresh)
   */

  CODE bool (*lost)(void);

  uint32_t flags;                      /* NX_ACCEL_* */
};

/* nx_accel_s flags */

#define NX_ACCEL_BC1          0x01     /* blend() has NX_BLEND_SRC_BC1 */

/* surface_alloc() kinds */

#define NX_SURFACE_WINDOW     0        /* A window's pixels */
#define NX_SURFACE_BUFFER     1        /* A display buffer */
#define NX_SURFACE_STAGE      2        /* Staging for windows in memory */
#define NX_SURFACE_IMAGE      3        /* A window of image data (BC1) */
#define NX_SURFACE_KINDMASK   0xff
#define NX_SURFACE_DISPLACE   0x100    /* Over windows' cells if it must be */

#define NX_CELL_NONE          0xffff   /* cells_alloc(): no cell */
#define NX_CELL_X(c)          ((c) & 0xff)
#define NX_CELL_Y(c)          ((c) >> 8)
#define NX_CELL(x, y)         ((uint16_t)((x) | (y) << 8))

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
