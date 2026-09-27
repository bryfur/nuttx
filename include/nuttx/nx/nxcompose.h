/****************************************************************************
 * include/nuttx/nx/nxcompose.h
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

#ifndef __INCLUDE_NUTTX_NX_NXCOMPOSE_H
#define __INCLUDE_NUTTX_NX_NXCOMPOSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxaccel.h>

#ifdef CONFIG_NX_COMPOSITOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The composited desktop (CONFIG_NX_COMPOSITOR): the server keeps every
 * window in a surface of the display hardware and builds the display from
 * them, bottom to top, with blending (nx_accel_s blend).  A window can
 * have effects (struct nx_effects_s); a window opened with
 * NXBE_WINDOW_LAYER has no pixels at all and is drawn from a list of
 * operations (nx_setdisplaylist()) each time the display is composited,
 * over whatever is below it: panels, docks and menus that are truly
 * translucent.  Clients can blend into their own windows (nx_gpuops()),
 * for anti-aliased text from a glyph atlas, round buttons, gradients.
 *
 * Shape tables (nx_setlut()): slots NX_LUT_CORNER and NX_LUT_SHADOW belong
 * to the server (the round corners of windows, of radius
 * NX_COMPOSITOR_RADIUS, and their shadows); clients may load the others.
 */

#define NX_LUT_CORNER          0       /* A disc of NX_COMPOSITOR_RADIUS */
#define NX_LUT_SHADOW          1       /* A soft shadow around it */
#define NX_LUT_CLIENT0         2       /* Free for clients: 2 to 6 */
#define NX_LUT_RING            7       /* A 1-pixel ring on the NX_LUT_CORNER disc */

#define NX_COMPOSITOR_RADIUS   10

/* Layers: windows stay in their layer when raised or lowered */

#define NX_LAYER_DESKTOP       0       /* Wallpaper, desktop icons */
#define NX_LAYER_NORMAL        1       /* Application windows (default) */
#define NX_LAYER_PANEL         2       /* Top bar, dock */
#define NX_LAYER_OVERLAY       3       /* Menus, popups, notifications */

/* nx_effects_s flags */

#define NX_FX_ROUND            0x01    /* Round corners (NX_COMPOSITOR_RADIUS) */
#define NX_FX_KEY              0x02    /* Pixels of colour key are clear */
#define NX_FX_DARKEDGE         0x04    /* The 1-pixel edge (border) is dark,
                                        * not light: for light windows */

/* nx_compositor() capabilities */

#define NX_CAP_COMPOSITOR      0x01    /* The display is composited */

/* nx_gpuop_s op */

#define NX_GPUOP_FILL          1       /* Fill with colour */
#define NX_GPUOP_BLEND         2       /* Blend (mode: NX_BLEND_*, nxaccel.h) */
#define NX_GPUOP_COPY          3       /* Copy from src (opaque) */

/* The most operations one nx_gpuops() takes, and a layer window's display
 * list (nx_setdisplaylist(): the server keeps a copy)
 */

#define NX_GPUOPS_MAX          256
#define NX_DLIST_MAX           1024

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nx_effects_s
{
  uint8_t  flags;                /* NX_FX_* */
  uint8_t  layer;                /* NX_LAYER_* */
  uint8_t  opacity;              /* 255: opaque */
  uint8_t  shadow;               /* Shadow strength, 0: none */
  int8_t   shadowdy;             /* Shadow offset down, pixels */
  uint8_t  backdrop;             /* Alpha of a tint drawn behind the window
                                  * (translucent panels), 0: none */
  uint8_t  border;               /* Alpha of a 1-pixel light edge (dark
                                  * with NX_FX_DARKEDGE), 0: none */
  nxgl_mxpixel_t tint;           /* ... its colour */
  nxgl_mxpixel_t key;            /* NX_FX_KEY: the clear colour */
};

/* One operation of the display hardware on a window.  Coordinates are the
 * window's own (0, 0 is its top-left); what falls outside the window is
 * clipped.  An image source (NX_GPUOP_COPY, and NX_GPUOP_BLEND with a
 * mode reading one) is src at (sx, sy): a window with a surface (NULL:
 * this window), typically a hidden one holding a glyph atlas or icons.
 * Shape parameters p[] (NX_BLEND_MOD_*, nxaccel.h) are in window
 * coordinates too.
 */

struct nx_gpuop_s
{
  uint8_t  op;                   /* NX_GPUOP_* */
  uint8_t  mode;                 /* NX_BLEND_* */
  uint8_t  alpha;                /* Global alpha (255: opaque) */
  uint8_t  shift;                /* LUT distance >> shift */
  uint8_t  slot;                 /* LUT: which table */
  nxgl_coord_t x;                /* Destination */
  nxgl_coord_t y;
  nxgl_coord_t w;
  nxgl_coord_t h;
  FAR void *src;                 /* NXWINDOW with a surface, or NULL */
  nxgl_coord_t sx;               /* Source position in it */
  nxgl_coord_t sy;
  nxgl_mxpixel_t colour;         /* FILL, colour and A8 sources */
  uint16_t key;                  /* NX_BLEND_KEY */
  int16_t  p[4];                 /* Shape: LUT inner x0, y0, x1, y1;
                                  * ramp origin, step, start */
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
 * Name: nx_compositor
 *
 * Description:
 *   NX_CAP_* of the server: 0 if the display is not composited (the
 *   hardware has no blending: the calls below then do nothing).
 *
 ****************************************************************************/

int nx_compositor(FAR void *handle);

/****************************************************************************
 * Name: nx_displaysize
 *
 * Description:
 *   The display's size now (it may change: display modes).
 *
 ****************************************************************************/

int nx_displaysize(FAR void *handle, FAR struct nxgl_size_s *size);

/****************************************************************************
 * Name: nx_getdisplaymode, nx_setdisplaymode
 *
 * Description:
 *   The display modes of the hardware (nx_accel_s getmode): the size of
 *   mode n (0, 1, ...), or of the current one for n < 0; returns the
 *   mode's number, -EINVAL past the last, -ENOSYS without modes.
 *
 *   Change the display mode (returns its number or a negated errno).  The
 *   server moves the surfaces out of the way of the new display, damages
 *   all of it, and tells its clients: every window's position callback
 *   gets the new display bounds, and one window of each client the event
 *   NXEVENT_DISPLAYMODE (arg: the mode).  Windows keep their places and
 *   sizes: it is for the clients to lay them out again.
 *
 ****************************************************************************/

int nx_getdisplaymode(FAR void *handle, int mode,
                      FAR struct nxgl_size_s *size);
int nx_setdisplaymode(FAR void *handle, int mode);

/****************************************************************************
 * Name: nx_seteffects
 *
 * Description:
 *   Set a window's effects and layer.
 *
 ****************************************************************************/

int nx_seteffects(FAR void *hwnd, FAR const struct nx_effects_s *fx);

/****************************************************************************
 * Name: nx_setdisplaylist
 *
 * Description:
 *   Give a layer window (NXBE_WINDOW_LAYER) the operations it is drawn
 *   with, replacing the ones it had (the server copies them).
 *
 ****************************************************************************/

int nx_setdisplaylist(FAR void *hwnd, FAR const struct nx_gpuop_s *ops,
                      int nops);

/****************************************************************************
 * Name: nx_gpuops
 *
 * Description:
 *   Draw into a window's pixels with the display hardware.  Returns once
 *   the server has taken the operations.
 *
 ****************************************************************************/

int nx_gpuops(FAR void *hwnd, FAR const struct nx_gpuop_s *ops, int nops);

/****************************************************************************
 * Name: nx_setlut
 *
 * Description:
 *   Load shape table slot (NX_LUT_CLIENT0 to NX_LUT_RING - 1) with
 *   NX_BLEND_LUT_N x NX_BLEND_LUT_N alpha values, [ddy][ddx].
 *
 ****************************************************************************/

int nx_setlut(FAR void *handle, int slot,
              FAR const uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N]);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NX_COMPOSITOR */
#endif /* __INCLUDE_NUTTX_NX_NXCOMPOSE_H */
