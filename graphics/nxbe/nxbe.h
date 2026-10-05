/****************************************************************************
 * graphics/nxbe/nxbe.h
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

#ifndef __GRAPHICS_NXBE_NXBE_H
#define __GRAPHICS_NXBE_NXBE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxcursor.h>
#include <nuttx/nx/nxbe.h>
#include <nuttx/nx/nxaccel.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These are the values for the clipping order provided to nx_clipper */

#define NX_CLIPORDER_TLRB    (0)   /* Top-left-right-bottom */
#define NX_CLIPORDER_TRLB    (1)   /* Top-right-left-bottom */
#define NX_CLIPORDER_BLRT    (2)   /* Bottom-left-right-top */
#define NX_CLIPORDER_BRLT    (3)   /* Bottom-right-left-top */
#define NX_CLIPORDER_DEFAULT NX_CLIPORDER_TLRB

/* Server flags and helper macros:
 *
 * NXBE_STATE_MODAL  - One window is in a focused, modal state
 */

#define NXBE_STATE_MODAL     (1 << 0) /* Bit 0: One window is in a focused,
                                       *        modal state */

/* Helpful flag macros */

#define NXBE_STATE_ISMODAL(nxbe) \
  (((nxbe)->flags & NXBE_STATE_MODAL) != 0)
#define NXBE_STATE_SETMODAL(nxbe) \
  do { (nxbe)->flags |= NXBE_STATE_MODAL; } while (0)
#define NXBE_STATE_CLRMODAL(nxbe) \
  do { (nxbe)->flags &= ~NXBE_STATE_MODAL; } while (0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Rasterization ************************************************************/

/* A vtable of raster operation function pointers.  The types of the
 * function points must match the device rasterizer types exported by nxglib.
 */

struct nxbe_dev_vtable_s
{
  CODE void (*setpixel)(FAR NX_PLANEINFOTYPE *pinfo,
                        FAR const struct nxgl_point_s *pos,
                        nxgl_mxpixel_t color);
  CODE void (*fillrectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             nxgl_mxpixel_t color);
  CODE void (*getrectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                            FAR const struct nxgl_rect_s *rect,
                           FAR void *dest, unsigned int deststride);
  CODE void (*filltrapezoid)(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             nxgl_mxpixel_t color);
  CODE void (*moverectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
  CODE void (*copyrectangle)(FAR NX_PLANEINFOTYPE *pinfo,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
};

#ifdef CONFIG_NX_RAMBACKED
/* A vtable of raster operation function pointers.  The types of the
 * function points must match the per-window framebuffer rasterizer types
 * exported by nxglib.
 */

struct nxbe_pwfb_vtable_s
{
  CODE void (*setpixel)(FAR struct nxbe_window_s *bwnd,
                        FAR const struct nxgl_point_s *pos,
                        nxgl_mxpixel_t color);
  CODE void (*fillrectangle)(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             nxgl_mxpixel_t color);
  CODE void (*getrectangle)(FAR struct nxbe_window_s *bwnd,
                            FAR const struct nxgl_rect_s *rect,
                           FAR void *dest, unsigned int deststride);
  CODE void (*filltrapezoid)(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_trapezoid_s *trap,
                             FAR const struct nxgl_rect_s *bounds,
                             nxgl_mxpixel_t color);
  CODE void (*moverectangle)(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *rect,
                             FAR struct nxgl_point_s *offset);
  CODE void (*copyrectangle)(FAR struct nxbe_window_s *bwnd,
                             FAR const struct nxgl_rect_s *dest,
                             FAR const void *src,
                             FAR const struct nxgl_point_s *origin,
                             unsigned int srcstride);
};
#endif

#ifdef CONFIG_NX_SWCURSOR
/* A vtable of raster operation function pointers.  The types of the
 * function points must match the cursor rasterizer types exported by
 * nxglib.
 */

struct nxbe_cursorops_s
{
  CODE void (*draw)(FAR struct nxbe_state_s *be,
                    FAR const struct nxgl_rect_s *bounds,
                    int planeno);
  CODE void (*erase)(FAR struct nxbe_state_s *be,
                     FAR const struct nxgl_rect_s *bounds,
                     int planeno);
  CODE void (*backup)(FAR struct nxbe_state_s *be,
                      FAR const struct nxgl_rect_s *bounds,
                      int planeno);
};
#endif

/* Encapsulates everything needed support window rasterization commands. */

struct nxbe_plane_s
{
  /* Raster device operations */

  struct nxbe_dev_vtable_s dev;

#ifdef CONFIG_NX_RAMBACKED
  /* Raster per-window framebuffer operations */

  struct nxbe_pwfb_vtable_s pwfb;
#endif

#ifdef CONFIG_NX_SWCURSOR
  /* Cursor device operations */

  struct nxbe_cursorops_s cursor;
#endif

  /* Framebuffer plane info describing destination video plane */

  NX_DRIVERTYPE *driver;
  NX_PLANEINFOTYPE pinfo;
};

/* Clipping *****************************************************************/

/* Clipping callback functions called nxbe_clipper for each visible and
 * obscured region of a rectangle within a window.
 */

struct nxbe_clipops_s
{
  CODE void (*visible)(FAR struct nxbe_clipops_s *cops,
                       FAR struct nxbe_plane_s *plane,
                       FAR const struct nxgl_rect_s *rect);

  CODE void (*obscured)(FAR struct nxbe_clipops_s *cops,
                        FAR struct nxbe_plane_s *plane,
                        FAR const struct nxgl_rect_s *rect);
};

/* Cursor *******************************************************************/

/* Cursor state structure */

#if defined(CONFIG_NX_SWCURSOR)
struct nxbe_cursor_s
{
  bool visible;                             /* True: the cursor is visible */
  struct nxgl_rect_s bounds;                /* Cursor image bounding box */
  nxgl_mxpixel_t color1[CONFIG_NX_NPLANES]; /* Color1 is main color of the cursor */
  nxgl_mxpixel_t color2[CONFIG_NX_NPLANES]; /* Color2 is color of any border */
  nxgl_mxpixel_t color3[CONFIG_NX_NPLANES]; /* Color3 is the blended color */
  size_t allocsize;                         /* Size of the background allocation */
  FAR const uint8_t *image;                 /* Cursor image at 2-bits/pixel */
  FAR nxgl_mxpixel_t *bkgd;                 /* Cursor background in device pixels */
};
#elif defined(CONFIG_NX_HWCURSOR)
struct nxbe_cursor_s
{
  bool visible;                    /* True: the cursor is visible */
  struct nxgl_point_s pos;         /* The current cursor position */
};
#endif

/* Back-end state ***********************************************************/

/* This structure describes the overall back-end window state */

struct nxbe_state_s
{
  uint8_t flags;                     /* NXBE_STATE_* flags */

#if defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)
  /* Cursor support */

  struct nxbe_cursor_s cursor;       /* Cursor support */
#endif

  /* The window list (with the background window always at the bottom) */

  FAR struct nxbe_window_s *topwnd;  /* The window at the top of the display */
  struct nxbe_window_s bkgd;         /* The background window is always at the bottom */

  /* At present, only a solid colored background is supported for refills.
   * The following provides the background color.  It would be nice to
   * support background bitmap images as well.
   */

  nxgl_mxpixel_t bgcolor[CONFIG_NX_NPLANES];

  /* vinfo describes the video controller and plane[n].pinfo describes color
   * plane 'n' supported by the video controller.  Most common color models
   * fit in one plane, but this array provides future support for hardware
   * with planar YUV types with 3 or 4 color planes.
   */

  struct fb_videoinfo_s vinfo;

  /* Rasterizing functions selected to match the BPP reported in pinfo[] */

  struct nxbe_plane_s plane[CONFIG_NX_NPLANES];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_colormap
 *
 * Description:
 *   Set the hardware color map to the palette expected by NX
 *
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
int nxbe_colormap(FAR NX_DRIVERTYPE *dev);
#endif

/****************************************************************************
 * Name: nxbe_notify_rectangle
 *
 * Description:
 *   When CONFIG_NX_UPDATE=y, then the graphics system will callout to
 *   inform some external module that the display has been updated.  This
 *   would be useful in a couple for cases.
 *
 *   - When a serial LCD is used, but a framebuffer is used to access the
 *     LCD.  In this case, the update callout can be used to refresh the
 *     affected region of the display.
 *
 *   - When VNC is enabled.  This is case, this callout is necessary to
 *     update the remote frame buffer to match the local framebuffer.
 *
 *   When this feature is enabled, some external logic must provide this
 *   interface.  This is the function that will handle the notification.  It
 *   receives the rectangular region that was updated on the provided plane.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_UPDATE
void nxbe_notify_rectangle(FAR NX_DRIVERTYPE *dev,
                           FAR const struct nxgl_rect_s *rect);
#endif

/****************************************************************************
 * Name: nxbe_accel_install, nxbe_accel_flush
 *
 * Description:
 *   CONFIG_NX_ACCEL (nxbe_accel.c): put the board's accelerated operations
 *   (board_nx_accel()) in place of the plane's framebuffer rasterizers;
 *   let the board send what it has batched.  g_nxaccel is non-NULL once
 *   they are in place.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_ACCEL
EXTERN FAR const struct nx_accel_s *g_nxaccel;

void nxbe_accel_install(FAR struct nxbe_plane_s *plane, int display,
                        int planeno);
void nxbe_accel_flush(void);

/* RAM-backed windows kept in surfaces of the display hardware instead of
 * memory (nx_accel_s surface_alloc): made, remade at a new size (false:
 * no room, the surface is gone; *lost: it is, but not what it had),
 * moved elsewhere (false: no room, gone), all moved out of a rectangle
 * (the display, grown), given up for memory, freed, and shown (dest in
 * window coordinates, clipped by the windows above).  When there is no
 * room for one, windows that are not seen are parked, the compositor's
 * second display buffer is given up, then the surfaces are packed
 * together, before a window goes to memory.  nxbe_surface_list(): all of
 * them, linked by surfnext.
 *
 * Parking (CONFIG_NX_COMPOSITOR): a window gives up its surface (or its
 * memory), its pixels dropped (park); seen again, it gets a surface back
 * and its client is asked to draw it (unpark).  Windows in memory move
 * into surfaces when there is room (frommemory).
 */

#ifdef CONFIG_NX_RAMBACKED
/* A window's surface is made of the hardware's cells (nx_accel_s
 * cells_alloc), wherever they were free: in pieces, each a rectangle of
 * the window (its own coordinates) whose pixels are together in memory,
 * r.pt1's at pos.  An image window's (BC1 blocks) is one piece, a
 * surface of its own (cw = 0).  nxbe_surface_pieces() gives them (made
 * again when the cells have changed); the operations below are done on
 * each piece a rectangle of the window (rel) meets.
 */

struct nxbe_piece_s
{
  struct nxgl_rect_s r;               /* Of the window */
  struct nxgl_point_s pos;            /* Where r.pt1 is */
};

struct nxbe_cells_s
{
  nxgl_coord_t w;                     /* The surface's size */
  nxgl_coord_t h;
  uint8_t cw;                         /* Its cells across and down (0: one */
  uint8_t ch;                         /* surface, at the window's surfpos) */
  bool dirty;                         /* pieces[] to be made again */
  uint16_t npieces;
  uint16_t maxpieces;
  uint16_t nres;                      /* Cells it has (resident) */
  uint16_t nswap;                     /* Cells with a copy in the swap */
  uint16_t nlost;                     /* Cells lost (CELL_LOST) */
  FAR struct nxbe_piece_s *pieces;
  FAR int16_t *swap;                  /* cw x ch: the swap's slot, or -1 */
  FAR uint8_t *flags;                 /* cw x ch: NXBE_CELL_* */
  uint16_t map[1];                    /* cw x ch: the cells, a row after
                                       * another (NX_CELL_NONE: none) */
};

/* Not all of a window's cells need be resident (CONFIG_NX_COMPOSITOR): a
 * cell of a window that is not seen there (covered) may give up its cell
 * when others need room, what it holds read back into the swap
 * (nxbe_swap.c), or, with no room there, dropped (lost: its client is
 * asked to draw it again when it is seen); seen again, it gets a cell
 * back, its pixels put back from the swap.  Drawing goes to the cells
 * that are resident; a cell drawn into that is not is lost.  The pieces
 * are the resident cells'.
 */

#define NXBE_CELL_SWAP  0x01          /* The swap's slot holds its pixels */
#define NXBE_CELL_LOST  0x02          /* Not resident, its pixels gone */
#define NXBE_CELL_NEED  0x04          /* Seen: must be resident */

#define NX_CELL_SKIP    0xfffe        /* (cells_alloc(): not to be placed,
                                       * an entry it takes as placed) */

int  nxbe_surface_pieces(FAR struct nxbe_window_s *wnd,
                         FAR const struct nxbe_piece_s **pieces);

/* The part x of rel (window coordinates) that piece p holds, and where
 * that is in the hardware's memory (abs); false if none
 */

static inline bool nxbe_piece_part(FAR const struct nxbe_piece_s *p,
                                   FAR const struct nxgl_rect_s *rel,
                                   FAR struct nxgl_rect_s *x,
                                   FAR struct nxgl_rect_s *abs)
{
  nxgl_rectintersect(x, rel, &p->r);
  if (x->pt1.x > x->pt2.x || x->pt1.y > x->pt2.y)
    {
      return false;
    }

  nxgl_rectoffset(abs, x, p->pos.x - p->r.pt1.x, p->pos.y - p->r.pt1.y);
  return true;
}
void nxbe_surface_fill(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *rel,
                       nxgl_mxpixel_t color);
void nxbe_surface_put(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rel,
                      FAR const void *src, unsigned int stride);
void nxbe_surface_get(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rel,
                      FAR void *dest, unsigned int stride);
void nxbe_surface_move(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *rel,
                       FAR const struct nxgl_point_s *offset);
void nxbe_surface_copyto(FAR struct nxbe_window_s *wnd,
                         FAR const struct nxgl_rect_s *rel,
                         FAR const struct nxgl_point_s *dest);
bool nxbe_surface_take(nxgl_coord_t w, nxgl_coord_t h, int kind,
                       FAR struct nxgl_point_s *pos);
void nxbe_surface_drawn(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *rel);
void nxbe_surface_lose(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *rel);
bool nxbe_surface_resident(FAR struct nxbe_window_s *wnd,
                           FAR const struct nxgl_rect_s *rel);
bool nxbe_surface_defrag(FAR struct nxbe_window_s *wnd);
#ifdef CONFIG_NX_COMPOSITOR
bool nxbe_surface_ensure(FAR struct nxbe_window_s *wnd);
int  nxbe_surface_evict(int want, bool io);
#endif

bool nxbe_surface_open(FAR struct nxbe_window_s *wnd);
bool nxbe_surface_resize(FAR struct nxbe_window_s *wnd, FAR bool *lost);
void nxbe_surface_evacuate(FAR const struct nxgl_rect_s *keep);
void nxbe_surface_tomemory(FAR struct nxbe_window_s *wnd);
void nxbe_surface_close(FAR struct nxbe_window_s *wnd);
void nxbe_surface_present(FAR struct nxbe_window_s *wnd,
                          FAR const struct nxgl_rect_s *dest);
FAR struct nxbe_window_s *nxbe_surface_list(void);
#ifdef CONFIG_NX_COMPOSITOR
bool nxbe_surface_park(FAR struct nxbe_window_s *wnd);
void nxbe_surface_unpark(FAR struct nxbe_window_s *wnd);
int  nxbe_surface_frommemory(void);
#endif
void nxbe_surface_lost(void);
#endif
#endif

/****************************************************************************
 * CONFIG_NX_COMPOSITOR (nxbe_compose.c): the display is composited from
 * the windows' surfaces.  Changes only record damage (display
 * coordinates; _wrect: window coordinates; _window: all the window
 * touches), composited by nxbe_compose_run() when the server is idle or
 * nxbe_compose_due(), and nxbe_compose_wait() says it may (a frame at a
 * time).  nxbe_compose_on() is false if the hardware cannot blend: NX then
 * draws as without the compositor.  A surface that does not fit may take
 * the compositor's second display buffer (nxbe_compose_reclaim()); freed
 * surfaces let it try for one again (nxbe_compose_freed()).  A window
 * that got no surface is drawn in memory, in software
 * (nxbe_compose_memops(), nxbe_compose_mem.c, with copies of the shape
 * tables loaded through nxbe_compose_setlut()).
 ****************************************************************************/

#ifdef CONFIG_NX_COMPOSITOR
void nxbe_compose_install(FAR struct nxbe_state_s *be);
bool nxbe_compose_on(void);
void nxbe_compose_damage(FAR const struct nxgl_rect_s *rect);
void nxbe_compose_damage_wrect(FAR struct nxbe_window_s *wnd,
                               FAR const struct nxgl_rect_s *rel);
void nxbe_compose_damage_window(FAR struct nxbe_window_s *wnd);
void nxbe_compose_setposition(FAR struct nxbe_window_s *wnd,
                              FAR const struct nxgl_point_s *pos);
bool nxbe_compose_restack(FAR struct nxbe_window_s *wnd, bool top);
void nxbe_compose_open(FAR struct nxbe_window_s *wnd);
void nxbe_compose_close(FAR struct nxbe_window_s *wnd);
void nxbe_compose_seteffects(FAR struct nxbe_window_s *wnd,
                             FAR const struct nx_effects_s *fx);
void nxbe_compose_setdlist(FAR struct nxbe_window_s *wnd,
                           FAR const struct nx_gpuop_s *ops, int nops);
void nxbe_compose_gpuops(FAR struct nxbe_window_s *wnd,
                         FAR const struct nx_gpuop_s *ops, int nops);
bool nxbe_compose_due(void);
int  nxbe_compose_wait(void);
void nxbe_compose_run(void);
bool nxbe_compose_reclaim(void);
void nxbe_compose_freed(void);
void nxbe_compose_setmode(bool begin);
void nxbe_compose_setlut(int slot,
                         FAR const uint8_t lut[NX_BLEND_LUT_N][NX_BLEND_LUT_N]);
void nxbe_compose_memops(FAR struct nxbe_window_s *wnd,
                         FAR const struct nx_gpuop_s *ops, int nops,
                         FAR struct nxgl_rect_s *dirty);

/* Parking: whether a window is seen (not hidden, on the display and not
 * all covered by opaque windows); the windows, surfaces or parking have
 * changed (for the statistics); a window's client has been asked to draw
 * all of it (the next frame waits a little for it); the server is idle:
 * park windows not seen for a while (returns how many ticks until it
 * wants to be called again, -1: not unless something changes).
 */

bool nxbe_compose_seen(FAR struct nxbe_window_s *wnd);
void nxbe_compose_changed(void);
void nxbe_compose_redrawing(FAR struct nxbe_window_s *wnd);
int  nxbe_compose_idle(void);

/* Windows in memory drawing from sources in surfaces (nxbe_compose_mem.c)
 * keep what they read back of them: drop what a source's drawing (window
 * coordinates) makes out of date; free all of it; free all copies if no
 * window is in memory any more
 */

void nxbe_srccache_drop(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *rel);
void nxbe_srccache_free(FAR struct nxbe_window_s *wnd);
void nxbe_srccache_trim(void);

/* The swap (nxbe_swap.c): slots of a cell each */

struct nxbe_swapstats_s
{
  uint32_t outs;                      /* Cells read back into it */
  uint32_t ins;                       /* ... put back from it */
  uint32_t lost;                      /* Cells lost (none to read into) */
  uint32_t redrawn;                   /* ... drawn again by their clients */
  uint32_t freed;                     /* Cells given up, a copy there */
  uint32_t refused;                   /* The heap said no */
  uint64_t outus;                     /* Time reading back */
  uint64_t inus;                      /* ... putting back */
};

EXTERN struct nxbe_swapstats_s g_nxbe_swap;

void nxbe_swap_init(int cellshift);
int  nxbe_swap_alloc(void);
void nxbe_swap_free(int slot);
FAR uint32_t *nxbe_swap_slot(int slot);
void nxbe_swap_usage(FAR uint32_t *slots, FAR uint32_t *bytes);

/* Which cells windows need (NXBE_CELL_NEED), made again if anything that
 * decides it has changed (or all says so)
 */

void nxbe_compose_needs(bool all);

/* Counters for nxbe_compose_stats() */

struct nxbe_counts_s
{
  uint32_t frames;
  uint32_t parks;
  uint32_t unparks;
  uint32_t srcread;
  uint32_t srccached;
  uint32_t tosurface;
};

EXTERN struct nxbe_counts_s g_nxbe_counts;
#endif

/****************************************************************************
 * Name: nx_configure
 *
 * Description:
 *   Configure the back end state structure based on information from the
 *   framebuffer or LCD driver
 *
 ****************************************************************************/

int nxbe_configure(FAR NX_DRIVERTYPE *dev, FAR struct nxbe_state_s *be);

#if defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)
/****************************************************************************
 * Name: nxbe_cursor_enable
 *
 * Description:
 *   Enable/disable presentation of the cursor
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *   enable - True: show the cursor, false: hide the cursor.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_enable(FAR struct nxbe_state_s *be, bool enable);

/****************************************************************************
 * Name: nxbe_cursor_setimage
 *
 * Description:
 *   Set the cursor image
 *
 *   The image is provided a a 2-bits-per-pixel image.  The two bit incoding
 *   is as following:
 *
 *   00 - The transparent background
 *   01 - Color1:  The main color of the cursor
 *   10 - Color2:  The color of any border
 *   11 - Color3:  A blend color for better imaging (fake anti-aliasing).
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *   image - Describes the cursor image in the expected format.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NX_HWCURSORIMAGE) || defined(CONFIG_NX_SWCURSOR)
void nxbe_cursor_setimage(FAR struct nxbe_state_s *be,
                          FAR struct nx_cursorimage_s *image);
#endif

/****************************************************************************
 * Name: nxbe_cursor_setposition
 *
 * Description:
 *   Move the cursor to the specified position
 *
 * Input Parameters:
 *   be  - The back-end state structure instance
 *   pos - The new cursor position
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_setposition(FAR struct nxbe_state_s *be,
                             FAR const struct nxgl_point_s *pos);

#endif /* CONFIG_NX_SWCURSOR || CONFIG_NX_HWCURSOR */

#ifdef CONFIG_NX_SWCURSOR
/****************************************************************************
 * Name: nxbe_cursor_backupdraw and nxbe_cursor_backupdraw_dev
 *
 * Description:
 *   Called after any modification to the display (in window coordinate
 *   frame) to perform the backup-draw operation on one color plane.
 *
 * Input Parameters:
 *   be    - The back-end state structure instance, or
 *   wnd   - Window state structure
 *   rect  - The modified region of the window.  In windows coordinates for
 *           nxbe_cursor_backupdraw(); in graphics device coordinates for
 *           nxbe_cursor_backupdraw_dev().
 *   plane - The plane number to use.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_backupdraw(FAR struct nxbe_window_s *wnd,
                            FAR const struct nxgl_rect_s *rect, int plane);
void nxbe_cursor_backupdraw_dev(FAR struct nxbe_state_s *be,
                                FAR const struct nxgl_rect_s *rect,
                                int plane);

/****************************************************************************
 * Name: nxbe_cursor_backupdraw_all and nxbe_cursor_backupdraw_devall
 *
 * Description:
 *   Called after any modification to the display to perform the backup-draw
 *   operation on all color planes.
 *
 * Input Parameters:
 *   be    - The back-end state structure instance, or
 *   wnd   - Window state structure
 *   rect  - The modified region of the window.  In windows coordinates for
 *           nxbe_cursor_backupdraw(); in graphics device coordinates for
 *           nxbe_cursor_backupdraw_dev().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_cursor_backupdraw_all(FAR struct nxbe_window_s *wnd,
                                FAR const struct nxgl_rect_s *rect);
void nxbe_cursor_backupdraw_devall(FAR struct nxbe_state_s *be,
                                   FAR const struct nxgl_rect_s *rect);
#endif /*  */

/****************************************************************************
 * Name: nxbe_closewindow
 *
 * Description:
 *   Close an existing window
 *
 * Input Parameters:
 *   wnd  - The window to be closed (and deallocated)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_closewindow(struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxbe_setposition
 *
 * Description:
 *   This function checks for intersections and redraws the display after
 *   a change in the position of a window.
 *
 ****************************************************************************/

void nxbe_setposition(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_point_s *pos);

/****************************************************************************
 * Name: nxbe_setsize
 *
 * Description:
 *   This function checks for intersections and redraws the display after
 *   a change in the size of a window.
 *
 ****************************************************************************/

void nxbe_setsize(FAR struct nxbe_window_s *wnd,
                  FAR const struct nxgl_size_s *size);

/****************************************************************************
 * Name: nxbe_raise
 *
 * Description:
 *   Bring the specified window to the top of the display.
 *
 ****************************************************************************/

void nxbe_raise(FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxbe_lower
 *
 * Description:
 *   Lower the specified window to the bottom of the display.
 *
 ****************************************************************************/

void nxbe_lower(FAR struct nxbe_window_s *wnd);

/****************************************************************************
 * Name: nxbe_modal
 *
 * Description:
 *   May be used to either (1) raise a window to the top of the display and
 *   select modal behavior, or (2) disable modal behavior.
 *
 ****************************************************************************/

void nxbe_modal(FAR struct nxbe_window_s *wnd, bool enable);

/****************************************************************************
 * Name: nxbe_setvisibility
 *
 * Description:
 *   Select if the window is visible or hidden.  A hidden window is still
 *   present will will update normally, but will be on the visible on the
 *   display until it is unhidden.
 *
 * Input Parameters:
 *   wnd  - The window to be modified
 *   hide - True: Window will be hidden; false: Window will be visible
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_setvisibility(FAR struct nxbe_window_s *wnd, bool hide);

/****************************************************************************
 * Name: nxbe_setpixel
 *
 * Description:
 *  Set a single pixel in the window to the specified color. This is simply a
 *  degenerate case of nxbe_fill, but may be optimized in some architectures.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   pos  - The pixel location to be set
 *   col  - The color to use in the set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_setpixel(FAR struct nxbe_window_s *wnd,
                   FAR const struct nxgl_point_s *pos,
                   nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxbe_fill
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be filled
 *   col  - The color to use in the fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_fill(FAR struct nxbe_window_s *wnd,
               FAR const struct nxgl_rect_s *rect,
               nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxbe_filltrapezoid
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   clip - Clipping region (may be null)
 *   rect - The location to be filled
 *   col  - The color to use in the fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_filltrapezoid(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *clip,
                        FAR const struct nxgl_trapezoid_s *trap,
                        nxgl_mxpixel_t color[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxbe_getrectangle
 *
 * Description:
 *  Get the raw contents of graphic memory within a rectangular region. NOTE:
 *  Since raw graphic memory is returned, the returned memory content may be
 *  the memory of windows above this one and may not necessarily belong to
 *  this window unless you assure that this is the top window.
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be copied
 *   plane - Specifies the color plane to get from.
 *   dest - The location to copy the memory region
 *   deststride - The width, in bytes, of the dest memory
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_getrectangle(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *rect,
                       unsigned int plane,
                       FAR uint8_t *dest, unsigned int deststride);

/****************************************************************************
 * Name: nxbe_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   wnd    - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move
 *   offset - The offset to move the region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_move(FAR struct nxbe_window_s *wnd,
               FAR const struct nxgl_rect_s *rect,
               FAR const struct nxgl_point_s *offset);

/****************************************************************************
 * Name: nxbe_bitmap_dev
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.  The graphics output is written to the graphics
 *   device unconditionally.
 *
 * Input Parameters:
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that will
 *            receive the bit map (window coordinate frame).
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in pixels.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_bitmap_dev(FAR struct nxbe_window_s *wnd,
                     FAR const struct nxgl_rect_s *dest,
                     FAR const void *src[CONFIG_NX_NPLANES],
                     FAR const struct nxgl_point_s *origin,
                     unsigned int stride);

/****************************************************************************
 * Name: nxbe_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.  This is a front end to nxbe_bitmap_dev() that is
 *   used only if CONFIG_NX_RAMBACKED=y.  If the per-window frame buffer is
 *   selected, then the bit map will be written to both the graphics device
 *   and shadowed in the per-window framebuffer.
 *
 * Input Parameters:
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region on the display that will
 *            receive the bit map (window coordinate frame).
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_bitmap(FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *dest,
                 FAR const void *src[CONFIG_NX_NPLANES],
                 FAR const struct nxgl_point_s *origin,
                 unsigned int stride);

/****************************************************************************
 * Name: nxbe_flush
 *
 * Description:
 *   After per-window framebuffer has been updated, the modified region must
 *   be written to device graphics memory.  That function is managed by this
 *   simple function.  It does the following:
 *
 *   1) It calls nxbe_bitmap_dev() to copy the modified per-window
 *      framebuffer into device graphics memory.
 *   2) If CONFIG_NX_SWCURSOR is enabled, it calls the cursor "draw"
 *      renderer to update re-draw the currsor image if any portion of
 *      graphics display update overwrote the cursor.  Since these
 *      operations are performed back-to-back, any resulting flicker
 *      should be minimized.
 *
 * Input Parameters (same as for nxbe_bitmap_dev):
 *   wnd    - The window that will receive the bitmap image
 *   dest   - Describes the rectangular region in the window that will
 *            receive the bit map (window coordinate frame).
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however,
 *            origin may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NX_RAMBACKED
void nxbe_flush(FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *dest,
                 FAR const void *src[CONFIG_NX_NPLANES],
                 FAR const struct nxgl_point_s *origin,
                 unsigned int stride);
#endif

/****************************************************************************
 * Name: nxbe_redraw
 *
 * Description:
 *   Re-draw the visible portions of the rectangular region for the
 *   specified window
 *
 ****************************************************************************/

void nxbe_redraw(FAR struct nxbe_state_s *be,
                 FAR struct nxbe_window_s *wnd,
                 FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxbe_redrawbelow
 *
 * Description:
 *   Re-draw the visible portions of the rectangular region for all windows
 *   below (and including) the specified window.  This function is called
 *   whenever a window is closed, moved, lowered or re-sized in order to
 *   expose newly visible portions of lower windows.
 *
 ****************************************************************************/

void nxbe_redrawbelow(FAR struct nxbe_state_s *be,
                      FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxbe_isvisible
 *
 * Description:
 *   Return true if the point, pt, in window wnd is visible.  pt is in
 *   absolute screen coordinates
 *
 ****************************************************************************/

bool nxbe_isvisible(FAR struct nxbe_window_s *wnd,
                    FAR const struct nxgl_point_s *pos);

/****************************************************************************
 * Name: nxbe_clipper
 *
 * Description:
 *   Perform flexible clipping operations.  Callbacks are executed for
 *   each obscured and visible portions of the window.
 *
 * Input Parameters:
 *   wnd  - The window to be clipped.
 *   rect   - The region of concern within the window
 *   order - Specifies the order to process the parts of the non-intersecting
 *           sub-rectangles.
 *   cops  - The callbacks to handle obscured and visible parts of the
 *           sub-rectangles.
 *   plane  - The raster operations to be used by the callback functions.
 *           These may vary with different color formats.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_clipper(FAR struct nxbe_window_s *wnd,
                  FAR const struct nxgl_rect_s *dest, uint8_t order,
                  FAR struct nxbe_clipops_s *cops,
                  FAR struct nxbe_plane_s *plane);

/****************************************************************************
 * Name: nxbe_clipnull
 *
 * Description:
 *   The do-nothing clipping callback function
 *
 ****************************************************************************/

void nxbe_clipnull(FAR struct nxbe_clipops_s *cops,
                   FAR struct nxbe_plane_s *plane,
                   FAR const struct nxgl_rect_s *rect);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __GRAPHICS_NXBE_NXBE_H */
