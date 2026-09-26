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
