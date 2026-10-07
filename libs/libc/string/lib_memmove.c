/****************************************************************************
 * libs/libc/string/lib_memmove.c
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
#include <sys/types.h>
#include <stdint.h>
#include <string.h>

#include "libc.h"

#if !defined(CONFIG_LIBC_ARCH_MEMMOVE) && defined(LIBC_BUILD_MEMMOVE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* A word from a pointer of any alignment (one LDR where the CPU allows
 * unaligned loads)
 */

static inline uint32_t memmove_ld32(FAR const unsigned char *p)
{
  uint32_t v;

  __builtin_memcpy(&v, p, sizeof(v));
  return v;
}

/* The destination after the source and overlapping it: backwards, a word
 * at a time once the destination is aligned (the source too where the CPU
 * cannot load unaligned words), four words read before they are written.
 * A function of its own, so that memmove() itself saves no registers on
 * its way to memcpy().
 */

static FAR void *memmove_back(FAR void *dest, FAR const void *src,
                              size_t count) noinline_function;

no_builtin("memmove")
static FAR void *memmove_back(FAR void *dest, FAR const void *src,
                              size_t count)
{
  FAR unsigned char *d = (FAR unsigned char *)dest + count;
  FAR const unsigned char *s = (FAR const unsigned char *)src + count;

  if (count >= 16)
    {
      while (((uintptr_t)d & 3) != 0)
        {
          *--d = *--s;
          count--;
        }

#ifndef __ARM_FEATURE_UNALIGNED
      if (((uintptr_t)s & 3) == 0)
#endif
        {
          while (count >= 16)
            {
              uint32_t w3;
              uint32_t w2;
              uint32_t w1;
              uint32_t w0;

              s  -= 16;
              d  -= 16;
              w3 = memmove_ld32(s + 12);
              w2 = memmove_ld32(s + 8);
              w1 = memmove_ld32(s + 4);
              w0 = memmove_ld32(s);
              ((FAR uint32_t *)d)[3] = w3;
              ((FAR uint32_t *)d)[2] = w2;
              ((FAR uint32_t *)d)[1] = w1;
              ((FAR uint32_t *)d)[0] = w0;
              count -= 16;
            }

          while (count >= 4)
            {
              s -= 4;
              d -= 4;
              *(FAR uint32_t *)d = memmove_ld32(s);
              count -= 4;
            }
        }
    }

  while (count-- > 0)
    {
      *--d = *--s;
    }

  return dest;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef memmove
no_builtin("memmove")
FAR void *memmove(FAR void *dest, FAR const void *src, size_t count)
{
  /* The destination before the source, or no overlap: forwards */

  if ((uintptr_t)dest - (uintptr_t)src >= count)
    {
      return memcpy(dest, src, count);
    }

  return memmove_back(dest, src, count);
}
#endif
