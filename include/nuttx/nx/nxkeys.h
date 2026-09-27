/****************************************************************************
 * include/nuttx/nx/nxkeys.h
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

#ifndef __INCLUDE_NUTTX_NX_NXKEYS_H
#define __INCLUDE_NUTTX_NX_NXKEYS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/input/kbd_codec.h>
#include <nuttx/nx/nx.h>

#ifdef CONFIG_NX_KBD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Key events in the keyboard stream.
 *
 * nx_kbdin() carries bytes: what a terminal expects (characters, VT100
 * sequences for the special keys).  A keyboard driver that knows more (a
 * key's press and release, the modifiers) may send key events in the same
 * stream instead, each one a record of NX_KEY_SIZE bytes:
 *
 *   NX_KEY_PREFIX, flags (NX_KEY_*), code (which key), ch (what it types)
 *
 * The byte NX_KEY_PREFIX (0xff: never in ASCII or UTF-8 text) starts a
 * record and appears nowhere else in one, so a stream can be resynchronised
 * at any 0xff; plain bytes and records may be mixed.
 *
 * The server hands a window's client the events as they are only if the
 * client asked for them (nx_kbdmode(NX_KBDMODE_KEYS)).  Every other client
 * gets bytes, as before: for each press (and repeat) the character the key
 * types, or the VT100 sequence of a special key (nx_key_bytes()); releases
 * are dropped.  Clients that know nothing of key events see no change.
 */

#define NX_KEY_PREFIX         0xff
#define NX_KEY_SIZE           4

/* flags */

#define NX_KEY_RELEASE        0x01     /* Released (else pressed) */
#define NX_KEY_REPEAT         0x02     /* A press repeated while held */
#define NX_KEY_SHIFT          0x10     /* Modifiers held, after this event */
#define NX_KEY_CTRL           0x20
#define NX_KEY_ALT            0x40
#define NX_KEY_GUI            0x80
#define NX_KEY_MODS           0xf0

/* code: which key, whatever the modifiers and the keyboard's layout.  A
 * key that types a character is that character on a US keyboard without
 * Shift: 'a' (for a, A and Ctrl-A), '1' (1 and !), '-', ' '.  Backspace,
 * Tab, Enter and Escape are their ASCII controls.  The others are
 * NX_KEY_SPECIAL() of their KEYCODE_* (nuttx/input/kbd_codec.h): the
 * arrows, Home, Delete, F1-F24, and the modifier keys themselves
 * (KEYCODE_LSHIFT ... KEYCODE_RGUI) when the keyboard reports them.
 */

#define NX_KEY_BACKSPACE      0x08
#define NX_KEY_TAB            0x09
#define NX_KEY_ENTER          0x0d
#define NX_KEY_ESCAPE         0x1b
#define NX_KEY_SPECIAL(k)     (0x80 + (k))
#define NX_KEY_ISSPECIAL(c)   ((c) >= 0x80)
#define NX_KEY_KEYCODE(c)     ((c) - 0x80)

/* ch: the character the event types with its modifiers ('A' for Shift-a,
 * 0x01 for Ctrl-a, '\n' for Enter, 0x7f or 0x08 for Backspace, as the
 * keyboard gives it), or 0 for a key that types none.
 */

/* nx_kbdmode() */

#define NX_KBDMODE_BYTES      0        /* Bytes only (the default) */
#define NX_KBDMODE_KEYS       1        /* Key events as they come */

/* nx_key_next() */

#define NX_KEYITEM_NONE       0        /* Nothing (left) */
#define NX_KEYITEM_BYTE       1        /* A plain byte */
#define NX_KEYITEM_KEY        2        /* A key event */

/* The most bytes nx_key_bytes() makes of one event */

#define NX_KEY_MAXBYTES       8

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nx_key_s
{
  uint8_t flags;                       /* NX_KEY_* */
  uint8_t code;                        /* Which key */
  uint8_t ch;                          /* What it types, or 0 */
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
 * Name: nx_kbdmode
 *
 * Description:
 *   What this connection's windows get from the keyboard:
 *   NX_KBDMODE_BYTES (the default) or NX_KBDMODE_KEYS (key events and
 *   bytes, as the keyboard driver sends them).
 *
 ****************************************************************************/

int nx_kbdmode(NXHANDLE handle, int mode);

/****************************************************************************
 * Name: nx_key_encode
 *
 * Description:
 *   The record of a key event, NX_KEY_SIZE bytes into buf.
 *
 ****************************************************************************/

void nx_key_encode(FAR uint8_t *buf, FAR const struct nx_key_s *key);

/****************************************************************************
 * Name: nx_key_next
 *
 * Description:
 *   Take the next item of a keyboard stream: a key event (into *key) or a
 *   plain byte (*byte).  *pch and *pnch are moved past it.  A record cut
 *   short at the end of the stream is dropped.
 *
 * Returned Value:
 *   NX_KEYITEM_KEY, NX_KEYITEM_BYTE, or NX_KEYITEM_NONE at the end.
 *
 ****************************************************************************/

int nx_key_next(FAR const uint8_t **pch, FAR int *pnch,
                FAR struct nx_key_s *key, FAR uint8_t *byte);

/****************************************************************************
 * Name: nx_key_bytes
 *
 * Description:
 *   What a terminal gets for a key event: the character a press (or
 *   repeat) types, else the VT100 sequence of the special key; nothing for
 *   a release.  At most NX_KEY_MAXBYTES bytes into buf.
 *
 * Returned Value:
 *   The number of bytes.
 *
 ****************************************************************************/

int nx_key_bytes(FAR const struct nx_key_s *key, FAR uint8_t *buf);

/****************************************************************************
 * Name: nx_key_split
 *
 * Description:
 *   How much of a keyboard stream of nch bytes to send in a message of at
 *   most max bytes without cutting a key event in two.
 *
 ****************************************************************************/

int nx_key_split(FAR const uint8_t *ch, int nch, int max);

/****************************************************************************
 * Name: nx_key_code
 *
 * Description:
 *   For keyboard drivers: which key (NX_KEY_* code) typed character ch on a
 *   US keyboard.  ctrl: Ctrl was held (0x01-0x1a are then Ctrl-a to
 *   Ctrl-z, not Backspace, Tab or Enter).
 *
 ****************************************************************************/

uint8_t nx_key_code(uint8_t ch, bool ctrl);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NX_KBD */
#endif /* __INCLUDE_NUTTX_NX_NXKEYS_H */
