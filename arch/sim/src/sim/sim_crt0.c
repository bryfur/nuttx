/****************************************************************************
 * arch/sim/src/sim/sim_crt0.c
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

/* (FPC) The start-up code of a module loaded by the simulator, as
 * arch/arm/src/common/crt0.c is the board's (a flat build's half of it):
 * the simulator then runs programs built against its native kit the way
 * the board does (tools/mkkit.py), rather than having them compiled in.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdlib.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* (nuttx/arch.h's: that header needs more than this directory compiles
 * with, the host's headers beside NuttX's)
 */

typedef CODE void (*initializer_t)(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Linker defined symbols to .ctors and .dtors */

extern initializer_t _sctors[];
extern initializer_t _ectors[];
extern initializer_t _sdtors[];
extern initializer_t _edtors[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int main(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LIBC_ELF_ABI_STAMP
/* The module ABI of the firmware build this module comes from (or, with
 * ELF_STAMP_SDK, the native kit's crt0_sdk.o, the SDK version's): a
 * reference only a firmware of that ABI or version satisfies
 */

#  include <nuttx/compiler.h>
#  include <nuttx/elf_abi.h>

#  if defined(ELF_STAMP_SDK) && defined(FPC_SDK_SYM)
extern const char FPC_SDK_SYM;
static const char *const g_elf_abi used_data = &FPC_SDK_SYM;
#  else
extern const char ELF_ABI_SYM;
static const char *const g_elf_abi used_data = &ELF_ABI_SYM;
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_CXXINITIALIZE
static void exec_ctors(void)
{
  for (initializer_t *ctor = _sctors; ctor != _ectors; ctor++)
    {
      (*ctor)();
    }
}

static void exec_dtors(void)
{
  for (initializer_t *dtor = _sdtors; dtor != _edtors; dtor++)
    {
      (*dtor)();
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _start
 *
 * Description:
 *   The module's entry point: C++ constructors, main(), exit().
 *
 ****************************************************************************/

void _start(int argc, char *argv[])
{
  int ret;

#ifdef CONFIG_HAVE_CXXINITIALIZE
  /* Call C++ constructors; destructors when the task exits */

  exec_ctors();
#  if CONFIG_LIBC_MAX_EXITFUNS > 0
  atexit(exec_dtors);
#  endif
#endif

  ret = main(argc, argv);

#if defined(CONFIG_HAVE_CXXINITIALIZE) && CONFIG_LIBC_MAX_EXITFUNS <= 0
  exec_dtors();
#endif

  exit(ret);
}
