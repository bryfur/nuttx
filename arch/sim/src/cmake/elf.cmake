# ##############################################################################
# arch/sim/src/cmake/elf.cmake
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

# Loadable and ELF module settings

nuttx_elf_compile_options(-fvisibility=hidden -fno-pic)

# (FPC) the host compiler's defaults NuttX does not have (Make.defs'
# CMODULEFLAGS: a module would call __stack_chk_fail)
nuttx_elf_compile_options(-fno-stack-protector)

# (FPC) amd64: modules load into NuttX's heap, out of a 32-bit relocation's
# reach of the firmware (-mcmodel=small); as CELFFLAGS in the board's Make.defs
# (there was CONFIG_LIBC_ARCH_ELF_64BIT here, which no Kconfig defines)
nuttx_elf_compile_options_ifdef(CONFIG_ARCH_64BIT -mcmodel=large)

# (FPC) With modules, a module starts at sim_crt0.c's _start (constructors,
# the ABI stamp, main, exit), as the board's do

if(CONFIG_MODULES)
  nuttx_elf_link_options(-r -e _start)
else()
  nuttx_elf_link_options(-r -e main)
endif()

nuttx_elf_link_options_ifdef(CONFIG_SIM_M32 -melf_i386)
