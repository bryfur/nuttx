# ##############################################################################
# cmake/nuttx_crt0_sdk.cmake
#
# SPDX-License-Identifier: Apache-2.0
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

# (FPC) nuttx_crt0_sdk(SRC): crt0_sdk.o in the build's directory, the start-up
# code SRC compiled as a module's with ELF_STAMP_SDK: the native kit's
# (tools/mkkit.py), whose programs import the SDK version's symbol rather than
# this build's ABI id, and so run on any firmware of that version

function(nuttx_crt0_sdk src)
  nuttx_add_aux_library(STARTUP_SDK_OBJS ${src})
  target_compile_options(
    STARTUP_SDK_OBJS
    PRIVATE
      $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx_global,NUTTX_ELF_APP_COMPILE_OPTIONS>>
  )
  target_compile_definitions(STARTUP_SDK_OBJS PRIVATE ELF_STAMP_SDK)
  add_custom_command(
    OUTPUT ${NUTTX_BINARY_DIR}/crt0_sdk.o
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_OBJECTS:STARTUP_SDK_OBJS>
            ${NUTTX_BINARY_DIR}/crt0_sdk.o
    DEPENDS STARTUP_SDK_OBJS $<TARGET_OBJECTS:STARTUP_SDK_OBJS>
    COMMAND_EXPAND_LISTS)
  # (and the build's own, crt0.o: its tests against the tree, PROG_TREE)
  add_custom_command(
    OUTPUT ${NUTTX_BINARY_DIR}/crt0.o
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_OBJECTS:STARTUP_OBJS>
            ${NUTTX_BINARY_DIR}/crt0.o
    DEPENDS STARTUP_OBJS $<TARGET_OBJECTS:STARTUP_OBJS>
    COMMAND_EXPAND_LISTS)
  add_custom_target(crt0_sdk ALL DEPENDS ${NUTTX_BINARY_DIR}/crt0_sdk.o
                                         ${NUTTX_BINARY_DIR}/crt0.o)

  # and the flags it is compiled with, a module's (crt0_sdk_C.flags, one a
  # line, no include paths): the kit's compiler flags come from the build
  # (evaluated once a language: the options test it)
  set(opts "$<TARGET_PROPERTY:STARTUP_SDK_OBJS,COMPILE_OPTIONS>")
  set(defs "$<TARGET_PROPERTY:STARTUP_SDK_OBJS,COMPILE_DEFINITIONS>")
  separate_arguments(cflags UNIX_COMMAND "${CMAKE_C_FLAGS}")
  string(REPLACE ";" "\n" cflags "${cflags}")
  file(
    GENERATE
    OUTPUT ${NUTTX_BINARY_DIR}/crt0_sdk_$<COMPILE_LANGUAGE>.flags
    CONTENT
      "${cflags}\n$<JOIN:${opts},\n>\n$<$<BOOL:${defs}>:-D$<JOIN:${defs},\n-D>>\n"
    TARGET STARTUP_SDK_OBJS)
endfunction()
