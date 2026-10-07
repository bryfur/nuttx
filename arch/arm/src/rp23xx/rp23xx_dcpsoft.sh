#!/bin/sh
############################################################################
# arch/arm/src/rp23xx/rp23xx_dcpsoft.sh
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

# rp23xx_dcpsoft.sh "CC FLAGS" CROSSDEV OUT: libgcc's double-precision
# routines in an archive, their symbols renamed (__aeabi_dadd ->
# __rp23xx_soft_dadd, __eqdf2 -> __rp23xx_soft_gnu_eqdf2): what
# rp23xx_dcp.S falls back on where the coprocessor's result is not
# libgcc's (denormals, NaNs)

set -e
CC="$1"
X="$2"
out="$3"
case "$out" in /*) ;; *) out="$PWD/$out" ;; esac
gcc=$($CC --print-libgcc-file-name)
t=$(mktemp -d)
trap 'rm -rf "$t"' EXIT
cd "$t"
${X}ar x "$gcc" _arm_addsubdf3.o _arm_muldivdf3.o _arm_cmpdf2.o \
  _arm_fixdfsi.o _arm_fixunsdfsi.o _arm_truncdfsf2.o
${X}ld -r -o rp23xx_dcpsoft.o _arm_*.o
${X}nm rp23xx_dcpsoft.o | awk '
  { n = $NF; t = $(NF - 1) }
  n ~ /^__aeabi_/ { print n, "__rp23xx_soft_" substr(n, 9); next }
  t == "T" && n ~ /^__/ { print n, "__rp23xx_soft_gnu_" substr(n, 3) }
' | sort -u > map
${X}objcopy --redefine-syms=map rp23xx_dcpsoft.o
rm -f "$out"
${X}ar rcs "$out" rp23xx_dcpsoft.o
