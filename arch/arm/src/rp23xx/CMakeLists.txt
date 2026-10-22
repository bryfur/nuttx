# ##############################################################################
# arch/arm/src/rp23xx/CMakeLists.txt
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

list(
  APPEND
  SRCS
  rp23xx_idle.c
  rp23xx_irq.c
  rp23xx_uart.c
  rp23xx_serial.c
  rp23xx_start.c
  rp23xx_timerisr.c
  rp23xx_gpio.c
  rp23xx_pio.c
  rp23xx_clock.c
  rp23xx_xosc.c
  rp23xx_pll.c
  rp23xx_interp.c)

if(CONFIG_SMP)
  list(APPEND SRCS rp23xx_cpustart.c rp23xx_smpcall.c rp23xx_cpuidlestack.c)
endif()

if(CONFIG_LIBC_ATOMIC_HWSPINLOCK)
  list(APPEND SRCS rp23xx_hwspinlock.c)
endif()

if(CONFIG_LIBC_ATOMIC_CHIP)
  list(APPEND SRCS rp23xx_atomic.c)
endif()

if(CONFIG_ARCH_HAVE_MULTICPU)
  list(APPEND SRCS rp23xx_cpuindex.c)
endif()

if(CONFIG_RP23XX_DMAC)
  list(APPEND SRCS rp23xx_dmac.c)
endif()

if(CONFIG_RP23XX_SPI)
  list(APPEND SRCS rp23xx_spi.c)
endif()

if(CONFIG_RP23XX_SDIO)
  list(APPEND SRCS rp23xx_sdio.c)
endif()

if(CONFIG_RP23XX_PWM)
  list(APPEND SRCS rp23xx_pwm.c)
endif()

if(CONFIG_RP23XX_TIMER)
  list(APPEND SRCS rp23xx_timer.c)
endif()

if(CONFIG_RP23XX_SYSTIMER_TICKLESS)
  list(APPEND SRCS rp23xx_oneshot.c)
endif()

if(CONFIG_RP23XX_I2C)
  list(APPEND SRCS rp23xx_i2c.c)
endif()

if(CONFIG_RP23XX_RNG)
  list(APPEND SRCS rp23xx_rng.c)
endif()

if(CONFIG_RP23XX_I2C_SLAVE)
  list(APPEND SRCS rp23xx_i2c_slave.c)
endif()

if(CONFIG_RP23XX_I2S)
  list(APPEND SRCS rp23xx_i2s.c)
  list(APPEND SRCS rp23xx_i2s_pio.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS rp23xx_usbdev.c)
endif()

if(CONFIG_WS2812)
  list(APPEND SRCS rp23xx_ws2812.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS rp23xx_adc.c)
endif()

if(CONFIG_WATCHDOG)
  list(APPEND SRCS rp23xx_wdt.c)
endif()

if(CONFIG_CRYPTO_CRYPTODEV_HARDWARE)
  list(APPEND SRCS rp23xx_crypto.c)
endif()

if(CONFIG_RP23XX_SHA256 OR CONFIG_CRYPTO_CRYPTODEV_HARDWARE)
  list(APPEND SRCS rp23xx_sha256.c)
endif()

# (FPC) The double-precision coprocessor (as Make.defs): rp23xx_dcp.S in the
# firmware; librp23xx_dcpsoft.a, libgcc's double routines renamed for the
# cases the coprocessor does not do as libgcc (the firmware's: modules reach
# them through the symbol table); modrt.a, rp23xx_dcp.S for each loadable
# module, linked before libgcc (Config.mk's MODULE_RTSRC: the native kit's
# lib/modrt.a, tools/mkkit.py)

if(CONFIG_RP23XX_DCP)
  list(APPEND SRCS rp23xx_dcp.S)

  get_directory_property(dir_opts COMPILE_OPTIONS)
  set(cpu_flags)
  foreach(opt ${dir_opts})
    if(opt MATCHES "^-m")
      list(APPEND cpu_flags ${opt})
    endif()
  endforeach()
  string(JOIN " " cc_cpu ${CMAKE_C_COMPILER} ${cpu_flags})

  set(dcpsoft ${CMAKE_BINARY_DIR}/librp23xx_dcpsoft.a)
  add_custom_command(
    OUTPUT ${dcpsoft}
    COMMAND sh ${CMAKE_CURRENT_LIST_DIR}/rp23xx_dcpsoft.sh "${cc_cpu}"
            ${TOOLCHAIN_PREFIX}- ${dcpsoft}
    DEPENDS ${CMAKE_CURRENT_LIST_DIR}/rp23xx_dcpsoft.sh
    COMMENT "Generating librp23xx_dcpsoft.a")
  add_custom_target(rp23xx_dcpsoft DEPENDS ${dcpsoft})
  add_library(rp23xx_dcpsoft_lib STATIC IMPORTED GLOBAL)
  set_target_properties(rp23xx_dcpsoft_lib PROPERTIES IMPORTED_LOCATION
                                                      ${dcpsoft})
  set_property(GLOBAL APPEND PROPERTY NUTTX_EXTRA_LIBRARIES rp23xx_dcpsoft_lib)

  if(CONFIG_MODULES)
    add_library(modrt STATIC rp23xx_dcp.S)
    nuttx_add_library_internal(modrt)
    target_compile_definitions(modrt PRIVATE RP23XX_DCP_MODULE)
    target_compile_options(
      modrt
      PRIVATE
        $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx_global,NUTTX_ELF_APP_COMPILE_OPTIONS>>
    )
    # (built for the kit, though nothing of the firmware links it: its
    # directory is otherwise left out of the default build)
    set_target_properties(modrt PROPERTIES OUTPUT_NAME modrt PREFIX ""
                                           EXCLUDE_FROM_ALL FALSE)
  endif()
endif()

if(CONFIG_RP23XX_FLASH_MTD)
  list(APPEND SRCS rp23xx_flash_mtd.c)
endif()

if(CONFIG_RP23XX_PSRAM)
  list(APPEND SRCS rp23xx_psram.c rp23xx_heaps.c)
endif()

if(CONFIG_RP23XX_RTC)
  list(APPEND SRCS rp23xx_rtc.c)
endif()

if(CONFIG_RP23XX_OTP)
  list(APPEND SRCS rp23xx_otp.c)
endif()

if(CONFIG_IEEE80211_INFINEON_CYW43439)
  list(APPEND SRCS rp23xx_cyw43439.c)
endif()

target_sources(arch PRIVATE ${SRCS})
