# SPDX-License-Identifier: Apache-2.0

# For the sake of demonstration, we add the driver directory as a zephyr module
# by hand. If your driver is a project that's managed by west, you can remove this line.
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/exDriver
  )

cmake_minimum_required(VERSION 3.20.0)
set (BOARD nrf5340dk_nrf5340_cpuapp)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(max30102DeviceDriver)

target_sources(app PRIVATE
  src/main.c
  )