# Copyright (c) 2022 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Add max30102.h to the set of global include paths.
if (CONFIG_MAX30102)
   zephyr_include_directories(.)
   zephyr_library()
   zephyr_library_sources( max30102.c 
                           max30102_trigger.c
                           algorithmRF.c
                           algorithm.c)
endif()