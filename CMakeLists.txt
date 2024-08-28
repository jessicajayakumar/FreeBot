cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(freebot_demo)

include_directories(lib/freebot/inc)

# FreeBot API sources
if(DEFINED FB_DUMMY_API)
    target_sources(app PRIVATE lib/freebot/src/fb_dummy.c)
else()
    target_sources(app PRIVATE lib/freebot/src/fb_motor.c lib/freebot/src/fb_io.c lib/freebot/src/fb_pwr.c)
endif()

# SCT sources
include_directories(lib/sct/inc)
# target_sources(app PRIVATE lib/sct/src/generic_mic.c)
target_sources(app PRIVATE lib/sct/src/sct_work_and_charge.c)

# Main application source
target_sources(app PRIVATE src/main.c)



















# #
# # Copyright (c) 2018 Nordic Semiconductor
# #
# # SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
# #
# cmake_minimum_required(VERSION 3.20.0)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(Freebot_sender)


# include_directories(lib/freebot/inc/)

# # FreeBot API sources
# if(DEFINED FB_DUMMY_API)
#     target_include_directories(app PRIVATE lib/freebot/inc)
#     target_sources(app PRIVATE lib/freebot/src/fb_dummy.c)
# else()
#     target_include_directories(app PRIVATE lib/freebot/inc)
#     target_sources(app PRIVATE lib/freebot/src/fb_motor.c lib/freebot/src/fb_io.c lib/freebot/src/fb_pwr.c)
# endif()

# # NORDIC SDK APP START
# target_include_directories(app PRIVATE lib/freebot/inc)
# target_sources(app PRIVATE src/main.c)

# # NORDIC SDK APP END

# zephyr_library_include_directories(.)