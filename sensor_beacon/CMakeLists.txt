cmake_minimum_required(VERSION 3.13.1)
# Copyright (c) 2019 PHYTEC Messtechnik GmbH
#
# SPDX-License-Identifier: Apache-2.0
#
set(BOARD reel_board)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(sensor_ble_beacon)

if(NOT ("${BOARD}" STREQUAL "reel_board"))
  message(FATAL_ERROR "${BOARD} was specified, but this sample only supports reel board")
endif()

target_sources(app PRIVATE src/main.c)
