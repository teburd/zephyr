# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

#set(SHIELD x_nucleo_iks01a3)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mosyn)


FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
