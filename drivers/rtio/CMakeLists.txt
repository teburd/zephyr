# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_RTIO_SENSOR sensor)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_RTIO_CONTEXT rtio_context.c)
zephyr_library_sources_ifdef(CONFIG_RTIO_RAMP rtio_ramp.c)
