load("//:zephyr_rules.bzl", "zephyr_library")
load("//boards:all.bzl", "BOARD_NAME_SELECT", "driver_src_select")

_LIBZEPHYR_SRCS = [
    "lib/heap/heap.c",
    "lib/heap/heap_constants.c",
    "lib/os/assert.c",
    "lib/os/boot_banner.c",
    "lib/os/cbprintf_complete.c",
    "lib/os/cbprintf_packaged.c",
    "lib/os/clock.c",
    "lib/os/printk.c",
    "lib/utils/bitarray.c",
    "lib/utils/bitmask.c",
    "lib/utils/dec.c",
    "lib/utils/getopt/getopt.c",
    "lib/utils/getopt/getopt_common.c",
    "lib/utils/hex.c",
    "lib/utils/rb.c",
    "lib/utils/ring_buffer.c",
    "lib/utils/set.c",
    "lib/utils/timeutil.c",
    "subsys/tracing/tracing_none.c",
]

zephyr_library(
    name = "libzephyr",
    board = BOARD_NAME_SELECT,
    srcs = _LIBZEPHYR_SRCS + driver_src_select("libzephyr_extra"),
    supervisor = True,
    visibility = ["PUBLIC"],
)
