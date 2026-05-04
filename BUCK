load("//:zephyr_rules.bzl", "zephyr_library", "CV_X86", "CV_ARM")

_BOARD = select({CV_X86: "qemu_x86", CV_ARM: "qemu_cortex_m3"})

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

_EXTRA_SRCS_X86 = ["build_qemu_x86/zephyr/misc/generated/configs.c"]
_EXTRA_SRCS_ARM = [
    "lib/utils/last_section_id.c",
    "build_qemu_cortex_m3/zephyr/misc/generated/configs.c",
]

zephyr_library(
    name = "libzephyr",
    board = _BOARD,
    srcs = _LIBZEPHYR_SRCS + select({CV_X86: _EXTRA_SRCS_X86, CV_ARM: _EXTRA_SRCS_ARM}),
    supervisor = True,
    visibility = ["PUBLIC"],
)
