load("//soc/intel/atom:soc.bzl", "ATOM_SOC")
load("//prelude:cfg_rules.bzl", "constraint_value", "platform")

QEMU_X86 = struct(
    name                   = "qemu_x86",
    soc                    = ATOM_SOC,
    build_dir              = "build_qemu_x86",
    constraint_value_label = "//boards/qemu/x86:qemu_x86_cv",
    platform_label         = "//boards/qemu/x86:qemu_x86",
    platform_libs          = ATOM_SOC.arch.platform_libs + ATOM_SOC.platform_libs + [
        "//drivers/console:console",
    ],
    extra_archives         = ATOM_SOC.arch.extra_archives,
    offsets_label          = ATOM_SOC.arch.offsets_label,
    driver_srcs = {
        "console":         ["uart_console.c"],
        "serial":          ["uart_ns16550.c"],
        "timer":           ["hpet.c", "sys_clock_init.c"],
        "arch_common_srcs": [],
        "libzephyr_extra": ["build_qemu_x86/zephyr/misc/generated/configs.c"],
    },
)

def declare_targets():
    constraint_value(
        name               = "qemu_x86_cv",
        constraint_setting = "//constraints:board",
        visibility         = ["PUBLIC"],
    )
    platform(
        name             = "qemu_x86",
        constraint_values = [":qemu_x86_cv"],
        visibility       = ["PUBLIC"],
    )
