load("//soc/ti/lm3s6965:soc.bzl", "LM3S6965_SOC")
load("//prelude:cfg_rules.bzl", "constraint_value", "platform")

QEMU_CORTEX_M3 = struct(
    name                   = "qemu_cortex_m3",
    soc                    = LM3S6965_SOC,
    build_dir              = "build_qemu_cortex_m3",
    constraint_value_label = "//boards/qemu/cortex_m3:qemu_cortex_m3_cv",
    platform_label         = "//boards/qemu/cortex_m3:qemu_cortex_m3",
    platform_libs          = LM3S6965_SOC.arch.platform_libs + LM3S6965_SOC.platform_libs + [
        "//drivers/console:console",
    ],
    extra_archives         = LM3S6965_SOC.arch.extra_archives,
    offsets_label          = LM3S6965_SOC.arch.offsets_label,
    driver_srcs = {
        "console":          ["uart_console.c"],
        "serial":           ["uart_stellaris.c"],
        "timer":            ["cortex_m_systick.c", "sys_clock_init.c"],
        "arch_common_srcs": ["sw_isr_common.c", "xip.c"],
        "libzephyr_extra":  ["lib/utils/last_section_id.c", "build_qemu_cortex_m3/zephyr/misc/generated/configs.c"],
    },
)

def declare_targets():
    constraint_value(
        name               = "qemu_cortex_m3_cv",
        constraint_setting = "//constraints:board",
        visibility         = ["PUBLIC"],
    )
    platform(
        name              = "qemu_cortex_m3",
        constraint_values = [":qemu_cortex_m3_cv"],
        visibility        = ["PUBLIC"],
    )
