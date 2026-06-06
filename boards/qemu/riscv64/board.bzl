load("//soc/qemu/virt_riscv:soc.bzl", "VIRT_RISCV64_SOC")
load("//prelude:cfg_rules.bzl", "constraint_value", "platform")

QEMU_RISCV64 = struct(
    name                   = "qemu_riscv64",
    soc                    = VIRT_RISCV64_SOC,
    build_dir              = "build_qemu_riscv64",
    constraint_value_label = "//boards/qemu/riscv64:qemu_riscv64_cv",
    platform_label         = "//boards/qemu/riscv64:qemu_riscv64",
    platform_libs          = VIRT_RISCV64_SOC.arch.platform_libs + VIRT_RISCV64_SOC.platform_libs + [
        "//drivers/console:console",
    ],
    extra_archives         = VIRT_RISCV64_SOC.arch.extra_archives,
    offsets_label          = VIRT_RISCV64_SOC.arch.offsets_label,
    driver_srcs = {
        "console":          ["uart_console.c"],
        "serial":           ["uart_ns16550.c"],
        "timer":            ["riscv_machine_timer.c", "sys_clock_init.c"],
        "arch_common_srcs": ["sw_isr_common.c", "multilevel_irq.c"],
        "libzephyr_extra":  ["lib/utils/last_section_id.c", "build_qemu_riscv64/zephyr/misc/generated/configs.c"],
    },
)

def declare_targets():
    constraint_value(
        name               = "qemu_riscv64_cv",
        constraint_setting = "//constraints:board",
        visibility         = ["PUBLIC"],
    )
    platform(
        name              = "qemu_riscv64",
        constraint_values = [":qemu_riscv64_cv"],
        visibility        = ["PUBLIC"],
    )
