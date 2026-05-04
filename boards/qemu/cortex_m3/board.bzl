load("//soc/ti/lm3s6965:soc.bzl", "LM3S6965_SOC")
load("//prelude:cfg_rules.bzl", "constraint_value", "platform")

QEMU_CORTEX_M3 = struct(
    name                   = "qemu_cortex_m3",
    soc                    = LM3S6965_SOC,
    build_dir              = "build_qemu_cortex_m3",
    constraint_value_label = "//boards/qemu/cortex_m3:qemu_cortex_m3_cv",
    platform_label         = "//boards/qemu/cortex_m3:qemu_cortex_m3",
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
