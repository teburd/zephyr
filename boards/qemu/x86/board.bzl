load("//soc/intel/atom:soc.bzl", "ATOM_SOC")
load("//prelude:cfg_rules.bzl", "constraint_value", "platform")

QEMU_X86 = struct(
    name                   = "qemu_x86",
    soc                    = ATOM_SOC,
    build_dir              = "build_qemu_x86",
    constraint_value_label = "//boards/qemu/x86:qemu_x86_cv",
    platform_label         = "//boards/qemu/x86:qemu_x86",
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
