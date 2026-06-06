load("//soc/native/inf_clock:soc.bzl", "INF_CLOCK_SOC")
load("//prelude:cfg_rules.bzl", "constraint_value", "platform")

NATIVE_SIM = struct(
    name                   = "native_sim",
    soc                    = INF_CLOCK_SOC,
    build_dir              = "build_native_sim",
    constraint_value_label = "//boards/native/native_sim:native_sim_cv",
    platform_label         = "//boards/native/native_sim:native_sim",
    platform_libs          = INF_CLOCK_SOC.arch.platform_libs + INF_CLOCK_SOC.platform_libs + [
        "//boards/native/native_sim:board_native_sim",
        "//drivers/console:console",
    ],
    extra_archives         = INF_CLOCK_SOC.arch.extra_archives,
    offsets_label          = INF_CLOCK_SOC.arch.offsets_label,
    driver_srcs = {
        "console":          ["posix_arch_console.c"],
        "timer":            ["native_sim_timer.c", "sys_clock_init.c"],
        "arch_common_srcs": [],
        "libzephyr_extra":  ["build_native_sim/zephyr/misc/generated/configs.c"],
    },
)

def declare_targets():
    constraint_value(
        name               = "native_sim_cv",
        constraint_setting = "//constraints:board",
        visibility         = ["PUBLIC"],
    )
    platform(
        name              = "native_sim",
        constraint_values = [":native_sim_cv"],
        visibility        = ["PUBLIC"],
    )
