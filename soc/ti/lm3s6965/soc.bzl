load("//arch/arm:arch.bzl", "ARM_CORTEX_M_ARCH")

_HAL_CMSIS = read_config("zephyr", "hal_cmsis", None)

LM3S6965_SOC = struct(
    name = "lm3s6965",
    arch = ARM_CORTEX_M_ARCH,
    soc_includes = [                   # repo-relative, no -I prefix
        "soc/ti/lm3s6965",
        "soc/ti/lm3s6965/.",
        "modules/cmsis_6/.",
    ],
    external_includes = (              # absolute paths, no -I prefix
        [_HAL_CMSIS + "/CMSIS/Core/Include"] if _HAL_CMSIS else []
    ),
    soc_lib_label = "//soc/ti/lm3s6965:soc_ti_lm3s6965_qemu_cortex_m3",
)
