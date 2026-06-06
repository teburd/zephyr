ARM_CORTEX_M_ARCH = struct(
    name = "arm_cortex_m",
    gcc_triple = "arm-zephyr-eabi",
    gcc_multilib_suffix = "lib/gcc/arm-zephyr-eabi/14.3.0/thumb/v7-m/nofp/space",
    arch_include = "arch/arm/include",
    postprocess = "isr_tables",

    # Compiler flags (architecture-specific, excluding sysroot and imacros)
    arch_compile_flags = [
        "-mcpu=cortex-m3", "-mthumb", "-mabi=aapcs",
        "-mfp16-format=ieee", "-mtp=soft",
        "-fno-asynchronous-unwind-tables", "-ftls-model=local-exec",
        "--param=min-pagesize=0", "-fno-defer-pop",
    ],
    extra_defines = ["-D__PROGRAM_START"],
    extra_warnings = ["-Wshadow"],

    # Linker flags
    arch_link_flags = ["-mcpu=cortex-m3", "-mthumb", "-mabi=aapcs", "-mfp16-format=ieee", "-mtp=soft"],
    extra_link_flags = ["-Wl,--undefined=_sw_isr_table", "-Wl,--undefined=_irq_vector_table"],
    isr_table_flags = ["--sw-isr-table", "--vector-table"],

    # Platform library deps contributed by this arch to every application.
    platform_libs = [
        "//:libzephyr",
        "//arch/common:arch_common",
        "//arch/arm/core:arch_arm_core",
        "//arch/arm/core/cortex_m:arch_arm_cortex_m",
        "//lib/libc:picolibc",
        "//lib/libc:libc_common",
        "//lib/libc:libc_validate",
        "//subsys/portability/posix:posix",
    ],
    extra_archives = ["//arch/common:isr_tables"],
    offsets_label  = "//arch/arm/core:offsets",
)
