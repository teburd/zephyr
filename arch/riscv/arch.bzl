RISCV32_ARCH = struct(
    name = "riscv32",
    gcc_triple = "riscv64-zephyr-elf",
    gcc_multilib_suffix = "lib/gcc/riscv64-zephyr-elf/14.3.0/rv32imac_zicsr_zifencei/ilp32/space",
    arch_include = "arch/riscv/include",
    postprocess = "isr_tables",

    arch_compile_flags = [
        "-mabi=ilp32", "-march=rv32imac_zicsr_zifencei", "-mcmodel=medlow",
        "-fno-asynchronous-unwind-tables", "-ftls-model=local-exec",
        "--param=min-pagesize=0", "-fno-defer-pop",
    ],
    extra_defines  = [],
    extra_warnings = [],

    arch_link_flags  = ["-mabi=ilp32", "-march=rv32imac_zicsr_zifencei", "-mcmodel=medlow"],
    extra_link_flags = [
        "-Wl,--undefined=_sw_isr_table",
        "-Wl,--undefined=_irq_vector_table",
    ],
    isr_table_flags = ["--sw-isr-table"],

    platform_libs = [
        "//:libzephyr",
        "//arch/common:arch_common",
        "//arch/riscv/core:arch_riscv_core",
        "//lib/libc:picolibc",
        "//lib/libc:libc_common",
        "//lib/libc:libc_validate",
        "//subsys/portability/posix:posix",
    ],
    extra_archives = ["//arch/common:isr_tables"],
    offsets_label  = "//arch/riscv/core:offsets",
)

RISCV64_ARCH = struct(
    name = "riscv64",
    gcc_triple = "riscv64-zephyr-elf",
    gcc_multilib_suffix = "lib/gcc/riscv64-zephyr-elf/14.3.0/rv64imac_zicsr_zifencei/lp64/medany/space",
    arch_include = "arch/riscv/include",
    postprocess = "isr_tables",

    arch_compile_flags = [
        "-mabi=lp64", "-march=rv64imac_zicsr_zifencei", "-mcmodel=medany",
        "-fno-asynchronous-unwind-tables", "-ftls-model=local-exec",
        "--param=min-pagesize=0", "-fno-defer-pop",
    ],
    extra_defines  = [],
    extra_warnings = [],

    arch_link_flags  = ["-mabi=lp64", "-march=rv64imac_zicsr_zifencei", "-mcmodel=medany"],
    extra_link_flags = [
        "-Wl,--undefined=_sw_isr_table",
        "-Wl,--undefined=_irq_vector_table",
    ],
    isr_table_flags = ["--sw-isr-table"],

    platform_libs = [
        "//:libzephyr",
        "//arch/common:arch_common",
        "//arch/riscv/core:arch_riscv_core",
        "//lib/libc:picolibc",
        "//lib/libc:libc_common",
        "//lib/libc:libc_validate",
        "//subsys/portability/posix:posix",
    ],
    extra_archives = ["//arch/common:isr_tables"],
    offsets_label  = "//arch/riscv/core:offsets",
)
