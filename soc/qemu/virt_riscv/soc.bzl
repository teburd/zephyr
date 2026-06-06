load("//arch/riscv:arch.bzl", "RISCV32_ARCH", "RISCV64_ARCH")

VIRT_RISCV32_SOC = struct(
    name = "qemu_virt_riscv32",
    arch = RISCV32_ARCH,
    soc_includes = [
        "soc/qemu/virt_riscv",
        "soc/common/riscv-privileged/.",
    ],
    external_includes = [],
    platform_libs = [
        "//soc/qemu/virt_riscv:soc_qemu_virt_riscv",
        "//soc/common/riscv-privileged:riscv_privileged",
        "//drivers/interrupt_controller:intc_plic",
        "//drivers/serial:serial",
        "//drivers/timer:timer",
    ],
)

VIRT_RISCV64_SOC = struct(
    name = "qemu_virt_riscv64",
    arch = RISCV64_ARCH,
    soc_includes = [
        "soc/qemu/virt_riscv",
        "soc/common/riscv-privileged/.",
    ],
    external_includes = [],
    platform_libs = [
        "//soc/qemu/virt_riscv:soc_qemu_virt_riscv",
        "//soc/common/riscv-privileged:riscv_privileged",
        "//drivers/interrupt_controller:intc_plic",
        "//drivers/serial:serial",
        "//drivers/timer:timer",
    ],
)
