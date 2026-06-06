load("//arch/posix:arch.bzl", "POSIX_ARCH")

INF_CLOCK_SOC = struct(
    name = "inf_clock",
    arch = POSIX_ARCH,
    soc_includes = [                   # repo-relative, no -I prefix
        "soc/native/inf_clock",
        "boards/native/native_sim",
        "scripts/native_simulator/common/src/include",
        "scripts/native_simulator/native/src/include",
    ],
    external_includes = [],
    platform_libs = [
        "//soc/native/inf_clock:soc_native_inf_clock",
        "//drivers/timer:timer",
    ],
)
