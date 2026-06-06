POSIX_ARCH = struct(
    name = "posix",
    # Native host compilation — gcc_triple = None signals no SDK cross-toolchain.
    gcc_triple       = None,
    gcc_multilib_suffix = None,
    host_gcc    = "/usr/bin/gcc",
    host_ar     = "/usr/bin/ar",
    host_objcopy = "/usr/bin/objcopy",
    arch_include = "arch/posix/include",
    postprocess  = "native_sim",

    # Compiler flags for 32-bit native posix target
    arch_compile_flags = [
        "-m32", "-msse2", "-mfpmath=sse",
        "-fvisibility=hidden", "-fno-freestanding",
        "-fno-asynchronous-unwind-tables",
        "--param=min-pagesize=0", "-fno-defer-pop",
    ],
    extra_defines  = [],
    extra_warnings = [],

    # Linker flags for the partial (relocatable) embedded CPU software link
    arch_link_flags  = ["-m32"],
    extra_link_flags = ["-Wl,--unresolved-symbols=ignore-all"],

    # Platform library deps contributed by this arch to every application.
    platform_libs = [
        "//:libzephyr",
        "//arch/common:arch_common",
        "//arch/posix/core:arch_posix_core",
    ],
    extra_archives = [],
    offsets_label  = "//arch/posix/core:offsets",
)
