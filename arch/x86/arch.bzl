X86_ARCH = struct(
    name = "x86",
    gcc_triple = "x86_64-zephyr-elf",
    gcc_multilib_suffix = "lib/gcc/x86_64-zephyr-elf/14.3.0/32/soft-float/space",
    arch_include = "arch/x86/include",
    postprocess = "x86",

    # Compiler flags (architecture-specific, excluding sysroot and imacros)
    arch_compile_flags = [
        "-m32", "-msoft-float", "-Wa,--divide",
        "-mpreferred-stack-boundary=2", "-mno-mmx", "-mno-sse",
        "-fno-asynchronous-unwind-tables", "-ftls-model=local-exec",
        "--param=min-pagesize=0", "-fno-defer-pop",
    ],
    extra_defines = [],
    extra_warnings = [],

    # Linker flags
    arch_link_flags  = ["-m32", "-msoft-float"],
    extra_link_flags = [],
)
