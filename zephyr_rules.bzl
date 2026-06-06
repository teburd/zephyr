# Buck2 build rules for Zephyr RTOS — multi-board support
# Non-hermetic: uses cmake-generated config headers from pre-run west builds.
# Prerequisites per board:
#   qemu_x86:       west build -b qemu_x86       samples/hello_world --build-dir build_qemu_x86
#   qemu_cortex_m3: west build -b qemu_cortex_m3 samples/hello_world --build-dir build_qemu_cortex_m3
#   qemu_riscv32:   west build -b qemu_riscv32   samples/hello_world --build-dir build_qemu_riscv32
#   qemu_riscv64:   west build -b qemu_riscv64   samples/hello_world --build-dir build_qemu_riscv64
#   native_sim:     west build -b native_sim      samples/hello_world --build-dir build_native_sim
#
# Required .buckconfig settings under [zephyr]:
#   root      = /absolute/path/to/this/zephyr/checkout
#   sdk       = /absolute/path/to/zephyr-sdk-<version>  (cross-compilation boards)
#   hal_cmsis = /absolute/path/to/west-workspace/modules/hal/cmsis_6  (ARM only)
#   python    = /absolute/path/to/python3  (defaults to "python3")

load("//boards:all.bzl", "ALL_BOARDS", "BOARD_NAME_SELECT")

_ZEPHYR_BASE = read_config("zephyr", "root") or fail("[zephyr] root must be set in .buckconfig")
_ZEPHYR_SDK  = read_config("zephyr", "sdk") or fail("[zephyr] sdk must be set in .buckconfig")
_PYTHON      = read_config("zephyr", "python", "python3")

# Individual constraint value labels — kept for backward compatibility and
# for BUCK files that need a single-board select arm.
CV_X86    = ALL_BOARDS["qemu_x86"].constraint_value_label
CV_ARM    = ALL_BOARDS["qemu_cortex_m3"].constraint_value_label
CV_NATIVE = ALL_BOARDS["native_sim"].constraint_value_label

# ---------------------------------------------------------------------------
# Runtime config derived from board → SOC → arch hierarchy
# ---------------------------------------------------------------------------

def _runtime_cfg(board_name):
    board = ALL_BOARDS[board_name]
    arch  = board.soc.arch
    build_abs = _ZEPHYR_BASE + "/" + board.build_dir

    if arch.gcc_triple:
        # Cross-compilation via SDK
        sdk_gnu     = _ZEPHYR_SDK + "/gnu/" + arch.gcc_triple
        gcc         = sdk_gnu + "/bin/" + arch.gcc_triple + "-gcc"
        ar          = sdk_gnu + "/bin/" + arch.gcc_triple + "-ar"
        objcopy     = sdk_gnu + "/bin/" + arch.gcc_triple + "-objcopy"
        sysroot     = sdk_gnu + "/" + arch.gcc_triple
        gcc_multilib = sdk_gnu + "/" + arch.gcc_multilib_suffix
        # Cross boards have both a pre0 and a final linker script.
        linker_pre0  = build_abs + "/zephyr/linker_zephyr_pre0.cmd"
        linker_final = build_abs + "/zephyr/linker.cmd"
    else:
        # Host-native compilation (e.g. native_sim)
        gcc         = arch.host_gcc
        ar          = arch.host_ar
        objcopy     = arch.host_objcopy
        sysroot     = None
        gcc_multilib = None
        # native_sim only has one linker script; use it for both passes.
        linker_pre0  = build_abs + "/zephyr/linker_zephyr_pre0.cmd"
        linker_final = build_abs + "/zephyr/linker_zephyr_pre0.cmd"

    return struct(
        gcc         = gcc,
        ar          = ar,
        objcopy     = objcopy,
        sysroot     = sysroot,
        gcc_multilib = gcc_multilib,

        gen_include      = build_abs + "/zephyr/include/generated",
        build_zephyr_dir = build_abs + "/zephyr",
        linker_pre0      = linker_pre0,
        linker_final     = linker_final,
        nsi_config       = build_abs + "/zephyr/NSI/nsi_config",

        arch_flags     = arch.arch_compile_flags,
        arch_defines   = arch.extra_defines,
        arch_include   = _ZEPHYR_BASE + "/" + arch.arch_include,
        soc_includes   = (
            ["-I" + _ZEPHYR_BASE + "/" + p for p in board.soc.soc_includes] +
            ["-I" + p for p in board.soc.external_includes]
        ),

        link_arch_flags  = arch.arch_link_flags,
        extra_link_flags = arch.extra_link_flags,
    )

# ---------------------------------------------------------------------------
# Supervisor-mode extra flags (internal; exposed via supervisor = True attr)
# ---------------------------------------------------------------------------

def _supervisor_flags(board_name):
    arch = ALL_BOARDS[board_name].soc.arch
    return [
        "-D__ZEPHYR_SUPERVISOR__",
        "-I" + _ZEPHYR_BASE + "/kernel/include",
        "-I" + _ZEPHYR_BASE + "/" + arch.arch_include,
        "-I" + _ZEPHYR_BASE + "/arch/common/include",
    ]

# ---------------------------------------------------------------------------
# Compile flag builders
# ---------------------------------------------------------------------------

_BASE_FLAGS = [
    "-fno-strict-aliasing", "-Os", "-fno-printf-return-value", "-fno-common",
    "-g", "-gdwarf-4", "-fdiagnostics-color=always",
    "-fno-pic", "-fno-pie",
    "-fno-reorder-functions",
    "-ffunction-sections", "-fdata-sections",
    "-Wall", "-Wformat", "-Wformat-security", "-Wno-format-zero-length",
    "-Wdouble-promotion", "-Wno-pointer-sign", "-Wpointer-arith",
    "-Wexpansion-to-defined", "-Wno-unused-but-set-variable",
    "-Werror=implicit-int",
]

_BASE_DEFINES = [
    "-DKERNEL", "-DK_HEAP_MEM_POOL_SIZE=0", "-D__ZEPHYR__=1",
]

# Additional defines and includes for cross-compiled boards using picolibc.
_CROSS_DEFINES = [
    "-DPICOLIBC_LONG_LONG_PRINTF_SCANF",
    "-D_POSIX_THREAD_SAFE_FUNCTIONS=200809L",
    "-D__LINUX_ERRNO_EXTENSIONS__",
]

def _board_compile_flags(cfg):
    cross = cfg.sysroot != None
    flags = list(_BASE_FLAGS) + cfg.arch_flags
    if cross:
        flags += ["-specs=picolibc.specs", "--sysroot=" + cfg.sysroot]
    flags += ["-imacros", cfg.gen_include + "/zephyr/autoconf.h"]
    if cross:
        flags += ["-imacros", _ZEPHYR_BASE + "/include/zephyr/toolchain/zephyr_stdint.h"]
    flags += list(_BASE_DEFINES)
    if cross:
        flags += _CROSS_DEFINES
    flags += cfg.arch_defines
    flags += ["-I" + cfg.gen_include + "/zephyr", "-I" + cfg.gen_include]
    flags += ["-I" + _ZEPHYR_BASE + "/include"]
    if cross:
        flags += [
            "-I" + _ZEPHYR_BASE + "/lib/libc/picolibc/include",
            "-I" + _ZEPHYR_BASE + "/subsys/portability/posix/c_lib_ext/getopt",
            "-isystem", _ZEPHYR_BASE + "/lib/libc/common/include",
        ]
    flags += cfg.soc_includes
    return flags

def _src_to_obj_name(src):
    path = src.short_path.replace("/", "__")
    if path.endswith(".S"):
        return path[:-2] + "_asm.o"
    return path[:-2] + ".o"

def _compile_c(ctx, src, extra_flags, cfg):
    obj = ctx.actions.declare_output(_src_to_obj_name(src))
    ctx.actions.run(
        cmd_args([cfg.gcc] + _board_compile_flags(cfg) +
                 extra_flags + ["-std=c17", "-c", src, "-o", obj.as_output()]),
        category   = "compile",
        identifier = src.short_path,
    )
    return obj

def _compile_asm(ctx, src, extra_flags, cfg):
    obj = ctx.actions.declare_output(_src_to_obj_name(src))
    ctx.actions.run(
        cmd_args([cfg.gcc] + _board_compile_flags(cfg) +
                 extra_flags + ["-D_ASMLANGUAGE", "-x", "assembler-with-cpp",
                                "-c", src, "-o", obj.as_output()]),
        category   = "compile_asm",
        identifier = src.short_path,
    )
    return obj

# ---------------------------------------------------------------------------
# zephyr_library — compile a group of C/S files into a static archive
# ---------------------------------------------------------------------------

def _zephyr_library_impl(ctx):
    board = ctx.attrs.board
    cfg   = _runtime_cfg(board)
    sup   = _supervisor_flags(board) if ctx.attrs.supervisor else []

    objs = []
    for src in ctx.attrs.srcs:
        if src.basename.endswith(".S"):
            obj = _compile_asm(ctx, src, sup, cfg)
        else:
            obj = _compile_c(ctx, src, sup, cfg)
        objs.append(obj)

    lib = ctx.actions.declare_output("lib" + ctx.label.name + ".a")
    ctx.actions.run(
        cmd_args([cfg.ar, "rcs", lib.as_output()] + objs),
        category = "archive",
    )
    return [DefaultInfo(default_output = lib)]

zephyr_library = rule(
    impl  = _zephyr_library_impl,
    attrs = {
        "srcs":       attrs.list(attrs.source(), default = []),
        "supervisor": attrs.bool(default = False),
        "board":      attrs.string(default = "qemu_x86"),
    },
)

# ---------------------------------------------------------------------------
# zephyr_object — compile a single C file to a standalone .o
# ---------------------------------------------------------------------------

def _zephyr_object_impl(ctx):
    board = ctx.attrs.board
    cfg   = _runtime_cfg(board)
    sup   = _supervisor_flags(board) if ctx.attrs.supervisor else []
    return [DefaultInfo(default_output = _compile_c(ctx, ctx.attrs.src, sup, cfg))]

zephyr_object = rule(
    impl  = _zephyr_object_impl,
    attrs = {
        "src":        attrs.source(),
        "supervisor": attrs.bool(default = False),
        "board":      attrs.string(default = "qemu_x86"),
    },
)

# ---------------------------------------------------------------------------
# _do_link — shared link implementation
# ---------------------------------------------------------------------------

def _do_link(ctx, cfg, output_name, linker_script, extra_objs, whole_libs, noarch_libs, extra_archive_libs):
    output    = ctx.actions.declare_output(output_name)
    cross     = cfg.sysroot != None

    empty_obj = ctx.actions.declare_output("empty_" + output_name + ".o")
    ctx.actions.run(
        cmd_args([cfg.gcc] + _board_compile_flags(cfg) +
                 ["-std=c17", "-c", _ZEPHYR_BASE + "/misc/empty_file.c",
                  "-o", empty_obj.as_output()]),
        category   = "compile",
        identifier = "empty_file_for_" + output_name,
    )

    offsets = ctx.attrs.offsets[DefaultInfo].default_outputs[0]

    if cross:
        link_flags = (
            cfg.link_arch_flags +
            ["-gdwarf-4", "-Os", "-fuse-ld=bfd"] +
            ["-Wl,--gc-sections", "-Wl,--build-id=none",
             "-Wl,--sort-common=descending", "-Wl,--sort-section=alignment",
             "-Wl,-u,_OffsetAbsSyms", "-Wl,-u,_ConfigAbsSyms",
             "-nostdlib", "-static", "-znoexecstack",
             "-Wl,-X", "-Wl,-N", "-Wl,--orphan-handling=warn", "-Wl,-no-pie"] +
            cfg.extra_link_flags +
            ["-specs=picolibc.specs", "-DPICOLIBC_LONG_LONG_PRINTF_SCANF",
             "-L", cfg.build_zephyr_dir,
             "-L", cfg.gcc_multilib, "-lc", "-lgcc"]
        )
    else:
        # Relocatable ("partial") link for native_sim embedded CPU software.
        # The final native executable is produced by the NSI Makefile in postprocess.
        link_flags = (
            cfg.link_arch_flags +
            ["-gdwarf-4", "-Os", "-fuse-ld=bfd", "-r"] +
            ["-Wl,--gc-sections", "-Wl,--build-id=none",
             "-Wl,--sort-common=descending", "-Wl,--sort-section=alignment",
             "-Wl,-u,_OffsetAbsSyms", "-Wl,-u,_ConfigAbsSyms"] +
            cfg.extra_link_flags +
            ["-L", cfg.build_zephyr_dir]
        )

    ctx.actions.run(
        cmd_args(
            [cfg.gcc, empty_obj, offsets] +
            extra_objs +
            ["-T", linker_script, "-Wl,--whole-archive"] +
            whole_libs +
            ["-Wl,--no-whole-archive"] +
            noarch_libs +
            extra_archive_libs +
            link_flags +
            ["-o", output.as_output()]
        ),
        category   = "link",
        identifier = output_name,
    )
    return output

# ---------------------------------------------------------------------------
# zephyr_link — generic link rule for pre0 or final ELF (any board/arch)
# ---------------------------------------------------------------------------

def _zephyr_link_impl(ctx):
    board = ctx.attrs.board
    cfg   = _runtime_cfg(board)

    # For native_sim the "final" link step is a passthrough — the NSI postprocess
    # in zephyr_postprocess already produced zephyr.exe.  Just forward its output.
    if cfg.sysroot == None and not ctx.attrs.use_pre0_linker:
        if ctx.attrs.extra_phase_deps:
            return [DefaultInfo(default_output =
                ctx.attrs.extra_phase_deps[0][DefaultInfo].default_outputs[0])]

    linker_script = cfg.linker_pre0 if ctx.attrs.use_pre0_linker else cfg.linker_final

    extra_objs = []
    for dep in ctx.attrs.extra_phase_deps:
        extra_objs += dep[DefaultInfo].default_outputs

    whole_libs     = [dep[DefaultInfo].default_outputs[0] for dep in ctx.attrs.whole_libs]
    noarch_libs    = [dep[DefaultInfo].default_outputs[0] for dep in ctx.attrs.noarch_libs]
    extra_archives = [dep[DefaultInfo].default_outputs[0] for dep in ctx.attrs.extra_archives]

    elf = _do_link(ctx, cfg, ctx.attrs.output_name, linker_script,
                   extra_objs, whole_libs, noarch_libs, extra_archives)
    return [DefaultInfo(default_output = elf)]

zephyr_link = rule(
    impl  = _zephyr_link_impl,
    attrs = {
        "board":            attrs.string(),
        "output_name":      attrs.string(default = "zephyr.elf"),
        "use_pre0_linker":  attrs.bool(default = False),
        "whole_libs":       attrs.list(attrs.dep(), default = []),
        "noarch_libs":      attrs.list(attrs.dep(), default = []),
        "extra_archives":   attrs.list(attrs.dep(), default = []),
        "extra_phase_deps": attrs.list(attrs.dep(), default = []),
        "offsets":          attrs.dep(),
    },
)

# ---------------------------------------------------------------------------
# zephyr_postprocess — unified post-processing rule (x86, ARM Cortex-M, native_sim)
# Dispatches at analysis time via ctx.attrs.board, so board can be select().
# ---------------------------------------------------------------------------

def _zephyr_postprocess_impl(ctx):
    pre0  = ctx.attrs.pre0[DefaultInfo].default_outputs[0]
    board = ctx.attrs.board
    cfg   = _runtime_cfg(board)
    arch  = ALL_BOARDS[board].soc.arch.postprocess

    if arch == "x86":
        static_idt = ctx.actions.declare_output("staticIdt.o")
        irq_map    = ctx.actions.declare_output("irq_int_vector_map.o")
        irq_alloc  = ctx.actions.declare_output("irq_vectors_alloc.o")
        gdt_o      = ctx.actions.declare_output("gdt.o")
        pages      = ctx.actions.declare_output("pagetables.o")

        script = """set -e
PRE0="$1"; SIDT="$2"; IRQMAP="$3"; IRQALLOC="$4"; GDT="$5"; PT="$6"
TMPD="$(mktemp -d)"; trap "rm -rf $TMPD" EXIT
PY="{py}"; OC="{oc}"; ZB="{zb}"

$PY "$ZB/arch/x86/gen_idt.py" --kernel "$PRE0" \\
    --output-idt $TMPD/idt.bin \\
    --vector-map $TMPD/irqmap.bin \\
    --output-vectors-alloc $TMPD/irqalloc.bin
$OC -I binary -B i386 -O elf32-i386 --rename-section .data=staticIdt,CONTENTS,ALLOC,LOAD,READONLY,DATA $TMPD/idt.bin "$SIDT"
$OC -I binary -B i386 -O elf32-i386 --rename-section .data=irq_int_vector_map,CONTENTS,ALLOC,LOAD,READONLY,DATA $TMPD/irqmap.bin "$IRQMAP"
$OC -I binary -B i386 -O elf32-i386 --rename-section .data=irq_vectors_alloc,CONTENTS,ALLOC,LOAD,READONLY,DATA $TMPD/irqalloc.bin "$IRQALLOC"

$PY "$ZB/arch/x86/gen_gdt.py" --kernel "$PRE0" --output-gdt $TMPD/gdt.bin
$OC -I binary -B i386 -O elf32-i386 --rename-section .data=gdt,CONTENTS,ALLOC,LOAD,READONLY,DATA $TMPD/gdt.bin "$GDT"

$PY "$ZB/arch/x86/gen_mmu.py" --kernel "$PRE0" --output $TMPD/pt.bin
$OC -I binary -B i386 -O elf32-i386 --rename-section .data=pagetables,CONTENTS,ALLOC,LOAD,READONLY,DATA $TMPD/pt.bin "$PT"
""".format(py = _PYTHON, oc = cfg.objcopy, zb = _ZEPHYR_BASE)

        ctx.actions.run(
            cmd_args(["/bin/sh", "-c", script, "--",
                      pre0, static_idt.as_output(), irq_map.as_output(),
                      irq_alloc.as_output(), gdt_o.as_output(), pages.as_output()]),
            category = "x86_postprocess",
        )
        return [DefaultInfo(default_outputs = [static_idt, irq_map, irq_alloc, gdt_o, pages])]

    elif arch == "isr_tables":
        # Used by ARM Cortex-M and RISC-V — any arch that uses gen_isr_tables.py.
        isr_src = ctx.actions.declare_output("isr_tables.c")
        isr_vt  = ctx.actions.declare_output("isr_tables_vt.ld")
        isr_swi = ctx.actions.declare_output("isr_tables_swi.ld")

        isr_flags = " ".join(ALL_BOARDS[board].soc.arch.isr_table_flags)

        script = """set -e
PRE0="$1"; ISRC="$2"; IVT="$3"; ISWI="$4"
PY="{py}"; ZB="{zb}"
$PY "$ZB/scripts/build/gen_isr_tables.py" \\
    --output-source "$ISRC" \\
    --linker-output-files "$IVT" "$ISWI" \\
    --kernel "$PRE0" \\
    --intlist-section .intList --intlist-section intList \\
    {flags}
""".format(py = _PYTHON, zb = _ZEPHYR_BASE, flags = isr_flags)

        ctx.actions.run(
            cmd_args(["/bin/sh", "-c", script, "--",
                      pre0, isr_src.as_output(), isr_vt.as_output(), isr_swi.as_output()]),
            category = "gen_isr_tables",
        )

        isr_obj = ctx.actions.declare_output("isr_tables.o")
        ctx.actions.run(
            cmd_args([cfg.gcc] + _board_compile_flags(cfg) +
                     _supervisor_flags(board) +
                     ["-std=c17", "-c", isr_src, "-o", isr_obj.as_output()]),
            category   = "compile",
            identifier = "isr_tables.c",
        )
        return [DefaultInfo(default_outputs = [isr_obj])]

    elif arch == "native_sim":
        # Run the Native Simulator Infrastructure (NSI) Makefile to produce
        # the final host executable (zephyr.exe) from the relocatable zephyr.elf.
        # Reuses cmake-pre-built runner.a / linker_script.ld from the NSI build dir.
        exe = ctx.actions.declare_output("zephyr.exe")

        script = """set -e
PRE0="$(pwd)/$1"
EXE="$(pwd)/$2"
make -f "{mk}" link_with_esw --warn-undefined-variables -r \\
  NSI_CONFIG_FILE="{conf}" \\
  NSI_EMBEDDED_CPU_SW="$PRE0" \\
  NSI_EXE="$EXE"
""".format(
            mk   = _ZEPHYR_BASE + "/scripts/native_simulator/Makefile",
            conf = cfg.nsi_config,
        )

        ctx.actions.run(
            cmd_args(["/bin/sh", "-c", script, "--", pre0, exe.as_output()]),
            category = "native_sim_nsi",
        )
        return [DefaultInfo(default_output = exe)]

zephyr_postprocess = rule(
    impl  = _zephyr_postprocess_impl,
    attrs = {
        "pre0":  attrs.dep(),
        "board": attrs.string(),
    },
)

# ---------------------------------------------------------------------------
# zephyr_application — high-level macro for a complete Zephyr application.
#
# Creates the full build graph (library → pre0 link → postprocess → final link)
# for all supported boards.  Platform deps (arch libs, SOC libs, board drivers)
# are declared on each board struct and composed here — no board-conditional
# logic lives in this macro.
#
# extra_libs: optional list of additional dep labels appended to whole_libs
#             for every board (e.g. subsystem libraries the app depends on).
# ---------------------------------------------------------------------------

def zephyr_application(srcs, extra_libs = None):
    _platform = select({b.constraint_value_label: b.platform_libs  for b in ALL_BOARDS.values()})
    _whole    = [":app"] + _platform + (extra_libs if extra_libs else [])
    _noarch   = ["//kernel:kernel"]
    _extra    = select({b.constraint_value_label: b.extra_archives  for b in ALL_BOARDS.values()})
    _offsets  = select({b.constraint_value_label: b.offsets_label   for b in ALL_BOARDS.values()})

    board = BOARD_NAME_SELECT

    zephyr_library(name = "app", board = board, srcs = srcs)

    zephyr_link(
        name = "zephyr_pre0", board = board, output_name = "zephyr_pre0.elf",
        use_pre0_linker = True,
        whole_libs = _whole, noarch_libs = _noarch, extra_archives = _extra, offsets = _offsets,
    )

    zephyr_postprocess(name = "pp", pre0 = ":zephyr_pre0", board = board)

    zephyr_link(
        name = "zephyr", board = board, output_name = "zephyr.elf",
        whole_libs = _whole, noarch_libs = _noarch, extra_archives = _extra, offsets = _offsets,
        extra_phase_deps = [":pp"],
    )
