# Buck2 build rules for Zephyr RTOS — multi-board support
# Non-hermetic: uses cmake-generated config headers from pre-run west builds.
# Prerequisites per board:
#   qemu_x86:       west build -b qemu_x86       samples/hello_world --build-dir build_qemu_x86
#   qemu_cortex_m3: west build -b qemu_cortex_m3 samples/hello_world --build-dir build_qemu_cortex_m3
#
# Required .buckconfig settings under [zephyr]:
#   root      = /absolute/path/to/this/zephyr/checkout
#   sdk       = /absolute/path/to/zephyr-sdk-<version>
#   hal_cmsis = /absolute/path/to/west-workspace/modules/hal/cmsis_6  (ARM only)
#   python    = /absolute/path/to/python3  (defaults to "python3")

load("//boards:all.bzl", "ALL_BOARDS")

_ZEPHYR_BASE = read_config("zephyr", "root") or fail("[zephyr] root must be set in .buckconfig")
_ZEPHYR_SDK  = read_config("zephyr", "sdk") or fail("[zephyr] sdk must be set in .buckconfig")
_PYTHON      = read_config("zephyr", "python", "python3")

# Constraint value labels for use in select() across BUCK files.
CV_X86 = ALL_BOARDS["qemu_x86"].constraint_value_label
CV_ARM = ALL_BOARDS["qemu_cortex_m3"].constraint_value_label

# ---------------------------------------------------------------------------
# Runtime config derived from board → SOC → arch hierarchy
# ---------------------------------------------------------------------------

def _runtime_cfg(board_name):
    board     = ALL_BOARDS[board_name]
    arch      = board.soc.arch
    sdk_gnu   = _ZEPHYR_SDK + "/gnu/" + arch.gcc_triple
    build_abs = _ZEPHYR_BASE + "/" + board.build_dir
    return struct(
        gcc         = sdk_gnu + "/bin/" + arch.gcc_triple + "-gcc",
        ar          = sdk_gnu + "/bin/" + arch.gcc_triple + "-ar",
        objcopy     = sdk_gnu + "/bin/" + arch.gcc_triple + "-objcopy",
        sysroot     = sdk_gnu + "/" + arch.gcc_triple,
        gcc_multilib = sdk_gnu + "/" + arch.gcc_multilib_suffix,

        gen_include      = build_abs + "/zephyr/include/generated",
        build_zephyr_dir = build_abs + "/zephyr",
        linker_pre0      = build_abs + "/zephyr/linker_zephyr_pre0.cmd",
        linker_final     = build_abs + "/zephyr/linker.cmd",

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
    "-specs=picolibc.specs",
    "-Wall", "-Wformat", "-Wformat-security", "-Wno-format-zero-length",
    "-Wdouble-promotion", "-Wno-pointer-sign", "-Wpointer-arith",
    "-Wexpansion-to-defined", "-Wno-unused-but-set-variable",
    "-Werror=implicit-int",
]

_BASE_DEFINES = [
    "-DKERNEL", "-DK_HEAP_MEM_POOL_SIZE=0", "-DPICOLIBC_LONG_LONG_PRINTF_SCANF",
    "-D_POSIX_THREAD_SAFE_FUNCTIONS=200809L", "-D__LINUX_ERRNO_EXTENSIONS__",
    "-D__ZEPHYR__=1",
]

_BASE_INCLUDES = [
    "-I" + _ZEPHYR_BASE + "/include",
    "-I" + _ZEPHYR_BASE + "/lib/libc/picolibc/include",
    "-I" + _ZEPHYR_BASE + "/subsys/portability/posix/c_lib_ext/getopt",
    "-isystem", _ZEPHYR_BASE + "/lib/libc/common/include",
]

def _board_compile_flags(cfg):
    return (
        _BASE_FLAGS +
        cfg.arch_flags +
        ["--sysroot=" + cfg.sysroot] +
        ["-imacros", cfg.gen_include + "/zephyr/autoconf.h",
         "-imacros", _ZEPHYR_BASE + "/include/zephyr/toolchain/zephyr_stdint.h"] +
        _BASE_DEFINES +
        cfg.arch_defines +
        ["-I" + cfg.gen_include + "/zephyr",
         "-I" + cfg.gen_include] +
        _BASE_INCLUDES +
        cfg.soc_includes
    )

def _src_to_obj_name(src):
    return src.short_path.replace("/", "__").replace(".c", ".o").replace(".S", ".o")

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
    output = ctx.actions.declare_output(output_name)

    empty_obj = ctx.actions.declare_output("empty_" + output_name + ".o")
    ctx.actions.run(
        cmd_args([cfg.gcc] + _board_compile_flags(cfg) +
                 ["-std=c17", "-c", _ZEPHYR_BASE + "/misc/empty_file.c",
                  "-o", empty_obj.as_output()]),
        category   = "compile",
        identifier = "empty_file_for_" + output_name,
    )

    offsets = ctx.attrs.offsets[DefaultInfo].default_outputs[0]

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
    cfg = _runtime_cfg(ctx.attrs.board)
    linker_script = cfg.linker_pre0 if ctx.attrs.use_pre0_linker else cfg.linker_final

    extra_objs = []
    for dep in ctx.attrs.extra_phase_deps:
        extra_objs += dep[DefaultInfo].default_outputs

    whole_libs    = [dep[DefaultInfo].default_outputs[0] for dep in ctx.attrs.whole_libs]
    noarch_libs   = [dep[DefaultInfo].default_outputs[0] for dep in ctx.attrs.noarch_libs]
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
# zephyr_postprocess — unified post-processing rule (x86 and ARM Cortex-M)
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

    elif arch == "arm_cortex_m":
        isr_src = ctx.actions.declare_output("isr_tables.c")
        isr_vt  = ctx.actions.declare_output("isr_tables_vt.ld")
        isr_swi = ctx.actions.declare_output("isr_tables_swi.ld")

        script = """set -e
PRE0="$1"; ISRC="$2"; IVT="$3"; ISWI="$4"
PY="{py}"; ZB="{zb}"
$PY "$ZB/scripts/build/gen_isr_tables.py" \\
    --output-source "$ISRC" \\
    --linker-output-files "$IVT" "$ISWI" \\
    --kernel "$PRE0" \\
    --intlist-section .intList --intlist-section intList \\
    --sw-isr-table --vector-table
""".format(py = _PYTHON, zb = _ZEPHYR_BASE)

        ctx.actions.run(
            cmd_args(["/bin/sh", "-c", script, "--",
                      pre0, isr_src.as_output(), isr_vt.as_output(), isr_swi.as_output()]),
            category = "arm_gen_isr",
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

zephyr_postprocess = rule(
    impl  = _zephyr_postprocess_impl,
    attrs = {
        "pre0":  attrs.dep(),
        "board": attrs.string(),
    },
)
