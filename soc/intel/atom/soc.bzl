load("//arch/x86:arch.bzl", "X86_ARCH")

ATOM_SOC = struct(
    name = "atom",
    arch = X86_ARCH,
    soc_includes     = ["soc/intel/atom"],  # repo-relative, no -I prefix
    external_includes = [],
    soc_lib_label = None,
)
