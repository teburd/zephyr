load("//boards/qemu/x86:board.bzl", "QEMU_X86")
load("//boards/qemu/cortex_m3:board.bzl", "QEMU_CORTEX_M3")
load("//boards/qemu/riscv32:board.bzl", "QEMU_RISCV32")
load("//boards/qemu/riscv64:board.bzl", "QEMU_RISCV64")
load("//boards/native/native_sim:board.bzl", "NATIVE_SIM")

# To add a new board: create boards/<vendor>/<board>/board.bzl with the
# standard struct fields, add an empty BUCK, then add one entry here.
ALL_BOARDS = {
    "qemu_x86":       QEMU_X86,
    "qemu_cortex_m3": QEMU_CORTEX_M3,
    "qemu_riscv32":   QEMU_RISCV32,
    "qemu_riscv64":   QEMU_RISCV64,
    "native_sim":     NATIVE_SIM,
}

# Ordered list of platform labels — one per board, same order as ALL_BOARDS.
ALL_PLATFORMS = [board.platform_label for board in ALL_BOARDS.values()]

# ---------------------------------------------------------------------------
# Select helpers — pre-built select() values for common BUCK patterns.
#
# BOARD_NAME_SELECT: maps each board's constraint value → board name string.
#   Use as `board = BOARD_NAME_SELECT` to replace per-file board selects.
#
# RISCV_BOARD_NAME_SELECT / ARM_CORTEX_M_BOARD_NAME_SELECT: arch-family
#   subsets, for targets that only exist for a specific arch family.
# ---------------------------------------------------------------------------

BOARD_NAME_SELECT = select({
    b.constraint_value_label: b.name
    for b in ALL_BOARDS.values()
})

RISCV_BOARD_NAME_SELECT = select({
    b.constraint_value_label: b.name
    for b in ALL_BOARDS.values()
    if b.soc.arch.name in ("riscv32", "riscv64")
})

ARM_CORTEX_M_BOARD_NAME_SELECT = select({
    b.constraint_value_label: b.name
    for b in ALL_BOARDS.values()
    if b.soc.arch.name == "arm_cortex_m"
})

# Boards that use gen_isr_tables.py (ARM Cortex-M and RISC-V).
ISR_TABLES_BOARD_NAME_SELECT = select({
    b.constraint_value_label: b.name
    for b in ALL_BOARDS.values()
    if b.soc.arch.postprocess == "isr_tables"
})

# ---------------------------------------------------------------------------
# driver_src_select(driver) — returns a select mapping each board's CV to
# that board's source file list for the named driver.  Boards that do not
# define the driver return an empty list.
#
# Usage in a driver BUCK:
#   srcs = driver_src_select("timer"),
# ---------------------------------------------------------------------------

def driver_src_select(driver):
    return select({
        b.constraint_value_label: b.driver_srcs.get(driver, [])
        for b in ALL_BOARDS.values()
    })
