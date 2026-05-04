load("//boards/qemu/x86:board.bzl", "QEMU_X86")
load("//boards/qemu/cortex_m3:board.bzl", "QEMU_CORTEX_M3")

# To add a new board: create boards/<vendor>/<board>/board.bzl,
# add an empty BUCK file in that directory, then add one entry here.
ALL_BOARDS = {
    "qemu_x86":       QEMU_X86,
    "qemu_cortex_m3": QEMU_CORTEX_M3,
}

# Ordered list of platform labels — one per board, same order as ALL_BOARDS.
ALL_PLATFORMS = [board.platform_label for board in ALL_BOARDS.values()]
