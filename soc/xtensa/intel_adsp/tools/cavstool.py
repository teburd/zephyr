#!/usr/bin/env python3
# Copyright(c) 2022 Intel Corporation. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import os
import sys
import struct
import logging
import asyncio
import time
import subprocess
import ctypes
import mmap
import argparse
import socketserver
import threading
import hashlib

# Global variable use to sync between log and request services.
# When it is true, the adsp is able to start running.
START_OUTPUT = False
lock = threading.Lock()

# INADDR_ANY as default
HOST = ""
PORT_LOG = 9999
PORT_REQ = PORT_LOG + 1
BUF_SIZE = 4096

# Define the command and the max size
CMD_LOG_START = "start_log"
CMD_DOWNLOAD = "download"
MAX_CMD_SZ = 16

# Define the header format and size for
# transmiting the firmware
PACKET_HEADER_FORMAT_FW = "I 42s 32s"
HEADER_SZ = 78


logging.basicConfig(level=logging.INFO)
log = logging.getLogger("adsptool")

# SRAM windows.  Each appears in a 128k region starting at 512k.
#
# Window 0 is the FW_STATUS area, and 4k after that the IPC "outbox"
# Window 1 is the IPC "inbox" (host-writable memory, just 384 bytes currently)
# Window 2 is unused by this script
# Window 3 is winstream-formatted log output
OUTBOX_OFFSET = (512 + (0 * 128)) * 1024 + 4096
INBOX_OFFSET = (512 + (1 * 128)) * 1024
WINSTREAM_OFFSET = (512 + (3 * 128)) * 1024

# ADSPCS bits
CRST = 0
CSTALL = 8
SPA = 16
CPA = 24


class Regs:
    """Provides named register access easily

    Syntactic sugar to make register block definition & use look nice.
    Instantiate from a base address, assign offsets to (uint32) named registers
    as fields, call freeze(), then the field acts as a direct alias for the
    register!
    """

    def __init__(self, base_addr):
        vars(self)["base_addr"] = base_addr
        vars(self)["ptrs"] = {}
        vars(self)["frozen"] = False

    def freeze(self):
        vars(self)["frozen"] = True

    def __setattr__(self, name, val):
        if not self.frozen and name not in self.ptrs:
            addr = self.base_addr + val
            self.ptrs[name] = ctypes.c_uint32.from_address(addr)
        else:
            self.ptrs[name].value = val

    def __getattr__(self, name):
        return self.ptrs[name].value


class HDAStreamRegMap:
    """HDAStream Register Mapping

    Map and manipulate an HDA host input or output stream
    """

    def __init__(self, hdamem, stream_id: int, is_input):
        self.dev = dev
        self.stream_id = stream_id
        self.hdamem = hdamem
        self.base = hdamem + 0x0080 + (stream_id * 0x20)
        log.info(
            f"Mapping registers for hda stream {self.stream_id} at base {self.base:x}"
        )

        self.hda = Regs(hdamem)
        self.hda.GCAP = 0x0000
        self.hda.GCTL = 0x0008
        self.hda.DPLBASE = 0x0070
        self.hda.DPUBASE = 0x0074
        self.hda.SPBFCH = 0x0700
        self.hda.SPBFCTL = 0x0704
        self.hda.PPCH = 0x0800
        self.hda.PPCTL = 0x0804
        self.hda.PPSTS = 0x0808
        self.hda.SPIB = 0x0708 + stream_id * 0x08
        self.hda.freeze()

        self.stream = Regs(self.base)
        self.stream.CTL = 0x00
        self.stream.STS = 0x03
        self.stream.LPIB = 0x04
        self.stream.CBL = 0x08
        self.stream.LVI = 0x0C
        self.stream.FIFOW = 0x0E
        self.stream.FIFOS = 0x10
        self.stream.FMT = 0x12
        self.stream.FIFOL = 0x14
        self.stream.BDPL = 0x18
        self.stream.BDPU = 0x1C
        self.stream.freeze()

        self.dbg0 = Regs(hdamem + 0x0084 + (0x20 * stream_id))
        self.dbg0.DPIB = 0x00
        self.dbg0.EFIFOS = 0x10
        self.dbg0.freeze()

    def debug(self):
        log.debug(
            "HDA %d: PPROC %d, CTL 0x%x, LPIB 0x%x, BDPU 0x%x, "
            " BDPL 0x%x, CBL 0x%x, LVI 0x%x",
            self.stream_id,
            (hda.PPCTL >> self.stream_id) & 1,
            self.regs.CTL,
            self.regs.LPIB,
            self.regs.BDPU,
            self.regs.BDPL,
            self.regs.CBL,
            self.regs.LVI,
        )
        log.debug(
            "    FIFOW %d, FIFOS %d, FMT %x, FIFOL %d, DPIB %d, " " EFIFOS %d",
            self.regs.FIFOW & 0x7,
            self.regs.FIFOS,
            self.regs.FMT,
            self.regs.FIFOL,
            self.dbg0.DPIB,
            self.dbg0.EFIFOS,
        )
        log.debug(
            "    status: FIFORDY %d, DESE %d, FIFOE %d, BCIS %d",
            (self.regs.STS >> 5) & 1,
            (self.regs.STS >> 4) & 1,
            (self.regs.STS >> 3) & 1,
            (self.regs.STS >> 2) & 1,
        )


class HDAStream:
    def __init__(self, reg_map, buf_lens, allocator):
        self.reg_map = reg_map
        self.buf_lens = buf_lens

        log.info(f"Stream {self.stream_id}: Allocating buffers")
        self.bufs = [allocator.alloc(buf_len) for buf_len in buf_lens]

        # buffer descriptor "list" is really an array with a high and low address range
        # each entry is two 32 bit words (buffer addr, buffer len) in sequence
        self.bdl = allocator.alloc(len(buf_lens) * 8)

        for i, buf in enum(self.bufs):
            self.bdl.mem[i * 8 : i * 8 + 8] = struct.pack(
                "<QQ", buf.phys_addr, buf.size
            )

        log.info(
            f"Stream {self.stream_id}: Setting buffer list, length,"
            " stream id and traffic priority"
        )
        # must be set to something other than 0?
        self.reg_map.stream.CTL = ((self.stream_id & 0xFF) << 20) | (1 << 18)
        self.reg_map.stream.BDPU = (self.bdl.phys_addr >> 32) & 0xFFFFFFFF
        self.reg_map.stream.BDPL = self.bdl.phys_addr & 0xFFFFFFFF
        self.reg_map.stream.CBL = sum(self.buf_lens)
        self.reg_map.stream.LVI = len(self.bufs) - 1
        self.reg_map.debug()
        log.info(f"Configured stream {self.stream_id}")
        self._reset()

    def __del__(self):
        self._reset()

    def start(self):
        log.info(f"Starting stream {self.stream_id}, CTL {self.regs.CTL:x}")
        self.regs.CTL |= 2
        log.info(f"Started stream {self.stream_id}, CTL {self.regs.CTL:x}")

    def stop(self):
        log.info(f"Stopping stream {self.stream_id}, CTL {self.regs.CTL:x}")
        self.regs.CTL &= 2
        time.sleep(0.1)
        self.regs.CTL |= 1
        log.info(f"Stopped stream {self.stream_id}, CTL {self.regs.CTL:x}")

    def _reset(self):
        """
        Turn DMA off and reset the stream.  Clearing START first is a
        noop per the spec, but absolutely required for stability.
        Apparently the reset doesn't stop the stream, and the next load
        starts before it's ready and kills the load (and often the DSP).
        The sleep too is required, on at least one board (a fast
        chromebook) putting the two writes next each other also hangs
        the DSP!
        """
        log.info(f"Resetting stream {self.stream_id}")
        self.debug()
        self.regs.CTL &= ~2  # clear START
        time.sleep(0.1)
        # set enter reset bit
        self.regs.CTL = 1
        while (self.regs.CTL & 1) == 0:
            pass
        # clear enter reset bit to exit reset
        self.regs.CTL = 0
        while (self.regs.CTL & 1) == 1:
            pass

        log.info(f"Disable SPIB and set position 0 of stream {self.stream_id}")
        self.hda.SPBFCTL = 0
        self.hda.SPIB = 0

        # log.info("Setting dma position buffer and enable it")
        # self.hda.DPUBASE = self.pos_buf_addr >> 32 & 0xffffffff
        # self.hda.DPLBASE = self.pos_buf_addr & 0xfffffff0 | 1

        log.info(f"Enabling dsp capture (PROCEN) of stream {self.stream_id}")
        self.hda.PPCTL |= 1 << self.stream_id

        self.debug()
        log.info(f"Reset stream {self.stream_id}")


class HDAInStream(HDAStream):
    def __init__(self, reg_map, buf_lens, allocator):
        super().__init__(reg_map, buf_lens, allocator)
        self.buf_idx = 0
        self.buf_pos = 0

    def read(self, len):
        """Read len bytes from the sequence of underlying DMA buffers"""

        # TODO implement this rather than reading the buffers directly in tests
        return self.bufs[self.buf_idx]


class HDAOutStream(HDAStream):
    def __init__(self, reg_map, buf_lens, allocator):
        super().__init__(reg_map, buf_lens, allocator)
        self.buf_idx = 0
        self.buf_pos = 0

    def write(self, buf):
        """Write (copy) buf to the sequence of underlying DMA buffers

        Once the write to each buffer is complete the hardware is notified
        """
        # TODO clean this up
        bufl = min(len(data), self.buf_len)
        log.info(
            f"Writing data to stream {self.stream_id}, len {bufl}, SPBFCTL {self.hda.SPBFCTL:x}, SPIB {self.hda.SPIB}"
        )
        self.mem[0:bufl] = data[0:bufl]
        self.mem[bufl : bufl + bufl] = data[0:bufl]
        self.hda.SPBFCTL |= 1 << self.stream_id
        self.hda.SPIB += bufl
        log.info(
            f"Wrote data to stream {self.stream_id}, SPBFCTL {self.hda.SPBFCTL:x}, SPIB {self.hda.SPIB}"
        )


class HWVersion:
    """HWVersion Mapping

    Compare a device identifier against known ids for hardware version
    checks.
    """

    def __init__(self, device_id):
        self.device_id = device_id

    def is_cavs15(self):
        return self.device_id in [0x5A98, 0x1A98, 0x3198]

    def is_cavs18(self):
        return self.device_id in [0x9DC8, 0xA348, 0x02C8, 0x06C8, 0xA3F0]

    def is_cavs25(self):
        self.device_id in [0xA0C8, 0x43C8, 0x4B55, 0x4B58, 0x7AD0, 0x51C8]


class DMABuf:
    """Wraps a physical memory buffer useful for DMA

    Provides the real hardware physical address, a view to the memory array,
    and size in bytes.
    """

    def __init__(self, phys_addr, mem, size):
        self.phys_addr = phys_addr
        self.mem = mem
        self.size = size


class LinuxDMAAlloc:
    """Allocate memory with a physical address

    HDA Streams require physical memory addresses to target/source, this
    provides that from a huge page mapped to memory. Notably no free is
    available as this simplifies avoids needing to do heap like
    management.
    """

    PAGESZ = 4096
    HUGEPAGESZ = 2 * 1024 * 1024
    HUGEPAGE_FILE = "/dev/hugepages/intel-adsp-fw-dma.tmp"

    def __init__(self):
        # Make sure hugetlbfs is mounted (not there on chromeos)
        os.system(
            "mount | grep -q hugetlbfs ||"
            + " (mkdir -p /dev/hugepages; "
            + "  mount -t hugetlbfs hugetlbfs /dev/hugepages)"
        )

        # Ensure the kernel has enough budget for one new page
        free = int(runx("awk '/HugePages_Free/ {print $2}' /proc/meminfo"))
        if free == 0:
            tot = 1 + int(runx("awk '/HugePages_Total/ {print $2}' /proc/meminfo"))
            os.system(f"echo {tot} > /proc/sys/vm/nr_hugepages")

        hugef_name = LinuxDMAAlloc.HUGEPAGE_FILE
        hugef = open(hugef_name, "w+")
        hugef.truncate(LinuxDMAAlloc.HUGEPAGESZ)
        mem = mmap.mmap(hugef.fileno(), LinuxDMAAlloc.HUGEPAGESZ)
        log.info("type of mem is %s", str(type(mem)))
        os.unlink(hugef_name)

        # Find the local process address of the mapping, then use that to extract
        # the physical address from the kernel's pagemap interface.  The physical
        # page frame number occupies the bottom bits of the entry.
        mem[0] = 0  # Fault the page in so it has an address!
        vaddr = ctypes.addressof(ctypes.c_int.from_buffer(mem))
        vpagenum = vaddr >> 12
        pagemap = open("/proc/self/pagemap", "rb")
        pagemap.seek(vpagenum * 8)
        pent = pagemap.read(8)
        paddr = (struct.unpack("Q", pent)[0] & ((1 << 55) - 1)) * LinuxDMAAlloc.PAGESZ
        pagemap.close()

        self.offset = 0
        self.mem = mem
        self.physical_addr = paddr
        self.page_file = hugef

    def alloc(self, size):
        """Allocate a block of memory from the page, aligned to 128 bytes"""
        mem = self.mem[self.offset : size]
        buf = DMABuf(self.physical_addr + self.offset, mem, size)
        self.offset += size
        return buf


class IntelADSP:
    """Intel Audio DSP Mapping

    Map memory to useful registers for an HDA Audio Device and enable
    configuration and IPC, memory window, and HDA stream communcations
    with the Intel ADSP.
    """

    def __init__(self, hwversion, hda_map, dsp_map, alloc):
        self.hwversion = hwversion
        self.hda_map = hda_map
        self.dsp_map = dsp_map
        self.hdamem = hda_map.mem
        self.dspmem = dsp_map.mem
        self.alloc = alloc

        self.hda = Regs(self.hdamem)
        self.hda.GCAP = 0x0000
        self.hda.VMIN = 0x0002
        self.hda.VMAX = 0x0003
        self.hda.GCTL = 0x0008
        self.hda.DPLBASE = 0x0070
        self.hda.DPUBASE = 0x0074
        self.hda.SPBFCH = 0x0700
        self.hda.SPBFCTL = 0x0704
        self.hda.PPCH = 0x0800
        self.hda.PPCTL = 0x0804
        self.hda.PPSTS = 0x0808
        self.hda.freeze()

        # Intel Audio DSP Registers
        self.dsp = Regs(self.dspmem)
        self.dsp.ADSPCS = 0x00004
        self.dsp.HIPCTDR = 0x00040 if hwversion.is_cavs15() else 0x000C0
        self.dsp.HIPCTDA = 0x000C4  # 1.8+ only
        self.dsp.HIPCTDD = 0x00044 if hwversion.is_cavs15() else 0x000C8
        self.dsp.HIPCIDR = 0x00048 if hwversion.is_cavs15() else 0x000D0
        self.dsp.HIPCIDA = 0x000D4  # 1.8+ only
        self.dsp.HIPCIDD = 0x0004C if hwversion.is_cavs15() else 0x000D8
        self.dsp.SRAM_FW_STATUS = 0x80000  # Start of first SRAM window
        self.dsp.freeze()
        self.input_stream_cnt = (self.hda.GCAP >> 8) & 0x0F  # number of input streams
        self.output_stream_cnt = (
            self.hda.GCAP >> 12
        ) & 0x0F  # number of output streams
        log.info(
            f"HDA Streams, Host In: {self.input_stream_cnt}, Host Out: {self.output_stream_cnt}"
        )

        # keep tabs on what streams have been used
        self.input_streams = dict()
        self.output_streams = dict()

        # TODO map ipc registers
        # TODO map memory windows


def linux_audio_dev():
    """Get an instance of AudioDev for Linux"""
    p = runx(f"grep -iEl 'PCI_CLASS=40(10|38)0' /sys/bus/pci/devices/*/uevent")
    pcidir = os.path.dirname(p)

    # Platform/quirk detection.  ID lists cribbed from the SOF kernel driver
    device_id = int(open(f"{pcidir}/device").read().rstrip(), 16)
    hwversion = HWVersion(device_id)

    # Check sysfs for a loaded driver and remove it
    if os.path.exists(f"{pcidir}/driver"):
        mod = os.path.basename(os.readlink(f"{pcidir}/driver/module"))
        found_msg = f'Existing driver "{mod}" found'
        if args.log_only:
            log.info(found_msg)
        else:
            log.warning(found_msg + ", unloading module")
            runx(f"rmmod -f {mod}")
            # Disengage runtime power management so the kernel doesn't put it to sleep
            log.info(f"Forcing {pcidir}/power/control to always 'on'")
            with open(f"{pcidir}/power/control", "w") as ctrl:
                ctrl.write("on")

    # Make sure PCI memory space access and busmastering are enabled.
    # Also disable interrupts so as not to confuse the kernel.
    with open(f"{pcidir}/config", "wb+") as cfg:
        cfg.seek(4)
        cfg.write(b"\x06\x04")

    # Map physical PCI Bars to a memory mapped file
    # Along with a page (huge page) for subsequent physical
    # memory allocation
    hda_map = LinuxBarMap(pcidir, 0)
    dsp_map = LinuxBarMap(pcidir, 4)

    log.info("setting up dma allocator")
    alloc = LinuxDMAAlloc()

    log.info("returning audio device")
    # DSP is really a set of PCI bar register (memory) maps and a dma allocator
    return IntelADSP(hwversion, hda_map, dsp_map, alloc)


# Maps a PCI BAR and returns the in-process address
class LinuxBarMap:
    def __init__(self, pcidir, barnum):
        f = open(pcidir + "/resource" + str(barnum), "r+")
        self.mm = mmap.mmap(f.fileno(), os.fstat(f.fileno()).st_size)
        log.info(
            "Mapped PCI bar %d of length %d bytes."
            % (barnum, os.fstat(f.fileno()).st_size)
        )
        self.mem = ctypes.addressof(ctypes.c_int.from_buffer(self.mm))


def runx(cmd):
    return subprocess.check_output(cmd, shell=True).decode().rstrip()


def mask(bit):
    if cavs25:
        return 0b1 << bit
    if cavs18:
        return 0b1111 << bit
    if cavs15:
        return 0b11 << bit


def load_firmware(adsp, fw_file):
    try:
        fw_bytes = open(fw_file, "rb").read()
    except Exception as e:
        log.error(f"Could not read firmware file: `{fw_file}'")
        log.error(e)
        sys.exit(1)

    (magic, sz) = struct.unpack("4sI", fw_bytes[0:8])
    if magic == b"XMan":
        log.info(f"Trimming {sz} bytes of extended manifest")
        fw_bytes = fw_bytes[sz : len(fw_bytes)]

    # This actually means "enable access to BAR4 registers"!
    adsp.hda.PPCTL |= 1 << 30  # GPROCEN, "global processing enable"

    log.info("Resetting HDA device")
    adsp.hda.GCTL = 0
    while adsp.hda.GCTL & 1:
        pass
    adsp.hda.GCTL = 1
    while not adsp.hda.GCTL & 1:
        pass

    log.info(f"Stalling and Resetting DSP cores, ADSPCS = 0x{adsp.dsp.ADSPCS:x}")
    adsp.dsp.ADSPCS |= mask(CSTALL)
    adsp.dsp.ADSPCS |= mask(CRST)
    while (adsp.dsp.ADSPCS & mask(CRST)) == 0:
        pass

    log.info(f"Powering down DSP cores, ADSPCS = 0x{dsp.ADSPCS:x}")
    adsp.dsp.ADSPCS &= ~mask(SPA)
    while adsp.dsp.ADSPCS & mask(CPA):
        pass

    ostream_id = 0
    log.info(f"Configuring HDA stream {ostream_id} to transfer firmware image")
    ostream = adsp.hda_out_stream(ostream_id, [len(fw_bytes)])
    # copy the firmware into the DMA buffer
    ostream.bufs[0][0 : len(fw_bytes)] = fw_bytes
    ostream.start()

    # Start DSP.  Host needs to provide power to all cores on 1.5
    # (which also starts them) and 1.8 (merely gates power, DSP also
    # has to set PWRCTL). On 2.5 where the DSP has full control,
    # and only core 0 is set.
    log.info(f"Starting DSP, ADSPCS = 0x{dsp.ADSPCS:x}")
    dsp.ADSPCS = mask(SPA)
    while (dsp.ADSPCS & mask(CPA)) == 0:
        pass

    log.info(f"Unresetting DSP cores, ADSPCS = 0x{dsp.ADSPCS:x}")
    dsp.ADSPCS &= ~mask(CRST)
    while (dsp.ADSPCS & 1) != 0:
        pass

    log.info(f"Running DSP cores, ADSPCS = 0x{dsp.ADSPCS:x}")
    dsp.ADSPCS &= ~mask(CSTALL)

    # Wait for the ROM to boot and signal it's ready.  This not so short
    # sleep seems to be needed; if we're banging on the memory window
    # during initial boot (before/while the window control registers
    # are configured?) the DSP hardware will hang fairly reliably.
    log.info(f"Wait for ROM startup, ADSPCS = 0x{dsp.ADSPCS:x}")
    time.sleep(1)
    while (dsp.SRAM_FW_STATUS >> 24) != 5:
        pass

    # Send the DSP an IPC message to tell the device how to boot.
    # Note: with cAVS 1.8+ the ROM receives the stream argument as an
    # index within the array of output streams (and we always use the
    # first one by construction).  But with 1.5 it's the HDA index,
    # and depends on the number of input streams on the device.

    stream_idx = ostream_id if cavs15 else 0
    ipcval = (
        (1 << 31)  # BUSY bit
        | (0x01 << 24)  # type = PURGE_FW
        | (1 << 14)  # purge_fw = 1
        | (stream_idx << 9)
    )  # dma_id
    log.info(f"Sending IPC command, HIPIDR = 0x{ipcval:x}")
    dsp.HIPCIDR = ipcval

    log.info(f"Starting DMA, FW_STATUS = 0x{dsp.SRAM_FW_STATUS:x}")
    sd.CTL |= 2  # START flag

    wait_fw_entered()

    ostream.stop()

    log.info(f"cAVS firmware load complete")


def fw_is_alive():
    return dsp.SRAM_FW_STATUS & ((1 << 28) - 1) == 5  # "FW_ENTERED"


def wait_fw_entered(timeout_s=2):
    log.info(
        "Waiting %s for firmware handoff, FW_STATUS = 0x%x",
        "forever" if timeout_s is None else f"{timeout_s} seconds",
        dsp.SRAM_FW_STATUS,
    )
    hertz = 100
    attempts = None if timeout_s is None else timeout_s * hertz
    while True:
        alive = fw_is_alive()
        if alive:
            break
        if attempts is not None:
            attempts -= 1
            if attempts < 0:
                break
        time.sleep(1 / hertz)

    if not alive:
        log.warning("Load failed?  FW_STATUS = 0x%x", dsp.SRAM_FW_STATUS)
    else:
        log.info("FW alive, FW_STATUS = 0x%x", dsp.SRAM_FW_STATUS)


class Winstream:
    """Takes a memory window and treats it as a winstream text output

    Acts as an iterator over the winstream producing lists of utf-8 characters
    from the ascii stream.
    """

    def __init__(self):
        self.last_seq = 0

    def __iter__(self):
        return self

    def _hdr(self):
        return struct.unpack("<IIII", win_read(self, 0, 16))

    def _read(self, start, length):
        try:
            # TODO do we really need to remap the bar? probably not! lets map once and pass in
            return b"".join(
                bar4_mmap[WINSTREAM_OFFSET + x].to_bytes(1, "little")
                for x in range(start, start + length)
            )
        except IndexError as ie:
            # A FW in a bad state may cause winstream garbage
            log.error("IndexError in bar4_mmap[%d + %d]", WINSTREAM_OFFSET, start)
            log.error("bar4_mmap.size()=%d", bar4_mmap.size())
            raise ie

    def __next__(self):
        while True:
            (wlen, start, end, seq) = self._hdr()
            if self.last_seq == 0:
                self.last_seq = (
                    seq if args.no_history else (seq - ((end - start) % wlen))
                )
            if seq == self.last_seq or start == end:
                return (seq, "")
            behind = seq - self.last_seq
            if behind > ((end - start) % wlen):
                return (seq, "")
            copy = (end - behind) % wlen
            suffix = min(behind, wlen - copy)
            result = self._read(16 + copy, suffix)
            if suffix < behind:
                result += win_read(16, behind - suffix)
            (wlen, start1, end, seq1) = self._hdr()
            if start1 == start and seq1 == seq:
                # Best effort attempt at decoding, replacing unusable characters
                # Found to be useful when it really goes wrong
                return (seq, result.decode("utf-8", "replace"))


async def ipc_delay_done():
    await asyncio.sleep(0.1)
    dsp.HIPCTDA = 1 << 31


ipc_timestamp = 0

# Super-simple command language, driven by the test code on the DSP
def ipc_command(data, ext_data):
    send_msg = False
    done = True
    log.debug("ipc data %d, ext_data %x", data, ext_data)
    if data == 0:  # noop, with synchronous DONE
        pass
    elif data == 1:  # async command: signal DONE after a delay (on 1.8+)
        if not cavs15:
            done = False
            asyncio.ensure_future(ipc_delay_done())
    elif data == 2:  # echo back ext_data as a message command
        send_msg = True
    elif data == 3:  # set ADSPCS
        dsp.ADSPCS = ext_data
    elif data == 4:  # echo back microseconds since last timestamp command
        global ipc_timestamp
        t = round(time.time() * 1e6)
        ext_data = t - ipc_timestamp
        ipc_timestamp = t
        send_msg = True
    elif data == 5:  # copy word at outbox[ext_data >> 16] to inbox[ext_data & 0xffff]
        src = OUTBOX_OFFSET + 4 * (ext_data >> 16)
        dst = INBOX_OFFSET + 4 * (ext_data & 0xFFFF)
        for i in range(4):
            bar4_mmap[dst + i] = bar4_mmap[src + i]
    elif data == 6:  # HDA RESET (init if not exists)
        stream_id = ext_data & 0xFF
        if stream_id in hda_streams:
            hda_streams[stream_id].reset()
        else:
            hda_str = HDAStream(stream_id)
            hda_streams[stream_id] = hda_str
    elif data == 7:  # HDA CONFIG
        stream_id = ext_data & 0xFF
        buf_len = ext_data >> 8 & 0xFFFF
        hda_str = hda_streams[stream_id]
        hda_str.config(buf_len)
    elif data == 8:  # HDA START
        stream_id = ext_data & 0xFF
        hda_streams[stream_id].start()
        hda_streams[stream_id].mem.seek(0)

    elif data == 9:  # HDA STOP
        stream_id = ext_data & 0xFF
        hda_streams[stream_id].stop()
    elif data == 10:  # HDA VALIDATE
        stream_id = ext_data & 0xFF
        hda_str = hda_streams[stream_id]
        hda_str.debug()
        is_ramp_data = True
        hda_str.mem.seek(0)
        for (i, val) in enumerate(hda_str.mem.read(256)):
            if i != val:
                is_ramp_data = False
            # log.info("stream[%d][%d]: %d", stream_id, i, val) # debug helper
        log.info("Is ramp data? " + str(is_ramp_data))
        ext_data = int(is_ramp_data)
        log.info(f"Ext data to send back on ramp status {ext_data}")
        send_msg = True
    elif data == 11:  # HDA HOST OUT SEND
        stream_id = ext_data & 0xFF
        buf = bytearray(256)
        for i in range(0, 256):
            buf[i] = i
        hda_streams[stream_id].write(buf)
    elif data == 12:  # HDA PRINT
        stream_id = ext_data & 0xFF
        buf_len = ext_data >> 8 & 0xFFFF
        hda_str = hda_streams[stream_id]
        # check for wrap here
        pos = hda_str.mem.tell()
        read_lens = [buf_len, 0]
        if pos + buf_len >= hda_str.buf_len * 2:
            read_lens[0] = hda_str.buf_len * 2 - pos
            read_lens[1] = buf_len - read_lens[0]
        # validate the read lens
        assert (read_lens[0] + pos) <= (hda_str.buf_len * 2)
        assert read_lens[0] % 128 == 0
        assert read_lens[1] % 128 == 0
        buf_data0 = hda_str.mem.read(read_lens[0])
        hda_msg0 = buf_data0.decode("utf-8", "replace")
        sys.stdout.write(hda_msg0)
        if read_lens[1] != 0:
            hda_str.mem.seek(0)
            buf_data1 = hda_str.mem.read(read_lens[1])
            hda_msg1 = buf_data1.decode("utf-8", "replace")
            sys.stdout.write(hda_msg1)
        pos = hda_str.mem.tell()
        sys.stdout.flush()
    else:
        log.warning(f"cavstool: Unrecognized IPC command 0x{data:x} ext 0x{ext_data:x}")
        if not fw_is_alive():
            if args.log_only:
                log.info("DSP power seems off")
                wait_fw_entered(timeout_s=None)
            else:
                log.warning("DSP power seems off?!")
                time.sleep(2)  # potential spam reduction

            return

    dsp.HIPCTDR = 1 << 31  # Ack local interrupt, also signals DONE on v1.5
    if cavs18:
        time.sleep(0.01)  # Needed on 1.8, or the command below won't send!

    if done and not cavs15:
        dsp.HIPCTDA = 1 << 31  # Signal done
    if send_msg:
        dsp.HIPCIDD = ext_data
        dsp.HIPCIDR = (1 << 31) | ext_data


async def _main(server, args):
    global START_OUTPUT

    log.info("getting audio device")
    audio_dev = None
    try:
        audio_dev = linux_audio_dev()
    except Exception as e:
        log.error("Could not map device in sysfs; run as root?")
        log.error(e)
        sys.exit(1)

    log.info(f"Audio Device? {audio_dev}")

    log.info(f"Detected cavs 1.5? {audio_dev.hwversion.is_cavs15()}")

    log.info(f"Log only? {args.log_only}")
    if args.log_only:
        log.info("Waiting on firmware")

        wait_fw_entered(timeout_s=None)
    else:

        if not args.fw_file:
            log.error("Firmware file argument missing")
            sys.exit(1)

        log.info("loading firmware")
        load_firmware(audio_dev, args.fw_file)
        log.info("loaded firmware")
        time.sleep(0.1)
        if not args.quiet:
            adsp_log("--\n", server)

    log.info(f"winstream iteration... START_OUTPUT {START_OUTPUT}")
    win_str = Winstream()
    while START_OUTPUT is True:
        await asyncio.sleep(0.03)
        output = next(win_str)
        if output:
            adsp_log(output, server)
        if audio_dev.dsp.HIPCIDA & 0x80000000:
            # must ACK any DONE interrupts that arrive!
            audio_dev.dsp.HIPCIDA = 1 << 31
        if audio_dev.dsp.HIPCTDR & 0x80000000:
            ipc_command(audio_dev.dsp.HIPCTDR & ~0x80000000, audio_dev.dsp.HIPCTDD)

        if server:
            # Check if the client connection is alive.
            if not is_connection_alive(server):
                lock.acquire()
                START_OUTPUT = False
                lock.release()


class ADSPRequestHandler(socketserver.BaseRequestHandler):
    """Request handler class for control the actions of server."""

    def receive_fw(self):
        log.info("Receiving...")
        # Receive the header first
        d = self.request.recv(HEADER_SZ)

        # Unpacked the header data
        # Include size(4), filename(42) and MD5(32)
        header = d[:HEADER_SZ]
        total = d[HEADER_SZ:]
        s = struct.Struct(PACKET_HEADER_FORMAT_FW)
        fsize, fname, md5_tx_b = s.unpack(header)
        log.info(f"size:{fsize}, filename:{fname}, MD5:{md5_tx_b}")

        # Receive the firmware. We only receive the specified amount of bytes.
        while len(total) < fsize:
            data = self.request.recv(min(BUF_SIZE, fsize - len(total)))
            if not data:
                raise EOFError("truncated firmware file")
            total += data

        log.info(f"Done Receiving {len(total)}.")

        try:
            with open(fname, "wb") as f:
                f.write(total)
        except Exception as e:
            log.error(f"Get exception {e} during FW transfer.")
            return None

        # Check the MD5 of the firmware
        md5_rx = hashlib.md5(total).hexdigest()
        md5_tx = md5_tx_b.decode("utf-8")

        if md5_tx != md5_rx:
            log.error(f"MD5 mismatch: {md5_tx} vs. {md5_rx}")
            return None

        return fname

    def handle(self):
        global START_OUTPUT, fw_file

        cmd = self.request.recv(MAX_CMD_SZ)
        log.info(f"{self.client_address[0]} wrote: {cmd}")
        action = cmd.decode("utf-8")
        log.debug(f"load {action}")

        if action == CMD_DOWNLOAD:
            self.request.sendall(cmd)
            recv_file = self.receive_fw()

            if recv_file:
                self.request.sendall("success".encode("utf-8"))
                log.info("Firmware well received. Ready to download.")
            else:
                self.request.sendall("failed".encode("utf-8"))
                log.error("Receive firmware failed.")

            lock.acquire()
            fw_file = recv_file
            START_OUTPUT = True
            lock.release()

        else:
            log.error("incorrect load communitcation!")


class ADSPLogHandler(socketserver.BaseRequestHandler):
    """Log handler class for grabbing output messages of server."""

    def run_adsp(self):
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(_main(self))

    def handle(self):
        cmd = self.request.recv(MAX_CMD_SZ)
        log.info(f"{self.client_address[0]} wrote: {cmd}")
        action = cmd.decode("utf-8")
        log.debug(f"monitor {action}")

        if action == CMD_LOG_START:
            global START_OUTPUT, fw_file

            self.request.sendall(cmd)

            log.info(f"Waiting for instruction...")
            while START_OUTPUT is False:
                time.sleep(1)
                if not is_connection_alive(self):
                    break

            if fw_file:
                log.info(f"Loaded FW {fw_file} and running...")
                if os.path.exists(fw_file):
                    self.run_adsp()
                    log.info("service complete.")
                else:
                    log.error("Cannot find the FW file.")

            lock.acquire()
            START_OUTPUT = False
            if fw_file:
                os.remove(fw_file)
            fw_file = None
            lock.release()

        else:
            log.error("incorrect monitor communitcation!")

        log.info("Wait for next service...")


def is_connection_alive(server):
    try:
        server.request.sendall(b" ")
    except (BrokenPipeError, ConnectionResetError):
        log.info("Client is disconnect.")
        return False

    return True


def adsp_log(output, server):
    if server:
        server.request.sendall(output.encode("utf-8"))
    else:
        sys.stdout.write(output)
        sys.stdout.flush()


def get_host_ip():
    """Helper to detect host's serving ip address."""
    interfaces = netifaces.interfaces()

    for i in interfaces:
        if i != "lo":
            try:
                netifaces.ifaddresses(i)
                ip = netifaces.ifaddresses(i)[netifaces.AF_INET][0]["addr"]
                log.info(f"Use interface {i}, IP address: {ip}")
            except Exception:
                log.info(f"Ignore the interface {i} which is not activated.")
    return ip


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="DSP loader/logger tool")
    ap.add_argument(
        "-q", "--quiet", action="store_true", help="No loader output, just DSP logging"
    )
    ap.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="More loader output, DEBUG logging level",
    )
    ap.add_argument(
        "-l",
        "--log-only",
        action="store_true",
        help="Don't load firmware, just show log output",
    )
    ap.add_argument(
        "-n",
        "--no-history",
        action="store_true",
        help="No current log buffer at start, just new output",
    )
    ap.add_argument(
        "-s", "--server-addr", help="Specify the IP address that the server to active"
    )
    ap.add_argument("fw_file", nargs="?", help="Firmware file")

    args = ap.parse_args()

    if args.quiet:
        log.setLevel(logging.WARN)
    elif args.verbose:
        log.setLevel(logging.DEBUG)

    if args.fw_file:
        fw_file = args.fw_file
    else:
        fw_file = None
    if args.server_addr:
        HOST = args.server_addr
    else:
        HOST = get_host_ip()

    # When fw_file is assigned or in log_only mode, it will
    # not serve as a daemon. That mean it just run load
    # firmware or read the log directly.
    if args.fw_file or args.log_only:
        START_OUTPUT = True
        try:
            asyncio.run(_main(None, args))
        except KeyboardInterrupt:
            START_OUTPUT = False
        except Exception as e:
            log.error(e)
        finally:
            sys.exit(0)

    # Launch the command request service
    socketserver.TCPServer.allow_reuse_address = True
    req_server = socketserver.TCPServer((HOST, PORT_REQ), ADSPRequestHandler)
    req_t = threading.Thread(target=req_server.serve_forever, daemon=True)

    # Activate the log service which output adsp execution
    with socketserver.TCPServer((HOST, PORT_LOG), ADSPLogHandler) as log_server:
        try:
            log.info("Req server start...")
            req_t.start()
            log.info("Log server start...")
            log_server.serve_forever()
        except KeyboardInterrupt:
            lock.acquire()
            START_OUTPUT = False
            lock.release()
            log_server.shutdown()
            req_server.shutdown()
